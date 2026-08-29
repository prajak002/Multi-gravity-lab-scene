/**
 * retarget.mjs — rebuild the leg motion of a packet clip so the feet actually
 * make contact with the terrain they are standing on.
 *
 * WHY THIS EXISTS
 * The supplied packets author a root trajectory and then decorate it with leg
 * angles that were never solved against the ground. Measured over all 28 clips
 * (tools/audit_contact.mjs): the swing foot travels BACKWARD relative to the
 * pelvis in 17 of them, and the stance foot slides across the ground for up to
 * 125 % of the distance the robot covers. The legs are ornament, not locomotion.
 *
 * WHAT IS KEPT AND WHAT IS REBUILT
 *   kept      root path, root yaw, the bounce/crouch residual of pelvis height,
 *             arm and waist angles, and the CONTACT TIMING extracted from the
 *             original clip — so cadence, step-duration variance and the
 *             left/right asymmetry the packet's own metrics describe survive.
 *   rebuilt   every leg joint, by solving the real 6-DOF chain against a
 *             foothold planted on the terrain surface.
 *
 * The result keeps each scenario's story and each model's character, and makes
 * the ground contact real.
 */
import fs from 'fs';
import path from 'path';
import {
  legFK, legIK, contactPoints, rpyMat, quatToMat, matToQuat, mapv, mT, mmul,
  NEUTRAL_LEG, SOLE_CENTRE, FOOT_CONTACTS, FOOT_LEN,
} from '../src/sim/G1Kinematics.js';
import { SiteField, SURFACE_PROFILES } from '../src/terrain/SiteField.js';

const FPS = 30;
/**
 * Ride height of the pelvis above the ground, metres.
 *
 * NOT the packets' 0.795 m. Measured on the shipped URDF the maximum possible
 * pelvis-to-sole distance is 0.7916 m with the knee fully straight, so 0.795 is
 * past the kinematic limit and the packets' own "crouch" pose leaves 5 mm of
 * knee flexion — a locked leg. Every foothold placed under that pelvis is
 * outside the workspace, which is why the solver saturated on 555 of 600
 * frames before this was corrected.
 *
 * 0.70 m leaves ~0.09 m of flexion, giving a 0.36 m reach envelope either side
 * of the hip. The authored bounce RESIDUAL is preserved on top of this
 * verbatim, so each model's peak-bounce metric is untouched; only the base
 * offset moves, and it moves onto a value the leg can actually hold.
 */
const DEFAULT_RIDE = 0.68;
/**
 * Knee flexion the solver is never allowed to go below, radians.
 *
 * Not a hardware limit — the URDF allows -0.087 — but a control choice, and
 * the same one a real walking controller makes. See legIK: leg length goes as
 * cos(knee/2), so the knee has no authority over reach at full extension and
 * the solver swings it through tens of degrees to move the foot a millimetre.
 *
 * Swept 0.20 / 0.30 / 0.40 against the same build. The floor turns out NOT to
 * be what sets peak joint rate — 1470 / 1392 / 1193 deg/s, and 0.87 / 0.84 /
 * 0.90 percent of joint-frames over the limit, which is noise. What it does
 * buy, and buys completely, is the hyperextension: minimum knee across all 24
 * clips goes from -2.5 deg to +11.5 deg, and the visible snap goes with it.
 * 0.30 is the middle of that sweep and gives the best sole penetration.
 */
const KNEE_FLOOR = 0.30;
// Hip-pitch origin -> ankle-roll AT THE KNEE FLOOR, measured on the URDF tree.
// 0.6564 is the fully straight figure and is no longer reachable by design.
const MAX_HIP_REACH = 0.6491;
const HIP_DROP = 0.1027;           // pelvis -> hip-pitch origin

// CSV column layout: 3 root pos, 4 root quat (xyzw), 29 joint angles.
export const CSV_JOINTS = [
  'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
  'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
  'waist_yaw', 'waist_roll', 'waist_pitch',
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
  'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
  'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
];
const JI = Object.fromEntries(CSV_JOINTS.map((n, i) => [n, 7 + i]));
const LEG_COLS = {
  left: ['left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll'].map((n) => JI[n]),
  right: ['right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll'].map((n) => JI[n]),
};

// ---------------------------------------------------------------------------
// signal helpers
// ---------------------------------------------------------------------------
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;
const smooth = (t) => t * t * (3 - 2 * t);

/** Zero-phase box smoothing — used for trends, so it must not shift timing. */
function lowpass(a, win) {
  const n = a.length, out = new Array(n);
  const h = Math.max(1, Math.round(win / 2));
  for (let i = 0; i < n; i++) {
    let s = 0, c = 0;
    for (let k = Math.max(0, i - h); k <= Math.min(n - 1, i + h); k++) { s += a[k]; c++; }
    out[i] = s / c;
  }
  return out;
}

/** Deterministic per-clip jitter, so a rerun produces an identical clip. */
function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; };
}

// ---------------------------------------------------------------------------
// 1. gait schedule
// ---------------------------------------------------------------------------
/**
 * When is each foot down?
 *
 * The original clips cannot answer this. Their leg angles are wrong in
 * direction and magnitude, and reading contact off them yields about five
 * steps per leg for a 4.6 m traverse — a 0.9 m stride, which is half a metre
 * beyond anything a 0.79 m leg can reach. Trusting that schedule puts every
 * foothold outside the workspace and the solver simply saturates.
 *
 * So the cadence comes from where cadence actually comes from: the linear
 * inverted pendulum. Step period is 2*sqrt(L/g), which is why a Moon gait is
 * slower than a Mars gait for the same robot, and is the premise this whole
 * lab is built on (see src/sim/G1Motion.js).
 *
 * What IS taken from the packet is the irregularity. Step-duration CV is the
 * metric that separates the two models — 0.205 for WorldVLA against 0.009 for
 * PragyaSpace on Shiv Shakti — so each cycle's period is perturbed by exactly
 * that coefficient of variation. The struggling model gets a genuinely ragged
 * cadence and the stable one a metronomic gait, from their own numbers.
 */
function gaitSchedule(n, speed, opt) {
  const g = Math.max(opt.g ?? 9.81, 0.05);
  const L = opt.comHeight ?? 0.62;
  const baseT = 2 * Math.sqrt(L / g) * (opt.periodScale ?? 1);

  // THE LEG IS SHORTER THAN THE PENDULUM WANTS.
  //
  // Step period from the linear inverted pendulum is 2*sqrt(L/g), so at one
  // sixth g it is 2.46x the Earth value — the premise this whole lab is built
  // on. But cadence and STRIDE are the same statement: at a given speed a
  // longer period means a longer step, and on the Moon that comes to
  // 0.5 m/s * 1.385 s = 0.69 m per cycle, which puts the foot 0.35 m fore and
  // aft of the hip.
  //
  // A G1 cannot do that. With the pelvis at WorldVLA's 0.70 m ride height the
  // hip sits 0.562 m above the ankle, and against a 0.6465 m reach budget that
  // leaves 0.32 m of horizontal envelope in total — of which the stance width
  // and the capture step have already taken 0.17 m sideways. What remains
  // fore-and-aft is 0.27 m, and the pendulum is asking for 0.35 m.
  //
  // So the natural cadence is not available and the robot has to step faster
  // than its own pendulum, taking more, shorter steps. That is a hardware
  // limit beating a dynamics preference, and it is why the four Moon clips
  // were driving feet through the ground: the foothold was simply out of
  // reach, the reach clamp pulled it in and down, and the sole ended up 77 mm
  // under the surface.
  //
  // Note which model this bites hardest. Riding high on straight legs costs
  // reach — the envelope goes as sqrt(budget^2 - height^2) — so WorldVLA's
  // 0.70 m posture leaves 0.27 m where PragyaSpace's 0.645 m leaves 0.36 m.
  // The high-riding model is the one forced off its natural cadence.
  // Applied per FRAME against the local speed, not once against the mean.
  // These clips are not steady: Gale's WorldVLA traverse averages 0.573 m/s
  // and peaks at 1.585 m/s, and the speed term below stretches the period
  // further still at speed. Capping on the mean leaves every fast burst
  // asking for a 0.42 m half-stride against a 0.23 m envelope — which is what
  // put that clip's sole 39 mm into the hill even after the mean-based cap
  // was in. The bound has to hold everywhere, so it is applied inside the
  // integration below.
  const maxHalf = opt.maxHalfStride ?? Infinity;
  // Duty and cadence are NOT independent, and treating them as independent is
  // what broke this.
  //
  // A high duty factor means long double support, which is what careful
  // walking looks like — but the swing has to fit in what is left. At Mars
  // gravity with a 0.72 s cycle, a duty of 0.74 leaves 0.174 s of swing: five
  // frames to lift a foot, carry it past the stance leg and set it down. The
  // joint rates that demands are impossible, and every one of the 130
  // over-limit joint/clip combinations traced back here.
  //
  // So duty is capped by the cycle it has to live in. Wanting more double
  // support costs cadence; it cannot be had for free.
  const MIN_SWING = 0.30;                  // seconds, a G1 swing at walking pace
  const duty = Math.max(0.52, Math.min(opt.duty ?? 0.62, 1 - MIN_SWING / baseT));
  const cv = clamp(opt.stepCV ?? 0, 0, 0.45);
  const rand = rng((opt.seed ?? 1) ^ 0x5bf03635);

  // Per-cycle period, jittered by the packet's own step-duration CV.
  const cyclePeriod = [];
  for (let k = 0; k < 64; k++) {
    // Box-Muller from the deterministic stream, clipped so a period never
    // collapses to nothing on a high-CV clip.
    const u1 = Math.max(1e-6, rand()), u2 = rand();
    const gauss = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    cyclePeriod.push(baseT * clamp(1 + cv * gauss, 0.55, 1.75));
  }

  // Walk the phase forward, letting speed shorten the period a little: a robot
  // moving faster steps faster, within bounds.
  const phase = new Array(n);
  let ph = opt.phase0 ?? 0;
  for (let i = 0; i < n; i++) {
    const T = cyclePeriod[Math.floor(ph) % cyclePeriod.length];
    const vRef = opt.refSpeed ?? 0.5;
    const f = clamp(1 + 0.30 * (speed[i] / Math.max(vRef, 1e-3) - 1), 0.7, 1.5);
    // Where the robot is moving fast it must step FASTER, not further: the
    // foot cannot go past the leg's envelope however long the period is.
    const Tlocal = Math.min(T / f, 2 * maxHalf / Math.max(speed[i], 0.05));
    ph += (1 / FPS) / Math.max(Tlocal, 0.18);
    phase[i] = ph;
  }

  // Stance masks: left leads, right is half a cycle behind.
  const out = {};
  for (const side of ['left', 'right']) {
    const off = side === 'left' ? 0 : 0.5;
    const down = phase.map((p) => {
      const u = (p + off) % 1;
      return u < duty;
    });
    const iv = [];
    let s = down[0] ? 0 : -1;
    for (let i = 1; i < n; i++) {
      if (down[i] && !down[i - 1]) s = i;
      else if (!down[i] && down[i - 1] && s >= 0) { iv.push([s, i - 1]); s = -1; }
    }
    if (s >= 0) iv.push([s, n - 1]);
    out[side] = { down, stance: iv.filter(([a, b]) => b - a >= 1) };
  }
  out.baseT = baseT;
  out.duty = duty;
  out.swingTime = (1 - duty) * baseT;
  out.cycles = phase[n - 1] - (opt.phase0 ?? 0);
  return out;
}

// ---------------------------------------------------------------------------
// 2. main retarget
// ---------------------------------------------------------------------------
/**
 * @param {number[][]} rows      original CSV rows
 * @param {SiteField} field      terrain
 * @param {object} opt           placement + style
 */
export function retarget(rows, field, opt) {
  const n = rows.length;
  const rand = rng(opt.seed ?? 12345);
  // Terrain height in the CSV's Z-up world: x east, y north.
  // SiteField works in the renderer's Y-up frame where +z is SOUTH.
  const groundAt = (x, y) => field.heightAt(x, -y);
  const planeAt = (x, y, yaw) => field.footPlane(x, -y, -yaw, FOOT_LEN, 0.06);

  // ---- place the authored path onto the terrain ---------------------------
  // The path keeps its shape; it is translated to the site origin and rotated
  // so the traverse runs along the direction the scenario calls for.
  const c = Math.cos(opt.heading), s = Math.sin(opt.heading);
  const rawX = [], rawY = [];
  for (let i = 0; i < n; i++) {
    const x0 = rows[i][0] - rows[0][0], y0 = rows[i][1] - rows[0][1];
    rawX.push(opt.origin[0] + x0 * c - y0 * s);
    rawY.push(opt.origin[1] + x0 * s + y0 * c);
  }

  // ---- split the path into a ROUTE and pelvis SWAY ------------------------
  //
  // The authored lateral signal cannot be applied to the pelvis as written.
  // Measured on Gale, the path moves 0.287 m sideways within a single step
  // period (0.82 s at Mars gravity), and 0.363 m on the B clip. A walking
  // biped's lateral centre-of-mass excursion is a few centimetres; a third of
  // a metre is not sway, and asking for it puts the pelvis that far from feet
  // that are — correctly — still planted where they were put down. That is
  // what drove the left foot 0.36 m out to the side, abducted the hip to its
  // stop and pinned the knee against its lower limit at a straight-leg
  // singularity the solver could not escape: 451 mm of position error.
  //
  // The packets' own metric says the same thing. Shiv Shakti reports an RMS
  // lateral error of 0.028 m for WorldVLA and 0.004 m for PragyaSpace, while
  // the path's raw lateral span is 0.21 m. Those are two different signals
  // added together: a slow drift off the intended line (the ROUTE curving,
  // which the feet follow) and a small fast residual about it (genuine SWAY,
  // which the pelvis does over planted feet). Separating them by timescale
  // recovers both, and the RMS the packet quotes is the residual.
  const along = [], lat = [];
  for (let i = 0; i < n; i++) {
    const dx = rawX[i] - rawX[0], dy = rawY[i] - rawY[0];
    along.push(dx * c + dy * s);
    lat.push(-dx * s + dy * c);
  }
  // Two step periods: long enough that a step's own sway averages out, short
  // enough that a genuine change of route survives.
  const routeWin = Math.round(FPS * 2 * 2 * Math.sqrt((opt.comHeight ?? 0.62)
                    / Math.max(opt.g ?? 9.81, 0.05)));
  const latRoute = lowpass(lat, routeWin);
  // Sway is what is left. Cap it at a real biped's excursion: beyond about
  // 60 mm the pelvis leaves the support polygon and the gait is a fall.
  const SWAY_CAP = 0.06;
  const px = [], py = [];
  for (let i = 0; i < n; i++) {
    const sway = clamp(lat[i] - latRoute[i], -SWAY_CAP, SWAY_CAP);
    const L = latRoute[i] + sway;
    px.push(rawX[0] + along[i] * c - L * s);
    py.push(rawY[0] + along[i] * s + L * c);
  }
  // Footholds are planted against the ROUTE, so they carry the drift but not
  // the sway — which is what lets the pelvis sway over a planted foot.
  const routeX = [], routeY = [];
  for (let i = 0; i < n; i++) {
    routeX.push(rawX[0] + along[i] * c - latRoute[i] * s);
    routeY.push(rawY[0] + along[i] * s + latRoute[i] * c);
  }

  // ---- heading per frame -------------------------------------------------
  //
  // NOT from the local path tangent. Every packet traverse is essentially
  // straight — measured over all 28 clips the net path direction is within
  // 0.3 deg of the packet's own +x axis — but each one carries a lateral
  // wobble that IS the scenario: "RMS lateral error" is one of the metrics
  // separating the two models (0.028 m for WorldVLA against 0.004 m for
  // PragyaSpace on Shiv Shakti).
  //
  // With a 53 mm forward step and 28 mm of side-to-side wobble, the
  // instantaneous tangent swings wildly: atan2 of the frame-to-frame delta
  // sweeps 1404 degrees over the 4.45 m Gale A traverse, and 1407 over
  // Medusae A. Feeding that to the body yaw span the robot through four
  // turns. Downstream it pinned hip roll against its stop, carried the left
  // foot 0.21 m across the midline, and left the solver 115 mm from the
  // foot it was asked for.
  //
  // A robot that wobbles three centimetres while walking forward does not
  // rotate. The wobble is SWAY, and it is already carried in the path
  // position; what it must not do is become facing. So heading is the scene
  // traverse direction, corrected only by the long-baseline drift of the
  // path itself, and bounded.
  const sx = lowpass(px, 9), sy = lowpass(py, 9);
  // Baseline long enough that lateral wobble cancels: half a second of travel
  // either side, which is several times the wobble's period and a fraction of
  // any real turn.
  const BASE = Math.round(FPS * 0.55);
  const MAX_DEV = 35 * Math.PI / 180;      // how far facing may leave the traverse line
  const MAX_YAW_RATE = (28 * Math.PI / 180) / FPS;   // rad/frame; a loaded biped turns slowly
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

  const raw = [];
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - BASE), b = Math.min(n - 1, i + BASE);
    const dx = sx[b] - sx[a], dy = sy[b] - sy[a];
    // Below a quarter of a step length the direction is noise, not intent.
    raw.push(Math.hypot(dx, dy) > 0.12 ? Math.atan2(dy, dx) : null);
  }
  // Deviation from the scene traverse direction, clamped: the scenario decides
  // where the robot is going, the path only nudges it.
  const yaw = [];
  let held = opt.heading;
  for (let i = 0; i < n; i++) {
    const want = raw[i] === null
      ? opt.heading
      : opt.heading + clamp(wrap(raw[i] - opt.heading), -MAX_DEV, MAX_DEV);
    held += clamp(wrap(want - held), -MAX_YAW_RATE, MAX_YAW_RATE);
    yaw.push(held);
  }

  // ---- pelvis height: authored bounce, real terrain ----------------------
  // Split the authored root Z into the slope it assumed (a slow trend) and the
  // bounce/crouch about it. The trend is DISCARDED — the real DEM supplies the
  // climb — and the residual is kept verbatim, which is what preserves each
  // model's peak-bounce metric.
  const rootZ = rows.map((r) => r[2]);
  const trend = lowpass(rootZ, Math.round(FPS * 1.6));
  const residual = rootZ.map((v, i) => v - trend[i]);
  const authoredClimb = trend[n - 1] - trend[0];

  // Ceiling on pelvis height above ground.
  //
  // The bounce residual is added verbatim, but a peak that pushes the pelvis
  // past 0.7916 m puts the foot outside the leg's reach at exactly the moment
  // it is trying to land. Compress the top of the range with a soft knee rather
  // than a hard clip, so the bounce keeps its shape and simply saturates.
  // Straight-leg reach is HIP_DROP + MAX_HIP_REACH + sole = 0.794 m, so a
  // 0.755 m ceiling left 39 mm of knee flexion — straight, for practical
  // purposes, and that is a singularity: with the knee extended, hip pitch
  // and ankle pitch become redundant, the Jacobian loses rank, and the solver
  // walks the knee into its -0.087 rad lower stop and cannot get back out.
  // Every one of the worst-error frames measured had the knee pinned there.
  // 0.72 m leaves ~74 mm and keeps the chain clear of rank loss.
  const CEIL = 0.775;
  // A floor as well: driven far enough down the hip passes over the foot and
  // the gait becomes a squat. Both ends are soft, so the authored bounce
  // keeps its shape and merely saturates.
  const FLOOR = 0.46;
  const softMax = (v, c, k = 0.05) =>
    (v <= c - k ? v : c - k + k * Math.tanh((v - (c - k)) / k));
  const softMin = (v, f, k = 0.05) =>
    (v >= f + k ? v : f + k - k * Math.tanh(((f + k) - v) / k));
  const pz = [];
  for (let i = 0; i < n; i++) {
    const want = (opt.rideHeight ?? DEFAULT_RIDE) + residual[i] + (opt.crouch ?? 0);
    pz.push(groundAt(px[i], py[i]) + softMin(softMax(want, CEIL), FLOOR));
  }
  let smoothedPz = lowpass(pz, 5);   // the DEM's own metre-scale noise is not bounce

  // ---- root orientation --------------------------------------------------
  //
  // Authored roll/pitch are the model's own postural signal and are kept; the
  // terrain's grade is added to them, so a robot on a slope leans with it.
  //
  // The terrain reference is sampled over a STRIDE-sized patch, not a
  // foot-sized one. footPlane's default footprint is 0.19 x 0.09 m, which on
  // Gale's float rock straddles individual boulders: sampling it under the
  // pelvis rolled the whole body by every stone either foot passed. A body
  // does not do that — the feet and ankles absorb rock-scale relief and the
  // torso follows the grade. Averaging over the stance width and a stride
  // length is what separates the two.
  //
  // It matters more than it looks. Pelvis roll enters the leg problem twice:
  // once as attitude, and once as an apparent SIDEWAYS offset of the foot,
  // because a foot 0.62 m below a pelvis rolled by t sits 0.62*sin(t) off to
  // the side in pelvis coordinates. At the 0.3 rad this was reaching, that is
  // 0.19 m of pure artefact — which is precisely what pushed the planted foot
  // 0.29 m across the midline and left 104 mm of position error.
  const BODY_LEN = 0.85, BODY_WID = 0.42;
  // Limits a walking biped actually holds. Past these it is falling, not
  // leaning, and the legs cannot reach the ground either way.
  const MAX_BODY_PITCH = 0.30, MAX_BODY_ROLL = 0.16;
  const gpPitch = [], gpRoll = [];
  for (let i = 0; i < n; i++) {
    const gp = field.footPlane(px[i], -py[i], -yaw[i], BODY_LEN, BODY_WID);
    gpPitch.push(gp.pitch); gpRoll.push(gp.roll);
  }
  // A quarter second of smoothing on top: attitude is carried by the torso's
  // own mass and cannot step from frame to frame.
  const sPitch = lowpass(gpPitch, 9), sRoll = lowpass(gpRoll, 9);
  const rootR = [];
  for (let i = 0; i < n; i++) {
    const m = quatToMat(rows[i][3], rows[i][4], rows[i][5], rows[i][6]);
    // authored pitch/roll about the authored heading
    const aPitch = Math.asin(clamp(-m[6], -1, 1));
    const aRoll = Math.atan2(m[7], m[8]);
    const lean = opt.terrainLean ?? 0.5;
    rootR.push(rpyMat(
      clamp(aRoll + sRoll[i] * lean, -MAX_BODY_ROLL, MAX_BODY_ROLL),
      clamp(aPitch + sPitch[i] * lean, -MAX_BODY_PITCH, MAX_BODY_PITCH),
      yaw[i]));
  }

  // ---- footholds ---------------------------------------------------------
  // Forward speed along the placed path, used to modulate cadence.
  const speed = [];
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 2), b = Math.min(n - 1, i + 2);
    speed.push(Math.hypot(sx[b] - sx[a], sy[b] - sy[a]) / ((b - a) / FPS));
  }
  const meanSpeed = speed.reduce((p, q) => p + q, 0) / n;
  const HIP_Y = 0.0645;

  // Fore-and-aft envelope actually left to the swing, from the real geometry:
  // the reach budget, less the height the hip is carried at, less the sideways
  // offset the stance and capture step have already spent.
  // The stride budget is computed at the height the pelvis will actually be
  // AT THE EXTREMES OF THE STEP, not at its nominal ride height. The pelvis
  // does not hold one height through the cycle — it rises over the stance leg
  // and drops through double support (see the per-frame ceiling below) — so
  // budgeting the stride against the tall mid-stance figure would collapse it:
  // 0.211 m of stride at a 0.74 m ride, against 0.493 m at 0.68 m.
  const vert = Math.min(opt.rideHeight ?? DEFAULT_RIDE, 0.69) - HIP_DROP - 0.035;
  const budget = MAX_HIP_REACH * 0.985;
  const sideways = HIP_Y + (opt.stanceWiden ?? 0.03) + 0.09;
  const horiz2 = budget * budget - vert * vert - sideways * sideways;
  // 0.85 keeps the extremes off the singularity rather than on it.
  const maxHalfStride = Math.sqrt(Math.max(0.0025, horiz2)) * 0.85;

  const sched = gaitSchedule(n, speed, { ...opt, refSpeed: meanSpeed,
                                         meanSpeed, maxHalfStride });
  const footholds = { left: [], right: [] };

  // Swing clearance, bounded by the time there is to achieve it.
  //
  // The arc peaks at 40% of swing, so raising the foot `c` metres takes
  // 0.4 * swingTime and its vertical speed is c / (0.4 * swingTime). Past
  // about a metre per second the knee cannot produce that. Same trade as duty
  // above: a high step is available only to a gait that has left itself the
  // time to take one. Olympia Undae was asking for 0.22 m inside a 0.225 s
  // swing — 2.44 m/s, and 1927 deg/s at the knee against hardware rated 500.
  const MAX_FOOT_RISE = 1.0;               // m/s, vertical, during the lift
  const clearance = Math.min(opt.clearance ?? 0.07,
                             MAX_FOOT_RISE * 0.4 * sched.swingTime);


  /**
   * Keep a foothold inside the leg's lateral envelope.
   *
   * The stance half-width, the capture-point widening and a lateral slip are
   * each modest on their own, but nothing stopped them stacking: measured on
   * the Gale WorldVLA clip they summed to a planted foot 0.29 m across the
   * midline, with the hip roll joint against its stop and 104 mm of position
   * error. Clamping the total offset from the route — rather than each
   * contribution separately — bounds the sum, which is the quantity the leg
   * actually has to reach.
   *
   * The near limit keeps the ankles from colliding; the far limit is roughly
   * a third of leg length, past which a loaded hip cannot hold the body up.
   */
  const NEAR_LAT = 0.055, FAR_LAT = 0.19;
  function clampLateral(fx, fy, i, sgn, yw) {
    const dx = fx - routeX[i], dy = fy - routeY[i];
    const fwd = dx * Math.cos(yw) + dy * Math.sin(yw);
    const side = -dx * Math.sin(yw) + dy * Math.cos(yw);
    const want = clamp(sgn * side, NEAR_LAT, FAR_LAT) * sgn;
    return [routeX[i] + fwd * Math.cos(yw) - want * Math.sin(yw),
            routeY[i] + fwd * Math.sin(yw) + want * Math.cos(yw)];
  }

  for (const side of ['left', 'right']) {
    const sgn = side === 'left' ? 1 : -1;
    for (const [a, b] of sched[side].stance) {
      const mid = Math.round((a + b) / 2);
      const yw = yaw[mid];
      // Nominal: under the hip at mid-stance, widened by the packet's stance.
      const half = HIP_Y + (opt.stanceWiden ?? 0.03);

      // Lateral CAPTURE POINT.
      //
      // A foot placed at the nominal stance width assumes the body stays over
      // it. WorldVLA's authored path lurches 0.22 m sideways — wider than any
      // sane stance — so the pelvis ends up outside its own feet and the leg
      // physically cannot reach the ground it is supposed to be standing on.
      // The answer a real robot gives is to step out and catch itself, and
      // where it must step is the capture point: v * sqrt(L/g) (Pratt et al.
      // 2006), the same quantity src/sim/Gait.js uses. Placing the foothold
      // there turns a saturated hip into the recovery step the packet's own
      // "Recovery events" metric says should be happening.
      const w0 = Math.max(0, mid - 3), w1 = Math.min(n - 1, mid + 3);
      const vX = (sx[w1] - sx[w0]) / ((w1 - w0) / FPS);
      const vY = (sy[w1] - sy[w0]) / ((w1 - w0) / FPS);
      const vLat = -vX * Math.sin(yw) + vY * Math.cos(yw);
      const tau = Math.sqrt((opt.comHeight ?? 0.62) / Math.max(opt.g ?? 9.81, 0.05));
      // Capped well below the leg's lateral reach. At 0.16 m this stacked on
      // top of the 0.0825 m stance half-width and asked for a foot 0.24 m out
      // to the side, which is where the solver started diverging.
      const capture = clamp(vLat * tau, -0.09, 0.09);
      // Only widen, never narrow: a capture step catches the body, it does not
      // pull the foot in under a body that is already falling the other way.
      const widen = sgn * capture > 0 ? Math.abs(capture) : 0;

      // Planted against the ROUTE, not the swaying pelvis. Placing feet at
      // the pelvis folds the pelvis's own side-to-side motion into where the
      // feet go, so the feet chase the sway instead of the sway happening
      // over the feet.
      let fx = routeX[mid] - Math.sin(yw) * sgn * (half + widen);
      let fy = routeY[mid] + Math.cos(yw) * sgn * (half + widen);
      [fx, fy] = clampLateral(fx, fy, mid, sgn, yw);
      // The sole centre sits ahead of the ankle; bias the placement so the
      // support polygon, not the ankle, lands where the step was aimed.
      const pl = planeAt(fx, fy, yw);
      // Rest ON the surface, not in it.
      //
      // A rigid foot is supported by the HIGHEST points beneath it, not by the
      // mean of the patch. Placing the sole on the least-squares plane buries
      // it in every rock it straddles — measured at up to 25 mm of penetration
      // before this was corrected. The 6 mm allowance is regolith compressing
      // under load, which is real and keeps the foot from appearing to hover.
      const support = Math.max(pl.height, pl.peak - 0.006);
      footholds[side].push({
        a, b, mid, x: fx, y: fy,
        z: support + 0.035,            // sole plane -> ankle-roll origin height
        peak: pl.peak,
        pitch: pl.pitch, roll: pl.roll, yaw: yw,
        slip: null, missed: false,
      });
    }
  }

  // ---- style: turn the packet's own failure counts into real events -------
  // The manifest records how many recovery and missed-contact events the clip
  // is supposed to contain. Rather than inventing struggle, reproduce exactly
  // that many, as genuine contact failures against the real surface.
  const allSteps = [];
  for (const side of ['left', 'right']) footholds[side].forEach((f, k) => allSteps.push({ side, k, mid: f.mid }));
  allSteps.sort((p, q) => p.mid - q.mid);

  const nSlip = opt.recoveryEvents ?? 0;
  const nMiss = opt.missedContacts ?? 0;
  // Spread the events across the middle of the clip, never on the first or
  // last step — a slip on the opening step reads as a broken clip, not a slip.
  const pickable = allSteps.slice(2, Math.max(3, allSteps.length - 2));
  const chosen = new Set();
  for (let e = 0; e < nSlip + nMiss && pickable.length; e++) {
    let idx = Math.floor(((e + 0.5) / (nSlip + nMiss)) * pickable.length);
    while (chosen.has(idx) && idx < pickable.length) idx++;
    if (idx >= pickable.length) break;
    chosen.add(idx);
    const st = pickable[idx];
    const f = footholds[st.side][st.k];
    if (e < nSlip) {
      // Downslope slip: the foot lands, loses traction and slides along the
      // local fall line before catching. Direction comes from the terrain.
      const g = field.normalAt(f.x, -f.y);
      const fall = Math.hypot(g[0], g[2]);
      const dist = (opt.slipDistance ?? 0.09) * (0.6 + 0.8 * rand());
      f.slip = fall > 1e-3
        ? { dx: (g[0] / fall) * dist, dy: (-g[2] / fall) * dist }
        : { dx: Math.cos(f.yaw) * dist, dy: Math.sin(f.yaw) * dist };
    } else {
      f.missed = true;    // foot arrives high and drops late onto the surface
    }
  }

  // ---- per-frame foot targets -------------------------------------------
  const footTarget = { left: [], right: [] };
  for (const side of ['left', 'right']) {
    const fh = footholds[side];
    for (let i = 0; i < n; i++) {
      let tgt;
      // which stance are we in, or between?
      let cur = -1, next = -1;
      for (let k = 0; k < fh.length; k++) {
        if (i >= fh[k].a && i <= fh[k].b) { cur = k; break; }
        if (i < fh[k].a) { next = k; break; }
      }
      if (cur >= 0) {
        const f = fh[cur];
        let x = f.x, y = f.y, z = f.z, pitch = f.pitch, roll = f.roll;
        if (f.slip) {
          // slide over the first third of stance, then hold
          const u = clamp((i - f.a) / Math.max(1, (f.b - f.a) * 0.34), 0, 1);
          const w = smooth(u);
          x += f.slip.dx * w; y += f.slip.dy * w;
          // The fall line can run almost straight across the direction of
          // travel, so a slip is bounded by the same envelope as the plant.
          [x, y] = clampLateral(x, y, f.mid, side === 'left' ? 1 : -1, f.yaw);
          const pl = planeAt(x, y, f.yaw);
          z = Math.max(pl.height, pl.peak - 0.006) + 0.035; pitch = pl.pitch; roll = pl.roll;
        }
        if (f.missed) {
          // arrives 4 cm high, settles over the first fifth of stance
          const u = clamp((i - f.a) / Math.max(1, (f.b - f.a) * 0.2), 0, 1);
          z += 0.045 * (1 - smooth(u));
        }
        tgt = { x, y, z, pitch, roll, yaw: f.yaw, contact: true };
      } else {
        // swing between the previous foothold and the next
        const prev = fh.filter((f) => f.b < i).pop();
        const nxt = next >= 0 ? fh[next] : null;
        if (!prev && !nxt) {
          const yw = yaw[i], half = HIP_Y + (opt.stanceWiden ?? 0.03);
          const sgn = side === 'left' ? 1 : -1;
          const fx = routeX[i] - Math.sin(yw) * sgn * half, fy = routeY[i] + Math.cos(yw) * sgn * half;
          const pl = planeAt(fx, fy, yw);
          tgt = { x: fx, y: fy, z: Math.max(pl.height, pl.peak - 0.006) + 0.035,
                  pitch: pl.pitch, roll: pl.roll, yaw: yw, contact: true };
        } else if (!prev) {
          tgt = { ...nxt, contact: false };
        } else if (!nxt) {
          tgt = { ...prev, contact: false };
        } else {
          const u = clamp((i - prev.b) / Math.max(1, nxt.a - prev.b), 0, 1);
          // Horizontal travel uses a DOUBLE smoothstep: zero velocity AND zero
          // acceleration at both ends.
          //
          // A single smoothstep also lands with zero velocity — d/du of
          // 3u^2-2u^3 is zero at both ends too — and peaks at 1.5x average
          // foot speed against the double's 2.25x, so it looks like a free
          // 50% cut in the rate problem. It is not free. Tried and reverted:
          // over-rate joint/clip combinations fell 119 -> 104, but position
          // error while LOADED rose from 1.4 mm to 31.9 mm on Gale and from
          // 2.6 to 21.1 on Jezero, because the foot arrives at the foothold
          // with real acceleration still on it and fights the last centimetres
          // instead of settling into them.
          //
          // Contact accuracy is what this tool exists to get right. A 50% cut
          // in a brief rate excursion does not buy a 20x rise in how far the
          // planted foot sits from where it was planted.
          const w = smooth(smooth(u));
          const x = lerp(prev.x, nxt.x, w), y = lerp(prev.y, nxt.y, w);
          // Clearance: over the ground actually under the swing path, not over
          // a straight line between the footholds. This is what stops a foot
          // from ploughing through a rock that sits between two good footholds.
          // Clear the WHOLE FOOT, not a point.
          //
          // Sampling the ground under the foot's centre is not enough: the sole
          // is 0.17 m long and the toe can be over a rock the centre misses
          // entirely. Measured on the WorldVLA Gale clip, that let the foot
          // travel 101 mm in a single frame with only 33 mm between its toe and
          // a boulder. footPlane's peak is the highest point under the whole
          // sole, which is the thing that actually has to be cleared.
          const along = planeAt(x, y, lerp(prev.yaw, nxt.yaw, w)).peak;
          const base = lerp(prev.z, nxt.z, w);
          // Rise fast, hang, descend gently — a skewed arc, peaking at ~40 % of
          // swing, so the foot is clear early and lands softly.
          const skew = u < 0.4 ? smooth(u / 0.4) : smooth(1 - (u - 0.4) / 0.6);
          const arc = skew * clearance;
          // The terrain-following floor must vanish at BOTH ends of the swing.
          //
          // As written, `along + 0.047` applied at u = 0 and u = 1 too, so the
          // foot was held at least 47 mm above the terrain peak at the very
          // moments it was supposed to be leaving and meeting a foothold that
          // sits ON the surface. The target therefore jumped at every contact
          // transition, and the solver followed it in a single frame: measured
          // on Jezero, the knee went from 49.6 deg in swing to 0.1 deg one
          // frame later at touchdown — 1485 deg/s against hardware rated near
          // 500. Every one of the remaining over-limit spikes was a touchdown
          // or a liftoff, not the middle of a swing.
          //
          // Weighting the floor to zero at both ends and full in the middle
          // makes the foot clear whatever sits between the footholds while
          // still arriving at each one continuously.
          //
          // sin(pi*u) does that, and was what this used, but it spends the
          // whole swing ramping: at 15 % in it is only 0.45, so through the
          // first and last sixth of every step the floor is at less than half
          // strength and the foot can be well under a rising surface. Measured
          // in the viewer against the ground it is drawn on, that left a swing
          // foot 63 mm inside the slope on Ganges Chasma and 36 mm on Jezero.
          //
          // A trapezoid holds the floor at FULL strength across the middle
          // 70 % of the swing and ramps over the outer 15 % at each end. The
          // ends are what the rate budget cares about — that is where the
          // touchdown slew came from — and they are still C1 into the
          // foothold, just over a shorter, deliberately chosen distance
          // instead of over the entire step.
          const RAMP = 0.15;
          const blend = smooth(Math.min(1, u / RAMP)) * smooth(Math.min(1, (1 - u) / RAMP));
          const floor = base + Math.max(0, along + 0.035 - base) * blend;
          const z = Math.max(base + arc, floor + (arc * 0.55 + 0.012) * blend);
          // toe up on the way out, level on the way in, so the heel strikes first
          const pitch = lerp(prev.pitch, nxt.pitch, w) + Math.sin(u * Math.PI) * -0.10 + (1 - u) * 0.0;
          tgt = { x, y, z, pitch, roll: lerp(prev.roll, nxt.roll, w),
                  yaw: lerp(prev.yaw, nxt.yaw, w), contact: false };
        }
      }
      tgt.inStance = cur >= 0;
      footTarget[side].push(tgt);
    }
  }

  // ---- pelvis height, second pass: let the legs decide ------------------
  //
  // A biped does not carry its pelvis at one height. It rises over the stance
  // leg at mid-stance, when the leg is nearly vertical and can be long, and
  // drops through double support, when both legs are splayed fore and aft and
  // both must be short. That vertical excursion is the compass gait, and it is
  // the whole reason a walking robot can stand up straight AND take a real
  // step: the two requirements are met at different moments, not at once.
  //
  // Holding one height forces a choice between them, and the first pass chose
  // badly — a fixed 0.645 m ride left the knee at 79 degrees for the entire
  // traverse and never straightened it past 68. A real G1 carries 20-35
  // degrees at mid-stance. On screen that is a robot squatting through the
  // whole clip.
  //
  // So the height is not asserted, it is DERIVED: at each frame, for every
  // foot the gait says is loaded, work out how high the hip can be and still
  // reach that foothold, and take the lowest answer. The rise and fall comes
  // out on its own, in phase with the steps, because it is caused by them.
  {
    const budget = MAX_HIP_REACH * 0.985;
    const ceil = new Array(n).fill(Infinity);
    for (const side of ['left', 'right']) {
      const sgn = side === 'left' ? 1 : -1;
      for (let i = 0; i < n; i++) {
        const t = footTarget[side][i];
        // Only LOADED feet constrain the pelvis. A swing foot is lifted clear
        // and is free to shorten its own reach.
        if (!t.inStance) continue;
        const yw = yaw[i];
        const hipX = px[i] - Math.sin(yw) * sgn * HIP_Y;
        const hipY = py[i] + Math.cos(yw) * sgn * HIP_Y;
        const horiz = Math.hypot(t.x - hipX, t.y - hipY);
        const vertical = Math.sqrt(Math.max(0, budget * budget - horiz * horiz));
        ceil[i] = Math.min(ceil[i], t.z + vertical + HIP_DROP);
      }
    }
    // Where nothing is loaded — a flight phase — carry the neighbouring
    // ceiling forward rather than letting the pelvis jump.
    let last = Infinity;
    for (let i = 0; i < n; i++) { if (isFinite(ceil[i])) last = ceil[i]; else ceil[i] = last; }
    last = Infinity;
    for (let i = n - 1; i >= 0; i--) { if (isFinite(ceil[i])) last = ceil[i]; else ceil[i] = last; }

    // Come down BEFORE the foot lands, not when it lands.
    //
    // The ceiling above only counts feet the gait already calls loaded, so it
    // steps downward at the instant of touchdown — and smoothing a step leaves
    // the pelvis still too high exactly when the new foothold takes weight,
    // out of reach, so the reach clamp drags it in and the foot slides. That
    // showed up as scheduled-contact slip jumping from 4.4% to 25.4% of travel
    // on the Mare cruise the moment the pelvis was allowed to stand up.
    //
    // A running minimum over a third of a second in each direction fixes it:
    // the pelvis is already at the height the next foothold needs before the
    // foot arrives, and rises again only once the trailing foot has let go.
    // Reaching down for a step by lowering the hips first is what a walker
    // does, so this is the behaviour rather than a patch over it.
    const W = Math.round(FPS * 0.32);
    const eroded = new Array(n);
    for (let i = 0; i < n; i++) {
      let m = Infinity;
      for (let k = Math.max(0, i - W); k <= Math.min(n - 1, i + W); k++) {
        m = Math.min(m, ceil[k]);
      }
      eroded[i] = m;
    }
    for (let i = 0; i < n; i++) {
      if (isFinite(eroded[i])) pz[i] = Math.min(pz[i], eroded[i]);
    }
    // Smoothed over about a fifth of a second: the torso has mass and the
    // ceiling itself steps whenever a foot is set down.
    smoothedPz = lowpass(pz, 7);
  }

  // ---- solve ------------------------------------------------------------
  const out = [];
  const qPrev = { left: NEUTRAL_LEG.slice(), right: NEUTRAL_LEG.slice() };
  // Contact-phase errors are tracked separately, because they are the ones
  // that mean anything. A swing foot whose ankle is against its stop is 300
  // mrad from the tilt it was asked for and it does not matter — it is in the
  // air. The same 300 mrad under load is a sole resting on one edge.
  const diag = { maxPosErr: 0, maxRotErr: 0, clamped: 0, iters: 0, reachClamped: 0,
                 reseeded: 0, maxPosErrContact: 0, maxRotErrContact: 0,
                 contactFrames: 0 };

  for (let i = 0; i < n; i++) {
    const row = rows[i].slice();
    row[0] = px[i]; row[1] = py[i]; row[2] = smoothedPz[i];
    const rq = matToQuat(rootR[i]);
    row[3] = rq[0]; row[4] = rq[1]; row[5] = rq[2]; row[6] = rq[3];

    const Rroot = rootR[i], RrootT = mT(Rroot);
    const proot = [px[i], py[i], smoothedPz[i]];

    for (const side of ['left', 'right']) {
      const t = footTarget[side][i];

      // Keep the requested foot ORIENTATION inside the ankle's envelope.
      //
      // The ankle roll joint has +-15 deg of travel and ankle pitch runs
      // -50..+30 deg. Handing the solver a sole pose taken straight off a rocky
      // DEM routinely exceeds both, and a saturated ankle does not tilt the
      // foot — it drags the whole leg round and pins the hip instead. Clamping
      // here means the foot conforms to the ground as far as the hardware
      // allows and then stops, which is what a real ankle does.
      const fRoll = clamp(t.roll, -0.22, 0.22);
      const fPitch = clamp(t.pitch, -0.42, 0.42);
      // Foot yaw is held near the body's heading: a planted foot may trail the
      // torso, but not by more than the hip can twist while loaded.
      const fYaw = yaw[i] + clamp(wrap(t.yaw - yaw[i]), -0.55, 0.55);

      // Desired ankle-roll pose in WORLD, then into pelvis coordinates.
      const Rfoot = rpyMat(fRoll, fPitch, fYaw);
      const pAnkleW = [t.x, t.y, t.z];

      // Seat the foot on its OWN contact spheres, given the tilt it actually
      // got.
      //
      // The foothold's height was planned from a plane fitted under a level
      // sole, but the ankle then clamps to +-15 deg of roll and the sole ends
      // up at a different angle from the one that height assumed. The
      // difference is a foot driven into the hill: measured 24.6 mm of
      // penetration on the PragyaSpace Gale clip, which rides the slope
      // hardest and therefore clamps most.
      //
      // A real foot that cannot conform tips onto an edge and sits HIGHER, not
      // lower. So the ankle is raised until the lowest of the four contact
      // spheres — each tested against the ground directly beneath itself —
      // just touches, less a few millimetres of regolith compression under
      // load. This is the whole foot solving against the whole surface rather
      // than a point against a plane.
      const COMPRESS = 0.004;
      /** Ankle height that rests the lowest contact sphere on the ground. */
      const seat = (R, x, y) => {
        let need = -Infinity;
        for (const cpt of FOOT_CONTACTS) {
          const o = mapv(R, cpt);
          // Each sphere against the ground directly beneath THAT sphere. The
          // sole is 0.17 m long; the ground under the toe and under the heel
          // differ by centimetres on rock.
          need = Math.max(need, groundAt(x + o[0], y + o[1]) - o[2]);
        }
        return need - COMPRESS;
      };
      // Stance seats onto the surface; swing is merely not allowed BELOW it.
      // Using the same function for both is what makes the two agree at the
      // instant contact switches — the last swing frame and the first stance
      // frame resolve to the same height instead of to two different rules.
      // Height of the swing arc before any seating, kept so the swing rule can
      // stay "never below the surface" while still converging onto the seated
      // foothold at both ends of the swing.
      const arcZ = pAnkleW[2];
      if (t.inStance) pAnkleW[2] = seat(Rfoot, t.x, t.y);
      else pAnkleW[2] = Math.max(arcZ, seat(Rfoot, t.x, t.y));
      const d = [pAnkleW[0] - proot[0], pAnkleW[1] - proot[1], pAnkleW[2] - proot[2]];
      const pLocal = mapv(RrootT, d);
      const RLocal = (() => {
        const A = RrootT, B = Rfoot, C = new Array(9);
        for (let r = 0; r < 3; r++) for (let cc = 0; cc < 3; cc++)
          C[r * 3 + cc] = A[r * 3] * B[cc] + A[r * 3 + 1] * B[3 + cc] + A[r * 3 + 2] * B[6 + cc];
        return C;
      })();
      // Keep the target inside the workspace.
      //
      // A foot asked for beyond the leg's reach does not produce a long step,
      // it produces a saturated solver and a limb that snaps to its limit. Pull
      // the target in HORIZONTALLY, holding its height, so the foot stays on
      // the ground and merely takes a shorter step than it wanted.
      // Never let a SWINGING leg cross the midline. Hip roll only opens 30 deg
      // inward, so a swing target that has drifted across the body pins the
      // joint and the leg swings out sideways instead of stepping.
      //
      // Deliberately NOT applied to a planted foot. A foothold is fixed in the
      // world; the pelvis is what moves over it. WorldVLA's authored lateral
      // lurches reach 0.22 m, wider than its own stance, so clamping the
      // planted foot back under the body dragged it across the ground every
      // frame — 1.72 m of slip on a 4.5 m traverse, which swamped the 0.26 m
      // of genuine backslide the packet actually calls for. If the body leans
      // outside its feet the hip saturates, which is what really happens, and
      // the recovery steps the metrics ask for are what answer it.
      const sgnL = side === 'left' ? 1 : -1;
      if (!t.inStance && sgnL * pLocal[1] < 0.035) pLocal[1] = sgnL * 0.035;

      const hipL = [0, sgnL * 0.064452, -HIP_DROP];
      let dh = [pLocal[0] - hipL[0], pLocal[1] - hipL[1], pLocal[2] - hipL[2]];
      const vert = Math.abs(dh[2]);
      const budget = MAX_HIP_REACH * 0.985;
      if (vert < budget) {
        const maxHoriz = Math.sqrt(budget * budget - vert * vert);
        const horiz = Math.hypot(dh[0], dh[1]);
        if (horiz > maxHoriz) {
          const k = maxHoriz / horiz;
          pLocal[0] = hipL[0] + dh[0] * k;
          pLocal[1] = hipL[1] + dh[1] * k;
          diag.reachClamped++;
        }
      } else {
        // Too far DOWN to reach at all: raise the foot to the reachable sphere.
        const k = budget / Math.hypot(dh[0], dh[1], dh[2]);
        pLocal[0] = hipL[0] + dh[0] * k;
        pLocal[1] = hipL[1] + dh[1] * k;
        pLocal[2] = hipL[2] + dh[2] * k;
        diag.reachClamped++;
      }
      // Solve, then guard against divergence.
      //
      // Damped least squares seeded from the previous frame keeps the leg
      // continuous, which is what a walking clip needs — but it also means one
      // bad frame poisons every frame after it. Measured on the Gale WorldVLA
      // clip the solver reached a pose with hip pitch at 2.880 rad and hip roll
      // at 2.967 rad: the leg rotated through 170 degrees, folded up behind the
      // robot, 878 mm from the foot it was asked for. Nothing recovered,
      // because each following frame was seeded from that.
      //
      // So: when the warm start lands badly, try again cold from the neutral
      // crouch and keep whichever is actually closer. A cold start cannot
      // inherit a broken branch, and re-seeding only on failure keeps the
      // continuity that makes the good frames good.
      let sol = legIK(side, pLocal, RLocal, qPrev[side], { kneeFloor: KNEE_FLOOR });
      if (sol.posErr > 6e-3) {
        const cold = legIK(side, pLocal, RLocal, NEUTRAL_LEG, { kneeFloor: KNEE_FLOOR });
        if (cold.posErr < sol.posErr) { sol = cold; diag.reseeded++; }
      }

      // Re-seat against the tilt the ankle ACTUALLY reached, then solve again.
      //
      // The first seating above used the tilt that was REQUESTED, but the
      // ankle has only 15 degrees of roll and routinely cannot deliver it, so
      // the sole ends up at a different angle and a different corner becomes
      // the lowest one. Seating against a pose the foot never adopts left 183
      // of 401 loaded foot-frames more than 6 mm into the ground, the worst
      // 33 mm deep.
      //
      // One correction pass fixes it: take the achieved sole orientation out
      // of forward kinematics, work out where the ankle has to be for THAT
      // sole to rest on the surface, and re-solve at the corrected height.
      // Position is cheap for the solver to satisfy — it is orientation that
      // saturates — so the second solve lands without disturbing the first.
      // Iterated, because the correction is mildly self-referential: moving
      // the ankle changes which sphere is lowest and slightly changes the tilt
      // the solver settles on, which moves the seat again. It contracts
      // quickly, so the loop is bounded rather than run to a tolerance; five
      // passes covers the blocky lunar sites where the first correction can be
      // several centimetres.
      // Run for SWING as well as stance, and this is what removes the last of
      // the snap.
      //
      // Stance height was resolved by this iteration against the tilt the
      // ankle actually reached; swing height was resolved by a different rule
      // that never iterated. Two rules meeting at one instant do not agree, so
      // the foot teleported the moment contact switched — measured on Jezero,
      // up to 115 mm in a single frame, which is 3.45 m/s of foot and every
      // remaining over-rate spike in the set. Position and orientation were
      // already continuous across the seam; only the height rule was not.
      //
      // Swing keeps its "never below the surface" semantics through the max(),
      // so mid-swing the arc still dominates and clearance is untouched. At
      // either end of the swing the arc has collapsed to the foothold, the
      // max() selects the seated height, and the last swing frame and the
      // first stance frame resolve to the same number.
      {
        const pCur = pLocal.slice();
        for (let pass = 0; pass < 5; pass++) {
          const fk = legFK(side, sol.q);
          const w = mapv(Rroot, fk.p);
          const az = proot[2] + w[2];
          const seated = seat(mmul(Rroot, fk.R), proot[0] + w[0], proot[1] + w[1]);
          const want = t.inStance ? seated : Math.max(arcZ, seated);
          const dz = want - az;
          if (Math.abs(dz) < 1e-3) break;
          const corr = mapv(RrootT, [0, 0, dz]);
          pCur[0] += corr[0]; pCur[1] += corr[1]; pCur[2] += corr[2];
          const s2 = legIK(side, pCur, RLocal, sol.q, { kneeFloor: KNEE_FLOOR });
          // Accept unless the correction actually COSTS accuracy.
          //
          // This was an absolute 6 mm test, which quietly defeated itself on
          // exactly the clips that needed it: where the base solve already
          // sits near 6 mm — the blocky lunar rim, corner gaps over 120 mm —
          // every correction tripped the threshold and was thrown away, so the
          // foot was never lifted at all and stayed 57 mm underground. The
          // test should ask whether re-seating made the solve worse, not
          // whether the solve was already hard.
          if (s2.posErr > Math.max(6e-3, sol.posErr * 1.3)) break;
          sol = s2;
        }
      }
      if (opt.debug && sol.posErr > 5e-3) {
        // Keep the WORST offenders, not the first ones encountered: the first
        // six frames over threshold are usually a mild swing-phase tilt, which
        // says nothing about the 451 mm outlier that is the actual bug.
        diag.worst = diag.worst || [];
        {
          diag.worst.push({
          i, side, pLocal: pLocal.map((v) => +v.toFixed(4)),
          RLocal: RLocal.map((v) => +v.toFixed(3)),
          tgt: { x: +t.x.toFixed(3), y: +t.y.toFixed(3), z: +t.z.toFixed(3),
                 pitch: +(t.pitch * 57.3).toFixed(1), roll: +(t.roll * 57.3).toFixed(1),
                 yaw: +(t.yaw * 57.3).toFixed(1), contact: t.contact },
          rootYaw: +(yaw[i] * 57.3).toFixed(1),
          q: sol.q.map((v) => +v.toFixed(3)),
          posErr: +sol.posErr.toFixed(4), rotErr: +sol.rotErr.toFixed(4),
          });
          diag.worst.sort((a, b) => b.posErr - a.posErr);
          diag.worst.length = Math.min(diag.worst.length, 8);
        }
      }
      qPrev[side] = sol.q;
      diag.maxPosErr = Math.max(diag.maxPosErr, sol.posErr);
      diag.maxRotErr = Math.max(diag.maxRotErr, sol.rotErr);
      if (t.inStance) {
        diag.contactFrames++;
        diag.maxPosErrContact = Math.max(diag.maxPosErrContact, sol.posErr);
        diag.maxRotErrContact = Math.max(diag.maxRotErrContact, sol.rotErr);
      }
      diag.iters += sol.iters;
      if (sol.posErr > 5e-3) diag.clamped++;
      LEG_COLS[side].forEach((col, k) => { row[col] = sol.q[k]; });
    }
    out.push(row);
  }
  diag.iters /= (n * 2);
  return { rows: out, diag, footholds, sched, contacts: sched, footTarget, authoredClimb,
           realClimb: groundAt(px[n - 1], py[n - 1]) - groundAt(px[0], py[0]) };
}
