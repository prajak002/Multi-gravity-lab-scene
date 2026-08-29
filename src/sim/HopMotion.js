/**
 * HopMotion — generate a hop, a lope or a trip against real terrain.
 *
 * This runs in BOTH the build tool and the browser, and that is the point.
 * tools/build_motions.mjs bakes the default thrust so a scene opens instantly;
 * the arena regenerates the whole clip on this same code the moment the thrust
 * slider moves, because a jump you cannot change the thrust of does not show
 * you what thrust does.
 *
 * It is the counterpart of tools/retarget.mjs rather than a copy of it. The
 * retargeter takes a motion someone else authored and reconciles it with the
 * ground; this one has no source motion at all. Given `g` and a thrust,
 * src/sim/Ballistic.js works out how fast the machine can leave the ground,
 * how long it stays up, how far it can push, and how much attitude it loses
 * doing it — and the gait is whatever falls out of that. The only thing the
 * two share is that the feet are solved against the same SiteField by the same
 * IK.
 */
import { SiteField, SURFACE_PROFILES } from '../terrain/SiteField.js';
import {
  MOTIONS, instability, toppleTime,
  ARM_INERTIA, BODY_INERTIA, ARM_TUCK, ARM_RATE, ARM_SWEEP,
  G1_MASS, FOOT_HALF,
} from './Ballistic.js';
import {
  legIK, legFK, LEG_CHAIN, NEUTRAL_LEG, rpyFixed, matToQuat, mmul, mapv, mT,
  contactPoints,
} from './G1Kinematics.js';

const FPS = 30;
const DURATION = 10;                      // seconds per clip
/**
 * Hip height above the sole when the robot is standing, metres.
 *
 * The working extension, not the straight leg: legLength() says a straight leg
 * is 0.766 m, and standing there would leave the knee on the singularity where
 * it has no authority at all. 0.70 m is the ride height the packets use and
 * keeps the knee near 0.5 rad, which is where it can both push and absorb.
 */
const LEG_STAND = 0.70;
/**
 * Height of the ankle-roll origin above the sole plane, metres.
 *
 * FOOT_CONTACTS in G1Kinematics puts the four contact spheres at z = -0.035 in
 * the foot link's frame, so this is where the sole is relative to the point the
 * IK actually places.
 */
const SOLE_DROP = 0.035;
/**
 * How far a contact sphere is allowed to sit below the surface, metres.
 *
 * Regolith compresses under load, so a few millimetres is real rather than an
 * error — tools/retarget.mjs plants the packets' soles to the same depth, and
 * matching it keeps the generated motions and the retargeted ones directly
 * comparable in the audit.
 */
const REGOLITH = 0.004;
/** Seating iterations. See the seating pass for why four rather than two. */
const SEAT_PASSES = 4;
/** The URDF's ankle roll stop, rad — LEG_LIMITS' last entry, either side. */
const ANKLE_ROLL_LIMIT = 0.2618;
/** The URDF's ankle dorsiflexion stop, rad — LEG_LIMITS' ankle pitch lower. */
const ANKLE_PITCH_LIMIT = 0.87267;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;
const smooth = (t) => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));

/** Half the distance between the hip yaw origins — the natural stance width. */
const HIP_Y = Math.abs(LEG_CHAIN.left[0].t[1]);

/**
 * The hop timeline: when each push happens and where each foot lands.
 *
 * Built before any pose is, because a hop's landing foothold has to be known
 * while the robot is still on its way up — the swing arc is planned between a
 * foothold it has left and one it has not reached yet, exactly as in the
 * retargeter.
 */
function plan(motion, phys, opt) {
  const { heading, origin, field } = opt;
  const ux = Math.cos(heading), uy = Math.sin(heading);
  const cycle = phys.stanceTime + phys.flightTime;
  const speed = phys.speedCeiling * (motion.pushFraction ?? 1);

  const hops = [];
  let t = 0, x = origin[0], y = origin[1];
  // `lead` alternates for a lope and stays null for a two-foot bound.
  let lead = 'left';
  while (t < DURATION + cycle) {
    const takeoff = t + phys.crouchTime + phys.pushTime;
    const land = takeoff + phys.flightTime;
    const dx = speed * phys.flightTime;
    hops.push({
      t0: t, takeoff, land, end: land + phys.landTime,
      x0: x, y0: y, x1: x + ux * dx, y1: y + uy * dx,
      // Which feet carry this hop. A bound uses both; a lope alternates, and
      // the swinging foot is the one that will carry the NEXT one.
      feet: motion.feet === 'together' ? ['left', 'right'] : [lead],
    });
    x += ux * dx; y += uy * dx;
    t = land + phys.landTime;
    lead = lead === 'left' ? 'right' : 'left';
  }
  return { hops, cycle, speed };
}

/**
 * Where a foot is planted for a given hop.
 *
 * Offset to its own side of the centreline by the stance width, and seated on
 * the terrain — the height comes from SiteField, so a foothold is on the
 * ground by construction and the pelvis is derived from IT rather than the
 * other way round.
 */
function foothold(side, x, y, heading, field, widen) {
  const sgn = side === 'left' ? 1 : -1;
  const half = HIP_Y + widen;
  const fx = x - Math.sin(heading) * sgn * half;
  const fy = y + Math.cos(heading) * sgn * half;
  const pl = field.footPlane(fx, -fy, heading);
  // legIK positions the ANKLE-ROLL ORIGIN, and the sole plane sits SOLE_DROP
  // below it — so a foothold taken straight off the terrain buries the foot by
  // exactly that much. Measured before the offset was here: -51 mm of sole
  // under the surface against the -4 mm the retargeter deliberately allows for
  // regolith compression, and 35 of those 51 mm were this.
  return { x: fx, y: fy, z: Math.max(pl.height, pl.peak - 0.006) + SOLE_DROP,
           pitch: pl.pitch, roll: pl.roll };
}

/**
 * How far this foothold has to rise for the sole to clear the ground.
 *
 * The foothold height is planned from a plane fitted under a LEVEL sole. The
 * ankle then clamps to its URDF limits — +-15 degrees of roll — so on ground
 * steeper than that the sole ends up at a different angle from the one it was
 * planned for, and a foot that cannot conform tips onto an edge and sits
 * HIGHER. Solved without accounting for it, the opposite corner goes into the
 * hill: 69.5 mm on Malapert Massif before this existed.
 *
 * So the achieved sole is taken back out of forward kinematics, the deepest of
 * the four contact spheres is measured against the ground directly beneath IT,
 * and the foothold is raised by that much less the regolith compression the
 * retargeter also allows. Iterated, because the correction changes the pose,
 * which changes the tilt, which changes the correction.
 *
 * The pose used is a nominal stance: the pelvis on the hop's own centreline at
 * standing height, body level. It is not the pose of any particular frame, and
 * that is the point — one answer for the whole stance, so the foothold does not
 * move under the robot while it is standing on it.
 *
 * The pelvis has to be on the CENTRELINE and not over the foot. Over the foot
 * the nominal hip roll is zero, which is right for a lope (one foot carries and
 * the body is above it) and badly wrong for a bound (both feet carry and the
 * body is between them). Solved over the foot, the bound's ankle came out at a
 * different roll from the one it actually reaches, the seating offset was
 * computed for a tilt the foot never has, and Malapert Massif went back to
 * 69.5 mm of sole in the hill while the lope on the same site sat at 0.0.
 */
function seatOffset(side, hold, field, heading, cx, cy, standZ) {
  const Rb = rpyFixed(0, 0, heading);
  const RbT = mT(Rb);
  const RT = mmul(RbT, rpyFixed(hold.roll, hold.pitch, heading));
  const px = cx, py = cy, pz = standZ;
  let lift = 0, q = NEUTRAL_LEG.slice();

  for (let pass = 0; pass < SEAT_PASSES; pass++) {
    const d = [hold.x - px, hold.y - py, (hold.z + lift) - pz];
    const pT = [
      RbT[0] * d[0] + RbT[1] * d[1] + RbT[2] * d[2],
      RbT[3] * d[0] + RbT[4] * d[1] + RbT[5] * d[2],
      RbT[6] * d[0] + RbT[7] * d[1] + RbT[8] * d[2],
    ];
    const r = legIK(side, pT, RT, q);
    q = r.q;
    let deepest = 0;
    for (const c of contactPoints(side, r.q)) {
      const w = mapv(Rb, c);
      const need = field.heightAt(px + w[0], -(py + w[1])) - (pz + w[2]);
      if (need > deepest) deepest = need;
    }
    if (deepest <= REGOLITH) break;
    lift += deepest - REGOLITH;
  }
  return lift;
}

/**
 * One clip: a motion, at one site, in that site's gravity.
 */
export function generateHop(field, site, motionId, opt = {}) {
  const motion = MOTIONS[motionId];
  if (!motion) throw new Error(`unknown motion ${motionId}`);

  const g = site.g;
  const phys = instability(g, { mu: site.friction ?? 0.45, thrust: opt.thrust ?? 1 });
  // The traverse runs along grid +x: pipeline/dem.py turned the patch's long
  // axis onto the scenario's own bearing, so +x IS the direction of travel and
  // there is nothing left to resolve here.
  const heading = 0;
  const origin = site.origin ?? [-Math.min(4, phys.range * 2), 0];

  const { hops, cycle } = plan(motion, phys, { heading, origin, field });
  const n = Math.round(DURATION * FPS);

  // Footholds, one pair per hop, planted and SEATED before any pose is solved.
  //
  // Seating has to happen here rather than per frame. Done per frame it is a
  // correction that switches on at the instant a foot becomes loaded, so the
  // target steps by however much the ankle could not conform — and the solver
  // takes that step in one frame. Measured on Hadley, that put 2030 deg/s
  // through the knee at touchdown, against the ~557 deg/s the landing itself
  // actually asks for. Seating the foothold instead means the swing arc ends
  // exactly where the stance begins, and nothing steps at all.
  const widen = 0.02 + (motion.feet === 'together' ? 0.015 : 0);
  for (const h of hops) {
    h.hold = {};
    const raw = {};
    for (const side of ['left', 'right']) raw[side] = foothold(side, h.x0, h.y0, heading, field, widen);
    const standZ = Math.max(raw.left.z, raw.right.z) + LEG_STAND;
    for (const side of ['left', 'right']) {
      const f = raw[side];
      f.z += seatOffset(side, f, field, heading, h.x0, h.y0, standZ);
      h.hold[side] = f;
    }

    // YOU CANNOT SQUAT AS DEEP ON A HILL.
    //
    // Absorbing a landing folds the leg, and folding the leg over a planted
    // foot is dorsiflexion — but on a grade the ankle has ALREADY spent part of
    // its range just getting the sole onto the slope. What is left is what the
    // absorb can use, and asking for more does not produce a deeper crouch, it
    // produces an ankle on its stop with the heel driven into the hill.
    //
    // Measured on Malapert Massif before this: six frames of three hundred, all
    // at the bottom of the landing absorb, with the sole 69 mm into the slope.
    // Everywhere else on the same clip was 0.0 mm — it is specifically the
    // deepest part of the crouch that the hardware cannot do here.
    const spent = Math.max(Math.abs(h.hold.left.pitch), Math.abs(h.hold.right.pitch));
    h.crouch = phys.crouch * clamp((ANKLE_PITCH_LIMIT - spent) / ANKLE_PITCH_LIMIT, 0.3, 1);
  }
  for (let i = 0; i < hops.length; i++) hops[i].after = hops[i + 1] || hops[i];

  // The trip: one hop where the leading foot catches instead of clearing.
  const tripHop = motion.catchAt ? Math.floor(hops.length * motion.catchAt) : -1;

  const rows = [];
  const contacts = [];
  // ATTITUDE IS INTEGRATED, NOT POSED.
  //
  // Every other quantity in this generator can be evaluated from the clock,
  // but attitude cannot: it is the running total of what the push put in, what
  // the arms took back in the air, and what the ankle managed to take back on
  // the ground. That is the whole instability story, and writing it as a
  // function of phase would erase it — a robot that starts every hop upright
  // can never fall over, however unstable the physics says it is.
  let attitude = motion.lean * 0.5;
  let lastT = 0;
  // How hard the ankle can push attitude back: bounded by tipping, not torque.
  const alphaAnkle = (G1_MASS * g * FOOT_HALF) / BODY_INERTIA;
  // What the arms take back per second of flight, spread over the flight.
  const armRate = phys.flightTime > 0 ? phys.armAuthority / phys.flightTime : 0;
  const FALLEN = 1.35;              // rad; past this the robot is on the ground
  // How far the elbow folds on the return stroke. ARM_TUCK is the inertia the
  // fold leaves; this is the joint travel that produces it.
  const ARM_TUCK_FOLD = 1.15;
  const qSeed = { left: NEUTRAL_LEG.slice(), right: NEUTRAL_LEG.slice() };
  // Saturation is REPORTED, not hidden. On a steep cross-slope the ankle runs
  // out of roll before the sole can lie flat, the foot rests on an edge, and no
  // amount of seating will fix it — measured on the Copernicus wall, a landing
  // bound sits with ankleRoll at exactly its -0.2618 stop and the knee fully
  // extended, 23 mm of sole into the hill. That is the machine, not the solver,
  // and it is the same limit that makes people switchback up steep ground.
  const diag = { maxPosErr: 0, maxRotErr: 0, rollSaturated: 0, loadedFrames: 0 };

  for (let i = 0; i < n; i++) {
    const t = i / FPS;
    // Which hop owns this instant.
    let hi = 0;
    while (hi + 1 < hops.length && hops[hi + 1].t0 <= t) hi++;
    const h = hops[hi];
    const nxt = h.after;

    // ---- pelvis -----------------------------------------------------------
    // Horizontal: constant through the flight, easing across the stance.
    let px, py, airborne = false, u = 0;
    if (t < h.takeoff) {
      u = clamp((t - h.t0) / Math.max(h.takeoff - h.t0, 1e-3), 0, 1);
      px = h.x0; py = h.y0;
    } else if (t < h.land) {
      u = clamp((t - h.takeoff) / Math.max(h.land - h.takeoff, 1e-3), 0, 1);
      airborne = true;
      px = lerp(h.x0, h.x1, u); py = lerp(h.y0, h.y1, u);
    } else {
      u = clamp((t - h.land) / Math.max(h.end - h.land, 1e-3), 0, 1);
      px = h.x1; py = h.y1;
    }

    // WHICH FOOTHOLDS ARE UNDER THE ROBOT RIGHT NOW.
    //
    // A hop has two of them and they are not the same place. The crouch and
    // the push happen on the footholds it takes off FROM; the absorption
    // happens on the ones it lands ON, a stride further along, which are also
    // the next hop's take-off holds. Using the take-off pair for the landing
    // too — which is what this did first — leaves the stance foot a full
    // stride behind the pelvis the instant the robot touches down, and the
    // solver drags it forward across the ground to catch up. It measured as
    // 146 % slip on the bound: the feet skated further than the robot moved.
    const hold = (t < h.land) ? h.hold : nxt.hold;
    const supFrom = Math.max(h.hold.left.z, h.hold.right.z);
    const supTo = Math.max(nxt.hold.left.z, nxt.hold.right.z);
    const support = (t < h.land) ? supFrom : supTo;
    let pz;
    if (airborne) {
      const tf = u * phys.flightTime;
      // THE ARC HAS TO LAND WHERE THE NEXT FOOTHOLDS ARE.
      //
      // Measured from the take-off height alone, the parabola returns to the
      // height it left at — but the landing footholds are a stride further
      // along, and on a slope that is somewhere else entirely. The pelvis
      // therefore teleported at the touchdown frame, by 191 mm on Malapert
      // Massif and 203 on Hadley: six metres per second of pelvis velocity in
      // a single frame, which is where the 2030 deg/s knee spike came from and,
      // on the steepest sites, why the sole ended up 69 mm into the hill.
      //
      // So the ballistic term rides on a baseline that runs from one support
      // height to the other. The arc above the take-off point is untouched —
      // apex and hang are still v0^2/2g and 2 v0/g, which is what the readout
      // quotes — and at u = 1 the ballistic term is exactly zero, so the body
      // arrives at standing height over the new footholds with nothing to jump.
      const base = lerp(supFrom, supTo, u);
      pz = base + LEG_STAND + phys.v0 * tf - 0.5 * g * tf * tf;
    } else if (t < h.takeoff) {
      // Crouch, then extend. The crouch is the slow half and the extension is
      // the fast one, which is what makes the push read as a push.
      const c = (t - h.t0) / Math.max(h.takeoff - h.t0, 1e-3);
      const cr = c < 0.62 ? smooth(c / 0.62) : 1 - smooth((c - 0.62) / 0.38);
      pz = support + LEG_STAND - h.crouch * cr;
    } else {
      // Landing: absorb, then come back up to standing.
      const a = clamp(u / 0.55, 0, 1);
      const back = clamp((u - 0.55) / 0.45, 0, 1);
      // The absorb belongs to the hop being LANDED on, which is the next one:
      // its footholds are the ones the ankle is fighting.
      pz = support + LEG_STAND - (nxt.crouch ?? h.crouch) * (smooth(a) - smooth(back));
    }

    // ---- attitude ---------------------------------------------------------
    const dt = Math.max(t - lastT, 0); lastT = t;
    if (airborne) {
      // FREE FLIGHT. No external torque, so the rate the push imparted is
      // conserved and simply integrates. The only way to change the body's
      // attitude is to move mass against it — which is what the arms are for,
      // and all they can give is (I_arm / I_body) of their own sweep.
      attitude += (phys.omega - Math.min(phys.omega, armRate)) * dt;
    } else {
      // STANCE. The ankle can drive it back toward upright, but only as hard
      // as m*g*d_foot allows before the centre of pressure leaves the sole.
      // At one sixth g that is one sixth the authority, whatever the motors
      // are rated at — which is why the error accumulates hop after hop on the
      // Moon and does not on Earth.
      const target = motion.lean * 0.5;
      const step = alphaAnkle * dt * phys.stanceTime * 0.5;
      attitude += clamp(target - attitude, -step, step);
    }
    attitude = clamp(attitude, -FALLEN, FALLEN);
    let pitchBody = attitude;

    // ---- the trip ---------------------------------------------------------
    // A caught toe is a pendulum about that toe, and the whole interest is how
    // LONG it takes: toppleTime() gives 0.93 s at 1 g against 2.28 s at one
    // sixth. So the pitch is driven on that clock rather than on the clip's.
    if (hi === tripHop) {
      const tt = toppleTime(g, 0.6);
      const since = clamp((t - h.takeoff) / tt, 0, 1);
      // Over, then caught: the arms come up and a foot is thrown out ahead.
      const over = since < 0.7 ? smooth(since / 0.7) : 1 - smooth((since - 0.7) / 0.3);
      pitchBody += 0.62 * over;
      attitude = pitchBody;
      pz -= 0.10 * over;
    }

    // ---- feet -------------------------------------------------------------
    const rowJoints = new Array(29).fill(0);
    const foot = {};
    for (const side of ['left', 'right']) {
      const carries = h.feet.includes(side);
      const down = !airborne && (carries || motion.feet === 'together');
      let f;
      if (down) {
        f = hold[side];
      } else {
        // Swing between this hop's foothold and the next, clearing whatever is
        // under the path — the same rule the retargeter uses, for the same
        // reason: a straight line between two good footholds goes through
        // anything sitting between them.
        const from = h.hold[side], to = nxt.hold[side];

        // ONE monotonic parameter for the whole swing.
        //
        // `u` restarts at each phase of the hop — crouch, flight, absorb — and
        // in a LOPE the trailing foot is off the ground for all three of them.
        // Driven by `u` it therefore travelled from one foothold to the next
        // during the crouch, snapped back at take-off and did it again, so the
        // target jumped twice per stride and the solver chased it in a single
        // frame: 3787 deg/s at the hip against hardware rated near 500, all of
        // it at contact transitions. A foot that is in the air for the whole
        // hop gets a parameter that spans the whole hop.
        const su = h.feet.includes(side)
          ? u                                            // down except in flight
          : clamp((t - h.t0) / Math.max(h.end - h.t0, 1e-3), 0, 1);
        const w = smooth(smooth(su));
        const fx = lerp(from.x, to.x, w), fy = lerp(from.y, to.y, w);
        const along = field.footPlane(fx, -fy, heading).peak;
        const base = lerp(from.z, to.z, w);
        // Clearance scales with the hop: a 1.18 m apex lifts the feet with it.
        const clear = clamp(0.06 + phys.apex * 0.35, 0.06, 0.55);
        const arc = Math.sin(Math.PI * su) * clear;
        const floorH = along + SOLE_DROP;
        // Orientation is interpolated between the two footholds, not held
        // level. Held level it snapped to the landing tilt in the single frame
        // the foot became loaded — ten to twenty degrees at once on a slope,
        // which the IK spreads up the whole leg and which was the other half of
        // that touchdown spike. The toe-up shaping is a sine, so it is zero at
        // both ends and adds no discontinuity of its own.
        f = {
          x: fx, y: fy, z: Math.max(base + arc, floorH),
          pitch: lerp(from.pitch, to.pitch, w) - Math.sin(Math.PI * su) * 0.12,
          roll: lerp(from.roll, to.roll, w),
        };
        if (hi === tripHop && side === h.feet[0]) {
          // The catch: the foot stops rising and stubs into the surface.
          // The stub: the sole stops rising and drives 15 mm into the
          // surface, which is what catching a toe on a rock is.
          f.z = Math.min(f.z, along + SOLE_DROP - 0.015);
        }
      }
      // Carried through to the solve, which is a separate loop: only a foot
      // that is taking load gets seated on its contact spheres.
      f.down = down;
      foot[side] = f;
    }

    // ---- solve the legs ---------------------------------------------------
    const Rbody = rpyFixed(0, pitchBody, heading);
    const RbodyT = [Rbody[0], Rbody[3], Rbody[6], Rbody[1], Rbody[4], Rbody[7], Rbody[2], Rbody[5], Rbody[8]];
    for (const side of ['left', 'right']) {
      const f = foot[side];
      // Foot pose in the PELVIS frame, which is what legIK wants.
      const d = [f.x - px, f.y - py, f.z - pz];
      const pT = [
        RbodyT[0] * d[0] + RbodyT[1] * d[1] + RbodyT[2] * d[2],
        RbodyT[3] * d[0] + RbodyT[4] * d[1] + RbodyT[5] * d[2],
        RbodyT[6] * d[0] + RbodyT[7] * d[1] + RbodyT[8] * d[2],
      ];
      const RT = mmul(RbodyT, rpyFixed(f.roll, f.pitch, heading));
      const r = legIK(side, pT, RT, qSeed[side]);

      // No per-frame seating: the foothold was already seated at plan time,
      // and correcting again here is exactly what put a step in the target at
      // every touchdown. What ankle roll cannot absorb is reported below
      // rather than chased.
      qSeed[side] = r.q;
      diag.maxPosErr = Math.max(diag.maxPosErr, r.posErr);
      diag.maxRotErr = Math.max(diag.maxRotErr, r.rotErr);
      if (f.down) {
        diag.loadedFrames++;
        if (Math.abs(Math.abs(r.q[5]) - ANKLE_ROLL_LIMIT) < 1e-3) diag.rollSaturated++;
      }
      const base = side === 'left' ? 0 : 6;
      for (let k = 0; k < 6; k++) rowJoints[base + k] = r.q[k];
    }

    // ---- waist and arms ---------------------------------------------------
    //
    // THE ARMS ARE THE ONLY ATTITUDE CONTROL A BODY IN FREE FLIGHT HAS.
    //
    // Not decoration, and not a walk cycle's counter-swing. In flight there is
    // nothing to push against, so the only way to rotate the body is to rotate
    // something else the other way: angular momentum is conserved, and moving
    // the arms through `sweep` counter-rotates the body by
    //
    //     (I_arm / I_body) * sweep  =  13.8 % of the sweep
    //
    // from the URDF's own inertia tensors. A single stroke is not enough, so
    // the arms WINDMILL — out with the elbow extended, back with it tucked, at
    // which point the arm's inertia is roughly 30 % of what it was and the
    // return costs back only a third of what the stroke earned. That asymmetry
    // is the whole trick, it is what the Apollo crews were doing, and it is why
    // the arms here extend on one half of the cycle and fold on the other.
    //
    // The amplitude is not chosen: it is how much correction is still owed.
    // With the attitude already back at nominal the arms hang; with 79 degrees
    // of tumble to work off they go to full sweep.
    const owed = Math.abs(attitude - motion.lean * 0.5);
    const effort = airborne ? clamp(owed / 0.35, 0, 1) : 0;
    // Which way to swing: to pitch the body nose-DOWN the arms go up and over.
    const dirn = Math.sign(attitude - motion.lean * 0.5) || 1;
    const cyc = 2 * Math.PI * ARM_RATE * (airborne ? u * phys.flightTime : 0);
    const stroke = Math.sin(cyc);
    // Extended through the working half, tucked through the return.
    const tuck = clamp(0.5 - 0.5 * Math.cos(cyc), 0, 1);

    rowJoints[12] = 0;                                    // waist yaw
    rowJoints[13] = 0;                                    // waist roll
    // The waist joins in: it is a second, smaller effector on the same axis.
    rowJoints[14] = clamp(pitchBody * 0.35 - dirn * effort * 0.18, -0.5, 0.5);

    const armUp = hi === tripHop ? 1.0 : 0;
    const swing = motion.armSwing * (airborne ? 0.25 : 0.22);
    for (const [b, sgn] of [[15, 1], [22, -1]]) {
      // shoulder pitch: the windmill itself, plus the reach on a trip
      rowJoints[b + 0] = clamp(
        -0.25 - swing - armUp * 1.1 - dirn * effort * ARM_SWEEP * 0.5 * stroke,
        -3.0, 1.5);
      // shoulder roll: open the arms out to raise their inertia on the stroke,
      // which is the half of the cycle that has to do the work
      rowJoints[b + 1] = sgn * clamp(0.18 + effort * 0.55 * (1 - tuck) + armUp * 0.45, 0, 1.6);
      rowJoints[b + 2] = 0;
      // elbow: straight through the stroke, folded through the return. This is
      // the inertia asymmetry that makes a windmill net anything at all.
      rowJoints[b + 3] = clamp(0.35 + effort * (ARM_TUCK_FOLD * tuck) + armUp * 0.6, 0, 1.5);
    }
    // A lope is asymmetric: the arms counter-rotate against the leading leg.
    if (motion.feet === 'alternating' && effort < 0.4) {
      const sd = h.feet[0] === 'left' ? 1 : -1;
      rowJoints[15] += sd * swing * 0.55;
      rowJoints[22] -= sd * swing * 0.55;
    }

    const q = matToQuat(Rbody);
    rows.push([px, py, pz, q[0], q[1], q[2], q[3], ...rowJoints]);
    contacts.push([
      !airborne && (h.feet.includes('left') || motion.feet === 'together'),
      !airborne && (h.feet.includes('right') || motion.feet === 'together'),
    ]);
  }

  return { rows, contacts, phys, field, heading, motion, site, g, diag };
}

// ---------------------------------------------------------------------------
