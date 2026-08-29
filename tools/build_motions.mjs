/**
 * build_motions.mjs — bake the gravity-driven motions onto a real NASA site.
 *
 * tools/retarget.mjs takes a packet someone else authored and reconciles it
 * with the ground. This does the opposite: there is no packet, the motion is
 * GENERATED from the field strength at the site, and the only thing it has in
 * common with the retargeter is that the feet are solved against the same
 * SiteField by the same IK.
 *
 * That is the point. A packet walk looks nearly identical on the Moon and on
 * Mars because it was authored once and re-planted twice — the gait is the
 * author's, and gravity only gets to adjust it. Here the gait does not exist
 * until `g` is known:
 *
 *     apex    = v0^2 / 2g          1.18 m on the Moon, 0.51 m on Mars
 *     hang    = 2 v0 / g           2.41 s on the Moon, 1.05 s on Mars
 *     duty    = stance / cycle     0.18 on the Moon, 0.34 on Mars
 *     speed  <= mu g t_stance      0.39 m/s on the Moon, 0.89 m/s on Mars
 *
 * with v0 the same to within 3 % on both, because it is set by the knee's rated
 * SPEED and not by the field. See src/sim/Ballistic.js for that derivation.
 *
 * The last of those four is the Apollo result that surprises people. Low
 * gravity does not make you fast — it makes you slow, because forward
 * acceleration comes from friction and friction comes from weight. The crews
 * loped not because it was exuberant but because at 0.39 m/s of available
 * push there was nothing else worth doing with a 2.4 second flight phase.
 *
 *   node tools/build_motions.mjs                    # every site x every motion
 *   node tools/build_motions.mjs moon_apollo15_hadley
 */
import fs from 'fs';
import { SiteField, SURFACE_PROFILES } from '../src/terrain/SiteField.js';
import { MOTIONS, strideFor, toppleTime } from '../src/sim/Ballistic.js';
import {
  legIK, legFK, LEG_CHAIN, NEUTRAL_LEG, rpyFixed, matToQuat, mmul, rotAxis,
} from '../src/sim/G1Kinematics.js';
import { audit } from './audit_contact.mjs';
import { rateAudit, JOINT_RATE_LIMIT } from './audit_rates.mjs';
import { JOINT_NAMES } from './build_scene.mjs';
import { SITES } from './sites.mjs';

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
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;
const smooth = (t) => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));

/** Half the distance between the hip yaw origins — the natural stance width. */
const HIP_Y = Math.abs(LEG_CHAIN.left[0].t[1]);

function loadField(demName, profile) {
  const meta = JSON.parse(fs.readFileSync(`public/dem/${demName}.json`, 'utf8'));
  const buf = fs.readFileSync(`public/dem/${demName}.f32`);
  const dem = new Float32Array(buf.buffer, buf.byteOffset, buf.length / 4);
  return new SiteField(dem, meta, SURFACE_PROFILES[profile] || SURFACE_PROFILES.moon_mare);
}

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
 * One clip: a motion, at one site, in that site's gravity.
 */
export function buildMotion(siteId, motionId) {
  const site = SITES[siteId];
  if (!site) throw new Error(`unknown site ${siteId}`);
  const motion = MOTIONS[motionId];
  if (!motion) throw new Error(`unknown motion ${motionId}`);

  const field = loadField(site.dem, site.profile);
  const g = site.g;
  const phys = strideFor(g, site.friction ?? 0.45);
  // The traverse runs along grid +x: pipeline/dem.py turned the patch's long
  // axis onto the scenario's own bearing, so +x IS the direction of travel and
  // there is nothing left to resolve here.
  const heading = 0;
  const origin = site.origin ?? [-Math.min(4, phys.range * 2), 0];

  const { hops, cycle } = plan(motion, phys, { heading, origin, field });
  const n = Math.round(DURATION * FPS);

  // Footholds, one pair per hop, planted before any pose is solved.
  const widen = 0.02 + (motion.feet === 'together' ? 0.015 : 0);
  for (const h of hops) {
    h.hold = {};
    for (const side of ['left', 'right']) {
      h.hold[side] = foothold(side, h.x0, h.y0, heading, field, widen);
      h.next = null;
    }
  }
  for (let i = 0; i < hops.length; i++) hops[i].after = hops[i + 1] || hops[i];

  // The trip: one hop where the leading foot catches instead of clearing.
  const tripHop = motion.catchAt ? Math.floor(hops.length * motion.catchAt) : -1;

  const rows = [];
  const contacts = [];
  const qSeed = { left: NEUTRAL_LEG.slice(), right: NEUTRAL_LEG.slice() };
  const diag = { maxPosErr: 0, maxRotErr: 0, clamped: 0 };

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
    const support = Math.max(hold.left.z, hold.right.z);
    let pz;
    if (airborne) {
      const tf = u * phys.flightTime;
      // z = v0 t - g t^2 / 2, on top of the height it left at.
      pz = support + LEG_STAND + phys.v0 * tf - 0.5 * g * tf * tf;
    } else if (t < h.takeoff) {
      // Crouch, then extend. The crouch is the slow half and the extension is
      // the fast one, which is what makes the push read as a push.
      const c = (t - h.t0) / Math.max(h.takeoff - h.t0, 1e-3);
      const cr = c < 0.62 ? smooth(c / 0.62) : 1 - smooth((c - 0.62) / 0.38);
      pz = support + LEG_STAND - phys.crouch * cr;
    } else {
      // Landing: absorb, then come back up to standing.
      const a = clamp(u / 0.55, 0, 1);
      const back = clamp((u - 0.55) / 0.45, 0, 1);
      pz = support + LEG_STAND - phys.crouch * (smooth(a) - smooth(back));
    }

    // ---- the trip ---------------------------------------------------------
    // A caught toe is a pendulum about that toe, and the whole interest is how
    // LONG it takes: toppleTime() gives 0.93 s at 1 g against 2.28 s at one
    // sixth. So the pitch is driven on that clock rather than on the clip's.
    let pitchBody = motion.lean * 0.5;
    if (hi === tripHop) {
      const tt = toppleTime(g, 0.6);
      const since = clamp((t - h.takeoff) / tt, 0, 1);
      // Over, then caught: the arms come up and a foot is thrown out ahead.
      const over = since < 0.7 ? smooth(since / 0.7) : 1 - smooth((since - 0.7) / 0.3);
      pitchBody += 0.62 * over;
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
        const w = smooth(smooth(u));
        const fx = lerp(from.x, to.x, w), fy = lerp(from.y, to.y, w);
        const along = field.footPlane(fx, -fy, heading).peak;
        const base = lerp(from.z, to.z, w);
        // Clearance scales with the hop: a 1.18 m apex lifts the feet with it.
        const clear = clamp(0.06 + phys.apex * 0.35, 0.06, 0.55);
        const arc = Math.sin(Math.PI * u) * clear;
        const floorH = along + SOLE_DROP;
        f = { x: fx, y: fy, z: Math.max(base + arc, floorH), pitch: 0, roll: 0 };
        if (hi === tripHop && side === h.feet[0]) {
          // The catch: the foot stops rising and stubs into the surface.
          // The stub: the sole stops rising and drives 15 mm into the
          // surface, which is what catching a toe on a rock is.
          f.z = Math.min(f.z, along + SOLE_DROP - 0.015);
        }
      }
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
      qSeed[side] = r.q;
      diag.maxPosErr = Math.max(diag.maxPosErr, r.posErr);
      diag.maxRotErr = Math.max(diag.maxRotErr, r.rotErr);
      const base = side === 'left' ? 0 : 6;
      for (let k = 0; k < 6; k++) rowJoints[base + k] = r.q[k];
    }

    // ---- waist and arms ---------------------------------------------------
    // Arms are not decoration here. In flight there is nothing to push
    // against, so the only way to change the body's attitude is to move mass
    // against it — which is exactly what the Apollo crews' windmilling was.
    const swingPhase = airborne ? Math.PI * u : 0;
    const sw = motion.armSwing * (airborne ? Math.sin(swingPhase) : 0.25);
    rowJoints[12] = 0;                                  // waist yaw
    rowJoints[13] = 0;                                  // waist roll
    rowJoints[14] = pitchBody * 0.35;                   // waist pitch
    const armUp = hi === tripHop ? 1.0 : 0;
    for (const [b, sgn] of [[15, 1], [22, -1]]) {
      rowJoints[b + 0] = -0.25 - sw * 0.9 - armUp * 0.9;   // shoulder pitch
      rowJoints[b + 1] = sgn * (0.18 + Math.abs(sw) * 0.35 + armUp * 0.45);
      rowJoints[b + 2] = 0;
      rowJoints[b + 3] = 0.35 + Math.abs(sw) * 0.5 + armUp * 0.6;  // elbow
    }
    // A lope is asymmetric: the arms counter-rotate against the leading leg.
    if (motion.feet === 'alternating') {
      const s = h.feet[0] === 'left' ? 1 : -1;
      rowJoints[15] += s * sw * 0.55;
      rowJoints[22] -= s * sw * 0.55;
    }

    const q = matToQuat(Rbody);
    rows.push([px, py, pz, q[0], q[1], q[2], q[3], ...rowJoints]);
    contacts.push([
      !airborne && (h.feet.includes('left') || motion.feet === 'together'),
      !airborne && (h.feet.includes('right') || motion.feet === 'together'),
    ]);
  }

  return { rows, contacts, phys, field, heading, motion, site, g };
}

// ---------------------------------------------------------------------------
export function buildMotionScene(siteId) {
  const site = SITES[siteId];
  const out = {
    id: siteId, name: site.name, body: site.body, g: site.g, fps: FPS,
    place: site.place, blurb: site.blurb,
    terrain: site.dem ? { dem: site.dem, profile: site.profile, origin: site.origin ?? [-4, 0], heading: 0 } : undefined,
    motions: {},
  };
  if (site.dem) {
    const f = loadField(site.dem, site.profile);
    out.terrain.meta = f.meta;
  }
  const rnd = (v) => Math.round(v * 1e5) / 1e5;

  for (const id of Object.keys(MOTIONS)) {
    const r = buildMotion(siteId, id);
    const clip = {
      label: MOTIONS[id].label,
      blurb: MOTIONS[id].blurb,
      generated: true,
      robot: 'g1', fps: FPS, frames: r.rows.length,
      joints: JOINT_NAMES,
      angles: r.rows.map((row) => row.slice(7, 36).map(rnd)),
      root: r.rows.map((row) => [rnd(row[0]), rnd(row[1]), rnd(row[2])]),
      quat: r.rows.map((row) => [rnd(row[3]), rnd(row[4]), rnd(row[5]), rnd(row[6])]),
      contacts: r.contacts,
      // Everything the readout quotes, computed from g and the URDF.
      physics: {
        g: r.g, v0: r.phys.v0, apex: r.phys.apex, hang: r.phys.hang,
        duty: r.phys.dutyFactor, speedCeiling: r.phys.speedCeiling,
        range: r.phys.range, boundBy: r.phys.boundBy,
        toppleTime: toppleTime(r.g, 0.6),
      },
      audit: { after: audit(r.rows, (x, y) => r.field.heightAt(x, -y), r.contacts) },
    };
    const ra = rateAudit(clip);
    let peak = 0, over = 0, frames = 0;
    for (const v of Object.values(ra)) { peak = Math.max(peak, v.peak); over += v.over; frames += v.frames; }
    clip.rates = {
      limitDegPerSec: JOINT_RATE_LIMIT, peakDegPerSec: Math.round(peak),
      overFrac: frames ? over / frames : 0,
      feasiblePlayback: peak > 0 ? Math.min(1, JOINT_RATE_LIMIT / peak) : 1,
    };
    out.motions[id] = clip;
  }
  fs.mkdirSync('public/motions_baked', { recursive: true });
  fs.writeFileSync(`public/motions_baked/${siteId}.json`, JSON.stringify(out));
  return out;
}

if (process.argv[1].endsWith('build_motions.mjs')) {
  const ids = process.argv.length > 2 ? process.argv.slice(2)
    : Object.keys(SITES).filter((k) => SITES[k].dem);
  console.log('site'.padEnd(30) + 'motion  apex    hang   duty   vmax   penet   slip%');
  for (const id of ids) {
    try {
      const out = buildMotionScene(id);
      for (const [m, c] of Object.entries(out.motions)) {
        const p = c.physics, a = c.audit.after;
        console.log(`${id.padEnd(30)}${m.padEnd(7)} `
          + `${p.apex.toFixed(2).padStart(5)}m ${p.hang.toFixed(2).padStart(6)}s `
          + `${p.duty.toFixed(2).padStart(6)} ${p.speedCeiling.toFixed(2).padStart(5)} `
          + `${(a.penetration * 1000).toFixed(1).padStart(7)}mm ${(a.schedSlipFrac * 100).toFixed(1).padStart(6)}`);
      }
    } catch (e) {
      console.log(`${id.padEnd(30)}FAILED: ${e.message}`);
    }
  }
}
