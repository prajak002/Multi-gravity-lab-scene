/**
 * build_scene.mjs — take one motion packet, plant it on a real NASA DEM, and
 * emit a scene the viewer can play.
 *
 * Run: node tools/build_scene.mjs <sceneId>
 */
import fs from 'fs';
import path from 'path';
import { SiteField, SURFACE_PROFILES } from '../src/terrain/SiteField.js';
import { retarget } from './retarget.mjs';
import { audit } from './audit_contact.mjs';
import { SCENES } from './scenes.mjs';

/**
 * Joint names exactly as the arena's baked motions use them, so a retargeted
 * clip drops into src/data/Motion.js with no translation layer. The ordering is
 * also the packets' own CSV column order — the two already agree, which is why
 * the angles can be copied straight across.
 */
export const JOINT_NAMES = [
  'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
  'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
  'waist_yaw', 'waist_roll', 'waist_pitch',
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
  'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
  'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
].map((n) => `${n}_joint_dof`);
import { packetMetrics, styleFromMetrics } from './packet_metrics.mjs';
import { rateAudit, JOINT_RATE_LIMIT } from './audit_rates.mjs';
import { retargetMicro, comAudit, detectPhases } from './retarget_micro.mjs';

const readCSV = (f) => fs.readFileSync(f, 'utf8').trim().split('\n')
  .map((l) => l.split(',').map(Number)).filter((r) => r.length >= 36);

function loadField(demName, profile) {
  const meta = JSON.parse(fs.readFileSync(`public/dem/${demName}.json`, 'utf8'));
  const buf = fs.readFileSync(`public/dem/${demName}.f32`);
  const dem = new Float32Array(buf.buffer, buf.byteOffset, buf.length / 4);
  return new SiteField(dem, meta, SURFACE_PROFILES[profile]);
}

/** Direction of steepest ascent, averaged over the traverse length. */
export function upslopeHeading(field, x0, y0, span = 12) {
  let gx = 0, gy = 0;
  for (let k = -2; k <= 2; k++) {
    const off = k * span * 0.25;
    const hE = field.heightAt(x0 + span / 2 + off, -y0), hW = field.heightAt(x0 - span / 2 + off, -y0);
    const hN = field.heightAt(x0, -(y0 + span / 2)), hS = field.heightAt(x0, -(y0 - span / 2));
    gx += (hE - hW) / span; gy += (hN - hS) / span;
  }
  return Math.atan2(gy, gx);
}

export function buildScene(id) {
  const sc = SCENES[id];
  if (!sc) throw new Error(`unknown scene ${id}`);
  if (sc.micro) return buildMicroScene(id, sc);
  const field = loadField(sc.dem, sc.profile);
  const pkgDir = sc.packet;
  const csvDir = path.join(pkgDir, 'CSV');
  const files = fs.readdirSync(csvDir).filter((f) => f.endsWith('.csv'));
  const pick = (p) => path.join(csvDir, files.find((f) => p.test(f)));

  const heading = sc.heading === 'upslope'
    ? upslopeHeading(field, sc.origin[0], sc.origin[1])
    : sc.heading === 'downslope'
      ? upslopeHeading(field, sc.origin[0], sc.origin[1]) + Math.PI
      : sc.heading;

  const out = { id, name: sc.name, body: sc.body, g: sc.g, fps: 30,
                terrain: { dem: sc.dem, profile: sc.profile, meta: field.meta,
                           origin: sc.origin, heading },
                blurb: sc.blurb, clips: {} };

  const report = [];
  for (const [key, re] of [['A', /^A_/], ['B', /^B_/]]) {
    const file = pick(re);
    const rows = readCSV(file);
    // 'before' is judged on flat ground, which is what the packet assumed.
    const before = audit(rows);
    const met = packetMetrics(pkgDir, key);
    // Scene style sets the posture; the packet's OWN measured numbers set the
    // failures. Metrics win, so a clip never contains struggle its authors did
    // not record.
    const style = styleFromMetrics(met, key === 'A' ? sc.styleA : sc.styleB);
    const r = retarget(rows, field, {
      origin: sc.origin, heading, g: sc.g,
      seed: sc.seed + (key === 'A' ? 0 : 977),
      recoveryEvents: 0, missedContacts: 0, stepCV: 0,
      ...style,
    });
    const contactFlags = r.rows.map((_, i) => [!!r.sched.left.down[i], !!r.sched.right.down[i]]);
    const after = audit(r.rows, (x, y) => field.heightAt(x, -y), contactFlags);
    // Emit in the arena's baked-motion shape: joints / angles / root / quat /
    // contacts. Rounded to 0.1 mm and 1e-5 rad, which is far finer than the
    // solver's own residual and roughly halves the file.
    const rnd = (v) => Math.round(v * 1e5) / 1e5;
    out.clips[key] = {
      label: key === 'A' ? 'WorldVLA' : 'PragyaSpace',
      source: path.basename(file),
      metrics: { ...met, raw: undefined },
      style,
      robot: 'g1', fps: 30, frames: r.rows.length,
      joints: JOINT_NAMES,
      angles: r.rows.map((row) => row.slice(7, 36).map(rnd)),
      root: r.rows.map((row) => [rnd(row[0]), rnd(row[1]), rnd(row[2])]),
      quat: r.rows.map((row) => [rnd(row[3]), rnd(row[4]), rnd(row[5]), rnd(row[6])]),
      // [left, right] loaded flags, straight from the gait schedule the feet
      // were actually solved against — not re-derived from geometry.
      contacts: r.rows.map((_, i) => [!!r.sched.left.down[i], !!r.sched.right.down[i]]),
      footholds: { left: r.footholds.left.map((f) => [f.x, f.y, f.z, !!f.slip, f.missed]),
                   right: r.footholds.right.map((f) => [f.x, f.y, f.z, !!f.slip, f.missed]) },
      audit: { before, after },
      ik: r.diag,
    };
    // Joint rates against what the hardware can actually turn.
    //
    // A clip can be correct at every frame and still be untrackable, because
    // per-frame correctness says nothing about the rate BETWEEN frames. This
    // is shipped rather than silently fixed because the cause is the SOURCE
    // packet, not the retarget: these traverses cover 4.5-5.7 m in 10 s, and a
    // 0.79 m leg cannot swing that fast. Slowing the clip to suit would
    // falsify the forward-distance figure the packets state.
    //
    // `feasiblePlayback` is the rate at which the whole clip comes inside the
    // G1's rating, so a viewer can offer an honest speed rather than a guess.
    {
      const ra = rateAudit(out.clips[key]);
      let peak = 0, over = 0, frames = 0;
      for (const v of Object.values(ra)) {
        peak = Math.max(peak, v.peak); over += v.over; frames += v.frames;
      }
      out.clips[key].rates = {
        limitDegPerSec: JOINT_RATE_LIMIT,
        peakDegPerSec: Math.round(peak),
        overFrac: frames ? over / frames : 0,
        feasiblePlayback: peak > 0 ? Math.min(1, JOINT_RATE_LIMIT / peak) : 1,
      };
    }
    report.push({ key, before, after, diag: r.diag, steps: r.footholds.left.length + r.footholds.right.length,
                  authoredClimb: r.authoredClimb, realClimb: r.realClimb });
  }
  fs.mkdirSync('public/scenes', { recursive: true });
  fs.writeFileSync(`public/scenes/${id}.json`, JSON.stringify(out));
  return { out, report };
}

// Only when run as the command, never when imported.
//
// build_motions.mjs imports JOINT_NAMES from here, and without this guard that
// import ALSO ran this block against build_motions' own argv — so asking for
// twenty generated motions tried to build a packet scene named after all
// twenty at once.
if (process.argv[1]?.endsWith('build_scene.mjs') && process.argv[2]) {
  const { out, report } = buildScene(process.argv[2]);
  if (out.micro) {
    console.log(`\n${out.name}   (${out.body}, microgravity)`);
    for (const r of report) {
      const b = r.before, a = r.after;
      console.log(`\n${r.key} — ${out.clips[r.key].label}   stroke: ${r.stroke}`);
      console.log(`   free flight  f${r.ph.releaseFrame}..${r.ph.captureFrame ?? 'end'} `
                + `at ${r.ph.plateauSpeed.toFixed(4)} m/s`);
      console.log(`   BEFORE  COM off its line by ${(b.maxDev * 1000).toFixed(1)} mm max, `
                + `${(b.meanDev * 1000).toFixed(1)} mm mean -> ${b.ghostForce.toFixed(1)} N from nowhere`);
      console.log(`   AFTER   COM off its line by ${(a.maxDev * 1000).toFixed(3)} mm max, `
                + `${(a.meanDev * 1000).toFixed(3)} mm mean -> ${a.ghostForce.toFixed(3)} N from nowhere`);
    }
    process.exit(0);
  }
  console.log(`\n${out.name}   (${out.body}, g=${out.g} m/s^2)`);
  console.log(`terrain  ${out.terrain.meta.citation}`);
  const _m = out.terrain.meta;
  const _span = _m.span_x_m ? `${Math.round(_m.span_x_m)} x ${Math.round(_m.span_y_m)} m` : `${_m.span_m} m`;
  console.log(`         ${_span} patch at ${out.terrain.meta.mpp} m/px, `
            + `lat ${out.terrain.meta.lat} lon ${out.terrain.meta.lon}`);
  console.log(`heading  ${(out.terrain.heading * 180 / Math.PI).toFixed(1)} deg\n`);
  for (const r of report) {
    const b = r.before, a = r.after;
    console.log(`${r.key} — ${out.clips[r.key].label}   ${r.steps} footfalls, `
              + `climb authored ${r.authoredClimb.toFixed(3)} m -> real DEM ${r.realClimb.toFixed(3)} m`);
    console.log(`   BEFORE  swingNet ${b.swingNet.toFixed(3)}  stanceSlip ${b.stanceSlip.toFixed(2)} m `
              + `(${(b.slipFrac * 100).toFixed(0)}% of travel)  ${b.verdict}`);
    console.log(`   AFTER   swingNet ${a.swingNet.toFixed(3)}  stanceSlip ${a.stanceSlip.toFixed(2)} m `
              + `(${(a.slipFrac * 100).toFixed(0)}% of travel)  ${a.verdict}`);
    if (a.meanCornerGap !== undefined) {
      console.log(`   SOLE    deepest penetration ${(a.penetration * 1000).toFixed(1)} mm, `
                + `worst corner gap ${(a.cornerGap * 1000).toFixed(1)} mm, `
                + `mean ${(a.meanCornerGap * 1000).toFixed(1)} mm`);
    }
    if (a.schedSlip !== undefined) {
      console.log(`           while the gait says LOADED: ${a.schedSlip.toFixed(3)} m `
                + `(${(a.schedSlipFrac * 100).toFixed(1)}% of travel) over ${a.schedFrames} frames`);
    }
    console.log(`   IK      LOADED: max pos err ${(r.diag.maxPosErrContact * 1000).toFixed(2)} mm, `
              + `max sole tilt err ${(r.diag.maxRotErrContact * 1000).toFixed(0)} mrad `
              + `over ${r.diag.contactFrames} foot-frames`);
    console.log(`           all frames: max pos err ${(r.diag.maxPosErr * 1000).toFixed(3)} mm, `
              + `max rot err ${(r.diag.maxRotErr * 1000).toFixed(0)} mrad, `
              + `${r.diag.iters.toFixed(1)} iters/solve, ${r.diag.clamped} unreachable, `
              + `${r.diag.reachClamped} reach-clamped, ${r.diag.reseeded} re-seeded\n`);
  }
}


/**
 * Microgravity scenes.
 *
 * There is no terrain and there are no footfalls, so none of the contact
 * machinery applies. What replaces it is conservation: through free flight the
 * centre of mass must hold the velocity the wall gave it and the body must hold
 * its angular momentum, and the pelvis is derived from those rather than
 * scripted. See tools/retarget_micro.mjs.
 */
function buildMicroScene(id, sc) {
  const csvDir = path.join(sc.packet, 'CSV');
  const files = fs.readdirSync(csvDir).filter((f) => f.endsWith('.csv'));
  const pick = (p) => path.join(csvDir, files.find((f) => p.test(f)));
  const rnd = (v) => Math.round(v * 1e5) / 1e5;

  const out = { id, name: sc.name, body: sc.body, g: sc.g, fps: 30,
                micro: true, blurb: sc.blurb, clips: {} };
  const report = [];

  for (const [key, re] of [['A', /^A_/], ['B', /^B_/]]) {
    const file = pick(re);
    const rows = readCSV(file);
    const ph = detectPhases(rows, sc.speedOverride ?? null);
    const cfg = key === 'A' ? sc.clipA : sc.clipB;
    const before = comAudit(rows, ph.releaseFrame, ph.captureFrame ?? rows.length);
    const r = retargetMicro(rows, {
      releaseFrame: ph.releaseFrame,
      captureFrame: ph.captureFrame,
      releaseSpeed: ph.plateauSpeed,
      ...cfg,
    });
    const after = comAudit(r.rows, ph.releaseFrame, ph.captureFrame ?? rows.length);
    const met = packetMetrics(sc.packet, key);

    out.clips[key] = {
      label: key === 'A' ? 'WorldVLA' : 'PragyaSpace',
      source: path.basename(file),
      metrics: { ...met, raw: undefined },
      phases: ph, stroke: cfg.stroke,
      robot: 'g1', fps: 30, frames: r.rows.length,
      joints: JOINT_NAMES,
      angles: r.rows.map((row) => row.slice(7, 36).map(rnd)),
      root: r.rows.map((row) => [rnd(row[0]), rnd(row[1]), rnd(row[2])]),
      quat: r.rows.map((row) => [rnd(row[3]), rnd(row[4]), rnd(row[5]), rnd(row[6])]),
      // In free fall nothing is ever loaded; contact exists only at the wall
      // and the rail, which the phase markers already carry.
      contacts: r.rows.map((_, i) =>
        [i < ph.releaseFrame, i < ph.releaseFrame]),
      comAudit: { before, after },
    };
    // Same rate check as the surface scenes. It matters here too: in free fall
    // a limb sweep is unopposed, so nothing but the motor limits how fast the
    // clip may ask it to move.
    {
      const ra = rateAudit(out.clips[key]);
      let peak = 0, over = 0, frames = 0;
      for (const v of Object.values(ra)) {
        peak = Math.max(peak, v.peak); over += v.over; frames += v.frames;
      }
      out.clips[key].rates = {
        limitDegPerSec: JOINT_RATE_LIMIT,
        peakDegPerSec: Math.round(peak),
        overFrac: frames ? over / frames : 0,
        feasiblePlayback: peak > 0 ? Math.min(1, JOINT_RATE_LIMIT / peak) : 1,
      };
    }
    report.push({ key, ph, before, after, stroke: cfg.stroke });
  }

  fs.mkdirSync('public/scenes', { recursive: true });
  fs.writeFileSync(`public/scenes/${id}.json`, JSON.stringify(out));
  return { out, report };
}
