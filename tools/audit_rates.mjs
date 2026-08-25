/**
 * audit_rates.mjs — joint angular rates against what the hardware can do.
 *
 * A clip can be perfectly correct at every frame and still be impossible,
 * because correctness per frame says nothing about the rate BETWEEN frames.
 * The G1's leg joints are rated around 500 deg/s; anything materially past
 * that is a pose sequence no motor could track, and on screen it reads as a
 * snap rather than as motion.
 *
 * Reported per joint: peak rate, and how much of the clip sits over the limit.
 */
import fs from 'fs';

export const JOINT_RATE_LIMIT = 500;   // G1 leg joint, approximate rated speed
const LIMIT_DEG_S = JOINT_RATE_LIMIT;
const FPS = 30;

const LEG = ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll'];

export function rateAudit(clip) {
  const out = {};
  for (const side of ['left', 'right']) {
    for (const j of LEG) {
      const idx = clip.joints.indexOf(`${side}_${j}_joint_dof`);
      if (idx < 0) continue;
      const a = clip.angles.map((r) => r[idx]);
      let peak = 0, over = 0;
      for (let i = 1; i < a.length; i++) {
        const r = Math.abs(a[i] - a[i - 1]) * FPS * 180 / Math.PI;
        if (r > peak) peak = r;
        if (r > LIMIT_DEG_S) over++;
      }
      const key = `${side}_${j}`;
      out[key] = { peak, over, frames: a.length - 1, frac: over / (a.length - 1) };
    }
  }
  return out;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const rows = [];
  for (const f of fs.readdirSync('public/scenes')) {
    const s = JSON.parse(fs.readFileSync(`public/scenes/${f}`, 'utf8'));
    if (s.micro) continue;
    for (const k of ['A', 'B']) {
      const r = rateAudit(s.clips[k]);
      for (const [j, v] of Object.entries(r)) {
        if (v.peak > LIMIT_DEG_S) rows.push({ id: s.id, k, label: s.clips[k].label, j, ...v });
      }
    }
  }
  rows.sort((a, b) => b.peak - a.peak);
  if (!rows.length) { console.log(`no joint exceeds ${LIMIT_DEG_S} deg/s on any clip`); }
  else {
    console.log(`joints over ${LIMIT_DEG_S} deg/s, worst first:\n`);
    console.log('scene'.padEnd(26) + ' ' + 'model'.padEnd(12) + ' joint'.padEnd(18)
      + '  peak deg/s   frames over');
    for (const r of rows.slice(0, 30)) {
      console.log(`${r.id.padEnd(26)} ${r.label.padEnd(12)} ${r.j.padEnd(18)} `
        + `${r.peak.toFixed(0).padStart(8)}   ${String(r.over).padStart(6)} `
        + `(${(r.frac * 100).toFixed(1)}%)`);
    }
    console.log(`\n${rows.length} joint/clip combinations over limit`);
  }
}
