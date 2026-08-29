/**
 * check_arms.mjs — does the arm studio actually put a human pose on the robot,
 * and does it stay inside the URDF while doing it?
 *
 * The tracking half cannot be asserted headlessly: Chromium's fake camera is a
 * test pattern with no person in it, and shipping a video of someone to the
 * repo to test against is not worth it. What CAN be checked is everything
 * downstream of the landmarks, which is where all the logic lives — so the
 * page's own `applyLandmarks()` is fed known poses and the result is measured
 * on the robot the page is actually rendering.
 *
 * Three things it asserts, each of which was wrong at some point while this
 * was being written:
 *
 *   THE ARM POINTS WHERE THE HUMAN'S DOES. Within a few degrees, in the torso
 *   frame, for poses inside the machine's envelope.
 *
 *   NO JOINT LEAVES ITS URDF RANGE. Including the shoulder roll, whose range
 *   is asymmetric between the two arms.
 *
 *   THE POSE IS NOT GROTESQUE. Shoulder yaw is redundant for pointing an upper
 *   arm, so a solver with only direction targets will happily wind it to its
 *   stop to satisfy the forearm — it produced -150 degrees before the posture
 *   regularisation went in. A solution can be numerically right and unusable.
 *
 *   npx vite build && npx vite preview --port 4199 &
 *   node tools/check_arms.mjs http://localhost:4199
 */
import { chromium } from 'playwright';

const base = process.argv[2] || 'http://localhost:4199';
const MAX_UPPER_ERR_DEG = 12;
const MAX_YAW_DEG = 110;

const browser = await chromium.launch({
  args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'],
});
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
const errors = [];
page.on('pageerror', (e) => errors.push(e.message.slice(0, 200)));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text().slice(0, 200)); });

await page.goto(`${base}/arms.html`, { waitUntil: 'load' });
await page.waitForFunction(() => window.__arms?.robot, null, { timeout: 90000 });

const result = await page.evaluate(async ({ MAX_UPPER_ERR_DEG, MAX_YAW_DEG }) => {
  const s = window.__arms;
  s.mirror = false;                       // test the direct mapping
  const mk = (x, y, z) => ({ x, y, z, visibility: 1 });

  // A body, and one arm placed by hand.
  //
  // MediaPipe world landmarks are metres about the hip centre with y DOWN,
  // which is why every height here is negative. FORWARD is -z, and that is not
  // a MediaPipe convention — it falls out of the torso frame this code builds
  // for itself: up runs hips to shoulders, left runs right shoulder to left,
  // and forward completes the right-handed set as left x up. Getting that
  // backwards in the poses below is what made two of them look like solver
  // failures when they were in fact requests to put a forearm behind the body,
  // which really does need the shoulder wound round.
  const body = () => {
    const L = [];
    for (let i = 0; i < 33; i++) L[i] = mk(0, 0, 0);
    L[11] = mk(0.20, -0.55, 0); L[12] = mk(-0.20, -0.55, 0);
    L[23] = mk(0.12, 0.00, 0); L[24] = mk(-0.12, 0.00, 0);
    // Park the right arm somewhere harmless.
    L[14] = mk(-0.24, -0.28, 0); L[16] = mk(-0.26, -0.30, -0.10);
    return L;
  };

  const POSES = [
    ['hanging, forearm forward', [0.24, -0.28, 0.00], [0.26, -0.30, -0.30]],
    ['out to the side', [0.55, -0.57, 0.00], [0.85, -0.59, 0.00]],
    ['out and up 45', [0.50, -0.85, 0.00], [0.70, -1.15, 0.00]],
    ['forward reach', [0.26, -0.55, -0.34], [0.28, -0.56, -0.66]],
    ['hand on chest', [0.34, -0.40, -0.10], [0.06, -0.50, -0.16]],
    ['overhead', [0.30, -0.85, 0.00], [0.32, -1.15, 0.00]],
  ];

  const out = [];
  for (const [name, elbow, wrist] of POSES) {
    const L = body();
    L[13] = mk(...elbow); L[15] = mk(...wrist);
    s.q.left = null;                       // solve each pose cold
    s.applyLandmarks(L);
    for (let i = 0; i < 40; i++) s.step(1 / 30, performance.now());

    const t = s._targets?.left;
    const q = s.q.left;
    // The robot's own upper-arm direction, out of its FK.
    const d = s.armDirsFor ? s.armDirsFor('left', q) : null;
    out.push({
      name,
      upperErrDeg: d && t
        ? Math.acos(Math.max(-1, Math.min(1,
            d.upper[0] * t.upper[0] + d.upper[1] * t.upper[1] + d.upper[2] * t.upper[2]))) * 180 / Math.PI
        : null,
      yawDeg: Math.abs(q[5]) * 180 / Math.PI,
      clamped: s.clamped.left.slice(),
      joints: [3, 4, 5, 6].map((k) => Math.round(q[k] * 180 / Math.PI)),
    });
  }

  // Every driven joint, on both arms, inside its own URDF range.
  const outside = [];
  for (const side of ['left', 'right']) {
    const q = s.q[side];
    if (!q) continue;
    for (const k of [3, 4, 5, 6]) {
      const [lo, hi] = s.limitsFor(side)[k];
      if (q[k] < lo - 1e-6 || q[k] > hi + 1e-6) {
        outside.push(`${side}[${k}] ${q[k].toFixed(3)} outside [${lo}, ${hi}]`);
      }
    }
  }
  return { out, outside, MAX_UPPER_ERR_DEG, MAX_YAW_DEG };
}, { MAX_UPPER_ERR_DEG, MAX_YAW_DEG });

const failures = [];
console.log('pose'.padEnd(28) + 'upper err   shoulder yaw   joints (p,r,y,elbow)');
for (const r of result.out) {
  const err = r.upperErrDeg;
  const bad = err !== null && err > MAX_UPPER_ERR_DEG && !r.clamped.length;
  const wild = r.yawDeg > MAX_YAW_DEG;
  if (bad) failures.push(`${r.name}: upper arm ${err.toFixed(0)}° off with nothing clamped`);
  if (wild) failures.push(`${r.name}: shoulder yaw wound to ${r.yawDeg.toFixed(0)}°`);
  console.log(`${(bad || wild ? 'FAIL ' : 'ok   ') + r.name}`.padEnd(28)
    + `${err === null ? '   —' : err.toFixed(0).padStart(6) + '°'}   `
    + `${r.yawDeg.toFixed(0).padStart(9)}°   ${r.joints.join(',').padEnd(20)}`
    + (r.clamped.length ? ' CLAMP ' + r.clamped.map((j) => j.replace(/^(left|right)_/, '').replace('_joint', '')).join(',') : ''));
}
if (result.outside.length) failures.push(...result.outside.map((s) => `outside URDF range: ${s}`));

console.log(result.outside.length
  ? `\n${result.outside.length} joint(s) outside their URDF range`
  : '\nevery driven joint inside its URDF range');
if (errors.length) console.log('console errors:\n' + errors.slice(0, 6).join('\n'));
if (failures.length) console.log('\n' + failures.join('\n'));
else console.log('OK — the arms follow the pose, stay inside the URDF, and do not wind up.');
await browser.close();
process.exit(failures.length || errors.length ? 1 : 0);
