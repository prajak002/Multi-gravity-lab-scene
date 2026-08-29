/**
 * arms.js — boot the arm studio.
 *
 * The robot loads immediately; the camera waits for a click, because asking
 * for a webcam before the page has shown what it is for is how a permission
 * prompt gets denied.
 */
import { ArmStudio } from './ui/ArmStudio.js';
import { ARM_CHAIN, ARM_LIMITS } from './sim/G1Kinematics.js';
import { instability } from './sim/Ballistic.js';

const DRIVEN = [3, 4, 5, 6];
const SHORT = (j) => j.replace(/^(left|right)_/, '').replace(/_joint$/, '').replace(/_/g, ' ');
const BODIES = [['Moon', 1.625], ['Mars', 3.721], ['Earth', 9.807]];

const boot = document.getElementById('boot');
const studio = new ArmStudio(
  document.getElementById('stage'),
  document.getElementById('video'),
  document.getElementById('overlay'),
  document.getElementById('panel'));

try {
  await studio.loadRobot();
  boot.remove();
} catch (e) {
  boot.innerHTML = `<span>could not load the robot: ${e.message}</span>`;
  throw e;
}

const startBtn = document.getElementById('start');
startBtn.addEventListener('click', async () => {
  startBtn.disabled = true;
  startBtn.textContent = 'STARTING…';
  try {
    await studio.startTracking();
    startBtn.textContent = 'TRACKING';
    startBtn.classList.add('on');
  } catch (e) {
    // A denied permission is a normal outcome, not a crash: say what happened
    // and leave the robot on screen to be orbited.
    startBtn.disabled = false;
    startBtn.textContent = 'ENABLE CAMERA';
    document.getElementById('stat').textContent = `camera unavailable — ${e.message}`;
  }
});

const mirrorBtn = document.getElementById('mirror');
mirrorBtn.addEventListener('click', () => {
  studio.mirror = !studio.mirror;
  mirrorBtn.textContent = studio.mirror ? 'MIRRORED' : 'DIRECT';
  mirrorBtn.classList.toggle('on', studio.mirror);
  document.getElementById('cam').classList.toggle('mirror', studio.mirror);
});
document.getElementById('cam').classList.add('mirror');

/**
 * One row per driven joint, showing where it is inside its OWN range.
 *
 * Per joint and per side, because the ranges are not the same on both: the
 * shoulder roll runs [-1.59, 2.25] on the left and [-2.25, 1.59] on the right.
 * A single shared scale would put the two arms' bars in different places for
 * the same physical pose.
 */
const jointsEl = document.getElementById('joints');
const authEl = document.getElementById('authority');
const statEl = document.getElementById('stat');

function renderPanel() {
  const rows = [];
  for (const side of ['left', 'right']) {
    const q = studio.q[side];
    rows.push(`<div class="side"><h2>${side.toUpperCase()} ARM</h2>`);
    for (const k of DRIVEN) {
      const name = SHORT(ARM_CHAIN[side][k].joint);
      const [lo, hi] = ARM_LIMITS[side][k];
      const v = q ? q[k] : 0;
      const f = (v - lo) / (hi - lo);
      const hot = studio.clamped[side]?.includes(ARM_CHAIN[side][k].joint);
      rows.push(`<div class="j${hot ? ' hot' : ''}">
        <span>${name}</span>
        <div class="track"><i style="left:${(f * 100).toFixed(1)}%"></i></div>
        <b>${(v * 180 / Math.PI).toFixed(0)}°</b></div>`);
    }
    rows.push('</div>');
  }
  jointsEl.innerHTML = rows.join('');

  const a = studio.armAuthority();
  if (a) {
    // What this sweep is worth as attitude control, per body. The arms are the
    // only thing that can turn a body in free flight, so this is the number
    // that decides whether a jump lands on its feet.
    const cells = BODIES.map(([n, g]) => {
      const s = instability(g, { thrust: 1 });
      const need = s.tumble * 180 / Math.PI;
      const got = a.perStroke * Math.max(1, 1.1 * s.hang) * 180 / Math.PI;
      const ok = got >= need;
      return `<div class="row ${ok ? 'ok' : 'bad'}"><span>${n}</span>
        <b>${got.toFixed(0)}°</b><u>needs ${need.toFixed(0)}°</u></div>`;
    }).join('');
    authEl.innerHTML = `<h2>THIS SWEEP AS ATTITUDE CONTROL</h2>
      <p class="note">In free flight nothing external can turn a body, so the only
      way to rotate the torso is to rotate something else the other way. Both arms
      are <b>13.8 %</b> of the G1's pitch inertia, so a sweep buys that fraction of
      itself back — extended on the stroke, tucked on the return, or it cancels.</p>
      ${cells}
      <p class="note">Against what a 4 mm thrust misalignment builds up over a jump
      on each body. The Moon gives the arms the most time to work and the most to
      undo, and the second wins.</p>`;
  }

  const t = studio.track;
  statEl.textContent = studio.tracking
    ? (t.seen
      ? `tracking · confidence ${(t.confidence * 100).toFixed(0)}% · `
        + `fit ${(studio.err.left).toFixed(3)}/${(studio.err.right).toFixed(3)} · `
        + `rate-limited ${(studio.smooth.left.limitedFraction * 100).toFixed(0)}% of frames`
      : 'no pose in frame — step back so your hips and shoulders are both visible')
    : 'camera off';
}

let last = performance.now(), acc = 0;
const loop = (now) => {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  studio.step(dt, now);
  studio.render();
  studio.drawOverlay();
  acc += dt;
  if (acc > 0.12) { acc = 0; renderPanel(); }
  requestAnimationFrame(loop);
};
requestAnimationFrame(loop);

// Exposed for the headless check, same as the arena.
window.__arms = studio;
