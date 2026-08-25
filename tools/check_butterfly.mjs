/**
 * check_butterfly.mjs — the static check that runs BEFORE anything moves.
 *
 * No browser, no renderer, no robot. It samples the commanded trajectory at
 * the rate a controller would receive it and asserts, against the G1's own
 * URDF numbers:
 *
 *   position      every sample inside the joint's hardware limits, and inside
 *                 SAFETY.posFraction of the range about its midpoint
 *   velocity      peak |dq/dt| under SAFETY.velFraction of the URDF limit
 *   acceleration  peak |d2q/dt2| under SAFETY.accelMax
 *   continuity    no step between consecutive samples big enough to read as a
 *                 jump, which is what a position controller turns into a
 *                 torque spike
 *   symmetry      both arms commanded identically where the mirror is about
 *                 the y axis, and exactly opposite where it is about x or z.
 *                 Butterfly is the stroke where this is the whole point.
 *   e-stop        the stop decays from wherever it was to neutral and holds,
 *                 rather than jumping
 *
 * Run:  node tools/check_butterfly.mjs [--amplitude 0.35] [--frequency 0.35]
 */
import { Butterfly, dryRun, validate, maxSafeFrequency, hardwareReadiness,
         G1_LIMITS, SAFETY, DEFAULTS, NEUTRAL } from '../src/sim/Butterfly.js';

const arg = (k, d) => {
  const i = process.argv.indexOf(`--${k}`);
  return i > 0 ? Number(process.argv[i + 1]) : d;
};
const amplitude = arg('amplitude', DEFAULTS.amplitude);
const frequency = arg('frequency', DEFAULTS.frequency);
const RATE = 200;                       // Hz, a plausible control rate
const fail = [];
const pad = (s, n) => String(s).padEnd(n);
const f = (v, n = 3) => v.toFixed(n).padStart(n + 4);

console.log(`\nbutterfly stroke — static check`);
console.log(`amplitude ${amplitude}   frequency ${frequency} Hz   sampled at ${RATE} Hz\n`);

// --- the envelope ----------------------------------------------------------
const ceiling = maxSafeFrequency(amplitude);
console.log(`envelope ceiling at this amplitude: ${ceiling.hz.toFixed(3)} Hz ` +
            `(bound by ${ceiling.boundBy})`);
if (frequency > ceiling.hz) fail.push(`frequency ${frequency} Hz is above the ${ceiling.hz.toFixed(3)} Hz ceiling`);

const run = dryRun({ amplitude, frequency }, { seconds: 60, rate: RATE });
const v = validate(run);
for (const x of v.violations) fail.push(`${x.joint}: ${x.kind} ${JSON.stringify(x.got)}`);

// --- per-joint table -------------------------------------------------------
console.log(`\n${pad('joint', 30)} ${pad('range cmd', 17)} ${pad('of limit', 9)} ` +
            `${pad('peak vel', 10)} ${pad('of cap', 8)} ${pad('peak acc', 10)} of cap`);
for (const j of run.joints) {
  const L = G1_LIMITS[j], p = run.peak[j];
  const mid = (L.lo + L.hi) / 2, half = (L.hi - L.lo) / 2;
  const posFrac = Math.max(Math.abs(p.qlo - mid), Math.abs(p.qhi - mid)) / half;
  const velCap = L.vel * SAFETY.velFraction;
  console.log(`${pad(j, 30)} ${f(p.qlo)} ${f(p.qhi)}  ${f(posFrac, 2)}    ` +
              `${f(p.vmax, 2)}    ${f(p.vmax / velCap, 2)}    ` +
              `${f(p.amax, 2)}   ${f(p.amax / SAFETY.accelMax, 2)}`);
}

// --- continuity ------------------------------------------------------------
// A position controller receiving a step commands whatever torque it takes to
// close it in one tick, so the size of the biggest step IS the safety property.
const STEP_LIMIT = 0.02;                // rad between 200 Hz samples ~= 1.15 deg
let worstStep = { j: null, d: 0 };
for (const j of run.joints) {
  const s = run.q[j];
  for (let i = 1; i < s.length; i++) {
    const d = Math.abs(s[i] - s[i - 1]);
    if (d > worstStep.d) worstStep = { j, d, i };
  }
}
console.log(`\nlargest step between samples: ${worstStep.d.toFixed(5)} rad ` +
            `(${(worstStep.d * 180 / Math.PI).toFixed(3)} deg) on ${worstStep.j}`);
if (worstStep.d > STEP_LIMIT) fail.push(`step of ${worstStep.d.toFixed(4)} rad on ${worstStep.j} exceeds ${STEP_LIMIT}`);

// --- symmetry --------------------------------------------------------------
// Both arms together is what makes it butterfly rather than front crawl.
// The mirror is geometric: y-axis joints (pitch, elbow) are NOT negated
// between left and right; x-axis (roll) and z-axis (yaw) are.
const SAME = [['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
              ['left_elbow_joint', 'right_elbow_joint'],
              ['left_knee_joint', 'right_knee_joint'],
              ['left_hip_pitch_joint', 'right_hip_pitch_joint'],
              ['left_ankle_pitch_joint', 'right_ankle_pitch_joint']];
const OPPOSITE = [['left_shoulder_roll_joint', 'right_shoulder_roll_joint'],
                  ['left_shoulder_yaw_joint', 'right_shoulder_yaw_joint']];
let symErr = 0;
for (const [a, b] of SAME) {
  for (let i = 0; i < run.q[a].length; i++) symErr = Math.max(symErr, Math.abs(run.q[a][i] - run.q[b][i]));
}
let oppErr = 0;
for (const [a, b] of OPPOSITE) {
  // Compare against the mirrored NEUTRAL, since amplitude interpolates from it.
  for (let i = 0; i < run.q[a].length; i++) oppErr = Math.max(oppErr, Math.abs(run.q[a][i] + run.q[b][i]));
}
console.log(`arm symmetry: same-axis pairs differ by at most ${symErr.toExponential(2)} rad, ` +
            `mirrored pairs by ${oppErr.toExponential(2)} rad`);
if (symErr > 1e-9) fail.push(`arms are not in phase: ${symErr} rad apart on a y-axis joint`);
if (oppErr > 1e-9) fail.push(`mirrored joints are not opposite: ${oppErr} rad`);

// --- the ramp --------------------------------------------------------------
const bfRamp = new Butterfly({ amplitude, frequency });
const a0 = bfRamp.amplitudeAt(0), aHalf = bfRamp.amplitudeAt(DEFAULTS.rampSeconds / 2);
console.log(`amplitude ramp: t=0 -> ${(a0 * 100).toFixed(1)}%, ` +
            `t=${DEFAULTS.rampSeconds / 2}s -> ${(aHalf * 100).toFixed(1)}%, ` +
            `t=${DEFAULTS.rampSeconds}s -> ${(bfRamp.amplitudeAt(DEFAULTS.rampSeconds) * 100).toFixed(1)}%`);
if (a0 > 1e-9) fail.push('the stroke does not start from zero amplitude');

// --- emergency stop --------------------------------------------------------
const bf = new Butterfly({ amplitude, frequency, rampSeconds: 0 });
bf.t = 3.7;
const held = bf.pose(3.7);
bf.estop('check');
let stepAtStop = 0, settled = true;
const first = bf.pose(3.7 + 1 / RATE);
for (const j of run.joints) stepAtStop = Math.max(stepAtStop, Math.abs(first[j] - held[j]));
const after = bf.pose(3.7 + DEFAULTS.estopSeconds + 0.5);
for (const j of run.joints) if (Math.abs(after[j] - NEUTRAL[j]) > 1e-6) settled = false;
console.log(`e-stop: first commanded step after the stop is ${stepAtStop.toFixed(5)} rad; ` +
            `settled on neutral after ${DEFAULTS.estopSeconds}s: ${settled}`);
if (stepAtStop > STEP_LIMIT) fail.push(`e-stop jumps ${stepAtStop.toFixed(4)} rad on the first tick`);
if (!settled) fail.push('e-stop does not settle on the neutral pose');

// --- hardware --------------------------------------------------------------
const hw = hardwareReadiness();
console.log(`\nhardware: ${hw.ready ? 'READY' : 'NOT READY'}`);
for (const m of hw.missing) console.log(`  missing: ${m}`);

// --- verdict ---------------------------------------------------------------
if (fail.length) {
  console.log(`\nFAIL (${fail.length})`);
  for (const m of fail) console.log(`  - ${m}`);
  process.exit(1);
}
console.log(`\nPASS — trajectory is inside the envelope. Simulation only; ` +
            `no hardware transport exists in this repo.`);
