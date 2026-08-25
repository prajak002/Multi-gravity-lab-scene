/**
 * fit_pose.mjs — drive the G1 from 2D keypoints observed in real footage.
 *
 * WHY THIS FITS BONE DIRECTIONS, NOT POSITIONS
 * A suited astronaut is about 1.9 m tall with a PLSS backpack; a G1 is 1.32 m
 * with proportionally shorter legs and a much shorter torso. Fitting the
 * robot's projected joint POSITIONS to a human's joint positions therefore
 * asks it to be a shape it is not, and the solver spends its freedom on
 * absorbing the proportion mismatch instead of on the pose.
 *
 * What transfers between bodies of different proportion is the DIRECTION each
 * limb segment points. So each bone contributes a unit-vector residual in the
 * image plane, which is invariant to the subject's size, to camera zoom, and
 * to where in frame they are — all three of which change constantly in
 * hand-held archival footage and none of which are recoverable here, because
 * the camera solve for this reel produced nothing but identity matrices.
 *
 * WHY A SINGLE CAMERA IS ENOUGH
 * Orthographic projection cannot tell a limb reaching toward the camera from
 * one reaching away: both project identically. The usual fix is a learned 3D
 * prior. It is not needed here, because the G1's knee joint is limited to
 * [-0.087, 2.88] rad — it physically cannot bend backwards. That one hardware
 * limit removes the reflection for the whole leg chain, and the arms are
 * resolved by temporal continuity from the previous frame.
 *
 * Run: node tools/fit_pose.mjs <keypoints2d.json> <out.json> [--limit N]
 */
import fs from 'fs';
import { bodyFK } from '../src/sim/G1Kinematics.js';
import { G1_TREE } from '../src/sim/G1Body.js';

// --- correspondences: a G1 link, and the COCO-17 keypoint it stands in for --
// Bones, not points. Each entry is [linkA, linkB, kpA, kpB].
const BONES = [
  ['left_hip_pitch_link',  'left_knee_link',        'hipL', 'kneeL'],
  ['left_knee_link',       'left_ankle_roll_link',  'kneeL', 'ankleL'],
  ['right_hip_pitch_link', 'right_knee_link',       'hipR', 'kneeR'],
  ['right_knee_link',      'right_ankle_roll_link', 'kneeR', 'ankleR'],
  ['left_shoulder_pitch_link',  'left_elbow_link',  'shoulderL', 'elbowL'],
  ['left_elbow_link',      'left_wrist_roll_link',  'elbowL', 'wristL'],
  ['right_shoulder_pitch_link', 'right_elbow_link', 'shoulderR', 'elbowR'],
  ['right_elbow_link',     'right_wrist_roll_link', 'elbowR', 'wristR'],
  // Trunk: hips to shoulders fixes the body's lean and facing.
  ['pelvis', 'torso_link', 'hipMid', 'shoulderMid'],
];

/** Free parameters: root yaw/pitch/roll, then the named joints. */
const FIT_JOINTS = [
  'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint',
  'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint',
  'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint',
  'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
  'waist_pitch_joint',
];
const NROOT = 3;
const NP = NROOT + FIT_JOINTS.length;

const LIM = Object.fromEntries(
  G1_TREE.filter((l) => l.joint).map((l) => [l.joint, l.limit || [-Math.PI, Math.PI]]));

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/** Rz(yaw) Ry(pitch) Rx(roll) applied to a 3-vector. */
function rot(p, yaw, pitch, roll) {
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll), sr = Math.sin(roll);
  let [x, y, z] = p;
  // roll about X
  let y1 = y * cr - z * sr, z1 = y * sr + z * cr;
  // pitch about Y
  let x2 = x * cp + z1 * sp, z2 = -x * sp + z1 * cp;
  // yaw about Z
  return [x2 * cy - y1 * sy, x2 * sy + y1 * cy, z2];
}

/**
 * Projected unit bone directions for one parameter vector.
 * Orthographic: image u is world X, image v is world -Z (image y grows down).
 */
function project(params, obsMask) {
  const q = {};
  FIT_JOINTS.forEach((n, i) => { q[n] = params[NROOT + i]; });
  const fk = bodyFK(q);
  const byName = Object.fromEntries(fk.map((l) => [l.name, l.p]));
  const [yaw, pitch, roll] = params;
  const out = [];
  for (let b = 0; b < BONES.length; b++) {
    if (!obsMask[b]) { out.push(null); continue; }
    const [la, lb] = BONES[b];
    const pa = rot(byName[la], yaw, pitch, roll);
    const pb = rot(byName[lb], yaw, pitch, roll);
    const du = pb[0] - pa[0], dv = -(pb[2] - pa[2]);
    const n = Math.hypot(du, dv) || 1e-9;
    out.push([du / n, dv / n]);
  }
  return out;
}

/** Observed unit bone directions from the keypoints of one frame. */
function observe(kp, ix, minConf) {
  const pt = (name) => {
    if (name === 'hipMid') {
      const a = kp[ix.hipL], b = kp[ix.hipR];
      return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.min(a[2], b[2])];
    }
    if (name === 'shoulderMid') {
      const a = kp[ix.shoulderL], b = kp[ix.shoulderR];
      return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.min(a[2], b[2])];
    }
    return kp[ix[name]];
  };
  const dirs = [], mask = [], conf = [];
  for (const [, , ka, kb] of BONES) {
    const a = pt(ka), b = pt(kb);
    const c = Math.min(a[2], b[2]);
    const du = b[0] - a[0], dv = b[1] - a[1];
    const n = Math.hypot(du, dv);
    // A bone shorter than a couple of pixels has no reliable direction; it is
    // either occluded or pointing straight at the camera. Drop it rather than
    // fit noise.
    const ok = c >= minConf && n > 2.5;
    dirs.push(ok ? [du / n, dv / n] : null);
    mask.push(ok);
    conf.push(ok ? c : 0);
  }
  return { dirs, mask, conf };
}

/** Levenberg-Marquardt on the bone-direction residual. */
function fitFrame(obs, seed, opt = {}) {
  const p = seed.slice();
  const lambda0 = opt.lambda ?? 1e-3;
  const REG = opt.reg ?? 0.06;          // pull toward the seed: temporal smoothness
  const EPS = 1e-4;
  const M = BONES.length * 2 + NP;      // residuals: bones + regulariser

  const residual = (pp) => {
    const pr = project(pp, obs.mask);
    const r = new Array(M).fill(0);
    for (let b = 0; b < BONES.length; b++) {
      if (!obs.mask[b]) continue;
      const w = Math.sqrt(obs.conf[b]);
      r[b * 2] = w * (pr[b][0] - obs.dirs[b][0]);
      r[b * 2 + 1] = w * (pr[b][1] - obs.dirs[b][1]);
    }
    for (let i = 0; i < NP; i++) r[BONES.length * 2 + i] = REG * (pp[i] - seed[i]);
    return r;
  };

  let r0 = residual(p);
  let cost = r0.reduce((s, v) => s + v * v, 0);
  let lambda = lambda0;

  for (let iter = 0; iter < (opt.iters ?? 40); iter++) {
    // numerical Jacobian, M x NP
    const J = [];
    for (let j = 0; j < NP; j++) {
      const pp = p.slice(); pp[j] += EPS;
      const rj = residual(pp);
      J.push(rj.map((v, k) => (v - r0[k]) / EPS));
    }
    // normal equations (JtJ + lambda I) dp = -Jt r
    const A = [], b = [];
    for (let i = 0; i < NP; i++) {
      let s = 0;
      for (let k = 0; k < M; k++) s += J[i][k] * r0[k];
      b.push(-s);
      const row = new Array(NP).fill(0);
      for (let j = 0; j < NP; j++) {
        let t = 0;
        for (let k = 0; k < M; k++) t += J[i][k] * J[j][k];
        row[j] = t;
      }
      A.push(row);
    }
    for (let i = 0; i < NP; i++) A[i][i] += lambda * (1 + A[i][i]);
    const dp = solve(A, b);
    if (!dp) break;

    const cand = p.slice();
    for (let i = 0; i < NP; i++) cand[i] += dp[i];
    // joint limits are hard; the knee limit is what resolves depth
    for (let i = 0; i < FIT_JOINTS.length; i++) {
      const [lo, hi] = LIM[FIT_JOINTS[i]];
      cand[NROOT + i] = clamp(cand[NROOT + i], lo, hi);
    }
    const rc = residual(cand);
    const cc = rc.reduce((s, v) => s + v * v, 0);
    if (cc < cost) {
      p.splice(0, NP, ...cand); r0 = rc; cost = cc; lambda = Math.max(1e-9, lambda * 0.5);
      if (Math.abs(cc) < 1e-10) break;
    } else {
      lambda *= 4;
      if (lambda > 1e7) break;
    }
  }
  return { p, cost, bones: obs.mask.filter(Boolean).length };
}

/** Gaussian elimination with partial pivoting. */
function solve(A, b) {
  const n = A.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-14) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

export function fitSequence(data, opt = {}) {
  const ix = Object.fromEntries(data.names.map((n, i) => [n, i]));
  const minConf = opt.minConf ?? 0.4;
  const frames = opt.limit ? Math.min(opt.limit, data.frames) : data.frames;

  let seed = new Array(NP).fill(0);
  seed[1] = 0.0;
  // Seed the legs in a slight crouch so the knee starts on the correct side of
  // its limit; from a straight leg the first step has no gradient to follow.
  seed[NROOT + FIT_JOINTS.indexOf('left_knee_joint')] = 0.35;
  seed[NROOT + FIT_JOINTS.indexOf('right_knee_joint')] = 0.35;

  const out = [], diag = [];
  for (let f = 0; f < frames; f++) {
    const obs = observe(data.kp[f], ix, minConf);
    const r = fitFrame(obs, seed, opt);
    seed = r.p.slice();
    out.push(r.p.slice());
    diag.push({ cost: r.cost, bones: r.bones });
  }
  return { params: out, diag, joints: FIT_JOINTS, bones: BONES };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const [src, dst] = process.argv.slice(2);
  const limArg = process.argv.indexOf('--limit');
  const data = JSON.parse(fs.readFileSync(src, 'utf8'));
  const t0 = Date.now();
  const r = fitSequence(data, { limit: limArg > 0 ? +process.argv[limArg + 1] : 0 });
  const costs = r.diag.map((d) => d.cost);
  const bones = r.diag.map((d) => d.bones);
  const mean = (a) => a.reduce((s, v) => s + v, 0) / a.length;
  console.log(`fitted ${r.params.length} frames in ${((Date.now() - t0) / 1000).toFixed(1)} s`);
  console.log(`  bones used per frame: mean ${mean(bones).toFixed(1)} of ${BONES.length}`);
  console.log(`  residual cost: mean ${mean(costs).toFixed(4)}  min ${Math.min(...costs).toFixed(4)}  max ${Math.max(...costs).toFixed(4)}`);
  if (dst) {
    fs.writeFileSync(dst, JSON.stringify({
      source: data.source, frames: r.params.length, joints: r.joints,
      root: r.params.map((p) => p.slice(0, NROOT)),
      angles: r.params.map((p) => p.slice(NROOT).map((v) => Math.round(v * 1e4) / 1e4)),
      diag: r.diag,
    }));
    console.log(`-> ${dst}`);
  }
}
