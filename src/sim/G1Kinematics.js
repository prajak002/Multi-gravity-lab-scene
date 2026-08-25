/**
 * G1Kinematics — exact forward kinematics and a 6-DOF leg solver for the
 * Unitree G1, built from the shipped g1_23dof.urdf rather than from a
 * simplified two-link stand-in.
 *
 * Frame here is the URDF's own: Z up, X forward, Y left, metres. That is also
 * the frame the motion packets' CSVs are written in, so nothing is converted
 * until render time.
 *
 * Why a numerical solver rather than closed-form 2-link IK: the G1 leg is not a
 * clean spherical hip. The hip yaw joint is offset 25 mm forward of the roll
 * axis and the knee is offset 78 mm BACK from the yaw axis, so a planar
 * law-of-cosines solution places the ankle several centimetres off. At walking
 * scale that error is the difference between a foot that is planted and a foot
 * that hovers or clips. Damped least squares over the real chain removes it.
 */

// ---------------------------------------------------------------------------
// Chain, DERIVED FROM THE URDF TREE rather than transcribed.
//
// Hand-transcribing this was a mistake once already. The G1's hip roll joint
// carries a fixed rpy of [0, -0.1749, 0] and the knee an equal and opposite
// [0, +0.1749, 0] — ten degrees each. They cancel exactly in the neutral pose,
// so a flat-sole check passes and the error stays invisible, but the moment hip
// roll or hip yaw moves they no longer cancel and the solved ankle sits
// somewhere the renderer does not draw it. urdf-loader honours those offsets;
// anything that solves against a simplified chain silently disagrees with the
// picture on screen.
//
// Building the chain from G1_TREE at load makes that class of bug impossible.
// ---------------------------------------------------------------------------
import { G1_TREE } from './G1Body.js';

/**
 * The foot's four contact spheres, read from the URDF collision blocks of
 * *_ankle_roll_link. Centres are at z = -0.03 with r = 0.005, so the sole
 * plane is z = -0.035 and the foot spans x = -0.05 (heel) to x = +0.12 (toe).
 */
export const FOOT_CONTACTS = [
  [-0.05, 0.025, -0.035], [-0.05, -0.025, -0.035],   // heel outer, heel inner
  [0.12, 0.03, -0.035], [0.12, -0.03, -0.035],       // toe outer, toe inner
];
export const SOLE_CENTRE = [0.035, 0, -0.035];        // centre of the support polygon
export const FOOT_LEN = 0.17;
export const FOOT_WID = 0.06;

/** Joint limits from the URDF, so a solved pose is always reachable. */
export const LEG_LIMITS = {
  left: [[-2.5307, 2.8798], [-0.5236, 2.9671], [-2.7576, 2.7576],
         [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618]],
  right: [[-2.5307, 2.8798], [-2.9671, 0.5236], [-2.7576, 2.7576],
          [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618]],
};

// ---------------------------------------------------------------------------
// Small 3x3 / vector helpers. Row-major, m[r*3+c].
// ---------------------------------------------------------------------------
export const mmul = (A, B) => {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
  }
  return C;
};
export const mapv = (A, v) => [
  A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
  A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
  A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
];
export const mT = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
export const rotAxis = (axis, t) => {
  const c = Math.cos(t), s = Math.sin(t);
  if (axis === 'x') return [1, 0, 0, 0, c, -s, 0, s, c];
  if (axis === 'y') return [c, 0, s, 0, 1, 0, -s, 0, c];
  return [c, -s, 0, s, c, 0, 0, 0, 1];
};
/** Intrinsic yaw-pitch-roll (Z then Y then X), the convention used for feet. */
export const rpyMat = (roll, pitch, yaw) =>
  mmul(mmul(rotAxis('z', yaw), rotAxis('y', pitch)), rotAxis('x', roll));

export const quatToMat = (x, y, z, w) => {
  const n = Math.hypot(x, y, z, w) || 1; x /= n; y /= n; z /= n; w /= n;
  return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
          2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
};
export const matToQuat = (m) => {
  const tr = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2; w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s; }
  else if (m[4] > m[8]) { const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2; w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s; }
  else { const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2; w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s; }
  return [x, y, z, w];
};
/** Axis-angle vector of a rotation matrix — the orientation error term. */
export const rotErr = (R) => {
  const c = Math.min(1, Math.max(-1, (R[0] + R[4] + R[8] - 1) / 2));
  const a = Math.acos(c);
  if (a < 1e-8) return [0, 0, 0];
  const k = a / (2 * Math.sin(a));
  return [k * (R[7] - R[5]), k * (R[2] - R[6]), k * (R[3] - R[1])];
};

/** Fixed rotation of a URDF <origin rpy="r p y"> — Rz(y) * Ry(p) * Rx(r). */
export function rpyFixed(r, p, y) {
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

const IDENT = [1, 0, 0, 0, 1, 0, 0, 0, 1];

/** Walk the tree from the pelvis to a named link, collecting the moving joints. */
function chainTo(linkName) {
  const byName = new Map(G1_TREE.map((l, i) => [l.name, i]));
  const path = [];
  let i = byName.get(linkName);
  while (i !== undefined && i >= 0) { path.unshift(G1_TREE[i]); i = G1_TREE[i].parent; }
  // Accumulate any fixed links into the next moving joint's offset.
  const out = [];
  let pendT = [0, 0, 0], pendR = IDENT;
  for (const l of path) {
    if (l.parent < 0) continue;                       // pelvis itself
    const t = mapv(pendR, l.xyz);
    pendT = [pendT[0] + t[0], pendT[1] + t[1], pendT[2] + t[2]];
    pendR = mmul(pendR, rpyFixed(l.rpy[0], l.rpy[1], l.rpy[2]));
    if (!l.joint) continue;                           // fixed: fold into the next
    out.push({ t: pendT, R: pendR, axis: l.axis, sign: l.sign, joint: l.joint });
    pendT = [0, 0, 0]; pendR = IDENT;
  }
  return out;
}

export const LEG_CHAIN = {
  left: chainTo('left_ankle_roll_link'),
  right: chainTo('right_ankle_roll_link'),
};

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------
/**
 * Ankle-roll-link pose from six joint angles, in PELVIS coordinates.
 * @returns {{p:number[], R:number[]}}
 */
export function legFK(side, q) {
  const chain = LEG_CHAIN[side];
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  let p = [0, 0, 0];
  for (let i = 0; i < chain.length; i++) {
    const c = chain[i];
    const t = mapv(R, c.t);
    p = [p[0] + t[0], p[1] + t[1], p[2] + t[2]];
    // fixed origin rotation FIRST, then the joint's own rotation about its axis
    R = mmul(mmul(R, c.R), rotAxis(c.axis, q[i] * c.sign));
  }
  return { p, R };
}

/** The four contact points in pelvis coordinates, for a given leg pose. */
export function contactPoints(side, q) {
  const { p, R } = legFK(side, q);
  return FOOT_CONTACTS.map((c) => {
    const v = mapv(R, c);
    return [p[0] + v[0], p[1] + v[1], p[2] + v[2]];
  });
}

// ---------------------------------------------------------------------------
// Inverse kinematics — damped least squares over the real 6-joint chain.
// ---------------------------------------------------------------------------
/** Solve a 6x6 linear system by Gaussian elimination with partial pivoting. */
function solve6(A, b) {
  const n = 6, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) continue;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => (Math.abs(row[i]) < 1e-12 ? 0 : row[n] / row[i]));
}

/**
 * Place the ankle-roll link at a target pose expressed in PELVIS coordinates.
 *
 * POSITION AND ORIENTATION ARE NOT EQUALLY IMPORTANT, so they are not weighted
 * equally. Where the foot IS decides whether the robot is in contact with the
 * ground; how the sole is tilted only decides how well it conforms to what it
 * is standing on. The ankle has 15 degrees of roll and 80 of pitch, and a sole
 * pose read off rocky terrain regularly asks for more than that. Solved with
 * equal weights, an unreachable tilt drags position off by centimetres and the
 * foot leaves the ground — trading the thing that matters for the thing that
 * does not.
 *
 * Weighting position an order of magnitude higher makes that trade the right
 * way round: the foot stays exactly where it was planted and simply conforms
 * as far as the ankle allows. Measured over the fourteen scenes this is worth
 * roughly 10x on peak position error.
 *
 * @param {string} side           'left' | 'right'
 * @param {number[]} pTarget      ankle-roll origin, pelvis frame
 * @param {number[]} RTarget      ankle-roll orientation, pelvis frame
 * @param {number[]} qSeed        starting angles (previous frame — keeps continuity)
 * @param {object} [opt]          { wPos, wRot } task weights, metres vs radians
 * @returns {{q:number[], posErr:number, rotErr:number, iters:number}}
 */
export function legIK(side, pTarget, RTarget, qSeed, opt = {}) {
  const lim = LEG_LIMITS[side];
  const q = qSeed.slice();
  // KNEE FLOOR — keep the leg off its own singularity.
  //
  // Leg length goes as cos(knee/2), so dL/dknee is exactly ZERO at full
  // extension. Measured on this chain: at knee = 0 a millimetre of reach costs
  // 8166 degrees of knee; at 0.05 rad it costs 7 degrees, which at 30 fps is
  // 211 deg/s for one millimetre of foot movement. That is the whole mechanism
  // behind the knee "snap" — not a bad pose, but a correct pose reached
  // through a region where the joint has no authority and the solver has to
  // swing it wildly to achieve nothing.
  //
  // At 0.30 rad the same millimetre costs 1.2 degrees, a factor of six better,
  // and the price is 7.3 mm of reach: 0.6564 m straight against 0.6491 m at
  // the floor. Anything that budgets reach must use the smaller number, which
  // is why MAX_HIP_REACH in the retargeter is quoted at the floor and not at
  // full extension.
  const kneeFloor = opt.kneeFloor ?? 0;
  if (kneeFloor > 0) q[3] = Math.max(q[3], kneeFloor);
  const EPS = 1e-4;
  // Squared weights, since they enter through JtWJ.
  const wP = (opt.wPos ?? 1.0) ** 2, wR = (opt.wRot ?? 0.12) ** 2;
  let posErr = Infinity, rErr = Infinity, iter = 0, lastRot = Infinity;

  for (; iter < 60; iter++) {
    const cur = legFK(side, q);
    const ep = [pTarget[0] - cur.p[0], pTarget[1] - cur.p[1], pTarget[2] - cur.p[2]];
    const er = rotErr(mmul(RTarget, mT(cur.R)));
    posErr = Math.hypot(...ep); rErr = Math.hypot(...er);
    if (posErr < 2e-5 && rErr < 2e-4) break;
    // Converged as far as this pose allows: position is met and orientation
    // has stopped improving because a joint is against its stop. Iterating
    // further only grinds against the limit.
    if (posErr < 1e-4 && iter > 12 && Math.abs(rErr - lastRot) < 1e-6) break;
    lastRot = rErr;

    // Numerical Jacobian: 6 task rows (3 position, 3 rotation) x 6 joints.
    const J = [[], [], [], [], [], []];
    for (let j = 0; j < 6; j++) {
      const qp = q.slice(); qp[j] += EPS;
      const f = legFK(side, qp);
      const dp = [(f.p[0] - cur.p[0]) / EPS, (f.p[1] - cur.p[1]) / EPS, (f.p[2] - cur.p[2]) / EPS];
      const dr = rotErr(mmul(f.R, mT(cur.R))).map((v) => v / EPS);
      for (let r = 0; r < 3; r++) J[r][j] = dp[r];
      for (let r = 0; r < 3; r++) J[3 + r][j] = dr[r];
    }

    // (Jt W J + lambda^2 I) dq = Jt W e — damping keeps it stable at full
    // extension, W puts the effort into position rather than tilt.
    const lambda = 0.06;
    const e = [...ep, ...er];
    const W = [wP, wP, wP, wR, wR, wR];
    const A = [], bb = [];
    for (let r = 0; r < 6; r++) {
      A.push(new Array(6).fill(0));
      let s = 0;
      for (let k = 0; k < 6; k++) s += J[k][r] * W[k] * e[k];
      bb.push(s);
    }
    for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) {
      let s = 0;
      for (let k = 0; k < 6; k++) s += J[k][r] * W[k] * J[k][c];
      A[r][c] = s + (r === c ? lambda * lambda : 0);
    }
    const dq = solve6(A, bb);

    // Step limit: a large jump past a singularity produces a leg that snaps.
    let scale = 1;
    for (const d of dq) if (Math.abs(d) > 0.25) scale = Math.min(scale, 0.25 / Math.abs(d));
    for (let j = 0; j < 6; j++) {
      const lo = j === 3 ? Math.max(lim[j][0], kneeFloor) : lim[j][0];
      q[j] = Math.min(lim[j][1], Math.max(lo, q[j] + dq[j] * scale));
    }
  }
  return { q, posErr, rotErr: rErr, iters: iter };
}

/** Neutral leg crouch used to seed the first frame. */
export const NEUTRAL_LEG = [-0.18, 0, 0, 0.36, -0.18, 0];

// ---------------------------------------------------------------------------
// Whole-body centre of mass
// ---------------------------------------------------------------------------
/**
 * COM of the entire robot in PELVIS coordinates, from the URDF's own link
 * masses and inertial origins.
 *
 * On the ground this is a convenience. In microgravity it is the whole story:
 * with no external force the COM travels in a straight line at constant speed
 * no matter what the limbs do, so the COM is the thing that must be held fixed
 * and the PELVIS is the thing that has to move to satisfy it. Sweep 1.4 kg of
 * arm forward and the body must shift back to compensate — that recoil is
 * precisely what makes a breaststroke in vacuum read as futile rather than as
 * swimming.
 *
 * @param {Object<string,number>} q  joint name -> angle (missing = 0)
 * @returns {{com:number[], mass:number}}
 */
export function bodyCOM(q) {
  const n = G1_TREE.length;
  const P = new Array(n), R = new Array(n);
  let sx = 0, sy = 0, sz = 0, M = 0;
  for (let i = 0; i < n; i++) {
    const l = G1_TREE[i];
    if (l.parent < 0) { P[i] = [0, 0, 0]; R[i] = [1, 0, 0, 0, 1, 0, 0, 0, 1]; }
    else {
      const Rp = R[l.parent], Pp = P[l.parent];
      const t = mapv(Rp, l.xyz);
      P[i] = [Pp[0] + t[0], Pp[1] + t[1], Pp[2] + t[2]];
      let Ri = mmul(Rp, rpyFixed(l.rpy[0], l.rpy[1], l.rpy[2]));
      if (l.joint && l.axis) Ri = mmul(Ri, rotAxis(l.axis, (q[l.joint] ?? 0) * l.sign));
      R[i] = Ri;
    }
    if (l.mass > 0) {
      const c = mapv(R[i], l.com);
      sx += l.mass * (P[i][0] + c[0]);
      sy += l.mass * (P[i][1] + c[1]);
      sz += l.mass * (P[i][2] + c[2]);
      M += l.mass;
    }
  }
  return { com: [sx / M, sy / M, sz / M], mass: M };
}

/** Joint-name map from a 29-column packet row, for bodyCOM. */
export const CSV_JOINT_ORDER = [
  'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
  'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
  'waist_yaw', 'waist_roll', 'waist_pitch',
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
  'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
  'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
].map((n) => `${n}_joint`);

export function jointMapFromRow(row) {
  const m = {};
  for (let i = 0; i < CSV_JOINT_ORDER.length; i++) m[CSV_JOINT_ORDER[i]] = row[7 + i];
  return m;
}

/**
 * Pose of EVERY link, in pelvis coordinates.
 *
 * bodyCOM already walks the tree but throws the per-link poses away. The packet
 * audit needs them: an NPZ ships body_pos_w alongside joint_pos, and the only
 * way to know whether those two describe the same robot is to run the URDF's
 * own forward kinematics on the joints and compare.
 *
 * @param {Object<string,number>} q  joint name -> angle (missing = 0)
 * @returns {{name:string, p:number[], R:number[]}[]}  tree order
 */
export function bodyFK(q) {
  const n = G1_TREE.length;
  const P = new Array(n), R = new Array(n);
  for (let i = 0; i < n; i++) {
    const l = G1_TREE[i];
    if (l.parent < 0) { P[i] = [0, 0, 0]; R[i] = IDENT; continue; }
    const Rp = R[l.parent], Pp = P[l.parent];
    const t = mapv(Rp, l.xyz);
    P[i] = [Pp[0] + t[0], Pp[1] + t[1], Pp[2] + t[2]];
    let Ri = mmul(Rp, rpyFixed(l.rpy[0], l.rpy[1], l.rpy[2]));
    if (l.joint && l.axis) Ri = mmul(Ri, rotAxis(l.axis, (q[l.joint] ?? 0) * l.sign));
    R[i] = Ri;
  }
  return G1_TREE.map((l, i) => ({ name: l.name, p: P[i], R: R[i] }));
}
