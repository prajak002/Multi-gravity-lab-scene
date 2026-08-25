/**
 * retarget_micro.mjs — rebuild the ISS clips so that free flight is actually
 * free flight.
 *
 * THE PROBLEM
 * Both ISS packets hold the PELVIS at exactly constant velocity during the
 * glide (0.1800 and 0.4218 m/s, flat to four decimals). That looks like careful
 * physics and is the wrong invariant. With no external force it is the CENTRE
 * OF MASS that travels in a straight line; the pelvis must move to compensate
 * for every limb that swings. Measured on the shipped clips, holding the pelvis
 * fixed instead drives the COM up to 36.6 mm off its line, a peak acceleration
 * of 3.14 m/s^2 — 107 N appearing from nowhere on a 34.1 kg body.
 *
 * For the breaststroke clip that inverts its entire meaning. The point of
 * "BREASTSTROKE_FUTILITY" is that swimming in vacuum achieves nothing; as
 * shipped, the swimming measurably propels the robot.
 *
 * WHAT THIS DOES
 *   - synthesises a real breaststroke (glide / outsweep / insweep / recovery,
 *     with the legs' whip kick offset from the arm cycle, as in the stroke)
 *   - pins the COM to a straight line at the packet's own stated release speed
 *   - derives the pelvis from the COM, so the body visibly recoils against
 *     every sweep — which is what makes the futility legible
 *   - conserves ANGULAR momentum too, so the torso counter-rotates against the
 *     arms and the robot rocks without ever turning, using the URDF's real
 *     inertia tensors
 *   - leaves the wall-push and rail-capture phases alone, because there an
 *     external contact genuinely exists
 */
import {
  G1_TREE, G1_TOTAL_MASS, inertiaMatrix,
} from '../src/sim/G1Body.js';
import {
  mmul, mapv, mT, rotAxis, quatToMat, matToQuat, rpyFixed,
} from '../src/sim/G1Kinematics.js';

const FPS = 30;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const smooth = (t) => t * t * (3 - 2 * t);
const lerp = (a, b, t) => a + (b - a) * t;

// ---------------------------------------------------------------------------
// whole-body pose: every link's transform, in BODY (pelvis) coordinates
// ---------------------------------------------------------------------------
function linkPoses(q) {
  const n = G1_TREE.length;
  const P = new Array(n), R = new Array(n);
  for (let i = 0; i < n; i++) {
    const l = G1_TREE[i];
    if (l.parent < 0) { P[i] = [0, 0, 0]; R[i] = [1, 0, 0, 0, 1, 0, 0, 0, 1]; continue; }
    const Rp = R[l.parent], Pp = P[l.parent];
    const t = mapv(Rp, l.xyz);
    P[i] = [Pp[0] + t[0], Pp[1] + t[1], Pp[2] + t[2]];
    let Ri = mmul(Rp, rpyFixed(l.rpy[0], l.rpy[1], l.rpy[2]));
    if (l.joint && l.axis) Ri = mmul(Ri, rotAxis(l.axis, (q[l.joint] ?? 0) * l.sign));
    R[i] = Ri;
  }
  return { P, R };
}

/** COM in body coordinates. */
export function comBody(q) {
  const { P, R } = linkPoses(q);
  let x = 0, y = 0, z = 0;
  for (let i = 0; i < G1_TREE.length; i++) {
    const l = G1_TREE[i];
    if (!l.mass) continue;
    const c = mapv(R[i], l.com);
    x += l.mass * (P[i][0] + c[0]);
    y += l.mass * (P[i][1] + c[1]);
    z += l.mass * (P[i][2] + c[2]);
  }
  return [x / G1_TOTAL_MASS, y / G1_TOTAL_MASS, z / G1_TOTAL_MASS];
}

/** Total inertia about the body COM, in body coordinates. */
function inertiaAboutCOM(q) {
  const { P, R } = linkPoses(q);
  const c = comBody(q);
  const J = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (let i = 0; i < G1_TREE.length; i++) {
    const l = G1_TREE[i];
    if (!l.mass) continue;
    const Ri = R[i];
    const Il = inertiaMatrix(l.I);
    const Iw = mmul(mmul(Ri, Il), mT(Ri));                 // rotate into body frame
    const cw = mapv(Ri, l.com);
    const d = [P[i][0] + cw[0] - c[0], P[i][1] + cw[1] - c[1], P[i][2] + cw[2] - c[2]];
    const d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    for (let r = 0; r < 3; r++) for (let s = 0; s < 3; s++) {
      J[r * 3 + s] += Iw[r * 3 + s] + l.mass * ((r === s ? d2 : 0) - d[r] * d[s]);
    }
  }
  return J;
}

/**
 * Angular momentum about the COM produced by the JOINTS alone, with the body
 * frame held still — the term the torso has to cancel.
 */
function internalMomentum(qPrev, qNext, dt) {
  const A = linkPoses(qPrev), B = linkPoses(qNext);
  const cA = comBody(qPrev), cB = comBody(qNext);
  const L = [0, 0, 0];
  for (let i = 0; i < G1_TREE.length; i++) {
    const l = G1_TREE[i];
    if (!l.mass) continue;
    const ca = mapv(A.R[i], l.com), cb = mapv(B.R[i], l.com);
    const pa = [A.P[i][0] + ca[0] - cA[0], A.P[i][1] + ca[1] - cA[1], A.P[i][2] + ca[2] - cA[2]];
    const pb = [B.P[i][0] + cb[0] - cB[0], B.P[i][1] + cb[1] - cB[1], B.P[i][2] + cb[2] - cB[2]];
    const v = [(pb[0] - pa[0]) / dt, (pb[1] - pa[1]) / dt, (pb[2] - pa[2]) / dt];
    const p = [(pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, (pa[2] + pb[2]) / 2];
    // m * (r x v)
    L[0] += l.mass * (p[1] * v[2] - p[2] * v[1]);
    L[1] += l.mass * (p[2] * v[0] - p[0] * v[2]);
    L[2] += l.mass * (p[0] * v[1] - p[1] * v[0]);
    // I * omega, with omega from the link's own rotation change
    const dR = mmul(B.R[i], mT(A.R[i]));
    const w = [(dR[7] - dR[5]) / (2 * dt), (dR[2] - dR[6]) / (2 * dt), (dR[3] - dR[1]) / (2 * dt)];
    const Iw = mmul(mmul(A.R[i], inertiaMatrix(l.I)), mT(A.R[i]));
    L[0] += Iw[0] * w[0] + Iw[1] * w[1] + Iw[2] * w[2];
    L[1] += Iw[3] * w[0] + Iw[4] * w[1] + Iw[5] * w[2];
    L[2] += Iw[6] * w[0] + Iw[7] * w[1] + Iw[8] * w[2];
  }
  return L;
}

/** 3x3 inverse. */
function inv3(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const id = 1 / det;
  return [A * id, (c * h - b * i) * id, (b * f - c * e) * id,
          B * id, (a * i - c * g) * id, (c * d - a * f) * id,
          C * id, (b * g - a * h) * id, (a * e - b * d) * id];
}

/** Rodrigues: rotation matrix for an angular-velocity step. */
function expMap(w, dt) {
  const t = Math.hypot(w[0], w[1], w[2]) * dt;
  if (t < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const k = [w[0] * dt / t, w[1] * dt / t, w[2] * dt / t];
  const c = Math.cos(t), s = Math.sin(t), v = 1 - c;
  return [
    c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s,
    k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s,
    k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v,
  ];
}

// ---------------------------------------------------------------------------
// the stroke
// ---------------------------------------------------------------------------
/**
 * One breaststroke cycle, phase u in [0,1).
 *
 * The real stroke is four phases, not a sine wave, and the legs are offset from
 * the arms: you pull with the arms while the legs recover, then kick while the
 * arms extend. Reproducing that offset is most of what makes it read as
 * swimming rather than as flapping.
 *
 *   0.00-0.30  outsweep   hands sweep out and apart, elbows high
 *   0.30-0.55  insweep    elbows bend hard, hands scull in to the chest
 *   0.55-0.80  recovery   hands driven forward together, legs draw up
 *   0.80-1.00  glide      arms extended, legs whip together and hold
 */
export function breaststroke(u) {
  const seg = (a, b) => clamp((u - a) / (b - a), 0, 1);
  let shPitch, shRoll, elbow;
  if (u < 0.30) {
    const t = smooth(seg(0, 0.30));
    shPitch = lerp(-1.20, -0.86, t); shRoll = lerp(0.16, 0.98, t); elbow = lerp(0.10, 0.42, t);
  } else if (u < 0.55) {
    const t = smooth(seg(0.30, 0.55));
    shPitch = lerp(-0.86, -0.28, t); shRoll = lerp(0.98, 0.52, t); elbow = lerp(0.42, 1.45, t);
  } else if (u < 0.80) {
    const t = smooth(seg(0.55, 0.80));
    shPitch = lerp(-0.28, -1.20, t); shRoll = lerp(0.52, 0.20, t); elbow = lerp(1.45, 0.22, t);
  } else {
    const t = smooth(seg(0.80, 1.0));
    shPitch = lerp(-1.20, -1.20, t); shRoll = lerp(0.20, 0.16, t); elbow = lerp(0.22, 0.10, t);
  }
  // Whip kick, a quarter cycle behind the arms.
  const k = (u + 0.55) % 1;
  let hipPitch, hipRoll, knee;
  if (k < 0.35) {                       // draw the heels up
    const t = smooth(k / 0.35);
    hipPitch = lerp(-0.10, -0.48, t); knee = lerp(0.16, 1.38, t); hipRoll = lerp(0.06, 0.30, t);
  } else if (k < 0.60) {                // sweep out and whip together
    const t = smooth((k - 0.35) / 0.25);
    hipPitch = lerp(-0.48, -0.05, t); knee = lerp(1.38, 0.14, t); hipRoll = lerp(0.30, 0.44, t);
  } else {                              // squeeze shut and stream
    const t = smooth((k - 0.60) / 0.40);
    hipPitch = lerp(-0.05, -0.10, t); knee = lerp(0.14, 0.16, t); hipRoll = lerp(0.44, 0.06, t);
  }
  return { shPitch, shRoll, elbow, hipPitch, hipRoll, knee };
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
const J = {};
[
  'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
  'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
  'waist_yaw', 'waist_roll', 'waist_pitch',
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
  'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
  'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
].forEach((n, i) => { J[n] = 7 + i; });

const jointMap = (row) => {
  const m = {};
  for (const [n, c] of Object.entries(J)) m[`${n}_joint`] = row[c];
  return m;
};

/**
 * @param {number[][]} rows   original CSV rows
 * @param {object} opt
 *   releaseFrame  first frame of free flight (feet clear of the wall)
 *   captureFrame  first frame of rail contact, or null for none
 *   releaseSpeed  m/s along +x, from the packet's own manifest
 *   stroke        'breaststroke' | 'glide'
 *   strokePeriod  s
 */
export function retargetMicro(rows, opt) {
  const n = rows.length;
  const rel = opt.releaseFrame;
  const cap = opt.captureFrame ?? n;
  const out = rows.map((r) => r.slice());

  // ---- 1. joints -------------------------------------------------------
  if (opt.stroke === 'breaststroke') {
    const T = (opt.strokePeriod ?? 1.1) * FPS;
    for (let i = rel; i < n; i++) {
      // ease the stroke in over half a cycle so it grows out of the launch pose
      const ramp = smooth(clamp((i - rel) / (T * 0.5), 0, 1));
      const s = breaststroke(((i - rel) / T) % 1);
      const set = (name, v) => {
        out[i][J[name]] = lerp(rows[i][J[name]], v, ramp);
      };
      set('left_shoulder_pitch', s.shPitch);   set('right_shoulder_pitch', s.shPitch);
      set('left_shoulder_roll', s.shRoll);     set('right_shoulder_roll', -s.shRoll);
      set('left_elbow', s.elbow);              set('right_elbow', s.elbow);
      set('left_hip_pitch', s.hipPitch);       set('right_hip_pitch', s.hipPitch);
      set('left_hip_roll', s.hipRoll);         set('right_hip_roll', -s.hipRoll);
      set('left_knee', s.knee);                set('right_knee', s.knee);
    }
  }

  // ---- 2. free flight: COM on a straight line -------------------------
  // Orientation first, because the pelvis position depends on it.
  const q = out.map(jointMap);
  const R = out.map((r) => quatToMat(r[3], r[4], r[5], r[6]));

  // Angular momentum at release, in world coordinates. Whatever the wall push
  // left the body with is conserved from then on.
  const dt = 1 / FPS;
  let Lworld = [0, 0, 0];
  if (rel > 0 && rel < n - 1) {
    const Lb = internalMomentum(q[rel - 1], q[rel + 1], 2 * dt);
    Lworld = mapv(R[rel], Lb);
  }

  for (let i = rel + 1; i < Math.min(cap, n); i++) {
    const Lb = mT(R[i - 1]);                                    // world -> body
    const Ltarget = mapv(Lb, Lworld);
    const Lint = internalMomentum(q[i - 1], q[i], dt);
    const Jc = inertiaAboutCOM(q[i - 1]);
    const w = mapv(inv3(Jc), [Ltarget[0] - Lint[0], Ltarget[1] - Lint[1], Ltarget[2] - Lint[2]]);
    R[i] = mmul(R[i - 1], expMap(w, dt));
  }

  // COM line, anchored at release, at the packet's stated speed.
  const comAt = (i) => {
    const c = mapv(R[i], comBody(q[i]));
    return [out[i][0] + c[0], out[i][1] + c[1], out[i][2] + c[2]];
  };
  const com0 = comAt(rel);
  for (let i = rel; i < Math.min(cap, n); i++) {
    const t = (i - rel) / FPS;
    const target = [com0[0] + opt.releaseSpeed * t, com0[1], com0[2]];
    // pelvis = COM - R * comBody : the body moves so the COM cannot.
    const c = mapv(R[i], comBody(q[i]));
    out[i][0] = target[0] - c[0];
    out[i][1] = target[1] - c[1];
    out[i][2] = target[2] - c[2];
    const quat = matToQuat(R[i]);
    out[i][3] = quat[0]; out[i][4] = quat[1]; out[i][5] = quat[2]; out[i][6] = quat[3];
  }

  // ---- 3. after capture: blend back to the authored arrest -------------
  if (cap < n) {
    const B = 12;
    const dx = out[cap - 1][0] - rows[cap - 1][0];
    for (let i = cap; i < n; i++) {
      const w = 1 - smooth(clamp((i - cap) / B, 0, 1));
      out[i][0] = rows[i][0] + dx * w;
    }
  }
  return { rows: out };
}

/** Straight-line error of the COM through free flight — the acceptance test. */
export function comAudit(rows, rel, cap) {
  const q = rows.map(jointMap);
  const com = rows.map((r, i) => {
    const c = mapv(quatToMat(r[3], r[4], r[5], r[6]), comBody(q[i]));
    return [r[0] + c[0], r[1] + c[1], r[2] + c[2]];
  });
  const a = rel + 3, b = Math.min(cap, rows.length) - 3;
  const vx = (com[b][0] - com[a][0]) / ((b - a) / FPS);
  let maxDev = 0, sum = 0, maxAcc = 0;
  for (let i = a; i <= b; i++) {
    const d = Math.abs(com[i][0] - (com[a][0] + vx * ((i - a) / FPS)));
    maxDev = Math.max(maxDev, d); sum += d;
  }
  for (let i = a + 1; i < b; i++) {
    maxAcc = Math.max(maxAcc, Math.abs((com[i + 1][0] - 2 * com[i][0] + com[i - 1][0]) * FPS * FPS));
  }
  return { vx, maxDev, meanDev: sum / (b - a + 1), maxAcc, ghostForce: maxAcc * G1_TOTAL_MASS };
}

/**
 * Find the free-flight window without being told.
 *
 * Free flight is the stretch where the root's along-track velocity is flat:
 * before it the feet are still driving against the wall, after it a hand is on
 * the rail (or the body is on the bulkhead). Detecting it from the data means
 * the same code handles both packets, which schedule their phases differently.
 */
export function detectPhases(rows, override = null) {
  const n = rows.length;
  const v = new Array(n).fill(0);
  for (let i = 1; i < n; i++) v[i] = (rows[i][0] - rows[i - 1][0]) * FPS;
  v[0] = v[1];
  // The glide speed is a HIGH PERCENTILE of the clip, not its median.
  //
  // BrakeGap's PragyaSpace run captures the rail at 4.77 s and then holds
  // station for the remaining five seconds, so more than half the clip sits at
  // zero and a median reports 0.13 m/s for a run the manifest states launched
  // at 0.72. The 85th percentile recovers the true plateau on every clip, and
  // an explicit value from the manifest overrides it when one is given.
  const sorted = v.slice(1).slice().sort((a, b) => a - b);
  const plateau = override ?? sorted[Math.floor(sorted.length * 0.85)];
  const tol = Math.max(0.01, Math.abs(plateau) * 0.04);
  let rel = 0, cap = n;
  for (let i = 1; i < n; i++) if (Math.abs(v[i] - plateau) <= tol) { rel = i; break; }
  for (let i = n - 1; i > rel; i--) if (Math.abs(v[i] - plateau) <= tol) { cap = i + 1; break; }
  return { releaseFrame: rel, captureFrame: cap >= n - 2 ? null : cap, plateauSpeed: plateau };
}
