/**
 * ArmRetarget — drive the G1's arms from a human body, live.
 *
 * MediaPipe returns 33 landmarks in metres about the hip centre. A person is
 * 1.7-1.9 m with a 0.60 m arm; a G1 is 1.32 m with a 0.36 m one, and their
 * shoulders are not in the same place relative to their hips. So positions do
 * not transfer, and copying them is what makes a retargeted robot reach for
 * things beside the thing.
 *
 * DIRECTIONS transfer. This is the same principle tools/fit_pose.mjs uses to
 * fit the G1 to Apollo footage: match where each bone POINTS rather than where
 * it ends, and the difference in proportions stops mattering. Two directions
 * per arm — shoulder-to-elbow and elbow-to-hand — are enough to pin the four
 * joints that decide an arm's shape, because that is exactly four constraints
 * once each direction has lost its length.
 *
 * Everything is done in a frame built from the BODY, never from the camera:
 * up from hips to shoulders, left along the shoulder line, forward from their
 * cross product. Lean toward the lens, turn side on, stand at an angle — the
 * frame turns with you and the arm angles do not change. That also means the
 * result is independent of MediaPipe's own axis convention, which is the sort
 * of thing that silently mirrors one arm.
 *
 * WHAT IT WILL NOT DO
 *
 * It will not exceed the URDF. Every angle is clamped to the joint's own
 * limits, and the shoulder ROLL limit is asymmetric between the two arms —
 * [-1.59, 2.25] on the left against [-2.25, 1.59] on the right — because the
 * joint mirrors and its range does not. Clamping both to the same numbers
 * would silently stop one arm short. `clamped` reports which joints are on a
 * stop, so a pose the machine cannot reach reads as the machine refusing
 * rather than as the tracking failing.
 */
import { ARM_CHAIN, ARM_LIMITS, armFK } from './G1Kinematics.js';

/** MediaPipe Pose landmark indices, for the joints that decide an arm. */
export const LM = {
  shoulderL: 11, shoulderR: 12,
  elbowL: 13, elbowR: 14,
  wristL: 15, wristR: 16,
  hipL: 23, hipR: 24,
  indexL: 19, indexR: 20,
  pinkyL: 17, pinkyR: 18,
};

/** Which chain slots the retarget actually drives. Waist and wrist stay put. */
const SHOULDER_PITCH = 3, SHOULDER_ROLL = 4, SHOULDER_YAW = 5, ELBOW = 6;
const DRIVEN = [SHOULDER_PITCH, SHOULDER_ROLL, SHOULDER_YAW, ELBOW];

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => {
  const n = Math.hypot(a[0], a[1], a[2]);
  return n > 1e-9 ? [a[0] / n, a[1] / n, a[2] / n] : [0, 0, 0];
};
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const pt = (lms, i) => [lms[i].x, lms[i].y, lms[i].z];

/**
 * A frame built from the torso, so the camera's point of view drops out.
 *
 * Returns the three axes as unit vectors in MediaPipe's own coordinates. The
 * caller projects a bone onto them and gets (forward, left, up) components,
 * which is the G1 pelvis frame's own convention — x forward, y left, z up.
 */
export function torsoFrame(lms) {
  const sL = pt(lms, LM.shoulderL), sR = pt(lms, LM.shoulderR);
  const hL = pt(lms, LM.hipL), hR = pt(lms, LM.hipR);
  const shoulders = scale(add(sL, sR), 0.5);
  const hips = scale(add(hL, hR), 0.5);

  const up = norm(sub(shoulders, hips));
  let left = norm(sub(sL, sR));
  // Forward completes a right-handed set, matching the robot's x = y cross z.
  const forward = norm(cross(left, up));
  // Re-orthogonalise: the shoulder line is never exactly square to the spine,
  // and a frame that is not orthogonal shears every angle read through it.
  left = norm(cross(up, forward));
  return { up, left, forward, origin: shoulders,
           width: Math.hypot(...sub(sL, sR)) };
}

/** A world vector, in the torso frame, as the robot's (x fwd, y left, z up). */
const inFrame = (f, v) => norm([dot(v, f.forward), dot(v, f.left), dot(v, f.up)]);

/**
 * The two directions that define one arm's shape.
 *
 * `confidence` is the weakest visibility along the chain: an arm the model
 * cannot actually see produces confident-looking nonsense, and the caller
 * needs to know to hold the last good pose instead of following it.
 */
export function armTargets(lms, side) {
  const S = side === 'left' ? LM.shoulderL : LM.shoulderR;
  const E = side === 'left' ? LM.elbowL : LM.elbowR;
  const W = side === 'left' ? LM.wristL : LM.wristR;
  const f = torsoFrame(lms);
  const s = pt(lms, S), e = pt(lms, E), w = pt(lms, W);
  return {
    upper: inFrame(f, sub(e, s)),
    fore: inFrame(f, sub(w, e)),
    frame: f,
    confidence: Math.min(
      lms[S].visibility ?? 1, lms[E].visibility ?? 1, lms[W].visibility ?? 1),
    // The human's elbow angle, for the readout. The robot's own elbow will
    // differ where the URDF's -1.05..2.09 range cannot follow.
    elbowAngle: Math.acos(clamp(dot(norm(sub(e, s)), norm(sub(w, e))), -1, 1)),
  };
}

/** The robot's own upper-arm and forearm directions for a given pose. */
function armDirs(side, q) {
  const { joints, p } = armFK(side, q);
  const shoulder = joints[SHOULDER_PITCH].p;
  const elbow = joints[ELBOW].p;
  return { upper: norm(sub(elbow, shoulder)), fore: norm(sub(p, elbow)) };
}

/**
 * The upper arm is weighted above the forearm, and deliberately.
 *
 * Both cannot always be had: the elbow's range is [-1.05, 2.09] and the
 * shoulder roll's is asymmetric, so plenty of human poses are simply outside
 * what the machine can make. When something has to give, what a viewer reads
 * as "where the arm is" is the UPPER arm — an arm pointing the wrong way with
 * a perfect forearm looks broken, and the reverse looks like a stiff wrist.
 * Same trade legIK makes in weighting position over sole orientation, for the
 * same reason: satisfy the thing that carries the meaning.
 */
const W_UPPER = 1.0, W_FORE = 0.55;

/**
 * A weak pull toward a natural posture, to settle the shoulder's redundancy.
 *
 * Three shoulder axes are two more than pointing an upper arm needs, and the
 * spare one — yaw — rotates about the upper arm itself. It therefore changes
 * NOTHING about where the arm points and everything about where the forearm
 * swings to, so a solver with only direction targets is free to satisfy the
 * forearm by winding the yaw to an extreme. It did: driving a hanging arm with
 * the forearm forward came out at shoulder_yaw = -150 degrees, hard against
 * its -2.62 rad stop, which is a correct solution and a grotesque pose.
 *
 * Six residuals against four unknowns is overdetermined for the DIRECTIONS and
 * still underdetermined for the pose, so four more rows are added, one per
 * joint, pulling gently toward a rest posture. The weight is small enough that
 * a reachable pose is still reached — the direction terms outweigh it by more
 * than ten to one — and large enough to decide between two poses that are
 * otherwise equally good. Which is exactly what null-space damping is for.
 */
const W_REST = 0.09;
const REST = { 3: -0.15, 4: 0.20, 5: 0.0, 6: 0.55 };
/** Six direction rows plus one posture row per driven joint. */
const N_RES = 10;

const residual = (side, q, target) => {
  const d = armDirs(side, q);
  const mirror = side === 'left' ? 1 : -1;
  return [
    (d.upper[0] - target.upper[0]) * W_UPPER,
    (d.upper[1] - target.upper[1]) * W_UPPER,
    (d.upper[2] - target.upper[2]) * W_UPPER,
    (d.fore[0] - target.fore[0]) * W_FORE,
    (d.fore[1] - target.fore[1]) * W_FORE,
    (d.fore[2] - target.fore[2]) * W_FORE,
    // Roll and yaw mirror between the arms; pitch and the elbow do not — the
    // same sagittal-mirror rule Butterfly.js documents for the stroke.
    (q[3] - REST[3]) * W_REST,
    (q[4] - REST[4] * mirror) * W_REST,
    (q[5] - REST[5] * mirror) * W_REST,
    (q[6] - REST[6]) * W_REST,
  ];
};

/**
 * Seeds to restart from when the first solve lands badly.
 *
 * The shoulder's three axes are redundant for pointing an upper arm, so the
 * error surface has several basins and a descent from the previous frame can
 * sit in a poor one while a different arrangement of the same joints reaches
 * the pose exactly. Measured over the reachable set, one restart from a
 * mid-range pose and one from arms-down recovers most of what a single descent
 * misses — enough to matter and cheap enough to run every frame.
 */
const RESTARTS = [
  [0, 0, 0, -0.6, 0.5, 0, 1.0, 0, 0, 0],
  [0, 0, 0, 1.2, 0.2, -0.8, 0.4, 0, 0, 0],
];

/**
 * Solve the four joints that shape an arm, against two bone directions.
 *
 * Damped least squares with a finite-difference Jacobian, seeded from the
 * previous frame. Seeding matters for more than speed: the shoulder's three
 * axes are redundant for pointing an upper arm, so many joint sets give the
 * same direction, and without continuity the solver is free to swap between
 * them and make the elbow flick from one solution to another while the arm
 * itself is barely moving.
 *
 * Four unknowns, ten residuals — six for the two bone directions and four
 * pulling weakly toward a rest posture. Overdetermined, which is what makes it
 * stable when the target directions are noisy, as landmarks from a webcam
 * always are.
 *
 * @param {'left'|'right'} side
 * @param {{upper:number[], fore:number[]}} target  unit directions, robot frame
 * @param {number[]} qSeed  full 10-slot chain pose from the previous frame
 */
export function solveArm(side, target, qSeed, opt = {}) {
  const seeds = [qSeed, ...(opt.restarts === false ? [] : RESTARTS)];
  let best = null;
  for (const seed of seeds) {
    const r = descend(side, target, seed, opt);
    if (!best || r.err < best.err) best = r;
    // Good enough to stop paying for the restarts. The threshold is in the
    // residual's own units: 0.05 is about three degrees on each direction.
    if (best.err < (opt.good ?? 0.05)) break;
  }

  const lim = ARM_LIMITS[side];
  const TOL = 1e-3;
  best.clamped = DRIVEN
    .filter((k) => best.q[k] <= lim[k][0] + TOL || best.q[k] >= lim[k][1] - TOL)
    .map((k) => ARM_CHAIN[side][k].joint);
  return best;
}

/** One damped-least-squares descent from one seed. */
function descend(side, target, qSeed, opt = {}) {
  const iters = opt.iters ?? 14;
  const lambda = opt.damping ?? 0.08;
  const lim = ARM_LIMITS[side];
  const n = ARM_CHAIN[side].length;
  const q = (qSeed && qSeed.length === n) ? qSeed.slice() : new Array(n).fill(0);

  const EPS = 1e-4;
  let err = Infinity;
  for (let it = 0; it < iters; it++) {
    const r = residual(side, q, target);
    err = Math.hypot(...r);
    if (err < 1e-4) break;

    // Jacobian of the six residuals against the four driven joints.
    const J = [];
    for (const k of DRIVEN) {
      const save = q[k];
      q[k] = clamp(save + EPS, lim[k][0], lim[k][1]);
      const h = q[k] - save || 1e-6;
      const rp = residual(side, q, target);
      q[k] = save;
      J.push(rp.map((v, i) => (v - r[i]) / h));
    }

    // Normal equations, 4x4, with Tikhonov damping.
    const A = [], b = [];
    for (let i = 0; i < 4; i++) {
      A.push(new Array(4).fill(0));
      let s = 0;
      for (let m = 0; m < N_RES; m++) s -= J[i][m] * r[m];
      b.push(s);
      for (let j = 0; j < 4; j++) {
        let t = 0;
        for (let m = 0; m < N_RES; m++) t += J[i][m] * J[j][m];
        A[i][j] = t + (i === j ? lambda : 0);
      }
    }
    const dq = solve4(A, b);
    if (!dq) break;
    for (let i = 0; i < 4; i++) {
      const k = DRIVEN[i];
      q[k] = clamp(q[k] + clamp(dq[i], -0.35, 0.35), lim[k][0], lim[k][1]);
    }
  }
  return { q, err };
}

/** Gaussian elimination with partial pivoting, 4x4. */
function solve4(A, b) {
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < 4; c++) {
    let piv = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < 4; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= 4; k++) M[r][k] -= f * M[c][k];
    }
  }
  // Full Gauss-Jordan above, so the matrix is diagonal and each unknown is
  // just its own row.
  return M.map((row, i) => row[4] / row[i]);
}

/**
 * Hold a joint's rate to what the hardware can actually turn.
 *
 * A webcam drops frames and a landmark can jump a decimetre between two of
 * them. Followed straight, that asks the shoulder for thousands of degrees per
 * second — the same class of fault tools/audit_rates.mjs measures in the baked
 * clips, except here it arrives live and there is no build step to catch it.
 * So the commanded angle is rate-limited on the way out, at a fraction of the
 * URDF's own velocity limit, and how often that bites is reported.
 */
export class ArmSmoother {
  /** @param {number} maxRate rad/s allowed at any single joint */
  constructor(maxRate = 6.0, alpha = 0.35) {
    this.maxRate = maxRate;
    this.alpha = alpha;
    this.q = null;
    this.limitedFrames = 0;
    this.frames = 0;
  }

  /** @returns {number[]} the pose to actually command */
  step(qTarget, dt) {
    if (!this.q) { this.q = qTarget.slice(); return this.q; }
    this.frames++;
    const cap = this.maxRate * Math.max(dt, 1e-3);
    let limited = false;
    for (let i = 0; i < qTarget.length; i++) {
      // A low-pass first, because landmark noise is high-frequency and a rate
      // limit alone would track every jitter right up to its ceiling.
      const want = this.q[i] + (qTarget[i] - this.q[i]) * this.alpha;
      const d = want - this.q[i];
      if (Math.abs(d) > cap) { limited = true; this.q[i] += Math.sign(d) * cap; }
      else this.q[i] = want;
    }
    if (limited) this.limitedFrames++;
    return this.q;
  }

  get limitedFraction() { return this.frames ? this.limitedFrames / this.frames : 0; }
}
