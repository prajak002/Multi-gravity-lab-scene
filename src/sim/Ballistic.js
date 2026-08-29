/**
 * Ballistic — what a G1 can launch with, and what gravity then does with it.
 *
 * WHY THIS EXISTS
 *
 * Every motion in this repository is a walk. Walks are where the contact work
 * is, but they are the worst possible way to show what gravity does, because a
 * walking robot keeps a foot on the ground almost all the time and gravity
 * only gets to act on the small fraction of the cycle that is flight. Watch the
 * Apollo film and the crews are not walking: they lope, they bound, they push
 * off with both feet, they hang, and — repeatedly — they fall over and get up.
 * That is what one sixth g looks like, and none of it was in here.
 *
 * THE NUMBER THAT MAKES IT WORTH DOING
 *
 * Ask what limits a jump and the answer is not what most people expect.
 *
 *   FORCE. Two knees at the URDF's own effort limit of 139 N m. Through most
 *   of the push the leg is bent, the moment arm is large and the force is
 *   modest; as the leg straightens the arm collapses and the available force
 *   goes up without bound. On the Moon the machine has six times the force
 *   margin it has on Earth.
 *
 *   SPEED. The same joints are rated at 20 rad/s, and the leg extends by
 *   |dL/dknee| metres per radian — which goes to ZERO as the leg straightens.
 *   So the faster the robot is already moving, the less extension it has left
 *   to accelerate through.
 *
 * Simulated against the real chain, the SPEED limit is what binds off Earth,
 * and it does not care about gravity at all — so takeoff speed comes out
 * within 3 % of the same value on all three bodies and everything else is the
 * field:
 *
 *              takeoff      apex     hang    duty    bound by
 *     Earth    1.90 m/s   0.183 m   0.39 s   0.59    knee torque
 *     Mars     1.94 m/s   0.508 m   1.05 s   0.34    knee speed
 *     Moon     1.96 m/s   1.181 m   2.41 s   0.18    knee speed
 *
 * Earth is the exception, and it is the interesting one: at 1 g the machine
 * spends its whole push fighting its own weight and runs out of TORQUE before
 * it runs out of joint speed. Step off Earth and the force margin stops
 * mattering, the speed ceiling takes over, and the push becomes a constant of
 * the hardware.
 *
 * A G1 is 1.32 m tall. On the Moon it clears very nearly its own height and
 * stays up for two and a half seconds. Neither figure is typed anywhere: they
 * fall out of `effort`, `velocity` and the link geometry in g1_29dof.urdf, and
 * change if the URDF does.
 *
 * That is also why the apex ratio is almost exactly the gravity ratio.
 * Moon:Mars apex is 2.32 against a gravity ratio of 3.721/1.625 = 2.29. It is
 * not 2.29 exactly because the small force-limited part of the push does
 * respond to g — and that residual is the honest signature of a real machine
 * rather than a projectile.
 *
 * WHAT THIS MODULE IS AND IS NOT
 *
 * It is the physics and the timing: how hard the robot can push, how long it
 * is in the air, and the phase structure of a hop, a lope stride and a trip.
 * It knows nothing about terrain or about joints. tools/build_motions.mjs
 * takes these and solves the legs against a real DEM, the same way
 * tools/retarget.mjs does for the packets.
 */
import { legFK } from './G1Kinematics.js';

// ---------------------------------------------------------------------------
// Hardware, straight out of public/robots/g1/g1_29dof.urdf.
// ---------------------------------------------------------------------------
/** Knee effort limit, N m. One knee. */
export const KNEE_TORQUE = 139;
/** Knee velocity limit, rad/s. */
export const KNEE_RATE = 20;
/** Total mass of the 29-DoF tree, kg — summed from G1_TREE's link masses. */
export const G1_MASS = 35.1;

/**
 * Fraction of the rated knee speed a jump is allowed to use.
 *
 * Butterfly.js holds its stroke to 25 % of rated because it is a continuous
 * cyclic motion with an acceleration budget to respect. A push-off is a single
 * transient at the top of the envelope, so it gets more — but not all of it,
 * because a rated limit is a limit and not a target, and because the recovery
 * has to be able to catch what the push throws.
 */
export const PUSH_RATE_FRACTION = 0.6;

/** Knee angle at the bottom of the crouch, and at the moment of leaving it. */
const CROUCH_KNEE = 1.35;
const EXTEND_KNEE = 0.10;

/**
 * Distance from hip to ankle at a given knee angle, metres.
 *
 * The hip and ankle pitch each take half the knee, which is what keeps the
 * shank under the body through the fold instead of swinging the foot out from
 * under it. Measured through legFK rather than from a two-link formula,
 * because the G1's thigh and shank are neither collinear nor equal.
 */
export function legLength(knee) {
  const q = [-knee / 2, 0, 0, knee, -knee / 2, 0];
  const { p } = legFK('left', q);
  return Math.hypot(p[0], p[1], p[2]);
}

const dLegLength = (knee) => Math.abs((legLength(knee + 0.005) - legLength(knee - 0.005)) / 0.01);

/**
 * How fast this machine can leave the ground in a field of strength `g`.
 *
 * Integrated up the extension rather than assumed, because the two limits bind
 * at opposite ends of it. Through the crouch the leg is folded: the moment arm
 * is long, so the force available is small, but the extension per radian is
 * large, so the speed ceiling is high. Near full extension it is the reverse.
 * The robot leaves the ground at the crossover, which is BEFORE the leg is
 * straight — as a real jumper does, and as a purely kinematic model of leg
 * extension cannot reproduce, since that model has the extension rate falling
 * to zero exactly when the robot is supposed to be fastest.
 *
 * @param {number} g   field strength, m/s^2
 * @returns {{v0:number, apex:number, hang:number, crouch:number,
 *            boundBy:'knee speed'|'knee torque'}}
 */
export function pushOff(g, opt = {}) {
  const rate = KNEE_RATE * (opt.rateFraction ?? PUSH_RATE_FRACTION);
  const mass = opt.mass ?? G1_MASS;
  const N = 400;
  let v2 = 0, best = 0, bestCapped = false;

  for (let i = 0; i < N; i++) {
    const k0 = CROUCH_KNEE + (EXTEND_KNEE - CROUCH_KNEE) * (i / N);
    const k1 = CROUCH_KNEE + (EXTEND_KNEE - CROUCH_KNEE) * ((i + 1) / N);
    const dl = Math.abs(legLength(k1) - legLength(k0));
    const arm = Math.max(dLegLength(k0), 1e-4);
    // Vertical force from knee torque, by power balance: F * dL = tau * dknee,
    // so F = tau / |dL/dknee|. Two legs push together.
    const a = (2 * KNEE_TORQUE / arm) / mass - g;
    if (a > 0) v2 += 2 * a * dl;
    // and it can never be moving faster than the joint can extend.
    const cap = arm * rate;
    const capped = v2 > cap * cap;
    if (capped) v2 = cap * cap;
    const v = Math.sqrt(Math.max(v2, 0));
    if (v > best) { best = v; bestCapped = capped; }
  }

  return {
    v0: best,
    apex: (best * best) / (2 * g),
    hang: (2 * best) / g,
    crouch: legLength(EXTEND_KNEE) - legLength(CROUCH_KNEE),
    boundBy: bestCapped ? 'knee speed' : 'knee torque',
  };
}

/**
 * The phase structure of one hop, in seconds.
 *
 * Only `flight` scales with gravity. The crouch and the push are the machine
 * moving its own legs, which one sixth g does not make faster, and the landing
 * absorption is set by how much momentum has to be taken out — which is the
 * SAME on every body, because the robot lands at the speed it left at. That
 * last point is the one people get wrong about low gravity: the fall is slow,
 * but the touchdown is not soft.
 */
export function hopPhases(g, opt = {}) {
  const p = pushOff(g, opt);
  // Extension distance over mean extension speed. The crouch is deliberately
  // slower than the push — dropping into it fast would need the same torque
  // the push does, spent achieving nothing.
  const push = p.crouch / Math.max(p.v0 * 0.5, 0.05);
  return {
    ...p,
    crouchTime: push * 1.8,
    pushTime: push,
    flightTime: p.hang,
    // Absorbing v0 over the same extension the push used.
    landTime: p.crouch / Math.max(p.v0 * 0.5, 0.05),
    get cycle() { return this.crouchTime + this.pushTime + this.flightTime + this.landTime; },
  };
}

/**
 * How far a hop travels, given a forward speed carried through the flight.
 *
 * Forward speed is NOT free at low gravity: with little weight on the feet
 * there is little friction to push against, so the horizontal impulse a stance
 * phase can deliver is bounded by mu * m * g * t_stance. This is the reason
 * the Apollo crews loped instead of running — not because they could not move
 * their legs quickly, but because there was nothing to push against.
 *
 * @param {number} g
 * @param {number} mu   coefficient of friction; lunar regolith on a smooth
 *                      sole is about 0.4, Martian sand a little more.
 */
export function strideFor(g, mu = 0.45, opt = {}) {
  const p = hopPhases(g, opt);
  const stance = p.crouchTime + p.pushTime + p.landTime;
  // Impulse available while there is weight on the feet.
  const vForward = mu * g * stance;
  return {
    ...p,
    speedCeiling: vForward,
    stanceTime: stance,
    range: vForward * p.flightTime,
    dutyFactor: stance / (stance + p.flightTime),
  };
}

/**
 * The three motions, as phase plans.
 *
 * Each returns a normalised description the builder turns into frames. They
 * are deliberately not poses: a pose depends on the terrain under the foot,
 * and that belongs to whatever has a SiteField in hand.
 */
export const MOTIONS = {
  /**
   * LOPE — the Apollo gait. Alternating feet, and airborne between every one.
   *
   * The crews converged on this within minutes of stepping out, on every
   * mission, without being trained to. It is what a walk becomes when the
   * flight phase gets long enough to be worth using: instead of the walk's
   * double support, each foot is a single push that throws the body at the
   * next one.
   */
  lope: {
    id: 'lope', label: 'LOPE',
    blurb: 'The gait the Apollo crews adopted within minutes on every mission. '
         + 'One foot at a time, airborne between every step — a walk whose '
         + 'double support has been replaced by flight.',
    feet: 'alternating', pushFraction: 0.62, armSwing: 0.55, lean: 0.14,
  },
  /**
   * BOUND — both feet together, straight up the way John Young saluted.
   *
   * The most direct reading of the field there is: nothing about the machine
   * changes between bodies, so the entire difference in height and hang time
   * is gravity.
   */
  bound: {
    id: 'bound', label: 'BOUND',
    blurb: 'Both feet together, everything into the vertical. The robot is '
         + 'identical on all three bodies and pushes identically, so the whole '
         + 'difference in height and hang is the field.',
    feet: 'together', pushFraction: 1.0, armSwing: 0.9, lean: 0.02,
  },
  /**
   * TRIP — catch a toe, and find out how long you have to do something.
   *
   * The interesting part of a low-gravity fall is not that it is slow, it is
   * how much TIME it buys. Toppling is a pendulum about the toe, so the time
   * to rotate through a given angle goes as sqrt(1/g): the same stumble that
   * puts a robot on the ground in 0.93 s on Earth takes 2.28 s on the Moon.
   * That is well over a second of extra warning for a controller that knows what
   * to do with it — which is why the Apollo crews could nearly always get a
   * hand or a foot down, and why they fell so gracefully when they could not.
   */
  trip: {
    id: 'trip', label: 'TRIP + RECOVER',
    blurb: 'A caught toe, and the fall that follows. Toppling is a pendulum '
         + 'about the toe, so the time to go over scales as 1/sqrt(g): 0.93 s '
         + 'on Earth against 2.28 s on the Moon. That extra second and a third '
         + 'is what the Apollo crews used to get a hand down.',
    feet: 'alternating', pushFraction: 0.5, armSwing: 1.0, lean: 0.0,
    catchAt: 0.42,        // fraction of the clip where the toe catches
  },
};

/**
 * Time for the body to topple through `angle` about a planted toe.
 *
 * An inverted pendulum released from near-vertical: t = sqrt(L/g)*acosh(...)
 * for the linearised case. Using the linear form is honest here because the
 * interesting part is the first 30-40 degrees, where it is accurate, and it
 * makes the sqrt(1/g) scaling explicit rather than hiding it in a solver.
 *
 * @param {number} g
 * @param {number} angle   radians from vertical
 * @param {number} L       height of the COM above the pivot, metres
 */
export function toppleTime(g, angle = 0.6, L = 0.62, theta0 = 0.03) {
  const w = Math.sqrt(g / L);
  return Math.acosh(angle / theta0) / w;
}
