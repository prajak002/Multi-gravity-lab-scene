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
/** Knee effort limit, N m. One knee. Straight out of the URDF. */
export const KNEE_TORQUE = 139;
/** Knee velocity limit, rad/s. */
export const KNEE_RATE = 20;

// Mass and inertia, summed over G1_TREE's own link masses and inertia tensors
// with the parallel-axis theorem, in the neutral pose. Not typed: see
// tools/check_inertia.mjs, which recomputes them and fails if they drift.
/** Total mass of the 29-DoF tree, kg. */
export const G1_MASS = 35.12;
/** Whole-body inertia about the pitch axis through the COM, kg m^2. */
export const BODY_INERTIA = 3.624;
/** Both arms, about the same axis. 13.8 % of the body — this is the number
 *  that decides how much attitude a windmill can actually buy. */
export const ARM_INERTIA = 0.502;
/** Arm inertia when tucked, as a fraction of extended. A sweep out and a
 *  return tucked is what turns a reciprocating motion into net rotation. */
export const ARM_TUCK = 0.30;
/** Usable shoulder sweep in one stroke, rad. */
export const ARM_SWEEP = 2.6;
/** Windmill rate the arms can sustain, Hz. */
export const ARM_RATE = 1.1;
/** Half-length of the sole's support polygon, m — FOOT_CONTACTS heel to toe. */
export const FOOT_HALF = 0.085;

/**
 * How far the thrust line misses the centre of mass, metres.
 *
 * No push is perfectly through the COM: the two legs never produce identical
 * force, the feet are never exactly level, and the body is never exactly
 * upright at the moment it leaves. Four millimetres on a 1.32 m machine is a
 * small, ordinary misalignment — a third of a percent of body height — and on
 * Earth it is completely unremarkable.
 *
 * It is the single input that makes the instability model say anything, so it
 * is stated here rather than buried, and the whole result is linear in it: a
 * robot with twice the misalignment is twice as unstable everywhere, and the
 * RATIO between the bodies does not move at all. That ratio is the finding.
 */
export const THRUST_OFFSET = 0.004;

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
 * THRUST is the input, and that is the whole point of the model. Two knees at
 * `thrust` times the URDF's own 139 N m produce a vertical force through the
 * leg's Jacobian, and what the robot gets out of it is the NET acceleration:
 *
 *     a = F/m - g
 *
 * Subtracting `g` is where gravity enters, and it enters twice. On the Moon the
 * same thrust has five sixths less weight to fight, so the machine accelerates
 * harder and leaves FASTER; and then the slower field turns that faster
 * take-off into a much higher apex. The two compound:
 *
 *     apex = v0^2 / 2g,   with v0 itself rising as g falls
 *
 * which is why the same push that clears 0.33 m on Earth clears 2.20 m on the
 * Moon — a factor of 6.7, not the 6.0 the gravity ratio alone would give.
 *
 * The speed ceiling is still enforced and still real: the leg extends by
 * |dL/dknee| metres per radian, which goes to ZERO as the leg straightens, so
 * the robot can never be moving faster than its knee can extend. What that
 * ceiling does under high thrust is interesting rather than limiting — the
 * energy curve meets it EARLIER, at a more folded knee where the ceiling is
 * higher, so the machine takes off before the leg is straight. Which is what a
 * real jumper does, and what a purely kinematic model of leg extension cannot
 * reproduce, since that model has the extension rate falling to zero exactly
 * when the robot is supposed to be fastest.
 *
 * @param {number} g              field strength, m/s^2
 * @param {object} [opt]          { thrust, rateFraction, mass }
 */
export function pushOff(g, opt = {}) {
  const thrust = opt.thrust ?? 1;
  const rate = KNEE_RATE * (opt.rateFraction ?? 1);
  const mass = opt.mass ?? G1_MASS;
  const N = 400;
  let v2 = 0, best = 0, bestCapped = false, fSum = 0;

  for (let i = 0; i < N; i++) {
    const k0 = CROUCH_KNEE + (EXTEND_KNEE - CROUCH_KNEE) * (i / N);
    const k1 = CROUCH_KNEE + (EXTEND_KNEE - CROUCH_KNEE) * ((i + 1) / N);
    const dl = Math.abs(legLength(k1) - legLength(k0));
    const arm = Math.max(dLegLength(k0), 1e-4);
    // Vertical force from knee torque, by power balance: F * dL = tau * dknee,
    // so F = tau / |dL/dknee|. Two legs push together.
    const F = thrust * 2 * KNEE_TORQUE / arm;
    fSum += F;
    const a = F / mass - g;
    if (a > 0) v2 += 2 * a * dl;
    // and it can never be moving faster than the joint can extend.
    const cap = arm * rate;
    const capped = v2 > cap * cap;
    if (capped) v2 = cap * cap;
    const v = Math.sqrt(Math.max(v2, 0));
    if (v > best) { best = v; bestCapped = capped; }
  }

  const crouch = legLength(EXTEND_KNEE) - legLength(CROUCH_KNEE);
  return {
    v0: best,
    apex: g > 0 ? (best * best) / (2 * g) : Infinity,
    hang: g > 0 ? (2 * best) / g : Infinity,
    crouch,
    thrust,
    /** Mean vertical force through the push, N — what the tumble is driven by. */
    force: fSum / N,
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
    mu,
  };
}

/**
 * WHY THRUST DESTABILISES A ROBOT, AND WHY IT GETS WORSE AS GRAVITY FALLS
 *
 * The jump model above says a bigger push in a weaker field goes higher. It
 * does not say whether the machine is still upright when it lands, and that is
 * the question the Apollo film actually answers: the crews fell over
 * constantly, and not because they were clumsy.
 *
 * Three quantities, and gravity is in all of them:
 *
 * 1. WHAT THE PUSH DOES TO ATTITUDE. The thrust line misses the COM by some
 *    small offset `e`, so the push applies a torque F*e for its duration. The
 *    angular impulse is
 *
 *        H = F * e * t_push,        omega = H / I_body
 *
 *    and `omega` barely depends on g at all — it is set by the machine.
 *
 * 2. WHAT FLIGHT DOES WITH IT. In free flight there is no external torque, so
 *    that rate is CONSERVED and simply integrates:
 *
 *        theta_flight = omega * t_flight,     t_flight = 2 v0 / g
 *
 *    Gravity enters here as 1/g. The same 4 mm misalignment that tips the body
 *    9 degrees on Earth tips it 68 on the Moon, because the Moon gives it four
 *    times as long to act and a faster take-off to act on.
 *
 * 3. WHAT THE ROBOT CAN DO ABOUT IT. Two things, and they pull opposite ways.
 *
 *    In the air, the ARMS. Angular momentum is conserved, so swinging them
 *    counter-rotates the body by (I_arm/I_body) * sweep — 13.8 % of the sweep,
 *    from the URDF's own inertia tensors. A stroke out with the arms extended
 *    and back with them tucked nets most of that, and a longer flight allows
 *    more strokes, so this authority GROWS as gravity falls. It is exactly the
 *    windmilling in the Apollo film, and it is the only attitude control a body
 *    in free flight has.
 *
 *    On the ground, the ANKLE — and here is the trap. The usable ankle torque
 *    is not the actuator's 139 N m; it is whatever keeps the centre of pressure
 *    inside the sole, which is
 *
 *        tau_max = m * g * d_foot
 *
 *    That is a GRAVITATIONAL limit, not a mechanical one. At one sixth g the
 *    robot has one sixth the authority to correct its attitude, however strong
 *    its motors are, because leaning on the ankle any harder just tips the foot
 *    off its edge.
 *
 * Put together, the residual attitude error the ankle has to absorb grows as
 * 1/g while the authority to absorb it falls as g, so the instability index
 *
 *        S = (theta_flight - theta_arms) / (correctable in stance)
 *
 * goes as roughly 1/g^2. Earth 0.5, Mars 5, Moon 27 on the default thrust:
 * fifty times less stable on the Moon than on Earth, from one small
 * misalignment that is harmless at 1 g.
 *
 * S < 1 means the stance can absorb what the flight built up. S > 1 means it
 * cannot, and the robot lands already committed to falling — which is what the
 * generated motions then do.
 */
export function instability(g, opt = {}) {
  const phys = strideFor(g, opt.mu ?? 0.45, opt);
  const e = opt.offset ?? THRUST_OFFSET;
  const I = opt.inertia ?? BODY_INERTIA;

  // 1. what the push does
  const angularImpulse = phys.force * e * phys.pushTime;
  const omega = angularImpulse / I;

  // 2. what flight does with it
  const tumble = omega * phys.flightTime;

  // 3a. what the arms can take back, by conservation
  const strokes = Math.max(1, (opt.armRate ?? ARM_RATE) * phys.flightTime);
  const armAuthority = strokes * (1 - ARM_TUCK) * (ARM_INERTIA / I) * ARM_SWEEP;
  const residual = Math.max(0, tumble - armAuthority);

  // 3b. what the ankle can take back, bounded by tipping and not by torque
  const alpha = (G1_MASS * g * FOOT_HALF) / I;
  const correctable = (alpha * phys.stanceTime * phys.stanceTime) / 4;

  // And separately: can the robot even PUT A FOOT where it would need to?
  // The capture point is where the foot must land to arrest a velocity v about
  // a pendulum of length L, and it runs away as 1/sqrt(g) while the leg does
  // not get any longer.
  const L = opt.comHeight ?? 0.62;
  const capture = phys.speedCeiling * Math.sqrt(L / Math.max(g, 1e-6));
  const reach = Math.sqrt(Math.max(legLength(0.5) ** 2 - L ** 2, 0.0025));

  return {
    ...phys,
    offset: e,
    omega,
    tumble,
    armAuthority,
    residual,
    correctable,
    index: correctable > 0 ? residual / correctable : Infinity,
    stable: correctable > 0 && residual <= correctable,
    capture,
    reach,
    canCapture: reach >= capture,
    ankleTorqueLimit: G1_MASS * g * FOOT_HALF,
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
