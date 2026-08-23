/**
 * Gravity-conditioned gait.
 *
 * This is a trajectory generator, not a clip player. Nothing here is keyframed
 * per environment: cadence and step length fall out of the gravity field, so
 * the same call produces a brisk Earth walk and a slow lunar bound because g
 * changed and for no other reason.
 *
 *   step period   T = 2*pi*sqrt(L/g) / 4        pendulum quarter-swing of a leg
 *                                               of length L about the hip
 *   capture point x = v * sqrt(L/g)             where the foot must land to
 *                                               arrest velocity v (LIPM)
 *
 * At one sixth g the pendulum is 2.46x slower and the capture point 2.46x
 * further, which is the whole visual difference between the Moon and Earth.
 *
 * Retargeted motion, once the GEM-X -> MuJoCo pipeline lands, replaces
 * evaluate() with a sampled trajectory; the interface is deliberately the same
 * so the app does not care which is driving.
 */
import { G_EARTH } from '../render/Environments.js';

const TAU = Math.PI * 2;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/**
 * The motions on offer.
 *
 * These are gait TEMPLATES, not clips: each one sets duty factor, stride and
 * limb amplitude, and gravity then acts on those. So "run on the Moon" is not
 * a separate animation from "run on Earth" — it is the same template with the
 * pendulum slowed 2.46x and the flight phase stretched to match.
 */
export const MOTIONS = [
  { id: 'walk',  name: 'Walk',  blurb: 'Nominal gait. A foot is down most of the cycle.' },
  { id: 'run',   name: 'Run',   blurb: 'Shorter contact, a real flight phase, deeper lean.' },
  { id: 'climb', name: 'Climb', blurb: 'High knee lift and a forward reach, as up a slope.' },
  { id: 'swim',  name: 'Swim',  blurb: 'Front crawl. Free fall only \u2014 nothing to push against but yourself.', issOnly: true },
];

/**
 * Per-motion shaping. dutyBias moves the stance fraction, strideScale the step
 * length, lift the swing-foot clearance, lean the trunk pitch.
 */
const TEMPLATE = {
  walk:  { dutyBias: 0.00, strideScale: 1.00, lift: 1.00, lean: 0.06, armAmp: 1.00 },
  run:   { dutyBias: -0.17, strideScale: 1.55, lift: 1.45, lean: 0.26, armAmp: 1.70 },
  climb: { dutyBias: 0.10, strideScale: 0.62, lift: 2.10, lean: 0.30, armAmp: 1.25 },
};

export class Gait {
  /**
   * @param {object} def  robot definition from Robots.js
   * @param {number} legLength  hip height in metres, measured from the model
   */
  constructor(def, legLength, mode = 'walk') {
    this.def = def;
    this.L = Math.max(0.25, legLength);
    this.phase = 0;
    this.distance = 0;
    this.mode = mode;
  }

  setMode(mode) { this.mode = mode; }

  /** Pendulum period of the swing leg in this field. */
  stepPeriod(g) {
    if (g <= 1e-4) return 2.4;                    // free fall: no pendulum at all
    return clamp((TAU * Math.sqrt(this.L / g)) / 4, 0.22, 2.2);
  }

  /** Where the foot must land to arrest v — the LIPM capture point. */
  capturePoint(g, v) {
    if (g <= 1e-4) return 0;
    return v * Math.sqrt(this.L / g);
  }

  /** Comfortable cruise speed: a stride per period, stride set by leg length. */
  cruiseSpeed(g) {
    if (this.mode === 'swim') return 0.55;         // a glide, not a stride
    const t = TEMPLATE[this.mode] || TEMPLATE.walk;
    const strideScale = clamp(Math.sqrt(G_EARTH / Math.max(g, 0.35)), 1, 2.6);
    return (this.L * 0.92 * strideScale * t.strideScale) / this.stepPeriod(g);
  }

  advance(dt, g) {
    const T = this.stepPeriod(g);
    this.phase = (this.phase + dt / (T * 2)) % 1;   // one full cycle = two steps
    const v = this.cruiseSpeed(g);
    this.distance += v * dt;
    return { T, v, phase: this.phase };
  }

  /**
   * Joint angles for the current phase.
   * @returns {{joints: Object<string, number>, bodyY: number, pitch: number, airborne: boolean}}
   */
  evaluate(g) {
    if (this.mode === 'swim') return this._swim(g);
    return this.def.kind === 'quadruped' ? this._quad(g) : this._biped(g);
  }

  /**
   * Free-fall motion.
   *
   * With no contact there is nothing to push against, so every limb sweep is
   * reaction only: the trunk counter-rotates against the arms and the body
   * translates smoothly rather than in the bursts a footfall produces. It is
   * the one motion here with no duty factor, because there is no stance.
   */
  _swim(g) {
    const p = this.phase;
    const joints = {};

    if (this.def.kind === 'quadruped') {
      // A quadruped has no stroke to imitate; all four paddle out of phase.
      let i = 0;
      for (const J of Object.values(this.def.quad)) {
        const ph = Math.sin((p + i * 0.25) * TAU);
        joints[J.abduct] = ph * 0.22;
        joints[J.thigh] = 0.45 + ph * 0.55;
        joints[J.calf] = -1.1 + ph * 0.45;
        i++;
      }
      return { joints, bodyY: 0, pitch: Math.sin(p * TAU) * 0.12, airborne: true, duty: 0, contacts: [] };
    }

    // ---- front crawl, by the four coaching phases ------------------------
    // Catch -> Pull -> Push -> Recovery, per arena's freestyle stroke guide.
    // The two arms run half a cycle apart, so one is always propelling while
    // the other resets.
    //
    //   CATCH     elbow HIGH, forearm presses down, palm down, wrist above
    //             the fingers. The elbow does not drop — that is the single
    //             most common fault the guide calls out.
    //   PULL      the hand travels UNDER the body rather than alongside it,
    //             so the arm adducts toward the centreline; palm finishes up.
    //   PUSH      backwards past the hip until the hand exits.
    //   RECOVERY  elbow lifts first, hand comes forward at shoulder height,
    //             then extends out in front to enter.
    const CATCH = 0.12, PULL = 0.32, PUSH = 0.50;   // fractions of the cycle
    const arm = (t) => {
      const ph = ((t % 1) + 1) % 1;
      if (ph < CATCH) {
        const u = ph / CATCH;
        return {
          pitch: -2.30 + 0.55 * u,        // extended ahead, beginning to press
          roll: 0.34 - 0.10 * u,          // held wide of the head
          elbow: -0.15 - 0.55 * u,        // elbow bends EARLY: the high elbow
        };
      }
      if (ph < PULL) {
        const u = (ph - CATCH) / (PULL - CATCH);
        return {
          pitch: -1.75 + 1.55 * u,
          roll: 0.24 - 0.30 * u,          // sweeps inward, under the body
          elbow: -0.70 - 0.45 * u,        // deepest bend at mid-pull
        };
      }
      if (ph < PUSH) {
        const u = (ph - PULL) / (PUSH - PULL);
        return {
          pitch: -0.20 + 1.30 * u,        // drives back past the hip
          roll: -0.06 + 0.16 * u,
          elbow: -1.15 + 1.00 * u,        // straightens as it pushes through
        };
      }
      const u = (ph - PUSH) / (1 - PUSH);
      // Recovery: the elbow leads, so bend peaks early and the arm only
      // straightens again as the hand reaches forward to enter.
      const lead = Math.sin(Math.PI * Math.min(1, u * 1.35));
      return {
        pitch: 1.10 - 3.40 * u,
        roll: 0.30 + 0.62 * lead,         // elbow carried high and wide
        elbow: -0.20 - 1.15 * lead,
      };
    };

    const A = this.def.arms;
    if (A) {
      const L = arm(p), R = arm(p + 0.5);
      joints[A.left.shoulderPitch] = L.pitch;
      joints[A.left.shoulderRoll] = L.roll;
      joints[A.left.elbow] = L.elbow;
      joints[A.right.shoulderPitch] = R.pitch;
      joints[A.right.shoulderRoll] = -R.roll;
      joints[A.right.elbow] = -R.elbow;
    }

    // Flutter kick: small, fast, alternating — six beats per arm cycle, which
    // is the usual crawl timing.
    const beat = Math.sin(p * TAU * 3);
    for (const side of ['left', 'right']) {
      const J = this.def.legs[side];
      const s = side === 'left' ? 1 : -1;
      const k = beat * s;
      joints[J.hipPitch] = -0.10 + k * 0.30;
      joints[J.hipRoll] = s * 0.03;
      joints[J.knee] = 0.10 + Math.max(0, -k) * 0.55;   // knee bends on the upbeat only
      joints[J.anklePitch] = -0.35 - k * 0.15;           // toes pointed, as they must be
    }

    // The body rolls TOWARD the arm that is pulling — that is what lets the
    // pull happen under the centreline, and what stops a crawl reading as
    // flailing. The guide's other posture note is head down, hips high, which
    // is the small nose-down trunk pitch below.
    const roll = Math.sin(p * TAU) * 0.34;
    if (this.def.waistYaw) joints[this.def.waistYaw] = roll * 0.45;

    return { joints, bodyY: 0, roll, pitch: 0.05, airborne: true, duty: 0, contacts: [] };
  }

  _biped(g) {
    const p = this.phase;
    const joints = {};
    // Duty factor: the fraction of the cycle a foot is down. On Earth a walk
    // keeps a foot down ~60% of the time; as g falls the flight phase grows
    // until both feet leave together, which is the lunar bound.
    const t = TEMPLATE[this.mode] || TEMPLATE.walk;
    const duty = clamp(0.34 + 0.30 * (g / G_EARTH) + t.dutyBias, 0.12, 0.70);
    const flight = Math.max(0.05, 1 - duty);

    const leg = (side, ph) => {
      const swinging = ph > duty;
      const s = swinging ? (ph - duty) / flight : ph / duty;
      const J = this.def.legs[side];
      let hip, knee, ankle;
      if (swinging) {
        // Swing: fold the knee to clear the ground, then EXTEND it again
        // before touchdown. The extension is the part that was missing —
        // without it the leg arrives still folded and the robot looks like it
        // is hovering rather than reaching for the floor.
        const lift = Math.sin(Math.PI * s);
        hip = -0.30 + 0.92 * s * t.strideScale;
        knee = 0.14 + 1.30 * t.lift * lift ** 1.5;
        ankle = -0.18 + 0.30 * lift;
      } else {
        // Stance: the leg is nearly straight and sweeps back under the body,
        // which is what actually carries the robot forward.
        hip = (0.46 - 0.86 * s) * t.strideScale;
        knee = 0.10 + 0.16 * Math.sin(Math.PI * s);
        ankle = -0.06 - 0.20 * s;
      }
      joints[J.hipPitch] = hip;
      joints[J.hipRoll] = side === 'left' ? 0.035 : -0.035;
      joints[J.knee] = knee;
      joints[J.anklePitch] = ankle;
      return { swinging, s };
    };

    const l = leg('left', p);
    const r = leg('right', (p + 0.5) % 1);

    // Arms counter-rotate against the legs — what an audience reads as "a body
    // balancing" rather than "a mannequin translating". Only where they exist.
    const swing = Math.sin(p * TAU);
    const armAmp = clamp((0.55 * (g / G_EARTH) + 0.18) * t.armAmp, 0.18, 1.05);
    const A = this.def.arms;
    if (A) {
      joints[A.left.shoulderPitch] = -swing * armAmp;
      joints[A.right.shoulderPitch] = swing * armAmp;
      joints[A.left.shoulderRoll] = 0.16;
      joints[A.right.shoulderRoll] = -0.16;
      joints[A.left.elbow] = -0.30 - 0.16 * Math.abs(swing);
      joints[A.right.elbow] = 0.30 + 0.16 * Math.abs(swing);
    }

    // Airborne only when BOTH feet are swinging — on Earth that never happens
    // at a walk, and on the Moon it is most of the cycle.
    const airborne = l.swinging && r.swinging;
    const hopHeight = clamp(0.05 * (G_EARTH / Math.max(g, 0.6)), 0.02, 0.34);
    // Ballistic arc across the flight phase rather than a sine on the whole
    // cycle: the body should rise and fall ONCE per flight, and sit still
    // otherwise.
    const flightT = airborne ? Math.min(l.s, r.s) : 0;
    const bodyY = airborne ? Math.sin(Math.PI * clamp(flightT, 0, 1)) * hopHeight : 0;
    // contacts[] is ordered to match def.feet, so a landing maps to the right foot
    return { joints, bodyY, pitch: t.lean + 0.05 * (1 - g / G_EARTH), airborne, duty,
             contacts: [l.swinging, r.swinging] };
  }

  _quad(g) {
    const p = this.phase;
    const joints = {};
    const contacts = [];
    // Trot: diagonal pairs move together. Four contacts means low gravity
    // costs a quadruped far less stability than it costs a biped.
    const t = TEMPLATE[this.mode] || TEMPLATE.walk;
    const duty = clamp(0.36 + 0.26 * (g / G_EARTH) + t.dutyBias, 0.16, 0.68);
    const offsets = { FL: 0, RR: 0, FR: 0.5, RL: 0.5 };
    for (const [legName, J] of Object.entries(this.def.quad)) {
      const ph = (p + offsets[legName]) % 1;
      const swinging = ph > duty;
      contacts.push(swinging);
      const s = swinging ? (ph - duty) / (1 - duty) : ph / duty;
      const sw = Math.sin(Math.PI * s);
      const thigh = swinging ? 0.78 - 0.80 * sw * t.lift : 0.62 + 0.34 * s * t.strideScale;
      const calf = swinging ? -1.42 + 0.62 * sw * t.lift : -1.56 - 0.22 * s;
      joints[J.abduct] = legName[1] === 'L' ? 0.04 : -0.04;
      joints[J.thigh] = thigh;
      joints[J.calf] = calf;
    }
    const hop = clamp(0.022 * (G_EARTH / Math.max(g, 0.6)), 0.01, 0.14);
    // A trot has no true flight phase at these speeds; the body just cycles
    // twice per stride as each diagonal pair loads.
    return { joints, bodyY: Math.abs(Math.sin(p * TAU * 2)) * hop, pitch: 0, airborne: false, duty, contacts };
  }
}
