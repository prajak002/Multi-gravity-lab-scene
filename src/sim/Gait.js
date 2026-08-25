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
import { Butterfly, DEFAULTS as BF_DEFAULTS } from './Butterfly.js';

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
  { id: 'swim',  name: 'Swim',  blurb: 'Butterfly. Free fall only \u2014 nothing to push against but yourself.', issOnly: true },
];

/**
 * Per-motion shaping. dutyBias moves the stance fraction, strideScale the step
 * length, lift the swing-foot clearance, lean the trunk pitch.
 */
const TEMPLATE = {
  // `froude` is v^2/(g*L) for the mode — the dimensionless speed that makes
  // gaits comparable across body sizes AND gravitational fields. A walk sits
  // well below the ~0.5 walk-run transition; a run sits above it.
  walk:  { froude: 0.25, dutyBias: 0.00, strideScale: 1.00, lift: 1.00, lean: 0.06, armAmp: 1.00 },
  run:   { froude: 0.75, dutyBias: -0.17, strideScale: 1.55, lift: 1.45, lean: 0.26, armAmp: 1.70 },
};

/**
 * The foot roll — which part of the sole is lowest, through the cycle.
 *
 * `left_ankle_pitch_joint` has axis +y in a frame with +x at the toe and +z
 * up, so a POSITIVE angle is plantarflexion: the toe goes down and the heel
 * comes up. The URDF allows -0.87267 (toe up) to +0.5236 (toe down), and
 * everything below stays inside that with margin.
 *
 * The old trajectory ran the ankle the wrong way through stance — it started
 * near flat and dorsiflexed steadily to -0.26, so the toe ROSE while the body
 * passed over the foot. That drives the heel edge down into the ground for the
 * whole of stance and leaves no push-off at all, which is what "the feet go
 * under the ground" looks like from outside.
 *
 * A step is a roll from one end of the sole to the other:
 *
 *   TOE STRIKE   the foot arrives pointed, so the toe pair is the lowest
 *                thing on the robot and touches first
 *   HEEL DOWN    the ankle gives way to flat over the first fifth of stance
 *                and the heel settles onto the ground
 *   ROLL OVER    the shank rotates forward over a planted sole, which IS
 *                dorsiflexion, so the angle goes negative through midstance
 *   TOE OFF      plantarflexion at the end of stance: the heel lifts and the
 *                toe is the last thing in contact
 *   CLEARANCE    the toe comes up hard in early swing so it does not catch,
 *                then points again to be ready for the next toe strike
 */
const TOE_STRIKE = 0.24;    // toe-down at touchdown, rad
const TOE_OFF = 0.44;       // plantarflexion at push-off, rad
const DORSI = -0.26;        // dorsiflexion over midstance, rad
const CLEAR = -0.34;        // toe-up during swing clearance, rad

/** Smoothstep, so no segment boundary shows up as a kink in the ankle rate. */
const ease = (u) => { const c = clamp(u, 0, 1); return c * c * (3 - 2 * c); };

/** Ankle through STANCE, s in [0,1]. Toe strike -> heel down -> toe off. */
export function stanceAnkle(s) {
  if (s < 0.20) return TOE_STRIKE + (0 - TOE_STRIKE) * ease(s / 0.20);
  if (s < 0.70) return 0 + (DORSI - 0) * ease((s - 0.20) / 0.50);
  return DORSI + (TOE_OFF - DORSI) * ease((s - 0.70) / 0.30);
}

/** Ankle through SWING, s in [0,1]. Clear the ground, then point the toe. */
export function swingAnkle(s) {
  if (s < 0.30) return TOE_OFF + (CLEAR - TOE_OFF) * ease(s / 0.30);
  if (s < 0.72) return CLEAR;                          // held up, clearing
  return CLEAR + (TOE_STRIKE - CLEAR) * ease((s - 0.72) / 0.28);
}

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
    // Swim time runs separately from gait phase: the stroke has an amplitude
    // ramp, so it needs a clock that only goes forward, not a phase that wraps.
    this.swimT = 0;
    // Butterfly is written against the G1's joint names and the G1's URDF
    // limits. H1 ships arms that are welded and Go2 has none, so neither can
    // run it and both keep the generic free-fall path below.
    this.butterfly = def.id === 'g1' ? new Butterfly() : null;
  }

  setMode(mode) {
    this.mode = mode;
    if (mode === 'swim' && this.butterfly) { this.swimT = 0; this.butterfly.reset(); }
  }

  /** Latching emergency stop for the stroke. No-op for anything else. */
  estop(reason = 'operator') { this.butterfly?.estop(reason); }
  get estopped() { return !!this.butterfly?.stopped; }

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

  /**
   * Comfortable cruise speed, from the Froude number.
   *
   * This was previously stride/period with the stride scaled by
   * sqrt(g_earth/g). That is wrong in a way that hides itself: the period
   * ALSO goes as sqrt(1/g), so the two cancel exactly and cruise speed came
   * out identical in every field. Three robots started side by side stayed
   * exactly level for the whole traverse, which made gravity look like it did
   * nothing to travel.
   *
   * The right invariant is the Froude number, Fr = v^2 / (g*L). Legged gaits
   * of every size and on every body compare at equal Fr — it is why a walk
   * breaks into a run near Fr = 0.5 for animals from a quail to an elephant.
   * Holding Fr fixed and solving for speed gives
   *
   *     v = sqrt(Fr * g * L)
   *
   * so walking speed falls as sqrt(g): the Moon's is sqrt(1.625/9.807) =
   * 0.407 of Earth's. That is the real Apollo result. The crews could not
   * walk quickly, and the reason they switched to loping is precisely that
   * the walk-run transition speed drops with g.
   *
   * Note what does NOT change: stride = v * T goes as sqrt(g) * sqrt(1/g),
   * which is constant. At equal Froude number the step LENGTH is the same in
   * every field, and the whole difference is in how long each step takes.
   */
  cruiseSpeed(g) {
    if (this.mode === 'swim') return 0.55;         // a glide, not a stride
    const t = TEMPLATE[this.mode] || TEMPLATE.walk;
    return Math.sqrt(t.froude * Math.max(g, 0.05) * this.L);
  }

  advance(dt, g) {
    const T = this.stepPeriod(g);
    this.phase = (this.phase + dt / (T * 2)) % 1;   // one full cycle = two steps
    this.swimT += Math.min(0.05, Math.max(0, dt));
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

    // ---- butterfly ------------------------------------------------------
    // One definition of the stroke, in Butterfly.js, which is also what the
    // dry run and the static check measure. Keeping a second hand-rolled copy
    // here is how the two drift: the copy that used to live in this function
    // negated the RIGHT elbow, so the two arms bent opposite ways through the
    // pull, and drove the LEFT elbow to -1.35 rad against a -1.0472 limit, so
    // one arm silently clamped and the other did not.
    if (this.butterfly) {
      const bf = this.butterfly;
      bf.t = this.swimT;                 // keep the stroke's own clock in step
      Object.assign(joints, bf.pose(this.swimT));
      // The trunk's own pitch follows the same undulation the waist joint is
      // taking, so the whole body reads as one wave rather than a torso that
      // is bending while the body it belongs to holds still.
      const w = joints.waist_pitch_joint ?? 0;
      return { joints, bodyY: 0, roll: 0, pitch: 0.05 + w * 0.70,
               airborne: true, duty: 0, contacts: [],
               stroke: { phase: bf.phaseAt(bf.t), amplitude: bf.amplitudeAt(bf.t),
                         frequency: bf.cfg.frequency, estopped: bf.stopped } };
    }

    // Everything below is the fallback for a description Butterfly cannot
    // drive — H1's arms are welded in the shipped URDF, so there is no stroke
    // to make, only a kick.

    // The defining property, and the reason it is the right stroke to show in
    // free fall: BOTH arms do the same thing at the same time. There is no
    // half-cycle offset and no body roll to sweep one arm under the
    // centreline, so the whole motion is symmetric about the spine — which
    // means every gram of momentum an arm throws forward is matched by the
    // other one, and the only thing left for the body to do is recoil. A
    // crawl hides that behind roll; a butterfly cannot.
    //
    //   ENTRY     hands enter ahead, wide of the shoulders, arms long.
    //   CATCH     elbows stay HIGH while the hands press out and down.
    //   PULL      hands sweep in under the chest — the narrow waist of the
    //             keyhole — with the deepest elbow bend at the middle.
    //   PUSH      hands accelerate back past the hips and exit thumbs-first.
    //   RECOVERY  arms come over the top STRAIGHT and wide, both together.
    //             This is what makes it read as butterfly and not as anything
    //             else, so the elbow stays extended through the whole sweep.
    const ENTRY = 0.10, CATCH = 0.26, PULL = 0.44, PUSH = 0.58;
    const arm = (t) => {
      const ph = ((t % 1) + 1) % 1;
      if (ph < ENTRY) {
        const u = ph / ENTRY;
        return { pitch: -2.55 + 0.15 * u, roll: 0.40 - 0.06 * u, elbow: -0.08 - 0.10 * u };
      }
      if (ph < CATCH) {
        const u = (ph - ENTRY) / (CATCH - ENTRY);
        // press out and down; the elbow bends early and stays above the hand
        return { pitch: -2.40 + 0.75 * u, roll: 0.34 + 0.16 * u, elbow: -0.18 - 0.62 * u };
      }
      if (ph < PULL) {
        const u = (ph - CATCH) / (PULL - CATCH);
        // the keyhole narrows: hands sweep inward under the chest
        return { pitch: -1.65 + 1.30 * u, roll: 0.50 - 0.62 * u, elbow: -0.80 - 0.55 * u };
      }
      if (ph < PUSH) {
        const u = (ph - PULL) / (PUSH - PULL);
        // accelerate back past the hip, arm straightening as it goes
        return { pitch: -0.35 + 1.55 * u, roll: -0.12 + 0.10 * u, elbow: -1.35 + 1.25 * u };
      }
      const u = (ph - PUSH) / (1 - PUSH);
      // Recovery, both arms together, over the top and WIDE, elbows straight.
      const sweep = Math.sin(Math.PI * u);
      return {
        pitch: 1.20 - 3.75 * u,
        roll: -0.02 + 1.05 * sweep,      // carried wide, the butterfly signature
        elbow: -0.10 - 0.12 * sweep,     // stays long; never folds like a crawl
      };
    };

    const A = this.def.arms;
    if (A) {
      // SAME phase for both arms. This is the whole difference from a crawl.
      const S = arm(p);
      joints[A.left.shoulderPitch] = S.pitch;
      joints[A.left.shoulderRoll] = S.roll;
      joints[A.left.elbow] = S.elbow;
      joints[A.right.shoulderPitch] = S.pitch;
      joints[A.right.shoulderRoll] = -S.roll;
      joints[A.right.elbow] = -S.elbow;
    }

    // Dolphin kick: legs together, TWO kicks per arm cycle — one as the hands
    // enter, one as they push out. Alternating a flutter here would be the
    // single most obvious thing wrong with it.
    const kick = Math.sin(p * TAU * 2);
    for (const side of ['left', 'right']) {
      const J = this.def.legs[side];
      const s = side === 'left' ? 1 : -1;
      joints[J.hipPitch] = -0.12 + kick * 0.34;
      joints[J.hipRoll] = s * 0.015;                  // legs held together
      joints[J.knee] = 0.14 + Math.max(0, -kick) * 0.62;   // bends on the up-beat
      joints[J.anklePitch] = -0.38 - kick * 0.18;     // toes pointed
    }

    // Body undulation rather than roll. The chest presses down as the hands
    // catch and the hips rise behind it, which is the wave that carries the
    // dolphin kick — and in free fall it is pure angular-momentum exchange,
    // since there is no water to press against.
    const wave = Math.sin(p * TAU * 2 - 0.9);
    if (this.def.waistPitch) joints[this.def.waistPitch] = wave * 0.20;
    else if (this.def.waistYaw) joints[this.def.waistYaw] = 0;
    if (this.def.waistRoll) joints[this.def.waistRoll] = 0;

    return { joints, bodyY: 0, roll: 0, pitch: 0.05 + wave * 0.14,
             airborne: true, duty: 0, contacts: [] };
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
        ankle = swingAnkle(s);
      } else {
        // Stance: the leg is nearly straight and sweeps back under the body,
        // which is what actually carries the robot forward.
        hip = (0.46 - 0.86 * s) * t.strideScale;
        knee = 0.10 + 0.16 * Math.sin(Math.PI * s);
        ankle = stanceAnkle(s);
      }
      joints[J.hipPitch] = hip;
      joints[J.hipRoll] = side === 'left' ? 0.035 : -0.035;
      joints[J.knee] = knee;
      joints[J.anklePitch] = ankle;
      // Roll is left flat here and taken over by Footing.conformAnkles when
      // there is terrain to conform to; on a flat floor flat is correct.
      if (J.ankleRoll) joints[J.ankleRoll] = 0;
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
