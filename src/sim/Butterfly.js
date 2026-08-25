/**
 * Butterfly — a parameterised butterfly stroke for the Unitree G1.
 *
 * WHAT THIS PROJECT CAN ACTUALLY DRIVE
 * ------------------------------------
 * Before writing this I checked what motion-control interface exists here,
 * rather than assuming a Unitree SDK is present. It is not. The complete set
 * of ways a joint can be commanded in this repository is:
 *
 *   URDFJoint.setJointValue(rad)   urdf-loader, called from src/main.js,
 *                                  src/ui/Arena.js, src/ui/Dashboard.js and
 *                                  src/ui/GravityCompare.js. This moves a
 *                                  three.js Object3D. It is a RENDERING call.
 *   Gait.evaluate(g)               this file's neighbour: a trajectory
 *                                  generator returning {jointName: rad}.
 *   Motion.js                      replays sampled retarget clips through the
 *                                  same interface.
 *   pipeline/track_gravity.py      would step MuJoCo if MuJoCo were installed
 *                                  in .venv. It is not.
 *
 * There is NO unitree_sdk2, no CycloneDDS, no LowCmd/LowState, no ROS, no
 * serial or UDP transport, and no torque or PD-gain interface anywhere in the
 * tree. So this module cannot move a real G1, and nothing here tries to. It
 * emits a joint-position trajectory and it checks that trajectory against the
 * G1's own URDF limits. Getting that onto hardware needs a transport that does
 * not exist yet; see hardwareReadiness() at the bottom for exactly what is
 * missing.
 *
 * THE STROKE
 * ----------
 * Butterfly, not front crawl. The defining property — and the reason it is the
 * right stroke to show in free fall — is that BOTH arms do the same thing at
 * the same time. There is no half-cycle offset and no body roll to sweep one
 * arm under the centreline, so the motion is symmetric about the spine: every
 * bit of momentum one arm throws forward is matched by the other, and all the
 * body can do is recoil. A crawl hides that behind roll; a butterfly cannot.
 *
 *   ENTRY     0.00  hands enter ahead, wide of the shoulders, arms long
 *   CATCH     0.16  elbows stay HIGH while the hands press out and down
 *   PULL      0.34  hands sweep in under the chest, deepest elbow bend
 *   PUSH      0.54  hands accelerate back past the hips, arms straightening
 *   RECOVERY  0.72  both arms over the top together, STRAIGHT and wide
 *
 * The legs do a dolphin kick: together, never alternating, TWO kicks per arm
 * cycle — one as the hands enter and one as they finish the push. An
 * alternating flutter would be the single most obvious thing wrong with it.
 * The waist carries the undulation that links the two.
 */

// Set while maxSafeFrequency() is probing, so the envelope check below does
// not recurse into the probe that answers it.
let _probing = false;

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// ---------------------------------------------------------------------------
// Joint limits, transcribed from public/robots/g1/g1_29dof.urdf.
//
// These are the HARDWARE limits, not the ones this module will use. Every
// command is clamped into SAFE_FRACTION of the range about its midpoint, so a
// bug in the stroke shape cannot walk a joint into its own end stop.
// ---------------------------------------------------------------------------
export const G1_LIMITS = {
  left_shoulder_pitch_joint:  { lo: -3.0892, hi: 2.6704, vel: 37, effort: 25 },
  left_shoulder_roll_joint:   { lo: -1.5882, hi: 2.2515, vel: 37, effort: 25 },
  left_shoulder_yaw_joint:    { lo: -2.6180, hi: 2.6180, vel: 37, effort: 25 },
  left_elbow_joint:           { lo: -1.0472, hi: 2.0944, vel: 37, effort: 25 },
  right_shoulder_pitch_joint: { lo: -3.0892, hi: 2.6704, vel: 37, effort: 25 },
  right_shoulder_roll_joint:  { lo: -2.2515, hi: 1.5882, vel: 37, effort: 25 },
  right_shoulder_yaw_joint:   { lo: -2.6180, hi: 2.6180, vel: 37, effort: 25 },
  right_elbow_joint:          { lo: -1.0472, hi: 2.0944, vel: 37, effort: 25 },
  waist_yaw_joint:            { lo: -2.6180, hi: 2.6180, vel: 32, effort: 88 },
  waist_roll_joint:           { lo: -0.5200, hi: 0.5200, vel: 37, effort: 50 },
  waist_pitch_joint:          { lo: -0.5200, hi: 0.5200, vel: 37, effort: 50 },
  left_hip_pitch_joint:       { lo: -2.5307, hi: 2.8798, vel: 32, effort: 88 },
  left_hip_roll_joint:        { lo: -0.5236, hi: 2.9671, vel: 32, effort: 88 },
  left_knee_joint:            { lo: -0.0873, hi: 2.8798, vel: 20, effort: 139 },
  left_ankle_pitch_joint:     { lo: -0.8727, hi: 0.5236, vel: 37, effort: 50 },
  right_hip_pitch_joint:      { lo: -2.5307, hi: 2.8798, vel: 32, effort: 88 },
  right_hip_roll_joint:       { lo: -2.9671, hi: 0.5236, vel: 32, effort: 88 },
  right_knee_joint:           { lo: -0.0873, hi: 2.8798, vel: 20, effort: 139 },
  right_ankle_pitch_joint:    { lo: -0.8727, hi: 0.5236, vel: 37, effort: 50 },
};

/**
 * The safety envelope. Every one of these is deliberately far inside what the
 * hardware would allow, because the failure mode of being wrong here is a
 * 35 kg humanoid throwing its arms.
 */
export const SAFETY = {
  posFraction: 0.90,   // of each joint's own range, about its midpoint
  velFraction: 0.25,   // of the URDF velocity limit
  accelMax: 12.0,      // rad/s^2, uniform: the URDF declares no accel limit
  torqueFraction: 0.30, // of the URDF effort limit, for whoever adds a PD loop
  maxAmplitude: 1.00,  // of the authored stroke
  maxFrequency: 1.20,  // Hz; maxSafeFrequency() usually binds first
};

export const DEFAULTS = {
  amplitude: 0.35,     // start SMALL. rampSeconds walks up to this from zero.
  // 0.35 Hz, not the 0.45 the stroke can just about hold. maxSafeFrequency()
  // puts the acceleration ceiling at 0.45 Hz for this amplitude, and a default
  // that sits ON its own limit has no margin for a config that nudges the
  // amplitude up. This leaves ~40 % headroom.
  frequency: 0.35,     // Hz, one full arm cycle
  phase: 0.0,          // where in the cycle t = 0 sits
  legPhase: 0.0,       // dolphin kick offset relative to the arms
  waistPhase: -0.143,  // waist leads the kick slightly; -0.9 rad at 2x
  kickRatio: 2,        // kicks per arm cycle. Butterfly is 2. Do not change.
  duration: Infinity,  // seconds, then it holds neutral
  rampSeconds: 6.0,    // amplitude 0 -> amplitude, smoothstepped
  estopSeconds: 0.8,   // decay to neutral once stopped
};

// ---------------------------------------------------------------------------
// The stroke, as keyframes in RADIANS at full amplitude.
//
// Amplitude does not scale these directly — it interpolates from NEUTRAL
// toward them (see pose()), so amplitude 0 is a still, safe, streamlined hold
// and amplitude 1 is the stroke as authored. That is what makes "start very
// small and increase gradually" a single scalar rather than a rewrite.
//
// Keys are [phase, radians] and MUST close: the value at phase 1 equals the
// value at phase 0, or the cycle steps at the wrap.
// ---------------------------------------------------------------------------

/**
 * Neutral: the streamline hold. Arms forward and long, legs together with a
 * little knee bend, toes pointed, trunk straight. Every joint is well inside
 * its limits, so this is also where an emergency stop decays to.
 */
export const NEUTRAL = {
  left_shoulder_pitch_joint: -1.90, right_shoulder_pitch_joint: -1.90,
  left_shoulder_roll_joint: 0.22, right_shoulder_roll_joint: -0.22,
  left_shoulder_yaw_joint: 0.0, right_shoulder_yaw_joint: 0.0,
  left_elbow_joint: 0.18, right_elbow_joint: 0.18,
  waist_yaw_joint: 0.0, waist_roll_joint: 0.0, waist_pitch_joint: 0.0,
  left_hip_pitch_joint: -0.12, right_hip_pitch_joint: -0.12,
  left_hip_roll_joint: 0.015, right_hip_roll_joint: -0.015,
  left_knee_joint: 0.16, right_knee_joint: 0.16,
  left_ankle_pitch_joint: -0.30, right_ankle_pitch_joint: -0.30,
};

/**
 * Arm channels. Both arms are driven from ONE set of keys — that is the whole
 * point of the stroke — and mirrored only where mirroring is geometrically
 * real: shoulder ROLL is about the URDF's x axis and shoulder YAW about z, so
 * those flip sign between left and right. Shoulder PITCH and the ELBOW are
 * about y, the axis the sagittal mirror leaves alone, so they do NOT flip.
 *
 * The previous hand-rolled version negated the right elbow. That is a real
 * error and not a cosmetic one: it bent the two elbows in opposite directions
 * through the pull, which is the one thing a butterfly cannot do. It also
 * drove the left elbow to -1.35 rad against a lower limit of -1.0472, so the
 * left arm silently clamped and the right did not, and the arms came apart.
 */
const ARM = {
  // -1.90 forward/overhead at entry, +1.05 back past the hips at the end of
  // the push, then over the top. Well inside [-3.0892, 2.6704].
  pitch: [[0.00, -1.90], [0.16, -1.62], [0.34, -0.92], [0.54, 1.05],
          [0.72, 0.30], [0.88, -1.28], [1.00, -1.90]],
  // Wide at entry, narrowing through the keyhole, WIDE again over the top.
  // Carrying the recovery wide is the butterfly signature.
  roll: [[0.00, 0.40], [0.16, 0.34], [0.34, 0.12], [0.54, -0.10],
         [0.72, 0.92], [0.88, 0.66], [1.00, 0.40]],
  // Internal rotation through the catch: the elbow stays high and the hand
  // presses outward, which is what "high elbow catch" physically is.
  yaw: [[0.00, 0.00], [0.16, -0.22], [0.34, -0.34], [0.54, -0.10],
        [0.72, 0.18], [0.88, 0.06], [1.00, 0.00]],
  // Positive is flexion. Nearly straight at entry, deepest bend at the middle
  // of the pull, straightening into the push, straight over the top.
  // Peak 1.15 against a limit of 2.0944.
  elbow: [[0.00, 0.12], [0.16, 0.48], [0.34, 1.15], [0.54, 0.26],
          [0.72, 0.10], [0.88, 0.10], [1.00, 0.12]],
};

/**
 * The dolphin kick, at kickRatio x the arm frequency. Legs stay together —
 * hip roll is a fixed few degrees of adduction and never oscillates, because
 * legs that scissor are a flutter kick and a flutter kick is not butterfly.
 */
const LEG = {
  hipPitch: [[0.00, 0.22], [0.25, -0.12], [0.50, -0.46], [0.75, -0.12], [1.00, 0.22]],
  // The knee bends on the UP-beat and straightens to snap the kick down.
  knee: [[0.00, 0.14], [0.25, 0.34], [0.50, 0.76], [0.75, 0.34], [1.00, 0.14]],
  // Toes pointed throughout; more so at the bottom of the downbeat.
  anklePitch: [[0.00, -0.52], [0.25, -0.36], [0.50, -0.20], [0.75, -0.36], [1.00, -0.52]],
};

/** Trunk undulation, also at kickRatio: chest presses down as the hands catch. */
const WAIST = {
  pitch: [[0.00, 0.20], [0.25, 0.00], [0.50, -0.20], [0.75, 0.00], [1.00, 0.20]],
};

// ---------------------------------------------------------------------------
// Smooth periodic interpolation.
//
// Straight lerp between keyframes gives a velocity that jumps at every key,
// which on hardware is a torque spike at every key. Cubic Hermite with
// central-difference tangents is C1 across the whole cycle INCLUDING the wrap,
// so velocity is continuous everywhere and acceleration is bounded.
// ---------------------------------------------------------------------------
function sampleCurve(keys, phase) {
  const p = ((phase % 1) + 1) % 1;
  const n = keys.length - 1;                 // last key duplicates the first
  let i = 0;
  while (i < n - 1 && keys[i + 1][0] <= p) i++;
  const [t0, v0] = keys[i];
  const [t1, v1] = keys[i + 1];
  const h = t1 - t0;
  if (h <= 0) return v0;
  const u = (p - t0) / h;

  // Neighbours, wrapping around the cycle.
  const prev = keys[(i - 1 + n) % n];
  const next = keys[(i + 2) % (n + 1) === 0 ? 1 : Math.min(i + 2, n)];
  const hPrev = t0 - prev[0] + (i === 0 ? 1 : 0);
  const hNext = next[0] - t1 + (i + 2 > n ? 1 : 0);

  // Central-difference tangents, scaled into this segment's parameterisation.
  const m0 = h * 0.5 * ((v0 - prev[1]) / Math.max(1e-6, hPrev) + (v1 - v0) / h);
  const m1 = h * 0.5 * ((v1 - v0) / h + (next[1] - v1) / Math.max(1e-6, hNext));

  const u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * v0 + (u3 - 2 * u2 + u) * m0
       + (-2 * u3 + 3 * u2) * v1 + (u3 - u2) * m1;
}

/** Smoothstep, for the amplitude ramp and the e-stop decay. */
const ease = (u) => { const c = clamp(u, 0, 1); return c * c * (3 - 2 * c); };

// ---------------------------------------------------------------------------

export class Butterfly {
  constructor(cfg = {}) {
    this.cfg = { ...DEFAULTS, ...cfg };
    this.reset();
    this._checkConfig();
  }

  reset() {
    this.t = 0;
    this.stopped = false;
    this.stopT = 0;
    this.stopPose = null;
    this.reason = null;
  }

  /** Refuse a configuration that is outside the envelope, loudly and early. */
  _checkConfig() {
    const c = this.cfg;
    if (!(c.amplitude >= 0 && c.amplitude <= SAFETY.maxAmplitude)) {
      throw new RangeError(`amplitude ${c.amplitude} outside [0, ${SAFETY.maxAmplitude}]`);
    }
    if (!(c.frequency > 0 && c.frequency <= SAFETY.maxFrequency)) {
      throw new RangeError(`frequency ${c.frequency} Hz outside (0, ${SAFETY.maxFrequency}]`);
    }
    if (c.kickRatio !== 2) {
      throw new RangeError(`kickRatio ${c.kickRatio}: butterfly is two kicks per arm cycle`);
    }
    // Amplitude and frequency are only safe TOGETHER: acceleration goes as
    // amplitude x frequency^2, so a pair that is fine at 0.35/0.35 is over the
    // limit at 1.0/0.35. Refusing it here means a caller cannot get an unsafe
    // trajectory by forgetting to call validate().
    if (!_probing) {
      const m = maxSafeFrequency(c.amplitude);
      if (c.frequency > m.hz + 1e-9) {
        throw new RangeError(
          `frequency ${c.frequency} Hz exceeds the ${m.boundBy} limit of ` +
          `${m.hz.toFixed(3)} Hz at amplitude ${c.amplitude}. ` +
          `Lower the frequency, or the amplitude, or both.`);
      }
    }
  }

  /**
   * EMERGENCY STOP. Latches. From here the trajectory decays smoothly from
   * wherever it was to NEUTRAL over estopSeconds and then holds — it does not
   * cut the command, because dropping a position command is how a robot falls
   * over rather than how it stops.
   */
  estop(reason = 'commanded') {
    if (this.stopped) return;
    // Sample BEFORE latching. pose() routes to _decay() the moment `stopped`
    // is set, and _decay() reads stopPose — so latching first froze the decay's
    // start point at null, fell through to NEUTRAL, and made the "smooth ramp
    // to a stop" an instantaneous jump to neutral. On hardware that is the
    // opposite of a safe stop: it is the largest step command in the run,
    // issued at the moment something has already gone wrong.
    this.stopPose = this.pose(this.t);      // freeze the last good command
    this.stopped = true;
    this.reason = reason;
    this.stopT = this.t;
  }

  /** Amplitude actually in force at time t: ramps up, then holds. */
  amplitudeAt(t) {
    const c = this.cfg;
    if (t >= c.duration) return 0;
    const ramp = c.rampSeconds > 0 ? ease(t / c.rampSeconds) : 1;
    return clamp(c.amplitude * ramp, 0, SAFETY.maxAmplitude);
  }

  /** Cycle phase at time t, in [0,1). */
  phaseAt(t) {
    return ((t * this.cfg.frequency + this.cfg.phase) % 1 + 1) % 1;
  }

  /**
   * The commanded pose at time t, in radians, already clamped into the safety
   * envelope. This is the only thing anything outside this file should call.
   */
  pose(t) {
    if (this.stopped) return this._decay(t);
    const amp = this.amplitudeAt(t);
    const p = this.phaseAt(t);
    const c = this.cfg;
    const kick = ((p * c.kickRatio + c.legPhase) % 1 + 1) % 1;
    const wave = ((p * c.kickRatio + c.waistPhase) % 1 + 1) % 1;

    const full = {};
    // --- arms: ONE set of keys, both shoulders, in phase -------------------
    const pitch = sampleCurve(ARM.pitch, p);
    const roll = sampleCurve(ARM.roll, p);
    const yaw = sampleCurve(ARM.yaw, p);
    const elbow = sampleCurve(ARM.elbow, p);
    full.left_shoulder_pitch_joint = pitch;
    full.right_shoulder_pitch_joint = pitch;       // y axis: not mirrored
    full.left_shoulder_roll_joint = roll;
    full.right_shoulder_roll_joint = -roll;        // x axis: mirrored
    full.left_shoulder_yaw_joint = yaw;
    full.right_shoulder_yaw_joint = -yaw;          // z axis: mirrored
    full.left_elbow_joint = elbow;
    full.right_elbow_joint = elbow;                // y axis: not mirrored

    // --- legs: dolphin kick, together, twice per arm cycle ------------------
    const hp = sampleCurve(LEG.hipPitch, kick);
    const kn = sampleCurve(LEG.knee, kick);
    const an = sampleCurve(LEG.anklePitch, kick);
    full.left_hip_pitch_joint = hp;  full.right_hip_pitch_joint = hp;
    full.left_knee_joint = kn;       full.right_knee_joint = kn;
    full.left_ankle_pitch_joint = an; full.right_ankle_pitch_joint = an;
    full.left_hip_roll_joint = 0.015;               // held together, no scissor
    full.right_hip_roll_joint = -0.015;

    // --- trunk -------------------------------------------------------------
    full.waist_pitch_joint = sampleCurve(WAIST.pitch, wave);
    full.waist_roll_joint = 0;                      // butterfly does not roll
    full.waist_yaw_joint = 0;

    // Amplitude interpolates NEUTRAL -> stroke, so amp 0 is a safe still hold.
    const out = {};
    for (const name of Object.keys(full)) {
      const n = NEUTRAL[name] ?? 0;
      out[name] = limit(name, n + amp * (full[name] - n));
    }
    return out;
  }

  /** Post-e-stop: smooth decay from the frozen pose to NEUTRAL, then hold. */
  _decay(t) {
    const u = ease((t - this.stopT) / Math.max(1e-3, this.cfg.estopSeconds));
    const out = {};
    for (const name of Object.keys(NEUTRAL)) {
      const from = this.stopPose?.[name] ?? NEUTRAL[name];
      out[name] = limit(name, from + (NEUTRAL[name] - from) * u);
    }
    return out;
  }

  /** Advance and return the pose. dt is clamped so a stalled tab cannot jump. */
  step(dt) {
    this.t += clamp(dt, 0, 0.05);
    return this.pose(this.t);
  }

  /** Every joint this stroke commands. Nothing else is touched. */
  static joints() { return Object.keys(NEUTRAL); }
}

/** Clamp into SAFETY.posFraction of the joint's range, about its midpoint. */
export function limit(name, v) {
  const L = G1_LIMITS[name];
  if (!L) return v;
  const mid = (L.lo + L.hi) / 2, half = ((L.hi - L.lo) / 2) * SAFETY.posFraction;
  return clamp(v, mid - half, mid + half);
}

// ---------------------------------------------------------------------------
// Dry run and validation. Nothing below moves anything.
// ---------------------------------------------------------------------------

/**
 * Sample a trajectory and measure it.
 *
 * Position, velocity and acceleration are all measured off the SAMPLED
 * command, not off the keyframes, because what a controller receives is the
 * samples — including whatever the amplitude ramp and the clamp did to them.
 *
 * @returns {{joints:string[], t:number[], q:object, peak:object, dt:number}}
 */
export function dryRun(cfg = {}, { seconds = 20, rate = 200 } = {}) {
  const bf = new Butterfly(cfg);
  const dt = 1 / rate;
  const n = Math.max(3, Math.round(seconds * rate));
  const joints = Butterfly.joints();
  const t = new Array(n);
  const q = {};
  for (const j of joints) q[j] = new Float64Array(n);

  for (let i = 0; i < n; i++) {
    t[i] = i * dt;
    const p = bf.pose(t[i]);
    for (const j of joints) q[j][i] = p[j] ?? 0;
  }

  // Central differences; the ends use one-sided, which is why n >= 3.
  const peak = {};
  for (const j of joints) {
    const s = q[j];
    let vmax = 0, amax = 0, qlo = Infinity, qhi = -Infinity;
    for (let i = 0; i < n; i++) {
      qlo = Math.min(qlo, s[i]); qhi = Math.max(qhi, s[i]);
      if (i > 0 && i < n - 1) {
        vmax = Math.max(vmax, Math.abs((s[i + 1] - s[i - 1]) / (2 * dt)));
        amax = Math.max(amax, Math.abs((s[i + 1] - 2 * s[i] + s[i - 1]) / (dt * dt)));
      }
    }
    peak[j] = { qlo, qhi, vmax, amax };
  }
  return { joints, t, q, peak, dt, cfg: bf.cfg };
}

/**
 * Check a dry run against the envelope.
 * @returns {{ok:boolean, violations:Array, worst:object}}
 */
export function validate(run) {
  const violations = [];
  const worst = { posFrac: 0, velFrac: 0, accelFrac: 0 };
  for (const j of run.joints) {
    const L = G1_LIMITS[j];
    const p = run.peak[j];
    if (!L) { violations.push({ joint: j, kind: 'unknown-joint' }); continue; }

    const mid = (L.lo + L.hi) / 2, half = (L.hi - L.lo) / 2;
    const posFrac = Math.max(Math.abs(p.qlo - mid), Math.abs(p.qhi - mid)) / half;
    const velFrac = p.vmax / (L.vel * SAFETY.velFraction);
    const accelFrac = p.amax / SAFETY.accelMax;
    worst.posFrac = Math.max(worst.posFrac, posFrac);
    worst.velFrac = Math.max(worst.velFrac, velFrac);
    worst.accelFrac = Math.max(worst.accelFrac, accelFrac);

    if (p.qlo < L.lo || p.qhi > L.hi) {
      violations.push({ joint: j, kind: 'position', got: [p.qlo, p.qhi], allowed: [L.lo, L.hi] });
    } else if (posFrac > SAFETY.posFraction + 1e-6) {
      violations.push({ joint: j, kind: 'position-envelope', got: posFrac, allowed: SAFETY.posFraction });
    }
    if (velFrac > 1 + 1e-6) {
      violations.push({ joint: j, kind: 'velocity', got: p.vmax, allowed: L.vel * SAFETY.velFraction });
    }
    if (accelFrac > 1 + 1e-6) {
      violations.push({ joint: j, kind: 'acceleration', got: p.amax, allowed: SAFETY.accelMax });
    }
  }
  return { ok: violations.length === 0, violations, worst };
}

/**
 * The highest cycle frequency this stroke can be run at and stay inside the
 * envelope, at a given amplitude.
 *
 * Worth stating because it is not a guess: joint velocity through a fixed
 * shape scales linearly with frequency and acceleration with its square, so
 * one dry run at a reference frequency fixes both, and the answer is
 * whichever of the two bounds binds first.
 */
export function maxSafeFrequency(amplitude = DEFAULTS.amplitude, ref = 0.5) {
  const was = _probing;
  _probing = true;
  let run;
  try {
    run = dryRun({ amplitude, frequency: ref, rampSeconds: 0 },
                 { seconds: 4 / ref, rate: 400 });
  } finally { _probing = was; }
  let fVel = Infinity, fAcc = Infinity;
  for (const j of run.joints) {
    const L = G1_LIMITS[j], p = run.peak[j];
    if (!L) continue;
    if (p.vmax > 1e-9) fVel = Math.min(fVel, ref * (L.vel * SAFETY.velFraction) / p.vmax);
    if (p.amax > 1e-9) fAcc = Math.min(fAcc, ref * Math.sqrt(SAFETY.accelMax / p.amax));
  }
  // Derate the analytic answer slightly. The scaling laws are exact but the
  // peaks they are fitted to come from finite differences, so a caller that
  // runs at EXACTLY the returned ceiling could still measure 1.003 of the
  // acceleration limit and fail its own validate(). A number you are allowed
  // to use has to be a number that passes.
  const DERATE = 0.97;
  return {
    hz: Math.min(fVel * DERATE, fAcc * DERATE, SAFETY.maxFrequency),
    boundBy: fVel < fAcc ? (fVel * DERATE < SAFETY.maxFrequency ? 'velocity' : 'policy')
                         : (fAcc * DERATE < SAFETY.maxFrequency ? 'acceleration' : 'policy'),
    fVel, fAcc,
  };
}

/**
 * What would have to exist before any of this could be sent to a real G1.
 * Kept in code, not in a README, so it is answered from the tree as it is.
 */
export function hardwareReadiness() {
  return {
    ready: false,
    have: [
      'joint-position trajectory, C1-continuous, clamped to URDF limits',
      'per-joint velocity and acceleration bounds, checked by validate()',
      'amplitude ramp from zero and a latching emergency stop',
      'a simulation target: urdf-loader + three.js, and the ISS scene',
    ],
    missing: [
      'a transport: no unitree_sdk2 / CycloneDDS / LowCmd path exists in this repo',
      'state feedback: nothing reads joint position, IMU or contact back',
      'a torque or PD interface: SAFETY.torqueFraction has nothing to apply to',
      'a whole-body balance controller: this is an open-loop trajectory and the ' +
        'G1 would need one to stay upright under it in 1 g',
      'a physics check: MuJoCo is not installed in .venv, so the stroke has ' +
        'never been stepped against dynamics — only against limits',
    ],
  };
}
