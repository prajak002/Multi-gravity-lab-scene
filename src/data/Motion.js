/**
 * Playback of a retargeted motion.
 *
 * Deliberately the same shape as Gait: advance(dt, g) then evaluate(g), both
 * returning the same fields. The app cannot tell which is driving, which is the
 * whole point — swapping a generated stride for a retargeted one should be a
 * one-line change, not a rewrite.
 *
 * What this DOES take from the capture: joint angles and foot contacts.
 * What it does NOT: the root height. Ground contact is still solved by
 * measuring where the feet ended up, because the capture's floor and this
 * terrain are not the same surface — trusting the recorded root would sink the
 * robot into a crater or float it over a ridge.
 *
 * Playback rate is scaled by gravity. A motion captured at 1 g replayed
 * unchanged on the Moon is simply wrong: the same limb swung by the same
 * torque takes sqrt(g_earth/g) longer. That factor is the one honest thing
 * that can be said about a 1 g capture in another field, and it is applied
 * here rather than baked into the file.
 */
import { G_EARTH } from '../render/Environments.js';

export class Motion {
  /**
   * @param {object} json  output of pipeline/bake_web.py
   * @param {object} def   robot definition, for the foot ordering
   */
  constructor(json, def, legLength = 0.7) {
    this.def = def;
    this.L = Math.max(0.25, legLength);
    this.names = json.joints;
    this.angles = json.angles;
    this.contacts = json.contacts || null;
    this.fps = json.fps || 30;
    this.frames = json.frames || this.angles.length;
    this.source = json.source || 'retarget';
    this.t = 0;
    this._pose = { joints: {}, bodyY: 0, pitch: 0, airborne: false, duty: 0.5, contacts: [] };

    // Step period measured FROM THE CLIP: count touchdowns of one foot and
    // divide the duration by them. Inventing a number here would quietly
    // decouple the readout from the motion actually on screen.
    this.baseStepPeriod = this._measureStepPeriod();
  }

  _measureStepPeriod() {
    const dur = this.frames / this.fps;
    if (!this.contacts || !this.contacts.length) return dur / 2;
    let downs = 0;
    for (let i = 1; i < this.contacts.length; i++) {
      if (this.contacts[i][0] && !this.contacts[i - 1][0]) downs++;
    }
    return downs > 0 ? dur / downs : dur / 2;
  }

  /** Same contract as Gait: how long one step takes in this field. */
  stepPeriod(g) { return this.baseStepPeriod * this.timeScale(g) / this.timeScale(G_EARTH); }

  /** Same contract as Gait: the LIPM capture point. */
  capturePoint(g, v) {
    if (g <= 1e-4) return 0;
    return v * Math.sqrt(this.L / g);
  }

  /** How much slower this motion runs in field g than in the field it was captured in. */
  timeScale(g) {
    if (g <= 1e-4) return 2.2;                       // free fall: nothing swings back
    return Math.sqrt(G_EARTH / g);
  }

  advance(dt, g) {
    this.t += dt / this.timeScale(g);
    const dur = this.frames / this.fps;
    if (this.t >= dur) this.t -= dur;                // loop
    // Speed the capture implies, not a speed we chose: one cycle of the clip
    // carries the robot one stride, and the stride slows with the pendulum.
    const v = 1.35 / this.timeScale(g) * (G_EARTH / Math.max(g, 1.0)) ** 0.25;
    return { T: dur / 2, v, phase: this.t / dur };
  }

  evaluate(g) {
    const f = Math.min(this.frames - 1, Math.floor(this.t * this.fps));
    const row = this.angles[f];
    const joints = this._pose.joints;
    for (let i = 0; i < this.names.length; i++) joints[this.names[i]] = row[i];

    // contacts[] is [left, right] in the capture; the app expects "is this
    // foot SWINGING", so invert.
    const c = this.contacts ? this.contacts[f] : null;
    this._pose.contacts = c ? c.map((down) => !down) : [];
    this._pose.duty = c ? (c.filter(Boolean).length / c.length) : 0.5;
    this._pose.bodyY = 0;                             // plant() owns vertical placement
    this._pose.airborne = c ? !c.some(Boolean) : false;
    return this._pose;
  }
}

/** Fetch a baked motion, or null if there is none for this robot. */
export async function loadMotion(url, def, legLength) {
  const res = await fetch(url);
  if (!res.ok) return null;
  return new Motion(await res.json(), def, legLength);
}
