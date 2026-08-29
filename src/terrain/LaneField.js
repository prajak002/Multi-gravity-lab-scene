/**
 * LaneField — the ground under two robots that are drawn side by side.
 *
 * THE PROBLEM
 *
 * Both clips in a scene start from the same origin and walk almost the same
 * line, so drawn honestly they occupy the same space and the viewer shows one
 * humanoid instead of two. The arena separates them: each robot is pushed half
 * the separation sideways along the lane axis, 0.9 m by default.
 *
 * That displacement is a lie the ground does not know about. Every foothold in
 * a clip was solved against SiteField at the position the packet walks, and the
 * clip's root height carries that solution frame by frame. Slide the robot
 * 0.9 m across a real slope and it is standing over terrain it was never
 * solved for. Measured before this existed, on the default separation:
 *
 *     Medusae Fossae   270 mm of robot below the surface
 *     Ganges Chasma    228 mm
 *     Shackleton rim   201 mm, both knees and both hips buried with the feet
 *
 * and with the two runs superimposed, where the offset is zero, the same
 * scenes measured 7-19 mm. The offset was the whole story.
 *
 * It corrupts the comparison as well as the contact. WorldVLA and PragyaSpace
 * are supposed to differ only in how they move; put them in lanes 1.8 m apart
 * and they also meet different rocks, so some of the difference on screen is
 * the terrain rather than the model.
 *
 * THE FIX
 *
 * Move the ground with the robot. Each lane is drawn on its own copy of the
 * same heightfield, translated by that lane's offset, so a robot in lane k
 * stands on exactly the surface its footholds were planted in — contact is
 * correct by construction rather than by correction, and both models now walk
 * over identical ground.
 *
 * Two copies of one field, abutting, would leave a step down the middle of the
 * frame: at the midline the two lanes sample points `separation` apart, which
 * on the Shackleton rim is a fifth of a metre. So the copies are not abutted,
 * they are BLENDED — one continuous height function, a weighted sum of the
 * field sampled through each lane's offset plus the untranslated field far
 * away:
 *
 *     h(p) = Σ w_k(p)·field(p − o_k)  +  (1 − Σ w_k)·field(p)
 *
 * `w_k` is 1 over the corridor its robot actually walks and falls smoothly to
 * 0 by the midline, so under either robot the sum collapses to that one lane
 * and the height is exact. Everywhere else the surface is a crossfade of the
 * same NASA window sampled from two nearby places — which is real terrain, but
 * it is not the terrain of THAT spot, and the arena says so in its provenance
 * panel rather than leaving it implied.
 *
 * There is one implementation and both callers use it: the mesh builder and
 * the contact guard. That is the same discipline SiteField enforces between
 * the renderer and the retargeter, and for the same reason — a surface drawn
 * from one function and stood on according to another is exactly how feet come
 * to float.
 */
import { Vector3 } from 'three';

const _d = new Vector3();

/** Smoothstep, so the blend has no kink for the contact guard to trip over. */
const smooth = (t) => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));

export class LaneField {
  /**
   * @param {import('./SiteField.js').SiteField} field  the site's one true field
   */
  constructor(field) {
    this.field = field;
    this.lanes = [];
    this.core = 0.45;
    this.blend = 0.45;
  }

  /**
   * Declare where the lanes are.
   *
   * @param {Array<{offset: Vector3, path: Vector3[]}>} lanes
   *        `offset` is the lane's lateral displacement; `path` is the traverse
   *        it covers, in world space, already displaced.
   *
   *        A POLYLINE, not the straight line between its ends, which is what
   *        this took first. A packet traverse is straight in net direction —
   *        within 0.3 deg over five metres — but it wanders on the way: the
   *        PragyaSpace run on Ganges Chasma strays 0.633 m from the chord. With
   *        a straight corridor the lane weight under that part of the walk fell
   *        to 0.157, so the ground drawn there was 80 mm away from the surface
   *        the footholds were solved on, and the foot went 54 mm into it. The
   *        corridor has to follow the walk, not approximate it.
   * @param {number} separation  metres between the lanes, which is what decides
   *        how much room the blend has: a lane's weight must reach zero by the
   *        midline or the two would overlap and neither robot would be standing
   *        on its own ground.
   */
  setLanes(lanes, separation) {
    this.lanes = lanes;
    const room = Math.max(0, separation * 0.5);
    // Keep the exact corridor wide enough for a stance (the feet sit within
    // ±0.3 m of the lane centre) whenever the separation can afford it.
    this.core = Math.min(0.45, room * 0.5);
    this.blend = Math.max(0.15, room - this.core);
  }

  /** Weight of lane `i` at (x, z): 1 on its corridor, 0 by the midline. */
  _weight(i, x, z) {
    const L = this.lanes[i];
    if (!L) return 0;
    // Distance to the lane's traverse segment, so the corridor follows the
    // walk rather than being a disc around its middle.
    const path = L.path;
    let dist = Infinity;
    for (let k = 0; k + 1 < path.length; k++) {
      const a = path[k], b = path[k + 1];
      _d.copy(b).sub(a); _d.y = 0;
      const len2 = _d.lengthSq();
      let t = 0;
      if (len2 > 1e-9) {
        t = ((x - a.x) * _d.x + (z - a.z) * _d.z) / len2;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
      }
      const dx = x - (a.x + _d.x * t), dz = z - (a.z + _d.z * t);
      const d2 = dx * dx + dz * dz;
      if (d2 < dist) dist = d2;
    }
    if (!isFinite(dist)) return 0;
    return 1 - smooth((Math.sqrt(dist) - this.core) / this.blend);
  }

  /**
   * Ground height as the viewer draws it AND as the contact guard reads it.
   *
   * Weights are normalised when they sum past one, which happens only as the
   * separation slider closes the lanes onto each other. There the offsets are
   * nearly equal too, so every branch is sampling almost the same point and
   * the normalisation costs nothing.
   */
  heightAt(x, z) {
    const n = this.lanes.length;
    if (!n) return this.field.heightAt(x, z);

    let w0 = 0, w1 = 0;
    if (n > 0) w0 = this._weight(0, x, z);
    if (n > 1) w1 = this._weight(1, x, z);
    const sum = w0 + w1;
    if (sum > 1) { w0 /= sum; w1 /= sum; }

    const rest = 1 - w0 - w1;
    let h = 0;
    if (w0 > 0) { const o = this.lanes[0].offset; h += w0 * this.field.heightAt(x - o.x, z - o.z); }
    if (w1 > 0) { const o = this.lanes[1].offset; h += w1 * this.field.heightAt(x - o.x, z - o.z); }
    if (rest > 1e-6) h += rest * this.field.heightAt(x, z);
    return h;
  }

  /** True when the lanes are displaced at all — i.e. when any blending happens. */
  get displaced() {
    return this.lanes.some((L) => L.offset.lengthSq() > 1e-8);
  }
}
