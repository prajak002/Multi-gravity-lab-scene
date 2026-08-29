/**
 * Footing — where the sole actually is, and putting it on the ground.
 *
 * The old plant() did two things wrong, and they hid each other.
 *
 *   1. It sampled the terrain ONCE, under the robot's root, and lifted the
 *      body so the lowest foot met that one height. A foot is half a metre
 *      away from the root at full stride, and on a DEM with real relief the
 *      ground under the leading foot is centimetres — sometimes tens of
 *      centimetres — from the ground under the pelvis. Stepping onto rising
 *      ground therefore drove the whole foot through the surface.
 *
 *   2. It measured the foot with `Box3.setFromObject`, a WORLD-axis-aligned
 *      box. The moment the ankle pitches or rolls, that box's min.y is a
 *      corner of a box that contains the foot, not a point on the foot: the
 *      more the foot is tilted the further below the real sole it sits, so the
 *      robot was lifted off the ground by the same rotation that was supposed
 *      to plant it. Combined with (1) the result reads as feet that sink on
 *      the way up a slope and hover on the way down.
 *
 * Both go away by working with the sole's REAL contact points, carried in the
 * foot link's own frame, and sampling the ground under each one of them:
 *
 *      lift = max over contact points of (ground(p) - p.y)
 *
 * That is the smallest rigid lift for which no contact point is below the
 * ground and at least one is exactly on it. No penetration is possible by
 * construction, and the support is decided by whichever point is really
 * lowest — which is how the toe gets to land before the heel.
 */
import { Vector3, Box3, Matrix4 } from 'three';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';
import { FOOT_CONTACTS } from './G1Kinematics.js';

const _v = new Vector3();
const _fwd = new Vector3();
const _lat = new Vector3();
const _c = new Vector3();
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/**
 * The sole of one foot, in that foot link's own frame.
 *
 * For the G1 these are the four contact spheres the URDF actually declares
 * (heel at x = -0.05, toe at x = +0.12, sole plane z = -0.035) rather than
 * anything inferred from the render mesh. For the other two descriptions,
 * which ship no contact geometry, the bottom face of the foot link's
 * LOCAL-space bounds is used — local, so it does not grow when the joint
 * turns, which is the whole failure mode above.
 */
function soleOf(def, link) {
  if (def.id === 'g1') {
    return {
      points: FOOT_CONTACTS.map((p) => new Vector3(p[0], p[1], p[2])),
      heel: [0, 1], toe: [2, 3],
      halfLen: 0.085, halfWid: 0.03,
      measured: true,
    };
  }
  const box = localBounds(link);
  if (!isFinite(box.min.z)) return null;
  // URDF convention: z is up in the link frame, so the sole is the min-z face.
  const z = box.min.z;
  const points = [
    new Vector3(box.min.x, box.max.y, z), new Vector3(box.min.x, box.min.y, z),
    new Vector3(box.max.x, box.max.y, z), new Vector3(box.max.x, box.min.y, z),
  ];
  return {
    points, heel: [0, 1], toe: [2, 3],
    halfLen: Math.max(0.02, (box.max.x - box.min.x) / 2),
    halfWid: Math.max(0.02, (box.max.y - box.min.y) / 2),
    measured: false,
  };
}

/** Bounds of everything under `link`, expressed in `link`'s own frame. */
function localBounds(link) {
  link.updateWorldMatrix(true, true);
  const inv = new Matrix4().copy(link.matrixWorld).invert();
  const m = new Matrix4();
  const box = new Box3();
  link.traverse((o) => {
    const g = o.geometry;
    if (!g) return;
    if (!g.boundingBox) g.computeBoundingBox();
    const bb = g.boundingBox;
    o.updateWorldMatrix(true, false);
    m.multiplyMatrices(inv, o.matrixWorld);
    for (let i = 0; i < 8; i++) {
      _v.set(i & 1 ? bb.max.x : bb.min.x, i & 2 ? bb.max.y : bb.min.y, i & 4 ? bb.max.z : bb.min.z);
      box.expandByPoint(_v.applyMatrix4(m));
    }
  });
  return box;
}

/**
 * Attach sole geometry to a loaded robot. Idempotent, so it is safe to call
 * from anywhere that plants — the measurement only happens once per model.
 */
export function measureSoles(loaded) {
  if (loaded.soles) return loaded.soles;
  loaded.soles = (loaded.feet || []).map((f) => soleOf(loaded.def, f)).filter(Boolean);
  return loaded.soles;
}

/**
 * Let the stance feet lie ON the ground instead of spearing into it.
 *
 * A sole held level over ground that is not level touches at one corner and
 * buries the opposite one. Real ankles solve that by conforming: the pitch
 * joint takes up the slope along the foot, the roll joint the slope across it.
 * Both are clamped to the URDF's own limits, so on ground steeper than the
 * ankle can follow the foot tilts as far as the hardware allows and no
 * further — which is a real constraint of the machine, not a rendering fudge.
 *
 * Only the loaded feet conform. A swing foot conforming to ground it has not
 * reached yet would flap at whatever it is passing over.
 *
 * @param {object} loaded    from loadRobot()
 * @param {object} pose      the gait pose; pose.contacts[i] true means SWINGING
 * @param {(x:number,z:number)=>number} heightAt
 */
export function conformAnkles(loaded, pose, heightAt) {
  const def = loaded.def;
  if (!heightAt || !def.legs) return;
  const soles = measureSoles(loaded);
  const sides = ['left', 'right'];
  loaded.root.updateMatrixWorld(true);

  for (let i = 0; i < sides.length; i++) {
    const J = def.legs[sides[i]];
    const foot = loaded.feet[i];
    const sole = soles[i];
    if (!J || !foot || !sole) continue;
    // pose.contacts holds `swinging`, so a true entry is a foot in the air.
    if (pose.contacts && pose.contacts[i]) continue;

    foot.getWorldPosition(_c);
    // The foot's own axes: URDF +x points at the toe, +y out to its left.
    _fwd.set(1, 0, 0).transformDirection(foot.matrixWorld); _fwd.y = 0;
    _lat.set(0, 1, 0).transformDirection(foot.matrixWorld); _lat.y = 0;
    if (_fwd.lengthSq() < 1e-8 || _lat.lengthSq() < 1e-8) continue;
    _fwd.normalize(); _lat.normalize();

    const L = sole.halfLen, W = sole.halfWid;
    const hToe = heightAt(_c.x + _fwd.x * L, _c.z + _fwd.z * L);
    const hHeel = heightAt(_c.x - _fwd.x * L, _c.z - _fwd.z * L);
    const hLeft = heightAt(_c.x + _lat.x * W, _c.z + _lat.z * W);
    const hRight = heightAt(_c.x - _lat.x * W, _c.z - _lat.z * W);

    // Ankle pitch is positive toe-DOWN (axis +y, URDF x-forward z-up), so
    // ground that rises ahead of the foot needs a NEGATIVE correction.
    const pitchSlope = Math.atan2(hToe - hHeel, 2 * L);
    // Ankle roll is positive about +x, which lifts the foot's left edge, so
    // ground that is higher on the left needs a POSITIVE correction.
    const rollSlope = Math.atan2(hLeft - hRight, 2 * W);

    if (J.anklePitch && loaded.joints[J.anklePitch]) {
      const j = loaded.joints[J.anklePitch];
      const base = pose.joints[J.anklePitch] ?? 0;
      setLimited(j, base - pitchSlope);
      pose.joints[J.anklePitch] = j.angle;
    }
    if (J.ankleRoll && loaded.joints[J.ankleRoll]) {
      const j = loaded.joints[J.ankleRoll];
      const base = pose.joints[J.ankleRoll] ?? 0;
      setLimited(j, base + rollSlope);
      pose.joints[J.ankleRoll] = j.angle;
    }
  }
}

/** Drive a urdf-loader joint, never past the limits its URDF declares. */
function setLimited(joint, value) {
  const lo = joint.limit?.lower, hi = joint.limit?.upper;
  const v = (typeof lo === 'number' && typeof hi === 'number' && hi > lo)
    ? clamp(value, lo, hi) : value;
  // setJointValue writes `angle` itself; urdf-loader defines it as a getter,
  // so assigning to it throws and takes the whole frame loop with it.
  joint.setJointValue(v);
  return v;
}

/**
 * Plant the robot by its real contact points.
 *
 * @param {object} loaded
 * @param {(x:number,z:number)=>number|null} heightAt  null for a flat floor at y = 0
 * @param {number} hop     flight-phase arc, added AFTER contact is resolved
 * @returns {{lift:number, contact:'toe'|'heel'|'mid'|null, clearance:number}}
 *          `clearance` is how far the lowest NON-supporting sole point sits
 *          above the ground — a positive number means the foot is resting on
 *          an edge, which is what a toe strike looks like.
 */
export function plantOnSoles(loaded, heightAt, hop = 0) {
  const soles = measureSoles(loaded);
  loaded.root.updateMatrixWorld(true);

  let lift = -Infinity, worst = -1, worstFoot = -1;
  const gaps = [];
  for (let f = 0; f < soles.length; f++) {
    const sole = soles[f];
    const foot = loaded.feet[f];
    if (!sole || !foot) continue;
    for (let k = 0; k < sole.points.length; k++) {
      _v.copy(sole.points[k]).applyMatrix4(foot.matrixWorld);
      const g = heightAt ? heightAt(_v.x, _v.z) : 0;
      const need = g - _v.y;                 // how far this point must rise
      gaps.push({ f, k, need });
      if (need > lift) { lift = need; worst = k; worstFoot = f; }
    }
  }
  if (!isFinite(lift)) return { lift: 0, contact: null, clearance: 0 };

  loaded.root.position.y += lift + hop;
  loaded.root.updateMatrixWorld(true);

  // Which part of the supporting foot took the load, and by how much it is
  // the only part touching. This is what the toe-first check reads.
  const sole = soles[worstFoot];
  let contact = 'mid', clearance = Infinity;
  if (sole) {
    contact = sole.toe.includes(worst) ? 'toe' : sole.heel.includes(worst) ? 'heel' : 'mid';
    const other = contact === 'toe' ? sole.heel : sole.toe;
    for (const g of gaps) {
      if (g.f !== worstFoot || !other.includes(g.k)) continue;
      clearance = Math.min(clearance, lift - g.need);
    }
  }
  return { lift, contact, clearance: isFinite(clearance) ? clearance : 0 };
}

/**
 * How far the deepest sole point is BELOW the ground. Zero is the only
 * acceptable answer after plantOnSoles(); it exists so a check can assert
 * that rather than trust it.
 */
export function penetration(loaded, heightAt) {
  const soles = measureSoles(loaded);
  loaded.root.updateMatrixWorld(true);
  let deepest = 0;
  for (let f = 0; f < soles.length; f++) {
    const sole = soles[f], foot = loaded.feet[f];
    if (!sole || !foot) continue;
    for (const p of sole.points) {
      _v.copy(p).applyMatrix4(foot.matrixWorld);
      const g = heightAt ? heightAt(_v.x, _v.z) : 0;
      deepest = Math.max(deepest, g - _v.y);
    }
  }
  return deepest;
}

/**
 * The points of a foot that must never be seen under the ground.
 *
 * `soleOf` above returns what the URDF says the robot COLLIDES with — for the
 * G1, four contact spheres on a sole plane at z = -0.035. That is the right
 * thing to plan against, and tools/retarget.mjs plants those spheres 4 mm into
 * the regolith on purpose. But the spheres are inset from the shell that is
 * actually DRAWN, so a clip whose spheres sit exactly where they were planned
 * still renders with the visible sole under the surface — measured across the
 * twelve surface scenes with the two runs superimposed, by 7 to 19 mm.
 *
 * So this returns the mesh's own sole. Two point sets, unioned, because they
 * fail in different places:
 *
 *   HULL VERTICES. The lowest point of a rigid mesh under any orientation is
 *   always a vertex of its convex hull, so the hull catches the case a
 *   local-frame band cannot: a foot pitched hard toe-down, where the lowest
 *   thing in the WORLD is the front edge of the toe, which in the foot's own
 *   frame is nowhere near the sole plane. Missing that is what left a swing
 *   foot 64 mm inside a hillside on Ganges Chasma. The hull is computed once,
 *   from 39,000 mesh vertices down to a few dozen.
 *
 *   A BAND ACROSS THE SOLE. The hull alone is not enough on rocky ground: a
 *   boulder can rise between two hull vertices and touch the flat of the sole,
 *   which is what left a LOADED foot 16 mm into a rock on Aristarchus. So the
 *   flat is sampled too, decimated to a fixed count.
 *
 * Both are measured in the foot link's own frame — local, so (as in soleOf)
 * the measurement does not inflate the moment the ankle rolls, which is the
 * bug that made a world-axis-aligned Box3 useless here.
 *
 * Cached on the loaded robot: this measures the model, not the pose.
 */
const SOLE_BAND = 0.008;    // m above the lowest vertex still counts as sole
const SOLE_MAX = 24;        // sampled sole points per foot, after decimation

export function visibleSole(loaded) {
  if (loaded.visibleSoles) return loaded.visibleSoles;
  const planned = measureSoles(loaded);
  loaded.visibleSoles = (loaded.feet || []).map((foot, i) => {
    const pts = planned[i] ? planned[i].points.map((p) => p.clone()) : [];

    // Every vertex of the foot, in the foot's own frame.
    foot.updateWorldMatrix(true, true);
    const inv = new Matrix4().copy(foot.matrixWorld).invert();
    const m = new Matrix4();
    const local = [];
    let lowest = Infinity;
    foot.traverse((o) => {
      const g = o.geometry;
      if (!o.isMesh || !g?.attributes?.position) return;
      o.updateWorldMatrix(true, false);
      m.multiplyMatrices(inv, o.matrixWorld);
      const a = g.attributes.position;
      for (let k = 0; k < a.count; k++) {
        const v = new Vector3().fromBufferAttribute(a, k).applyMatrix4(m);
        if (v.z < lowest) lowest = v.z;
        local.push(v);
      }
    });
    if (!local.length) return pts;

    // The whole silhouette, so no orientation can hide the lowest point.
    try {
      for (const v of new ConvexHull().setFromPoints(local).vertices) {
        pts.push(v.point.clone());
      }
    } catch { /* degenerate mesh: the band below still covers the flat sole */ }

    // The flat of the sole, so a rock between hull vertices is still felt.
    const band = local.filter((v) => v.z <= lowest + SOLE_BAND);
    const stride = Math.max(1, Math.ceil(band.length / SOLE_MAX));
    for (let k = 0; k < band.length; k += stride) pts.push(band[k]);

    return pts;
  });
  return loaded.visibleSoles;
}

/**
 * Raise a SWING foot out of the ground with its own knee.
 *
 * The stance guard cannot help here. It lifts the whole robot, which is right
 * for a foot carrying the machine and wrong for one in the air: hoisting the
 * body every time a swinging foot passes over a boulder makes the pelvis bob
 * at every rock, and worse, it erases the low foot clearance that is one of
 * the two behaviours the A/B page exists to compare. WorldVLA swings at 48-55
 * mm and PragyaSpace at 85-135 mm, and that difference has to survive.
 *
 * But a swing foot ploughing through a hillside still reads as a rendering
 * fault, and on Ganges Chasma it reached 66 mm. So the correction is made
 * where the real machine would make it — at the knee. Flexing the knee
 * shortens the hip-to-ankle distance and lifts the foot while the pelvis stays
 * exactly where the clip put it, so nothing about the body's trajectory moves.
 *
 * The gain d(height)/d(knee) is measured rather than derived: it depends on
 * the whole leg's pose, and one finite difference on the joint that is about
 * to be driven is both cheaper and more honest than a small-angle formula that
 * is wrong at the extremes of the swing.
 *
 * Bounded by MAX_FLEX. Past that the clip is asking for something the leg
 * cannot do, and silently bending further would hide it.
 */
const MAX_FLEX = 0.20;      // rad of extra knee flexion this may add

export function clearSwingFoot(loaded, footIndex, kneeJoint, need, groundAt) {
  const joint = loaded.joints?.[kneeJoint];
  const foot = loaded.feet?.[footIndex];
  const soles = visibleSole(loaded);
  const sole = soles[footIndex];
  if (!joint || !foot || !sole?.length || need <= 0) return 0;

  const base = joint.angle;
  const depth = () => {
    loaded.root.updateMatrixWorld(true);
    let d = 0;
    for (const p of sole) {
      _v.copy(p).applyMatrix4(foot.matrixWorld);
      const g = groundAt(_v.x, _v.z) - _v.y;
      if (g > d) d = g;
    }
    return d;
  };

  // The knee's sign convention is the URDF's: positive is flexion, and the
  // limit is [-0.087, 2.88], so there is always room to bend further.
  const PROBE = 0.05;
  setLimited(joint, base + PROBE);
  const probed = depth();
  const gain = (need - probed) / PROBE;          // metres of lift per radian
  if (!(gain > 1e-4)) { setLimited(joint, base); return 0; }

  const delta = clamp(need / gain, 0, MAX_FLEX);
  setLimited(joint, base + delta);
  if (depth() > 0) {
    // One refinement, for the leg poses where the gain is not locally linear.
    const extra = clamp(delta + depth() / gain, 0, MAX_FLEX);
    setLimited(joint, base + extra);
    return extra;
  }
  return delta;
}
