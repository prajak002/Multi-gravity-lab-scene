/**
 * SceneLoader — fetch a built scene and the terrain it was solved against.
 *
 * A scene in public/scenes/ carries two retargeted clips (A: WorldVLA,
 * B: PragyaSpace) plus a reference to the DEM patch they were planted on. The
 * viewer MUST rebuild its ground from that same DEM through the same SiteField
 * the retargeter used, because the footholds were solved against it: any other
 * surface and the feet visibly float or sink.
 *
 * Frames are stored in the packets' own convention — Z up, X east, Y north —
 * and converted to the renderer's Y-up frame here, in one place.
 */
import { Quaternion, Vector3 } from 'three';
import { SiteField, SURFACE_PROFILES } from '../terrain/SiteField.js';
import { generateHop } from '../sim/HopMotion.js';
import { G1_TREE } from '../sim/G1Body.js';

/** Scenes the viewer offers, in the order they should be shown. */
export const SCENE_IDS = [
  'moon_shiv_shakti', 'moon_shackleton_rim', 'moon_mare_tranquillitatis',
  'moon_aristarchus', 'moon_tycho_flank', 'moon_schrodinger_basin',
  'mars_jezero_delta', 'mars_gale_crater', 'mars_olympia_undae',
  'mars_cerberus_fossae', 'mars_medusae_fossae', 'mars_ganges_chasma',
  'iss_momentum_gap', 'iss_brake_gap',
];

/**
 * Z-up (packet) -> Y-up (three).
 *
 * The packets place +x east, +y north, +z up. three wants +y up, and this app
 * uses +z south so that a heading of 0 still runs east. That is the rotation
 * -90 degrees about X, applied to positions directly and to orientations by
 * conjugation, since the robot's own geometry is rotated by the same amount
 * inside loadRobot.
 */
const Q_ZUP_TO_YUP = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), -Math.PI / 2);
const Q_INV = Q_ZUP_TO_YUP.clone().invert();

export const posToRender = (p, out = new Vector3()) => out.set(p[0], p[2], -p[1]);

export function quatToRender(q, out = new Quaternion()) {
  out.set(q[0], q[1], q[2], q[3]);
  return out.premultiply(Q_ZUP_TO_YUP).multiply(Q_INV);
}

/**
 * The surface profile the scene was BUILT with.
 *
 * Taken from the scene, never guessed. The micro-relief layer is not
 * decoration — the retargeter planted every foothold against it, so choosing a
 * different profile here would rebuild a different surface from the one the
 * feet were solved on and they would visibly float or sink.
 *
 * An earlier version fell back to inferring a profile from the scene id when
 * the field looked absent. That was wrong twice over: the field is always
 * present, and the guess matched against a list of four profiles that has since
 * grown to ten, so a site like medusae_fossae would have been handed mars_sand
 * on the strength of "fossae" appearing in its name when it is built as
 * mars_yardang.
 */
function profileFor(scene) {
  const name = scene.terrain?.profile;
  const p = name && SURFACE_PROFILES[name];
  if (p) return p;
  console.warn(`[arena] scene ${scene.id}: surface profile ` +
    `${name ? `"${name}" is not in SURFACE_PROFILES` : 'missing'} — ` +
    `terrain will not match the surface the footholds were solved against.`);
  return SURFACE_PROFILES[scene.body === 'Mars' ? 'mars_rocky' : 'moon_mare'];
}

/** Scenes already fetched, so probing and loading are the same round trip. */
const _cache = new Map();

async function fetchJSON(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} -> ${r.status}`);
  return r.json();
}

/**
 * @returns {{scene:object, field:SiteField|null}}
 */
export async function loadScene(id) {
  const scene = _cache.get(id) ?? await fetchJSON(`scenes/${id}.json`);
  _cache.set(id, scene);
  return { scene, field: await fieldFor(scene) };
}

/**
 * The SiteField a scene stands on.
 *
 * Cached by patch and profile, because the arena now switches between forty
 * places and rebuilding a field means re-reading a two-megabyte heightfield.
 * The cache key includes the profile: the same DEM read with a different
 * micro-relief layer is a different surface, and handing back the wrong one
 * would put the feet through ground that was solved against the other.
 */
const _fields = new Map();
async function fieldFor(scene) {
  const demName = scene.terrain?.dem;
  if (!demName) return null;
  const profile = scene.terrain?.profile ?? '';
  const key = `${demName}|${profile}`;
  if (_fields.has(key)) return _fields.get(key);
  // The .f32 is a raw row-major Float32 heightfield; its sidecar carries the
  // pixel scale and the provenance of the observation it came from.
  const meta = scene.terrain.meta ?? await fetchJSON(`dem/${demName}.json`);
  const buf = await (await fetch(`dem/${demName}.f32`)).arrayBuffer();
  const field = new SiteField(new Float32Array(buf), meta, profileFor(scene));
  if (!scene.terrain.meta) scene.terrain.meta = meta;
  _fields.set(key, field);
  return field;
}

/**
 * One place, playing one motion.
 *
 * `motion` is 'compare' for the two packet runs the arena was built around, or
 * the id of a generated motion. Both come back in the same shape.
 */
export async function loadPlace(id, motion = 'compare', thrust = 1) {
  if (!motion || motion === 'compare') return loadScene(id);
  const scene = await loadMotionScene(id, motion, thrust);
  return { scene, field: await fieldFor(scene) };
}

/**
 * The catalogue: every place the build produced, grouped by body.
 *
 * One request, written by tools/build_index.mjs. The old discoverScenes()
 * probed fourteen candidate URLs and kept whatever parsed, which was a sound
 * answer to Vite's dev server returning index.html with a 200 for any missing
 * path — but it does not scale to forty places, and it cannot tell the picker
 * which body a place is on or what motions exist for it. A manifest can, and a
 * place whose terrain was never fetched simply is not in it.
 */
let _places = null;
export async function loadPlaces() {
  if (_places) return _places;
  _places = await fetchJSON('places.json');
  return _places;
}

/**
 * A generated-motion clip, in the same shape a packet scene has.
 *
 * The viewer should not care which it is playing — one is retargeted from a
 * packet and one is generated from the field strength, but both arrive as
 * joints, root, quat and contacts, and both were solved against the same
 * SiteField. So this normalises to the scene shape and the arena has one code
 * path.
 */
export async function loadMotionScene(id, motion, thrust = 1) {
  const key = `motion:${id}`;
  const doc = _cache.get(key) ?? await fetchJSON(`motions_baked/${id}.json`);
  _cache.set(key, doc);
  let clip = doc.motions?.[motion];
  if (!clip) throw new Error(`${id} has no ${motion} motion`);

  // THRUST IS LIVE.
  //
  // The baked clip is the default push. Anything else is generated here and
  // now, by the same src/sim/HopMotion.js that baked it — because a jump whose
  // thrust you cannot change does not show you what thrust does, and baking a
  // clip per setting would be a hundred and twenty files per site.
  if (Math.abs(thrust - 1) > 1e-3 && doc.terrain && doc.site) {
    const field = await fieldFor(doc);
    if (field) clip = regenerate(doc, clip, motion, field, thrust);
  }
  return {
    id, name: doc.name, body: doc.body, g: doc.g, blurb: doc.blurb,
    terrain: doc.terrain,
    // ONE robot. A generated motion is not a comparison between two models —
    // it is a comparison between two GRAVITIES, and the other one is not on
    // this terrain. The readout carries that instead.
    clips: { A: clip },
    motion, physics: clip.physics, generated: true, thrust,
  };
}

/**
 * Re-solve one motion at a different thrust, in the browser.
 *
 * ~300 frames, two legs, a damped-least-squares solve each: a few hundred
 * milliseconds, which is fast enough to drive from a slider and far cheaper
 * than the alternative of shipping a clip per thrust setting.
 */
function regenerate(doc, baked, motion, field, thrust) {
  const r = generateHop(field, { ...doc.site, g: doc.g }, motion, { thrust });
  const rnd = (v) => Math.round(v * 1e5) / 1e5;
  return {
    ...baked,
    frames: r.rows.length,
    angles: r.rows.map((row) => row.slice(7, 36).map(rnd)),
    root: r.rows.map((row) => [rnd(row[0]), rnd(row[1]), rnd(row[2])]),
    quat: r.rows.map((row) => [rnd(row[3]), rnd(row[4]), rnd(row[5]), rnd(row[6])]),
    contacts: r.contacts,
    physics: {
      ...baked.physics,
      thrust, v0: r.phys.v0, apex: r.phys.apex, hang: r.phys.hang,
      duty: r.phys.dutyFactor, speedCeiling: r.phys.speedCeiling,
      force: r.phys.force, omega: r.phys.omega, tumble: r.phys.tumble,
      armAuthority: r.phys.armAuthority, residual: r.phys.residual,
      correctable: r.phys.correctable, index: r.phys.index,
      stable: r.phys.stable, ankleTorqueLimit: r.phys.ankleTorqueLimit,
      capture: r.phys.capture, reach: r.phys.reach, canCapture: r.phys.canCapture,
    },
  };
}

/**
 * How often a joint is pinned against its own mechanical stop while carrying
 * load — computed here rather than shipped, because it is a display concern.
 *
 * This matters on the steep sites. Walking straight up a 13 degree rim with
 * local slopes near 30 degrees runs the ankle out of DORSIFLEXION: it reaches
 * its -50 degree stop, the sole can no longer lie flat, and the foot tips onto
 * its heel edge. That is not the solver failing, it is the hardware running
 * out, and it is the same reason people switchback up steep ground rather than
 * attacking it head-on. Worth saying out loud when it happens, because
 * otherwise it reads as feet not sitting properly on the ground.
 */
export function jointSaturation(clip, jointName, tol = 0.03) {
  const link = G1_TREE.find((l) => l.joint === jointName);
  if (!link?.limit || !clip?.angles) return null;
  const idx = clip.joints.indexOf(`${jointName}_dof`);
  if (idx < 0) return null;
  const [lo, hi] = link.limit;
  const side = jointName.startsWith('left') ? 0 : 1;

  let loaded = 0, atLow = 0, atHigh = 0, sum = 0;
  for (let i = 0; i < clip.angles.length; i++) {
    if (!clip.contacts?.[i]?.[side]) continue;
    loaded++;
    const v = clip.angles[i][idx];
    if (v <= lo + tol) { atLow++; sum += v; }
    else if (v >= hi - tol) atHigh++;
  }
  if (!loaded) return null;
  return {
    loaded, atLow, atHigh,
    fracLow: atLow / loaded, fracHigh: atHigh / loaded,
    frac: (atLow + atHigh) / loaded,
    meanAtLow: atLow ? sum / atLow : 0,
    lo, hi,
  };
}

/**
 * Per-frame pose of one clip, in RENDER coordinates.
 *
 * Interpolates between frames so playback is smooth at any rate and any
 * display refresh, rather than stepping at the capture's 30 Hz.
 */
export class ClipPlayer {
  constructor(clip) {
    this.clip = clip;
    this.frames = clip.frames ?? clip.angles.length;
    this.fps = clip.fps ?? 30;
    this.names = clip.joints.map((n) => n.replace(/_dof$/, ''));
    this.pos = new Vector3();
    this.quat = new Quaternion();
    this._qa = new Quaternion(); this._qb = new Quaternion();
    this._pa = new Vector3(); this._pb = new Vector3();
    this.joints = {};
  }

  get duration() { return this.frames / this.fps; }

  /** @param {number} t seconds into the clip */
  sample(t) {
    const f = Math.max(0, Math.min(this.frames - 1, t * this.fps));
    const i = Math.floor(f), j = Math.min(this.frames - 1, i + 1);
    const a = f - i;

    posToRender(this.clip.root[i], this._pa);
    posToRender(this.clip.root[j], this._pb);
    this.pos.copy(this._pa).lerp(this._pb, a);

    quatToRender(this.clip.quat[i], this._qa);
    quatToRender(this.clip.quat[j], this._qb);
    this.quat.copy(this._qa).slerp(this._qb, a);

    const ra = this.clip.angles[i], rb = this.clip.angles[j];
    for (let k = 0; k < this.names.length; k++) {
      this.joints[this.names[k]] = ra[k] + (rb[k] - ra[k]) * a;
    }
    // Contacts are boolean; take the frame we are nearest to.
    const c = this.clip.contacts?.[a < 0.5 ? i : j];
    this.contacts = c || [false, false];
    return this;
  }
}
