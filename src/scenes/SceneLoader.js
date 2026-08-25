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
  let field = null;
  const demName = scene.terrain?.dem;
  if (demName) {
    // The .f32 is a raw row-major Float32 heightfield; its sidecar carries the
    // pixel scale and the provenance of the observation it came from.
    const meta = scene.terrain.meta ?? await fetchJSON(`dem/${demName}.json`);
    const buf = await (await fetch(`dem/${demName}.f32`)).arrayBuffer();
    field = new SiteField(new Float32Array(buf), meta, profileFor(scene));
  }
  return { scene, field };
}

/**
 * Which scenes actually exist, so the picker never offers a dead entry.
 *
 * A HEAD request is not enough. Vite's dev server answers any unmatched path
 * with index.html and a 200, so probing by status offered all fourteen scenes
 * when only three were built and the missing ones failed later with
 * "Unexpected token '<'". The only reliable test is to fetch it and confirm it
 * parses as a scene.
 *
 * Results are cached, so the picker's probe is also the load of the first
 * scene rather than a second round trip.
 */
export async function discoverScenes() {
  await Promise.all(SCENE_IDS.map(async (id) => {
    if (_cache.has(id)) return;
    try {
      const r = await fetch(`scenes/${id}.json`);
      if (!r.ok) return;
      if (!/json/i.test(r.headers.get('content-type') || '')) return;
      const j = await r.json();
      if (j && j.clips && (j.clips.A || j.clips.B)) _cache.set(id, j);
    } catch { /* not built yet, or served as the SPA fallback */ }
  }));
  return SCENE_IDS.filter((id) => _cache.has(id));
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
