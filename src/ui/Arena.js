/**
 * Arena — the A/B viewer.
 *
 * One camera, one terrain, two robots, one clock. A vertical wipe reveals
 * WorldVLA on the left of the split and PragyaSpace on the right, both running
 * the same frame of the same scenario at the same instant, so the difference
 * you see at the seam is a difference in the motion and nothing else.
 *
 * The ground is rebuilt from the same DEM and the same SiteField the
 * retargeter solved the footholds against. That is not an optimisation — it is
 * the only way the feet land where they were planted. A prettier surface built
 * any other way would put the robot's soles through it.
 */
import {
  Scene, PerspectiveCamera, WebGLRenderer, Group, Mesh, MeshStandardMaterial,
  BufferGeometry, BufferAttribute, DirectionalLight, HemisphereLight, Color,
  Vector3, Vector2, ACESFilmicToneMapping, PCFSoftShadowMap, FogExp2, Box3,
  Points, PointsMaterial, BufferGeometry as BG, DoubleSide,
} from 'three';
import { ENVIRONMENTS, byId, sunDir } from '../render/Environments.js';
import { IBL } from '../render/IBL.js';
import { loadRobot, robotById } from '../render/Robots.js';
import { loadScene, loadPlace, loadPlaces, ClipPlayer, jointSaturation, posToRender } from '../scenes/SceneLoader.js';
import { visibleSole, clearSwingFoot } from '../sim/Footing.js';
import { LaneField } from '../terrain/LaneField.js';
import { instability } from '../sim/Ballistic.js';

// A G1 is 1.32 m tall and about 0.5 m across at the arms. This is the radius
// of the ball one of them needs to sit inside, which is what the framing has
// to hold on top of however far apart the two runs are.
const BODY_RADIUS = 0.85;
// Headroom, so a robot is never touching the edge of the frame.
const FRAME_MARGIN = 1.18;

const _half = new Vector3();
const _size = new Vector2();
const _view = new Vector3();
const _pt = new Vector3();
const clampTo = (v, a, b) => Math.min(b, Math.max(a, v));

const CLIP_KEYS = ['A', 'B'];
// loaded.feet is ordered left, right — the same order as loaded.soles.
const SIDES = ['left', 'right'];

/** Generated motions, as the picker names them. */
const MOTION_LABEL = { lope: 'LOPE', bound: 'BOUND', trip: 'TRIP + RECOVER' };
const MOTION_TITLE = {
  lope: 'The gait the Apollo crews adopted within minutes on every mission: '
      + 'one foot at a time, airborne between every step.',
  bound: 'Both feet together, everything into the vertical. The same push on '
       + 'every body, so the whole difference in height is the field.',
  trip: 'A caught toe and the fall that follows. Toppling scales as 1/sqrt(g), '
      + 'so one sixth gravity buys well over a second of extra warning.',
};
// Corridor samples per lane. See LaneField.setLanes for why a traverse needs a
// polyline rather than its chord.
const LANE_PATH_POINTS = 24;

/** Both arms' share of the body's pitch inertia — what a windmill can buy. */
const ARM_SHARE = 0.138;

/**
 * The same motion on the other body, evaluated rather than scaled.
 *
 * With thrust as the input, take-off speed is NOT the same on both — the same
 * force has less weight to fight in a weaker field, so it leaves faster AND
 * hangs longer, and the two compound. Scaling this body's numbers by a ratio
 * of g would understate the difference, so the model is simply run again.
 */
function otherBody(g, p) {
  const s = instability(g, { thrust: p.thrust ?? 1, mu: 0.45, offset: p.offset });
  return { apex: s.apex, hang: s.hang, tumble: s.tumble,
           armAuthority: s.armAuthority, correctable: s.correctable, index: s.index };
}

/** Which environment's lighting a scene borrows. */
const envForBody = (body) =>
  byId(body === 'Mars' ? 'mars' : body === 'ISS' ? 'iss' : 'moon');

export class Arena {
  constructor(canvas, ui) {
    this.canvas = canvas;
    this.ui = ui;

    this.renderer = new WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.toneMapping = ACESFilmicToneMapping;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = PCFSoftShadowMap;

    this.scene = new Scene();
    this.scene.fog = new FogExp2(0x000000, 0);
    this.camera = new PerspectiveCamera(40, 1, 0.05, 4000);

    this.sun = new DirectionalLight(0xffffff, 3);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -14; sc.right = 14; sc.top = 14; sc.bottom = -14; sc.near = 0.5; sc.far = 120;
    sc.updateProjectionMatrix();
    this.sun.shadow.normalBias = 0.03;
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);

    this.fill = new HemisphereLight(0x8fa6c8, 0x2b2622, 0.25);
    this.scene.add(this.fill);
    this.ibl = new IBL(this.renderer);

    this.world = new Group();
    this.scene.add(this.world);
    this.scene.add(this._stars());

    this.terrain = null;
    this.robots = {};          // 'A' | 'B' -> loaded robot
    this.players = {};
    this.scene_ = null;
    this.field = null;

    // playback
    this.t = 0;
    this.playing = true;
    this.rate = 1;
    this.split = 0.5;
    // Lateral separation of the two runs, metres.
    //
    // Both clips start at the same origin and follow almost the same path, so
    // drawn honestly they occupy the same space. Split-screen then shows the
    // left half of one robot and the right half of the other, which the eye
    // reads as a SINGLE humanoid rather than as a comparison. Pushing them into
    // separate lanes is what makes two robots visible; the wipe mode below puts
    // them back on top of each other when you want to compare pose exactly.
    this.separation = 1.8;
    this.mode = 'lanes';          // 'lanes' | 'wipe'
    this.laneAxis = new Vector3(0, 0, 1);

    // camera rig
    this.orbit = -0.6;
    this.elev = 0.22;
    this.dist = 6.2;
    this.aim = new Vector3();
    this._drag = null;
    this._bindInput();
    addEventListener('resize', () => this.resize());
    this.resize();
  }

  _stars() {
    const n = 2200, pos = new Float32Array(n * 3);
    // Deterministic: a star field that reshuffles on reload reads as noise.
    let s = 8675309;
    const rnd = () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; };
    for (let i = 0; i < n; i++) {
      const u = rnd() * 2 - 1, th = rnd() * Math.PI * 2, r = Math.sqrt(1 - u * u);
      pos.set([Math.cos(th) * r * 1800, Math.abs(u) * 1800, Math.sin(th) * r * 1800], i * 3);
    }
    const g = new BG();
    g.setAttribute('position', new BufferAttribute(pos, 3));
    this.stars = new Points(g, new PointsMaterial({ color: 0xffffff, size: 2.2, sizeAttenuation: false }));
    return this.stars;
  }

  _bindInput() {
    const c = this.canvas;
    c.addEventListener('pointerdown', (e) => {
      // Dragging the seam moves the wipe; dragging anywhere else orbits.
      const x = e.clientX / innerWidth;
      if (Math.abs(x - this.split) < 0.02) this._drag = { mode: 'split' };
      else this._drag = { mode: 'orbit', x: e.clientX, y: e.clientY };
      c.setPointerCapture(e.pointerId);
    });
    c.addEventListener('pointermove', (e) => {
      if (!this._drag) {
        this.canvas.style.cursor =
          Math.abs(e.clientX / innerWidth - this.split) < 0.02 ? 'ew-resize' : 'grab';
        return;
      }
      if (this._drag.mode === 'split') {
        this.split = Math.max(0.04, Math.min(0.96, e.clientX / innerWidth));
      } else {
        this.orbit -= (e.clientX - this._drag.x) * 0.006;
        this.elev = Math.max(-0.15, Math.min(1.2, this.elev + (e.clientY - this._drag.y) * 0.004));
        this._drag.x = e.clientX; this._drag.y = e.clientY;
      }
    });
    const end = () => { this._drag = null; };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', end);
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      // Zoom from the distance the camera is ACTUALLY at, not from the
      // requested one. A scene starts with dist = 0 meaning "as close as the
      // pair allows", and 0 multiplied by 1.12 is still 0 — so scrolling out
      // from a fresh scene would have done nothing at all.
      const base = Math.max(this.dist, this._holdBothDistance());
      this.dist = Math.max(1.6, Math.min(40, base * (1 + Math.sign(e.deltaY) * 0.12)));
    }, { passive: false });
  }

  resize() {
    const w = innerWidth, h = innerHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // -------------------------------------------------------------------------
  // terrain
  // -------------------------------------------------------------------------
  /**
   * A grid sampled from SiteField.heightAt, over the ground the traverse
   * actually crosses.
   *
   * Resolution is the point: the DEM is 1 m/px at best, but the contact solver
   * placed feet against a surface that includes synthesised centimetre relief,
   * so the mesh has to resolve that too. Anything coarser and the sole rests on
   * a triangle that does not exist in the field the footholds were solved in.
   */
  _grid(x0, x1, z0, z1, cell, skip) {
    const nx = Math.max(2, Math.round((x1 - x0) / cell));
    const nz = Math.max(2, Math.round((z1 - z0) / cell));
    const pos = new Float32Array((nx + 1) * (nz + 1) * 3);
    const idx = [];
    for (let j = 0; j <= nz; j++) {
      for (let i = 0; i <= nx; i++) {
        const x = x0 + (i / nx) * (x1 - x0);
        const z = z0 + (j / nz) * (z1 - z0);
        const k = (j * (nx + 1) + i) * 3;
        pos[k] = x; pos[k + 1] = this.ground(x, z); pos[k + 2] = z;
      }
    }
    for (let j = 0; j < nz; j++) {
      for (let i = 0; i < nx; i++) {
        const x = x0 + ((i + 0.5) / nx) * (x1 - x0);
        const z = z0 + ((j + 0.5) / nz) * (z1 - z0);
        if (skip && x > skip[0] && x < skip[1] && z > skip[2] && z < skip[3]) continue;
        const a = j * (nx + 1) + i, b = a + 1, c = a + nx + 1, d = c + 1;
        idx.push(a, c, b, b, c, d);
      }
    }
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
  }

  /**
   * The ground, for everything: the mesh that is drawn and the guard that
   * decides a foot is in it. One function, so the two cannot disagree.
   */
  /**
   * True only when there really are two runs to compare.
   *
   * A generated motion has one clip, and everything that exists to separate,
   * wipe between or frame A PAIR has to switch off — otherwise the empty lane
   * still gets its half of the separation, the camera still frames a robot
   * that is not there, and the previous scene's robot B is left standing in
   * the shot holding its last pose.
   */
  get paired() { return !!(this.players.A && this.players.B); }

  ground(x, z) {
    if (this.laneField) return this.laneField.heightAt(x, z);
    return this.field ? this.field.heightAt(x, z) : 0;
  }

  /**
   * Tell the lane field where the two runs are about to be drawn.
   *
   * Called whenever anything that moves a lane changes — the scene, the
   * separation slider, the mode toggle — because the ground has to follow the
   * robot for the footholds to stay solved. See LaneField for why.
   */
  _updateLanes() {
    if (!this.field) { this.laneField = null; return; }
    if (!this.laneField || this.laneField.field !== this.field) {
      this.laneField = new LaneField(this.field);
    }
    const sep = (this.mode === 'lanes' && this.paired) ? this.separation : 0;
    const lanes = [];
    for (const k of CLIP_KEYS) {
      const p = this.players[k];
      if (!p) continue;
      const offset = this.laneAxis.clone()
        .multiplyScalar((k === 'A' ? -1 : 1) * sep * 0.5);
      // The corridor follows the traverse itself. Decimated, because a lane
      // weight is evaluated once per terrain vertex and the shape of a five
      // metre walk survives being sampled every few frames.
      const root = p.clip.root;
      const stride = Math.max(1, Math.floor(root.length / LANE_PATH_POINTS));
      const path = [];
      for (let i = 0; i < root.length; i += stride) path.push(posToRender(root[i]).add(offset));
      const last = posToRender(root[root.length - 1]).add(offset);
      if (path.length < 2 || path[path.length - 1].distanceToSquared(last) > 1e-6) path.push(last);
      lanes.push({ offset, path });
    }
    this.laneField.setLanes(lanes, sep);
    this._terrainDirty = true;
  }

  buildTerrain(env) {
    if (this.terrain) { this.world.remove(this.terrain); this.terrain = null; }
    this._terrainDirty = false;
    if (!this.field) return;

    // Bounds of the traverse, from the clips themselves.
    const box = new Box3();
    for (const k of CLIP_KEYS) {
      const p = this.players[k];
      if (!p) continue;
      for (const r of p.clip.root) box.expandByPoint(new Vector3(r[0], r[2], -r[1]));
    }
    // Pad covers the lane offset as well as the traverse itself.
    const pad = 5.0 + this.separation;
    const x0 = box.min.x - pad, x1 = box.max.x + pad;
    const z0 = box.min.z - pad, z1 = box.max.z + pad;

    const mat = new MeshStandardMaterial({
      color: new Color(env.groundAlbedo ?? 0x8a7f74),
      roughness: 0.97, metalness: 0.0, side: DoubleSide,
    });
    const grp = new Group();
    // Near ground at contact resolution; the far ring only has to read as
    // landscape, so it is sampled far more cheaply.
    const near = new Mesh(this._grid(x0, x1, z0, z1, 0.075), mat);
    near.receiveShadow = true; near.castShadow = false;
    grp.add(near);

    // The far ring fills the patch, which is now 2:1 with its long axis along
    // the traverse — so it reaches further ahead of the walk than beside it,
    // which is also where a viewer looks.
    const spanX = Math.min(this.field.halfX * 2 - 2, 760);
    const spanZ = Math.min(this.field.halfZ * 2 - 2, 380);
    const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2;
    const far = new Mesh(this._grid(cx - spanX / 2, cx + spanX / 2, cz - spanZ / 2, cz + spanZ / 2,
                                    Math.max(spanX, spanZ) / 240, [x0, x1, z0, z1]), mat);
    far.receiveShadow = true;
    grp.add(far);

    this.terrain = grp;
    this.world.add(grp);
  }

  /**
   * Reference geometry for the free-flight scenes.
   *
   * These clips carry micro:true and no terrain, so there is nothing to stand
   * on and nothing to judge motion against — a robot coasting at 0.42 m/s in
   * front of a starfield looks stationary. The obvious move, dropping in
   * env/iss_station.glb, is wrong: that is the EXTERIOR station, roughly a
   * hundred metres across, and it simply swallows a 1.3 m robot.
   *
   * What the scenario is actually about is two fixed points — the wall the
   * robot pushes off and the handrail it is trying to reach — so those are what
   * get drawn, taken from the clip's own phase markers. The gap between them is
   * the whole question the packets pose.
   */
  buildMicroSet(scene) {
    if (this.microSet) { this.world.remove(this.microSet); this.microSet = null; }
    if (!scene.micro) return;
    const clip = scene.clips.B || scene.clips.A;
    if (!clip?.root?.length) return;

    const grp = new Group();
    const toRender = (r) => new Vector3(r[0], r[2], -r[1]);
    const start = toRender(clip.root[0]);
    const capF = clip.phases?.captureFrame ?? (clip.root.length - 1);
    const target = toRender(clip.root[Math.min(capF, clip.root.length - 1)]);

    // Travel axis, from the clip itself — the wall goes behind the start and
    // the rail across the far end, whichever way the traverse actually runs.
    const dir = Math.sign(target.x - start.x) || 1;

    // BOTH ends of the module, because both matter and each scenario turns on
    // a different one. MomentumGap is about the launch wall: whether the push
    // off it was enough. BrakeGap is about the far bulkhead: WorldVLA misses
    // the rail and takes its 25.2 N s of momentum into that instead. Drawing
    // only the wall behind the start left the collision the clip is named for
    // happening against nothing.
    const panel = () => new MeshStandardMaterial({
      color: 0x9aa4b0, roughness: 0.8, metalness: 0.1, side: DoubleSide });

    // Where the panels go is decided by the robots, not by the pelvis path.
    //
    // These were placed at a fixed offset from the clip's own start and
    // capture points — 0.35 m behind one, 0.42 m past the other. A pelvis is
    // not the robot: through the push-off the feet reach back well past it and
    // through the reach the hands go well forward. Measured on BrakeGap, the
    // pair swept x from -0.07 to 4.16 against panels standing at 0.28 and
    // 3.82, so the robot went straight through BOTH walls, by about a third of
    // a metre each. Taking the swept bounds puts each panel where the furthest
    // part of either robot actually arrives, which is what makes the push-off
    // land on the wall and the bulkhead stop the clip that hits it.
    const swept = this._sweptBounds();
    const launchX = dir > 0 ? swept.min.x : swept.max.x;
    const farX = dir > 0 ? swept.max.x : swept.min.x;

    // Inside a module the bulkheads ARE the two walls, so drawing free-standing
    // panels as well would put a second surface a few centimetres in front of
    // each of them.
    if (!this.module) {
      const wall = new Mesh(this._quad(1.7, 1.7), panel());
      wall.position.copy(start);
      wall.position.x = launchX;
      wall.rotation.y = Math.PI / 2;
      grp.add(wall);

      const far = new Mesh(this._quad(1.9, 1.9), panel());
      far.position.copy(target);
      far.position.x = farX;
      far.position.y += 0.1;
      far.rotation.y = Math.PI / 2;
      grp.add(far);
    }

    // The handrail it is reaching for, across the far end of the glide.
    const railMat = new MeshStandardMaterial({ color: 0xd8c07a, metalness: 0.65, roughness: 0.3 });
    const rail = new Mesh(this._tube(0.022, 1.2), railMat);
    rail.position.copy(target);
    rail.position.y += 0.30;
    grp.add(rail);
    for (const s2 of [-0.5, 0.5]) {
      const post = new Mesh(this._tube(0.014, 0.34), railMat);
      post.rotation.x = Math.PI / 2;
      post.position.set(target.x, target.y + 0.13, target.z + s2);
      grp.add(post);
    }

    // Longitudinal markers, so distance covered is readable frame to frame.
    const tickMat = new MeshStandardMaterial({ color: 0x4a5560, roughness: 0.9 });
    const span = Math.abs(target.x - start.x) + 1.0;
    for (let d = 0; d <= span; d += 0.5) {
      const t = new Mesh(this._quad(0.045, 0.42), tickMat);
      t.position.set(start.x + dir * d, start.y - 0.95, start.z);
      t.rotation.x = -Math.PI / 2;
      grp.add(t);
    }
    this.microSet = grp;
    this.world.add(grp);
  }

  /**
   * The box both robots pass through over the whole clip.
   *
   * Sampled rather than solved: there is no closed form for the reach of a
   * 29-joint chain across three hundred frames, and the answer is only needed
   * once per scene load. Playback position is saved and restored, so building
   * the set does not jog the clock the viewer is showing.
   */
  _sweptBounds(samples = 48) {
    const box = new Box3();
    const t0 = this.t, playing = this.playing;
    this.playing = false;
    const dur = this.players.A?.duration ?? this.players.B?.duration ?? 10;
    for (let s = 0; s <= samples; s++) {
      this.t = (s / samples) * dur;
      for (const k of CLIP_KEYS) {
        const p = this.players[k], r = this.robots[k];
        if (!p || !r) continue;
        p.sample(this.t);
        r.root.position.copy(p.pos);
        r.root.quaternion.copy(p.quat);
        for (const [name, v] of Object.entries(p.joints)) r.joints[name]?.setJointValue(v);
        r.root.updateMatrixWorld(true);
        box.expandByObject(r.root);
      }
    }
    this.t = t0; this.playing = playing;
    return box;
  }

  /**
   * The pressurised module a microgravity scenario is playing inside.
   *
   * Drawn at the element's REAL published dimensions — Destiny is 8.53 m long
   * and 4.27 m across, Kibo 11.19 by 4.4, Cupola 1.5 by 2.95 — because that is
   * the entire reason to draw an interior at all. An enclosure earns its place
   * by giving the eye a known measurement to compare a 1.32 m robot against,
   * and a shell fitted to an arbitrary target length throws exactly that away.
   * It is also what makes the eight modules different from each other: the
   * scenario is identical in all of them and the room is not, so a push that
   * overshoots by half a metre is a caught handrail in Kibo and a collision in
   * Cupola.
   *
   * Built rather than loaded. env/iss_corridor.glb is one specific 43.8 m
   * run; rescaling it to stand in for a 6.87 m laboratory would put the same
   * lie back in by another route.
   *
   * DoubleSide throughout, because the camera clamps INSIDE the cross-section
   * and every face is being viewed from behind.
   */
  buildModule(place) {
    if (this.module) { this.world.remove(this.module); this.module = null; }
    this.moduleBounds = null;
    const dims = place?.module;
    if (!dims) return;

    const r = dims.diameter / 2, len = dims.length;
    const grp = new Group();
    const shell = new MeshStandardMaterial({
      color: 0xb9bec4, roughness: 0.72, metalness: 0.12, side: DoubleSide });
    const rackMat = new MeshStandardMaterial({
      color: 0x6f7783, roughness: 0.55, metalness: 0.35, side: DoubleSide });

    // The tube. Along x, which is the axis every micro clip travels on.
    const SEG = 40;
    const pos = [], idx = [];
    for (let i = 0; i <= SEG; i++) {
      const a = (i / SEG) * Math.PI * 2;
      pos.push(-len / 2, Math.sin(a) * r, Math.cos(a) * r);
      pos.push(len / 2, Math.sin(a) * r, Math.cos(a) * r);
    }
    for (let i = 0; i < SEG; i++) {
      const a = i * 2, b = a + 1, c = a + 2, d = a + 3;
      idx.push(a, c, b, b, c, d);
    }
    const tube = new BufferGeometry();
    tube.setAttribute('position', new BufferAttribute(new Float32Array(pos), 3));
    tube.setIndex(idx);
    tube.computeVertexNormals();
    const hull = new Mesh(tube, shell);
    hull.receiveShadow = true;
    grp.add(hull);

    // Rack faces down each side, which is what actually narrows the usable
    // corridor: Destiny's shell is 4.27 m across but the free lane between the
    // racks is nearer 2 m, and that is the number a glide has to fit through.
    const rackDepth = Math.min(0.62, r * 0.38);
    for (const s2 of [-1, 1]) {
      for (const yy of [-1, 1]) {
        const face = new Mesh(this._quad(len, r * 0.9), rackMat);
        face.position.set(0, yy * (r - rackDepth) * 0.72, s2 * (r - rackDepth));
        face.rotation.y = s2 > 0 ? 0 : Math.PI;
        grp.add(face);
      }
    }

    // Bulkheads, with a hatch through each — the module is a place you pass
    // through, and a sealed tube reads as a dead end.
    for (const s2 of [-1, 1]) {
      const cap = new Mesh(this._quad(dims.diameter, dims.diameter), shell);
      cap.position.set(s2 * len / 2, 0, 0);
      cap.rotation.y = Math.PI / 2;
      grp.add(cap);
      const hatch = new Mesh(this._quad(0.8, 0.8), new MeshStandardMaterial({
        color: 0x2a2f36, roughness: 0.9, side: DoubleSide }));
      hatch.position.set(s2 * (len / 2 - 0.01), 0, 0);
      hatch.rotation.y = Math.PI / 2;
      grp.add(hatch);
    }

    // Handrails along both sides. These are the only things in a module a crew
    // member actually touches to move, which is what the scenarios are about.
    const railMat = new MeshStandardMaterial({ color: 0xd8c07a, metalness: 0.6, roughness: 0.32 });
    for (const s2 of [-1, 1]) {
      const rail = new Mesh(this._tube(0.021, len * 0.92), railMat);
      // _tube runs along z, so turn it onto the module axis.
      rail.rotation.y = Math.PI / 2;
      rail.position.set(0, -r * 0.35, s2 * (r - rackDepth - 0.12));
      grp.add(rail);
    }

    // Centre the module on the run, not on the world origin.
    //
    // The clips start whereever their packet put them — BrakeGap sweeps x from
    // -0.07 to 4.16 — so a shell built about the origin has the robot leaving
    // it through the side and the camera standing in the racks. The module goes
    // where the robots are.
    const swept = this._sweptBounds();
    if (!swept.isEmpty()) {
      swept.getCenter(_pt);
      grp.position.set(_pt.x, _pt.y, _pt.z);
    }
    // Cross-section the camera has to stay inside, so a solved shot does not
    // end up outside a closed shell looking at its back faces. Stage.js does
    // the same thing for the corridor, and for the same reason.
    // `free` is the half-width of the corridor BETWEEN the racks, which is the
    // space anything actually moves through — Destiny's shell is 4.27 m across
    // and its free lane is nearer 3 m.
    this.moduleBounds = { centre: grp.position.clone(), r, len,
                          free: Math.max(0.4, r - rackDepth - 0.2) };

    this.module = grp;
    this.world.add(grp);
  }

  _quad(w, h) {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array([
      -w / 2, -h / 2, 0, w / 2, -h / 2, 0, w / 2, h / 2, 0, -w / 2, h / 2, 0]), 3));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    g.computeVertexNormals();
    return g;
  }

  /** A rail along Z, since the module axis here runs across the travel path. */
  _tube(r, len, seg = 14) {
    const pos = [], idx = [];
    for (let i = 0; i <= seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      pos.push(Math.cos(a) * r, Math.sin(a) * r, -len / 2);
      pos.push(Math.cos(a) * r, Math.sin(a) * r, len / 2);
    }
    for (let i = 0; i < seg; i++) {
      const a = i * 2, b = a + 1, c = a + 2, d = a + 3;
      idx.push(a, c, b, b, c, d);
    }
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(pos), 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
  }

  // -------------------------------------------------------------------------
  // scene switching
  // -------------------------------------------------------------------------
  /**
   * Show one place, playing one motion.
   *
   * `motion` is 'compare' for the two packet runs this viewer was built
   * around, the id of a generated motion (lope / bound / trip), or — inside a
   * module, where there is no ground and therefore no gait — the id of one of
   * the microgravity scenarios.
   *
   * A generated motion has ONE clip, and that is deliberate. The packet view
   * compares two CONTROLLERS on one body; a generated motion is a statement
   * about the body itself, and the thing it should be compared against is the
   * same robot on Mars, which is not standing on this terrain. So the second
   * lane is empty and the readout carries the comparison instead.
   */
  async load(place, motion = 'compare', thrust = this.thrust ?? 1) {
    this.ready = false;
    this.place = place;
    this.motionKey = motion;
    this.thrust = thrust;

    const { scene, field } = place.micro
      ? await loadScene(motion)
      : await loadPlace(place.id, motion, thrust);
    this.scene_ = scene;
    this.field = field;

    // Clear first. Switching from a two-clip comparison to a one-clip motion
    // used to leave the previous B player in place, so PragyaSpace went on
    // walking its old traverse beside a robot that was now hopping.
    for (const k of CLIP_KEYS) this.players[k] = null;
    for (const k of CLIP_KEYS) {
      if (scene.clips[k]) this.players[k] = new ClipPlayer(scene.clips[k]);
    }

    // Robots are expensive to load; do it once and reuse across scenes.
    if (!this.robots.A) {
      const def = robotById('g1');
      const [a, b] = await Promise.all([loadRobot(def), loadRobot(def)]);
      this.robots.A = a; this.robots.B = b;
      for (const k of CLIP_KEYS) {
        // The loader lifts the model so its soles sit on y=0. These clips carry
        // an absolute pelvis height solved against the terrain, so that lift
        // has to come back off or the whole robot rides high by a sole.
        this.robots[k].root.children[0].position.y = 0;
        this.world.add(this.robots[k].root);
      }
      this._tintB();
    }
    for (const k of CLIP_KEYS) {
      if (this.robots[k]) this.robots[k].root.visible = !!this.players[k];
    }

    // Lane axis: perpendicular to the traverse, so the two runs sit abreast
    // rather than one behind the other whatever direction the course runs.
    const rootA = (scene.clips.A || scene.clips.B).root;
    const d = new Vector3(rootA.at(-1)[0] - rootA[0][0], 0, -(rootA.at(-1)[1] - rootA[0][1]));
    if (d.lengthSq() > 1e-6) this.laneAxis.set(-d.z, 0, d.x).normalize();
    else this.laneAxis.set(0, 0, 1);

    const env = envForBody(scene.body ?? place.body);
    this.env = env;
    // Start as close as the pair can be held from, rather than at a fixed
    // number that was tuned for one separation and one aspect ratio. The
    // clamp in step() is the real floor; this just asks for it.
    this.dist = 0;
    this.applyEnv(env);
    this._updateLanes();
    this.buildTerrain(env);
    this.buildModule(place);
    this.buildMicroSet(scene);
    this.t = 0;
    this.ready = true;
    this.renderUI();
    this.onRateContext?.();
  }

  /** Give PragyaSpace a distinguishable shell so the seam reads instantly. */
  _tintB() {
    this.robots.B.root.traverse((o) => {
      if (!o.isMesh) return;
      o.material = o.material.clone();
      o.material.color = new Color(0x9fd4c4);
    });
    this.robots.A.root.traverse((o) => {
      if (!o.isMesh) return;
      o.material = o.material.clone();
      o.material.color = new Color(0xe2b7a4);
    });
  }

  applyEnv(env) {
    const d = sunDir(env.sunElev, env.sunAz);
    this.sun.position.copy(d).multiplyScalar(60);
    this.sun.color.copy(env.sunColor);
    this.sun.intensity = env.sunIntensity;
    // A pressurised module is uniformly and brightly lit; borrowing an airless
    // surface's near-zero ambient leaves the robot a silhouette against black.
    this.fill.intensity = this.scene_?.micro ? 1.15 : (env.ambientIntensity ?? 0.2);
    this.scene.fog.color.copy(env.fogColor ?? new Color(0, 0, 0));
    this.scene.fog.density = env.fogDensity ?? 0;
    this.scene.environment = this.ibl.update(env);
    this.stars.visible = (env.starIntensity ?? 1) > 0.1;
    this.renderer.toneMappingExposure = env.exposure ?? 1;
  }

  // -------------------------------------------------------------------------
  // frame
  /**
   * The closest distance from which both robots still fit in frame.
   *
   * Only the part of their separation that lies ACROSS the view counts: seen
   * from directly down the lane axis the two are one behind the other and the
   * camera can come in as close as a single body needs, which is what makes
   * orbiting round to the end of the lane a genuinely close look rather than
   * something the clamp fights.
   *
   * The pair is treated as a sphere about the aim point and fitted to the
   * NARROWER of the two half-angles, so it holds in portrait windows too.
   */
  _holdBothDistance() {
    if (!this.paired) return 0;
    const ra = this.robots.A?.root, rb = this.robots.B?.root;
    if (!ra || !rb) return 0;

    // Half the separation, and the direction the camera looks from.
    _half.copy(rb.position).sub(ra.position).multiplyScalar(0.5);
    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    _view.set(Math.cos(this.orbit) * ce, se, Math.sin(this.orbit) * ce).normalize();
    // Drop the component along the view: depth costs no frame.
    _half.addScaledVector(_view, -_half.dot(_view));

    const radius = _half.length() + BODY_RADIUS;
    const halfV = Math.tan((this.camera.fov * Math.PI) / 180 / 2);
    const halfH = halfV * Math.max(0.2, this.camera.aspect);
    return (radius * FRAME_MARGIN) / Math.min(halfV, halfH);
  }

  /**
   * Refuse to draw a foot below the ground.
   *
   * The lane offset itself is no longer a problem — LaneField moves the ground
   * with the robot, so a clip is drawn on exactly the surface it was solved
   * against and the 270 mm of buried machine that displacement used to cause
   * is gone by construction. What is left is a smaller, permanent discrepancy
   * that exists even with the two runs superimposed: 7-19 mm across the twelve
   * surface scenes.
   *
   * That residual is not an error, it is a difference of definition. The
   * retargeter plans against what the URDF says the robot collides with — for
   * the G1, four contact spheres on a sole plane at z = -0.035 — and plants
   * them 4 mm into the regolith on purpose. The shell that is actually DRAWN
   * extends past those spheres, so a foot sitting exactly where it was planned
   * still renders a centimetre or so under the surface, and more as the ankle
   * tilts and the mesh's corner swings below the sphere.
   *
   * A viewer cannot re-solve the chain per frame, but it can lift by the
   * millimetres it takes for nothing visible to be under the ground, which is
   * what this does. visibleSole() supplies the mesh's own extent.
   *
   * Loaded feet only. A swing foot arcing over a boulder is not carrying the
   * robot, and hoisting the whole body to clear something it is passing above
   * would make the pelvis bob at every rock — and would erase the low foot
   * clearance that is one of the two things this page is comparing.
   */
  _seat(r, p) {
    if (!this.field) return;                 // micrograv: nothing to stand on

    const soles = visibleSole(r);
    if (!soles.length) return;
    r.root.updateMatrixWorld(true);

    const ground = (x, z) => this.ground(x, z);
    let lift = 0;
    for (let i = 0; i < soles.length; i++) {
      const foot = r.feet?.[i];
      if (!foot) continue;

      let deep = 0;
      for (const pt of soles[i]) {
        _pt.copy(pt).applyMatrix4(foot.matrixWorld);
        const need = this.ground(_pt.x, _pt.z) - _pt.y;
        if (need > deep) deep = need;
      }
      if (deep <= 0) continue;

      // The clip's contacts say LOADED, unlike the live gait's pose.contacts,
      // which says swinging. jointSaturation() reads them the same way.
      if (p.contacts && !p.contacts[i]) {
        // A swing foot is corrected at its own knee, so the pelvis does not
        // move and the model's foot clearance is left intact.
        clearSwingFoot(r, i, `${SIDES[i]}_knee_joint`, deep, ground);
      } else if (deep > lift) {
        lift = deep;
      }
    }
    if (lift > 0) r.root.position.y += lift;
  }

  // -------------------------------------------------------------------------
  step(dt) {
    if (!this.ready) return;
    if (this._terrainDirty) this.buildTerrain(this.env);
    const dur = this.players.A?.duration ?? this.players.B?.duration ?? 10;
    if (this.playing) this.t = (this.t + dt * this.rate) % dur;

    for (const k of CLIP_KEYS) {
      const p = this.players[k], r = this.robots[k];
      if (!p || !r) continue;
      p.sample(this.t);
      r.root.position.copy(p.pos);
      if (this.mode === 'lanes' && this.paired) {
        const sgn = k === 'A' ? -1 : 1;
        r.root.position.addScaledVector(this.laneAxis, sgn * this.separation * 0.5);
      }
      r.root.quaternion.copy(p.quat);
      for (const [name, v] of Object.entries(p.joints)) {
        const j = r.joints[name];
        if (j) j.setJointValue(v);
      }
      this._seat(r, p);
    }

    // Frame the pair where they ACTUALLY are, lane offsets included — aiming
    // at the un-offset clip positions leaves both robots off to one side.
    const ra = this.players.A ? this.robots.A?.root : null;
    const rb = this.players.B ? this.robots.B?.root : null;
    const mid = new Vector3();
    if (ra && rb) mid.copy(ra.position).add(rb.position).multiplyScalar(0.5);
    else if (ra || rb) mid.copy((ra || rb).position);
    mid.y += 0.5;
    this.aim.lerp(mid, 1 - Math.exp(-dt * 3.5));

    // Never sit closer than the pair can be held from.
    //
    // The camera aims at the MIDPOINT of the two robots, so in side-by-side
    // each of them is half the lane separation off the view axis. Zooming in
    // moves that midpoint toward the screen centre and pushes both humanoids
    // off opposite edges — at the 1.6 m zoom stop with 1.8 m of separation,
    // half the frame is 0.58 m of world and each robot is 0.90 m out, so the
    // close view showed an empty patch of regolith between them.
    const d = Math.max(this.dist, this._holdBothDistance());
    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    this.camera.position.set(
      this.aim.x + Math.cos(this.orbit) * ce * d,
      this.aim.y + se * d + 0.4,
      this.aim.z + Math.sin(this.orbit) * ce * d);
    this._clampIntoModule();
    this.camera.lookAt(this.aim);
    this.sun.target.position.copy(this.aim);
    this.sun.position.copy(sunDir(this.env.sunElev, this.env.sunAz))
      .multiplyScalar(50).add(this.aim);
  }

  /**
   * Keep the camera inside the module.
   *
   * Every shot in this viewer is written for open ground, where standing a few
   * metres to the side is exactly right. Indoors that is through a wall, and
   * since the shell is closed what you get is the OUTSIDE of the module — which
   * is the one thing an interior exists to prevent. So the solved position is
   * pulled back into the cross-section, and the module's own radius is what
   * decides how far it may go.
   */
  _clampIntoModule() {
    const m = this.moduleBounds;
    if (!m) return;
    const c = this.camera.position;

    // The distance the rig asked for. It is preserved, not discarded — the
    // clamp decides WHERE the camera may stand, not how far away it is.
    const want = c.distanceTo(this.aim);

    // Into the free corridor, which is narrower than the shell: the rack faces
    // down each side are what a camera actually collides with, and standing
    // 1.68 m off the axis of a 4.27 m module put it inside one of them, which
    // renders as a flat grey wall filling the frame.
    const dy = c.y - m.centre.y, dz = c.z - m.centre.z;
    const rad = Math.hypot(dy, dz);
    const max = Math.max(0.25, m.free);
    if (rad > max) {
      const k = max / rad;
      c.y = m.centre.y + dy * k;
      c.z = m.centre.z + dz * k;
    }

    // Spend what is left along the tube, because that is the only axis with
    // room in it. A module is 8.5 m long and 4.3 m across, so a shot that wants
    // four metres of standoff can have it — just not sideways.
    const off = Math.hypot(c.y - this.aim.y, c.z - this.aim.z);
    const along = Math.sqrt(Math.max(want * want - off * off, 0.36));
    const dir = Math.sign(c.x - this.aim.x) || -1;
    const half = Math.max(0.3, m.len / 2 - 0.3);
    c.x = clampTo(this.aim.x + dir * along, m.centre.x - half, m.centre.x + half);
  }

  render() {
    const r = this.renderer;
    // CSS pixels, not drawing-buffer pixels.
    //
    // setViewport() and setScissor() take CSS pixels and multiply by the
    // pixel ratio themselves. domElement.width/height are the BUFFER, already
    // multiplied — so at devicePixelRatio 2 this asked for a 5600x3200 viewport
    // on a 2800x1600 buffer, and only the bottom-left quarter of the projected
    // image landed on the canvas, magnified. getSize() reports the CSS size the
    // renderer was actually given.
    r.getSize(_size);
    const w = _size.x, h = _size.y;

    // Lanes: both robots on screen at once, one pass, nothing hidden. A robot
    // with no clip this scene stays hidden — it has nothing to play, and left
    // visible it holds its pose from the previous scene in the middle of the
    // shot.
    if (this.mode === 'lanes' || !this.paired) {
      for (const k of CLIP_KEYS) {
        if (this.robots[k]) this.robots[k].root.visible = !!this.players[k];
      }
      r.setScissorTest(false);
      r.setViewport(0, 0, w, h);
      r.render(this.scene, this.camera);
      return;
    }

    const x = Math.round(w * this.split);
    r.setScissorTest(true);
    r.setViewport(0, 0, w, h);

    // Left of the seam: WorldVLA. Right: PragyaSpace. Scissor confines both the
    // clear and the draw, so one camera renders two robots into one frame with
    // no compositing pass.
    if (this.robots.A) this.robots.A.root.visible = true;
    if (this.robots.B) this.robots.B.root.visible = false;
    r.setScissor(0, 0, x, h);
    r.render(this.scene, this.camera);

    if (this.robots.A) this.robots.A.root.visible = false;
    if (this.robots.B) this.robots.B.root.visible = true;
    r.setScissor(x, 0, w - x, h);
    r.render(this.scene, this.camera);

    r.setScissorTest(false);
    if (this.robots.A) this.robots.A.root.visible = !!this.players.A;
  }

  /**
   * The playback speed at which this scene is within the G1's joint rating.
   *
   * The packets command 4.5-5.7 m of traverse in ten seconds, and a 0.79 m leg
   * cannot swing that fast: mid-swing the commanded foot speed reaches ~5 m/s
   * and the knee is asked for well over its ~500 deg/s rating. It is a brief
   * excess — under 1% of joint-frames — but it is real, and the clip only
   * becomes hardware-legal when slowed.
   *
   * Deliberately OFFERED rather than applied. The packets assert forward
   * distances that hold at 1.00x, and quietly slowing playback would make the
   * motion legal while falsifying a number its authors published. The binding
   * constraint is the slower of the two clips, since both share one clock.
   */
  feasibleRate() {
    const rs = CLIP_KEYS.map((k) => this.scene_?.clips?.[k]?.rates?.feasiblePlayback)
      .filter((v) => typeof v === 'number' && v > 0);
    return rs.length ? Math.min(...rs) : null;
  }

  // -------------------------------------------------------------------------
  // UI
  // -------------------------------------------------------------------------
  renderUI() {
    const s = this.scene_;
    if (!s) return;
    const m = s.terrain?.meta;
    const clip = (k) => s.clips[k] || {};
    const au = (k) => clip(k).audit?.after || clip(k).comAudit?.after || {};
    const num = (v, d = 2, suf = '') => (v === undefined || v === null ? '—' : v.toFixed(d) + suf);

    const stat = (k) => {
      const a = au(k);
      if (s.micro) {
        return `<div class="row"><span>COM off its line</span><b>${num(a.maxDev * 1000, 3, ' mm')}</b></div>`
             + `<div class="row"><span>unphysical force</span><b>${num(a.ghostForce, 3, ' N')}</b></div>`;
      }
      return `<div class="row"><span>swing reach</span><b>${num(a.swingNet, 2, ' m')}</b></div>`
           + `<div class="row"><span>slip while loaded</span><b>${num(a.schedSlipFrac * 100, 1, ' %')}</b></div>`
           + `<div class="row"><span>sole into ground</span><b>${num(a.penetration * 1000, 1, ' mm')}</b></div>`;
    };

    // One label for one robot. Leaving "PRAGYASPACE" in the corner of a scene
    // that is not playing a PragyaSpace clip is simply a false caption.
    this.ui.querySelector('#titles').innerHTML = this.players.B
      ? `<div class="lbl a"><i></i>${clip('A').label || 'WorldVLA'}</div>
         <div class="lbl b">${clip('B').label || 'PragyaSpace'}<i></i></div>`
      : `<div class="lbl a"><i></i>${clip('A').label || ''}</div>`;

    const gen = s.generated ? this._physics(s) : '';
    this.ui.querySelector('#panel').innerHTML = `
      <h1>${this.place?.micro ? this.place.name : s.name}</h1>
      <div class="sub">${s.body} · g = ${s.g.toFixed(2)} m/s²${s.micro ? ' · free fall' : ''}</div>
      <p class="blurb">${(s.generated ? clip('A').blurb : s.blurb) || s.blurb || ''}</p>
      ${gen || `<div class="cols">
        <div class="col a"><h2>${clip('A').label || 'A'}</h2>${stat('A')}</div>
        <div class="col b"><h2>${clip('B').label || 'B'}</h2>${stat('B')}</div>
      </div>`}
      ${s.micro ? '' : this._hardwareNote(s)}
      ${this.place?.micro ? this._moduleNote() : ''}
      ${m ? this._provenance(m) : ''}`;
  }

  /**
   * What gravity is doing to this motion, and what it would do elsewhere.
   *
   * The whole reason the generated motions exist. A packet walk looks nearly
   * the same on the Moon and on Mars because it was authored once and
   * re-planted twice; these were not authored at all. Given `g`, the URDF's own
   * knee effort and velocity limits and the link geometry, src/sim/Ballistic.js
   * works out how fast the machine can leave the ground and everything else
   * follows — so the figures below are computed, and the Moon column is not a
   * multiple anybody typed.
   *
   * The last row is the one that surprises people, and it is why the Apollo
   * crews loped rather than ran. Forward acceleration comes from friction and
   * friction comes from weight, so one sixth g does not make you fast — it
   * makes you SLOW, and leaves you a two-and-a-half second flight phase to
   * spend on getting nowhere in particular.
   */
  _physics(s) {
    const p = s.physics;
    if (!p) return '';
    const here = s.body === 'Mars' ? 'Mars' : 'Moon';
    const other = here === 'Moon' ? 'Mars' : 'Moon';
    const og = here === 'Moon' ? 3.721 : 1.625;
    const ratio = s.g / og;

    // The other body's figures, from the same thrust. v0 is not held fixed any
    // more — with thrust as the input it rises as gravity falls, because the
    // same force has less weight to fight — so the comparison is made through
    // the model rather than by scaling.
    const o = otherBody(og, p);
    const stance = (p.duty * p.hang) / Math.max(1 - p.duty, 1e-6);
    const oDuty = stance / (stance + o.hang);

    const row = (label, a, b, unit, d = 2) =>
      `<div class="row"><span>${label}</span><b>${a.toFixed(d)}${unit}</b>` +
      `<u>${b.toFixed(d)}${unit}</u></div>`;
    const deg = (label, a, b) =>
      `<div class="row"><span>${label}</span><b>${(a * 180 / Math.PI).toFixed(0)}°</b>` +
      `<u>${(b * 180 / Math.PI).toFixed(0)}°</u></div>`;

    const idx = p.index ?? 0;
    const verdict = idx <= 1 ? 'HOLDS' : idx <= 4 ? 'STAGGERS' : 'GOES OVER';
    const cls = idx <= 1 ? 'ok' : idx <= 4 ? 'warn' : 'bad';

    return `<div class="phys">
      <div class="head"><span>thrust ${(p.thrust ?? 1).toFixed(2)}× · ${Math.round(p.force)} N</span>
        <b>${here.toUpperCase()}</b><u>${other.toUpperCase()}</u></div>
      ${row('jump height', p.apex, o.apex, ' m')}
      ${row('time in the air', p.hang, o.hang, ' s')}
      ${row('fraction on the ground', p.duty, oDuty, '')}
      ${row('top speed', p.speedCeiling, p.speedCeiling * (og / s.g), ' m/s')}

      <div class="head sub"><span>what the thrust does to attitude</span><b></b><u></u></div>
      ${deg('tips in flight', p.tumble, o.tumble)}
      ${deg('arms take back', p.armAuthority, o.armAuthority)}
      ${deg('ankle can take back', p.correctable, o.correctable)}
      <div class="row verdict ${cls}"><span>instability index</span>
        <b>${idx.toFixed(1)}</b><u>${o.index.toFixed(1)}</u></div>
      <div class="verdictline ${cls}">${verdict}</div>

      <p class="why">The push misses the centre of mass by
      <b>${((p.offset ?? 0) * 1000).toFixed(0)} mm</b>. In free flight nothing can
      stop the rotation that starts, so it runs for the whole
      <b>${p.hang.toFixed(2)} s</b>; the arms claw back
      ${(ARM_SHARE * 100).toFixed(0)}% of their own sweep by conservation, and on
      the ground the ankle can only push as hard as
      <b>m·g·d = ${p.ankleTorqueLimit.toFixed(1)} N·m</b> before the foot tips off
      its own edge. That last limit is gravitational, not mechanical — at one
      sixth g the motors are unchanged and the authority is not.</p>
    </div>`;
  }

  /** The module's real size, which is the only thing that varies between them. */
  _moduleNote() {
    const m = this.place?.module;
    if (!m) return '';
    return `<div class="note"><b>${m.length.toFixed(2)} m long, ${m.diameter.toFixed(2)} m across.</b>
      Published pressurised dimensions, drawn at scale and not fitted to the
      clip — the point of an interior is to give the eye a known measurement to
      put beside a 1.32 m robot. The scenario is the same in every module; how
      much room there is to be wrong in is not.</div>`;
  }

  /**
   * Call out when the ANKLE, not the solver, is what limits the foot.
   *
   * On the steep sites the ankle runs out of dorsiflexion: it sits on its -50
   * degree stop, the sole can no longer lie flat against the grade, and the
   * foot rests on its heel edge. Without saying so this reads as feet failing
   * to sit properly on the ground — a rendering fault. It is the opposite: it
   * is the hardware limit that makes steep ground hard, and the reason a
   * sensible route zig-zags rather than going straight up.
   */
  _hardwareNote(s) {
    // A generated motion carries the figure directly, measured while it was
    // being solved. Here it is ROLL rather than pitch that runs out, because
    // what a hop meets on a steep site is a cross-slope: the sole cannot lie
    // flat across the grade, the foot rests on one edge, and no amount of
    // seating will change it. On the Copernicus wall that is 100 % of loaded
    // frames with the ankle exactly on its 15 degree stop.
    const roll = s.clips?.A?.ik?.ankleRollSaturated;
    if (roll !== undefined) {
      if (roll < 0.15) return '';
      return `<div class="note"><b>Ankle at its roll limit.</b> On
        ${Math.round(roll * 100)}% of loaded frames the ankle is against its
        15° roll stop and the sole cannot lie flat across the grade, so the
        foot rests on an edge. That is the hardware running out, not the
        contact solver — and it is why a sensible route across a steep face
        traverses rather than attacking it square on.</div>`;
    }

    const worst = CLIP_KEYS
      .map((k) => ['left', 'right'].map((side) =>
        ({ k, side, r: jointSaturation(s.clips[k], `${side}_ankle_pitch_joint`) })))
      .flat().filter((x) => x.r).sort((a, b) => b.r.fracLow - a.r.fracLow)[0];
    if (!worst || worst.r.fracLow < 0.15) return '';
    return `<div class="note"><b>Ankle at its limit.</b> On
      ${Math.round(worst.r.fracLow * 100)}% of loaded frames the ankle is
      against its ${Math.round(worst.r.lo * 180 / Math.PI)}° dorsiflexion stop
      and the sole cannot lie flat on the grade. That is the hardware running
      out, not the contact solver — and it is why steep ground is normally
      taken in switchbacks.</div>`;
  }

  /**
   * Say what the data actually is.
   *
   * The caption quotes NATIVE_MPP — the source product's real ground sample
   * distance — not the grid spacing the patch was resampled to. The resampled
   * figure is finer and quoting it would claim resolution the observation does
   * not have. The synthesised sub-metre relief is called out for the same
   * reason: it is what the feet contact, and it is not NASA's.
   */
  _provenance(m) {
    const px = (v) => (v === undefined ? '—' : v < 1.01 ? `${v.toFixed(2)} m/px` : `${Math.round(v)} m/px`);
    const native = m.native_mpp ?? m.mpp;
    const n = m.dem_samples_across;
    return `<div class="prov">
      <b>Terrain</b> ${m.citation}<br>
      Source resolution ${px(native)}${m.native_mpp && m.mpp && m.mpp < m.native_mpp
        ? ` · resampled to a ${px(m.mpp)} grid` : ''}<br>
      ${this._extent(m)} · lat ${Number(m.lat).toFixed(4)}, lon ${Number(m.lon).toFixed(4)}<br>
      ${this._fidelity(n)}
      ${m.resolution_note ? `<em>${m.resolution_note}</em>` : ''}</div>`;
  }

  /** The patch's size on the ground, whichever grid it was fetched on. */
  _extent(m) {
    if (m.span_x_m && m.span_y_m) {
      const km = (v) => (v >= 1000 ? `${(v / 1000).toFixed(1)} km` : `${Math.round(v)} m`);
      return `${km(m.span_x_m)} x ${km(m.span_y_m)} patch, long axis along the traverse`;
    }
    return `${m.span_m} m patch`;
  }

  /**
   * How much of what you are looking at was actually measured.
   *
   * The grid is 512 wide on every site, which tells you nothing — what matters
   * is how many REAL source posts the patch spans. Gale and Shackleton span
   * 128; five of the lunar sites span 34; five of the six Mars sites span ten.
   * Below roughly twenty posts the DEM contributes a slope and a broad landform
   * and nothing else, and every feature the foot actually meets belongs to the
   * synthetic layer.
   *
   * Stated plainly and per-scene, because the alternative — one blanket
   * disclaimer — lets a 10-post site borrow the credibility of a 128-post one.
   */
  _fidelity(n) {
    if (n === undefined) {
      return `<em>Relief below the source resolution is synthesised.</em><br>`;
    }
    const posts = `<b>${n >= 10 ? Math.round(n) : n.toFixed(1)}</b> source posts across the patch`;
    if (n >= 100) {
      return `<span class="fid hi">MEASURED</span> ${posts} — real landforms at
        walking scale; sub-metre relief is synthesised.<br>`;
    }
    if (n >= 20) {
      return `<span class="fid mid">PARTLY MEASURED</span> ${posts} — the broad
        landform is real; everything the foot meets is synthesised.<br>`;
    }
    return `<span class="fid lo">SLOPE ONLY</span> ${posts} — the DEM supplies
      this site's true grade and elevation and nothing finer. All terrain
      detail you can see at this range is synthesised.<br>`;
  }
}

/**
 * Boot: wire the DOM, build the picker, load the first place, run.
 *
 * The picker is three levels, because the question has three parts: WHICH BODY
 * (which decides the gravity), WHICH PLACE on it (which decides the ground),
 * and WHICH MOTION (which decides what the robot is trying to do there). The
 * old picker was a single flat list of fourteen scene ids, which conflated all
 * three and could not have held forty places.
 */
export async function startArena(canvas, ui) {
  const arena = new Arena(canvas, ui);
  const index = await loadPlaces();
  if (!index?.places?.length) {
    ui.querySelector('#panel').innerHTML =
      '<h1>Nothing built</h1><p class="blurb">Run <code>.venv/bin/python pipeline/dem.py site</code>, '
      + 'then <code>node tools/build_all.mjs &amp;&amp; node tools/build_motions.mjs &amp;&amp; '
      + 'node tools/build_index.mjs</code>.</p>';
    return arena;
  }

  const tabsEl = ui.querySelector('#tabs');
  const placesEl = ui.querySelector('#places');
  const motionsEl = ui.querySelector('#motions');
  // Looked up here rather than beside the handler below, because applyMode()
  // reads thrustWrap and is called while the mode toggle is being wired —
  // which is before the handler's own declarations would have run.
  const thrustWrap = ui.querySelector('#thrustwrap');
  const thrustInput = ui.querySelector('#thrust');
  const thrustVal = ui.querySelector('#thrustval');

  const bodies = index.bodies.filter((b) => index.places.some((p) => p.body === b));
  let body = bodies[0];
  let place = null;
  let motion = 'compare';

  const placesOf = (b) => index.places.filter((p) => p.body === b);

  /**
   * What can be played at this place.
   *
   * A module offers the microgravity scenarios and nothing else — there is no
   * ground, so there is no gait and a hop has no meaning. A surface place
   * offers the packet comparison where a packet exists, plus every generated
   * motion the build produced for it.
   */
  const motionsOf = (p) => {
    if (!p) return [];
    if (p.micro) return index.microScenes.map((m) => ({ id: m.id, label: m.name.split(' — ')[0], title: m.blurb }));
    const out = [];
    if (p.compare) {
      out.push({ id: 'compare', label: 'A / B PACKETS',
                 title: 'WorldVLA against PragyaSpace, both walking the same authored traverse.' });
    }
    for (const m of p.motions) {
      out.push({ id: m, label: MOTION_LABEL[m] || m.toUpperCase(),
                 title: MOTION_TITLE[m] || '' });
    }
    return out;
  };

  const renderTabs = () => {
    tabsEl.innerHTML = bodies.map((b) =>
      `<button data-body="${b}" class="${b === body ? 'on' : ''}">${b.toUpperCase()}`
      + `<i>${placesOf(b).length}</i></button>`).join('');
  };

  const renderPlaces = () => {
    placesEl.innerHTML = placesOf(body).map((p) => {
      // Say how much of the ground under a place was actually measured, right
      // on the button — the alternative is a picker where a 1 m/px HiRISE site
      // and a 200 m/px global-blend site look equally authoritative.
      const t = p.terrain;
      const tag = p.micro ? `${p.module.length.toFixed(1)} m`
        : t ? `${Math.round(t.posts)} posts` : '';
      return `<button data-id="${p.id}" class="${p.id === place?.id ? 'on' : ''}"`
        + ` title="${(p.name || '').replace(/"/g, '&quot;')}">${p.place}<i>${tag}</i></button>`;
    }).join('');
  };

  const renderMotions = () => {
    const list = motionsOf(place);
    motionsEl.innerHTML = list.map((m) =>
      `<button data-motion="${m.id}" class="${m.id === motion ? 'on' : ''}"`
      + ` title="${m.title.replace(/"/g, '&quot;')}">${m.label}</button>`).join('');
    motionsEl.style.display = list.length > 1 ? '' : 'none';
  };

  const show = async (p, m) => {
    place = p;
    const avail = motionsOf(p).map((x) => x.id);
    motion = avail.includes(m) ? m : avail[0];
    renderTabs(); renderPlaces(); renderMotions();
    await arena.load(place, motion);
    applyMode();
  };

  tabsEl.addEventListener('click', async (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    body = b.dataset.body;
    // Keep the motion across a body change when the new place can play it, so
    // clicking MOON then MARS while watching a bound compares the same thing.
    await show(placesOf(body)[0], motion);
  });
  placesEl.addEventListener('click', async (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    await show(index.places.find((p) => p.id === b.dataset.id), motion);
  });
  motionsEl.addEventListener('click', async (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    await show(place, b.dataset.motion);
  });

  const modeBtn = ui.querySelector('#mode');
  const seam = ui.querySelector('#seam');
  const sepWrap = ui.querySelector('#sepwrap');
  const applyMode = () => {
    // With a single generated motion there is no pair to separate or wipe
    // between, so the controls that only mean something for a comparison are
    // taken away rather than left to do nothing.
    const pair = !!(arena.players.A && arena.players.B);
    modeBtn.style.display = pair ? '' : 'none';
    // A packet clip is a recording; there is no thrust to turn up in it.
    thrustWrap.style.display = arena.scene_?.generated ? '' : 'none';
    const lanes = arena.mode === 'lanes';
    modeBtn.textContent = lanes ? 'SIDE BY SIDE' : 'OVERLAY + WIPE';
    modeBtn.title = lanes
      ? 'Both robots, in separate lanes. Click for superimposed pose comparison.'
      : 'Superimposed, revealed by the wipe. Click to separate them.';
    seam.style.display = (pair && !lanes) ? '' : 'none';
    sepWrap.style.display = (pair && lanes) ? '' : 'none';
  };
  modeBtn.addEventListener('click', () => {
    arena.mode = arena.mode === 'lanes' ? 'wipe' : 'lanes';
    applyMode();
    arena._updateLanes();
  });
  ui.querySelector('#sep').addEventListener('input', (e) => {
    arena.separation = Number(e.target.value);
    ui.querySelector('#sepval').textContent = `${arena.separation.toFixed(1)} m`;
    arena._updateLanes();
  });
  applyMode();

  /**
   * THRUST, live.
   *
   * The slider re-solves the whole clip through src/sim/HopMotion.js — the
   * same code that baked it — rather than scaling a canned animation, because
   * what thrust changes is not the size of the motion but its whole structure:
   * a harder push leaves faster, hangs longer, spends less of the cycle on the
   * ground, and arrives with more attitude error than the ankle can take out.
   * None of that is a multiplier on anything.
   *
   * Debounced to the next frame. Dragging fires input events far faster than a
   * three-hundred-frame solve, and queueing them all would put the viewer
   * seconds behind the slider.
   */
  let thrustPending = null, thrustBusy = false;
  const applyThrust = async () => {
    if (thrustBusy || thrustPending === null) return;
    thrustBusy = true;
    const want = thrustPending; thrustPending = null;
    try { await arena.load(place, motion, want); } catch (e) { console.error(e); }
    thrustBusy = false;
    if (thrustPending !== null) applyThrust();
  };
  thrustInput.addEventListener('input', (e) => {
    const v = Number(e.target.value);
    thrustVal.textContent = `${v.toFixed(2)}×`;
    thrustPending = v;
    requestAnimationFrame(applyThrust);
  });

  const play = ui.querySelector('#play');
  play.addEventListener('click', () => {
    arena.playing = !arena.playing;
    play.textContent = arena.playing ? '❚❚' : '▶';
  });
  const rateInput = ui.querySelector('#rate');
  const rateVal = ui.querySelector('#rateval');
  const g1Btn = ui.querySelector('#g1rate');

  /**
   * Show whether what is on screen is a speed the hardware could actually do.
   * 1.00x is "as the packet commands it" and stays the default; the button
   * offers the slower, hardware-legal reading of the same clip.
   */
  const refreshRate = () => {
    const f = arena.feasibleRate();
    rateVal.textContent = `${arena.rate.toFixed(2)}×`;
    if (f === null) { g1Btn.style.display = 'none'; return; }
    g1Btn.style.display = '';
    const legal = arena.rate <= f + 1e-3;
    g1Btn.textContent = legal ? `WITHIN G1 RATING` : `G1 REAL-TIME ${f.toFixed(2)}×`;
    g1Btn.classList.toggle('ok', legal);
    g1Btn.title = legal
      ? `Every joint is inside the G1's ~500 °/s rating at this speed.`
      : `This clip exceeds the G1's ~500 °/s joint rating on ` +
        `${(Math.max(...CLIP_KEYS.map((k) => arena.scene_?.clips?.[k]?.rates?.overFrac ?? 0)) * 100).toFixed(2)}% ` +
        `of joint-frames. Click to play it at ${f.toFixed(2)}×, where it is hardware-legal. ` +
        `Note the packets' stated distances hold at 1.00×.`;
  };
  arena.onRateContext = refreshRate;

  rateInput.addEventListener('input', (e) => {
    arena.rate = Number(e.target.value);
    refreshRate();
  });
  g1Btn.addEventListener('click', () => {
    const f = arena.feasibleRate();
    if (f === null) return;
    arena.rate = arena.rate <= f + 1e-3 ? 1 : f;   // toggle back to as-authored
    rateInput.value = String(arena.rate);
    refreshRate();
  });
  refreshRate();
  addEventListener('keydown', (e) => {
    if (e.key === ' ') { e.preventDefault(); play.click(); }
    if (e.key === 'ArrowRight') arena.t += 1 / 30;
    if (e.key === 'ArrowLeft') arena.t = Math.max(0, arena.t - 1 / 30);
  });

  await show(placesOf(body)[0], 'compare');

  const scrub = ui.querySelector('#scrub');
  let last = performance.now();
  const loop = (now) => {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    arena.step(dt);
    arena.render();
    if (arena.mode === 'wipe') seam.style.left = `${arena.split * 100}%`;
    const dur = arena.players.A?.duration || 10;
    scrub.value = String((arena.t / dur) * 1000);
    requestAnimationFrame(loop);
  };
  scrub.addEventListener('input', (e) => {
    const dur = arena.players.A?.duration || 10;
    arena.t = (Number(e.target.value) / 1000) * dur;
  });
  requestAnimationFrame(loop);
  return arena;
}
