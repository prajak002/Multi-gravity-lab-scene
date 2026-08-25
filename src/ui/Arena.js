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
import { loadScene, discoverScenes, ClipPlayer, jointSaturation } from '../scenes/SceneLoader.js';

const CLIP_KEYS = ['A', 'B'];

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
      this.dist = Math.max(1.6, Math.min(40, this.dist * (1 + Math.sign(e.deltaY) * 0.12)));
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
        pos[k] = x; pos[k + 1] = this.field.heightAt(x, z); pos[k + 2] = z;
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

  buildTerrain(env) {
    if (this.terrain) { this.world.remove(this.terrain); this.terrain = null; }
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

    const span = Math.min(this.field.half * 2 - 2, 380);
    const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2;
    const far = new Mesh(this._grid(cx - span / 2, cx + span / 2, cz - span / 2, cz + span / 2,
                                    span / 240, [x0, x1, z0, z1]), mat);
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

    const wall = new Mesh(this._quad(1.7, 1.7), panel());
    wall.position.copy(start);
    wall.position.x -= dir * 0.35;
    wall.rotation.y = Math.PI / 2;
    grp.add(wall);

    const far = new Mesh(this._quad(1.9, 1.9), panel());
    far.position.copy(target);
    far.position.x += dir * 0.42;
    far.position.y += 0.1;
    far.rotation.y = Math.PI / 2;
    grp.add(far);

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
  async load(id) {
    this.ready = false;
    const { scene, field } = await loadScene(id);
    this.scene_ = scene;
    this.field = field;

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

    // Lane axis: perpendicular to the traverse, so the two runs sit abreast
    // rather than one behind the other whatever direction the course runs.
    const rootA = (scene.clips.A || scene.clips.B).root;
    const d = new Vector3(rootA.at(-1)[0] - rootA[0][0], 0, -(rootA.at(-1)[1] - rootA[0][1]));
    if (d.lengthSq() > 1e-6) this.laneAxis.set(-d.z, 0, d.x).normalize();
    else this.laneAxis.set(0, 0, 1);

    const env = envForBody(scene.body);
    this.env = env;
    this.dist = scene.micro ? 5.6 : 7.4;
    this.applyEnv(env);
    this.buildTerrain(env);
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
  // -------------------------------------------------------------------------
  step(dt) {
    if (!this.ready) return;
    const dur = this.players.A?.duration ?? this.players.B?.duration ?? 10;
    if (this.playing) this.t = (this.t + dt * this.rate) % dur;

    for (const k of CLIP_KEYS) {
      const p = this.players[k], r = this.robots[k];
      if (!p || !r) continue;
      p.sample(this.t);
      r.root.position.copy(p.pos);
      if (this.mode === 'lanes') {
        const sgn = k === 'A' ? -1 : 1;
        r.root.position.addScaledVector(this.laneAxis, sgn * this.separation * 0.5);
      }
      r.root.quaternion.copy(p.quat);
      for (const [name, v] of Object.entries(p.joints)) {
        const j = r.joints[name];
        if (j) j.setJointValue(v);
      }
    }

    // Frame the pair where they ACTUALLY are, lane offsets included — aiming
    // at the un-offset clip positions leaves both robots off to one side.
    const ra = this.robots.A?.root, rb = this.robots.B?.root;
    const mid = new Vector3();
    if (ra && rb) mid.copy(ra.position).add(rb.position).multiplyScalar(0.5);
    else mid.copy((ra || rb).position);
    mid.y += 0.5;
    this.aim.lerp(mid, 1 - Math.exp(-dt * 3.5));

    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    this.camera.position.set(
      this.aim.x + Math.cos(this.orbit) * ce * this.dist,
      this.aim.y + se * this.dist + 0.4,
      this.aim.z + Math.sin(this.orbit) * ce * this.dist);
    this.camera.lookAt(this.aim);
    this.sun.target.position.copy(this.aim);
    this.sun.position.copy(sunDir(this.env.sunElev, this.env.sunAz))
      .multiplyScalar(50).add(this.aim);
  }

  render() {
    const w = this.renderer.domElement.width, h = this.renderer.domElement.height;
    const r = this.renderer;

    // Lanes: both robots on screen at once, one pass, nothing hidden.
    if (this.mode === 'lanes') {
      if (this.robots.A) this.robots.A.root.visible = true;
      if (this.robots.B) this.robots.B.root.visible = true;
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
    if (this.robots.A) this.robots.A.root.visible = true;
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

    this.ui.querySelector('#titles').innerHTML = `
      <div class="lbl a"><i></i>${clip('A').label || 'WorldVLA'}</div>
      <div class="lbl b">${clip('B').label || 'PragyaSpace'}<i></i></div>`;

    this.ui.querySelector('#panel').innerHTML = `
      <h1>${s.name}</h1>
      <div class="sub">${s.body} · g = ${s.g.toFixed(2)} m/s²${s.micro ? ' · free fall' : ''}</div>
      <p class="blurb">${s.blurb || ''}</p>
      <div class="cols">
        <div class="col a"><h2>${clip('A').label || 'A'}</h2>${stat('A')}</div>
        <div class="col b"><h2>${clip('B').label || 'B'}</h2>${stat('B')}</div>
      </div>
      ${s.micro ? '' : this._hardwareNote(s)}
      ${m ? this._provenance(m) : ''}`;
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
      ${m.span_m} m patch · lat ${Number(m.lat).toFixed(4)}, lon ${Number(m.lon).toFixed(4)}<br>
      ${this._fidelity(n)}
      ${m.resolution_note ? `<em>${m.resolution_note}</em>` : ''}</div>`;
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

/** Boot: wire the DOM, load the first scene, run. */
export async function startArena(canvas, ui) {
  const arena = new Arena(canvas, ui);
  const ids = await discoverScenes();
  if (!ids.length) {
    ui.querySelector('#panel').innerHTML =
      '<h1>No scenes built</h1><p class="blurb">Run <code>node tools/build_scene.mjs &lt;id&gt;</code> first.</p>';
    return arena;
  }

  const picker = ui.querySelector('#scenes');
  picker.innerHTML = ids.map((id, i) =>
    `<button data-id="${id}"${i === 0 ? ' class="on"' : ''}>${id.replace(/_/g, ' ')}</button>`).join('');
  picker.addEventListener('click', async (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    picker.querySelectorAll('button').forEach((x) => x.classList.toggle('on', x === b));
    await arena.load(b.dataset.id);
  });

  // Mode toggle: two robots side by side, or superimposed under a wipe.
  const modeBtn = ui.querySelector('#mode');
  const seam = ui.querySelector('#seam');
  const sepWrap = ui.querySelector('#sepwrap');
  const applyMode = () => {
    const lanes = arena.mode === 'lanes';
    modeBtn.textContent = lanes ? 'SIDE BY SIDE' : 'OVERLAY + WIPE';
    modeBtn.title = lanes
      ? 'Both robots, in separate lanes. Click for superimposed pose comparison.'
      : 'Superimposed, revealed by the wipe. Click to separate them.';
    seam.style.display = lanes ? 'none' : '';
    sepWrap.style.display = lanes ? '' : 'none';
  };
  modeBtn.addEventListener('click', () => {
    arena.mode = arena.mode === 'lanes' ? 'wipe' : 'lanes';
    applyMode();
  });
  ui.querySelector('#sep').addEventListener('input', (e) => {
    arena.separation = Number(e.target.value);
    ui.querySelector('#sepval').textContent = `${arena.separation.toFixed(1)} m`;
  });
  applyMode();

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

  await arena.load(ids[0]);

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
