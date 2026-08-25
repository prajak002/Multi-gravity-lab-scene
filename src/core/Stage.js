/**
 * Stage — renderer, lighting, and the camera that carries the cinematics.
 *
 * The camera is driven by named SHOTS rather than by direct writes. Entering a
 * scene, switching robot, or pressing a shot key all do the same thing: hand
 * the rig a target shot and a duration, and let it fly there. That is what
 * keeps every transition in the piece feeling like one camera operator instead
 * of a cut.
 */
import {
  WebGLRenderer, Scene, PerspectiveCamera, DirectionalLight, HemisphereLight,
  Vector3, Color, FogExp2, ACESFilmicToneMapping, PCFSoftShadowMap, Group,
  BufferGeometry, BufferAttribute, Points, PointsMaterial, MathUtils,
} from 'three';
import { IBL } from '../render/IBL.js';

/**
 * Shots are expressed RELATIVE to the subject: an offset from it and a
 * look-at height, so the same definition works wherever the robot has walked
 * to and whichever robot it is.
 */
export const SHOTS = {
  // wide, high, sun-side — the establishing frame the lobby sits against
  establish: { offset: new Vector3(-8.5, 4.2, 8.5), aim: 0.85, fov: 40 },
  // the working view: side-on and close enough to read a gait
  chase:     { offset: new Vector3(-2.6, 1.05, 2.5), aim: 0.62, fov: 34 },
  // hero: low and near, the frame that makes a stride believable
  hero:      { offset: new Vector3(-1.35, 0.5, 1.65), aim: 0.55, fov: 30 },
  // straight across the line of travel, for comparing stride length
  profile:   { offset: new Vector3(0.15, 0.95, 3.6), aim: 0.60, fov: 32 },
};

const ease = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);

export class Stage {
  constructor(canvas) {
    this.renderer = new WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.toneMapping = ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = PCFSoftShadowMap;

    this.scene = new Scene();
    this.scene.fog = new FogExp2(0x000000, 0);
    this.camera = new PerspectiveCamera(38, 1, 0.05, 3000);

    this.sun = new DirectionalLight(0xffffff, 3.0);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const c = this.sun.shadow.camera;
    c.left = -30; c.right = 30; c.top = 30; c.bottom = -30; c.near = 0.5; c.far = 260;
    // Depth bias alone smears at low sun angles; normalBias scales with the
    // geometry that actually causes the acne.
    this.sun.shadow.normalBias = 0.035;
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);

    this.fill = new HemisphereLight(0x8fa6c8, 0x2b2622, 0.25);
    this.scene.add(this.fill);

    this.ibl = new IBL(this.renderer);
    this.world = new Group();
    this.scene.add(this.world);
    this.stars = this._buildStars();
    this.scene.add(this.stars);

    // --- camera rig state -------------------------------------------------
    this.shot = SHOTS.establish;
    this.fromPos = new Vector3(); this.fromAim = new Vector3();
    this.curPos = new Vector3(); this.curAim = new Vector3();
    this.subject = new Vector3();
    this.blend = 1; this.blendDur = 1;
    this.orbit = 0;
    this.side = 1;          // which side of the travel axis the camera sits on
    this._p = new Vector3(); this._a = new Vector3();
  }

  _buildStars() {
    const n = 2600, pos = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      // even-ish spread on a sphere, deterministic
      const u = (i + 0.5) / n, phi = Math.acos(1 - 2 * u), th = Math.PI * (1 + Math.sqrt(5)) * i;
      pos[i * 3] = Math.sin(phi) * Math.cos(th) * 1400;
      pos[i * 3 + 1] = Math.abs(Math.cos(phi)) * 1400;
      pos[i * 3 + 2] = Math.sin(phi) * Math.sin(th) * 1400;
    }
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(pos, 3));
    const pts = new Points(g, new PointsMaterial({ color: 0xffffff, size: 2.4, sizeAttenuation: false }));
    pts.frustumCulled = false;
    return pts;
  }

  applyEnvironment(env, sunDirection) {
    this.sun.position.copy(sunDirection).multiplyScalar(60);
    this.sun.target.position.set(0, 0, 0);
    this.sun.color.copy(env.sunColor);
    this.sun.intensity = env.sunIntensity;
    this.fill.color.copy(env.skyColor);
    this.fill.groundColor.copy(env.groundAlbedo);
    this.fill.intensity = env.ambientIntensity;
    this.scene.fog.color.copy(env.skyColor);
    this.scene.fog.density = env.fogDensity;
    this.renderer.setClearColor(new Color().copy(env.skyColor).multiplyScalar(env.vacuum ? 0 : 0.55), 1);
    this.stars.visible = env.stars > 0;
    this.stars.material.opacity = env.stars;
    this.renderer.toneMappingExposure = env.exposure;
    const tex = this.ibl.update(env);
    if (tex) this.scene.environment = tex;
  }

  /**
   * Fly to a shot. Interpolating position and aim SEPARATELY, each with an
   * ease, is what stops the move reading as a linear slide: the aim settles
   * before the dolly does, so the subject stays framed through the whole move.
   */
  flyTo(shot, seconds = 2.2) {
    this.fromPos.copy(this.curPos);
    this.fromAim.copy(this.curAim);
    this.shot = shot;
    this.blend = 0;
    this.blendDur = Math.max(0.001, seconds);
  }

  /** Snap with no transition — used once, on first entry. */
  cut(shot) { this.shot = shot; this.blend = 1; this._solve(shot, this.curPos, this.curAim); }

  /**
   * Solve a shot into world space.
   *
   * The offset is rotated by the subject's heading so the camera stays side-on
   * to travel — which is what lets an audience read stride length. But side-on
   * has two solutions, and on a curved course one of them puts the sun in the
   * lens and turns the robot into a silhouette. Pick the side the sun is
   * BEHIND, with hysteresis so the camera does not flip-flop while the subject
   * turns through the crossover.
   */
  _solve(shot, outPos, outAim) {
    // Deliberately NOT re-picking the side per frame. Tried it: on rolling
    // terrain the flip walks the camera through the hill between it and the
    // robot, and the subject vanishes behind it. The side is chosen once per
    // run, in enter().
    const s = Math.sin(this.orbit), c = Math.cos(this.orbit);
    const o = shot.offset;
    const oz = o.z * this.side;
    outPos.set(
      this.subject.x + o.x * c - oz * s,
      this.subject.y + o.y,
      this.subject.z + o.x * s + oz * c,
    );
    outAim.set(this.subject.x, this.subject.y + shot.aim, this.subject.z);
  }

  /**
   * Which side of the travel axis to stand on.
   *
   * Side-on framing is what lets an audience read stride length, but it has
   * two solutions and on a curved course one of them puts the sun in the lens
   * and reduces the robot to a silhouette. sunDir points TOWARDS the sun, so
   * the camera sees the lit face when it stands on the sun's side of the
   * subject — that is, when (camera - subject) agrees with sunDir.
   *
   * The margin is what makes this usable: without it the score crosses zero as
   * the subject turns and the camera flips every frame, which averages out to
   * a camera stuck on the travel axis, far away and looking down the line.
   */
  _pickSide(shot) {
    const s = Math.sin(this.orbit), c = Math.cos(this.orbit);
    const o = shot.offset;
    const score = (side) => {
      const oz = o.z * side;
      const dx = o.x * c - oz * s;
      const dz = o.x * s + oz * c;
      const len = Math.hypot(dx, dz) || 1;
      return (dx / len) * this._sunDir.x + (dz / len) * this._sunDir.z;
    };
    const keep = score(this.side);
    const other = score(-this.side);
    return other > keep + 0.35 ? -this.side : this.side;
  }

  /**
   * Confine the camera to a module interior, or release it with null.
   *
   * The four shots are written for open ground: `establish` sits 8.5 m to the
   * side and 4.2 m up. A corridor is 7.2 m wide and 4.0 m tall, so every one
   * of them puts the camera through a wall — and since the shell is a closed
   * mesh, what you get is the OUTSIDE of the module against the stars and no
   * robot at all. Clamping the solved position into the cross-section keeps
   * each shot's intent (which side, how far along, how high) and gives up only
   * the part that cannot be honoured indoors.
   */
  setInterior(tube) { this.tube = tube || null; }

  _clampToTube(p) {
    const t = this.tube;
    if (!t) return p;
    const STANDOFF = 0.35;                          // clear of the wall panels
    const hw = Math.max(0.2, t.width / 2 - STANDOFF);
    const hh = Math.max(0.2, t.height / 2 - STANDOFF);
    p.y = MathUtils.clamp(p.y, -hh, hh);
    if (t.axis === 'x') p.z = MathUtils.clamp(p.z, -hw, hw);
    else p.x = MathUtils.clamp(p.x, -hw, hw);
    return p;
  }

  update(dt, subject, heading = 0) {
    this.subject.lerp(subject, 1 - Math.exp(-9 * dt));
    this.orbit = heading;
    this._solve(this.shot, this._p, this._a);
    this._clampToTube(this._p);

    if (this.blend < 1) {
      this.blend = Math.min(1, this.blend + dt / this.blendDur);
      const kp = ease(this.blend);
      const ka = ease(Math.min(1, this.blend * 1.45));   // aim leads the dolly
      this.curPos.copy(this.fromPos).lerp(this._p, kp);
      this.curAim.copy(this.fromAim).lerp(this._a, ka);
      this.camera.fov = MathUtils.lerp(this.camera.fov, this.shot.fov, kp * 0.5);
      this.camera.updateProjectionMatrix();
    } else {
      // settled: follow with a light spring so the frame breathes
      this.curPos.lerp(this._p, 1 - Math.exp(-5.5 * dt));
      this.curAim.lerp(this._a, 1 - Math.exp(-7.0 * dt));
    }
    // Clamp the BLENDED position too: a fly-to that interpolates between two
    // legal points can still bow outside the tube on the way across.
    this._clampToTube(this.curPos);
    this.camera.position.copy(this.curPos);
    this.camera.lookAt(this.curAim);
    this.sun.target.position.copy(this.subject);
    this.sun.position.copy(this.subject).add(this._sunOffset || new Vector3(30, 40, 20));
  }

  setSunOffset(dir) {
    this._sunOffset = dir.clone().multiplyScalar(60);
    this._sunDir = dir.clone().normalize();
  }

  resize(w, h) {
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render() { this.renderer.render(this.scene, this.camera); }
}
