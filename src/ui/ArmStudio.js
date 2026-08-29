/**
 * ArmStudio — your arms, on the robot's arms, in real time.
 *
 * A webcam, MediaPipe's pose model, and the G1's own arm chain: 7 joints per
 * side, every one of them clamped to the range its URDF declares. Move, and
 * the machine moves; ask for something outside its envelope and it stops
 * exactly where the hardware would, and says which joint ran out.
 *
 * WHY THIS IS A REAL RETARGET AND NOT A PUPPET
 *
 * The naive version copies landmark positions onto the robot. It cannot work:
 * a person is 1.7-1.9 m with a 0.60 m arm, a G1 is 1.32 m with a 0.36 m one,
 * and their shoulders sit differently relative to their hips. Positions carry
 * proportions; directions do not. So src/sim/ArmRetarget.js matches where each
 * bone POINTS — shoulder-to-elbow, elbow-to-hand — which is the same principle
 * tools/fit_pose.mjs uses on Apollo footage, and for the same reason.
 *
 * Three things it refuses to fake:
 *
 *   THE LIMITS ARE THE URDF'S. Including the shoulder roll's, which is
 *   asymmetric between the arms — [-1.59, 2.25] on the left, [-2.25, 1.59] on
 *   the right — because the joint mirrors and its range does not.
 *
 *   THE RATE IS BOUNDED. A webcam drops frames and a landmark can jump a
 *   decimetre between two of them; followed straight that asks the shoulder for
 *   thousands of degrees a second. The command is rate-limited on the way out
 *   and the page reports how often that bites.
 *
 *   WHAT IT CANNOT REACH IS SHOWN, NOT HIDDEN. When a joint sits on a stop the
 *   bar goes red and the arm stops there. A pose the machine cannot make should
 *   read as the machine refusing, not as the tracking failing.
 *
 * AND WHY ARMS, HERE
 *
 * Because in free flight they are the only attitude control a body has. Angular
 * momentum is conserved, so the only way to rotate the torso is to rotate
 * something else the other way, and the arms are 13.8 % of the body's pitch
 * inertia. That is what the Apollo windmilling was, and it is what
 * src/sim/Ballistic.js uses to decide whether a jump lands on its feet. This
 * page reads the sweep you are actually making and tells you how much attitude
 * it would buy you on each body.
 */
import {
  Scene, PerspectiveCamera, WebGLRenderer, Group, Color, Vector3,
  DirectionalLight, HemisphereLight, ACESFilmicToneMapping, PCFSoftShadowMap,
  Mesh, MeshStandardMaterial, PlaneGeometry, EquirectangularReflectionMapping,
} from 'three';
import { FilesetResolver, PoseLandmarker } from '@mediapipe/tasks-vision';
import { loadRobot, robotById } from '../render/Robots.js';
import { IBL } from '../render/IBL.js';
import { byId } from '../render/Environments.js';
import { armTargets, solveArm, ArmSmoother, torsoFrame, LM } from '../sim/ArmRetarget.js';
import { ARM_CHAIN, ARM_LIMITS, armFK } from '../sim/G1Kinematics.js';
import { ARM_INERTIA, BODY_INERTIA, ARM_TUCK, instability } from '../sim/Ballistic.js';

const SIDES = ['left', 'right'];
/** Chain slots the retarget drives, and the labels the panel shows. */
const DRIVEN = [3, 4, 5, 6];
const SHORT = (j) => j.replace(/^(left|right)_/, '').replace(/_joint$/, '').replace(/_/g, ' ');

/** Skeleton edges worth drawing over the video — the ones that drive the arms. */
const BONES = [
  [LM.shoulderL, LM.shoulderR], [LM.shoulderL, LM.hipL], [LM.shoulderR, LM.hipR],
  [LM.hipL, LM.hipR],
  [LM.shoulderL, LM.elbowL], [LM.elbowL, LM.wristL],
  [LM.shoulderR, LM.elbowR], [LM.elbowR, LM.wristR],
];

export class ArmStudio {
  constructor(canvas, video, overlay, ui) {
    this.canvas = canvas;
    this.video = video;
    this.overlay = overlay;
    this.ui = ui;

    this.renderer = new WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.toneMapping = ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = PCFSoftShadowMap;

    this.scene = new Scene();
    this.scene.background = new Color(0x0b0d11);
    this.camera = new PerspectiveCamera(34, 1, 0.02, 60);

    // A three-light rig rather than one lamp. The shells are metal at
    // metalness 0.42; with a single source they read as two flat tones and the
    // joints disappear, which is the one thing this page cannot afford.
    const key = new DirectionalLight(0xffffff, 2.6);
    key.position.set(2.2, 3.0, 2.6);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const sc = key.shadow.camera;
    sc.left = -1.6; sc.right = 1.6; sc.top = 1.8; sc.bottom = -1.2; sc.near = 0.3; sc.far = 12;
    sc.updateProjectionMatrix();
    key.shadow.normalBias = 0.02;
    const fill = new DirectionalLight(0x9fc4ff, 0.9);
    fill.position.set(-2.6, 1.4, 1.2);
    const rim = new DirectionalLight(0xffd9b0, 1.6);
    rim.position.set(-1.2, 2.0, -3.0);
    this.scene.add(key, fill, rim, new HemisphereLight(0x8fa6c8, 0x191b20, 0.5));

    // The same ground-bounce environment the arena builds, so the metal has
    // something to reflect. Without it these shells render as silhouettes.
    this.ibl = new IBL(this.renderer);
    this.scene.environment = this.ibl.update(byId('moon'));

    const floor = new Mesh(new PlaneGeometry(14, 14),
      new MeshStandardMaterial({ color: 0x14171c, roughness: 0.92, metalness: 0.0 }));
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.world = new Group();
    this.scene.add(this.world);

    this.robot = null;
    this.q = { left: null, right: null };
    this.smooth = { left: new ArmSmoother(), right: new ArmSmoother() };
    this.track = { seen: false, confidence: 0, fps: 0 };
    this.err = { left: 0, right: 0 };
    this.clamped = { left: [], right: [] };
    this.mirror = true;
    this.gravity = 1.625;

    // camera rig
    this.orbit = -0.35; this.elev = 0.12; this.dist = 1.9;
    this.aim = new Vector3(0, 0.95, 0);
    this._bindInput();
    addEventListener('resize', () => this.resize());
    this.resize();
  }

  _bindInput() {
    const c = this.canvas;
    let drag = null;
    c.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY }; c.setPointerCapture(e.pointerId); });
    c.addEventListener('pointermove', (e) => {
      if (!drag) return;
      this.orbit -= (e.clientX - drag.x) * 0.007;
      this.elev = Math.max(-0.4, Math.min(1.1, this.elev + (e.clientY - drag.y) * 0.005));
      drag.x = e.clientX; drag.y = e.clientY;
    });
    const end = () => { drag = null; };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', end);
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.dist = Math.max(0.7, Math.min(5, this.dist * (1 + Math.sign(e.deltaY) * 0.1)));
    }, { passive: false });
  }

  resize() {
    const r = this.canvas.parentElement.getBoundingClientRect();
    this.renderer.setSize(r.width, r.height, false);
    this.camera.aspect = r.width / Math.max(r.height, 1);
    this.camera.updateProjectionMatrix();
  }

  async loadRobot() {
    const def = robotById('g1');
    this.robot = await loadRobot(def);
    this.robot.root.traverse((o) => {
      if (!o.isMesh) return;
      o.castShadow = true; o.receiveShadow = true;
      o.material = o.material.clone();
      o.material.envMapIntensity = 1.15;
    });
    this.world.add(this.robot.root);
    return this.robot;
  }

  /**
   * Start the camera and the pose model.
   *
   * Both the wasm runtime and the 9 MB model are served from this origin
   * rather than a CDN, so the page works with no network at all once it has
   * been built — which also means it cannot quietly start depending on a
   * third party being up.
   */
  async startTracking() {
    const files = await FilesetResolver.forVisionTasks('/wasm');
    this.landmarker = await PoseLandmarker.createFromOptions(files, {
      baseOptions: { modelAssetPath: '/models/pose_landmarker_full.task', delegate: 'GPU' },
      runningMode: 'VIDEO',
      numPoses: 1,
      minPoseDetectionConfidence: 0.6,
      minPosePresenceConfidence: 0.6,
      minTrackingConfidence: 0.6,
    });

    const stream = await navigator.mediaDevices.getUserMedia({
      video: { width: { ideal: 960 }, height: { ideal: 720 }, facingMode: 'user' },
      audio: false,
    });
    this.video.srcObject = stream;
    await this.video.play();
    this.tracking = true;
  }

  /** One tracking pass, if the video has a new frame for us. */
  _track(now) {
    if (!this.tracking || this.video.readyState < 2) return;
    if (this.video.currentTime === this._lastVideoTime) return;
    this._lastVideoTime = this.video.currentTime;

    const res = this.landmarker.detectForVideo(this.video, now);
    const world = res?.worldLandmarks?.[0];
    const screen = res?.landmarks?.[0];
    this.track.seen = !!world;
    this._screen = screen;
    if (world) this.applyLandmarks(world);
  }

  /**
   * Turn one set of world landmarks into a commanded arm pose.
   *
   * Separate from _track() on purpose: where the landmarks came from is not
   * this code's business, and splitting it means the retarget can be driven
   * from a recorded pose in a headless check without a camera in the room.
   *
   * @param {Array<{x,y,z,visibility}>} world  MediaPipe world landmarks
   */
  applyLandmarks(world) {
    for (const side of SIDES) {
      // Mirror by default: your left arm should drive the arm on the left of
      // the screen, which — because the preview is mirrored, as every camera
      // preview is — is the robot's RIGHT. Reading the human's other side is
      // what makes it feel like a reflection rather than a puzzle.
      const human = this.mirror ? (side === 'left' ? 'right' : 'left') : side;
      const t = armTargets(world, human);
      this.track.confidence = Math.max(this.track.confidence * 0.9, t.confidence);
      // Below this the model is guessing at an arm it cannot see, and guessing
      // confidently. Hold the last good pose instead of following the guess.
      if (t.confidence < 0.5) continue;
      const sol = solveArm(side, t, this.q[side]);
      this.err[side] = sol.err;
      this.clamped[side] = sol.clamped;
      this.q[side] = sol.q;
      this._targets = this._targets || {};
      this._targets[side] = t;
    }
  }

  step(dt, now) {
    this._track(now);
    if (!this.robot) return;

    for (const side of SIDES) {
      if (!this.q[side]) continue;
      const q = this.smooth[side].step(this.q[side], dt);
      const chain = ARM_CHAIN[side];
      for (const k of DRIVEN) {
        const j = this.robot.joints[chain[k].joint];
        if (j) j.setJointValue(q[k]);
      }
    }

    const ce = Math.cos(this.elev), se = Math.sin(this.elev);
    this.camera.position.set(
      this.aim.x + Math.cos(this.orbit) * ce * this.dist,
      this.aim.y + se * this.dist,
      this.aim.z + Math.sin(this.orbit) * ce * this.dist);
    this.camera.lookAt(this.aim);
  }

  render() { this.renderer.render(this.scene, this.camera); }

  /** The pose skeleton, drawn over the video so you can see what it saw. */
  drawOverlay() {
    const c = this.overlay, ctx = c.getContext('2d');
    const w = c.width = c.clientWidth, h = c.height = c.clientHeight;
    ctx.clearRect(0, 0, w, h);
    const lms = this._screen;
    if (!lms) return;
    const X = (p) => (this.mirror ? 1 - p.x : p.x) * w;
    const Y = (p) => p.y * h;

    ctx.lineWidth = 2.5;
    ctx.strokeStyle = 'rgba(127,214,168,0.85)';
    ctx.beginPath();
    for (const [a, b] of BONES) {
      if (!lms[a] || !lms[b]) continue;
      ctx.moveTo(X(lms[a]), Y(lms[a]));
      ctx.lineTo(X(lms[b]), Y(lms[b]));
    }
    ctx.stroke();
    ctx.fillStyle = '#e8eef5';
    for (const i of [LM.shoulderL, LM.shoulderR, LM.elbowL, LM.elbowR, LM.wristL, LM.wristR, LM.hipL, LM.hipR]) {
      const p = lms[i];
      if (!p) continue;
      ctx.beginPath();
      ctx.arc(X(p), Y(p), 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  /**
   * The robot's own bone directions for a pose, and its limits.
   *
   * Exposed so tools/check_arms.mjs can measure what the arm is DOING rather
   * than trust what it was asked to do — the check compares the robot's actual
   * upper-arm direction against the human's, which needs the same FK the
   * solver used and not a re-implementation of it.
   */
  armDirsFor(side, q) {
    const { joints, p } = armFK(side, q);
    const s = joints[3].p, e = joints[6].p;
    const n = (v) => { const m = Math.hypot(...v); return m > 1e-9 ? v.map((c) => c / m) : v; };
    return {
      upper: n([e[0] - s[0], e[1] - s[1], e[2] - s[2]]),
      fore: n([p[0] - e[0], p[1] - e[1], p[2] - e[2]]),
    };
  }

  limitsFor(side) { return ARM_LIMITS[side]; }

  /**
   * How much body attitude the sweep you are making right now would buy.
   *
   * Conservation of angular momentum: rotating the arms through an angle
   * counter-rotates the body by (I_arm / I_body) of it, and a stroke out with
   * the elbow extended against a return with it tucked nets most of that
   * rather than cancelling. This is the same authority src/sim/Ballistic.js
   * gives the arms when it decides whether a jump lands on its feet, so the
   * number here and the number there are the same number.
   */
  armAuthority() {
    if (!this.q.left || !this.q.right) return null;
    // Sweep actually being used, as a fraction of the shoulder's full range.
    let sweep = 0;
    for (const side of SIDES) {
      const lim = ARM_LIMITS[side][3];
      const q = this.q[side][3];
      sweep = Math.max(sweep, Math.abs(q - (lim[0] + lim[1]) / 2));
    }
    const perStroke = (1 - ARM_TUCK) * (ARM_INERTIA / BODY_INERTIA) * sweep * 2;
    return { sweep, perStroke };
  }
}
