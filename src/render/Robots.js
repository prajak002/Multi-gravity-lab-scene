/**
 * The robot registry, and one loader that copes with three different URDF
 * conventions.
 *
 * Unitree ship these three descriptions with mesh paths written three
 * different ways and in two different formats, so the loader resolves against
 * each robot's own base directory rather than assuming a shared layout:
 *
 *   G1   meshes/x.STL      relative to the urdf's own folder,  64 STL
 *   H1   ../meshes/x.STL   relative to urdf/,                  21 STL
 *   Go2  ../dae/x.dae      relative to urdf/,                   7 Collada
 */
import { Group, Mesh, MeshStandardMaterial, Box3, Vector3, LoadingManager, Color } from 'three';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';

export const ROBOTS = [
  {
    id: 'g1', name: 'Unitree G1', short: 'G1', kind: 'biped',
    // 29 DoF, not the 23 DoF variant. The motion packets are authored on the
    // 29 DoF G1 and their CSV column order IS this URDF's joint order; the
    // 23 DoF description welds waist roll/pitch and wrist pitch/yaw, which
    // silently dropped six of the packets' channels and put the shoulders up
    // to 65 mm from where the packet says they are. On this description the
    // packets' own body_pos_w reproduces to 0.0 mm (tools/audit_packets.mjs).
    url: 'robots/g1/g1_29dof.urdf',
    blurb: '29 DoF humanoid, 1.32 m. The retarget target.',
    // Named, never positional: these three descriptions order their leg chains
    // differently, and indexing by position quietly drives the wrong joint.
    legs: {
      // ankleRoll matters: without it the sole is held level while the ground
      // is not, so on any real DEM one edge of the foot is buried and the
      // opposite one is in the air. It is the joint that lets a foot LIE on
      // rough ground rather than spear into it.
      left:  { hipPitch: 'left_hip_pitch_joint',  hipRoll: 'left_hip_roll_joint',
               knee: 'left_knee_joint',  anklePitch: 'left_ankle_pitch_joint',
               ankleRoll: 'left_ankle_roll_joint' },
      right: { hipPitch: 'right_hip_pitch_joint', hipRoll: 'right_hip_roll_joint',
               knee: 'right_knee_joint', anklePitch: 'right_ankle_pitch_joint',
               ankleRoll: 'right_ankle_roll_joint' },
    },
    arms: {
      left:  { shoulderPitch: 'left_shoulder_pitch_joint',  shoulderRoll: 'left_shoulder_roll_joint',  elbow: 'left_elbow_joint' },
      right: { shoulderPitch: 'right_shoulder_pitch_joint', shoulderRoll: 'right_shoulder_roll_joint', elbow: 'right_elbow_joint' },
    },
    waistYaw: 'waist_yaw_joint',
    waistRoll: 'waist_roll_joint',
    waistPitch: 'waist_pitch_joint',
    feet: ['left_ankle_roll_link', 'right_ankle_roll_link'],
  },
  {
    id: 'h1', name: 'Unitree H1', short: 'H1', kind: 'biped',
    url: 'robots/h1/urdf/h1.urdf',
    blurb: 'Full-size humanoid, 1.80 m. Longer legs, so a slower natural cadence.',
    legs: {
      left:  { hipPitch: 'left_hip_pitch_joint',  hipRoll: 'left_hip_roll_joint',
               knee: 'left_knee_joint',  anklePitch: 'left_ankle_joint' },
      right: { hipPitch: 'right_hip_pitch_joint', hipRoll: 'right_hip_roll_joint',
               knee: 'right_knee_joint', anklePitch: 'right_ankle_joint' },
    },
    // This H1 description is legs-only: its 10 movable joints are all in the
    // legs, and the arm links are welded. There is nothing to swing, so the
    // gait must not pretend otherwise.
    arms: null,
    feet: ['left_ankle_link', 'right_ankle_link'],
  },
  {
    id: 'go2', name: 'Unitree Go2', short: 'GO2', kind: 'quadruped',
    url: 'robots/go2/urdf/go2.urdf',
    blurb: '12 DoF quadruped. Four contacts, so low gravity costs it far less.',
    quad: {
      FL: { abduct: 'FL_hip_joint', thigh: 'FL_thigh_joint', calf: 'FL_calf_joint' },
      FR: { abduct: 'FR_hip_joint', thigh: 'FR_thigh_joint', calf: 'FR_calf_joint' },
      RL: { abduct: 'RL_hip_joint', thigh: 'RL_thigh_joint', calf: 'RL_calf_joint' },
      RR: { abduct: 'RR_hip_joint', thigh: 'RR_thigh_joint', calf: 'RR_calf_joint' },
    },
    feet: ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'],
  },
];

export const robotById = (id) => ROBOTS.find((r) => r.id === id) || ROBOTS[0];

function shellMaterial() {
  const mat = new MeshStandardMaterial({
    color: new Color(0xdadde2), metalness: 0.42, roughness: 0.50,
    // The shell is metal, so without an environment it has no diffuse term and
    // nothing to reflect but the sun, and renders as a black silhouette. The
    // IBL supplies that environment; this is how much of it the shell takes.
    envMapIntensity: 1.5,
  });
  // Keep any mode tint to a grazing-angle rim. three's `emissive` is a flat
  // additive term over the whole surface, so writing a colour into it floods
  // the entire robot rather than edge-lighting it.
  mat.onBeforeCompile = (shader) => {
    shader.fragmentShader = shader.fragmentShader.replace(
      '#include <emissivemap_fragment>',
      `#include <emissivemap_fragment>
       float rimFacing = abs(dot(normalize(vViewPosition), normal));
       totalEmissiveRadiance *= pow(1.0 - clamp(rimFacing, 0.0, 1.0), 3.0);`
    );
  };
  mat.customProgramCacheKey = () => 'arena-shell-rim';
  return mat;
}

/**
 * Load one robot description. Rejects loudly — a robot presented as a Unitree
 * that silently lost half its links is worse than an error message.
 */
export function loadRobot(def) {
  return new Promise((resolve, reject) => {
    const base = def.url.replace(/[^/]+$/, '');       // the urdf's own directory
    const failures = [];
    let pending = 0, started = false, settled = false, robotRef = null;

    const manager = new LoadingManager();
    manager.onError = (u) => failures.push(u);
    const loader = new URDFLoader(manager);
    loader.packages = {};
    const mat = shellMaterial();

    const fail = (msg) => { if (settled) return; settled = true; clearTimeout(timer); reject(new Error(msg)); };

    loader.loadMeshCb = (path, mgr, urdfMaterial, done) => {
      pending++; started = true;
      const settle = (obj, err) => { pending--; done(obj, err); queueMicrotask(finish); };
      // Resolve "../dae/base.dae" and "meshes/x.STL" alike against the URDF's
      // own folder, then normalise the ".." away so fetch sees a clean path.
      const url = new URL(base + path, window.location.href).pathname;
      const ext = path.split('.').pop().toLowerCase();
      if (ext === 'stl') {
        new STLLoader(mgr).load(url, (geo) => {
          geo.computeVertexNormals();
          const m = new Mesh(geo, mat);
          m.castShadow = true; m.receiveShadow = false;
          settle(m);
        }, undefined, (e) => { failures.push(`${path}: ${e?.message || e}`); settle(null, e); });
      } else {
        new ColladaLoader(mgr).load(url, (res) => {
          res.scene.traverse((o) => {
            if (!o.isMesh) return;
            // Collada brings its own materials; replace them so every robot in
            // the arena shades identically and the comparison stays honest.
            o.material = mat;
            o.castShadow = true; o.receiveShadow = false;
          });
          settle(res.scene);
        }, undefined, (e) => { failures.push(`${path}: ${e?.message || e}`); settle(null, e); });
      }
    };

    function finish() {
      if (settled || !robotRef || !started || pending > 0) return;
      const robot = robotRef;
      if (failures.length) return fail(`${def.name}: ${failures.length} mesh(es) failed:\n  ` + failures.join('\n  '));
      try {
        // URDF is Z-up, three is Y-up.
        const root = new Group(); root.name = def.id;
        const yUp = new Group(); yUp.rotation.x = -Math.PI / 2;
        yUp.add(robot); root.add(yUp);

        for (const name of Object.keys(robot.joints)) robot.joints[name].setJointValue(0);
        robot.updateMatrixWorld(true); root.updateMatrixWorld(true);

        const box = new Box3().setFromObject(root);
        if (!isFinite(box.min.y)) return fail(`${def.name} loaded with no renderable geometry.`);
        const size = new Vector3(); box.getSize(size);
        yUp.position.y = -box.min.y;                  // soles measured onto y=0, not eyeballed
        root.updateMatrixWorld(true);

        // Grab the foot links by name; ground contact is solved by measuring
        // where they actually are, not by assuming the model's origin is on
        // the floor once the joints have moved.
        const feet = (def.feet || []).map((n) => robot.links?.[n]).filter(Boolean);
        if (def.feet && feet.length !== def.feet.length) {
          return fail(`${def.name}: expected foot links ${def.feet.join(', ')} — found ${feet.length}`);
        }

        settled = true; clearTimeout(timer);
        resolve({ def, root, robot, joints: robot.joints, feet, height: size.y,
                  jointNames: Object.keys(robot.joints) });
      } catch (e) { fail(`${def.name} post-processing failed: ${e.message}`); }
    }

    manager.onLoad = () => queueMicrotask(finish);
    fetch(def.url).then((r) => {
      if (!r.ok) throw new Error(`HTTP ${r.status} for ${def.url}`);
      return r.text();
    }).then((text) => {
      robotRef = loader.parse(text);
      if (!robotRef) return fail(`urdf-loader returned nothing for ${def.url}`);
      queueMicrotask(finish);
    }).catch((e) => fail(`Could not load ${def.url}: ${e.message}`));

    const timer = setTimeout(() => fail(`Timed out loading ${def.url} (45 s).`), 45000);
  });
}
