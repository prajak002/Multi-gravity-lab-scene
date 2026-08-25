/**
 * Multi-Gravity Arena.
 *
 * One motion, four gravitational fields, three Unitree platforms. Everything
 * that differs between runs differs because g differs.
 */
import { Vector3, Box3 } from 'three';
import { Stage, SHOTS } from './core/Stage.js';
import { ENVIRONMENTS, byId, sunDir, gRatio, G_EARTH } from './render/Environments.js';
import { ROBOTS, robotById, loadRobot } from './render/Robots.js';
import { buildTerrain } from './render/Terrain.js';
import { loadInterior } from './render/Interior.js';
import { plantOnSoles, conformAnkles, penetration } from './sim/Footing.js';
import { buildRocks } from './render/Rocks.js';
import { Gait } from './sim/Gait.js';
import { Dust } from './render/Dust.js';
import { loadMotion } from './data/Motion.js';
import { Lobby } from './ui/Lobby.js';
import { HUD } from './ui/HUD.js';
import { Dashboard } from './ui/Dashboard.js';

const canvas = document.getElementById('stage');
const uiRoot = document.getElementById('ui');
const boot = document.getElementById('boot');

const stage = new Stage(canvas);
const hud = new HUD(uiRoot);
const dashboard = new Dashboard(stage.renderer, uiRoot);
const dust = new Dust();
stage.scene.add(dust.points);

const robotCache = new Map();     // id -> loaded robot (URDF loads are expensive)
let terrain = null;
let interior = null;
// Set when an interior is loaded: the corridor's own measured geometry, which
// is what the ISS run is flown down instead of an open-course heading.
let tube = null;
let rocks = null;
let current = null;               // { def, root, robot, joints, height }
let gait = null;
let env = byId('moon');
let travelled = 0;
let courseHeading = 0;
let running = false;
let motionLabel = 'generated';
let currentMotion = 'walk';
let hudSlope = 0;
// ?motion=retarget plays a baked capture instead of the generated stride
const useRetarget = new URLSearchParams(location.search).get('motion') === 'retarget';
const COURSE_LENGTH = 120;      // metres before the traverse restarts
const FREE_FALL_Y = 0.0;        // free fall has no floor; drift about the module axis
const ISS_COURSE = 11;          // metres of module to swim down before looping

async function getRobot(id) {
  if (robotCache.has(id)) return robotCache.get(id);
  boot.classList.remove('gone');
  boot.querySelector('span').textContent = `loading ${robotById(id).name}`;
  const loaded = await loadRobot(robotById(id));
  robotCache.set(id, loaded);
  boot.classList.add('gone');
  return loaded;
}

/**
 * Show a load failure instead of sitting on the splash forever.
 *
 * enter() is async and nothing awaited it, so a rejected URDF load became an
 * unhandled rejection: the console had the reason, but the page just said
 * "loading" indefinitely. On a deploy where the meshes are missing entirely
 * that is the only symptom the user ever sees.
 */
function showFailure(err) {
  boot.classList.remove('gone');
  boot.innerHTML = '';
  const box = document.createElement('div');
  box.className = 'boot-error';
  box.innerHTML = `<b>Could not load the robot</b><pre></pre>
    <span>Check that <code>public/robots/</code> was deployed \u2014 the meshes are
    fetched at runtime, so an ignored asset folder fails exactly like this.</span>`;
  box.querySelector('pre').textContent = String(err && err.message || err);
  boot.appendChild(box);
}

function setTerrain(nextEnv) {
  if (terrain) { stage.world.remove(terrain); terrain.geometry.dispose(); terrain.material.dispose(); }
  terrain = buildTerrain(nextEnv);
  if (terrain) stage.world.add(terrain);

  if (rocks) { stage.world.remove(rocks); rocks.geometry.dispose(); rocks.material.dispose(); rocks = null; }
  if (terrain) {
    rocks = buildRocks(nextEnv, terrain.heightAt, (nextEnv.sunAz - 90) * Math.PI / 180);
    if (rocks) stage.world.add(rocks);
  }
}

/** Swap the module interior in and out. Only the ISS declares one. */
async function setInterior(nextEnv) {
  if (interior) { stage.world.remove(interior); interior = null; }
  tube = null;
  stage.setInterior(null);
  if (!nextEnv.interior) return;
  interior = await loadInterior(nextEnv.interior);
  if (!interior) return;
  stage.world.add(interior);
  // The corridor is 43.8 m of real module. How far the robot may swim before
  // it wraps comes from the asset rather than from a constant that was a guess
  // about a different asset — leave a margin at each end so it turns around
  // inside the tube rather than in the end wall.
  tube = {
    axis: interior.userData.axis,
    length: Math.max(4, (interior.userData.length || 14) - 6),
    width: interior.userData.width || 7,
    height: interior.userData.height || 4,
  };
  stage.setInterior(tube);
}

async function enter({ robot: robotId, env: envId, motion: motionId = 'walk' }) {
  env = byId(envId);
  const loaded = await getRobot(robotId);

  if (current && current !== loaded) stage.world.remove(current.root);
  current = loaded;
  if (!current.root.parent) stage.world.add(current.root);

  setTerrain(env);
  await setInterior(env);
  dust.applyEnvironment(env);
  dust.clear();
  stage.applyEnvironment(env, sunDir(env.sunElev, env.sunAz));
  stage.setSunOffset(sunDir(env.sunElev, env.sunAz));

  // Hip height drives the pendulum, so measure it rather than assume it.
  gait = new Gait(current.def, current.height * 0.52, motionId);

  // Prefer a retargeted capture where one exists for this robot; fall back to
  // the generated stride otherwise. Both satisfy the same interface, so
  // nothing below this line knows the difference.
  if (useRetarget) {
    try {
      const m = await loadMotion(`motions/${robotId}_squat_walk.json`, current.def, current.height * 0.52);
      if (m) { gait = m; motionLabel = `retarget · ${m.source}`; }
      else motionLabel = 'generated';
    } catch { motionLabel = 'generated'; }
  } else motionLabel = 'generated';
  travelled = 0;

  // Lay the course ACROSS the sun.
  //
  // The camera frames side-on so stride length is readable, which leaves two
  // possible sides; on a curved course the heading rotates through the
  // crossover and the camera flips sides mid-run. A straight traverse
  // perpendicular to the sun azimuth removes the choice: one side is lit for
  // the entire run, and the camera can simply stay there.
  courseHeading = (env.sunAz - 90) * Math.PI / 180;
  // ...except inside a module, where the corridor decides. A heading taken
  // from the sun azimuth sent the ISS run off at 110 degrees to the tube, so
  // the robot swam straight out through the wall within a few seconds and
  // finished the traverse alone against the stars — which is the exact
  // reference the interior exists to provide.
  if (tube) courseHeading = tube.axis === 'x' ? 0 : Math.PI / 2;
  stage.side = 1;

  if (dashboard.panes.length) dashboard.dispose();
  currentMotion = motionId;
  hud.setTitle(`${current.def.short} · ${env.short}`, `${env.name} · ${motionId}`);
  hud.show(true);

  // The cinematic: begin wide, then settle into the working view. First entry
  // cuts to the establishing frame so there is nothing to fly FROM.
  if (!running) { stage.cut(SHOTS.establish); running = true; }
  else stage.flyTo(SHOTS.establish, 1.1);
  setTimeout(() => stage.flyTo(SHOTS.chase, 3.4), 1300);
}

const lobby = new Lobby(uiRoot, (sel) => { enter(sel).catch(showFailure); });

// Shot keys — the same rig, so every one of these is a move rather than a cut.
addEventListener('keydown', (e) => {
  const map = { '1': SHOTS.establish, '2': SHOTS.chase, '3': SHOTS.hero, '4': SHOTS.profile };
  if (map[e.key]) stage.flyTo(map[e.key], 2.0);
  if (e.key.toLowerCase() === 'l') lobby.show();
  if (e.key.toLowerCase() === 'd') toggleDashboard();
  // EMERGENCY STOP for the swim stroke. Escape, and also the space bar,
  // because the two keys a person reaches for under stress are the big one and
  // the one that means "stop" everywhere else. It latches: the stroke decays
  // to the neutral streamline pose over 0.8 s and stays there until the motion
  // is re-selected. Nothing un-stops it by accident.
  if (e.key === 'Escape' || e.key === ' ') {
    if (gait?.butterfly && !gait.estopped) { gait.estop('operator'); e.preventDefault(); }
  }
});

function toggleDashboard() {
  if (!current) return;
  const on = !dashboard.active;
  if (on && !dashboard.panes.length) dashboard.build(current, currentMotion);
  dashboard.show(on);
  hud.show(!on);
  lobby.hide();
}

const subject = new Vector3();
const footBox = new Box3();
const contactAt = new Vector3();
const travelDir = new Vector3();
let elapsed = 0;
let lastAlong = 0;
let footing = { contact: null, lift: 0, clearance: 0 };
// True while the swimmer is laid flat down the module, which changes where
// the camera should aim.
let prone = false;
// per-foot swing state, so a landing is detected as a transition rather than
// fired every frame the foot happens to be down
let wasSwinging = [];
let last = performance.now();

/**
 * Plant the robot on its soles, against the ground under each contact point.
 *
 * The measurement lives in Footing.js; see the note there for why the previous
 * version — one terrain sample under the root, lowest world-AABB corner of the
 * foot — both sank the feet on rising ground and floated them on falling
 * ground. Nothing here samples the terrain under the pelvis any more.
 */
function plant(loaded, hop) {
  return plantOnSoles(loaded, terrain ? (x, z) => terrain.heightAt(x, z) : null, hop);
}

function frame(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;

  if (current && gait) {
    const { v } = gait.advance(dt, env.g);
    travelled += v * dt;

    const pose = gait.evaluate(env.g);
    elapsed += dt;
    dust.update(elapsed, innerHeight);
    for (const [name, value] of Object.entries(pose.joints)) {
      const joint = current.joints[name];
      if (joint) joint.setJointValue(value);
    }

    // Straight traverse. Wrapping rather than turning around keeps the heading
    // — and therefore the lit side — constant for the whole run.
    // A module is 14 m long, not 120. Traversing the surface course in free
    // fall swims the robot straight out through the wall and leaves it alone
    // against the stars, which is exactly the reference the interior exists to
    // provide. Keep an orbital run inside the module.
    const courseLen = terrain ? COURSE_LENGTH : (tube ? tube.length : ISS_COURSE);
    const along = travelled % courseLen;
    if (along < lastAlong) { dust.clear(); stage.cut(stage.shot); }   // wrapped
    lastAlong = along;
    const theta = courseHeading;
    const s0 = along - courseLen * 0.5;
    const x = Math.cos(theta) * s0;
    const z = Math.sin(theta) * s0;
    const groundY = terrain ? terrain.heightAt(x, z) : 0;
    current.root.position.set(x, groundY, z);
    // The URDF is x-forward, z-up; after the Z-up -> Y-up correction the
    // robot's anatomical forward is its local +X. A local +X of (cos t, 0, sin t)
    // needs rotation.y = -t. The extra +PI/2 that used to be here turned the
    // robot 90 degrees, so it faced across its own line of travel and appeared
    // to slide sideways down the course.
    current.root.rotation.y = -theta;
    if (terrain) {
      // Lean into the hill. A body that stays vertical while the ground tilts
      // reads as sliding up a ramp rather than climbing it.
      //
      // Order matters: with the default XYZ the lean would be applied about a
      // world axis BEFORE the yaw, which tips the robot sideways instead of
      // nose-up. YZX yaws first, so the lean then happens about the robot's
      // own across-travel axis.
      current.root.rotation.order = 'YZX';
      const [gx, gz] = gradient(terrain, x, z);
      const along = gx * Math.cos(theta) + gz * Math.sin(theta);
      const slope = Math.atan(along);
      // Clamp what the body will lean to. The terrain can be far steeper than
      // anything a biped could hold, and matching it exactly lays the robot flat.
      current.root.rotation.z = -clampNum(slope, -0.42, 0.42) * 0.8;
      hudSlope = slope;
    }
    // On the ISS there is no floor to stand on, so nothing is planted; the
    // body simply drifts and rotates about its own centre of mass.
    if (terrain) {
      // Conform BEFORE planting: the ankles decide the sole's attitude, and
      // the plant then finds the lowest point of the sole in that attitude.
      // The other order plants a level foot and then tilts it into the ground.
      conformAnkles(current, pose, (x, z) => terrain.heightAt(x, z));
      footing = plant(current, pose.bodyY);
    }
    // Free fall: nothing to plant against, so the body just drifts. The camera
    // has to follow it up there — aiming at a ground plane that does not exist
    // leaves the robot out of frame.
    else {
      // Drift about the tube's centreline, with the swing scaled to the
      // cross-section so the robot never rises through the ceiling or sinks
      // through the floor of a module it is supposed to be inside.
      const swing = tube ? Math.min(0.45, tube.height * 0.5 - current.height * 0.75) : 0.45;
      current.root.position.y = FREE_FALL_Y + Math.sin(travelled * 0.35) * Math.max(0, swing);

      // Lay a swimmer DOWN.
      //
      // pose.pitch was being computed by the stroke and then dropped on the
      // floor: nothing outside the terrain branch ever read it, so the G1 did
      // the entire butterfly bolt upright, arms overhead, travelling sideways
      // down the corridor like someone doing star jumps in a lift.
      //
      // A swimmer's spine is horizontal and along the direction of travel. The
      // model's spine is its local +Y and its face is its local +X, and with
      // rotation order YZX the yaw is applied first, so a rotation of -90
      // degrees about the resulting Z takes +Y onto the travel axis and +X
      // face-down. The stroke's own undulation rides on top of that.
      prone = currentMotion === 'swim' && pose.airborne;
      if (prone) {
        current.root.rotation.order = 'YZX';
        current.root.rotation.z = -Math.PI / 2 + (pose.pitch || 0);
        current.root.position.x -= Math.cos(theta) * current.height * 0.5;
        current.root.position.z -= Math.sin(theta) * current.height * 0.5;
        // Rotating about the root, which sits on the soles, swings the whole
        // body out along the travel axis; slide it back so the middle of the
        // robot is on the tube's centreline rather than its feet. Only along
        // the axis — the rotation lays the body flat, so it needs no lift, and
        // lifting it here put the swimmer's back through the ceiling.
      } else {
        current.root.rotation.z = 0;
      }
    }

    // Touchdown -> dust. Each foot is measured where it actually is, so the
    // plume starts at the contact point rather than under the body's origin.
    if (terrain && pose.contacts) {
      if (wasSwinging.length !== pose.contacts.length) wasSwinging = pose.contacts.map(() => false);
      travelDir.set(Math.sin(-theta + Math.PI / 2), 0, Math.cos(-theta + Math.PI / 2));
      for (let i = 0; i < pose.contacts.length; i++) {
        const swinging = pose.contacts[i];
        if (wasSwinging[i] && !swinging && current.feet[i]) {
          footBox.setFromObject(current.feet[i]);
          footBox.getCenter(contactAt);
          contactAt.y = footBox.min.y;
          // Landing speed sets the plume: a lunar bound arrives with far more
          // vertical velocity than an Earth walk, off a longer fall.
          dust.burst(contactAt, Math.min(1, 0.35 + v * 0.28), travelDir);
        }
        wasSwinging[i] = swinging;
      }
    }

    // Aim at the middle of the body. Standing, that is a bit over half its
    // height above the ground; lying flat down a corridor, the body IS the
    // centreline and the same offset aims the camera at the ceiling above it.
    const aimY = (terrain ? groundY + current.height * 0.55
                          : current.root.position.y + (prone ? 0 : current.height * 0.55));
    subject.set(x, aimY, z);
    stage.update(dt, subject, theta);

    const T = gait.stepPeriod(env.g);
    hud.row('g', 'gravity', env.g.toFixed(2), 'm/s²');
    hud.row('rel', 'relative', env.g === 0 ? '0' : gRatio(env).toFixed(3), 'g⊕');
    hud.row('T', 'step period', T.toFixed(3), 's');
    hud.row('cad', 'cadence', (1 / (T * 2)).toFixed(2), 'Hz');
    hud.row('cp', 'capture point', gait.capturePoint(env.g, v).toFixed(2), 'm');
    hud.row('v', 'speed', v.toFixed(2), 'm/s');
    hud.row('duty', 'duty factor', pose.duty.toFixed(2), '');
    hud.row('gait', 'gait', currentMotion, '');
    hud.row('slope', 'slope', (hudSlope * 180 / Math.PI).toFixed(1), '°');
    if (terrain) {
      // Two numbers that used to be invisible: which end of the sole is
      // carrying, and how far the deepest contact point is below the ground.
      // The second must read 0.0 mm; anything else is a foot in the floor.
      hud.row('ct', 'sole contact', footing.contact || '—', '');
      hud.row('pen', 'sole into ground',
        (penetration(current, (x, z) => terrain.heightAt(x, z)) * 1000).toFixed(1), 'mm');
    }
    hud.row('src', 'source', motionLabel, '');
    if (pose.stroke) {
      // The stroke's own state, so the amplitude ramp and the emergency stop
      // are visible rather than something to take on trust.
      const k = pose.stroke;
      hud.row('bf', 'stroke', k.estopped ? 'E-STOP' : 'butterfly', '');
      hud.row('bfa', 'amplitude', (k.amplitude * 100).toFixed(0), '%');
      hud.row('bff', 'stroke rate', k.frequency.toFixed(2), 'Hz');
    } else {
      hud.dropRow('bf'); hud.dropRow('bfa'); hud.dropRow('bff');
    }
    if (!terrain) { hud.dropRow('ct'); hud.dropRow('pen'); }
    hud.row('vs', 'vs 1 g', `×${(gait.stepPeriod(G_EARTH) > 0 ? T / gait.stepPeriod(G_EARTH) : 1).toFixed(2)}`, '');
  }

  if (dashboard.active) {
    dashboard.update(dt);
    dashboard.render(innerWidth, innerHeight);
  } else {
    stage.render();
  }
  requestAnimationFrame(frame);
}

// Debug handle: lets a headless check compare the robot's facing against its
// actual displacement, rather than eyeballing a screenshot.
// Camera handle for headless framing checks.
window.__stage = stage;

window.__arena = {
  get robot() { return current; },
  get heading() { return courseHeading; },
  get env() { return env; },
  get tube() { return tube; },
  get footing() { return footing; },
  get stroke() { return gait?.butterfly ? {
    estopped: gait.estopped, t: gait.swimT, cfg: gait.butterfly.cfg } : null; },
  estop: () => gait?.estop('headless-check'),
  /**
   * How far the deepest sole contact point is below the ground, in mm.
   * This is the number "the feet do not go under the ground" reduces to, so
   * it is exported rather than left to a screenshot.
   */
  penetrationMM: () => (terrain && current
    ? penetration(current, (x, z) => terrain.heightAt(x, z)) * 1000 : 0),
};

/** Terrain gradient by central difference — dh/dx, dh/dz. */
function gradient(t, x, z, h = 0.6) {
  return [
    (t.heightAt(x + h, z) - t.heightAt(x - h, z)) / (2 * h),
    (t.heightAt(x, z + h) - t.heightAt(x, z - h)) / (2 * h),
  ];
}

const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clampNum = (v, a, b) => Math.min(b, Math.max(a, v));
function resize() { stage.resize(innerWidth, innerHeight); }
addEventListener('resize', resize);
resize();
requestAnimationFrame(frame);
boot.classList.add('gone');
