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
let rocks = null;
let current = null;               // { def, root, robot, joints, height }
let gait = null;
let env = byId('moon');
let travelled = 0;
let courseHeading = 0;
let running = false;
let motionLabel = 'generated';
let currentMotion = 'walk';
let climbing = false;
let climbX = 0, climbZ = 0, climbBestY = -Infinity, climbSeed = 1;
let climbTack = 1, climbTackT = 0;
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
  if (!nextEnv.interior) return;
  interior = await loadInterior(nextEnv.interior);
  if (interior) stage.world.add(interior);
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
  stage.side = 1;

  if (dashboard.panes.length) dashboard.dispose();
  currentMotion = motionId;
  climbing = motionId === 'climb';
  if (climbing && terrain) {
    const start = pickSlope(terrain, climbSeed++, (env.sunAz * Math.PI) / 180);
    climbX = start.x; climbZ = start.z; courseHeading = start.heading;
    climbBestY = terrain.heightAt(climbX, climbZ);
  }
  hud.setTitle(`${current.def.short} · ${env.short}`, `${env.name} · ${motionId}`);
  hud.show(true);

  // The cinematic: begin wide, then settle into the working view. First entry
  // cuts to the establishing frame so there is nothing to fly FROM.
  if (!running) { stage.cut(SHOTS.establish); running = true; }
  else stage.flyTo(SHOTS.establish, 1.1);
  setTimeout(() => stage.flyTo(SHOTS.chase, 3.4), 1300);
}

const lobby = new Lobby(uiRoot, enter);

// Shot keys — the same rig, so every one of these is a move rather than a cut.
addEventListener('keydown', (e) => {
  const map = { '1': SHOTS.establish, '2': SHOTS.chase, '3': SHOTS.hero, '4': SHOTS.profile };
  if (map[e.key]) stage.flyTo(map[e.key], 2.0);
  if (e.key.toLowerCase() === 'l') lobby.show();
  if (e.key.toLowerCase() === 'd') toggleDashboard();
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
// per-foot swing state, so a landing is detected as a transition rather than
// fired every frame the foot happens to be down
let wasSwinging = [];
let last = performance.now();

/**
 * Plant the robot by MEASUREMENT.
 *
 * The model's origin was put on its soles in the neutral pose, but once the
 * joints move the lowest foot is somewhere else entirely — which is why a
 * pose-driven robot appears to hover. Measure where the feet actually ended
 * up, then lift the root so the lowest one rests on the terrain, and add the
 * flight arc on top of that.
 */
function plant(loaded, groundY, hop) {
  loaded.root.updateMatrixWorld(true);
  let lowest = Infinity;
  for (const foot of loaded.feet) {
    footBox.setFromObject(foot);
    if (footBox.min.y < lowest) lowest = footBox.min.y;
  }
  if (!isFinite(lowest)) return;                 // no feet declared: leave as placed
  loaded.root.position.y += (groundY - lowest) + hop;
  loaded.root.updateMatrixWorld(true);
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

    // Climbing STEERS; every other gait runs the straight traverse.
    //
    // A climb animation played along flat ground is just marching. To actually
    // ascend, read the terrain gradient under the robot each frame and turn
    // toward the uphill direction. When the slope runs out — a summit, or a
    // flat — pick a fresh steep start rather than milling about on the top.
    if (climbing && terrain) {
      const sunAzRad = (env.sunAz * Math.PI) / 180;
      const here = terrain.heightAt(climbX, climbZ);
      if (here > climbBestY) climbBestY = here;

      const [gx, gz] = gradient(terrain, climbX, climbZ);
      const steep = Math.hypot(gx, gz);
      // Summited, or heading back down the far side. Gradient magnitude alone
      // cannot tell those apart — a downhill is exactly as steep as the climb
      // that led to it — so the test is loss of ALTITUDE, not loss of slope.
      const summited = steep < 0.05;
      const descending = here < climbBestY - 0.8;
      if (summited || descending) {
        const start = pickSlope(terrain, climbSeed++, sunAzRad);
        climbX = start.x; climbZ = start.z; courseHeading = start.heading;
        climbBestY = terrain.heightAt(climbX, climbZ);
      } else {
        // Uphill is where height rises fastest — but straight up a face the
        // feet cannot hold is not climbing, it is clipping. Past the traversable
        // limit, switchback: follow the contour, angled uphill as far as the
        // slope allows. This is what a person does on a steep hill too.
        let want = Math.atan2(gz, gx);
        if (steep > MAX_CLIMB_TAN) {
          const contour = want + Math.PI / 2 * climbTack;
          const blend = Math.min(1, (steep - MAX_CLIMB_TAN) / MAX_CLIMB_TAN);
          want = want + wrapAngle(contour - want) * blend;
          // flip the tack now and then so it zig-zags up rather than
          // circling the hill forever
          climbTackT += dt;
          if (climbTackT > 4.5) { climbTackT = 0; climbTack *= -1; }
        }
        courseHeading += wrapAngle(want - courseHeading) * Math.min(1, dt * 3.0);
      }
      climbX += Math.cos(courseHeading) * v * dt;
      climbZ += Math.sin(courseHeading) * v * dt;
    }

    // Straight traverse. Wrapping rather than turning around keeps the heading
    // — and therefore the lit side — constant for the whole run.
    // A module is 14 m long, not 120. Traversing the surface course in free
    // fall swims the robot straight out through the wall and leaves it alone
    // against the stars, which is exactly the reference the interior exists to
    // provide. Keep an orbital run inside the module.
    const courseLen = terrain ? COURSE_LENGTH : ISS_COURSE;
    const along = travelled % courseLen;
    if (along < lastAlong) { dust.clear(); stage.cut(stage.shot); }   // wrapped
    lastAlong = along;
    const theta = courseHeading;
    const s0 = along - courseLen * 0.5;
    const x = climbing && terrain ? climbX : Math.cos(theta) * s0;
    const z = climbing && terrain ? climbZ : Math.sin(theta) * s0;
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
    if (terrain) plant(current, groundY, pose.bodyY);
    // Free fall: nothing to plant against, so the body just drifts. The camera
    // has to follow it up there — aiming at a ground plane that does not exist
    // leaves the robot out of frame.
    else current.root.position.y = FREE_FALL_Y + Math.sin(travelled * 0.35) * 0.45;

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

    subject.set(x, (terrain ? groundY : current.root.position.y) + current.height * 0.55, z);
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
    hud.row('src', 'source', motionLabel, '');
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
window.__arena = {
  get robot() { return current; },
  get heading() { return courseHeading; },
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
// Steepest grade the gait will take head-on (~26 deg); beyond this it tacks.
const MAX_CLIMB_TAN = 0.49;

/**
 * Find somewhere worth climbing: sample the field and take the steepest spot
 * that is still walkable. Too gentle and the gait has nothing to do; too steep
 * and the feet cannot stay planted.
 */
function pickSlope(t, seed = 0, sunAzRad = 0) {
  // Look for the FOOT of a climb, not merely a steep spot: somewhere with a
  // real slope that still has height above it. Ranking by steepness alone
  // parks the robot on the steepest face it can find, which is often just
  // below a summit and gives it two strides before it runs out of hill.
  let best = { x: 0, z: 0, heading: 0, score: -1 };
  const jitter = (seed * 0.618033) % 1;
  for (let i = 0; i < 260; i++) {
    const a = (i + jitter * 260) * 2.399963;      // golden angle, offset per call
    const r = 22 + ((i / 260 + jitter) % 1) * 130;
    const x = Math.cos(a) * r, z = Math.sin(a) * r;
    const [gx, gz] = gradient(t, x, z);
    const steep = Math.hypot(gx, gz);
    if (steep > 0.62 || steep < 0.08) continue;   // cliffs and flats are both useless
    // how much height lies ahead if we walk uphill from here
    const ux = gx / steep, uz = gz / steep;
    const ahead = t.heightAt(x + ux * 26, z + uz * 26) - t.heightAt(x, z);
    // Prefer a face the sun is ON. Ascending away from the sun is perfectly
    // valid climbing and completely unwatchable: the robot spends the whole
    // run on the shadowed side of the hill it is climbing.
    const lit = 0.5 + 0.5 * (ux * Math.cos(sunAzRad) + uz * Math.sin(sunAzRad));
    const score = ahead * (0.25 + 0.75 * lit);
    if (score > best.score) best = { x, z, heading: Math.atan2(gz, gx), score };
  }
  return best.score > 0 ? best : { x: 0, z: 0, heading: 0, score: 0 };
}

function resize() { stage.resize(innerWidth, innerHeight); }
addEventListener('resize', resize);
resize();
requestAnimationFrame(frame);
boot.classList.add('gone');
