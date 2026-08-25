/**
 * GravityCompare — three G1s, one motion, three gravitational fields, in one
 * shot.
 *
 * The arena can already show a robot on the Moon and then, separately, on
 * Mars. That does not answer the question, because nobody can hold a gait in
 * their head for the four seconds it takes to switch scenes. The difference
 * only becomes legible when the SAME motion runs at the SAME instant in three
 * fields, and the three bodies pull apart on screen.
 *
 * They start on the same line, in phase, with identical joint templates. What
 * separates them is only g. Within a couple of seconds the Moon robot is a
 * stride ahead and airborne most of the time while the Earth robot is taking
 * quick short steps with a foot down two thirds of the time — which is the
 * whole claim, made visible rather than tabulated.
 *
 *   T = 2*pi*sqrt(L/g)/4      pendulum quarter-swing of the leg
 *   x = v*sqrt(L/g)           capture point: where the foot must land
 */
import { Vector3, Box3, Group } from 'three';
import { plantOnSoles, conformAnkles, penetration } from '../sim/Footing.js';
import { Stage } from '../core/Stage.js';
import { Vector3 as V3 } from 'three';
import { byId, sunDir } from '../render/Environments.js';
import { robotById, loadRobot } from '../render/Robots.js';
import { buildTerrain } from '../render/Terrain.js';
import { Gait, MOTIONS } from '../sim/Gait.js';

// The three fields, and the lane each one walks in. Ordered heaviest first so
// the robot that falls behind is nearest the camera and reads as the baseline.
const LANES = [
  { id: 'earth', label: 'EARTH', g: 9.807, z: -2.5, tint: 0x7fa8ff },
  { id: 'mars',  label: 'MARS',  g: 3.721, z:  0.0, tint: 0xff8f5a },
  { id: 'moon',  label: 'MOON',  g: 1.625, z:  2.5, tint: 0xe6ecf4 },
];

const COURSE = 90;                      // metres before the lane wraps

/**
 * This view needs its own shot, and none of the arena's will do.
 *
 * Every shot in Stage.SHOTS frames ONE subject close enough to read a gait.
 * Here the subject is the GAP: after ten seconds Earth is 8.5 m ahead of the
 * Moon, and a camera tight enough to show a stride cannot hold that. So the
 * camera sits well back and high, square to the direction of travel, which
 * puts the separation across the screen where it is legible.
 *
 * The lanes are kept only 2.5 m apart in depth against a 21 m standoff, so
 * the three robots render at within a few percent of the same size. Spread
 * them wider and the nearest one looks bigger, which reads as "closer to the
 * camera" and undoes the comparison.
 */
const WIDE = { offset: new V3(-1.5, 7.4, 21), aim: 0.85, fov: 36 };

export class GravityCompare {
  constructor(canvas, uiRoot) {
    this.stage = new Stage(canvas);
    this.ui = uiRoot;
    this.lanes = [];
    this.motion = 'walk';
    this.running = false;
    this.elapsed = 0;
    this.paused = false;
    this._box = new Box3();
    // One ground function, shared by conform and plant, so they can never be
    // asked about two different surfaces.
    this._heightAt = (x, z) => (this.terrain ? this.terrain.heightAt(x, z) : 0);
    this._subject = new Vector3();
    this._buildUI();
  }

  async start(motion = 'walk') {
    this.motion = motion;
    const env = byId('moon');                 // one sky and one ground for all
    this.stage.applyEnvironment(env, sunDir(env.sunElev, env.sunAz));
    this.stage.setSunOffset(sunDir(env.sunElev, env.sunAz));
    if (!this.terrain) {
      this.terrain = buildTerrain(env);
      if (this.terrain) this.stage.world.add(this.terrain);
    }

    const def = robotById('g1');
    for (const lane of LANES) {
      const loaded = await loadRobot(def);     // a separate instance per lane
      // Tint the shell to match this lane's readout. Three identical grey
      // robots is exactly as unreadable as one: the whole point is knowing at
      // a glance which body is in which field.
      loaded.root.traverse((o) => {
        if (!o.isMesh || !o.material) return;
        o.material = o.material.clone();
        o.material.color.setHex(lane.tint);
      });
      loaded.root.position.set(0, 0, lane.z);
      this.stage.world.add(loaded.root);
      this.lanes.push({
        ...lane, loaded,
        gait: new Gait(def, loaded.height * 0.52, motion),
        travelled: 0, wasSwinging: [],
      });
    }
    this._reset();
    this.stage.cut(WIDE);
    this.running = true;
  }

  /** Put all three back on the start line, in phase. */
  _reset() {
    this.elapsed = 0;
    for (const lane of this.lanes) {
      lane.travelled = 0;
      lane.gait = new Gait(robotById('g1'), lane.loaded.height * 0.52, this.motion);
    }
  }

  setMotion(m) {
    this.motion = m;
    this._reset();
    for (const b of this.ui.querySelectorAll('[data-motion]')) {
      b.classList.toggle('on', b.dataset.motion === m);
    }
  }

  _buildUI() {
    const el = document.createElement('div');
    el.className = 'gc';
    el.innerHTML = `
      <div class="gc-head">
        <h1>One motion. Three gravitational fields.</h1>
        <p>The same Unitree G1, the same joint template, started on the same
           line in phase. The only difference is <b>g</b>. Cadence, stride,
           duty factor and flight time are computed from it, not chosen.</p>
      </div>
      <div class="gc-modes"></div>
      <div class="gc-readout"></div>
      <div class="gc-foot">
        <button data-act="reset">restart in phase</button>
        <button data-act="pause">pause</button>
        <span class="gc-eq">T = 2&pi;&radic;(L/g) / 4 &nbsp;·&nbsp; capture point = v&radic;(L/g)</span>
      </div>`;
    const modes = el.querySelector('.gc-modes');
    for (const m of MOTIONS) {
      if (m.issOnly) continue;                 // no ground here means no swim
      const b = document.createElement('button');
      b.dataset.motion = m.id;
      b.textContent = m.name;
      b.className = m.id === 'walk' ? 'on' : '';
      b.addEventListener('click', () => this.setMotion(m.id));
      modes.appendChild(b);
    }
    el.querySelector('[data-act="reset"]').addEventListener('click', () => this._reset());
    el.querySelector('[data-act="pause"]').addEventListener('click', (e) => {
      this.paused = !this.paused;
      e.target.textContent = this.paused ? 'resume' : 'pause';
    });
    this.ui.appendChild(el);
    this.readout = el.querySelector('.gc-readout');

    // Labels pinned to each robot in the world, not to a fixed corner. A
    // legend the reader has to map back onto three near-identical figures is
    // the same problem as no legend.
    this.tags = document.createElement('div');
    this.tags.className = 'gc-tags';
    for (const lane of LANES) {
      const t = document.createElement('div');
      t.className = 'gc-tag';
      t.dataset.lane = lane.id;
      t.style.color = `#${lane.tint.toString(16).padStart(6, '0')}`;
      t.innerHTML = `<b>${lane.label}</b><i></i>`;
      this.tags.appendChild(t);
    }
    this.ui.appendChild(this.tags);
  }

  /**
   * Plant on the soles, the same way the main arena does — through the same
   * Footing code, so the three lanes and the single-robot page cannot drift
   * apart on the one thing they must agree about.
   */
  _plant(loaded, hop) {
    return plantOnSoles(loaded, this._heightAt, hop);
  }

  /**
   * Deepest sole point below the ground, per lane, in millimetres.
   *
   * Exposed because "the feet do not go through the floor" is a claim, and a
   * screenshot cannot check a claim. tools/check_gravity.mjs asserts on this.
   */
  penetrationMM() {
    return this.lanes.map((l) => ({
      label: l.label,
      mm: penetration(l.loaded, this._heightAt) * 1000,
      contact: l.footing?.contact || null,
    }));
  }

  frame(dt) {
    if (!this.running) return;
    if (!this.paused) this.elapsed += dt;
    let lead = -Infinity;

    for (const lane of this.lanes) {
      const step = this.paused ? 0 : dt;
      const { v } = lane.gait.advance(step, lane.g);
      lane.travelled += v * step;
      const pose = lane.gait.evaluate(lane.g);
      for (const [name, value] of Object.entries(pose.joints)) {
        const j = lane.loaded.joints[name];
        if (j) j.setJointValue(value);
      }
      const x = (lane.travelled % COURSE) - COURSE * 0.5;
      const groundY = this.terrain ? this.terrain.heightAt(x, lane.z) : 0;
      lane.loaded.root.position.set(x, groundY, lane.z);
      lane.loaded.root.rotation.y = 0;          // all three walk +x, side-on
      conformAnkles(lane.loaded, pose, this._heightAt);
      lane.footing = this._plant(lane.loaded, pose.bodyY);
      lane.speed = v;
      lane.pose = pose;
      lead = Math.max(lead, x);
    }

    // Frame the GROUP, not one robot: the point of the shot is the gap that
    // opens between them, so the camera has to hold all three.
    // Aim at the middle of the SPREAD, so both the leader and the straggler
    // stay in frame as the gap opens, rather than at the mean position, which
    // drifts toward whichever lane happens to be fastest.
    const xs = this.lanes.map((l) => l.loaded.root.position.x);
    const mid = (Math.min(...xs) + Math.max(...xs)) / 2;
    this._subject.set(mid, (this.terrain ? this.terrain.heightAt(mid, 0) : 0) + 0.75, 0);
    this.stage.update(dt, this._subject, 0);
    this.stage.render();
    this._paint();
    this._placeTags();
  }

  _paint() {
    if (this.elapsed - (this._lastPaint || 0) < 0.12) return;
    this._lastPaint = this.elapsed;
    const earthT = this.lanes[0].gait.stepPeriod(this.lanes[0].g);
    this.readout.innerHTML = this.lanes.map((l) => {
      const T = l.gait.stepPeriod(l.g);
      const p = l.pose || { duty: 0 };
      return `<div class="gc-lane">
        <b style="color:#${l.tint.toString(16).padStart(6, '0')}">${l.label}</b>
        <div class="gc-rows">
          <span>gravity</span><i>${l.g.toFixed(2)}</i><u>m/s²</u>
          <span>step period</span><i>${T.toFixed(3)}</i><u>s</u>
          <span>cadence</span><i>${(1 / (T * 2)).toFixed(2)}</i><u>Hz</u>
          <span>duty factor</span><i>${p.duty.toFixed(2)}</i><u></u>
          <span>flight</span><i>${((1 - p.duty) * 100).toFixed(0)}</i><u>%</u>
          <span>stride</span><i>${(l.speed * T).toFixed(2)}</i><u>m</u>
          <span>capture pt</span><i>${l.gait.capturePoint(l.g, l.speed).toFixed(2)}</i><u>m</u>
          <span>distance</span><i>${l.travelled.toFixed(1)}</i><u>m</u>
          <span>vs Earth</span><i>×${(T / earthT).toFixed(2)}</i><u></u>
          <span>sole contact</span><i>${l.footing?.contact || '—'}</i><u></u>
        </div></div>`;
    }).join('');
  }

  /** Project each robot's head into screen space and hang its label there. */
  _placeTags() {
    if (!this.tags) return;
    const cam = this.stage.camera;
    for (const lane of this.lanes) {
      const el = this.tags.querySelector(`[data-lane="${lane.id}"]`);
      if (!el) continue;
      const p = lane.loaded.root.position;
      this._tagV = this._tagV || new Vector3();
      this._tagV.set(p.x, p.y + lane.loaded.height * 1.06, p.z).project(cam);
      // Behind the camera projects to a mirrored point in front of it; hide
      // rather than draw a label on the wrong side of the frame.
      const on = this._tagV.z < 1 && Math.abs(this._tagV.x) < 1.3;
      el.style.display = on ? 'block' : 'none';
      if (!on) continue;
      el.style.left = `${(this._tagV.x * 0.5 + 0.5) * innerWidth}px`;
      el.style.top = `${(-this._tagV.y * 0.5 + 0.5) * innerHeight}px`;
      const T = lane.gait.stepPeriod(lane.g);
      el.querySelector('i').textContent =
        `${lane.g.toFixed(2)} m/s² · ${lane.travelled.toFixed(1)} m`;
    }
  }

  resize(w, h) { this.stage.resize(w, h); }
}
