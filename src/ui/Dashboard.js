/**
 * Comparison dashboard: one robot, one motion, four gravitational fields,
 * running at once.
 *
 * The experimental design is the point. Every pane shares the same robot, the
 * same controller and the same phase clock; the ONLY thing that differs
 * between them is g. So any difference you can see — cadence, flight time,
 * stride, how far the foot has to be placed to stop — is attributable to
 * gravity and to nothing else.
 *
 * Four panes means four scenes, because the lighting, sky and ground bounce
 * differ per field and a single scene cannot hold four environments. They
 * share one WebGL context via scissored viewports, and the robot geometry is
 * cloned rather than reloaded so four G1s cost one G1's worth of meshes.
 */
import { Scene, PerspectiveCamera, DirectionalLight, HemisphereLight, Vector3,
         Color, FogExp2, Box3, Group } from 'three';
import { ENVIRONMENTS, sunDir, gRatio } from '../render/Environments.js';
import { buildTerrain } from '../render/Terrain.js';
import { buildRocks } from '../render/Rocks.js';
import { IBL } from '../render/IBL.js';
import { Gait } from '../sim/Gait.js';

const COURSE = 120;
const FREE_FALL_Y = 1.6;   // where the ISS body drifts, with no floor to define it

export class Dashboard {
  constructor(renderer, uiRoot) {
    this.renderer = renderer;
    this.ibl = new IBL(renderer);
    this.panes = [];
    this.active = false;
    this.travelled = 0;

    this.el = document.createElement('div');
    this.el.className = 'dash';
    this.el.innerHTML = `<table class="dash-table"><thead><tr>
        <th>field</th><th>g</th><th>step T</th><th>cadence</th>
        <th>capture pt</th><th>duty</th><th>vs 1g</th></tr></thead><tbody></tbody></table>`;
    this.tbody = this.el.querySelector('tbody');
    uiRoot.appendChild(this.el);

    this.labels = document.createElement('div');
    this.labels.className = 'dash-labels';
    uiRoot.appendChild(this.labels);

    this._box = new Box3();
    this._subject = new Vector3();
  }

  /**
   * Build four panes around one loaded robot.
   * @param {object} loaded  result of loadRobot()
   */
  build(loaded, mode = 'walk') {
    this.dispose();
    this.robotDef = loaded.def;

    for (const env of ENVIRONMENTS) {
      const scene = new Scene();
      scene.fog = new FogExp2(env.skyColor.getHex(), env.fogDensity);

      const sun = new DirectionalLight(env.sunColor, env.sunIntensity);
      sun.castShadow = true;
      sun.shadow.mapSize.set(1024, 1024);
      const c = sun.shadow.camera;
      c.left = -14; c.right = 14; c.top = 14; c.bottom = -14; c.near = 0.5; c.far = 120;
      sun.shadow.normalBias = 0.035;
      scene.add(sun, sun.target);

      const fill = new HemisphereLight(env.skyColor, env.groundAlbedo, env.ambientIntensity);
      scene.add(fill);
      scene.environment = this.ibl.update(env);

      // Lower-resolution ground: four panes at full terrain resolution is
      // 1.2M triangles of scenery nobody is looking closely at.
      const terrain = buildTerrain(env, { seg: 168 });
      if (terrain) scene.add(terrain);
      const rocks = terrain
        ? buildRocks(env, terrain.heightAt, (env.sunAz - 90) * Math.PI / 180) : null;
      if (rocks) scene.add(rocks);

      // Clone shares geometry and materials; only the transform tree is new.
      const root = loaded.root.clone(true);
      scene.add(root);
      const robot = this._findRobot(root);
      const feet = (loaded.def.feet || [])
        .map((n) => robot?.links?.[n]).filter(Boolean);

      const camera = new PerspectiveCamera(34, 1, 0.05, 2000);
      // The ISS pane always swims: a contact gait in free fall would be a
      // robot walking on nothing, which is worse than no comparison at all.
      const gait = new Gait(loaded.def, loaded.height * 0.52,
                            env.g <= 1e-4 ? 'swim' : mode);

      this.panes.push({ env, scene, camera, sun, terrain, rocks, root, robot, feet, gait,
                        height: loaded.height });
    }
    this._paintLabels();
  }

  /** urdf-loader's clone keeps the joint map on the URDFRobot node itself. */
  _findRobot(root) {
    let found = null;
    root.traverse((o) => { if (!found && o.joints && o.links) found = o; });
    return found;
  }

  _paintLabels() {
    this.labels.innerHTML = '';
    for (const p of this.panes) {
      const d = document.createElement('div');
      d.className = 'dash-label';
      d.innerHTML = `<b>${p.env.short}</b><span>${p.env.g.toFixed(2)} m/s²</span>`;
      this.labels.appendChild(d);
    }
  }

  show(on) {
    this.active = on;
    this.el.classList.toggle('in', on);
    this.labels.classList.toggle('in', on);
  }

  update(dt) {
    if (!this.active || !this.panes.length) return;
    let rows = '';
    for (const p of this.panes) {
      const { v } = p.gait.advance(dt, p.env.g);
      const pose = p.gait.evaluate(p.env.g);
      if (p.robot) {
        for (const [name, value] of Object.entries(pose.joints)) {
          const j = p.robot.joints[name];
          if (j) j.setJointValue(value);
        }
      }

      // Each pane walks its OWN distance: the whole point is that the same
      // controller covers different ground in different fields.
      p.travelled = (p.travelled || 0) + v * dt;
      const along = p.travelled % COURSE - COURSE * 0.5;
      const heading = (p.env.sunAz - 90) * Math.PI / 180;
      const x = Math.cos(heading) * along, z = Math.sin(heading) * along;
      const groundY = p.terrain ? p.terrain.heightAt(x, z) : 0;
      p.root.position.set(x, groundY, z);
      p.root.rotation.y = -heading;   // local +X is forward; see main.js
      p.root.updateMatrixWorld(true);

      if (p.terrain && p.feet.length) {
        let lowest = Infinity;
        for (const f of p.feet) { this._box.setFromObject(f); lowest = Math.min(lowest, this._box.min.y); }
        if (isFinite(lowest)) {
          p.root.position.y += (groundY - lowest) + pose.bodyY;
          p.root.updateMatrixWorld(true);
        }
      } else if (!p.terrain) {
        // Free fall: nothing to stand on, so the body simply drifts. The
        // camera has to follow it up there — aiming at a ground plane that
        // does not exist puts the robot out of frame entirely.
        p.root.position.y = FREE_FALL_Y;
      }

      // Identical framing in every pane, so pane-to-pane differences are the
      // robot's and not the camera's.
      const aimY = (p.terrain ? groundY : FREE_FALL_Y) + p.height * 0.55;
      this._subject.set(x, aimY, z);
      const s = Math.sin(heading), c = Math.cos(heading);
      const ox = -2.4, oz = 2.4;
      p.camera.position.set(
        this._subject.x + ox * c - oz * s,
        this._subject.y + 1.0,
        this._subject.z + ox * s + oz * c);
      p.camera.lookAt(this._subject);
      p.sun.position.copy(this._subject).add(sunDir(p.env.sunElev, p.env.sunAz).multiplyScalar(40));
      p.sun.target.position.copy(this._subject);
      p.sun.target.updateMatrixWorld();

      const T = p.gait.stepPeriod(p.env.g);
      const base = p.gait.stepPeriod(9.80665);
      rows += `<tr><td>${p.env.short}</td><td>${p.env.g.toFixed(2)}</td>`
           + `<td>${T.toFixed(3)}s</td><td>${(1 / (T * 2)).toFixed(2)}Hz</td>`
           + `<td>${p.gait.capturePoint(p.env.g, v).toFixed(2)}m</td>`
           + `<td>${pose.duty.toFixed(2)}</td>`
           + `<td>×${(T / base).toFixed(2)}</td></tr>`;
    }
    this.tbody.innerHTML = rows;
  }

  /** Scissored 2x2 render into the shared context. */
  render(w, h) {
    if (!this.active || !this.panes.length) return;
    const halfW = Math.floor(w / 2), halfH = Math.floor(h / 2);
    this.renderer.setScissorTest(true);
    this.panes.forEach((p, i) => {
      const cx = (i % 2) * halfW;
      const cy = h - (Math.floor(i / 2) + 1) * halfH;   // GL origin is bottom-left
      this.renderer.setViewport(cx, cy, halfW, halfH);
      this.renderer.setScissor(cx, cy, halfW, halfH);
      p.camera.aspect = halfW / halfH;
      p.camera.updateProjectionMatrix();
      this.renderer.toneMappingExposure = p.env.exposure;
      this.renderer.render(p.scene, p.camera);
    });
    this.renderer.setScissorTest(false);
    this.renderer.setViewport(0, 0, w, h);
  }

  dispose() {
    for (const p of this.panes) {
      if (p.terrain) { p.terrain.geometry.dispose(); p.terrain.material.dispose(); }
      if (p.rocks) { p.rocks.geometry.dispose(); p.rocks.material.dispose(); }
    }
    this.panes.length = 0;
  }
}
