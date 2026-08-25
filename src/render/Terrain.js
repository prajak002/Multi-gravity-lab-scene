/**
 * Ground. A displaced grid with craters stamped into it — enough relief for
 * shadows to describe the surface, which is what sells scale.
 */
import { PlaneGeometry, Mesh, MeshStandardMaterial, Color, DoubleSide } from 'three';

const EXTENT = 460;        // metres across — a horizon you can believe
const SEG = 384;

/** Cheap value noise; deterministic per seed so a scene looks the same twice. */
function makeNoise(seed) {
  const hash = (x, y) => {
    const n = Math.sin(x * 127.1 + y * 311.7 + seed * 74.7) * 43758.5453;
    return n - Math.floor(n);
  };
  const smooth = (t) => t * t * (3 - 2 * t);
  return (x, y) => {
    const xi = Math.floor(x), yi = Math.floor(y);
    const xf = x - xi, yf = y - yi;
    const u = smooth(xf), v = smooth(yf);
    return (hash(xi, yi) * (1 - u) + hash(xi + 1, yi) * u) * (1 - v)
         + (hash(xi, yi + 1) * (1 - u) + hash(xi + 1, yi + 1) * u) * v;
  };
}

export function buildTerrain(env, opts = {}) {
  if (!env.terrain) return null;                    // ISS has no ground
  const seg = opts.seg || SEG;
  const { amp, scale, craters } = env.terrain;
  const noise = makeNoise(env.id.length * 37 + craters);
  const bowlCount = craters ? craters + 26 : 0;

  // Crater bowls, placed once and kept off the middle so the robots have a
  // clear traverse rather than spawning inside a hole.
  const bowls = [];
  for (let i = 0; i < bowlCount; i++) {
    const a = noise(i * 3.1, 7.7) * Math.PI * 2;
    const rad = 16 + noise(i * 1.7, 2.3) * (EXTENT * 0.44);
    // Craters scale with distance: small ones you can walk around near the
    // course, basin-sized ones out where they read as landscape.
    const far = Math.min(1, rad / (EXTENT * 0.4));
    bowls.push({
      x: Math.cos(a) * rad, z: Math.sin(a) * rad,
      r: 2.5 + noise(i, 11.3) * 9 + far * 34,
      d: 0.35 + noise(i, 5.9) * 1.5 + far * 5.5,
    });
  }

  const geo = new PlaneGeometry(EXTENT, EXTENT, seg, seg);
  geo.rotateX(-Math.PI / 2);
  /**
   * One height function, used for BOTH the mesh and the foot sampler. Two
   * copies of this drift apart the moment either is edited, and the robot
   * starts sinking into ground that looks solid.
   *
   * The distance ramp is what gives the scene scale: near the course the
   * ground stays walkable, and beyond ~90 m the amplitude climbs into ranges
   * that give the horizon something to be measured against.
   */
  const heightAt = (x, z) => {
    const r = Math.hypot(x, z);
    // Relief starts rising much closer in than it used to. The far ranges
    // still give the horizon its scale, but the traverse needs real slope
    // within walking distance of the course, not 90 m away — it is what the
    // per-foot ground sampling in Footing.js has to cope with.
    const far = Math.min(1, Math.max(0, (r - 26) / 150));
    const relief = amp * (1 + far * 8.5);
    let h = 0, f = 1, a = relief;
    for (let o = 0; o < 5; o++) { h += noise(x * scale * f, z * scale * f) * a; f *= 2.07; a *= 0.5; }
    h -= relief * 0.5;                    // centre the noise so near ground is level
    for (const b of bowls) {
      const d = Math.hypot(x - b.x, z - b.z);
      if (d >= b.r) continue;
      const t = d / b.r;
      // bowl floor with a raised rim, the shape that reads as an impact crater
      h += (-b.d * (1 - t * t) + b.d * 0.42 * Math.exp(-((t - 0.92) ** 2) * 60));
    }
    return h;
  };

  const pos = geo.attributes.position;
  for (let i = 0; i < pos.count; i++) pos.setY(i, heightAt(pos.getX(i), pos.getZ(i)));
  geo.computeVertexNormals();

  const mat = new MeshStandardMaterial({
    color: new Color().copy(env.groundAlbedo).multiplyScalar(2.1),  // albedo -> base colour
    roughness: env.terrain.roughness, metalness: 0.0, side: DoubleSide,
  });
  const mesh = new Mesh(geo, mat);
  mesh.receiveShadow = true;
  mesh.castShadow = false;      // self-shadowing a 320^2 grid buys nothing here
  mesh.name = 'terrain';

  mesh.heightAt = heightAt;      // the same function the vertices were built from
  return mesh;
}
