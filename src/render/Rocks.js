/**
 * Boulder fields.
 *
 * Two jobs. First, scale: a bare displaced plane gives an audience nothing of
 * known size to measure the robot against, and rocks at a spread of sizes do.
 * Second, the climb gait needs something to be climbing — a high knee lift on
 * flat ground reads as marching, not ascending.
 *
 * One InstancedMesh per environment: a few hundred boulders cost one draw
 * call, and the per-instance variation comes from the transform rather than
 * from unique geometry.
 */
import {
  IcosahedronGeometry, InstancedMesh, MeshStandardMaterial, Object3D, Color,
  DynamicDrawUsage, Matrix4,
} from 'three';

const COUNT = 200;

/** Deterministic per-scene scatter, so a field looks the same twice. */
function rng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/**
 * A rock, not a ball: displace an icosahedron per-vertex so the silhouette is
 * faceted and irregular. Shared by every instance — the variety a viewer reads
 * comes from scale and rotation, not from 260 unique meshes.
 */
function boulderGeometry(detail = 2) {
  const geo = new IcosahedronGeometry(1, detail);
  const pos = geo.attributes.position;
  const r = rng(9137);
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    // Gentle per-vertex displacement. Pushed harder than this the icosahedron
    // stops reading as weathered stone and starts reading as a crystal shard.
    const k = 0.84 + r() * 0.26;
    pos.setXYZ(i, x * k, y * k * 0.80, z * k);   // squat: rocks sit, they do not float
  }
  geo.computeVertexNormals();
  return geo;
}

/**
 * @param {object} env      environment definition
 * @param {(x:number,z:number)=>number} heightAt  terrain sampler
 * @param {number} courseHeading  radians; the traverse the robot walks
 */
export function buildRocks(env, heightAt, courseHeading) {
  if (!env.terrain) return null;              // nothing to rest on in free fall

  const geo = boulderGeometry(2);
  const mat = new MeshStandardMaterial({
    // Rock has to be plainly DARKER than the regolith around it. The terrain
    // renders its albedo at x2.1; matching that makes a boulder field vanish
    // into the ground and read as pale shards rather than stone.
    color: new Color().copy(env.groundAlbedo).multiplyScalar(0.95),
    roughness: 1.0, metalness: 0.0, flatShading: true,
    envMapIntensity: 0.55,
  });

  const mesh = new InstancedMesh(geo, mat, COUNT);
  mesh.instanceMatrix.setUsage(DynamicDrawUsage);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.name = 'rocks';

  const r = rng(env.id.length * 7919 + 17);
  const dummy = new Object3D();
  const ux = Math.cos(courseHeading), uz = Math.sin(courseHeading);   // along the course
  const rx = -uz, rz = ux;                                            // across it

  for (let i = 0; i < COUNT; i++) {
    let x, z, scale;
    if (i < COUNT * 0.16) {
      // A ridge of climbable rock ALONG the course, offset to one side so it
      // is in shot without the robot walking through it.
      const s = (r() - 0.5) * 90;
      const off = (3.4 + r() * 4.0) * (r() < 0.5 ? 1 : -1);
      x = ux * s + rx * off;
      z = uz * s + rz * off;
      scale = 0.30 + r() * 0.85;
    } else {
      // Scattered field, thinning with distance.
      const a = r() * Math.PI * 2;
      const rad = 12 + Math.pow(r(), 0.6) * 190;
      x = Math.cos(a) * rad;
      z = Math.sin(a) * rad;
      scale = 0.3 + r() * (0.9 + rad / 110);
    }
    // Bed each rock BELOW the surface it sits on. A boulder resting exactly on
    // a heightfield shows a hairline gap wherever the ground slopes away, and
    // a field of them reads as floating.
    // Bed shallowly. Sink a squat rock too far and the terrain clips its
    // waist, leaving only flat top facets showing — which reads as shattered
    // glass, not stone.
    const vy = scale * (0.88 + r() * 0.34);
    const y = heightAt(x, z) - vy * 0.10;
    dummy.position.set(x, y, z);
    dummy.rotation.set(r() * 0.5 - 0.25, r() * Math.PI * 2, r() * 0.5 - 0.25);
    dummy.scale.set(scale, vy, scale * (0.88 + r() * 0.24));
    dummy.updateMatrix();
    mesh.setMatrixAt(i, dummy.matrix);
  }
  mesh.instanceMatrix.needsUpdate = true;
  mesh.computeBoundingSphere();
  return mesh;
}
