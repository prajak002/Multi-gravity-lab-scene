/**
 * The ISS module interior.
 *
 * On the surface scenes the ground gives an audience its sense of scale and
 * motion — a shadow, a horizon, footprints. In free fall there is none of
 * that, and a robot swimming against a starfield has nothing to be moving
 * RELATIVE to. The module walls are what restore that: they are the only
 * reason an ISS run reads as travel rather than as a figure waving in place.
 */
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { Box3, Vector3, MeshStandardMaterial, Color } from 'three';

// This asset ships with EXT_meshopt_compression, so the decoder has to be
// registered before the first load or GLTFLoader refuses the file outright.
const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
const cache = new Map();

export async function loadInterior(url) {
  if (cache.has(url)) return cache.get(url).clone(true);

  const gltf = await loader.loadAsync(url);
  const root = gltf.scene;
  root.name = 'interior';

  // The module is lit by the same IBL as everything else; give it a plainly
  // engineered surface rather than trusting whatever the export carried.
  root.traverse((o) => {
    if (!o.isMesh) return;
    o.material = new MeshStandardMaterial({
      color: new Color(0xbfc4cc), metalness: 0.25, roughness: 0.62,
      envMapIntensity: 1.0,
    });
    o.castShadow = false;      // there is no sun in here to cast from
    o.receiveShadow = false;
  });

  // Sit the module around the origin and scale it so a 1.3 m robot fits the
  // way a person fits a real module — measured, not eyeballed.
  const box = new Box3().setFromObject(root);
  const size = new Vector3(); box.getSize(size);
  const centre = new Vector3(); box.getCenter(centre);
  const longest = Math.max(size.x, size.y, size.z) || 1;
  const TARGET_LENGTH = 14;                     // metres of module along its long axis
  const k = TARGET_LENGTH / longest;
  root.scale.setScalar(k);
  root.position.copy(centre).multiplyScalar(-k);

  cache.set(url, root);
  return root.clone(true);
}
