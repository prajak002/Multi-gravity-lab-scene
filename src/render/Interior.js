/**
 * The space-station interior.
 *
 * On the surface scenes the ground gives an audience its sense of scale and
 * motion — a shadow, a horizon, footprints. In free fall there is none of
 * that, and a robot swimming against a starfield has nothing to be moving
 * RELATIVE to. The module walls are what restore that: they are the only
 * reason an ISS run reads as travel rather than as a figure waving in place.
 *
 * The asset is Corridor.blend, exported to glTF by tools/build_interior.sh.
 * It is a real 43.8 m corridor — 7.2 m across, 4.0 m floor to ceiling — and it
 * is modelled in metres, so it is NOT rescaled here. Fitting it to an
 * arbitrary target length, which is what this module used to do to the old
 * station asset, would have thrown away the one thing that makes an interior
 * useful: that a 1.32 m robot is 1.32 m against a wall of known size.
 */
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { Box3, Vector3, MeshStandardMaterial, Color, Group, PointLight,
         DoubleSide, HemisphereLight } from 'three';

// The old station asset ships with EXT_meshopt_compression, so the decoder has
// to be registered before the first load or GLTFLoader refuses the file
// outright. The corridor export is uncompressed and does not care.
const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
const cache = new Map();

/**
 * The corridor's own lighting, measured from the blend.
 *
 * The source file lights it with 40 area lamps in two rows just under the
 * ceiling, one row down each wall, at 27–79 W and a cold 0.51/0.92/1.00 white.
 * Forty real-time lights is not a thing to hand a browser, so the rows are
 * rebuilt at a fraction of the count — the layout is regular enough that a
 * decimated row reads the same, and what the lights are actually for is the
 * receding perspective down the tube, which survives fewer of them.
 */
const LAMP = {
  colour: new Color(0.5125, 0.915, 1.0),
  wallZ: 3.19,                 // metres either side of the axis
  ceilingDrop: 0.77,           // below the ceiling
  spacing: 4.2,                // metres between lamps in one row
  intensity: 24.0,
  distance: 13.0,
};

/**
 * A dim bounce term for the module.
 *
 * The ISS environment sets ambientIntensity to 0.05 because outside a module
 * there is nothing to bounce off — which is right for a robot against the
 * stars and wrong the moment it is in a white-walled corridor with forty
 * lamps in it. Without this the interior renders as black panels with lit
 * edges: technically what the point lights give you, and nothing like a
 * lit room.
 */
const BOUNCE = { sky: new Color(0.62, 0.72, 0.82), ground: new Color(0.30, 0.32, 0.36), intensity: 0.45 };

/**
 * @param {string|{url:string, metric?:boolean, targetLength?:number, lamps?:boolean}} spec
 * @returns {Promise<Group>} with `userData.bounds` (a Box3, post-placement) and
 *          `userData.length` — how far a robot can travel
 *          inside before it must wrap, in metres.
 */
export async function loadInterior(spec) {
  const opts = typeof spec === 'string' ? { url: spec } : spec;
  const key = JSON.stringify(opts);
  if (cache.has(key)) return cloneWithData(cache.get(key));

  const gltf = await loader.loadAsync(opts.url);
  const shell = gltf.scene;
  shell.name = 'interior-shell';

  // The 4k textures the corridor materials reference are not shipped with the
  // blend, so nothing is thrown away by re-materialising: give the module a
  // plainly engineered surface lit by the same IBL as everything else.
  shell.traverse((o) => {
    if (!o.isMesh) return;
    o.material = new MeshStandardMaterial({
      color: new Color(0xbfc4cc), metalness: 0.25, roughness: 0.62,
      envMapIntensity: 1.0,
      // DoubleSide, because the camera is INSIDE this mesh. The corridor is
      // modelled as a solid shell with its normals facing out, so with the
      // default FrontSide every surface between the camera and the outside —
      // which indoors is the ceiling and the far wall — is backface-culled and
      // the starfield shows straight through the module.
      side: DoubleSide,
    });
    o.castShadow = false;      // there is no sun in here to cast from
    o.receiveShadow = false;
  });

  const box = new Box3().setFromObject(shell);
  const size = new Vector3(); box.getSize(size);
  const centre = new Vector3(); box.getCenter(centre);

  // Fit-to-length is kept for the old station asset, which is not modelled at
  // any particular scale. A metric asset is left alone.
  let k = 1;
  if (!opts.metric) {
    const longest = Math.max(size.x, size.y, size.z) || 1;
    k = (opts.targetLength || 14) / longest;
  }
  shell.scale.setScalar(k);
  // Centre the module on the origin in all three axes. In free fall the robot
  // has no floor to stand on, so the interesting place for it is the middle of
  // the tube, and the middle of the tube is where the origin should be.
  shell.position.copy(centre).multiplyScalar(-k);

  const root = new Group();
  root.name = 'interior';
  root.add(shell);

  if (opts.lamps !== false) addLamps(root, size, k);

  root.userData.bounds = new Box3(
    size.clone().multiplyScalar(-k / 2), size.clone().multiplyScalar(k / 2));
  // Which way the tube runs, and how long it is. main.js flies the robot down
  // this axis rather than down a heading derived from a sun that is not in here.
  root.userData.axis = size.x >= size.z ? 'x' : 'z';
  root.userData.length = Math.max(size.x, size.z) * k;
  root.userData.width = Math.min(size.x, size.z) * k;
  root.userData.height = size.y * k;

  cache.set(key, root);
  return cloneWithData(root);
}

/** Two rows of lamps down the walls, as the blend has them. */
function addLamps(root, size, k) {
  const halfLen = (Math.max(size.x, size.z) * k) / 2;
  const y = (size.y * k) / 2 - LAMP.ceilingDrop;
  const lamps = new Group();
  lamps.name = 'interior-lamps';
  for (let x = -halfLen + LAMP.spacing * 0.5; x < halfLen; x += LAMP.spacing) {
    for (const z of [LAMP.wallZ, -LAMP.wallZ]) {
      const l = new PointLight(LAMP.colour, LAMP.intensity, LAMP.distance, 2);
      l.position.set(x, y, z);
      l.castShadow = false;
      lamps.add(l);
    }
  }
  root.add(lamps);
  const bounce = new HemisphereLight(BOUNCE.sky, BOUNCE.ground, BOUNCE.intensity);
  bounce.name = 'interior-bounce';
  root.add(bounce);
  return lamps;
}

/** clone(true) drops userData on the clone's root; carry it across by hand. */
function cloneWithData(root) {
  const c = root.clone(true);
  c.userData = { ...root.userData };
  return c;
}
