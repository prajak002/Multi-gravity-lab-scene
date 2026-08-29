/**
 * SiteField — the ONE height function for a landing site.
 *
 * This module is imported by BOTH the browser renderer and the Node retargeting
 * tool, and that is the whole point. The retargeter plants each footfall by
 * asking this function where the ground is; the renderer builds its mesh from
 * the same function. If the two ever diverged the robot would visibly float or
 * sink, so there is exactly one implementation and no per-side approximation.
 *
 * Two layers, because they come from genuinely different places:
 *
 *   MACRO — a real NASA/USGS DEM window (see tools/terrain/dem.py). This is the
 *   actual place: its slope, its rim, its relief. Best available is 1 m/px on
 *   Mars (HiRISE) and 2 m/px on the Moon (LROC NAC).
 *
 *   MICRO — synthesised regolith and rock at the scale a foot actually touches.
 *   A 5 m traverse spans 3–5 DEM samples, so the centimetre-scale geometry the
 *   contact solver needs cannot come from orbit and is generated here instead,
 *   with per-site statistics (rock abundance, ripple wavelength) rather than
 *   one generic noise field. It is synthetic and is labelled as such in the UI.
 *
 * Frame: three.js Y-up. +x east, +z SOUTH, y up, metres, origin at patch centre.
 */

// ---------------------------------------------------------------------------
// Deterministic hash noise. No Math.random anywhere: the renderer and the
// retargeter must agree bit-for-bit, and a scene must look the same on reload.
// ---------------------------------------------------------------------------
function hash2(ix, iy, seed) {
  let h = Math.imul(ix | 0, 0x27d4eb2d) ^ Math.imul(iy | 0, 0x165667b1) ^ Math.imul(seed | 0, 0x9e3779b1);
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b);
  h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

/** Quintic fade — C2 continuous, so IK never sees a kink in the ground. */
const fade = (t) => t * t * t * (t * (t * 6 - 15) + 10);

function valueNoise(x, y, seed) {
  const ix = Math.floor(x), iy = Math.floor(y);
  const fx = fade(x - ix), fy = fade(y - iy);
  const a = hash2(ix, iy, seed), b = hash2(ix + 1, iy, seed);
  const c = hash2(ix, iy + 1, seed), d = hash2(ix + 1, iy + 1, seed);
  return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}

/** Fractal Brownian motion, returned in [-1, 1]. */
function fbm(x, y, seed, octaves, lacunarity = 2.03, gain = 0.5) {
  let sum = 0, amp = 1, norm = 0, fx = x, fy = y;
  for (let o = 0; o < octaves; o++) {
    sum += amp * (valueNoise(fx, fy, seed + o * 131) * 2 - 1);
    norm += amp;
    amp *= gain;
    fx *= lacunarity; fy *= lacunarity;
  }
  return sum / norm;
}

/**
 * Scattered rocks as smooth domes on a jittered lattice.
 *
 * A dome rather than a sphere so the surface stays single-valued and C1: the
 * contact solver needs a height function it can differentiate, and an
 * overhanging boulder has no height at all. Rocks are what the foot rolls off
 * on a slope, so this term is what actually produces ankle work.
 */
function rockField(x, z, p) {
  if (p.rockDensity <= 0) return 0;
  const cell = p.rockSpacing;
  const cx = Math.floor(x / cell), cz = Math.floor(z / cell);
  let h = 0;
  for (let j = -1; j <= 1; j++) {
    for (let i = -1; i <= 1; i++) {
      const gx = cx + i, gz = cz + j;
      const present = hash2(gx, gz, p.seed + 7717);
      if (present > p.rockDensity) continue;
      // jittered centre, and a radius drawn from the site's size distribution
      const ox = hash2(gx, gz, p.seed + 13), oz = hash2(gx, gz, p.seed + 29);
      const rs = hash2(gx, gz, p.seed + 53);
      const rx = (gx + ox) * cell, rz = (gz + oz) * cell;
      // power law: many small rocks, few large ones
      const radius = p.rockMin + (p.rockMax - p.rockMin) * Math.pow(rs, 2.6);
      const d = Math.hypot(x - rx, z - rz);
      if (d >= radius) continue;
      const t = 1 - d / radius;
      // smootherstep dome, height a fraction of radius (rocks are partly buried)
      h += radius * p.rockAspect * (t * t * (3 - 2 * t));
    }
  }
  return h;
}

/**
 * Per-site surface statistics. Amplitudes in metres, wavelengths in metres.
 *
 * These are not one noise field with the knobs moved. Rock abundance, rock
 * size distribution, ripple wavelength and the presence of a meso band differ
 * because the places differ: a lunar mare is a fine regolith blanket with a
 * few percent rock cover, a crater rim is blocky ejecta, a north-polar Martian
 * dune sea is sand with essentially no rock at all, and a yardang field is
 * wind-cut ridges running one way.
 */
export const SURFACE_PROFILES = {
  // ---- Mars -------------------------------------------------------------
  // Gale: Murray-formation bedrock and float rock. The DEM here is 1 m/px and
  // spans 128 source posts, so it carries its own landform — no meso term.
  mars_rocky: {
    seed: 1301,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 0.0, mesoLen: 20,
    coarse: 0.075, coarseLen: 3.2,      // metre-scale undulation between DEM posts
    fine: 0.016, fineLen: 0.42,         // gravel bed the sole actually rests on
    rockDensity: 0.30, rockSpacing: 0.85, rockMin: 0.045, rockMax: 0.26, rockAspect: 0.55,
  },
  // Jezero delta front: cobbles and boulders shed from the delta scarp.
  mars_delta: {
    seed: 1607,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 1.5, mesoLen: 26,
    coarse: 0.085, coarseLen: 3.6,
    fine: 0.014, fineLen: 0.38,
    rockDensity: 0.38, rockSpacing: 0.95, rockMin: 0.06, rockMax: 0.34, rockAspect: 0.6,
  },
  // Olympia Undae: deep aeolian sand, transverse ripples, essentially no rock.
  mars_sand: {
    seed: 2207,
    ripple: 0.055, rippleLen: 2.6, rippleDir: 0.35,
    meso: 2.2, mesoLen: 34,             // the dune train itself
    coarse: 0.045, coarseLen: 5.0,
    fine: 0.006, fineLen: 0.30,
    rockDensity: 0.0, rockSpacing: 1.0, rockMin: 0, rockMax: 0, rockAspect: 0,
  },
  // Cerberus Fossae: young basalt plain, blocky, cut by fissures.
  mars_fissure: {
    seed: 2711,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 1.1, mesoLen: 22,
    coarse: 0.070, coarseLen: 3.0,
    fine: 0.013, fineLen: 0.36,
    rockDensity: 0.34, rockSpacing: 0.80, rockMin: 0.05, rockMax: 0.30, rockAspect: 0.62,
  },
  // Medusae Fossae: friable ignimbrite cut into yardangs — ridges all aligned
  // with the prevailing wind, which is what makes the corridor a corridor.
  mars_yardang: {
    seed: 3119,
    ripple: 0.42, rippleLen: 11.0, rippleDir: 0.15,   // the yardangs themselves
    meso: 0.8, mesoLen: 30,
    coarse: 0.055, coarseLen: 4.0,
    fine: 0.010, fineLen: 0.34,
    rockDensity: 0.10, rockSpacing: 1.5, rockMin: 0.03, rockMax: 0.16, rockAspect: 0.5,
  },
  // Ganges Chasma wall: steep talus, loose debris, scattered blocks.
  mars_talus: {
    seed: 3517,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 1.8, mesoLen: 24,
    coarse: 0.095, coarseLen: 2.6,
    fine: 0.020, fineLen: 0.30,
    rockDensity: 0.36, rockSpacing: 0.78, rockMin: 0.05, rockMax: 0.28, rockAspect: 0.58,
  },

  // ---- Moon -------------------------------------------------------------
  // Lunar mare: fine regolith, low rock abundance, superposed small craters.
  moon_mare: {
    seed: 3301,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 0.9, mesoLen: 28,
    coarse: 0.055, coarseLen: 4.0,
    fine: 0.012, fineLen: 0.36,
    rockDensity: 0.12, rockSpacing: 1.4, rockMin: 0.03, rockMax: 0.18, rockAspect: 0.5,
  },
  // Lunar highlands / crater rim: blocky ejecta, high rock abundance. Used at
  // Shackleton, whose 5 m/px DEM already carries the rim, so the meso term is
  // small — it is filling a gap, not inventing a hill.
  moon_blocky: {
    seed: 4409,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 0.35, mesoLen: 18,
    coarse: 0.11, coarseLen: 2.8,
    fine: 0.018, fineLen: 0.40,
    rockDensity: 0.42, rockSpacing: 0.75, rockMin: 0.05, rockMax: 0.34, rockAspect: 0.6,
  },
  // Shiv Shakti: mare-highland transition regolith, uneven, moderate rock —
  // the scenario is precisely that the ground is uneven.
  moon_regolith: {
    seed: 5023,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 1.3, mesoLen: 24,
    coarse: 0.090, coarseLen: 3.0,
    fine: 0.015, fineLen: 0.34,
    rockDensity: 0.24, rockSpacing: 1.0, rockMin: 0.04, rockMax: 0.22, rockAspect: 0.55,
  },
  // Tycho flank: young ejecta on a steep grade, loose over blocky.
  moon_slope: {
    seed: 5711,
    ripple: 0.0, rippleLen: 1.0, rippleDir: 0,
    meso: 1.6, mesoLen: 21,
    coarse: 0.100, coarseLen: 2.7,
    fine: 0.019, fineLen: 0.32,
    rockDensity: 0.33, rockSpacing: 0.85, rockMin: 0.05, rockMax: 0.28, rockAspect: 0.58,
  },
};

export class SiteField {
  /**
   * @param {Float32Array} dem  size_px * size_px heights, metres, relative
   * @param {object} meta       sidecar JSON from tools/terrain/dem.py
   * @param {object} profile    entry from SURFACE_PROFILES
   */
  constructor(dem, meta, profile) {
    this.dem = dem;
    // Rectangular, long axis along the traverse — see pipeline/dem.py. The
    // square `size_px` is still read as a fallback, so a patch fetched before
    // the grid became 2:1 still loads.
    this.nx = meta.size_px_x ?? meta.size_px;
    this.nz = meta.size_px_y ?? meta.size_px;
    this.mpp = meta.mpp;
    this.meta = meta;
    this.p = profile;
    this.halfX = (this.nx * this.mpp) / 2;
    this.halfZ = (this.nz * this.mpp) / 2;
    // The largest square centred on the patch that is entirely inside it —
    // what a caller wanting one number for "how far the ground reaches" needs.
    this.half = Math.min(this.halfX, this.halfZ);
    // Detrend so the patch centre is the origin and the mean plane is level
    // enough that the scene camera and the sun rig behave predictably. The
    // real slope is kept — only the constant offset is removed.
    this.originY = this.sampleDEM(0, 0);
  }

  /** Bilinear DEM sample. x,z in metres from patch centre; +z south. */
  sampleDEM(x, z) {
    const nx = this.nx, nz = this.nz;
    // +z south, DEM rows run north→south, so row index grows with z.
    const u = (x + this.halfX) / this.mpp;
    const v = (z + this.halfZ) / this.mpp;
    const i0 = Math.floor(u), j0 = Math.floor(v);
    const fx = u - i0, fz = v - j0;
    const cx = (k) => Math.min(nx - 1, Math.max(0, k));
    const cz = (k) => Math.min(nz - 1, Math.max(0, k));
    const i1 = cx(i0 + 1), j1 = cz(j0 + 1), ii = cx(i0), jj = cz(j0);
    const d = this.dem;
    const a = d[jj * nx + ii], b = d[jj * nx + i1];
    const c = d[j1 * nx + ii], e = d[j1 * nx + i1];
    return (a + (b - a) * fx) + ((c + (e - c) * fx) - (a + (b - a) * fx)) * fz;
  }

  /** Synthetic centimetre-scale relief on top of the orbital DEM. */
  micro(x, z) {
    const p = this.p;
    let h = 0;
    // MESO — landform the orbital DEM cannot resolve.
    //
    // Only Gale (1 m/px) and Shackleton (5 m/px) span more than a hundred
    // source posts. Every other site spans ten to thirty-five, so between the
    // DEM's few-hundred-metre landform and the metre-scale `coarse` term there
    // is a whole band — the ten-to-forty-metre swells, hummocks and benches
    // that give ground its shape — which no available product carries. Left
    // out, a real slope renders as a smooth ramp. This fills that band, and is
    // synthetic; `dem_samples_across` in each patch's sidecar is what says how
    // much of a site is measured and how much is this.
    if (p.meso) h += p.meso * fbm(x / p.mesoLen, z / p.mesoLen, p.seed + 4441, 3);
    if (p.coarse) h += p.coarse * fbm(x / p.coarseLen, z / p.coarseLen, p.seed, 4);
    if (p.fine) h += p.fine * fbm(x / p.fineLen, z / p.fineLen, p.seed + 991, 3);
    if (p.ripple) {
      // Transverse dunes: a directed wave, smeared by noise so it is not a grating.
      const c = Math.cos(p.rippleDir), s = Math.sin(p.rippleDir);
      const t = (x * c + z * s) / p.rippleLen;
      const jitter = 0.35 * fbm(x / (p.rippleLen * 3), z / (p.rippleLen * 3), p.seed + 77, 2);
      h += p.ripple * Math.sin((t + jitter) * Math.PI * 2);
    }
    h += rockField(x, z, p);
    return h;
  }

  /** Ground height at (x, z), metres. This is the contract. */
  heightAt(x, z) {
    return this.sampleDEM(x, z) - this.originY + this.micro(x, z);
  }

  /** Surface normal by central difference of heightAt. */
  normalAt(x, z, eps = 0.04) {
    const hx = this.heightAt(x + eps, z) - this.heightAt(x - eps, z);
    const hz = this.heightAt(x, z + eps) - this.heightAt(x, z - eps);
    const nx = -hx / (2 * eps), nz = -hz / (2 * eps);
    const len = Math.hypot(nx, 1, nz);
    return [nx / len, 1 / len, nz / len];
  }

  /**
   * Best-fit plane under a foot-sized patch.
   *
   * A single normal at a point makes the sole chase every pebble. A real foot
   * is a 0.19 x 0.09 m rigid plate that bridges small features and tips on the
   * highest few, so contact is solved against the plane that supports the
   * whole sole plus the highest point inside it.
   */
  footPlane(x, z, yaw, len = 0.19, wid = 0.09) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    let sx = 0, sz = 0, sh = 0, sxx = 0, szz = 0, sxz = 0, sxh = 0, szh = 0, n = 0;
    let peak = -Infinity;
    const NL = 5, NW = 3;
    for (let a = 0; a < NL; a++) {
      for (let b = 0; b < NW; b++) {
        const u = (a / (NL - 1) - 0.5) * len;
        const v = (b / (NW - 1) - 0.5) * wid;
        // Foot-local (u forward, v LEFT) into world.
        //
        // With +z south and +y up, left of a heading phi is (sin phi, -cos phi)
        // in (x, z). Using (-sin, +cos) here points v to the RIGHT and silently
        // inverts every reported roll, which tips each foot the wrong way on a
        // side slope.
        const px = x + u * c + v * s;
        const pz = z + u * s - v * c;
        const h = this.heightAt(px, pz);
        if (h > peak) peak = h;
        sx += u; sz += v; sh += h;
        sxx += u * u; szz += v * v; sxz += u * v;
        sxh += u * h; szh += v * h; n++;
      }
    }
    // least-squares plane h = c0 + c1*u + c2*v  (u,v are centred, so sx=sz=0)
    const c0 = sh / n;
    const c1 = sxx > 1e-9 ? (sxh - sx * sh / n) / (sxx - sx * sx / n) : 0;
    const c2 = szz > 1e-9 ? (szh - sz * sh / n) / (szz - sz * sz / n) : 0;
    return {
      height: c0,          // mean support height under the sole
      peak,                // highest point the sole would touch
      pitch: -Math.atan(c1), // nose-up positive
      roll: Math.atan(c2),   // left-side-up positive
    };
  }
}
