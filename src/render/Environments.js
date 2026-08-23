/**
 * The gravity fields, as data.
 *
 * Every number here is sourced, because the whole point of the arena is that
 * the SAME motion behaves differently only because these values differ.
 */
import { Color, Vector3 } from 'three';

export const G_EARTH = 9.80665;   // standard gravity
export const G_MOON = 1.62;       // mean lunar surface gravity
export const G_MARS = 3.72076;    // mean martian surface gravity
export const G_ISS = 0.0;         // free fall; microgravity residual is ~1e-5 g

const deg = (d) => d * Math.PI / 180;

export function sunDir(elevDeg, azDeg) {
  const e = deg(elevDeg), a = deg(azDeg);
  return new Vector3(Math.cos(e) * Math.cos(a), Math.sin(e), Math.cos(e) * Math.sin(a)).normalize();
}

export const ENVIRONMENTS = [
  {
    id: 'moon', name: 'Moon — Mare', short: 'MOON',
    blurb: 'One sixth g, no air. Flight phases run long and dust flies on straight lines.',
    g: G_MOON, vacuum: true,
    // A mid-elevation sun, deliberately: at the polar 2.6 deg the subject is
    // pure silhouette, and an audience cannot judge a gait it cannot see.
    sunElev: 28, sunAz: 118, sunColor: new Color(1.0, 0.98, 0.95), sunIntensity: 3.2,
    skyColor: new Color(0.0, 0.0, 0.0), ambientIntensity: 0.04,
    groundAlbedo: new Color(0.132, 0.128, 0.124),   // Apollo soil samples, 0.11-0.14
    terrain: { amp: 3.4, scale: 0.021, craters: 34, roughness: 0.98 },
    fogDensity: 0.0, stars: 1.0,
    dust: { drag: 0.0, life: 7.5, size: 0.040, color: new Color(0.52, 0.50, 0.48) },
    exposure: 1.05,
  },
  {
    id: 'mars', name: 'Mars — Jezero', short: 'MARS',
    blurb: 'Three eighths g with a thin CO2 atmosphere — dust billows instead of arcing.',
    g: G_MARS, vacuum: false,
    sunElev: 41, sunAz: 62, sunColor: new Color(1.0, 0.87, 0.72), sunIntensity: 2.1,
    skyColor: new Color(0.52, 0.34, 0.22), ambientIntensity: 0.40,
    groundAlbedo: new Color(0.28, 0.16, 0.10),
    terrain: { amp: 4.2, scale: 0.017, craters: 12, roughness: 0.94 },
    fogDensity: 0.0016, stars: 0.0,
    dust: { drag: 1.1, life: 3.0, size: 0.065, color: new Color(0.56, 0.36, 0.24) },
    exposure: 1.0,
  },
  {
    id: 'iss', name: 'ISS — Free Fall', short: 'ISS',
    blurb: 'No weight to push against. Every motion is reaction only; the body rotates about its own centre of mass.',
    g: G_ISS, vacuum: true,
    sunElev: 22, sunAz: 200, sunColor: new Color(1.0, 0.99, 0.97), sunIntensity: 3.6,
    skyColor: new Color(0.0, 0.0, 0.0), ambientIntensity: 0.05,
    groundAlbedo: new Color(0.30, 0.30, 0.32),
    terrain: null,                                   // no ground at all
    interior: 'env/iss_station.glb',                 // module interior to float inside
    fogDensity: 0.0, stars: 1.0,
    dust: { drag: 0.0, life: 9.0, size: 0.03, color: new Color(0.6, 0.6, 0.62) },
    exposure: 0.95,
  },
  {
    id: 'earth', name: 'Earth — Reference', short: 'EARTH',
    blurb: 'The field every one of these controllers was trained in. The control condition.',
    g: G_EARTH, vacuum: false,
    sunElev: 34, sunAz: 42, sunColor: new Color(1.0, 0.965, 0.90), sunIntensity: 2.6,
    skyColor: new Color(0.36, 0.44, 0.58), ambientIntensity: 0.55,
    groundAlbedo: new Color(0.20, 0.19, 0.155),
    terrain: { amp: 1.6, scale: 0.014, craters: 0, roughness: 0.92 },
    fogDensity: 0.0011, stars: 0.0,
    dust: { drag: 3.2, life: 1.5, size: 0.075, color: new Color(0.42, 0.38, 0.31) },
    exposure: 1.0,
  },
];

export const byId = (id) => ENVIRONMENTS.find((e) => e.id === id) || ENVIRONMENTS[0];

/** Relative to Earth, for the readout. */
export const gRatio = (env) => env.g / G_EARTH;
