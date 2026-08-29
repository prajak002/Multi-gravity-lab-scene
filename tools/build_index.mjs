/**
 * build_index.mjs — write the one manifest the viewer reads to know what exists.
 *
 * The arena used to discover its scenes by fetching all fourteen candidate
 * files and keeping the ones that parsed, which worked but does not scale to
 * forty places across three bodies and does not carry what a tabbed picker
 * needs: which body a place is on, what it is called, and whether it has an A/B
 * packet comparison, generated motions, or both.
 *
 * So the build writes it down. Nothing here is inferred at runtime, and a place
 * whose terrain has not been fetched simply does not appear.
 *
 *   node tools/build_index.mjs
 */
import fs from 'fs';
import { SITES, BODIES } from './sites.mjs';

const exists = (p) => { try { return fs.statSync(p).isFile(); } catch { return false; } };

const places = [];
for (const [id, s] of Object.entries(SITES)) {
  const hasDem = !s.dem || exists(`public/dem/${s.dem}.json`);
  const scene = exists(`public/scenes/${id}.json`);
  // A surface place needs its terrain AND the A/B pair that plays on it; a
  // module needs nothing but its dimensions, which are in the catalogue.
  if (s.dem && !hasDem) continue;
  if (!s.micro && !scene) continue;

  const entry = {
    id, body: s.body, place: s.place, name: s.name, g: s.g,
    blurb: s.blurb, micro: !!s.micro,
    // Every surface place carries the same thing: WorldVLA against
    // PragyaSpace, on that place's own terrain.
    compare: scene,
  };
  if (s.module) entry.module = s.module;
  if (s.dem) {
    const meta = JSON.parse(fs.readFileSync(`public/dem/${s.dem}.json`, 'utf8'));
    // Enough for the picker to say how good the terrain is without fetching
    // every sidecar up front.
    entry.terrain = {
      dem: s.dem, native_mpp: meta.native_mpp,
      posts: meta.dem_samples_across,
      span_x_m: meta.span_x_m ?? meta.span_m,
      span_y_m: meta.span_y_m ?? meta.span_m,
    };
  }
  places.push(entry);
}

// The ISS scenarios are scenes rather than places, and they play in whichever
// module is selected — so they are listed once, not once per module.
const microScenes = ['iss_momentum_gap', 'iss_brake_gap']
  .filter((id) => exists(`public/scenes/${id}.json`))
  .map((id) => {
    const j = JSON.parse(fs.readFileSync(`public/scenes/${id}.json`, 'utf8'));
    return { id, name: j.name, blurb: j.blurb };
  });

const index = { bodies: BODIES, places, microScenes, built: new Date().toISOString() };
fs.mkdirSync('public', { recursive: true });
fs.writeFileSync('public/places.json', JSON.stringify(index));

for (const b of BODIES) {
  const inBody = places.filter((p) => p.body === b);
  const withCompare = inBody.filter((p) => p.compare).length;
  console.log(`${b.padEnd(5)} ${String(inBody.length).padStart(2)} places  `
    + `${String(withCompare).padStart(2)} with the A/B pair`);
}
console.log(`microgravity scenarios: ${microScenes.map((m) => m.id).join(', ') || 'none'}`);
console.log('wrote public/places.json');
