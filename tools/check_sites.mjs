/**
 * check_sites.mjs — the catalogue in tools/sites.mjs and the one in
 * pipeline/dem.py describe the same places, and this fails if they stop doing.
 *
 * They are deliberately two files. dem.py owns the OBSERVATION — coordinates,
 * product, the grade the scenario needs — and needs numpy and rasterio to do
 * anything at all. sites.mjs owns the SCENE — the label a person reads, the
 * body, its gravity, the surface profile — and is imported by both the build
 * tools and the browser. Neither can import the other.
 *
 * What they share is a set of ids, and an id that exists on one side and not
 * the other fails silently in the worst possible way: the place either never
 * appears in the picker, or appears and cannot load its terrain.
 *
 *   node tools/check_sites.mjs
 */
import fs from 'fs';
import { SITES } from './sites.mjs';
import { SURFACE_PROFILES } from '../src/terrain/SiteField.js';

const py = fs.readFileSync('pipeline/dem.py', 'utf8');
const table = py.slice(py.indexOf('SITES = {'));
const pyIds = new Set([...table.matchAll(/"(\w+)": dict\(/g)].map((m) => m[1]));

const jsSurface = Object.entries(SITES).filter(([, s]) => !s.micro).map(([id]) => id);
const problems = [];

for (const id of jsSurface) {
  if (!pyIds.has(id)) problems.push(`${id}: in sites.mjs but pipeline/dem.py cannot fetch it`);
}
for (const id of pyIds) {
  if (!SITES[id]) problems.push(`${id}: dem.py fetches it but sites.mjs has no scene for it`);
}
for (const [id, s] of Object.entries(SITES)) {
  if (s.micro) {
    if (!s.module?.length || !s.module?.diameter) problems.push(`${id}: module has no dimensions`);
    continue;
  }
  if (!SURFACE_PROFILES[s.profile]) {
    problems.push(`${id}: surface profile "${s.profile}" is not in SURFACE_PROFILES`);
  }
  if (!s.dem) problems.push(`${id}: no terrain patch named`);
}

// Which of them have actually been fetched. Not a failure — the catalogue is
// allowed to run ahead of the download — but worth saying out loud, because a
// place with no patch is silently dropped from the picker.
const missing = Object.entries(SITES)
  .filter(([, s]) => s.dem)
  .filter(([, s]) => !fs.existsSync(`public/dem/${s.dem}.f32`))
  .map(([id]) => id);

const bodies = {};
for (const s of Object.values(SITES)) bodies[s.body] = (bodies[s.body] || 0) + 1;
console.log(Object.entries(bodies).map(([b, n]) => `${b} ${n}`).join('   '));
console.log(`${Object.keys(SITES).length} places, ${pyIds.size} fetchable`);
if (missing.length) {
  console.log(`\nnot yet fetched (${missing.length}): ${missing.join(', ')}`);
  console.log('  .venv/bin/python pipeline/dem.py site ' + missing.join(' '));
}
if (problems.length) {
  console.log('\n' + problems.join('\n'));
  process.exit(1);
}
console.log('\nok — the two catalogues agree');
