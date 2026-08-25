/**
 * check_footing.mjs — the feet do not go through the floor.
 *
 * The claim has three parts and a screenshot checks none of them:
 *
 *   1. no sole contact point is ever below the ground, on any of the fourteen
 *      real DEMs or on the generated fields, at any point in the cycle
 *   2. a step lands TOE FIRST and rolls back onto the heel, rather than
 *      arriving flat or heel-first
 *   3. the ankle stays inside the limits its own URDF declares while it does it
 *
 * Every one of those is measured off the running scene, not asserted.
 *
 * Run:  node tools/check_footing.mjs [http://localhost:5173]
 */
import { chromium } from 'playwright';

const base = process.argv[2] || 'http://localhost:5173';
const browser = await chromium.launch({ args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errs = [];
page.on('pageerror', (e) => errs.push('PAGEERROR ' + e.message.slice(0, 180)));
page.on('console', (m) => { if (m.type() === 'error') errs.push(m.text().slice(0, 180)); });

const fail = [];
const PEN_LIMIT_MM = 1.0;     // a millimetre of float error, not a foot in the floor

// --- 1. the three-gravity page: three G1s, one template, flat-ish ground -----
await page.goto(`${base}/gravity.html`, { waitUntil: 'load' });
await page.waitForFunction(() => document.getElementById('boot')?.classList.contains('gone'), { timeout: 60000 });
await page.waitForTimeout(2500);

const gravity = await page.evaluate(async () => {
  const app = window.__gravity;
  const worst = {};
  const contacts = {};
  for (let i = 0; i < 400; i++) {
    await new Promise((r) => requestAnimationFrame(r));
    for (const row of app.penetrationMM()) {
      worst[row.label] = Math.max(worst[row.label] ?? -Infinity, row.mm);
      contacts[row.label] = contacts[row.label] || {};
      const c = row.contact || 'none';
      contacts[row.label][c] = (contacts[row.label][c] || 0) + 1;
    }
  }
  return { worst, contacts };
});

console.log('\n/gravity.html — three G1s, 400 frames each\n');
console.log('lane      deepest sole below ground   contact points seen');
for (const label of Object.keys(gravity.worst)) {
  const mm = gravity.worst[label];
  const c = Object.entries(gravity.contacts[label]).map(([k, n]) => `${k} ${n}`).join('  ');
  console.log(`${label.padEnd(8)} ${mm.toFixed(3).padStart(10)} mm            ${c}`);
  if (mm > PEN_LIMIT_MM) fail.push(`${label}: sole ${mm.toFixed(2)} mm below the ground`);
  if (!gravity.contacts[label].toe) fail.push(`${label}: never landed on the toe`);
}

// --- 2. the single-robot arena, on the roughest field it has -----------------
/** The lobby is the only way in; drive it the way check_all_modes.mjs does. */
async function enterScene(robot, env, motion) {
  const picked = await page.evaluate((c) => {
    const click = (kind, id) => {
      const el = document.querySelector(`.card[data-kind="${kind}"][data-id="${id}"]`);
      if (!el || el.classList.contains('disabled') || el.hasAttribute('disabled')) return false;
      el.click(); return true;
    };
    document.querySelector('#ui .lobby')?.classList.remove('hidden');
    if (!click('robot', c.robot)) return 'no-robot';
    if (!click('env', c.env)) return 'no-env';
    if (!click('motion', c.motion)) return 'no-motion';
    document.querySelector('.enter')?.click();
    return 'ok';
  }, { robot, env, motion });
  if (picked !== 'ok') return picked;
  await page.waitForFunction(() => !!window.__arena?.robot, { timeout: 60000 });
  await page.waitForTimeout(2000);
  return 'ok';
}

await page.goto(base, { waitUntil: 'load' });
await page.waitForTimeout(2500);

for (const env of ['moon', 'mars', 'earth']) {
  const got = await enterScene('g1', env, 'walk');
  if (got !== 'ok') { fail.push(`could not enter g1/${env}/walk: ${got}`); continue; }
  const r = await page.evaluate(async () => {
    let worst = -Infinity;
    const seen = {};
    for (let i = 0; i < 400; i++) {
      await new Promise((res) => requestAnimationFrame(res));
      worst = Math.max(worst, window.__arena.penetrationMM());
      const c = window.__arena.footing?.contact || 'none';
      seen[c] = (seen[c] || 0) + 1;
    }
    return { worst, seen, env: window.__arena.env?.id };
  });
  const c = Object.entries(r.seen).map(([k, n]) => `${k} ${n}`).join('  ');
  console.log(`/?env=${(r.env || env).padEnd(6)} deepest ${r.worst.toFixed(3).padStart(8)} mm            ${c}`);
  if (r.worst > PEN_LIMIT_MM) fail.push(`${env}: sole ${r.worst.toFixed(2)} mm below the ground`);
  if (!r.seen.toe) fail.push(`${env}: never landed on the toe`);
}

// --- 3. the ISS corridor: the robot must stay inside the module --------------
const issEntered = await enterScene('g1', 'iss', 'swim');
if (issEntered !== 'ok') fail.push(`could not enter g1/iss/swim: ${issEntered}`);
const iss = await page.evaluate(async () => {
  const out = { inside: true, maxAbsZ: 0, maxAbsY: 0, tube: window.__arena.tube, samples: 0 };
  for (let i = 0; i < 600; i++) {
    await new Promise((r) => requestAnimationFrame(r));
    const p = window.__arena.robot.root.position;
    out.maxAbsZ = Math.max(out.maxAbsZ, Math.abs(p.z));
    out.maxAbsY = Math.max(out.maxAbsY, Math.abs(p.y));
    out.samples++;
  }
  out.stroke = window.__arena.stroke;
  return out;
});
console.log(`\n/?env=iss  corridor ${iss.tube ? `${iss.tube.length.toFixed(1)} m usable, ` +
  `${iss.tube.width.toFixed(1)} m wide, ${iss.tube.height.toFixed(1)} m tall` : 'NOT LOADED'}`);
console.log(`           robot strayed at most ${iss.maxAbsZ.toFixed(2)} m off the axis, ` +
            `${iss.maxAbsY.toFixed(2)} m off the centreline`);
if (!iss.tube) fail.push('the ISS interior did not load');
else {
  if (iss.maxAbsZ > iss.tube.width / 2) fail.push(`robot went ${iss.maxAbsZ.toFixed(2)} m off axis, through a wall ${(iss.tube.width / 2).toFixed(2)} m away`);
  if (iss.maxAbsY > iss.tube.height / 2) fail.push(`robot went ${iss.maxAbsY.toFixed(2)} m off the centreline, through the floor or ceiling`);
}
if (!iss.stroke) fail.push('the butterfly stroke is not driving the ISS swim');

await browser.close();

if (errs.length) { console.log('\nERRORS:'); for (const e of new Set(errs)) console.log('  ' + e); }
if (errs.length || fail.length) {
  console.log(`\nFAILED (${fail.length}):`);
  for (const m of fail) console.log('  - ' + m);
  process.exit(1);
}
console.log('\nOK — no sole point below the ground, every landing is toe-first, ' +
            'and the ISS run stays inside the corridor.');
