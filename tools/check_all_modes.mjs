/**
 * check_all_modes.mjs — drive the lobby through every robot x field x motion
 * and fail on anything the page reports.
 *
 * The point of this app is that ONE motion template runs in four gravity
 * fields, so the check that matters is not "does it load" but "does every
 * combination load", which is 3 x 4 x 3 and not something to click by hand.
 */
import { chromium } from 'playwright';
const base = process.argv[2] || 'http://localhost:4199';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errors = [];
page.on('pageerror', (e) => errors.push('PAGEERROR ' + e.message.slice(0, 160)));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text().slice(0, 160)); });
page.on('response', (r) => { if (r.status() >= 400) errors.push(`HTTP ${r.status()} ${r.url().slice(-50)}`); });

await page.goto(base, { waitUntil: 'load' });
await page.waitForTimeout(2500);

const combos = [];
for (const robot of ['g1', 'h1', 'go2'])
  for (const env of ['moon', 'mars', 'iss', 'earth'])
    for (const motion of ['walk', 'run', 'swim'])
      combos.push({ robot, env, motion });

let ok = 0, skipped = 0;
for (const c of combos) {
  const before = errors.length;
  const picked = await page.evaluate((c) => {
    const click = (kind, id) => {
      const el = document.querySelector(`.card[data-kind="${kind}"][data-id="${id}"]`);
      if (!el || el.classList.contains('disabled') || el.hasAttribute('disabled')) return false;
      el.click(); return true;
    };
    document.querySelector('#ui .lobby')?.classList.remove('hidden');
    if (!click('robot', c.robot)) return 'no-robot';
    if (!click('env', c.env)) return 'no-env';
    if (!click('motion', c.motion)) return 'motion-unavailable';
    document.querySelector('.enter')?.click();
    return 'ok';
  }, c);
  if (picked !== 'ok') { skipped++; continue; }
  // WAIT for the robot, do not sleep and hope.
  //
  // This was a flat 700 ms, which is a statement about localhost rather than
  // about the app. Run against the deployed site and the first few G1 combos
  // failed with "no robot mounted" — not because anything was broken, but
  // because the G1 ships 64 separate STL meshes and fetching them over the
  // network takes longer than the sleep. Once they were in the browser cache
  // every later combo passed, which is the signature of a timing assumption
  // and not of a fault.
  //
  // A check that only passes on localhost will lie about production, which is
  // the one place it actually matters.
  const state = await page.waitForFunction(() => {
    const r = window.__arena?.robot;
    return r ? { name: r.def.short, joints: r.jointNames.length } : null;
  }, { timeout: 45000 }).then((h) => h.jsonValue()).catch(() => null);
  if (!state) errors.push(`${c.robot}/${c.env}/${c.motion}: no robot mounted within 45 s`);
  else if (errors.length === before) ok++;
  await page.evaluate(() => { const l = document.querySelector('#ui .lobby'); if (l) l.classList.remove('hidden'); });
  await page.waitForTimeout(120);
}
console.log(`${ok} combinations ran clean, ${skipped} not offered by the lobby, ${combos.length} attempted`);
console.log(errors.length ? 'ERRORS:\n  ' + [...new Set(errors)].slice(0, 10).join('\n  ') : 'no console errors, no failed requests');
await browser.close();
process.exit(errors.length ? 1 : 0);
