/**
 * check_gravity.mjs — assert the three robots actually diverge.
 *
 * A screenshot proves the page rendered, not that the physics is doing
 * anything. The claim is that identical joint templates started in phase pull
 * apart because g differs, so the check is on the DISTANCES: after ten
 * seconds the Moon lane must be measurably ahead of Mars, and Mars of Earth.
 */
import { chromium } from 'playwright';
const base = process.argv[2] || 'http://localhost:5173';
const browser = await chromium.launch({ args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 800 } });
const errs = [];
page.on('pageerror', (e) => errs.push('PAGEERROR ' + e.message.slice(0, 160)));
page.on('console', (m) => { if (m.type() === 'error') errs.push(m.text().slice(0, 160)); });
page.on('response', (r) => { if (r.status() >= 400) errs.push(`HTTP ${r.status()} ${r.url().slice(-46)}`); });

await page.goto(`${base}/gravity.html`, { waitUntil: 'load' });
await page.waitForFunction(() => document.getElementById('boot')?.classList.contains('gone'), { timeout: 60000 });
await page.waitForTimeout(11000);

const lanes = await page.evaluate(() => (window.__gravity?.lanes || []).map((l) => ({
  label: l.label, g: l.g, travelled: +l.travelled.toFixed(2),
  T: +l.gait.stepPeriod(l.g).toFixed(3), duty: +(l.pose?.duty ?? 0).toFixed(2),
})));
console.log('lane      g      step T   duty   distance after ~10 s');
for (const l of lanes) {
  console.log(`${l.label.padEnd(8)} ${String(l.g).padStart(6)}  ${String(l.T).padStart(7)}s ${String(l.duty).padStart(6)}  ${String(l.travelled).padStart(7)} m`);
}
const by = Object.fromEntries(lanes.map((l) => [l.label, l]));
const fail = [];
if (lanes.length !== 3) fail.push(`expected 3 lanes, got ${lanes.length}`);
if (by.MOON && by.EARTH && !(by.MOON.T > by.EARTH.T * 2)) fail.push('Moon step period is not >2x Earth');
// Earth leads. Walking speed goes as sqrt(g) at constant Froude number, so
// the HEAVIEST field covers the most ground — which is the Apollo result, and
// the reason the crews loped instead of walking.
if (by.EARTH && by.MARS && !(by.EARTH.travelled > by.MARS.travelled)) fail.push('Earth did not out-travel Mars');
if (by.MARS && by.MOON && !(by.MARS.travelled > by.MOON.travelled)) fail.push('Mars did not out-travel Moon');
if (by.MOON && by.EARTH) {
  const sep = by.EARTH.travelled - by.MOON.travelled;
  console.log(`\nEarth leads Moon by ${sep.toFixed(2)} m  (step period ratio x${(by.MOON.T / by.EARTH.T).toFixed(2)}, sqrt(9.807/1.625)=${Math.sqrt(9.807 / 1.625).toFixed(3)})`);
  if (sep < 1) fail.push(`separation only ${sep.toFixed(2)} m — the lanes are not diverging`);
}
// ---------------------------------------------------------------------------
// The mode buttons, clicked with the MOUSE.
//
// This is not the same test as calling button.click() from a page script, and
// the difference is the whole reason it is here. A scripted .click() dispatches
// straight at the element and skips hit-testing entirely, so it passes happily
// while a full-screen transparent overlay sits on top eating every real click.
// That is exactly what shipped: `#ui > *` sets pointer-events:auto and beats a
// bare `.gc-tags` rule on specificity, so the world-space label layer — inset:0,
// the size of the window — swallowed the pointer and the page was stuck on Walk
// with no way to reach Run, restart or pause.
//
// Assert on the READOUT text too, not just internal state: the panels are what
// the page is for, and they have been frozen while the gait switched correctly
// underneath.
// ---------------------------------------------------------------------------
const dutyOf = () => page.evaluate(() => [...document.querySelectorAll('.gc-lane')].map((el) => {
  const rows = [...el.querySelectorAll('.gc-rows > *')].map((n) => n.textContent.trim());
  const i = rows.indexOf('duty factor');
  return i >= 0 ? rows[i + 1] : '?';
}));

const overlay = await page.evaluate(() => {
  const W = innerWidth, H = innerHeight;
  return [...document.querySelectorAll('#ui *')].filter((e) => {
    const r = e.getBoundingClientRect(), cs = getComputedStyle(e);
    return r.width > W * 0.9 && r.height > H * 0.9 && cs.pointerEvents === 'auto'
        && cs.display !== 'none' && cs.visibility !== 'hidden' && +cs.opacity < 0.02;
  }).map((e) => e.className || e.tagName);
});
if (overlay.length) fail.push(`invisible full-window click-eater over the UI: ${overlay.join(', ')}`);

console.log('\nmode buttons, clicked with the mouse:');
const seen = {};
for (const mode of ['run', 'walk']) {
  const el = await page.$(`[data-motion="${mode}"]`);
  if (!el) { fail.push(`no ${mode} button`); continue; }
  const box = await el.boundingBox();
  if (!box) { fail.push(`${mode} button has no box`); continue; }
  await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
  await page.waitForTimeout(2200);
  const got = await page.evaluate(() => ({
    motion: window.__gravity.motion,
    gaits: [...new Set(window.__gravity.lanes.map((l) => l.gait.mode))],
    on: [...document.querySelectorAll('[data-motion]')].filter((b) => b.classList.contains('on')).map((b) => b.dataset.motion),
  }));
  const panel = await dutyOf();
  seen[mode] = panel.join('/');
  console.log(`  ${mode.padEnd(6)} gait ${got.gaits.join(',').padEnd(6)} highlighted ${String(got.on).padEnd(6)} panel duty ${panel.join(' ')}`);
  if (got.motion !== mode) fail.push(`clicking ${mode} left the app on ${got.motion} — the click never reached the button`);
  if (got.gaits.length !== 1 || got.gaits[0] !== mode) fail.push(`${mode}: lanes are running ${got.gaits.join(',')}`);
  if (!got.on.includes(mode)) fail.push(`${mode} button did not light up`);
}
// Each mode must produce DIFFERENT numbers, or the readout is frozen again.
const distinct = new Set(Object.values(seen));
if (Object.keys(seen).length === 2 && distinct.size !== 2) {
  fail.push(`the readout shows the same duty factors for different modes (${[...distinct].join(' | ')}) — panel is frozen`);
}

await page.screenshot({ path: process.argv[3] || '/tmp/gravity.png' });
await browser.close();
console.log(errs.length ? '\nERRORS:\n  ' + [...new Set(errs)].slice(0, 6).join('\n  ') : '\nno console errors, no failed requests');
if (fail.length) { console.log('FAILED:\n  ' + fail.join('\n  ')); process.exit(1); }
console.log('OK — the three lanes diverge as gravity predicts');
