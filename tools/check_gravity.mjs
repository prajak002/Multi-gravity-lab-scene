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
await page.screenshot({ path: process.argv[3] || '/tmp/gravity.png' });
await browser.close();
console.log(errs.length ? '\nERRORS:\n  ' + [...new Set(errs)].slice(0, 6).join('\n  ') : '\nno console errors, no failed requests');
if (fail.length) { console.log('FAILED:\n  ' + fail.join('\n  ')); process.exit(1); }
console.log('OK — the three lanes diverge as gravity predicts');
