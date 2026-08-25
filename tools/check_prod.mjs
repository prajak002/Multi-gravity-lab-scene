import { chromium } from 'playwright';
const base = process.argv[2] || 'http://localhost:4173';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errors = [], failed = [];
page.on('console', m => { if (m.type()==='error') errors.push(m.text().slice(0,200)); });
page.on('pageerror', e => errors.push(e.message.slice(0,200)));
page.on('requestfailed', r => failed.push(`${r.failure()?.errorText} ${r.url().slice(-60)}`));
page.on('response', r => { if (r.status() >= 400) failed.push(`HTTP ${r.status()} ${r.url().slice(-60)}`); });

const t0 = Date.now();
await page.goto(base, { waitUntil: 'load' });
await page.waitForTimeout(1500);
await page.evaluate(() => {
  document.querySelector('.card[data-kind="robot"][data-id="g1"]')?.click();
  document.querySelector('.card[data-kind="env"][data-id="moon"]')?.click();
  document.querySelector('.enter')?.click();
});
// wait for the splash to actually clear — the exact thing that hung on Vercel
let cleared = false;
for (let i = 0; i < 60; i++) {
  cleared = await page.evaluate(() => document.getElementById('boot').classList.contains('gone'));
  if (cleared) break;
  await page.waitForTimeout(1000);
}
const secs = ((Date.now() - t0) / 1000).toFixed(1);
const robot = await page.evaluate(() => {
  const r = window.__arena?.robot;
  return r ? { name: r.def.name, joints: r.jointNames.length, height: +r.height.toFixed(3) } : null;
});
await page.screenshot({ path: process.argv[3] || 'prod.png' });
await browser.close();
console.log(`splash cleared: ${cleared}  after ${secs}s`);
console.log('robot:', JSON.stringify(robot));
if (failed.length) console.log('failed requests:\n  ' + failed.slice(0,6).join('\n  '));
console.log(errors.length ? 'console errors:\n  ' + errors.slice(0,5).join('\n  ') : 'no console errors');
