import { chromium } from 'playwright';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 900, height: 600 } });
await page.goto('http://localhost:5173/', { waitUntil: 'load' });
await page.waitForTimeout(1500);
await page.evaluate(() => {
  document.querySelector('.card[data-kind="robot"][data-id="g1"]')?.click();
  document.querySelector('.card[data-kind="env"][data-id="mars"]')?.click();
  document.querySelector('.card[data-kind="motion"][data-id="climb"]')?.click();
  document.querySelector('.enter')?.click();
});
await page.waitForTimeout(7000);
const samples = [];
for (let i = 0; i < 8; i++) {
  samples.push(await page.evaluate(() => {
    const r = window.__arena.robot.root.position;
    return { y: +r.y.toFixed(2), x: +r.x.toFixed(1), z: +r.z.toFixed(1) };
  }));
  await page.waitForTimeout(2000);
}
// Cumulative ascent, not first-vs-last: the climb deliberately resets to a
// fresh hill once it summits, and an endpoint difference scores that as a
// failure when it is the intended behaviour.
let ascent = 0, resets = 0;
for (let i = 1; i < samples.length; i++) {
  const d = samples[i].y - samples[i - 1].y;
  if (d > 0) ascent += d;
  else if (d < -2.5) resets++;          // a drop that large is a re-target
}
console.log('y over 16s:', samples.map(s => s.y).join(' -> '));
console.log('cumulative ascent:', ascent.toFixed(2), 'm  | re-targets:', resets);
await browser.close();
