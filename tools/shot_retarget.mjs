import { chromium } from 'playwright';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
const errors = [];
page.on('console', m => { if (m.type()==='error') errors.push(m.text().slice(0,300)); });
page.on('pageerror', e => errors.push(e.message.slice(0,300)));
await page.goto('http://localhost:5173/?motion=retarget', { waitUntil: 'load' });
await page.waitForTimeout(2000);
await page.evaluate(() => {
  document.querySelector('.card[data-kind="robot"][data-id="g1"]')?.click();
  document.querySelector('.card[data-kind="env"][data-id="moon"]')?.click();
  document.querySelector('.enter')?.click();
});
await page.waitForTimeout(22000);
await page.screenshot({ path: process.argv[2] });
await browser.close();
console.log(errors.length ? 'ERRORS:\n' + errors.slice(0,8).join('\n') : 'no console errors');
