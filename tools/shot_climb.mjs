import { chromium } from 'playwright';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
await page.goto('http://localhost:5173/', { waitUntil: 'load' });
await page.waitForTimeout(1500);
await page.evaluate(() => {
  document.querySelector('.card[data-kind="robot"][data-id="g1"]')?.click();
  document.querySelector('.card[data-kind="env"][data-id="moon"]')?.click();
  document.querySelector('.card[data-kind="motion"][data-id="climb"]')?.click();
  document.querySelector('.enter')?.click();
});
await page.waitForTimeout(11000);
await page.keyboard.press('4');          // profile: across the line of travel
await page.waitForTimeout(4000);
await page.screenshot({ path: process.argv[2] });
await browser.close();
console.log('ok');
