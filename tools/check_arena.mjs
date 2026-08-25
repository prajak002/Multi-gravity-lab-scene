/**
 * check_arena.mjs — load the built A/B viewer in a real browser and fail on
 * anything the page reports.
 *
 * Run against a PREVIEW server, not the dev server. Vite's dev server answers
 * any unmatched path with index.html and a 200, so a missing scene file looks
 * like a success; the preview server serves dist/ as it will actually deploy.
 *
 *   npx vite preview --port 4199 &
 *   node tools/check_arena.mjs http://localhost:4199 shot.png
 */
import { chromium } from 'playwright';

const base = process.argv[2] || 'http://localhost:4199';
const shot = process.argv[3];

const browser = await chromium.launch({
  args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 810 } });
const errors = [], failed = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text().slice(0, 200)); });
page.on('pageerror', (e) => errors.push('PAGEERROR ' + e.message.slice(0, 200)));
page.on('requestfailed', (r) => failed.push(`${r.failure()?.errorText} ${r.url().slice(-70)}`));
page.on('response', (r) => { if (r.status() >= 400) failed.push(`HTTP ${r.status()} ${r.url().slice(-70)}`); });

await page.goto(`${base}/arena.html`, { waitUntil: 'load' });
await page.waitForTimeout(9000);

const info = await page.evaluate(() => ({
  canvases: document.querySelectorAll('canvas').length,
  sliders: document.querySelectorAll('input[type=range]').length,
  text: document.body.innerText.slice(0, 800),
}));

console.log(`canvases ${info.canvases}   range inputs ${info.sliders}`);
console.log('--- visible text ---\n' + info.text);
console.log('--- failed requests ---\n' + (failed.length ? failed.slice(0, 8).join('\n') : 'none'));
console.log('--- console errors ---\n' + (errors.length ? errors.slice(0, 8).join('\n') : 'none'));
if (shot) await page.screenshot({ path: shot });
await browser.close();
process.exit(errors.length || failed.length ? 1 : 0);
