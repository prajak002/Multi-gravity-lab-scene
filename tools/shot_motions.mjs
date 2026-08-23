import { chromium } from 'playwright';
import { mkdirSync } from 'node:fs';
const out = process.argv[2]; mkdirSync(out, { recursive: true });
const CASES = [
  { name: 'moon-run',   robot: 'g1',  env: 'moon',  motion: 'run' },
  { name: 'mars-climb', robot: 'g1',  env: 'mars',  motion: 'climb' },
  { name: 'iss-swim',   robot: 'g1',  env: 'iss',   motion: 'swim' },
  { name: 'moon-go2-run', robot: 'go2', env: 'moon', motion: 'run' },
];
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
const errors = [];
page.on('console', m => { if (m.type()==='error') errors.push(m.text().slice(0,240)); });
page.on('pageerror', e => errors.push(e.message.slice(0,240)));
await page.goto('http://localhost:5173/', { waitUntil: 'load' });
await page.waitForTimeout(2000);
for (const c of CASES) {
  await page.evaluate(({ robot, env, motion }) => {
    document.querySelector('.lobby-tab')?.click();
    const pick = (k, id) => document.querySelector(`.card[data-kind="${k}"][data-id="${id}"]`)?.click();
    pick('robot', robot); pick('env', env); pick('motion', motion);
    document.querySelector('.enter')?.click();
  }, c);
  await page.waitForTimeout(8500);
  await page.screenshot({ path: `${out}/${c.name}.png` });
  console.log(c.name.padEnd(16), 'ok');
}
await browser.close();
console.log(errors.length ? 'ERRORS:\n' + errors.slice(0,6).join('\n') : 'no console errors');
