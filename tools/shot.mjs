#!/usr/bin/env node
/** Boot the arena headless, drive it, screenshot, and fail loudly on any console error. */
import { chromium } from 'playwright';
import { mkdirSync } from 'node:fs';

const base = process.argv[2] || 'http://localhost:5173';
const outDir = process.argv[3] || 'shots';
mkdirSync(outDir, { recursive: true });

const SHOTS = [
  { name: 'lobby', wait: 2500 },
  { name: 'moon-g1', enter: { robot: 'g1', env: 'moon' }, wait: 7000 },
  { name: 'mars-h1', enter: { robot: 'h1', env: 'mars' }, wait: 7000 },
  { name: 'moon-go2', enter: { robot: 'go2', env: 'moon' }, wait: 7000 },
  { name: 'earth-g1', enter: { robot: 'g1', env: 'earth' }, wait: 6000 },
  { name: 'iss-g1', enter: { robot: 'g1', env: 'iss' }, wait: 6000 },
];

const browser = await chromium.launch({ args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 900 }, deviceScaleFactor: 1 });
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push('CONSOLE: ' + m.text().slice(0, 300)); });
page.on('pageerror', (e) => errors.push('PAGE: ' + e.message.slice(0, 300)));

await page.goto(base, { waitUntil: 'load' });

for (const s of SHOTS) {
  if (s.enter) {
    await page.evaluate(({ robot, env }) => {
      const pick = (kind, id) => document.querySelector(`.card[data-kind="${kind}"][data-id="${id}"]`)?.click();
      document.querySelector('.lobby-tab')?.click();
      pick('robot', robot); pick('env', env);
      document.querySelector('.enter')?.click();
    }, s.enter);
  }
  await page.waitForTimeout(s.wait);
  await page.screenshot({ path: `${outDir}/${s.name}.png` });
  console.log(`${s.name.padEnd(12)} ok`);
}

await browser.close();
if (errors.length) { console.error('\n' + errors.slice(0, 12).join('\n')); process.exit(1); }
console.log('no console errors');
