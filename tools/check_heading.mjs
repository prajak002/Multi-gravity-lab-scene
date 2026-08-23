import { chromium } from 'playwright';
const browser = await chromium.launch({ args: ['--use-angle=metal','--ignore-gpu-blocklist','--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 900, height: 600 } });
await page.goto('http://localhost:5173/', { waitUntil: 'load' });
await page.waitForTimeout(1500);
await page.evaluate(() => {
  document.querySelector('.card[data-kind="robot"][data-id="g1"]')?.click();
  document.querySelector('.card[data-kind="env"][data-id="moon"]')?.click();
  document.querySelector('.card[data-kind="motion"][data-id="walk"]')?.click();
  document.querySelector('.enter')?.click();
});
await page.waitForTimeout(8000);
const r = await page.evaluate(async () => {
  const a = window.__arena;
  const p0 = a.robot.root.position.clone();
  // the robot's own forward axis in world space
  const fwd = new (p0.constructor)(0, 0, 1).applyQuaternion(a.robot.root.quaternion);
  const fwdX = new (p0.constructor)(1, 0, 0).applyQuaternion(a.robot.root.quaternion);
  await new Promise(res => setTimeout(res, 2500));
  const p1 = a.robot.root.position.clone();
  const d = p1.sub(p0); d.y = 0;
  const norm = (v) => { const l = Math.hypot(v.x, v.z) || 1; return { x: v.x/l, z: v.z/l }; };
  const dn = norm(d), fz = norm(fwd), fx = norm(fwdX);
  return {
    travelled: Math.hypot(d.x, d.z).toFixed(2),
    dot_localZ: (dn.x*fz.x + dn.z*fz.z).toFixed(3),
    dot_localX: (dn.x*fx.x + dn.z*fx.z).toFixed(3),
  };
});
console.log(JSON.stringify(r));
await browser.close();
