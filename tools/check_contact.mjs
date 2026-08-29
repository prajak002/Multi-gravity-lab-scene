/**
 * check_contact.mjs — does any part of either robot go inside anything?
 *
 * The other checks in this directory measure the SOLVER: audit_contact.mjs
 * asks whether the retargeted footholds hold, check_footing.mjs asks whether
 * the live gait plants. Neither of them looks at what the arena actually draws,
 * and that is where the visible faults were:
 *
 *   - Side-by-side mode displaced each robot half the separation sideways and
 *     kept the pelvis height its clip had solved against the ground under the
 *     UNDISPLACED path. On terrain with relief that buried the machine: 270 mm
 *     on Medusae Fossae, 228 on Ganges, 201 on the Shackleton rim, knees and
 *     hips under the surface along with the feet. Every contact number in the
 *     scene files was nevertheless perfect, because the solver had done its job
 *     and the viewer then moved the robot.
 *
 *   - The ISS panels were placed at a fixed offset from the pelvis path, so
 *     both robots passed straight through both walls by about a third of a
 *     metre.
 *
 * So this measures the drawn thing against the drawn thing: every vertex of
 * every mesh of both robots, at 90 instants across each clip, against
 * Arena.ground() — the same height function the terrain mesh is built from —
 * and against the microgravity panels for the ISS scenes.
 *
 *   npx vite build && npx vite preview --port 4199 &
 *   node tools/check_contact.mjs http://localhost:4199
 *
 * Reports the worst offender per scene by LINK NAME, which is what tells a
 * whole-body sink (knees and hips listed beside the feet) apart from a foot
 * clipping a rock (only the ankle link, single-digit millimetres).
 */
import { chromium } from 'playwright';

const base = process.argv[2] || 'http://localhost:4199';
// A tolerance, not a target. The retargeter deliberately plants the URDF
// contact spheres 4 mm into the regolith, and the guard in Arena._seat lifts
// the visible shell back out of it; what is left is rounding and the finite
// sampling below.
const TOL_MM = 3;

const browser = await chromium.launch({
  args: ['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'],
});
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errors = [];
page.on('pageerror', (e) => errors.push(e.message.slice(0, 160)));

await page.goto(`${base}/arena.html`, { waitUntil: 'load' });
await page.waitForFunction(() => window.__arena?.ready, null, { timeout: 90000 });

const ids = await page.evaluate(() =>
  [...document.querySelectorAll('#scenes button')].map((b) => b.dataset.id));

let worstOverall = 0;
const failures = [];

for (const id of ids) {
  const r = await page.evaluate(async (id) => {
    const a = window.__arena;
    await a.load(id);
    a.playing = false;
    const V = a.aim.constructor;
    const v = new V();
    const dur = a.players.A?.duration ?? a.players.B?.duration ?? 10;
    const N = 90;

    const nameOf = (o) => { let p = o; while (p) { if (p.name) return p.name; p = p.parent; } return '?'; };

    // The microgravity panels, as planes of constant x with the module axis
    // running through them.
    const panels = [];
    if (a.scene_.micro) {
      a.microSet?.traverse((o) => {
        if (o.isMesh && Math.abs(Math.abs(o.rotation.y) - Math.PI / 2) < 0.01) panels.push(o.position.x);
      });
      panels.sort((p, q) => p - q);
    }

    const worst = {};
    let deepest = { d: 0, link: '', run: '', t: 0 };
    for (let s = 0; s < N; s++) {
      a.t = (s / N) * dur;
      a.step(1 / 60);
      for (const key of ['A', 'B']) {
        const rb = a.robots[key];
        if (!rb) continue;
        rb.root.updateMatrixWorld(true);
        rb.root.traverse((o) => {
          if (!o.isMesh || !o.geometry?.attributes?.position) return;
          const pos = o.geometry.attributes.position;
          const step = Math.max(1, Math.floor(pos.count / 80));
          for (let i = 0; i < pos.count; i += step) {
            v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
            let d = 0;
            if (a.scene_.micro) {
              // Outside either end panel is inside the bulkhead.
              if (panels.length === 2) {
                d = Math.max(panels[0] - v.x, v.x - panels[1], 0);
              }
            } else {
              d = a.ground(v.x, v.z) - v.y;
            }
            if (d > 0) {
              const n = nameOf(o);
              if (!worst[n] || d > worst[n]) worst[n] = d;
              if (d > deepest.d) deepest = { d, link: n, run: key, t: a.t };
            }
          }
        });
      }
    }
    return {
      micro: !!a.scene_.micro,
      deepest,
      links: Object.entries(worst).sort((x, y) => y[1] - x[1]).slice(0, 5)
        .map(([k, d]) => `${k} ${(d * 1000).toFixed(0)}mm`),
    };
  }, id);

  const mm = r.deepest.d * 1000;
  worstOverall = Math.max(worstOverall, mm);
  const tag = r.micro ? 'panels' : 'ground';
  const ok = mm <= TOL_MM;
  if (!ok) failures.push(`${id}: ${mm.toFixed(0)}mm into the ${tag} (${r.deepest.link}, run ${r.deepest.run}, t=${r.deepest.t.toFixed(2)}s)`);
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${id.padEnd(28)} ${mm.toFixed(0).padStart(4)}mm into ${tag}`
    + (r.links.length ? `   ${r.links.join(', ')}` : ''));
}

console.log(`\nworst penetration across all scenes: ${worstOverall.toFixed(1)} mm (tolerance ${TOL_MM} mm)`);
if (errors.length) console.log('console errors:\n' + errors.slice(0, 6).join('\n'));
if (failures.length) console.log('\n' + failures.join('\n'));
await browser.close();
process.exit(failures.length || errors.length ? 1 : 0);
