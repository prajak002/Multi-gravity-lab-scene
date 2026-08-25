/**
 * build_all.mjs — build every scene and print one table of contact quality.
 *
 * The per-scene output is useful while iterating on one site; this is the view
 * that matters when changing the solver, because a fix that helps Gale and
 * quietly breaks Ganges is not a fix.
 */
import { buildScene } from './build_scene.mjs';
import { SCENES } from './scenes.mjs';

const ids = process.argv.length > 2 ? process.argv.slice(2) : Object.keys(SCENES);
const rows = [];
for (const id of ids) {
  try {
    const { out, report } = buildScene(id);
    for (const r of report) {
      const c = out.clips[r.key];
      rows.push(out.micro
        ? { id, key: r.key, label: c.label, micro: true,
            ghostBefore: r.before.ghostForce, ghostAfter: r.after.ghostForce }
        : { id, key: r.key, label: c.label,
            pos: r.diag.maxPosErrContact, rot: r.diag.maxRotErrContact,
            pen: r.after.penetration, gap: r.after.meanCornerGap,
            slip: r.after.schedSlipFrac, swing: r.after.swingNet,
            steps: r.steps, unreach: r.diag.clamped, reseed: r.diag.reseeded });
    }
    process.stderr.write(`built ${id}\n`);
  } catch (e) {
    process.stderr.write(`FAILED ${id}: ${e.message}\n`);
    rows.push({ id, key: '-', label: 'BUILD FAILED: ' + e.message });
  }
}

console.log('\n' + 'scene'.padEnd(26) + ' ' + 'model'.padEnd(12)
  + '  loadedPos  soleTilt   penet   gap   slip%  swing steps unre rsd');
for (const r of rows) {
  if (r.micro) {
    console.log(`${r.id.padEnd(26)} ${r.label.padEnd(12)}  `
      + `ghost force ${r.ghostBefore.toFixed(1)} N -> ${r.ghostAfter.toFixed(3)} N`);
  } else if (r.pos === undefined) {
    console.log(`${r.id.padEnd(26)} ${r.label}`);
  } else {
    console.log(`${r.id.padEnd(26)} ${r.label.padEnd(12)}  `
      + `${(r.pos * 1000).toFixed(2).padStart(7)}mm `
      + `${(r.rot * 1000).toFixed(0).padStart(6)}mr `
      + `${(r.pen * 1000).toFixed(1).padStart(6)}mm `
      + `${(r.gap * 1000).toFixed(0).padStart(4)}mm `
      + `${(r.slip * 100).toFixed(1).padStart(6)} `
      + `${r.swing.toFixed(2).padStart(6)} `
      + `${String(r.steps).padStart(5)} ${String(r.unreach).padStart(4)} ${String(r.reseed).padStart(3)}`);
  }
}
