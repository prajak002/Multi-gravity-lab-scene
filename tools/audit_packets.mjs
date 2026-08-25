/**
 * audit_packets.mjs — check what the fourteen motion packets actually contain.
 *
 * The packets are the input to everything downstream, and their READMEs are
 * explicit that they are "research-informed synthetic reference trajectories,
 * not physics rollouts". This tool measures the consequences of that, per clip:
 *
 *   csvVsNpz    do the CSV and the NPZ describe the same motion? The CSV is a
 *               re-export, so any disagreement means one of the two is stale.
 *
 *   fkResidual  does the packet's own body_pos_w agree with running the URDF's
 *               forward kinematics on its own joint_pos? If it does not, the
 *               link positions were authored independently of the joint angles
 *               and the "joints" are decoration.
 *
 *   footZ       how far the lowest foot sits above or below the plane the clip
 *               is nominally walking on. A clip whose feet never reach the
 *               ground cannot be planted on terrain by any amount of placement.
 *
 * Run: node tools/audit_packets.mjs [packetDir ...]
 */
import fs from 'fs';
import path from 'path';
import { loadNPZ, rows } from './npz.mjs';
import { bodyFK, quatToMat, mapv, CSV_JOINT_ORDER } from '../src/sim/G1Kinematics.js';

/** Every packet directory under packets/, extracted or zipped. */
export function findPackets(root = 'packets') {
  const out = [];
  for (const body of fs.readdirSync(root)) {
    const dir = path.join(root, body);
    if (!fs.statSync(dir).isDirectory()) continue;
    for (const entry of fs.readdirSync(dir)) {
      const p = path.join(dir, entry);
      if (fs.statSync(p).isDirectory() && fs.existsSync(path.join(p, 'NPZ'))) out.push(p);
    }
  }
  return out;
}

const readCSV = (f) => fs.readFileSync(f, 'utf8').trim().split('\n')
  .map((l) => l.split(',').map(Number)).filter((r) => r.length >= 36);

const mean = (a) => a.reduce((p, q) => p + q, 0) / (a.length || 1);

/**
 * The 29 CSV columns in packet order. The shipped g1_23dof URDF has no waist
 * roll/pitch and no wrist pitch/yaw, so six of these have nowhere to go; they
 * are reported rather than silently dropped.
 */
const PACKET_JOINTS = CSV_JOINT_ORDER;

export function auditClip(npzFile, csvFile) {
  const z = loadNPZ(npzFile);
  const jp = rows(z.joint_pos);                 // [frames][29]
  const bp = rows(z.body_pos_w);                // [frames][30*3]
  const bq = rows(z.body_quat_w);               // [frames][30*4]
  const n = jp.length;

  // ---- CSV vs NPZ -------------------------------------------------------
  let csvVsNpz = null;
  if (csvFile && fs.existsSync(csvFile)) {
    const csv = readCSV(csvFile);
    let dj = 0, dr = 0, cmp = 0;
    for (let i = 0; i < Math.min(n, csv.length); i++) {
      for (let k = 0; k < 29; k++) dj = Math.max(dj, Math.abs(csv[i][7 + k] - jp[i][k]));
      for (let k = 0; k < 3; k++) dr = Math.max(dr, Math.abs(csv[i][k] - bp[i][k]));
      cmp++;
    }
    csvVsNpz = { frames: cmp, maxJointDiff: dj, maxRootDiff: dr,
                 agree: dj < 1e-4 && dr < 1e-4 };
  }

  // ---- packet body_pos_w vs URDF forward kinematics ----------------------
  // Compare in PELVIS coordinates: the packet's body 0 is the pelvis, so
  // rotating each link offset back through the root quaternion removes the
  // root trajectory and leaves only what the joints are responsible for.
  const tree = bodyFK({});                       // names in our tree order
  const nameIdx = new Map(tree.map((l, i) => [l.name, i]));
  const bodyNames = fs.existsSync(path.join(path.dirname(npzFile), 'BODY_ORDER_BFS.txt'))
    ? fs.readFileSync(path.join(path.dirname(npzFile), 'BODY_ORDER_BFS.txt'), 'utf8')
        .trim().split('\n').map((l) => l.trim().split(/\s+/)[1])
    : [];
  const shared = bodyNames.map((nm, k) => ({ nm, k, t: nameIdx.get(nm) }))
    .filter((e) => e.t !== undefined && e.nm !== 'pelvis');

  let fkMax = 0; const fkPer = [];
  for (let i = 0; i < n; i++) {
    const q = {};
    for (let k = 0; k < 29; k++) q[PACKET_JOINTS[k]] = jp[i][k];
    const fk = bodyFK(q);
    // packet link offset from pelvis, rotated into the pelvis frame
    const R = quatToMat(bq[i][1], bq[i][2], bq[i][3], bq[i][0]);   // stored wxyz
    const Rt = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
    for (const e of shared) {
      const d = [bp[i][e.k * 3] - bp[i][0], bp[i][e.k * 3 + 1] - bp[i][1],
                 bp[i][e.k * 3 + 2] - bp[i][2]];
      const local = mapv(Rt, d);
      const want = fk[e.t].p;
      const err = Math.hypot(local[0] - want[0], local[1] - want[1], local[2] - want[2]);
      if (err > fkMax) fkMax = err;
      fkPer.push(err);
    }
  }

  // ---- do the feet ever reach the ground? -------------------------------
  // In the packets' own world the ground is z = 0, so the lowest ankle-roll
  // link should come within a sole thickness of it on every step.
  const SOLE = 0.035;
  const lowest = [];
  for (let i = 0; i < n; i++) {
    const q = {};
    for (let k = 0; k < 29; k++) q[PACKET_JOINTS[k]] = jp[i][k];
    const fk = bodyFK(q);
    const R = quatToMat(bq[i][1], bq[i][2], bq[i][3], bq[i][0]);
    let lo = Infinity;
    for (const side of ['left', 'right']) {
      const t = nameIdx.get(`${side}_ankle_roll_link`);
      const w = mapv(R, fk[t].p);
      lo = Math.min(lo, bp[i][2] + w[2] - SOLE);
    }
    lowest.push(lo);
  }

  return {
    frames: n, fps: z.fps.data[0],
    csvVsNpz,
    fk: { maxErr: fkMax, meanErr: mean(fkPer), links: shared.length,
          agree: fkMax < 5e-3 },
    foot: { minZ: Math.min(...lowest), maxZ: Math.max(...lowest), meanZ: mean(lowest),
            everPlanted: Math.min(...lowest) < 0.02 },
  };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const dirs = process.argv.length > 2 ? process.argv.slice(2) : findPackets();
  const table = [];
  for (const dir of dirs) {
    const npzDir = path.join(dir, 'NPZ'), csvDir = path.join(dir, 'CSV');
    const npzs = fs.readdirSync(npzDir).filter((f) => f.endsWith('.npz')).sort();
    console.log(`\n=== ${path.basename(dir)} ===`);
    for (const f of npzs) {
      const key = f[0];
      const csv = fs.existsSync(csvDir)
        ? fs.readdirSync(csvDir).find((c) => c.startsWith(`${key}_`) && c.endsWith('.csv'))
        : null;
      const r = auditClip(path.join(npzDir, f), csv && path.join(csvDir, csv));
      const c = r.csvVsNpz;
      console.log(`  ${key}  ${r.frames}f @${r.fps}fps`);
      console.log(`     CSV vs NPZ   ${c ? (c.agree ? 'agree' : `DIFFER  joint ${c.maxJointDiff.toExponential(2)}  root ${c.maxRootDiff.toExponential(2)}`) : 'no csv'}`);
      console.log(`     body_pos_w vs URDF FK   max ${(r.fk.maxErr * 1000).toFixed(1)} mm  `
                + `mean ${(r.fk.meanErr * 1000).toFixed(1)} mm over ${r.fk.links} links  `
                + `${r.fk.agree ? 'CONSISTENT' : 'INCONSISTENT'}`);
      console.log(`     lowest sole  min ${(r.foot.minZ * 1000).toFixed(0)} mm  `
                + `mean ${(r.foot.meanZ * 1000).toFixed(0)} mm  max ${(r.foot.maxZ * 1000).toFixed(0)} mm  `
                + `${r.foot.everPlanted ? 'touches ground' : 'NEVER TOUCHES GROUND'}`);
      table.push({ packet: path.basename(dir), clip: key, ...r });
    }
  }
  fs.mkdirSync('pipeline/out', { recursive: true });
  fs.writeFileSync('pipeline/out/packet_audit.json', JSON.stringify(table, null, 2));
  console.log(`\n${table.length} clips audited -> pipeline/out/packet_audit.json`);
}
