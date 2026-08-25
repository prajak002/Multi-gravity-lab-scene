/**
 * packet_metrics.mjs — read each packet's declared behaviour, whatever schema
 * it happens to use.
 *
 * The fourteen packets do not agree on a manifest format. Some put a two-row
 * array under `metrics`, some under `motion_metrics`, Mare Tranquillitatis uses
 * `A_metrics` / `B_metrics`, Gale has no metrics in its manifest at all and
 * only a METRICS/*.csv. Key names are scenario-specific too: the slip signal is
 * "Total slip proxy" on Ganges, "Mean rearward slip per contact proxy" on
 * Olympia Undae and "backslide_m" on Gale.
 *
 * This normalises all of it to one vocabulary, by fuzzy key match. The point is
 * that a clip's contact behaviour should be driven by what its own authors
 * measured, not by numbers invented here — so if a packet says WorldVLA
 * backslides 0.258 m, the retargeted clip backslides 0.258 m, as real foot slip
 * down the real fall line.
 */
import fs from 'fs';
import path from 'path';

const num = (v) => {
  const x = typeof v === 'string' ? parseFloat(v) : v;
  return Number.isFinite(x) ? x : undefined;
};

/** First value whose key contains all the given fragments (case-insensitive). */
function find(obj, ...frags) {
  if (!obj) return undefined;
  for (const [k, v] of Object.entries(obj)) {
    const lk = k.toLowerCase();
    if (frags.every((f) => lk.includes(f))) {
      const n = num(v);
      if (n !== undefined) return n;
    }
  }
  return undefined;
}

/** Pull the row for model A (WorldVLA) or B (PragyaSpace) out of any layout. */
function modelRow(manifest, which) {
  const isA = which === 'A';
  const match = (r) => (isA ? /worldvla|world_vla/i : /pragya/i).test(String(r?.Model ?? r?.model ?? ''));
  for (const key of ['metrics', 'motion_metrics']) {
    const m = manifest?.[key];
    if (Array.isArray(m)) {
      const row = m.find(match);
      if (row) return row;
    }
  }
  const direct = manifest?.[isA ? 'A_metrics' : 'B_metrics'];
  if (direct) return direct;
  return null;
}

/** Fallback: METRICS/motion_metrics.csv, keyed by the A_/B_ filename prefix. */
function csvRow(pkgDir, which) {
  const f = path.join(pkgDir, 'METRICS', 'motion_metrics.csv');
  if (!fs.existsSync(f)) return null;
  const lines = fs.readFileSync(f, 'utf8').trim().split('\n');
  if (lines.length < 2) return null;
  const head = lines[0].split(',');
  const want = which === 'A' ? /^A[_-]|worldvla/i : /^B[_-]|pragya/i;
  const line = lines.slice(1).find((l) => want.test(l.split(',')[0]));
  if (!line) return null;
  const cells = line.split(',');
  return Object.fromEntries(head.map((h, i) => [h.trim(), cells[i]]));
}

/**
 * @returns {{raw:object, peakBounce, stepCV, recoveryEvents, missedContacts,
 *            forward, slip, sinkage, clearance, lateral, peakPitch, peakRoll}}
 */
export function packetMetrics(pkgDir, which) {
  const mf = path.join(pkgDir, 'manifest.json');
  const manifest = fs.existsSync(mf) ? JSON.parse(fs.readFileSync(mf, 'utf8')) : {};
  const row = modelRow(manifest, which) || {};
  const csv = csvRow(pkgDir, which) || {};
  const all = { ...csv, ...row };

  // Slip appears under several names; take whichever the packet used.
  const slip = find(all, 'total slip')
    ?? find(all, 'rearward slip')
    ?? find(all, 'backslide')
    ?? find(all, 'slip');

  return {
    raw: all,
    signature: manifest[`motion_signature_${which}`] || manifest.task || '',
    peakBounce: find(all, 'peak', 'bounce') ?? find(all, 'peak', 'oscillation'),
    stepCV: find(all, 'step-duration cv') ?? find(all, 'step', 'cv'),
    recoveryEvents: find(all, 'recovery event'),
    missedContacts: find(all, 'missed-contact') ?? find(all, 'missed', 'contact'),
    forward: find(all, 'forward distance') ?? find(all, 'forward_distance'),
    slip,
    sinkage: find(all, 'sinkage'),
    extraction: find(all, 'extraction'),
    clearance: find(all, 'min', 'clearance') ?? find(all, 'clearance'),
    lateral: find(all, 'rms lateral') ?? find(all, 'lateral'),
    peakPitch: find(all, 'peak', 'pitch'),
    peakRoll: find(all, 'peak', 'roll'),
    peakYaw: find(all, 'peak yaw', 'deg'),
  };
}

/**
 * Turn measured behaviour into contact-solver settings.
 *
 * Counts are taken literally where the packet states them. Where it states only
 * a total slip distance, that distance is split into a plausible number of
 * discrete losses of traction rather than smeared across every step, because a
 * gait that slips a little on every footfall reads as a rendering error and a
 * gait that slips hard three times reads as a robot in trouble.
 */
export function styleFromMetrics(m, base = {}) {
  const o = { ...base };
  if (m.stepCV !== undefined) o.stepCV = Math.min(0.45, m.stepCV);

  let events = m.recoveryEvents;
  if (events === undefined && m.slip !== undefined && m.slip > 0.02) {
    events = Math.max(1, Math.min(5, Math.round(m.slip / 0.09)));
  }
  if (events !== undefined) o.recoveryEvents = Math.round(events);

  if (m.missedContacts !== undefined) o.missedContacts = Math.round(m.missedContacts);

  if (m.slip !== undefined && o.recoveryEvents > 0) {
    o.slipDistance = m.slip / o.recoveryEvents;
  } else if (m.slip !== undefined && m.slip <= 0.02) {
    o.recoveryEvents = 0;
  }

  if (m.sinkage !== undefined) o.sinkage = m.sinkage;
  // "Clearance" does not mean the same thing in every packet.
  //
  // On Olympia Undae it is 0.401 m for WorldVLA and 0.293 m for PragyaSpace,
  // and those are not foot clearances — they are the VERTICAL EXTRACTION
  // needed to pull a foot back out of sand, which the same packet also reports
  // separately as 0.148 m and 0.072 m. Taking it as swing clearance asked the
  // leg to lift 0.22 m and put it down again inside a 0.225 s swing, i.e. at
  // 2.44 m/s, and produced knee rates of 1927 deg/s against hardware rated
  // near 500.
  //
  // So a value that is too large to be a swing clearance is not treated as
  // one. 0.16 m is already a high step for a 0.79 m leg; beyond that the
  // number is describing something else and is left to `extraction`, which
  // has its own field.
  if (m.clearance !== undefined && m.clearance > 0.02 && m.clearance <= 0.16) {
    o.clearance = Math.max(o.clearance ?? 0, m.clearance);
  }
  return o;
}
