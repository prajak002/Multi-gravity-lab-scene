/**
 * audit_contact.mjs — measure whether a clip's feet actually work.
 *
 * Everything here is computed by real forward kinematics on the shipped URDF
 * chain and against the four contact spheres the URDF actually defines, not
 * against an idealised sole plane.
 *
 *   swingNet    net displacement of the swing foot relative to the pelvis,
 *               projected on the direction of travel. Positive means the foot
 *               reaches FORWARD as it swings, which is what walking is.
 *               Negative means the robot is moonwalking.
 *
 *   slipFrac    distance the loaded foot slides across the ground, as a
 *               fraction of the distance the robot covers. Zero is a planted
 *               foot; 1.0 means the feet skate as far as the robot travels.
 *
 * Contact is decided per CONTACT SPHERE against the ground directly beneath
 * that sphere. Using the foot's centroid instead is wrong on rocky terrain,
 * where the ground under the centroid and under the toe differ by centimetres
 * and a foot that is genuinely planted reads as sliding.
 */
import { contactPoints, quatToMat, mapv } from '../src/sim/G1Kinematics.js';

const LEG = {
  left: [7, 8, 9, 10, 11, 12],
  right: [13, 14, 15, 16, 17, 18],
};

/** The four contact spheres of one foot, in world coordinates. */
function footPoints(row, side) {
  const q = LEG[side].map((c) => row[c]);
  const R = quatToMat(row[3], row[4], row[5], row[6]);
  return contactPoints(side, q).map((p) => {
    const v = mapv(R, p);
    return [row[0] + v[0], row[1] + v[1], row[2] + v[2]];
  });
}

export function audit(rows, groundAt, contacts) {
  const n = rows.length;
  const d = [rows[n - 1][0] - rows[0][0], rows[n - 1][1] - rows[0][1]];
  const travel = Math.hypot(d[0], d[1]) || 1e-6;
  const dir = [d[0] / travel, d[1] / travel];

  let swingNet = 0, stanceSlip = 0, swingFrames = 0, stanceFrames = 0;
  let maxStanceStep = 0, peakClear = 0;
  // Slip measured against the gait's OWN contact schedule.
  //
  // The geometric threshold above is independent, which is what makes it a fair
  // test, but on ground with 14 cm boulders it also misfires: a foot arcing
  // PAST a rock momentarily reads as touching it. This second number asks the
  // question that actually matters — while the gait says this foot is carrying
  // load, how far did it travel? — and it is checkable against the contact
  // flags shipped in the clip.
  let schedSlip = 0, schedFrames = 0;

  for (const side of ['left', 'right']) {
    const pts = rows.map((r) => footPoints(r, side));
    // Clearance of the LOWEST sphere above the ground beneath THAT sphere.
    let clear;
    if (groundAt) {
      clear = pts.map((P) => Math.min(...P.map((p) => p[2] - groundAt(p[0], p[1]))));
    } else {
      const W = 18;
      const lows = pts.map((P) => Math.min(...P.map((p) => p[2])));
      clear = lows.map((v, i) => {
        let lo = Infinity;
        for (let k = Math.max(0, i - W); k <= Math.min(n - 1, i + W); k++) lo = Math.min(lo, lows[k]);
        return v - lo;
      });
    }
    // Centroid track, for measuring how far the foot travels.
    const C = pts.map((P) => [P.reduce((s, p) => s + p[0], 0) / 4,
                              P.reduce((s, p) => s + p[1], 0) / 4]);

    // Hysteresis: a foot is DOWN below 15 mm and stays down until it clears
    // 45 mm. A single threshold chatters across the touchdown frame and counts
    // swing motion as slip.
    const DOWN = 0.012, UP = 0.035;
    const down = new Array(n);
    let state = clear[0] < UP;
    for (let i = 0; i < n; i++) {
      if (state && clear[i] > UP) state = false;
      else if (!state && clear[i] < DOWN) state = true;
      down[i] = state;
    }

    for (let i = 1; i < n; i++) {
      peakClear = Math.max(peakClear, clear[i]);
      const relPrev = (C[i - 1][0] - rows[i - 1][0]) * dir[0] + (C[i - 1][1] - rows[i - 1][1]) * dir[1];
      const relNow = (C[i][0] - rows[i][0]) * dir[0] + (C[i][1] - rows[i][1]) * dir[1];
      const step = Math.hypot(C[i][0] - C[i - 1][0], C[i][1] - C[i - 1][1]);
      if (!down[i] && !down[i - 1]) { swingNet += relNow - relPrev; swingFrames++; }
      else if (down[i] && down[i - 1]) {
        stanceSlip += step; stanceFrames++;
        maxStanceStep = Math.max(maxStanceStep, step);
      }
      if (contacts) {
        const k = side === 'left' ? 0 : 1;
        if (contacts[i][k] && contacts[i - 1][k]) { schedSlip += step; schedFrames++; }
      }
    }
  }

  // ---- how the sole actually sits on the ground, while loaded -------------
  //
  // Position error at the ankle says the solver hit its target; it does not
  // say the foot is on the ground, because the target itself is a plane fitted
  // under a rigid sole. This measures the thing directly: for every frame the
  // gait says a foot is carrying load, where are its four contact spheres
  // relative to the terrain beneath each one?
  //
  //   penetration  the deepest a sphere goes below the surface. Regolith
  //                compresses, so a few millimetres is real; a centimetre is
  //                a foot inside a rock.
  //   corner gap   the highest sphere's clearance. A sole flat on the ground
  //                has all four within a few millimetres; a large gap means
  //                the foot is balanced on one edge.
  let penetration = 0, cornerGap = 0, gapSum = 0, gapN = 0;
  if (groundAt && contacts) {
    for (const side of ['left', 'right']) {
      const k = side === 'left' ? 0 : 1;
      for (let i = 0; i < n; i++) {
        if (!contacts[i][k]) continue;
        const P = footPoints(rows[i], side);
        const d = P.map((p) => p[2] - groundAt(p[0], p[1]));
        penetration = Math.min(penetration, Math.min(...d));
        const spread = Math.max(...d) - Math.min(...d);
        cornerGap = Math.max(cornerGap, spread);
        gapSum += spread; gapN++;
      }
    }
  }

  const slipFrac = stanceSlip / travel;
  const schedSlipFrac = contacts ? schedSlip / travel : undefined;
  return {
    penetration, cornerGap, meanCornerGap: gapN ? gapSum / gapN : undefined,
    travel, swingNet, stanceSlip, slipFrac,
    schedSlip: contacts ? schedSlip : undefined, schedSlipFrac, schedFrames,
    swingFrames, stanceFrames, maxStanceStep, peakClear,
    verdict: swingNet < 0 ? 'SWING INVERTED'
           : slipFrac > 0.25 ? 'SLIPPING'
           : slipFrac > 0.10 ? 'MINOR SLIP' : 'PLANTED',
  };
}
