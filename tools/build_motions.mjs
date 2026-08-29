/**
 * build_motions.mjs — bake the gravity-driven motions onto a real NASA site.
 *
 * tools/retarget.mjs takes a packet someone else authored and reconciles it
 * with the ground. This does the opposite: there is no packet, the motion is
 * GENERATED from the field strength at the site, and the only thing it has in
 * common with the retargeter is that the feet are solved against the same
 * SiteField by the same IK.
 *
 * That is the point. A packet walk looks nearly identical on the Moon and on
 * Mars because it was authored once and re-planted twice — the gait is the
 * author's, and gravity only gets to adjust it. Here the gait does not exist
 * until `g` is known:
 *
 *     apex    = v0^2 / 2g          1.18 m on the Moon, 0.51 m on Mars
 *     hang    = 2 v0 / g           2.41 s on the Moon, 1.05 s on Mars
 *     duty    = stance / cycle     0.18 on the Moon, 0.34 on Mars
 *     speed  <= mu g t_stance      0.39 m/s on the Moon, 0.89 m/s on Mars
 *
 * with v0 the same to within 3 % on both, because it is set by the knee's rated
 * SPEED and not by the field. See src/sim/Ballistic.js for that derivation.
 *
 * The last of those four is the Apollo result that surprises people. Low
 * gravity does not make you fast — it makes you slow, because forward
 * acceleration comes from friction and friction comes from weight. The crews
 * loped not because it was exuberant but because at 0.39 m/s of available
 * push there was nothing else worth doing with a 2.4 second flight phase.
 *
 *   node tools/build_motions.mjs                    # every site x every motion
 *   node tools/build_motions.mjs moon_apollo15_hadley
 */
import fs from 'fs';
import { SiteField, SURFACE_PROFILES } from '../src/terrain/SiteField.js';
import { MOTIONS, toppleTime } from '../src/sim/Ballistic.js';
import { generateHop } from '../src/sim/HopMotion.js';
import { audit } from './audit_contact.mjs';
import { rateAudit, JOINT_RATE_LIMIT } from './audit_rates.mjs';
import { JOINT_NAMES } from './build_scene.mjs';
import { SITES } from './sites.mjs';


const FPS = 30;

function loadField(demName, profile) {
  const meta = JSON.parse(fs.readFileSync(`public/dem/${demName}.json`, 'utf8'));
  const buf = fs.readFileSync(`public/dem/${demName}.f32`);
  const dem = new Float32Array(buf.buffer, buf.byteOffset, buf.length / 4);
  return new SiteField(dem, meta, SURFACE_PROFILES[profile] || SURFACE_PROFILES.moon_mare);
}

const buildMotion = (siteId, motionId, opt) =>
  generateHop(loadField(SITES[siteId].dem, SITES[siteId].profile), SITES[siteId], motionId, opt);
export { buildMotion };

export function buildMotionScene(siteId) {
  const site = SITES[siteId];
  const out = {
    id: siteId, name: site.name, body: site.body, g: site.g, fps: FPS,
    place: site.place, blurb: site.blurb,
    terrain: site.dem ? { dem: site.dem, profile: site.profile, origin: site.origin ?? [-4, 0], heading: 0 } : undefined,
    // Enough for the browser to regenerate this motion at a different thrust
    // without another round trip. src/sim/HopMotion.js is the same code that
    // baked it, so what the slider produces and what shipped are the same
    // thing at two settings.
    site: { g: site.g, body: site.body, friction: site.friction ?? 0.45,
            origin: site.origin ?? [-4, 0] },
    motions: {},
  };
  if (site.dem) {
    const f = loadField(site.dem, site.profile);
    out.terrain.meta = f.meta;
  }
  const rnd = (v) => Math.round(v * 1e5) / 1e5;

  for (const id of Object.keys(MOTIONS)) {
    const r = buildMotion(siteId, id);
    const clip = {
      label: MOTIONS[id].label,
      blurb: MOTIONS[id].blurb,
      generated: true,
      robot: 'g1', fps: FPS, frames: r.rows.length,
      joints: JOINT_NAMES,
      angles: r.rows.map((row) => row.slice(7, 36).map(rnd)),
      root: r.rows.map((row) => [rnd(row[0]), rnd(row[1]), rnd(row[2])]),
      quat: r.rows.map((row) => [rnd(row[3]), rnd(row[4]), rnd(row[5]), rnd(row[6])]),
      contacts: r.contacts,
      // Everything the readout quotes, computed from g and the URDF.
      // Everything the readout quotes, computed from g, the thrust and the
      // URDF — including what the thrust does to the robot's attitude, which
      // is the half of the story a jump height alone does not tell.
      physics: {
        g: r.g, thrust: r.phys.thrust, v0: r.phys.v0,
        apex: r.phys.apex, hang: r.phys.hang,
        duty: r.phys.dutyFactor, speedCeiling: r.phys.speedCeiling,
        range: r.phys.range, boundBy: r.phys.boundBy,
        force: r.phys.force, offset: r.phys.offset,
        omega: r.phys.omega, tumble: r.phys.tumble,
        armAuthority: r.phys.armAuthority, residual: r.phys.residual,
        correctable: r.phys.correctable, index: r.phys.index,
        stable: r.phys.stable, ankleTorqueLimit: r.phys.ankleTorqueLimit,
        capture: r.phys.capture, reach: r.phys.reach, canCapture: r.phys.canCapture,
        toppleTime: toppleTime(r.g, 0.6),
      },
      audit: { after: audit(r.rows, (x, y) => r.field.heightAt(x, -y), r.contacts) },
      ik: {
        maxPosErr: r.diag.maxPosErr, maxRotErr: r.diag.maxRotErr,
        // Fraction of loaded foot-frames with the ankle against its roll stop.
        ankleRollSaturated: r.diag.loadedFrames
          ? r.diag.rollSaturated / r.diag.loadedFrames : 0,
      },
    };
    const ra = rateAudit(clip);
    let peak = 0, over = 0, frames = 0;
    for (const v of Object.values(ra)) { peak = Math.max(peak, v.peak); over += v.over; frames += v.frames; }
    clip.rates = {
      limitDegPerSec: JOINT_RATE_LIMIT, peakDegPerSec: Math.round(peak),
      overFrac: frames ? over / frames : 0,
      feasiblePlayback: peak > 0 ? Math.min(1, JOINT_RATE_LIMIT / peak) : 1,
    };
    out.motions[id] = clip;
  }
  fs.mkdirSync('public/motions_baked', { recursive: true });
  fs.writeFileSync(`public/motions_baked/${siteId}.json`, JSON.stringify(out));
  return out;
}

if (process.argv[1]?.endsWith('build_motions.mjs')) {
  const ids = process.argv.length > 2 ? process.argv.slice(2)
    : Object.keys(SITES).filter((k) => SITES[k].dem);
  console.log('site'.padEnd(30) + 'motion  apex    hang   duty   vmax   penet   slip%  rollSat%');
  for (const id of ids) {
    try {
      const out = buildMotionScene(id);
      for (const [m, c] of Object.entries(out.motions)) {
        const p = c.physics, a = c.audit.after;
        console.log(`${id.padEnd(30)}${m.padEnd(7)} `
          + `${p.apex.toFixed(2).padStart(5)}m ${p.hang.toFixed(2).padStart(6)}s `
          + `${p.duty.toFixed(2).padStart(6)} ${p.speedCeiling.toFixed(2).padStart(5)} `
          + `${(a.penetration * 1000).toFixed(1).padStart(7)}mm ${(a.schedSlipFrac * 100).toFixed(1).padStart(6)}`
          + `${(c.ik.ankleRollSaturated * 100).toFixed(0).padStart(8)}`);
      }
    } catch (e) {
      console.log(`${id.padEnd(30)}FAILED: ${e.message}`);
    }
  }
}
