/**
 * scenes.mjs — the scene table: which motion packet plays on which real
 * terrain, and how each model's character is expressed as contact behaviour.
 *
 * styleA / styleB are NOT arbitrary dials, and they are not where the drama
 * comes from. Every event COUNT — recoveries, missed contacts, slip distance,
 * step-duration variance — is read out of each packet's own manifest by
 * tools/packet_metrics.mjs and overrides whatever is written here
 * (styleFromMetrics wins). What these fields set is POSTURE: how the model
 * carries itself between events. A clip never contains struggle its authors
 * did not record.
 *
 * The recurring contrast, which is the same physical story at all twelve
 * surface sites:
 *
 *   WorldVLA      rides high on near-straight legs, so there is no knee
 *                 compliance when the ground moves; low foot clearance;
 *                 narrow base; stays upright while the terrain tilts; long
 *                 slow strides with a ragged period.
 *
 *   PragyaSpace   sits lower with real knee flexion; lifts the foot higher;
 *                 widens the base; rolls the torso with the grade; takes
 *                 shorter, more frequent, metronomic steps.
 */
export const G_MOON = 1.625, G_MARS = 3.721, G_ISS = 0.0;

// Posture defaults for the two models. Per-scene entries override only what
// the scenario actually changes, so a reader can see at a glance what is
// specific to a site and what is just the model being itself.
const A = { clearance: 0.055, stanceWiden: 0.018, rideHeight: 0.745,
            slipDistance: 0.11, terrainLean: 0.25, duty: 0.58, periodScale: 1.12 };
// PragyaSpace's cadence is NOT faster, it is REGULAR, and that distinction was
// wrong here for a while. The packets' own step summaries say so: Shiv Shakti
// B runs 0.87-0.89 s per step while A ranges 0.62-1.18 s, so the means are
// close and the variance is the whole story — which is exactly what the
// step-duration CV of 0.009 against 0.205 measures, and that comes from the
// metrics rather than from here.
//
// Setting B faster AND higher-duty, as this did, is self-contradictory: both
// shorten the swing, and together they left 0.174 s of it on Ganges. A gait
// that wants long double support has to give itself a cycle long enough to
// hold one.
const B = { clearance: 0.095, stanceWiden: 0.045, rideHeight: 0.725,
            slipDistance: 0.03, terrainLean: 0.80, duty: 0.66, periodScale: 1.00 };

export const SCENES = {
  // =========================================================================
  // MOON — six sites, six different motion problems.
  // =========================================================================
  moon_shiv_shakti: {
    name: 'SHIV SHAKTI POINT — uneven regolith traversal',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_ShivShakti_v2',
    dem: 'moon_shiv_shakti', profile: 'moon_regolith',
    origin: [-5, -1], heading: 'upslope', seed: 1101,
    blurb: 'The Chandrayaan-3 landing site, on undulating mare-highland '
         + 'regolith. Earth-tuned control puts in more push than one sixth g '
         + 'needs and over-bounces: the packet records a 0.44 m peak pelvis '
         + 'bounce against PragyaSpace\'s 0.055 m, a step-duration CV of 0.205 '
         + 'against 0.009, three recoveries and a missed contact against none.',
    styleA: { ...A, rideHeight: 0.750, clearance: 0.050 },
    styleB: { ...B, rideHeight: 0.720, clearance: 0.100 },
  },

  moon_shackleton_rim: {
    name: 'SHACKLETON RIM — steep uphill on the polar rim',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_ShackletonRim_v1',
    dem: 'moon_shackleton_rim', profile: 'moon_blocky',
    origin: [-4, 0], heading: 'upslope', seed: 1207,
    blurb: 'A real 12 deg rim grade over 184 m of relief, from LOLA polar '
         + 'topography at 5 m/px — the finest terrain in this set. WorldVLA '
         + 'pushes as if it were on the flat and its own metrics show the '
         + 'cost: 0.67 m of pelvis bounce, 22.9 deg of peak pitch, four '
         + 'recoveries and two missed contacts. PragyaSpace takes short '
         + 'controlled steps and keeps the mass over the feet.',
    styleA: { ...A, rideHeight: 0.755, clearance: 0.048, terrainLean: 0.20 },
    styleB: { ...B, rideHeight: 0.712, clearance: 0.110, terrainLean: 0.90,
              duty: 0.70, periodScale: 1.00 },
  },

  moon_mare_tranquillitatis: {
    name: 'MARE TRANQUILLITATIS — flat regolith cruise',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_MareTranquillitatis_v1',
    dem: 'moon_mare_tranquillitatis', profile: 'moon_mare',
    origin: [0, 0], heading: 0.0, seed: 1303,
    blurb: 'The flattest window the terrain scan could find, which is the '
         + 'point: with nothing to negotiate, the only difference left is '
         + 'efficiency. WorldVLA still bounces 0.19 m and wanders 0.026 m off '
         + 'line; PragyaSpace bounces 0.032 m, holds 0.001 m, and its '
         + 'step-duration CV is zero to sixteen decimal places.',
    styleA: { ...A, rideHeight: 0.745, clearance: 0.052, terrainLean: 0.15 },
    styleB: { ...B, rideHeight: 0.728, clearance: 0.085, periodScale: 1.00 },
  },

  moon_aristarchus: {
    name: 'ARISTARCHUS — high-step, side-step, pivot, edge',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_Aristarchus_v1',
    dem: 'moon_aristarchus', profile: 'moon_blocky',
    origin: [-5, 2], heading: 0.5, seed: 1409,
    blurb: 'Blocky ejecta on the Aristarchus plateau, where the problem is '
         + 'obstacle GEOMETRY rather than grade. WorldVLA corrects late and '
         + 'swings 42 deg of yaw doing it; PragyaSpace reads the block it is '
         + 'about to meet, lifts early and contours the edge, and needs 31 '
         + 'deg. Both cover the same 3.9 m.',
    styleA: { ...A, clearance: 0.054, stanceWiden: 0.022 },
    styleB: { ...B, clearance: 0.135, stanceWiden: 0.055, terrainLean: 0.85 },
  },

  moon_tycho_flank: {
    name: 'TYCHO FLANK — steep downhill braking',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_TychoFlank_v1',
    dem: 'moon_tycho_flank', profile: 'moon_slope',
    origin: [4, 1], heading: 'downslope', seed: 1511,
    blurb: 'Down a 13.6 deg flank of young Tycho ejecta. Downhill is a braking '
         + 'problem, not a propulsion one: at one sixth g there is little '
         + 'weight to brake with and a long time to fall. WorldVLA accelerates '
         + 'and covers 4.80 m to PragyaSpace\'s 3.56 m — the shorter traverse '
         + 'is the better one, because it is the one under control.',
    styleA: { ...A, rideHeight: 0.745, clearance: 0.050, terrainLean: 0.20,
              periodScale: 1.20 },
    styleB: { ...B, rideHeight: 0.705, clearance: 0.100, terrainLean: 0.95,
              duty: 0.72, periodScale: 1.02 },
  },

  moon_schrodinger_basin: {
    name: 'SCHRÖDINGER BASIN — mixed-terrain adaptation',
    body: 'Moon', g: G_MOON,
    packet: 'packets/moon/PragyaSpace_SchrodingerBasin_v1',
    dem: 'moon_schrodinger_basin', profile: 'moon_regolith',
    origin: [-6, -3], heading: 'upslope', seed: 1613,
    blurb: 'Cruise, then rough walking, then climbing — one traverse across '
         + 'three regimes, testing whether the controller notices the regime '
         + 'has changed. WorldVLA switches late; its foot clearance stays at '
         + '47 mm throughout, the same number it used on the flat.',
    styleA: { ...A, clearance: 0.047, terrainLean: 0.22 },
    styleB: { ...B, clearance: 0.105, terrainLean: 0.85, duty: 0.68 },
  },

  // =========================================================================
  // MARS — six sites. Higher gravity, so the pendulum is 1.5x faster than the
  // Moon's and the capture point correspondingly closer.
  // =========================================================================
  mars_jezero_delta: {
    name: 'JEZERO DELTA — obstacle slalom',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_JezeroDelta_v1',
    dem: 'mars_jezero_delta', profile: 'mars_delta',
    origin: [-6, 1], heading: 0.35, seed: 4007,
    blurb: 'Weaving the boulder field at the delta front. Same 5.65 m for '
         + 'both, but WorldVLA reacts to each rock as it arrives — 41 mm of '
         + 'foot clearance and 42 deg of yaw swing — while PragyaSpace plans '
         + 'the line, clears 179 mm and needs only 20 deg.',
    styleA: { ...A, clearance: 0.048, stanceWiden: 0.020 },
    styleB: { ...B, clearance: 0.150, stanceWiden: 0.050 },
  },

  mars_gale_crater: {
    name: 'GALE CRATER — rocky uphill traction',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_GaleCrater_v1',
    dem: 'mars_gale_crater', profile: 'mars_rocky',
    origin: [-6, -2], heading: 'upslope', seed: 4021,
    blurb: 'A sustained 8.9 deg ascent over Murray-formation float rock, on '
         + 'the 1 m/px HiRISE DEM built for MSL operations — 128 real source '
         + 'posts across the patch, the best-resolved site here alongside '
         + 'Shackleton. The question is traction: WorldVLA commits weight '
         + 'before the foot has settled and backslides 0.258 m doing it; '
         + 'PragyaSpace contours the slope and loads each foothold only once '
         + 'it is supported.',
    styleA: { ...A, rideHeight: 0.745, clearance: 0.055, slipDistance: 0.13 },
    styleB: { ...B, rideHeight: 0.725, clearance: 0.095 },
  },

  mars_olympia_undae: {
    name: 'OLYMPIA UNDAE — deep sand',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_OlympiaUndae_v1',
    dem: 'mars_olympia_undae', profile: 'mars_sand',
    origin: [-5, 0], heading: 0.2, seed: 4103,
    blurb: 'The north polar dune sea, where the ground gives way under load. '
         + 'The packet measures it directly: WorldVLA sinks 122 mm and needs '
         + '148 mm of vertical extraction to get the foot back out, against '
         + '47 mm and 72 mm for PragyaSpace, which loads the sand gradually '
         + 'instead of dropping onto it.',
    styleA: { ...A, clearance: 0.060, rideHeight: 0.740, periodScale: 1.15 },
    styleB: { ...B, clearance: 0.125, rideHeight: 0.718, duty: 0.70 },
  },

  mars_cerberus_fossae: {
    name: 'CERBERUS FOSSAE — gap crossing',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_CerberusFossae_v1',
    dem: 'mars_cerberus_fossae', profile: 'mars_fissure',
    origin: [-6, 0], heading: 0.0, seed: 4211,
    blurb: 'A fissure across the line of travel. WorldVLA arrives at speed '
         + 'and bounds late — 20.7 deg of pitch and 11.9 deg of roll as it '
         + 'commits — while PragyaSpace stops, preloads, spans deliberately '
         + 'and lands where it aimed, at 9.5 deg and 0.7 deg.',
    styleA: { ...A, clearance: 0.058, periodScale: 1.18, terrainLean: 0.20 },
    styleB: { ...B, clearance: 0.130, duty: 0.70, periodScale: 1.00 },
  },

  mars_medusae_fossae: {
    name: 'MEDUSAE FOSSAE — threading the yardangs',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_MedusaeFossae_v1',
    dem: 'mars_medusae_fossae', profile: 'mars_yardang',
    origin: [-5, -1], heading: 1.72, seed: 4307,
    blurb: 'Wind-cut ridges all running the same way, leaving a corridor '
         + 'narrower than a comfortable stance. Both cover 5.25 m. WorldVLA '
         + 'oscillates across the corridor and accumulates 0.271 m of lateral '
         + 'error; PragyaSpace turns side-on and cross-steps through, holding '
         + '0.020 m.',
    styleA: { ...A, stanceWiden: 0.012, clearance: 0.052 },
    styleB: { ...B, stanceWiden: 0.062, clearance: 0.115, duty: 0.72 },
  },

  mars_ganges_chasma: {
    name: 'GANGES CHASMA — steep descent by switchback',
    body: 'Mars', g: G_MARS,
    packet: 'packets/mars/PragyaSpace_GangesChasma_v1',
    dem: 'mars_ganges_chasma', profile: 'mars_talus',
    origin: [5, 2], heading: 'downslope', seed: 4409,
    blurb: 'Down a wall of Valles Marineris on loose talus. WorldVLA takes '
         + 'the fall line and slides 0.49 m doing it, through 18 deg of '
         + 'pitch; PragyaSpace cuts diagonally across the grade, brakes into '
         + 'each step, and gives up 0.09 m.',
    styleA: { ...A, rideHeight: 0.750, clearance: 0.050, slipDistance: 0.16,
              terrainLean: 0.18, periodScale: 1.22 },
    styleB: { ...B, rideHeight: 0.702, clearance: 0.100, terrainLean: 0.95,
              duty: 0.74, periodScale: 1.04 },
  },

  // =========================================================================
  // ISS — microgravity. No terrain, no footfalls: the only contacts are the
  // start wall, the handrail and — for the model that gets it wrong — the
  // bulkhead. See tools/retarget_micro.mjs; none of the contact machinery
  // above applies, and what replaces it is conservation.
  // =========================================================================
  iss_momentum_gap: {
    name: 'MOMENTUM GAP — is there enough push to arrive?',
    body: 'ISS', g: G_ISS, micro: true,
    packet: 'packets/iss/PragyaSpace_ISS_MomentumGap_v6_WallPushFaceDown',
    seed: 6101,
    blurb: 'Both robots push off the same wall. WorldVLA leaves with 0.18 m/s '
         + 'and, finding itself short, swims — a full breaststroke that cannot '
         + 'work, because with no external contact the centre of mass keeps '
         + 'exactly the velocity the wall gave it. Every sweep of the arms '
         + 'only rocks the body about a COM that will not move. PragyaSpace '
         + 'computes the impulse the gap requires, leaves at 0.42 m/s and '
         + 'coasts to the rail.',
    clipA: { stroke: 'breaststroke', strokePeriod: 1.05 },
    clipB: { stroke: 'glide' },
  },

  iss_brake_gap: {
    name: 'BRAKE GAP — can the momentum be removed on arrival?',
    body: 'ISS', g: G_ISS, micro: true,
    packet: 'packets/iss/PragyaSpace_ISS_BrakeGap_v1',
    seed: 6203, speedOverride: 0.72,
    blurb: 'Identical 0.72 m/s departures, 25.2 N s of momentum to shed. '
         + 'PragyaSpace positions the lead hand early, catches the rail at '
         + '4.8 s and lets the elbow and shoulder absorb the arrest over a '
         + 'quarter second. WorldVLA reaches late, misses the capture and '
         + 'takes the impulse from the bulkhead instead.',
    clipA: { stroke: 'glide' },
    clipB: { stroke: 'glide' },
  },
};
