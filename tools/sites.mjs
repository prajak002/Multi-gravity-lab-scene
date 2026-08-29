/**
 * sites.mjs — every place the arena can stand, and which body it is on.
 *
 * The ids are the same ones `pipeline/dem.py` fetches under, and
 * `tools/check_sites.mjs` asserts the two lists have not drifted apart. What
 * lives where is deliberate:
 *
 *   dem.py   coordinates, product, the grade the scenario needs, how far the
 *            window scan may wander. Everything about the OBSERVATION.
 *   here     the label a person reads, the body, its gravity, the surface
 *            profile the micro-relief layer uses, and what makes the place
 *            worth standing on. Everything about the SCENE.
 *
 * `g` is the standard surface value for each body. Not a per-site number: the
 * variation across a body is in the fifth decimal and the arena would be
 * claiming a precision the DEM cannot support.
 *
 * `friction` is what bounds forward speed, because at low gravity horizontal
 * acceleration is limited by weight rather than by anything about the legs. A
 * smooth sole on lunar regolith is about 0.4; Martian sand grips a little
 * better; a dune field or a talus slope rather less. This is the number that
 * makes the Moon SLOW, which is the part of one sixth g that surprises people.
 */
export const MOON_G = 1.625, MARS_G = 3.721, ISS_G = 0.0;

const moon = (o) => ({ body: 'Moon', g: MOON_G, friction: 0.42, ...o });
const mars = (o) => ({ body: 'Mars', g: MARS_G, friction: 0.50, ...o });
const iss = (o) => ({ body: 'ISS', g: ISS_G, micro: true, ...o });

export const SITES = {
  // =========================================================================
  // MOON
  // =========================================================================
  moon_shiv_shakti: moon({
    place: 'Shiv Shakti Point',
    name: 'SHIV SHAKTI POINT — Chandrayaan-3 landing site',
    dem: 'moon_shiv_shakti', profile: 'moon_regolith', origin: [-5, -1],
    blurb: 'Where Vikram set down in August 2023, on undulating mare-highland '
         + 'regolith south of 69 S. The ground is not steep; it is simply never '
         + 'the same twice.',
  }),
  moon_shackleton_rim: moon({
    place: 'Shackleton Rim',
    name: 'SHACKLETON RIM — the lunar south pole',
    dem: 'moon_shackleton_rim', profile: 'moon_blocky', origin: [-4, 0],
    friction: 0.38,
    blurb: 'A real 13 degree rim grade, from LOLA polar topography at 5 m/px — '
         + 'the finest lunar terrain published anywhere. Steep enough that the '
         + 'ankle runs out of dorsiflexion before the solver runs out of ideas.',
  }),
  moon_mare_tranquillitatis: moon({
    place: 'Mare Tranquillitatis',
    name: 'MARE TRANQUILLITATIS — flat basalt plain',
    dem: 'moon_mare_tranquillitatis', profile: 'moon_mare', origin: [0, 0],
    blurb: 'The flattest window the terrain scan could find, which is the '
         + 'point: with nothing to negotiate, the only thing left to compare is '
         + 'efficiency.',
  }),
  moon_aristarchus: moon({
    place: 'Aristarchus Plateau',
    name: 'ARISTARCHUS — blocky ejecta and scarps',
    dem: 'moon_aristarchus', profile: 'moon_blocky', origin: [-5, 2],
    friction: 0.40,
    blurb: 'The brightest region on the Moon, and the roughest ground in this '
         + 'set. The problem here is obstacle GEOMETRY rather than grade.',
  }),
  moon_tycho_flank: moon({
    place: 'Tycho Flank',
    name: 'TYCHO FLANK — steep downhill braking',
    dem: 'moon_tycho_flank', profile: 'moon_slope', origin: [4, 1],
    friction: 0.38,
    blurb: 'Down a 13.6 degree flank of young Tycho ejecta. Downhill at one '
         + 'sixth g is a braking problem: there is little weight to brake with '
         + 'and a very long time to fall.',
  }),
  moon_schrodinger_basin: moon({
    place: 'Schrödinger Basin',
    name: 'SCHRÖDINGER BASIN — mixed terrain',
    dem: 'moon_schrodinger_basin', profile: 'moon_regolith', origin: [-6, -3],
    blurb: 'A 320 km farside impact basin with a volcanic vent on its floor. '
         + 'Cruise, rough walking and a climb in one traverse.',
  }),
  moon_apollo11_tranquility: moon({
    place: 'Apollo 11',
    name: 'APOLLO 11 — Tranquility Base',
    dem: 'moon_apollo11_tranquility', profile: 'moon_mare', origin: [-4, 0],
    blurb: 'The first walk, 20 July 1969. Armstrong described the surface as '
         + 'fine and powdery and reported that moving about was easier than the '
         + 'simulations had been — which is exactly what a duty factor of 0.18 '
         + 'looks like from the inside.',
  }),
  moon_apollo15_hadley: moon({
    place: 'Apollo 15',
    name: 'APOLLO 15 — Hadley Rille and the Apennine Front',
    dem: 'moon_apollo15_hadley', profile: 'moon_slope', origin: [-5, 0],
    friction: 0.40,
    blurb: 'The first rover mission, working up the Apennine Front beside a '
         + '300 m deep sinuous rille. Slope walking, where a lope stops being '
         + 'efficient and control matters more than range.',
  }),
  moon_apollo17_taurus_littrow: moon({
    place: 'Apollo 17',
    name: 'APOLLO 17 — Taurus-Littrow valley',
    dem: 'moon_apollo17_taurus_littrow', profile: 'moon_regolith', origin: [-5, 1],
    blurb: 'The last walk, and the longest: a valley deeper than the Grand '
         + 'Canyon, between massifs. This is the mission with the most film of '
         + 'crews falling over and getting up again.',
  }),
  moon_copernicus: moon({
    place: 'Copernicus',
    name: 'COPERNICUS — terraced crater wall',
    dem: 'moon_copernicus', profile: 'moon_blocky', origin: [4, 1],
    friction: 0.36,
    blurb: 'A 93 km young impact crater with a terraced inner wall — the '
         + 'steepest sustained ground in this set, and the place where a hop '
         + 'that lands badly has the furthest to go.',
  }),
  moon_marius_hills: moon({
    place: 'Marius Hills',
    name: 'MARIUS HILLS — volcanic domes and a lava-tube skylight',
    dem: 'moon_marius_hills', profile: 'moon_regolith', origin: [-4, 0],
    blurb: 'A field of low volcanic domes in Oceanus Procellarum, and the most '
         + 'discussed lava-tube skylight on the Moon — a 65 m hole into a tube '
         + 'that would make a ready-made habitat.',
  }),
  moon_reiner_gamma: moon({
    place: 'Reiner Gamma',
    name: 'REINER GAMMA — magnetic swirl',
    dem: 'moon_reiner_gamma', profile: 'moon_mare', origin: [0, 0],
    blurb: 'A magnetic anomaly with almost no topographic expression at all. '
         + 'The swirl is bright and invisible to a DEM, so what a traverse here '
         + 'tests is pure flat-ground efficiency.',
  }),
  moon_plato: moon({
    place: 'Plato',
    name: 'PLATO — flooded crater floor',
    dem: 'moon_plato', profile: 'moon_mare', origin: [0, 0],
    blurb: 'A lava-flooded crater floor ringed by massifs, and one of the '
         + 'smoothest large surfaces on the nearside.',
  }),
  moon_tsiolkovskiy: moon({
    place: 'Tsiolkovskiy',
    name: 'TSIOLKOVSKIY — farside central peak',
    dem: 'moon_tsiolkovskiy', profile: 'moon_slope', origin: [4, 0],
    friction: 0.38,
    blurb: 'The farside\'s most striking crater: a bright central peak standing '
         + 'in a dark mare-flooded floor. Steep ground off the peak complex.',
  }),
  moon_malapert_massif: moon({
    place: 'Malapert Massif',
    name: 'MALAPERT MASSIF — Artemis candidate',
    dem: 'moon_malapert_massif', profile: 'moon_blocky', origin: [-4, 0],
    friction: 0.38,
    blurb: 'A south-polar massif with near-permanent line of sight to Earth, '
         + 'which is what puts it on the Artemis candidate list. Steep, and lit '
         + 'at a grazing angle that never changes.',
  }),
  moon_de_gerlache_rim: moon({
    place: 'de Gerlache Rim',
    name: 'DE GERLACHE RIM — Artemis candidate',
    dem: 'moon_de_gerlache_rim', profile: 'moon_blocky', origin: [-4, 0],
    friction: 0.38,
    blurb: 'Polar rim beside a permanently shadowed floor that has not seen '
         + 'sunlight in two billion years. Read at 5 m/px, the finest lunar '
         + 'topography there is.',
  }),

  // =========================================================================
  // MARS
  // =========================================================================
  mars_jezero_delta: mars({
    place: 'Jezero Delta',
    name: 'JEZERO CRATER — the western delta front',
    dem: 'mars_jezero_delta', profile: 'mars_delta', origin: [-6, 1],
    blurb: 'Where Perseverance landed in February 2021, at the front of a '
         + 'river delta that emptied into a crater lake. Cobbles and boulders '
         + 'shed from the scarp.',
  }),
  mars_gale_crater: mars({
    place: 'Gale Crater',
    name: 'GALE CRATER — lower Mount Sharp',
    dem: 'mars_gale_crater', profile: 'mars_rocky', origin: [-6, -2],
    blurb: 'Curiosity\'s ground, read from the 1 m/px HiRISE mosaic — the finest '
         + 'DEM of anywhere on Mars, and the only site here whose landforms are '
         + 'resolved at the scale a robot walks.',
  }),
  mars_olympia_undae: mars({
    place: 'Olympia Undae',
    name: 'OLYMPIA UNDAE — north polar dune sea',
    dem: 'mars_olympia_undae', profile: 'mars_sand', origin: [-5, 0],
    friction: 0.38,
    blurb: 'The largest dune field on Mars, wrapped around the north polar cap. '
         + 'Deep sand: the DEM gives the regional grade and the ripple the foot '
         + 'sinks into is synthesised.',
  }),
  mars_cerberus_fossae: mars({
    place: 'Cerberus Fossae',
    name: 'CERBERUS FOSSAE — fissure system',
    dem: 'mars_cerberus_fossae', profile: 'mars_fissure', origin: [-6, 0],
    blurb: 'Young tectonic fissures, and the source of most of the marsquakes '
         + 'InSight recorded. The fissure is narrower than a 200 m pixel, so '
         + 'the DEM supplies the plain it cuts and nothing more.',
  }),
  mars_medusae_fossae: mars({
    place: 'Medusae Fossae',
    name: 'MEDUSAE FOSSAE — yardang field',
    dem: 'mars_medusae_fossae', profile: 'mars_yardang', origin: [-5, -1],
    friction: 0.44,
    blurb: 'The largest deposit of loose material on Mars, carved by wind into '
         + 'parallel ridges. Threading between yardangs, all of them running '
         + 'the same way.',
  }),
  mars_ganges_chasma: mars({
    place: 'Ganges Chasma',
    name: 'GANGES CHASMA — Valles Marineris wall',
    dem: 'mars_ganges_chasma', profile: 'mars_talus', origin: [5, 2],
    friction: 0.42,
    blurb: 'The steepest wall section the terrain scan could find in the '
         + 'eastern Valles Marineris. Descent by switchback, on talus.',
  }),
  mars_olympus_mons: mars({
    place: 'Olympus Mons',
    name: 'OLYMPUS MONS — the flank',
    dem: 'mars_olympus_mons', profile: 'mars_rocky', origin: [-5, 0],
    blurb: 'The largest volcano in the solar system, 22 km high and 600 km '
         + 'across. The flank grade is gentle and utterly relentless — the '
         + 'endurance case rather than the difficulty one.',
  }),
  mars_melas_chasma: mars({
    place: 'Melas Chasma',
    name: 'MELAS CHASMA — the deepest wall on Mars',
    dem: 'mars_melas_chasma', profile: 'mars_talus', origin: [5, 1],
    friction: 0.40,
    blurb: 'Central Valles Marineris, where the trough is 8 km deep. The '
         + 'steepest sustained descent available anywhere on the planet.',
  }),
  mars_elysium_insight: mars({
    place: 'InSight',
    name: 'ELYSIUM PLANITIA — InSight landing site',
    dem: 'mars_elysium_insight', profile: 'mars_rocky', origin: [0, 0],
    blurb: 'Chosen by NASA precisely for being dull: the flattest, safest, '
         + 'least interesting plain they could find, because the mission '
         + 'listened to the inside of the planet rather than looking at the '
         + 'outside. Here that makes it the control.',
  }),
  mars_meridiani_opportunity: mars({
    place: 'Opportunity',
    name: 'MERIDIANI PLANUM — Opportunity landing site',
    dem: 'mars_meridiani_opportunity', profile: 'mars_sand', origin: [0, 0],
    friction: 0.42,
    blurb: 'The haematite plain Opportunity drove for fourteen years and 45 km. '
         + 'Flat at the DEM\'s scale, and covered in the ripple field that the '
         + 'synthetic layer supplies.',
  }),
  mars_gusev_spirit: mars({
    place: 'Spirit',
    name: 'GUSEV CRATER — Spirit and the Columbia Hills',
    dem: 'mars_gusev_spirit', profile: 'mars_rocky', origin: [-5, 0],
    blurb: 'Spirit climbed Husband Hill here, which is still the steepest '
         + 'sustained ascent any rover has driven on another planet.',
  }),
  mars_utopia_zhurong: mars({
    place: 'Zhurong',
    name: 'UTOPIA PLANITIA — Zhurong landing site',
    dem: 'mars_utopia_zhurong', profile: 'mars_sand', origin: [0, 0],
    friction: 0.44,
    blurb: 'Northern lowland plain, mantled and very flat — and the same basin '
         + 'Viking 2 set down in 45 years earlier.',
  }),
  mars_chryse_viking1: mars({
    place: 'Viking 1',
    name: 'CHRYSE PLANITIA — Viking 1 landing site',
    dem: 'mars_chryse_viking1', profile: 'mars_rocky', origin: [-4, 0],
    blurb: 'The first successful Mars landing, 20 July 1976. An outflow-channel '
         + 'plain, and boulder-strewn at exactly the scale a foot cares about.',
  }),
  mars_hellas_basin: mars({
    place: 'Hellas Planitia',
    name: 'HELLAS PLANITIA — the deepest floor on Mars',
    dem: 'mars_hellas_basin', profile: 'mars_sand', origin: [3, 0],
    blurb: 'Seven kilometres below datum, where the atmosphere is thick enough '
         + 'for liquid water to be briefly stable. The floor grade is shallow '
         + 'and never stops.',
  }),
  mars_arsia_mons: mars({
    place: 'Arsia Mons',
    name: 'ARSIA MONS — southern Tharsis Montes',
    dem: 'mars_arsia_mons', profile: 'mars_rocky', origin: [-5, 0],
    blurb: 'A volcanic flank pocked with collapse pits, some of which are '
         + 'openings into lava tubes large enough to lose a town in.',
  }),
  mars_nili_fossae: mars({
    place: 'Nili Fossae',
    name: 'NILI FOSSAE — graben system',
    dem: 'mars_nili_fossae', profile: 'mars_talus', origin: [-5, 0],
    blurb: 'Concentric graben northeast of Isidis, carrying the strongest '
         + 'clay-mineral signature on the planet. Contour walking between fault '
         + 'scarps.',
  }),


  // =========================================================================
  // ISS — no terrain, because there is none. The "place" is a pressurised
  // module, and what makes it a place is its SIZE.
  //
  // Dimensions are the published pressurised-volume figures for each element:
  // length along the module axis and internal diameter, in metres. They are
  // what the interior is for — an enclosure earns its place by giving the eye
  // a known measurement to compare a 1.32 m robot against, and fitting a shell
  // to an arbitrary target length throws exactly that away, which is the
  // mistake the old station asset made.
  //
  // Every module runs the same two microgravity scenarios, because the
  // scenario is about conservation rather than about the room. What changes
  // between them is how much room there is to be wrong in: a push that
  // overshoots by half a metre is a caught handrail in Kibo and a collision in
  // Cupola.
  // =========================================================================
  iss_destiny: iss({
    place: 'Destiny',
    name: 'DESTINY — the US laboratory',
    module: { length: 8.53, diameter: 4.27 },
    blurb: 'The US laboratory, and the module with the most rack-mounted equipment to collide with. Twenty-four rack bays, so the free corridor down the middle is narrower than the shell suggests.',
  }),
  iss_harmony: iss({
    place: 'Harmony',
    name: 'HARMONY — Node 2',
    module: { length: 7.2, diameter: 4.4 },
    blurb: 'The forward node: six berthing ports, so it is the busiest junction on the station and the one where a push in the wrong axis has the most ways to end badly.',
  }),
  iss_columbus: iss({
    place: 'Columbus',
    name: 'COLUMBUS — ESA laboratory',
    module: { length: 6.87, diameter: 4.5 },
    blurb: 'The ESA laboratory. Short and wide, which makes it the hardest module to cross without touching anything: there is less distance to bleed off a bad push in.',
  }),
  iss_kibo: iss({
    place: 'Kibo',
    name: 'KIBO — Japanese Experiment Module',
    module: { length: 11.19, diameter: 4.4 },
    blurb: 'The largest single module on the station, and the one the crews use to demonstrate free flight because it is the only place with room to get a clean run at it.',
  }),
  iss_unity: iss({
    place: 'Unity',
    name: 'UNITY — Node 1',
    module: { length: 5.47, diameter: 4.57 },
    blurb: 'The first US node, and the oldest American element in orbit. Short, with hatches on all six faces.',
  }),
  iss_tranquility: iss({
    place: 'Tranquility',
    name: 'TRANQUILITY — Node 3',
    module: { length: 6.7, diameter: 4.4 },
    blurb: 'Node 3, which carries the exercise equipment and the life support. Where the crew spend the most time deliberately loading their legs.',
  }),
  iss_zvezda: iss({
    place: 'Zvezda',
    name: 'ZVEZDA — service module',
    module: { length: 13.1, diameter: 4.15 },
    blurb: 'The Russian service module and the station\'s original core: crew quarters, galley, and the longest single run on the station.',
  }),
  iss_cupola: iss({
    place: 'Cupola',
    name: 'CUPOLA — the observation module',
    module: { length: 1.5, diameter: 2.95 },
    blurb: 'Seven windows and almost no length at all. There is not enough room here to push off and glide; the whole scenario becomes arriving and stopping.',
  }),
};

/** The places on one body, in the order the picker should show them. */
export const byBody = (body) =>
  Object.entries(SITES).filter(([, s]) => s.body === body).map(([id]) => id);

export const BODIES = ['Moon', 'Mars', 'ISS'];
