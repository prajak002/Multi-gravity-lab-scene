"""
dem.py — pull real NASA/USGS planetary terrain straight from the published
archives by HTTP range request. Nothing is mirrored; only the pixels a scene
needs are ever read.

Sources are the USGS Astrogeology cloud mosaics and the PDS Geosciences LOLA
gridded data records. Every fetched patch records its source URL, product,
ground sample distance and terrain statistics in a sidecar JSON, so a scene can
always be traced back to the observation it came from.

WHY EVERY PATCH IS REPROJECTED
The published mosaics are simple-cylindrical: a pixel is a fixed number of
projection units, not a fixed number of metres. At Olympia Undae's 81 N a
200-unit pixel is 200 m north-south but only 31 m east-west, so reading a
window straight out of the grid hands the engine a heightfield stretched 6.4x
in one axis — a dune field that leans. The polar products have the opposite
problem: they are stereographic, so scale grows away from the pole.

Both are fixed the same way. Each patch is warped into an azimuthal
equidistant projection centred on the site itself, which is true-to-scale in
every direction through that centre. What comes out is a square, isotropic,
metric heightfield, which is the only thing SiteField can honestly consume.

  sources  list what is available and where it applies
  scan     rank candidate windows near a site by terrain character
  fetch    export one window as a Float32 heightfield + metadata
  site     fetch a named site from the SITES table
  all      fetch every site
"""
import argparse, json, math, sys
import numpy as np
import rasterio
from rasterio.env import Env
from rasterio.vrt import WarpedVRT
from rasterio.windows import Window
from pyproj import CRS, Transformer

S3 = "https://asc-pds-services.s3.us-west-2.amazonaws.com/"
LOLA = ("https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/"
        "lrolol_1xxx/data/lola_gdr/polar/img/")

MOON_R, MARS_R = 1737400.0, 3396190.0

# --- published products ----------------------------------------------------
# `native` is the product's own ground sample distance at the latitude it is
# meant for; it is the honest resolution of the data and is reported as such,
# regardless of the grid the patch is finally resampled onto.
SOURCES = {
    # ---- Mars ----
    "gale_1m": dict(
        url=S3 + "mosaic/Mars/MSL/MSL_Gale_DEM_Mosaic_1m_v3.tif",
        body="Mars", radius=MARS_R, native=1.0,
        covers="Gale Crater / Aeolis Mons only",
        cite="USGS Astrogeology / MSL — HiRISE DEM mosaic of Gale Crater, 1 m/px, v3"),
    "hrsc_mola_200m": dict(
        url=S3 + "mosaic/Mars/HRSC_MOLA_Blend/Mars_HRSC_MOLA_BlendDEM_Global_200mp_v2.tif",
        body="Mars", radius=MARS_R, native=200.0, covers="global",
        cite="ESA Mars Express HRSC + NASA MGS MOLA blended global DEM, 200 m/px, v2"),
    "mola_463m": dict(
        url=S3 + "mosaic/Mars_MGS_MOLA_DEM_mosaic_global_463m.tif",
        body="Mars", radius=MARS_R, native=463.0935415503709, covers="global",
        cite="NASA MGS MOLA global DEM mosaic, 463 m/px"),

    # ---- Moon ----
    "lolakaguya_59m": dict(
        url=S3 + "mosaic/LolaKaguya_Topo/Lunar_LRO_LOLAKaguya_DEMmerge_60N60S_512ppd.tif",
        body="Moon", radius=MOON_R, native=59.225, covers="60 N – 60 S",
        cite="NASA LRO LOLA + JAXA Kaguya TC merged DEM, 512 ppd (~59 m/px)"),
    "lola_118m": dict(
        url=S3 + "mosaic/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014.tif",
        body="Moon", radius=MOON_R, native=118.4505876, covers="global",
        cite="NASA LRO LOLA global LDEM, 118 m/px (Mar 2014)"),
    # PDS3 polar gridded records. Opened through the detached .lbl, which is
    # what carries the projection; the .img beside it is raw int16 and is read
    # by range request.
    "lola_60s_60m": dict(
        url=LOLA + "ldem_60s_60m.lbl", body="Moon", radius=MOON_R, native=60.0,
        covers="60 S – 90 S",
        cite="NASA LRO LOLA polar GDR, south of 60 S, 60 m/px"),
    "lola_75s_30m": dict(
        url=LOLA + "ldem_75s_30m.lbl", body="Moon", radius=MOON_R, native=30.0,
        covers="75 S – 90 S",
        cite="NASA LRO LOLA polar GDR, south of 75 S, 30 m/px"),
    "lola_85s_10m": dict(
        url=LOLA + "ldem_85s_10m.lbl", body="Moon", radius=MOON_R, native=10.0,
        covers="85 S – 90 S",
        cite="NASA LRO LOLA polar GDR, south of 85 S, 10 m/px"),
    "lola_875s_5m": dict(
        url=LOLA + "ldem_875s_5m.lbl", body="Moon", radius=MOON_R, native=5.0,
        covers="87.5 S – 90 S",
        cite="NASA LRO LOLA polar GDR, south of 87.5 S, 5 m/px"),
}

GDAL = dict(GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR", VSI_CACHE="TRUE",
            VSI_CACHE_SIZE="268435456", GDAL_HTTP_MAX_RETRY="5",
            GDAL_HTTP_RETRY_DELAY="2", GDAL_CACHEMAX=512, CPL_VSIL_CURL_CHUNK_SIZE="4194304",
            CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif,.img,.lbl")


# ---------------------------------------------------------------------------
# the twelve surface sites
#
# Coordinates are the published feature locations. `want_slope` is the grade
# the SCENARIO needs, and the scan step searches a neighbourhood around the
# nominal coordinate for a window that actually has it — a downhill-braking
# clip laid on a flat patch demonstrates nothing. `search_km` is how far the
# scan is allowed to wander to find it.
# ---------------------------------------------------------------------------
SITES = {
    # ---- Moon ----
    "moon_shiv_shakti": dict(
        source="lola_60s_60m", lat=-69.373, lon=32.319, want_slope=4.0, traverse="upslope", search_km=9,
        name="Shiv Shakti Point — Chandrayaan-3 landing site",
        note="Uneven mare-highland regolith; the traverse is over undulating "
             "regolith rather than a named grade."),
    "moon_shackleton_rim": dict(
        source="lola_875s_5m", lat=-89.68, lon=129.2, want_slope=14.0, traverse="upslope", search_km=6,
        name="Shackleton Crater rim — lunar south pole",
        note="Steep rim ascent. Best public lunar topography anywhere: 5 m/px."),
    "moon_mare_tranquillitatis": dict(
        source="lolakaguya_59m", lat=8.5, lon=31.4, want_slope=0.6, traverse="contour", search_km=20,
        name="Mare Tranquillitatis — flat basalt plain",
        note="Deliberately the flattest window found; the scenario is a cruise."),
    "moon_aristarchus": dict(
        source="lolakaguya_59m", lat=23.73, lon=-47.49, want_slope=7.0, traverse="contour", search_km=16,
        name="Aristarchus Plateau — blocky ejecta and scarps",
        note="Obstacle geometry: high-step, side-step, pivot, edge contouring."),
    "moon_tycho_flank": dict(
        source="lolakaguya_59m", lat=-43.31, lon=-11.36, want_slope=13.0, traverse="downslope", search_km=18,
        name="Tycho crater flank — steep descent",
        note="Steep downhill; the scenario is braking, so the patch is chosen "
             "for grade and the traverse runs down the fall line."),
    "moon_schrodinger_basin": dict(
        source="lola_60s_60m", lat=-74.9, lon=133.5, want_slope=6.5, traverse="upslope", search_km=25,
        name="Schrödinger Basin — mixed terrain",
        note="Cruise, rough walking and climbing in one traverse."),

    # ---- Mars ----
    "mars_jezero_delta": dict(
        source="hrsc_mola_200m", lat=18.47, lon=77.38, want_slope=5.0, traverse="contour", search_km=14,
        name="Jezero Crater western delta",
        note="Obstacle slalom across the delta front."),
    "mars_gale_crater": dict(
        source="gale_1m", lat=-4.74723, lon=137.3785, want_slope=5.5, traverse="upslope", search_km=2.0,
        name="Gale Crater — lower Mount Sharp flank",
        # The grade wanted here came down from 9 degrees when the patch grew
        # from a 128 m square to 1200 m. Slope is scale-dependent: 9 degrees
        # measured over 128 m of the Mount Sharp flank is a walkable rise, and
        # the same figure over 1200 m is a window whose local relief put the
        # WorldVLA clip's sole 31 mm into the hill with a third of its stance
        # sliding. The search radius grew with it, because a 2 km neighbourhood
        # of a 1200 m patch is the same amount of choice 0.4 km gave a 128 m one.
        note="Rocky uphill traction. 1 m/px HiRISE, the finest Mars DEM there is."),
    "mars_olympia_undae": dict(
        source="hrsc_mola_200m", lat=81.0, lon=180.0, want_slope=1.2, traverse="contour", search_km=30,
        name="Olympia Undae — north polar dune sea",
        note="Deep sand. The DEM gives the dune field's regional grade; the "
             "ripple geometry the foot sinks into is the synthetic layer."),
    "mars_cerberus_fossae": dict(
        source="hrsc_mola_200m", lat=11.28, lon=166.37, want_slope=3.0, traverse="contour", search_km=20,
        name="Cerberus Fossae — fissure system",
        note="Gap crossing. The fissure itself is narrower than a 200 m pixel, "
             "so the gap is authored; the DEM supplies the plain it cuts."),
    "mars_medusae_fossae": dict(
        source="hrsc_mola_200m", lat=-2.5, lon=197.0, want_slope=6.0, traverse="contour", search_km=25,
        name="Medusae Fossae Formation — yardang field",
        note="Narrow threading between wind-carved ridges."),
    "mars_ganges_chasma": dict(
        source="hrsc_mola_200m", lat=-7.5, lon=311.5, want_slope=16.0, traverse="downslope", search_km=30,
        name="Ganges Chasma — Valles Marineris wall",
        note="Steep descent by switchback. Chosen for the steepest wall "
             "section the scan can find."),

    # ---- Moon: the places a mission actually went, or is going -------------
    # Landing sites are the published coordinates of the spacecraft; the scan
    # still searches around them for a window with the grade each scenario
    # needs, so "Apollo 15" means the Hadley terrain, not a claim about the
    # exact square metre of the LM footpads.
    "moon_apollo11_tranquility": dict(
        source="lolakaguya_59m", lat=0.674, lon=23.473, want_slope=0.8,
        traverse="contour", search_km=12,
        name="Apollo 11 — Tranquility Base",
        note="The first walk. Flat mare regolith, which is why it was chosen: "
             "the scenario is a cruise with nothing to negotiate."),
    "moon_apollo15_hadley": dict(
        source="lolakaguya_59m", lat=26.132, lon=3.634, want_slope=9.0,
        traverse="upslope", search_km=16,
        name="Apollo 15 — Hadley Rille and the Apennine Front",
        note="The first rover traverse, up the Apennine Front. Slope walking "
             "beside a 300 m sinuous rille."),
    "moon_apollo17_taurus_littrow": dict(
        source="lolakaguya_59m", lat=20.191, lon=30.772, want_slope=7.5,
        traverse="upslope", search_km=16,
        name="Apollo 17 — Taurus-Littrow valley",
        note="The last walk, and the longest. A deep valley between massifs, "
             "with the light-mantle avalanche deposit on its floor."),
    "moon_copernicus": dict(
        source="lolakaguya_59m", lat=9.62, lon=-20.08, want_slope=17.0,
        traverse="downslope", search_km=30,
        name="Copernicus Crater — terraced wall",
        note="A 93 km young impact crater. The terraced inner wall is the "
             "steepest sustained ground in this set."),
    "moon_marius_hills": dict(
        source="lolakaguya_59m", lat=13.6, lon=-55.0, want_slope=6.0,
        traverse="upslope", search_km=22,
        name="Marius Hills — volcanic domes and a lava-tube skylight",
        note="Volcanic dome field in Oceanus Procellarum, and the most "
             "discussed lava-tube skylight on the Moon."),
    "moon_reiner_gamma": dict(
        source="lolakaguya_59m", lat=7.4, lon=-59.0, want_slope=0.9,
        traverse="contour", search_km=20,
        name="Reiner Gamma — magnetic swirl",
        note="A magnetic anomaly with almost no topographic expression at all: "
             "the albedo swirl is invisible to a DEM, so what a walk here tests "
             "is flat-ground efficiency."),
    "moon_plato": dict(
        source="lolakaguya_59m", lat=51.6, lon=-9.3, want_slope=1.0,
        traverse="contour", search_km=25,
        name="Plato Crater — flooded floor",
        note="A lava-flooded crater floor ringed by massifs; one of the "
             "smoothest large surfaces on the nearside."),
    "moon_tsiolkovskiy": dict(
        source="lolakaguya_59m", lat=-21.2, lon=128.9, want_slope=11.0,
        traverse="downslope", search_km=28,
        name="Tsiolkovskiy Crater — farside central peak",
        note="The farside's most prominent dark-floored crater. Steep ground "
             "off the central peak complex."),
    "moon_malapert_massif": dict(
        source="lola_85s_10m", lat=-86.0, lon=2.7, want_slope=12.0,
        traverse="upslope", search_km=12,
        name="Malapert Massif — Artemis candidate",
        note="A south-polar massif with near-permanent Earth line of sight, "
             "which is what makes it a landing candidate. Steep and lit at a "
             "grazing angle."),
    "moon_de_gerlache_rim": dict(
        source="lola_875s_5m", lat=-88.5, lon=-87.1, want_slope=10.0,
        traverse="upslope", search_km=8,
        name="de Gerlache Crater rim — Artemis candidate",
        note="Polar rim beside permanently shadowed floor, at the finest "
             "lunar topography published anywhere: 5 m/px."),

    # ---- Mars: the places a mission actually went ---------------------------
    "mars_olympus_mons": dict(
        source="hrsc_mola_200m", lat=18.65, lon=-133.8, want_slope=5.0,
        traverse="upslope", search_km=60,
        name="Olympus Mons — the flank",
        note="The largest volcano in the solar system. The flank grade is "
             "gentle and utterly relentless — 5 degrees for hundreds of km."),
    "mars_melas_chasma": dict(
        source="hrsc_mola_200m", lat=-9.8, lon=-76.5, want_slope=20.0,
        traverse="downslope", search_km=40,
        name="Melas Chasma — the deepest wall of Valles Marineris",
        note="The steepest descent available on Mars: 8 km of relief in the "
             "central Valles Marineris trough."),
    "mars_elysium_insight": dict(
        source="hrsc_mola_200m", lat=4.502, lon=135.623, want_slope=0.6,
        traverse="contour", search_km=25,
        name="Elysium Planitia — InSight landing site",
        note="Chosen by NASA precisely for being boring: the flattest, safest "
             "plain they could find. Here that makes it the control."),
    "mars_meridiani_opportunity": dict(
        source="hrsc_mola_200m", lat=-1.95, lon=-5.53, want_slope=0.8,
        traverse="contour", search_km=25,
        name="Meridiani Planum — Opportunity landing site",
        note="Haematite plain. Flat at the DEM's scale, and covered in the "
             "ripple field that the synthetic layer supplies."),
    "mars_gusev_spirit": dict(
        source="hrsc_mola_200m", lat=-14.57, lon=175.47, want_slope=6.0,
        traverse="upslope", search_km=25,
        name="Gusev Crater — Spirit and the Columbia Hills",
        note="Spirit climbed Husband Hill here, which remains the steepest "
             "sustained ascent any Mars rover has driven."),
    "mars_utopia_zhurong": dict(
        source="hrsc_mola_200m", lat=25.066, lon=109.926, want_slope=0.7,
        traverse="contour", search_km=25,
        name="Utopia Planitia — Zhurong landing site",
        note="Northern lowland plain, mantled and very flat."),
    "mars_chryse_viking1": dict(
        source="hrsc_mola_200m", lat=22.48, lon=-47.97, want_slope=1.5,
        traverse="contour", search_km=25,
        name="Chryse Planitia — Viking 1 landing site",
        note="The first successful Mars landing. Outflow-channel plain, "
             "boulder-strewn at the scale a foot cares about."),
    "mars_hellas_basin": dict(
        source="hrsc_mola_200m", lat=-42.4, lon=70.5, want_slope=2.5,
        traverse="downslope", search_km=60,
        name="Hellas Planitia — the deepest floor on Mars",
        note="Seven kilometres below datum, where the atmosphere is thickest. "
             "The floor grade is shallow but never stops."),
    "mars_arsia_mons": dict(
        source="hrsc_mola_200m", lat=-8.35, lon=-120.09, want_slope=8.0,
        traverse="upslope", search_km=40,
        name="Arsia Mons — southern Tharsis Montes",
        note="Volcanic flank with collapse pits; the steepest of the three "
             "Tharsis Montes at this scale."),
    "mars_nili_fossae": dict(
        source="hrsc_mola_200m", lat=22.0, lon=77.0, want_slope=10.0,
        traverse="contour", search_km=30,
        name="Nili Fossae — graben system",
        note="Concentric graben northeast of Isidis, and the strongest "
             "clay-mineral exposure on the planet. Contour walking between "
             "fault scarps."),
}


# ---------------------------------------------------------------------------
# One open dataset per product, reused for every read.
#
# Each probe used to call rasterio.open() on the remote product again, which
# threw away the HTTP connection and GDAL's block cache between probes — and
# the probes of one site are all within a few kilometres of each other, so they
# want the same blocks. Scanning Shackleton with eight probes took 3 min 52 s
# that way. Holding the dataset open turns almost all of that into cache hits.
_OPEN = {}


def open_src(key):
    """Open a product and return (dataset, geographic CRS, scale)."""
    if key in _OPEN:
        return _OPEN[key]
    src = SOURCES[key]
    ds = rasterio.open("/vsicurl/" + src["url"])
    proj = CRS.from_wkt(ds.crs.to_wkt())
    geo = proj.geodetic_crs or CRS.from_proj4(
        f"+proj=longlat +R={src['radius']} +no_defs")
    # PDS products carry SCALING_FACTOR / OFFSET; rasterio reports them but
    # does not apply them on read. The LOLA offset is the lunar radius, so
    # subtracting it is what turns a radius into an elevation.
    scale = ds.scales[0] if ds.scales else 1.0
    _OPEN[key] = (ds, geo, scale)
    return _OPEN[key]


def local_crs(lat, lon, radius):
    """Azimuthal equidistant about the site — true scale through the centre."""
    return CRS.from_proj4(
        f"+proj=aeqd +lat_0={lat} +lon_0={lon} +R={radius} +units=m +no_defs")


def read_patch(key, lat, lon, w_px, h_px, mpp, bearing=90.0):
    """
    A rectangular, isotropic, metric heightfield centred on (lat, lon).

    The warp is what makes the grid metric; `mpp` is the grid it is resampled
    onto and is chosen per site, never finer than a fraction of the product's
    own resolution — resampling cannot invent detail and pretending otherwise
    is how a 200 m/px product ends up labelled as centimetre terrain.

    RECTANGULAR, AND TURNED TO FACE THE WALK

    A traverse is a line, not a disc: the robot spends its whole clip going one
    way and a few metres either side of it. A square patch therefore buys most
    of its pixels for ground nobody visits, and the arena paid for that twice —
    once in download and once in the cap that kept the square small enough to
    be affordable, which held Gale Crater to a 128 m window of the best DEM on
    Mars.

    So the grid is 2:1 and its long axis is turned to point along the
    scenario's traverse. `bearing` is the compass bearing grid +x should face,
    and the rotation is applied in the azimuthal-equidistant frame, which is
    true to scale in every direction through the site — so turning the grid
    costs no fidelity at all, unlike rotating a simple-cylindrical window.

    Everything downstream keeps working in GRID coordinates, where +x is still
    the direction the traverse runs; `grid_bearing_deg` in the sidecar is what
    ties that back to the compass.
    """
    src = SOURCES[key]
    ds, geo, scale = open_src(key)
    dst = local_crs(lat, lon, src["radius"])
    halfw, halfh = w_px * mpp / 2.0, h_px * mpp / 2.0
    # North-up grid: +x east, rows running north to south. Site at the centre.
    base = rasterio.Affine(mpp, 0, -halfw, 0, -mpp, halfh)
    # Turn it so grid +x faces `bearing`. A compass bearing is measured
    # clockwise from north; Affine.rotation is counter-clockwise from +x
    # (east), and east is bearing 90 — hence 90 - bearing.
    transform = rasterio.Affine.rotation(90.0 - bearing) * base
    with WarpedVRT(ds, crs=dst.to_wkt(), transform=transform,
                   width=w_px, height=h_px,
                   resampling=rasterio.enums.Resampling.bilinear,
                   src_nodata=ds.nodata, nodata=ds.nodata) as vrt:
        a = vrt.read(1).astype("float64")
    if ds.nodata is not None:
        a = np.where(a == ds.nodata, np.nan, a)
    a = a * scale
    a = np.where(np.abs(a) > 1e7, np.nan, a)
    return a


def terrain_stats(a, mpp):
    """Slope and roughness of one patch, in engineering units."""
    if not np.isfinite(a).any() or np.isnan(a).mean() > 0.02:
        return None
    gy, gx = np.gradient(a, mpp)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    ny, nx = a.shape
    yy, xx = np.mgrid[0:ny, 0:nx].astype("float64") * mpp
    A = np.column_stack([xx.ravel(), yy.ravel(), np.ones(xx.size)])
    good = ~np.isnan(a.ravel())
    coef, *_ = np.linalg.lstsq(A[good], a.ravel()[good], rcond=None)
    resid = (a.ravel() - A @ coef).reshape(a.shape)
    return dict(
        mean_slope=float(np.nanmean(slope)),
        p95_slope=float(np.nanpercentile(slope, 95)),
        max_slope=float(np.nanmax(slope)),
        plane_slope=float(math.degrees(math.atan(math.hypot(coef[0], coef[1])))),
        # Aspect measured in the OUTPUT grid, where +x is east and +y is NORTH
        # (the affine flips the row direction), so this is a compass-consistent
        # downslope direction the scene builder can align a traverse against.
        plane_aspect=float(math.degrees(math.atan2(-coef[1], coef[0]))),
        # Compass bearing of UPHILL, in the grid the patch was read on. This is
        # what decides which way a rectangular patch is turned: a scenario that
        # says "upslope" wants its long axis pointing here.
        uphill_bearing=float(math.degrees(math.atan2(coef[0], coef[1]))),
        roughness_rms=float(np.sqrt(np.nanmean(resid ** 2))),
        relief=float(np.nanmax(a) - np.nanmin(a)),
        mean_elev=float(np.nanmean(a)),
    )


def pick_window(key, site, scan_px, mpp, probes=25, verbose=True):
    """
    Search a neighbourhood for the window whose grade matches the scenario.

    A golden-angle spiral rather than a lattice: it covers the disc evenly at
    any probe count, so the search can be cut short without leaving a sector
    unvisited.
    """
    src = SOURCES[key]
    want, R_km = site["want_slope"], site["search_km"]
    best = None
    for i in range(probes):
        if i == 0:
            dlat, dlon = 0.0, 0.0
        else:
            t = (i / probes) ** 0.5 * R_km * 1000.0
            a = i * 2.399963229728653
            dx, dy = math.cos(a) * t, math.sin(a) * t
            dlat = math.degrees(dy / src["radius"])
            dlon = math.degrees(dx / (src["radius"] *
                                      max(0.05, math.cos(math.radians(site["lat"])))))
        lat, lon = site["lat"] + dlat, site["lon"] + dlon
        if abs(lat) > 89.99:
            continue
        try:
            a = read_patch(key, lat, lon, scan_px, scan_px, mpp)
        except Exception as e:
            if verbose:
                print(f"    probe {i}: {e}", file=sys.stderr)
            continue
        st = terrain_stats(a, mpp)
        if not st:
            continue
        # Match the grade first; break ties toward more relief, so a site with
        # the right slope and something to walk over wins over a smooth ramp.
        st["score"] = abs(st["plane_slope"] - want) - 0.25 * st["roughness_rms"]
        st.update(lat=lat, lon=lon)
        if verbose:
            print(f"    probe {i:2d}  lat {lat:+9.4f} lon {lon:9.4f}  "
                  f"grade {st['plane_slope']:5.2f}deg (want {want:4.1f})  "
                  f"mean {st['mean_slope']:5.2f}  rough {st['roughness_rms']:6.2f} m  "
                  f"relief {st['relief']:7.1f} m", file=sys.stderr)
        if best is None or st["score"] < best["score"]:
            best = st
    return best


def export(key, name, lat, lon, w_px, h_px, mpp, bearing, out, site=None):
    src = SOURCES[key]
    a = read_patch(key, lat, lon, w_px, h_px, mpp, bearing)
    if np.isnan(a).any():
        from scipy.ndimage import distance_transform_edt
        idx = distance_transform_edt(np.isnan(a), return_distances=False,
                                     return_indices=True)
        a = a[tuple(idx)]
    st = terrain_stats(a, mpp)
    centre = float(a[h_px // 2, w_px // 2])
    rel = (a - centre).astype("float32")
    rel.tofile(out + ".f32")
    posts_long = w_px * mpp / src["native"]
    posts_short = h_px * mpp / src["native"]
    meta = dict(
        site=name, source=key, source_url=src["url"], citation=src["cite"],
        body=src["body"], covers=src["covers"],
        native_mpp=src["native"], mpp=mpp,
        size_px_x=w_px, size_px_y=h_px,
        span_x_m=w_px * mpp, span_y_m=h_px * mpp,
        # The long axis is turned to face the traverse; this is what ties the
        # grid back to the compass. Grid +x runs along it.
        grid_bearing_deg=bearing,
        lat=lat, lon=lon, centre_elev_m=centre,
        # The grid is metric because the patch was warped onto it; say so, and
        # say what it was warped FROM, because that is the real resolution.
        grid=("azimuthal equidistant about the site, resampled bilinear, "
              f"long axis on bearing {bearing:.0f} deg"),
        # The honest number. The grid is a fixed number of samples wide whatever
        # the source, so the only figure that says how much real terrain is in
        # it is how many SOURCE posts the patch spans. Below about twenty, the
        # DEM is supplying a slope and a broad landform and nothing else, and
        # every feature the robot's foot meets comes from the synthetic micro
        # layer in SiteField. That is not a defect — no orbital product resolves
        # a 0.19 m sole — but it must not be presented as resolved terrain.
        dem_samples_across=posts_long,
        dem_samples_short=posts_short,
        upsample=src["native"] / mpp,
        resolution_note=(
            f"source product is {src['native']:g} m/px; this {w_px}x{h_px} grid at "
            f"{mpp:g} m/px spans {posts_long:.0f} x {posts_short:.0f} source posts "
            + (f"and is a {src['native'] / mpp:.0f}x bilinear resampling of them — "
               f"interpolation, not added detail"
               if src["native"] > mpp else
               f"and is sampled at {mpp / src['native']:.2f}x the source spacing — "
               f"no interpolated detail is claimed")),
        dtype="float32", layout="row-major, +x along the traverse, +y to its left",
        stats=st, note=(site or {}).get("note"),
    )
    with open(out + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return meta


# Output grid. 2:1, long axis along the traverse.
GRID_W, GRID_H = 1024, 512
# How far the long axis is allowed to reach. The floor keeps a fine product
# from being cropped to a courtyard — Gale's 1 m/px DEM used to be read as a
# 128 m square, which is 25 robot-lengths of the best topography on Mars. The
# ceiling keeps a coarse one from spanning a province.
SPAN_MIN, SPAN_MAX = 1200.0, 4096.0
# The scan window is the export patch's SHORT side, and that is not a detail.
#
# It was briefly 256 px, on the reasoning that the scan is only judging the
# character of a neighbourhood and does not need the export grid to do it. That
# is wrong: grade and roughness are scale-dependent, so a 320 m window and a
# 640 m one rank candidate sites differently. Measured on the Shackleton rim,
# the smaller scan chose a window with 19.4 m of roughness where the larger one
# chose 11.3, and the retargeter's sole penetration on that scene went from
# -4.9 mm to -32.6 mm with 25 % slip. The window a scan picks IS the site, so
# the statistics it picks on have to be the statistics of the patch.


def grid_for(src_key):
    """
    The grid one site is read onto: (width, height, metres per pixel).

    `mpp` starts at a quarter of the product's own ground sample distance,
    which is the most resampling this tool is willing to call terrain, and the
    long span follows from the pixel count. Clamping the span then sets the
    real mpp — so a coarse global product is resampled harder in exchange for
    covering ground a traverse can actually use, and that multiple is reported
    in every sidecar rather than buried.
    """
    native = SOURCES[src_key]["native"]
    span = min(SPAN_MAX, max(SPAN_MIN, GRID_W * max(0.25, native / 4.0)))
    return GRID_W, GRID_H, span / GRID_W


def traverse_bearing(site, stats):
    """
    Which way the long axis faces.

    Declared by the SCENARIO, resolved against the window that was actually
    picked: "upslope" on a rim means something different once the scan has
    chosen where on the rim to stand. Everything downstream then works in grid
    coordinates with +x along the walk, which is also how build_scene.mjs
    already reads a heading.
    """
    want = site.get("traverse", "upslope")
    if isinstance(want, (int, float)):
        return float(want) % 360.0
    up = stats["uphill_bearing"] if stats else 0.0
    return {"upslope": up, "downslope": up + 180.0, "contour": up + 90.0}[want] % 360.0


def cmd_sources(args):
    for k, s in SOURCES.items():
        print(f"{k:18s} {s['body']:5s} {s['native']:9.3f} m/px   {s['covers']:20s}  {s['cite']}")


def cmd_site(args):
    names = args.name or list(SITES)
    unknown = [n for n in names if n not in SITES]
    if unknown:
        sys.exit("unknown site(s): " + ", ".join(unknown)
                 + "\nknown: " + ", ".join(SITES))
    for key in names:
        site = SITES[key]
        src = site["source"]
        w, h, mpp = grid_for(src)
        print(f"\n=== {key} — {site['name']}")
        print(f"    product {src} ({SOURCES[src]['native']:g} m/px native), "
              f"grid {w}x{h}px at {mpp:.3f} m/px = {w*mpp:.0f} x {h*mpp:.0f} m")
        with Env(**GDAL):
            # The scan uses a SQUARE window on the short dimension: it is
            # looking for the character of a neighbourhood, and it has to run
            # before the traverse bearing is known, because the bearing is
            # derived from the slope of whichever window it settles on.
            best = (pick_window(src, site, h, mpp, probes=args.probes)
                    if site["search_km"] > 0 else None)
            lat = best["lat"] if best else site["lat"]
            lon = best["lon"] if best else site["lon"]
            if best is None:
                with Env(**GDAL):
                    best = terrain_stats(read_patch(src, lat, lon, h, h, mpp), mpp)
            bearing = traverse_bearing(site, best)
            meta = export(src, site["name"], lat, lon, w, h, mpp, bearing,
                          f"public/dem/{key}", site)
        s = meta["stats"]
        print(f"    picked  lat {lat:+.4f} lon {lon:.4f}, "
              f"traverse on bearing {bearing:.0f} deg ({site.get('traverse', 'upslope')})")
        print(f"    grade {s['plane_slope']:.2f} deg toward {s['plane_aspect']:.0f} deg, "
              f"mean slope {s['mean_slope']:.2f}, roughness {s['roughness_rms']:.2f} m, "
              f"relief {s['relief']:.1f} m, "
              f"{meta['dem_samples_across']:.0f} x {meta['dem_samples_short']:.0f} source posts")


p = argparse.ArgumentParser(description=__doc__,
                            formatter_class=argparse.RawDescriptionHelpFormatter)
sub = p.add_subparsers(dest="cmd", required=True)

c = sub.add_parser("sources", help="list available products")
c.set_defaults(func=cmd_sources)

c = sub.add_parser("site", help="fetch named site(s) from the SITES table")
# No argparse `choices` here: with nargs="*" it validates the empty default
# too and refuses to run with no arguments, which is the "fetch everything"
# case. Validate by hand instead.
c.add_argument("name", nargs="*", help="site id(s); omit for all")
c.add_argument("--probes", type=int, default=25)
c.set_defaults(func=cmd_site)

a = p.parse_args()
a.func(a)
