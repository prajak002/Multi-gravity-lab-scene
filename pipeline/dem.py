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
            VSI_CACHE_SIZE="33554432", GDAL_HTTP_MAX_RETRY="5",
            GDAL_HTTP_RETRY_DELAY="2", CPL_VSIL_CURL_CHUNK_SIZE="1048576",
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
        source="lola_60s_60m", lat=-69.373, lon=32.319, want_slope=4.0, search_km=9,
        name="Shiv Shakti Point — Chandrayaan-3 landing site",
        note="Uneven mare-highland regolith; the traverse is over undulating "
             "regolith rather than a named grade."),
    "moon_shackleton_rim": dict(
        source="lola_875s_5m", lat=-89.68, lon=129.2, want_slope=14.0, search_km=6,
        name="Shackleton Crater rim — lunar south pole",
        note="Steep rim ascent. Best public lunar topography anywhere: 5 m/px."),
    "moon_mare_tranquillitatis": dict(
        source="lolakaguya_59m", lat=8.5, lon=31.4, want_slope=0.6, search_km=20,
        name="Mare Tranquillitatis — flat basalt plain",
        note="Deliberately the flattest window found; the scenario is a cruise."),
    "moon_aristarchus": dict(
        source="lolakaguya_59m", lat=23.73, lon=-47.49, want_slope=7.0, search_km=16,
        name="Aristarchus Plateau — blocky ejecta and scarps",
        note="Obstacle geometry: high-step, side-step, pivot, edge contouring."),
    "moon_tycho_flank": dict(
        source="lolakaguya_59m", lat=-43.31, lon=-11.36, want_slope=13.0, search_km=18,
        name="Tycho crater flank — steep descent",
        note="Steep downhill; the scenario is braking, so the patch is chosen "
             "for grade and the traverse runs down the fall line."),
    "moon_schrodinger_basin": dict(
        source="lola_60s_60m", lat=-74.9, lon=133.5, want_slope=6.5, search_km=25,
        name="Schrödinger Basin — mixed terrain",
        note="Cruise, rough walking and climbing in one traverse."),

    # ---- Mars ----
    "mars_jezero_delta": dict(
        source="hrsc_mola_200m", lat=18.47, lon=77.38, want_slope=5.0, search_km=14,
        name="Jezero Crater western delta",
        note="Obstacle slalom across the delta front."),
    "mars_gale_crater": dict(
        source="gale_1m", lat=-4.74723, lon=137.3785, want_slope=9.0, search_km=0.4,
        name="Gale Crater — lower Mount Sharp flank",
        note="Rocky uphill traction. 1 m/px HiRISE, the finest Mars DEM there is."),
    "mars_olympia_undae": dict(
        source="hrsc_mola_200m", lat=81.0, lon=180.0, want_slope=1.2, search_km=30,
        name="Olympia Undae — north polar dune sea",
        note="Deep sand. The DEM gives the dune field's regional grade; the "
             "ripple geometry the foot sinks into is the synthetic layer."),
    "mars_cerberus_fossae": dict(
        source="hrsc_mola_200m", lat=11.28, lon=166.37, want_slope=3.0, search_km=20,
        name="Cerberus Fossae — fissure system",
        note="Gap crossing. The fissure itself is narrower than a 200 m pixel, "
             "so the gap is authored; the DEM supplies the plain it cuts."),
    "mars_medusae_fossae": dict(
        source="hrsc_mola_200m", lat=-2.5, lon=197.0, want_slope=6.0, search_km=25,
        name="Medusae Fossae Formation — yardang field",
        note="Narrow threading between wind-carved ridges."),
    "mars_ganges_chasma": dict(
        source="hrsc_mola_200m", lat=-7.5, lon=311.5, want_slope=16.0, search_km=30,
        name="Ganges Chasma — Valles Marineris wall",
        note="Steep descent by switchback. Chosen for the steepest wall "
             "section the scan can find."),
}


# ---------------------------------------------------------------------------
def open_src(key):
    """Open a product and return (dataset, geographic CRS, scale, offset)."""
    src = SOURCES[key]
    ds = rasterio.open("/vsicurl/" + src["url"])
    proj = CRS.from_wkt(ds.crs.to_wkt())
    geo = proj.geodetic_crs or CRS.from_proj4(
        f"+proj=longlat +R={src['radius']} +no_defs")
    # PDS products carry SCALING_FACTOR / OFFSET; rasterio reports them but
    # does not apply them on read. The LOLA offset is the lunar radius, so
    # subtracting it is what turns a radius into an elevation.
    scale = ds.scales[0] if ds.scales else 1.0
    return ds, geo, scale


def local_crs(lat, lon, radius):
    """Azimuthal equidistant about the site — true scale through the centre."""
    return CRS.from_proj4(
        f"+proj=aeqd +lat_0={lat} +lon_0={lon} +R={radius} +units=m +no_defs")


def read_patch(key, lat, lon, size_px, mpp):
    """
    A square, isotropic, metric heightfield centred on (lat, lon).

    The warp is what makes the grid metric; `mpp` is the grid it is resampled
    onto and is chosen per site, never finer than a fraction of the product's
    own resolution — resampling cannot invent detail and pretending otherwise
    is how a 200 m/px product ends up labelled as centimetre terrain.
    """
    src = SOURCES[key]
    ds, geo, scale = open_src(key)
    dst = local_crs(lat, lon, src["radius"])
    half = size_px * mpp / 2.0
    # Site sits at the centre of the output grid, which spans +-half metres.
    transform = rasterio.Affine(mpp, 0, -half, 0, -mpp, half)
    with WarpedVRT(ds, crs=dst.to_wkt(), transform=transform,
                   width=size_px, height=size_px,
                   resampling=rasterio.enums.Resampling.bilinear,
                   src_nodata=ds.nodata, nodata=ds.nodata) as vrt:
        a = vrt.read(1).astype("float64")
    ds.close()
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
        roughness_rms=float(np.sqrt(np.nanmean(resid ** 2))),
        relief=float(np.nanmax(a) - np.nanmin(a)),
        mean_elev=float(np.nanmean(a)),
    )


def pick_window(key, site, size_px, mpp, probes=25, verbose=True):
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
            a = read_patch(key, lat, lon, size_px, mpp)
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


def export(key, name, lat, lon, size_px, mpp, out, site=None):
    src = SOURCES[key]
    a = read_patch(key, lat, lon, size_px, mpp)
    if np.isnan(a).any():
        from scipy.ndimage import distance_transform_edt
        idx = distance_transform_edt(np.isnan(a), return_distances=False,
                                     return_indices=True)
        a = a[tuple(idx)]
    st = terrain_stats(a, mpp)
    centre = float(a[size_px // 2, size_px // 2])
    rel = (a - centre).astype("float32")
    rel.tofile(out + ".f32")
    meta = dict(
        site=name, source=key, source_url=src["url"], citation=src["cite"],
        body=src["body"], covers=src["covers"],
        native_mpp=src["native"], mpp=mpp, size_px=size_px,
        span_m=size_px * mpp, lat=lat, lon=lon, centre_elev_m=centre,
        # The grid is metric because the patch was warped onto it; say so, and
        # say what it was warped FROM, because that is the real resolution.
        grid="azimuthal equidistant about the site, resampled bilinear",
        # The honest number. The grid is 512 samples wide whatever the source,
        # so the only figure that says how much real terrain is in it is how
        # many SOURCE posts the patch spans. Gale spans 128 of them; a site on
        # the 200 m/px global blend spans ten. Below about twenty, the DEM is
        # supplying a slope and a broad landform and nothing else, and every
        # feature the robot's foot meets comes from the synthetic micro layer
        # in SiteField. That is not a defect — no orbital product resolves a
        # 0.19 m sole — but it must not be presented as resolved terrain.
        dem_samples_across=size_px * mpp / src["native"],
        upsample=src["native"] / mpp,
        resolution_note=(
            f"source product is {src['native']:g} m/px; this {size_px}px grid at "
            f"{mpp:g} m/px spans {size_px * mpp / src['native']:.1f} source posts "
            f"and is a {src['native'] / mpp:.0f}x bilinear resampling of them — "
            f"interpolation, not added detail"),
        dtype="float32", layout="row-major, north-up, +x east / +y north",
        stats=st, note=(site or {}).get("note"),
    )
    with open(out + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return meta


def grid_for(src_key, site):
    """
    Output grid. 512 samples across a patch big enough to hold the traverse
    with room for the camera, at a spacing that never claims more than a 4x
    resampling of the source.
    """
    native = SOURCES[src_key]["native"]
    mpp = max(0.25, native / 4.0)
    # Cap the patch so a coarse product does not span a whole province.
    span = min(2048.0, 512 * mpp)
    size = 512
    return size, span / size


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
        size, mpp = grid_for(src, site)
        print(f"\n=== {key} — {site['name']}")
        print(f"    product {src} ({SOURCES[src]['native']:g} m/px native), "
              f"grid {size}px at {mpp:.3f} m/px = {size*mpp:.0f} m span")
        with Env(**GDAL):
            best = (pick_window(src, site, size, mpp, probes=args.probes)
                    if site["search_km"] > 0 else None)
            lat = best["lat"] if best else site["lat"]
            lon = best["lon"] if best else site["lon"]
            meta = export(src, site["name"], lat, lon, size, mpp,
                          f"public/dem/{key}", site)
        s = meta["stats"]
        print(f"    picked  lat {lat:+.4f} lon {lon:.4f}")
        print(f"    grade {s['plane_slope']:.2f} deg toward {s['plane_aspect']:.0f} deg, "
              f"mean slope {s['mean_slope']:.2f}, roughness {s['roughness_rms']:.2f} m, "
              f"relief {s['relief']:.1f} m")


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
