#!/usr/bin/env bash
# build_interior.sh — Corridor.blend -> public/env/iss_corridor.glb
#
# The corridor is a single 43.8 x 7.2 x 4.0 m shell (37,805 triangles) lit in
# the blend by 40 area lamps. Two things are deliberately NOT carried across:
#
#   the lamps      40 real-time lights is not something to hand a browser.
#                  Interior.js rebuilds the two wall rows at a fraction of the
#                  count, from the layout measured here into
#                  public/env/iss_corridor_lights.json.
#   the textures   the materials reference //../../texture/[4k] ... , which is
#                  not next to the blend and is not in this repo. Only base
#                  colours survive, and Interior.js re-materialises the module
#                  anyway so nothing is lost.
#
# The mesh is exported in METRES with no rescale: an interior earns its place
# by giving the scene a known size to compare the robot against, and fitting it
# to an arbitrary target length throws exactly that away.
#
# Usage: tools/build_interior.sh [path/to/Corridor.blend]
set -euo pipefail

BLEND="${1:-$HOME/Downloads/Corridor.blend}"
BLENDER="${BLENDER:-/Applications/Blender.app/Contents/MacOS/Blender}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/public/env/iss_corridor.glb"

[ -f "$BLEND" ] || { echo "no blend at $BLEND" >&2; exit 1; }
[ -x "$BLENDER" ] || { echo "no blender at $BLENDER (set BLENDER=)" >&2; exit 1; }

"$BLENDER" -b "$BLEND" --python "$ROOT/tools/export_interior.py" -- "$OUT"
ls -la "$OUT" "${OUT%.glb}_lights.json"
