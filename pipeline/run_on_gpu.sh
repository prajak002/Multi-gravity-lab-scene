#!/usr/bin/env bash
#
# Full retarget pipeline, for a machine with an NVIDIA GPU.
#
#   bash pipeline/run_on_gpu.sh VIDEO.mp4 [MOTION_NAME]
#
# Why this exists: every stage below runs fine on Apple Silicon EXCEPT the GEM
# inference itself. The gem/ package carries 71 hardcoded CUDA call sites across
# 17 files, so it is CUDA-only in practice regardless of what the macOS install
# guide implies. Detection, 2D keypoints and the camera solve all completed on
# CPU here; only this one stage needs the GPU.
#
# Everything downstream of the NPZ already works anywhere.
set -euo pipefail

VIDEO="${1:?usage: run_on_gpu.sh VIDEO.mp4 [MOTION_NAME]}"
NAME="${2:-$(basename "${VIDEO%.*}")}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GEMX="$ROOT/vendor/GEM-X"
OUT="$ROOT/pipeline/out"

step() { printf '\n\033[0;32m==>\033[0m %s\n' "$1"; }

# ---------------------------------------------------------------------------
step "0/4  Checking for a usable GPU"
python -c "import torch; assert torch.cuda.is_available(), 'no CUDA device visible'; \
print('  GPU:', torch.cuda.get_device_name(0))" || {
  echo "  This script needs CUDA. On Apple Silicon the GEM stage cannot run —"
  echo "  see the note at the top of this file."; exit 1; }

# ---------------------------------------------------------------------------
step "1/4  GEM-X: video -> SOMA pose -> retargeted joint trajectory"
if [[ ! -d "$GEMX" ]]; then
  git clone --recursive https://github.com/NVlabs/GEM-X.git "$GEMX"
fi
cd "$GEMX"
# NOTE: use the stock upstream checkout on a CUDA box. The local patches in this
# repo's vendor/ (GEM_DISABLE_COREML, GEM_DEVICE) exist only to route around
# Apple Silicon and are unnecessary — and unhelpful — with a real GPU.
python scripts/demo/demo_soma_onnx.py \
    --video "$VIDEO" \
    --output_root "$OUT" \
    --retarget \
    --verbose

# ---------------------------------------------------------------------------
step "2/4  Locating the retargeted trajectory"
NPZ="$(find "$OUT/$NAME" -name '*.npz' -newer "$VIDEO" | head -1)"
[[ -n "$NPZ" ]] || { echo "  no NPZ produced under $OUT/$NAME"; exit 1; }
echo "  $NPZ"

# ---------------------------------------------------------------------------
step "3/4  MuJoCo: re-track the motion in each gravitational field"
cd "$ROOT"
for FIELD in earth moon mars iss; do
  python pipeline/track_gravity.py "$NPZ" \
      --field "$FIELD" \
      --out "pipeline/out/${NAME}_${FIELD}.npz"
done

# ---------------------------------------------------------------------------
step "4/4  Baking for the web arena"
mkdir -p public/motions
for FIELD in earth moon mars iss; do
  python pipeline/bake_web.py \
      "pipeline/out/${NAME}_${FIELD}.npz" \
      "public/motions/g1_${NAME}_${FIELD}.json" \
      --robot g1
done

printf '\n\033[0;32mDone.\033[0m Open the arena with ?motion=retarget\n'
