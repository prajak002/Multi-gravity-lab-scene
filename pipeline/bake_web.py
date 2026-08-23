#!/usr/bin/env python3
"""Bake a retargeted motion NPZ into the compact JSON the arena plays.

    python pipeline/bake_web.py IN.npz OUT.json [--fps 30] [--robot g1]

Input is the KalariSena / GEM-X retarget schema:

    joint_pos        (T, J)  joint angles, radians
    joint_cols       (J,)    joint names, bytes
    root_pos         (T, 3)  metres, Z-up
    root_quat_xyzw   (T, 4)
    contacts         (T, 2)  per-foot, bool
    fps              scalar

Output keeps only the joints the web robot actually has, and stores angles as
a flat Float32 list per frame so the player can index without allocating.
Axis handling stays in the WEB side (the loader already puts URDF Z-up into
three's Y-up); baking a second rotation in here would apply it twice.
"""
import argparse
import json
import sys

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--fps", type=float, default=0.0, help="resample; 0 keeps source rate")
    ap.add_argument("--robot", default="g1")
    args = ap.parse_args()

    d = np.load(args.src, allow_pickle=True)
    for required in ("joint_pos", "joint_cols"):
        if required not in d.files:
            print(f"error: {args.src} has no '{required}' (found: {', '.join(d.files)})", file=sys.stderr)
            return 1

    names = [c.decode() if isinstance(c, bytes) else str(c) for c in d["joint_cols"]]
    angles = np.asarray(d["joint_pos"], dtype=np.float32)
    if angles.shape[1] != len(names):
        print(f"error: {angles.shape[1]} angle columns but {len(names)} names", file=sys.stderr)
        return 1

    src_fps = float(d["fps"]) if "fps" in d.files else 30.0
    frames = angles.shape[0]

    # Resample by index rather than interpolating: joint angles near a limit
    # interpolate into poses the robot cannot hold, and at 30->30 this is a
    # no-op anyway.
    if args.fps and abs(args.fps - src_fps) > 1e-3:
        n = max(2, int(round(frames * args.fps / src_fps)))
        idx = np.clip(np.round(np.linspace(0, frames - 1, n)).astype(int), 0, frames - 1)
        out_fps = args.fps
    else:
        idx = np.arange(frames)
        out_fps = src_fps

    root = np.asarray(d["root_pos"], dtype=np.float32)[idx] if "root_pos" in d.files else None
    quat = np.asarray(d["root_quat_xyzw"], dtype=np.float32)[idx] if "root_quat_xyzw" in d.files else None
    contacts = np.asarray(d["contacts"])[idx].astype(bool) if "contacts" in d.files else None

    payload = {
        "robot": args.robot,
        "source": args.src.split("/")[-1],
        "fps": round(float(out_fps), 4),
        "frames": int(len(idx)),
        "joints": names,
        # radians, frame-major, rounded to a milliradian — well under the
        # repeatability of the hardware and it halves the file
        "angles": [[round(float(v), 4) for v in angles[i]] for i in idx],
    }
    if root is not None:
        payload["root"] = [[round(float(v), 4) for v in root[i]] for i in range(len(idx))]
    if quat is not None:
        payload["quat"] = [[round(float(v), 5) for v in quat[i]] for i in range(len(idx))]
    if contacts is not None:
        payload["contacts"] = [[bool(v) for v in contacts[i]] for i in range(len(idx))]

    with open(args.dst, "w") as fh:
        json.dump(payload, fh, separators=(",", ":"))

    print(f"{args.src} -> {args.dst}")
    print(f"  {payload['frames']} frames @ {payload['fps']} fps, {len(names)} joints")
    if contacts is not None:
        print(f"  contacts: {int(contacts.sum())} foot-frames down of {contacts.size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
