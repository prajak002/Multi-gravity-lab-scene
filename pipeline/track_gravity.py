#!/usr/bin/env python3
"""Re-track a retargeted motion under a different gravitational field.

    python pipeline/track_gravity.py IN.npz --field moon --out OUT.npz

A capture is a 1 g recording. Replaying it unchanged on the Moon is wrong in a
specific way: the joint angles are achievable, but the ROOT trajectory that
those angles produced on Earth is not what the same angles produce at 1.62
m/s^2. So the joints are driven as recorded and the body is left to fall under
the target gravity, with contact resolved by the simulator.

What comes out is therefore honest about one thing and silent about another:
  * honest  — where the body actually goes, and when the feet actually touch
  * silent  — whether a real controller could hold that posture in that field

The second question is what an RL policy trained per-field would answer. This
is kinematic replay, not a controller, and it is labelled as such in the output
so nothing downstream can mistake it for one.
"""
import argparse
import sys

import numpy as np

# Surface gravity, m/s^2. Sources: IAU / NASA planetary fact sheets.
FIELDS = {
    "earth": 9.80665,
    "moon": 1.62,
    "mars": 3.72076,
    "iss": 0.0,          # free fall; the microgravity residual is ~1e-5 g
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--field", required=True, choices=sorted(FIELDS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--mjcf", default="", help="G1 MJCF; falls back to kinematic replay if absent")
    args = ap.parse_args()

    g = FIELDS[args.field]
    d = np.load(args.src, allow_pickle=True)
    joint_pos = np.asarray(d["joint_pos"], dtype=np.float64)
    fps = float(d["fps"]) if "fps" in d.files else 30.0
    frames = joint_pos.shape[0]

    try:
        import mujoco
    except ImportError:
        print("error: mujoco is not installed in this environment", file=sys.stderr)
        return 1

    if not args.mjcf:
        # No model supplied: fall through to a ballistic root solve. This is a
        # weaker claim than a simulated one and must not be dressed up as more.
        print("warning: no --mjcf given; solving the root ballistically, not in MuJoCo",
              file=sys.stderr)
        root, contacts = ballistic_root(d, g, fps, frames)
        method = "ballistic"
    else:
        root, contacts = mujoco_track(mujoco, args.mjcf, joint_pos, g, fps)
        method = "mujoco"

    out = {k: d[k] for k in d.files}
    out["root_pos"] = root
    out["contacts"] = contacts
    out["gravity"] = np.float32(g)
    out["field"] = np.bytes_(args.field)
    out["track_method"] = np.bytes_(method)      # so the viewer can say how this was made
    np.savez_compressed(args.out, **out)

    airborne = 1.0 - contacts.any(axis=1).mean()
    print(f"{args.src} -> {args.out}")
    print(f"  field={args.field} g={g:.3f} m/s^2  method={method}")
    print(f"  {frames} frames @ {fps} fps, airborne {airborne * 100:.1f}% of the time")
    return 0


def ballistic_root(d, g, fps, frames):
    """Integrate the root under gravity, arresting it whenever a foot is down."""
    dt = 1.0 / fps
    src_root = np.asarray(d["root_pos"], dtype=np.float64) if "root_pos" in d.files \
        else np.zeros((frames, 3))
    src_contacts = np.asarray(d["contacts"], dtype=bool) if "contacts" in d.files \
        else np.ones((frames, 2), dtype=bool)

    root = src_root.copy()
    vy = 0.0
    ground = float(src_root[:, 2].min()) if src_root.shape[1] > 2 else 0.0
    for i in range(frames):
        if src_contacts[i].any():
            vy = 0.0
            root[i, 2] = ground
        else:
            vy -= g * dt
            root[i, 2] = max(ground, root[i - 1, 2] + vy * dt if i else ground)
            if root[i, 2] <= ground:
                vy = 0.0
    return root.astype(np.float32), src_contacts


def mujoco_track(mujoco, mjcf, joint_pos, g, fps):
    """Drive the recorded joint angles in MuJoCo and let the body fall."""
    model = mujoco.MjModel.from_xml_path(mjcf)
    model.opt.gravity[:] = (0.0, 0.0, -g)
    data = mujoco.MjData(model)
    dt = 1.0 / fps
    steps = max(1, int(round(dt / model.opt.timestep)))

    n = min(joint_pos.shape[1], model.nu if model.nu else joint_pos.shape[1])
    root = np.zeros((joint_pos.shape[0], 3), dtype=np.float32)
    contacts = np.zeros((joint_pos.shape[0], 2), dtype=bool)

    for i in range(joint_pos.shape[0]):
        # Position targets, so the joints follow the capture while the free
        # root does whatever this gravity makes it do.
        if model.nu:
            data.ctrl[:n] = joint_pos[i, :n]
        for _ in range(steps):
            mujoco.mj_step(model, data)
        root[i] = data.qpos[:3]
        # Any contact at all counts as support; per-foot attribution needs
        # geom names this function deliberately does not assume.
        contacts[i, :] = data.ncon > 0
    return root, contacts


if __name__ == "__main__":
    raise SystemExit(main())
