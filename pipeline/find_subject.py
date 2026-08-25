#!/usr/bin/env python3
"""Find stretches of footage where a person is big enough to pose-fit.

    python pipeline/find_subject.py VIDEO [--from S] [--to S] [--every S]

Pose extraction needs a subject that is large, unoccluded and present for
several consecutive seconds. Most archival reels are not that: they are wide
vistas, cutaways and titles. Running a pose model over a whole reel and hoping
wastes hours and produces confident nonsense on whatever it does find — which
is exactly what happened to this project's first attempt, where ViTPose fitted
a skeleton to the distributor's title card and every downstream stage inherited
it.

So this scans FIRST and reports what is actually there, in engineering terms:
how tall the person is as a fraction of frame height, and how long they stay.
Below roughly 0.25 of frame height there are not enough pixels on a limb for
2D keypoints to mean anything.
"""
import argparse, json, sys
import numpy as np
import torch, torchvision
import imageio.v2 as iio

MIN_SCORE = 0.85
MIN_REL_H = 0.25          # person height as a fraction of frame height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=0.0)
    ap.add_argument("--every", type=float, default=1.0, help="seconds between samples")
    ap.add_argument("--out", default="pipeline/out/subject_scan.json")
    args = ap.parse_args()

    rdr = iio.get_reader(args.video, "ffmpeg")
    md = rdr.get_meta_data()
    fps, (W, H) = md["fps"], md["size"]
    t1 = args.t1 or md["duration"]

    weights = torchvision.models.detection.KeypointRCNN_ResNet50_FPN_Weights.DEFAULT
    model = torchvision.models.detection.keypointrcnn_resnet50_fpn(weights=weights)
    model.eval()
    torch.set_num_threads(max(1, torch.get_num_threads()))

    hits = []
    t = args.t0
    n = 0
    while t < t1:
        f = int(round(t * fps))
        try:
            img = rdr.get_data(f)
        except Exception:
            break
        x = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        with torch.no_grad():
            out = model([x])[0]
        keep = out["scores"] > MIN_SCORE
        boxes = out["boxes"][keep].numpy()
        scores = out["scores"][keep].numpy()
        best = None
        for b, s in zip(boxes, scores):
            rel_h = (b[3] - b[1]) / H
            if rel_h >= MIN_REL_H and (best is None or rel_h > best[1]):
                best = (float(s), float(rel_h), [float(v) for v in b])
        if best:
            hits.append({"t": round(t, 2), "frame": f, "score": round(best[0], 3),
                         "rel_h": round(best[1], 3), "box": [round(v, 1) for v in best[2]]})
            print(f"  t={t:7.1f}s  frame {f:6d}  score {best[0]:.2f}  "
                  f"person is {best[1]*100:.0f}% of frame height", flush=True)
        n += 1
        t += args.every
    rdr.close()

    # Group hits into runs of consecutive samples — a usable clip is a RUN,
    # not a scattering of lucky frames.
    runs, cur = [], []
    for h in hits:
        if cur and h["t"] - cur[-1]["t"] <= args.every * 1.6:
            cur.append(h)
        else:
            if len(cur) >= 3: runs.append(cur)
            cur = [h]
    if len(cur) >= 3: runs.append(cur)

    print(f"\nsampled {n} frames, {len(hits)} with a usable-size person, "
          f"{len(runs)} runs of 3+ consecutive samples")
    runs.sort(key=lambda r: -(r[-1]["t"] - r[0]["t"]))
    for r in runs[:10]:
        print(f"  RUN {r[0]['t']:7.1f}s -> {r[-1]['t']:7.1f}s "
              f"({r[-1]['t']-r[0]['t']:5.1f}s)  max size {max(x['rel_h'] for x in r)*100:.0f}%")
    json.dump({"video": args.video, "fps": fps, "size": [W, H],
               "hits": hits, "runs": [[x["t"] for x in r] for r in runs]},
              open(args.out, "w"))
    print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
