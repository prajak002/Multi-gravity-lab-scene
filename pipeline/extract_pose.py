#!/usr/bin/env python3
"""Extract COCO-17 2D keypoints for one segment of footage.

    python pipeline/extract_pose.py VIDEO --from S --to S --out out.json

Deliberately NOT a whole-reel run. This project's first attempt pointed a pose
model at a 20-minute reel, took whatever it found in the first four seconds —
the distributor's title card — and fitted a confident skeleton to a slide of
text. Every stage downstream inherited it. So a segment is chosen first, by
pipeline/find_subject.py, and only then are keypoints extracted from it.

Tracks the SAME person across the segment by picking, each frame, the
detection whose box best overlaps the previous one. Archival footage cuts
between shots without warning, and a detector that silently switches subject
mid-sequence produces a "gait" that is two different people.
"""
import argparse, json, sys
import numpy as np
import torch, torchvision
import imageio.v2 as iio

COCO = ['nose','eyeL','eyeR','earL','earR','shoulderL','shoulderR','elbowL','elbowR',
        'wristL','wristR','hipL','hipR','kneeL','kneeR','ankleL','ankleR']


def iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    if x1 <= x0 or y1 <= y0:
        return 0.0
    inter = (x1 - x0) * (y1 - y0)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / max(ua, 1e-6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--from", dest="t0", type=float, required=True)
    ap.add_argument("--to", dest="t1", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-score", type=float, default=0.80)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    rdr = iio.get_reader(args.video, "ffmpeg")
    md = rdr.get_meta_data(); fps = md["fps"]; W, H = md["size"]

    weights = torchvision.models.detection.KeypointRCNN_ResNet50_FPN_Weights.DEFAULT
    model = torchvision.models.detection.keypointrcnn_resnet50_fpn(weights=weights)
    model.eval()

    f0, f1 = int(round(args.t0 * fps)), int(round(args.t1 * fps))
    kps, boxes, kept, prev = [], [], [], None
    for f in range(f0, f1 + 1):
        try:
            img = rdr.get_data(f)
        except Exception:
            break
        x = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        with torch.no_grad():
            out = model([x])[0]
        sel, best = None, -1.0
        for i, s in enumerate(out["scores"]):
            if float(s) < args.min_score:
                continue
            b = out["boxes"][i].numpy()
            # Continuity first, size second. Without the overlap term the
            # detector happily jumps to whoever is largest in the frame.
            score = float(s) + (2.0 * iou(prev, b) if prev is not None else 0.0)
            if score > best:
                best, sel, selb = score, i, b
        if sel is None:
            continue
        prev = selb
        k = out["keypoints"][sel].numpy()           # (17, 3) x, y, visibility
        sc = out["keypoints_scores"][sel].numpy()   # (17,)  per-keypoint logit
        conf = 1.0 / (1.0 + np.exp(-sc))            # -> 0..1
        kps.append([[round(float(k[j][0]), 2), round(float(k[j][1]), 2),
                     round(float(conf[j]), 3)] for j in range(17)])
        boxes.append([round(float(v), 1) for v in selb])
        kept.append(f)
    rdr.close()

    if not kps:
        print("no frames with a confident detection", file=sys.stderr)
        return 1
    out = {"source": args.label or f"{args.video} {args.t0:.1f}-{args.t1:.1f}s",
           "video": args.video, "fps": fps, "size": [W, H],
           "t0": args.t0, "t1": args.t1, "frames": len(kps),
           "frame_index": kept, "names": COCO, "kp": kps, "bbox": boxes}
    json.dump(out, open(args.out, "w"))
    a = np.array(kps)
    print(f"{len(kps)} frames ({f0}..{f1}), mean keypoint confidence {a[:,:,2].mean():.3f}")
    h = np.array(boxes)[:, 3] - np.array(boxes)[:, 1]
    print(f"subject height {h.min():.0f}..{h.max():.0f} px of {H}  "
          f"({h.mean()/H*100:.0f}% of frame)")
    for nm in ("hipL", "kneeL", "ankleL", "shoulderL"):
        j = COCO.index(nm)
        print(f"  {nm:10s} mean conf {a[:, j, 2].mean():.3f}")
    print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
