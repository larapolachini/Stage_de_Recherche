#!/usr/bin/env python3
"""
annotate_leds.py  – Detect coloured LED dots, draw fat rings, save overlays.

Usage
-----
    python annotate_leds.py <png-file | directory> <output-directory>
"""
import argparse, json
from pathlib import Path
from typing   import List

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
#  Colour prototypes & drawing colours
# ─────────────────────────────────────────────────────────────────────────────
PROTO_BGR = {
    "red"   : np.array([153,153,255]),   # FF9999 → BGR
    "blue"  : np.array([255, 38, 38]),
    "violet": np.array([255, 51,255]),
    "cyan"  : np.array([255,255,179]),
}
DRAW_COLOURS = {
    "red"   : (  0,  0,255),
    "blue"  : (255,  0,  0),
    "violet": (255,  0,255),
    "cyan"  : (255,255,  0),
}

def closest_label(mean_bgr: np.ndarray) -> str:
    return min(PROTO_BGR, key=lambda k: np.sum((PROTO_BGR[k]-mean_bgr)**2))

# ─────────────────────────────────────────────────────────────────────────────
def detect_and_annotate(src: Path,
                        dst: Path,
                        *,
                        min_radius: int = 12,
                        pad_radius: int = 6,
                        thickness: int = 5) -> List[dict]:
    """Detect blobs in *src*, draw coloured rings, write to *dst*."""
    bgr = cv2.imread(str(src))
    if bgr is None:
        raise RuntimeError(f"Could not read {src}")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # broad mask that catches all bright coloured LEDs
    mask = np.zeros_like(hsv[:, :, 0])
    ranges = [((0,   70,70), (10, 255,255)),    # red
              ((160, 70,70), (179,255,255)),    # red (wrap-around)
              ((90,  70,70), (140,255,255))]    # green → blue
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    k     = np.ones((5,5), np.uint8)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    annotated = bgr.copy()
    blobs     = []
    for c in cnts:
        (x, y), r = cv2.minEnclosingCircle(c)
        if r < min_radius:
            continue

        # mean colour inside the contour → robust relabelling
        dot_mask = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(dot_mask, [c], -1, 255, -1)
        mean_bgr = cv2.mean(bgr, mask=dot_mask)[:3]
        label    = closest_label(np.array(mean_bgr))

        centre   = (int(x), int(y))
        cv2.circle(annotated, centre, int(r)+pad_radius,
                   DRAW_COLOURS[label], thickness)

        blobs.append(dict(colour=label, x=int(x), y=int(y), radius=float(r)))

    cv2.imwrite(str(dst), annotated)
    return blobs

# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Detect coloured LED dots and draw circles around them.")
    ap.add_argument("input",  type=Path,
                    help="PNG file or directory containing PNGs")
    ap.add_argument("output", type=Path,
                    help="Directory where annotated images will be written")
    args = ap.parse_args()

    # Prepare output directory
    out_dir: Path = args.output
    out_dir.mkdir(parents=True, exist_ok=True)

    # Build list of PNGs to process
    inp: Path = args.input
    if inp.is_file():
        if inp.suffix.lower() != ".png":
            ap.error("Single-file mode requires a .png image.")
        files = [inp]
    elif inp.is_dir():
        files = sorted(f for f in inp.iterdir() if f.suffix.lower() == ".png")
        if not files:
            ap.error("Directory contains no .png files.")
    else:
        ap.error("Input path is neither a file nor a directory.")

    print(f"Processing {len(files)} file(s)…")

    for src in files:
        dst = out_dir / f"{src.stem}.png"
        dots = detect_and_annotate(src, dst)
        print(f" ✔ {src.name:<30} → {dst.name:<30}  ({len(dots)} blobs)")
        # To also dump coordinates JSON side-cars into out_dir, uncomment:
        # (out_dir / f"{src.stem}.json").write_text(json.dumps(dots, indent=2))

if __name__ == "__main__":
    main()

