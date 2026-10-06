"""
Sobel edge detector: software reference model for an FPGA build.

Runs live on a webcam. Every stage uses only integer adds, subtracts, shifts
and compares, so each step maps directly onto FPGA logic.

Pipeline (per pixel)                                result
  1. RGB to gray   (77*R + 150*G + 29*B) >> 8       8-bit unsigned
  2. 3x3 window    2 line buffers + 9 registers     nine 8-bit pixels
  3. gradients     Gx, Gy from the Sobel kernels    11-bit signed (-1020..1020)
  4. magnitude     |Gx| + |Gy|, clamped to 255      8-bit unsigned (raw max 1530)
  5. threshold     magnitude > T                    1 bit

Window pixel names (row, column) and kernels:

    p00 p01 p02        Gx: -1  0 +1        Gy: -1 -2 -1
    p10 p11 p12            -2  0 +2             0  0  0
    p20 p21 p22            -1  0 +1            +1 +2 +1

The outer 1-pixel border has no complete 3x3 window, so it is output as 0.

Setup:  pip install opencv-python numpy
Run:    python sobel_webcam.py               live webcam
        python sobel_webcam.py --selftest    check the math, no camera needed

Keys (click the video window first):
        1 gray   2 |Gx|   3 |Gy|   4 magnitude   5 edges
        + / -   raise or lower the threshold     q or Esc   quit
"""

import argparse
import sys
import time

import cv2
import numpy as np



WINDOW = "Sobel edge detector (left: camera, right: selected stage)"
VIEWS = ["gray", "|Gx|", "|Gy|", "magnitude", "edges"]  # keys 1 to 5


# ---------------------------------------------------------------------------
# The pipeline. These four functions are the design; everything else is I/O.
# ---------------------------------------------------------------------------

def rgb_to_gray(frame_bgr):
    """Stage 1. Integer luma: the weights sum to 256, so >> 8 normalizes."""
    f = frame_bgr.astype(np.uint16)            # 255 * 256 needs 16 bits
    b, g, r = f[..., 0], f[..., 1], f[..., 2]  # OpenCV stores pixels as B, G, R
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def sobel(gray):
    """Stages 2 and 3 for a whole frame at once. Returns signed Gx, Gy."""
    p = gray.astype(np.int16)
    p00, p01, p02 = p[:-2, :-2], p[:-2, 1:-1], p[:-2, 2:]
    p10, p12 = p[1:-1, :-2], p[1:-1, 2:]
    p20, p21, p22 = p[2:, :-2], p[2:, 1:-1], p[2:, 2:]

    gx = np.zeros_like(p)
    gy = np.zeros_like(p)
    gx[1:-1, 1:-1] = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)  # right - left
    gy[1:-1, 1:-1] = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)  # bottom - top
    return gx, gy


def magnitude(gx, gy):
    """Stage 4. |Gx| + |Gy| stands in for sqrt(Gx^2 + Gy^2): no multipliers."""
    return np.minimum(np.abs(gx) + np.abs(gy), 255).astype(np.uint8)


def threshold(mag, t):
    """Stage 5. One-bit edge decision, shown as 0 (no edge) or 255 (edge)."""
    return np.where(mag > t, 255, 0).astype(np.uint8)


# ---------------------------------------------------------------------------
# The same gradients, shaped like the hardware.
# ---------------------------------------------------------------------------

def sobel_streaming(gray):
    """
    Gx and Gy computed the way the FPGA sees the data: one pixel per clock in
    raster order, with no random access to the frame.

      line1, line2   two line buffers, each one row long (the previous 2 rows)
      win            3x3 shift register; a new column shifts in on every pixel

    When pixel (r, c) arrives, the window is centered on (r-1, c-1), so the
    result trails the input by one row plus one pixel. It is valid once
    r >= 2 and c >= 2, which is what the row and column counters gate in RTL.

    Too slow for live video in Python (a few frames per second). It exists so
    --selftest can prove that sobel() above is bit-identical to this
    hardware-shaped version.
    """
    h, w = gray.shape
    line1 = [0] * w
    line2 = [0] * w
    win = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]  # win[row][col], col 2 is newest
    gx = np.zeros((h, w), np.int16)
    gy = np.zeros((h, w), np.int16)

    for r in range(h):
        for c in range(w):
            pixel = int(gray[r, c])

            # Shift the window one column left and load the incoming column.
            win[0] = [win[0][1], win[0][2], line2[c]]  # row r-2
            win[1] = [win[1][1], win[1][2], line1[c]]  # row r-1
            win[2] = [win[2][1], win[2][2], pixel]     # row r

            # Push the incoming pixel down the line buffers.
            line2[c] = line1[c]
            line1[c] = pixel

            if r >= 2 and c >= 2:
                (p00, p01, p02), (p10, _, p12), (p20, p21, p22) = win
                gx[r - 1, c - 1] = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
                gy[r - 1, c - 1] = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
    return gx, gy


# ---------------------------------------------------------------------------
# Live webcam view.
# ---------------------------------------------------------------------------

def process(frame_bgr, t):
    """Run the full pipeline on one frame and return every stage as 8-bit."""
    gray = rgb_to_gray(frame_bgr)
    gx, gy = sobel(gray)
    mag = magnitude(gx, gy)
    return {
        "gray": gray,
        "|Gx|": np.minimum(np.abs(gx), 255).astype(np.uint8),  # vertical edges
        "|Gy|": np.minimum(np.abs(gy), 255).astype(np.uint8),  # horizontal edges
        "magnitude": mag,
        "edges": threshold(mag, t),
    }


def run_webcam(camera, width, height):
    print(f"Opening camera {camera} (this can take a few seconds)...")
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {camera}. Close other apps that are "
                 "using it, try a different index with --camera, and on macOS "
                 "allow camera access for VS Code in System Settings.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    view, t = "edges", 100
    fps, last = 0.0, time.perf_counter()
    print("Click the video window, then: 1-5 change view, +/- threshold, q quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("No frame from the camera "
                      "(unplugged, busy, or access denied).")
                break
            if frame.shape[1] != width:  # camera ignored the requested size
                scaled_h = round(frame.shape[0] * width / frame.shape[1])
                frame = cv2.resize(frame, (width, scaled_h),
                                   interpolation=cv2.INTER_AREA)

            stage = process(frame, t)[view]

            now = time.perf_counter()
            fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)  # smoothed
            last = now

            # Camera on the left, selected stage on the right, status bar below
            # (kept off the images so no pixels are covered).
            images = np.hstack([frame, cv2.cvtColor(stage, cv2.COLOR_GRAY2BGR)])
            bar = np.zeros((28, images.shape[1], 3), np.uint8)
            status = (f"view: {view}   threshold: {t}   {fps:.0f} fps   |   "
                      "keys: 1-5 view, +/- threshold, q quit")
            cv2.putText(bar, status, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW, np.vstack([images, bar]))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if ord("1") <= key <= ord("5"):
                view = VIEWS[key - ord("1")]
            elif key in (ord("+"), ord("=")):
                t = min(t + 5, 255)
            elif key in (ord("-"), ord("_")):
                t = max(t - 5, 0)
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Self-test: no camera needed.
# ---------------------------------------------------------------------------

def selftest():
    """Check sobel() against the streaming model, OpenCV, and the bit widths."""
    rng = np.random.default_rng(0)
    noise = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    black_or_white = rng.choice(np.array([0, 255], np.uint8), (48, 64, 1))
    extremes = np.repeat(black_or_white, 3, axis=2)  # every pixel is 0 or 255
    results = []

    scalar_gray = np.array(
        [[(77 * int(r) + 150 * int(g) + 29 * int(b)) >> 8 for b, g, r in row]
         for row in noise], dtype=np.uint8)
    results.append(("gray matches the scalar integer formula",
                    np.array_equal(rgb_to_gray(noise), scalar_gray)))

    inner = (slice(1, -1), slice(1, -1))  # OpenCV pads the border, this zeroes it
    for name, frame in (("random noise", noise), ("0/255 extremes", extremes)):
        gray = rgb_to_gray(frame)
        gx, gy = sobel(gray)
        sx, sy = sobel_streaming(gray)
        cx = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
        cy = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3)
        results.append((f"{name}: sobel() == sobel_streaming()",
                        np.array_equal(gx, sx) and np.array_equal(gy, sy)))
        results.append((f"{name}: sobel() == cv2.Sobel on the interior",
                        np.array_equal(gx[inner], cx[inner])
                        and np.array_equal(gy[inner], cy[inner])))

    # Register widths, measured on the worst-case 0/255 frame.
    gx, gy = sobel(rgb_to_gray(extremes))
    results.append(("Gx and Gy span -1020..1020 (11-bit signed)",
                    min(gx.min(), gy.min()) == -1020
                    and max(gx.max(), gy.max()) == 1020))
    results.append(("raw |Gx| + |Gy| peaks at 1530 (11-bit unsigned)",
                    (np.abs(gx) + np.abs(gy)).max() == 1530))

    for text, ok in results:
        print(("PASS  " if ok else "FAIL  ") + text)
    return all(ok for _, ok in results)


def main():
    parser = argparse.ArgumentParser(
        description="Live Sobel edge detector (FPGA reference model).")
    parser.add_argument("--camera", type=int, default=0,
                        help="camera index (default 0)")
    parser.add_argument("--width", type=int, default=640,
                        help="processing width in pixels (default 640)")
    parser.add_argument("--height", type=int, default=480,
                        help="height requested from the camera (default 480)")
    parser.add_argument("--selftest", action="store_true",
                        help="verify the model and exit")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)
    run_webcam(args.camera, args.width, args.height)


if __name__ == "__main__":
    main()