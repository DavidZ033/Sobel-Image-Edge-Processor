
import argparse
import sys
import time
 
import cv2
import numpy as np
 
WINDOW = "Sobel edge detector (left: camera, right: selected stage)"
VIEWS = ["gray", "|Gx|", "|Gy|", "magnitude", "edges"]  # keys 1 to 5
 
 
# ---------------------------------------------------------------------------
# The pipeline. These functions are the design; everything else is I/O.
# ---------------------------------------------------------------------------
 
def rgb_to_gray(frame_bgr):
    """Stage 1. Integer luma: the weights sum to 256, so >> 8 normalizes."""
    f = frame_bgr.astype(np.uint16)            # 255 * 256 needs 16 bits
    b, g, r = f[..., 0], f[..., 1], f[..., 2]  # OpenCV stores pixels as B, G, R
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
 
 
def smooth(a, axis):
    """
    1D smoothing [1 2 1] down the columns (axis 0) or along the rows (axis 1).
    Each output needs a neighbor on both sides, so the result is 2 shorter.
    """
    if axis == 0:
        return a[:-2] + 2 * a[1:-1] + a[2:]
    return a[:, :-2] + 2 * a[:, 1:-1] + a[:, 2:]
 
 
def derivative(a, axis):
    """1D derivative [-1 0 +1], same axes: next value minus previous value."""
    if axis == 0:
        return a[2:] - a[:-2]
    return a[:, 2:] - a[:, :-2]
 
 
def sobel(gray):
    """Stages 2 and 3 for a whole frame at once. Returns signed Gx, Gy."""
    p = gray.astype(np.int16)
    gx = np.zeros_like(p)
    gy = np.zeros_like(p)
    # Gx: smooth down the columns, then the derivative along the rows.
    gx[1:-1, 1:-1] = derivative(smooth(p, axis=0), axis=1)
    # Gy: the derivative down the columns, then smooth along the rows.
    gy[1:-1, 1:-1] = smooth(derivative(p, axis=0), axis=1)
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
      s, d           column pass results for the last 3 columns, oldest
                     first: 3 smoothed values and 3 derivatives stand in for
                     the 9 pixels of a 3x3 window
 
    When pixel (r, c) arrives, the line buffers hold the 2 pixels above it, so
    column c of rows r-2..r is complete. The column pass turns it into one
    smoothed value and one derivative, which shift into s and d. The row pass
    then combines columns c-2, c-1 and c into the result centered on
    (r-1, c-1), so the result trails the input by one row plus one pixel. It
    is valid once r >= 2 and c >= 2, which is what the row and column counters
    gate in RTL.
 
    Too slow for live video in Python (a few frames per second). It exists so
    --selftest can prove that sobel() above is bit-identical to this
    hardware-shaped version.
    """
    h, w = gray.shape
    line1 = [0] * w
    line2 = [0] * w
    s = [0, 0, 0]  # smoothed columns c-2, c-1, c
    d = [0, 0, 0]  # derivative columns c-2, c-1, c
    gx = np.zeros((h, w), np.int16)
    gy = np.zeros((h, w), np.int16)
 
    for r in range(h):
        for c in range(w):
            pixel = int(gray[r, c])
            up, mid, down = line2[c], line1[c], pixel  # column c, rows r-2..r
 
            # Push the incoming pixel down the line buffers.
            line2[c] = mid
            line1[c] = down
 
            # Column pass: one smoothed value and one derivative per pixel.
            s = [s[1], s[2], up + 2 * mid + down]
            d = [d[1], d[2], down - up]
 
            # Row pass, once the 3 columns around (r-1, c-1) are complete.
            if r >= 2 and c >= 2:
                gx[r - 1, c - 1] = s[2] - s[0]             # derivative across
                gy[r - 1, c - 1] = d[0] + 2 * d[1] + d[2]  # smooth across
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
    """Check sobel() against a 3x3 convolution, the streaming model, and widths."""
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
 
    # The reference: the full 3x3 kernels, applied in a single pass by OpenCV.
    kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], np.float32)
    ky = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], np.float32)
    inner = (slice(1, -1), slice(1, -1))  # OpenCV pads the border, this zeroes it
    for name, frame in (("random noise", noise), ("0/255 extremes", extremes)):
        gray = rgb_to_gray(frame)
        gx, gy = sobel(gray)
        sx, sy = sobel_streaming(gray)
        cx = cv2.filter2D(gray, cv2.CV_16S, kx)
        cy = cv2.filter2D(gray, cv2.CV_16S, ky)
        results.append((f"{name}: two 1D passes == one 3x3 convolution",
                        np.array_equal(gx[inner], cx[inner])
                        and np.array_equal(gy[inner], cy[inner])))
        results.append((f"{name}: sobel() == sobel_streaming()",
                        np.array_equal(gx, sx) and np.array_equal(gy, sy)))
 
    # Register widths, measured on the worst-case 0/255 frame.
    p = rgb_to_gray(extremes).astype(np.int16)
    s, d = smooth(p, axis=0), derivative(p, axis=0)
    results.append(("column pass: smooth spans 0..1020 (10-bit), "
                    "derivative -255..255 (9-bit signed)",
                    s.min() == 0 and s.max() == 1020
                    and d.min() == -255 and d.max() == 255))
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
 