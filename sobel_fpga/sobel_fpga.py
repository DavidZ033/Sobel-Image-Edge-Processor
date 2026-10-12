"""
Sobel edge detector: laptop side of the FPGA build (Digilent Cmod S7).

Grabs frames from the webcam, sends each one to the FPGA over the board's USB
serial port, and shows what comes back next to the camera picture. Every
result is also compared with sobel_webcam.py, the software model the FPGA
design was built from, so even one wrong pixel shows up in the status bar.

Setup:  pip install opencv-python numpy pyserial
        Keep sobel_webcam.py in the same folder as this file.
        Load the design onto the board first (see fpga/build.tcl).

Run:    python sobel_fpga.py --selftest    check the link and the math, no camera needed
        python sobel_fpga.py               live webcam through the FPGA

Keys (click the video window first):
        1 gray   2 |Gx|   3 |Gy|   4 magnitude   5 edges
        + / -   raise or lower the threshold
        c       who turns color into gray: the laptop (fast) or the FPGA
        q or Esc   quit

Speed. The serial link carries baud / 10 bytes per second each way and a gray
frame is width x height bytes, so a frame takes 10 x width x height / baud
seconds to cross it. The script waits for each reply before it sends the next
frame, and the board's USB chip holds on to the last few bytes of a reply for
up to 16 ms, which costs small frames a little more. Roughly:

        baud          160x120    320x240    640x480
         3,000,000    12 fps      4 fps      1 fps
        12,000,000    30 fps     12 fps      4 fps     (30 = camera limit)

--baud here must match the BAUD the FPGA design was built with.
"""

import argparse
import sys
import threading
import time

import cv2
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing. Install it with:  pip install pyserial")

try:
    import sobel_webcam as model
except ImportError:
    sys.exit("Put sobel_webcam.py in the same folder as this script. It is the "
             "software model the FPGA results are checked against.")

VIEWS = model.VIEWS                 # ["gray", "|Gx|", "|Gy|", "magnitude", "edges"]
WINDOW = "Sobel on the FPGA (left: camera, right: from the FPGA)"

MAX_WIDTH = 2048                    # MAX_WIDTH the FPGA design was built with
FPGA_TIMEOUT_S = 0.2                # TIMEOUT_MS the FPGA design was built with
FTDI_FT2232 = (0x0403, 0x6010)      # USB IDs of the serial chip on the Cmod S7


# ---------------------------------------------------------------------------
# The wire format. The FPGA's side of this is fpga/rtl/frame_rx.v.
# ---------------------------------------------------------------------------

def make_packet(pixels, view, t):
    """
    One frame, ready to send.

    pixels is height x width (gray, 1 byte per pixel) or height x width x 3
    (color in OpenCV's B, G, R order, 3 bytes per pixel, in which case the
    FPGA does the grayscale step itself). view is 0 to 4, t is the threshold.
    """
    if (pixels.dtype != np.uint8 or pixels.ndim not in (2, 3)
            or (pixels.ndim == 3 and pixels.shape[2] != 3)):
        raise ValueError("pixels must be 8-bit, shaped height x width (gray) "
                         "or height x width x 3 (color)")
    if not (0 <= view < len(VIEWS) and 0 <= t <= 255):
        raise ValueError(f"view must be 0 to {len(VIEWS) - 1} and the threshold 0 to 255")
    h, w = pixels.shape[:2]
    if not (1 <= w <= MAX_WIDTH and 1 <= h <= 0xFFFF):
        raise ValueError(f"frame size {w}x{h} is outside what the FPGA accepts "
                         f"(width 1 to {MAX_WIDTH}, height 1 to 65535)")
    color = 0x80 if pixels.ndim == 3 else 0x00
    settings = bytes([color | view, t, w & 0xFF, w >> 8, h & 0xFF, h >> 8])
    check = 0
    for value in settings:
        check ^= value
    return (b"\xA5\x5A" + settings + bytes([check])
            + np.ascontiguousarray(pixels).tobytes())


def unpack_reply(raw, width, height, view):
    """
    Turn the width x height bytes the FPGA sent back into a picture.

    The gray view comes back as is. For the other views the FPGA answers
    each pixel (r, c) with the result for the 3x3 window that pixel
    completes, which is centered on (r-1, c-1). So the picture arrives
    shifted down and right by one pixel, and is rolled back up and left here.

    The first row and first column the FPGA sends (always 0, no window is
    complete yet) wrap around to the bottom and right edges. Those are border
    pixels, which are 0 in the software model too, so comparing the two
    pictures really does check every byte that came back.
    """
    stream = np.frombuffer(raw, np.uint8).reshape(height, width)
    if view == 0:
        return stream.copy()
    return np.roll(stream, (-1, -1), axis=(0, 1))


# ---------------------------------------------------------------------------
# The serial link.
# ---------------------------------------------------------------------------

class LinkError(Exception):
    """The FPGA did not answer as expected."""


class FpgaLink:
    """
    The serial connection to the FPGA.

    process() sends one frame and returns the FPGA's answer. The FPGA starts
    answering while the frame is still being sent, so a background thread
    reads the port the whole time; otherwise the reply would pile up in the
    serial driver and overflow it.
    """

    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.bytes_per_s = baud / 10          # start bit + 8 data bits + stop bit
        # write_timeout: a write that gets stuck gives up instead of hanging forever.
        self._ser = serial.Serial(port, baud, timeout=0.05, write_timeout=120)
        try:
            self._ser.set_buffer_size(rx_size=1 << 20)    # roomier driver buffer (Windows)
        except Exception:
            pass
        self._received = bytearray()
        self._arrived = threading.Condition()
        self._problem = None
        self._open = True
        threading.Thread(target=self._read_forever, daemon=True).start()

    def _read_forever(self):
        while self._open:
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except Exception as error:        # unplugged, or the port was closed
                with self._arrived:
                    if self._open:
                        self._problem = error
                    self._arrived.notify_all()
                return
            if chunk:
                with self._arrived:
                    self._received += chunk
                    self._arrived.notify_all()

    def resync(self):
        """
        Get both ends back to a known state: nothing on the wire, and the FPGA
        waiting for the start of a frame. The FPGA gives up on a half-received
        frame after FPGA_TIMEOUT_S of silence, so silence is all it takes.
        """
        try:
            self._ser.reset_output_buffer()
        except Exception:
            pass
        quiet_since = time.monotonic()
        give_up = quiet_since + 5.0
        while time.monotonic() - quiet_since < FPGA_TIMEOUT_S + 0.15:
            time.sleep(0.05)
            with self._arrived:
                if self._problem is not None:
                    raise LinkError(f"lost the serial port ({self._problem})")
                if self._received:            # the tail of an old reply is still coming in
                    self._received.clear()
                    quiet_since = time.monotonic()
            if time.monotonic() > give_up:
                raise LinkError("the port never goes quiet")

    def process(self, pixels, view, t):
        """
        Send one frame and return the FPGA's answer as a picture.

        Raises LinkError if the answer is not complete in time. After that,
        call resync() before trying again.
        """
        packet = make_packet(pixels, view, t)
        h, w = pixels.shape[:2]
        with self._arrived:
            self._received.clear()            # nothing should be left from earlier
        try:
            self._ser.write(packet)
        except Exception as error:
            raise LinkError(f"could not write to the serial port ({error})") from None

        # The reply is finished about when the frame has gone out. Allow
        # twice the time that takes, plus a second for the PC to be busy.
        deadline = time.monotonic() + 1.0 + 2 * len(packet) / self.bytes_per_s
        with self._arrived:
            while len(self._received) < w * h:
                if self._problem is not None:
                    raise LinkError(f"lost the serial port ({self._problem})")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LinkError(f"expected {w * h} bytes back, "
                                    f"got {len(self._received)}")
                self._arrived.wait(remaining)
            raw = bytes(self._received[:w * h])
            del self._received[:w * h]
        return unpack_reply(raw, w, h, view)

    def close(self):
        self._open = False
        try:
            self._ser.close()
        except Exception:
            pass


def answers_like_the_fpga(link):
    """Send a tiny frame in the gray view; the design returns it unchanged."""
    probe = np.random.default_rng(7).integers(0, 256, (2, 16), dtype=np.uint8)
    try:
        link.resync()
        return np.array_equal(link.process(probe, 0, 0), probe)
    except LinkError:
        return False


def connect(port, baud):
    """Open the given port, or find the board by trying the likely ones."""
    ports = list(list_ports.comports())
    if port:
        candidates = [port]
    else:
        ftdi = [p for p in ports if (p.vid, p.pid) == FTDI_FT2232]
        # The chip has two channels: A is the programming cable (JTAG), B is
        # the serial port. Try B first; how it is labeled depends on the OS.
        ftdi.sort(key=lambda p: not ((p.location or "").endswith(".1")
                                     or (p.serial_number or "").upper().endswith("B")))
        candidates = [p.device for p in ftdi]

    tried = []
    for device in candidates:
        try:
            link = FpgaLink(device, baud)
        except (serial.SerialException, OSError, ValueError) as error:
            tried.append(f"  {device}: could not open ({error})")
            continue
        if answers_like_the_fpga(link):
            return link
        link.close()
        tried.append(f"  {device}: opened, but no valid answer at {baud} baud")

    sys.exit("Could not reach the FPGA.\n"
             "Tried:\n"
             + ("\n".join(tried) or "  nothing: no port looks like the Cmod S7's "
                                    "USB serial port") + "\n"
             "Serial ports on this computer:\n"
             + ("\n".join(f"  {p.device}: {p.description}" for p in ports)
                or "  none") + "\n"
             "Things to check:\n"
             "  * the board is plugged in and the design is loaded (LED0 blinks)\n"
             "  * --baud matches the BAUD the design was built with\n"
             "  * no other program has the port open (serial terminal, another "
             "copy of this script)\n"
             "  * or name the port yourself, for example --port COM5\n"
             "On the board, LED1 glows when bytes reach the FPGA and LED3 lights "
             "when they arrive garbled, which usually means a baud rate mismatch.")


# ---------------------------------------------------------------------------
# Self-test: no camera needed.
# ---------------------------------------------------------------------------

def selftest(link):
    """Push known frames through the FPGA and compare with the software model."""
    rng = np.random.default_rng(0)
    passed = []

    def report(text, test):
        """Run one test. If the link fails in the middle, that test fails."""
        try:
            ok = bool(test())
        except LinkError as error:
            ok = False
            text += f" (link failure: {error})"
            try:
                link.resync()
            except LinkError:
                pass
        passed.append(ok)
        print(("PASS  " if ok else "FAIL  ") + text, flush=True)

    def views_match(frame_bgr, t, color):
        """True if all five views of this frame come back bit-identical."""
        expected = model.process(frame_bgr, t)
        pixels = frame_bgr if color else model.rgb_to_gray(frame_bgr)
        return all(np.array_equal(link.process(pixels, v, t), expected[name])
                   for v, name in enumerate(VIEWS))

    every_value = np.arange(256, dtype=np.uint8).reshape(16, 16)
    report("gray view returns all 256 pixel values unchanged",
           lambda: np.array_equal(link.process(every_value, 0, 0), every_value))

    noise = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    black_or_white = rng.choice(np.array([0, 255], np.uint8), (48, 64, 1))
    extremes = np.repeat(black_or_white, 3, axis=2)  # every pixel is 0 or 255
    for name, frame in (("random noise", noise), ("0/255 extremes", extremes)):
        for color in (False, True):
            report(f"{name}, {'color' if color else 'gray'} pixels sent: "
                   "all 5 views match the software model",
                   lambda frame=frame, color=color: views_match(frame, 100, color))

    gray = model.rgb_to_gray(noise)
    report("edges view matches at thresholds 0, 1, 127, 254 and 255",
           lambda: all(np.array_equal(link.process(gray, 4, t),
                                      model.process(noise, t)["edges"])
                       for t in (0, 1, 127, 254, 255)))

    sizes = [(1, 1), (2, 2), (3, 3), (5, 2), (2, 5), (7, 3), (333, 5), (MAX_WIDTH, 3)]
    report(f"frame sizes from 1x1 up to the widest row ({MAX_WIDTH}) match",
           lambda: all(views_match(rng.integers(0, 256, (h, w, 3), dtype=np.uint8),
                                   40, color=False)
                       for w, h in sizes))

    # Many frames in a row, the way the live view works. This is the test
    # that catches bytes getting lost at high baud rates.
    frames = [rng.integers(0, 256, (120, 160, 3), dtype=np.uint8) for _ in range(40)]
    start = time.perf_counter()
    report(f"{len(frames)} frames of 160x120 in a row all match",
           lambda: all(np.array_equal(link.process(model.rgb_to_gray(frame), 3, 0),
                                      model.process(frame, 0)["magnitude"])
                       for frame in frames))
    seconds = time.perf_counter() - start

    print()
    if all(passed):
        print("The FPGA matches the software model bit for bit "
              f"({len(frames) / seconds:.1f} frames per second at 160x120).")
    else:
        print("Something is off. If a test reports fewer bytes back than expected "
              "at a high baud rate, bytes are getting lost on the way: rebuild "
              "the FPGA design at a lower BAUD.")
    return all(passed)


# ---------------------------------------------------------------------------
# Live webcam view.
# ---------------------------------------------------------------------------

class NewestFrame:
    """
    Reads the camera in the background and hands out only the newest frame.

    The link is usually slower than the camera. Reading the camera only when
    the link is ready would return frames that have been waiting in the camera
    driver's queue, and the picture would lag behind by a second or more.
    """

    def __init__(self, cap):
        self._cap = cap
        self._frame = None
        self._ok = True
        self._fresh = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._read_forever, daemon=True)
        self._thread.start()

    def _read_forever(self):
        while self._running:
            ok, frame = self._cap.read()
            with self._fresh:
                self._ok, self._frame = ok, frame
                self._fresh.notify_all()
            if not ok:
                return

    def read(self, patience=15.0):
        """The newest frame not handed out yet. Waits for one, but not forever."""
        give_up = time.monotonic() + patience
        with self._fresh:
            while self._frame is None and self._ok and time.monotonic() < give_up:
                self._fresh.wait(0.5)
            frame, self._frame = self._frame, None
            return frame is not None, frame

    def release(self):
        self._running = False
        self._thread.join(2.0)
        self._cap.release()


def window_is_open():
    """False once the video window has been closed with its X button."""
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def run_webcam(link, camera, width, color):
    print(f"Opening camera {camera} (this can take a few seconds)...")
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {camera}. Close other apps that are "
                 "using it, or try a different index with --camera.")
    if isinstance(camera, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap = NewestFrame(cap)      # a video file is read frame by frame instead

    view, t = VIEWS.index("edges"), 100
    fps, last = 0.0, time.perf_counter()
    failures = failures_in_a_row = 0
    window_seen = False
    print("Click the video window, then: 1-5 change view, +/- threshold, "
          "c laptop or FPGA grayscale, q quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("No frame from the camera (unplugged, busy, access denied, "
                      "or the video file ended).")
                break
            height = max(1, round(frame.shape[0] * width / frame.shape[1]))
            small = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

            try:
                from_fpga = link.process(small if color else model.rgb_to_gray(small),
                                         view, t)
                failures_in_a_row = 0
            except LinkError as error:
                from_fpga = None
                failures += 1
                failures_in_a_row += 1
                print(f"Link problem, starting over: {error}")
                if failures_in_a_row == 3:
                    sys.exit("Three failures in a row. Check that the board is "
                             "still plugged in with the design loaded, and that "
                             "--baud matches the BAUD it was built with.")
                if failures == 3:
                    print("This keeps happening. Run --selftest: if bytes are "
                          "getting lost at this baud rate, rebuild at a lower BAUD.")
                try:
                    link.resync()
                except LinkError:
                    pass

            if from_fpga is not None:
                expected = model.process(small, t)[VIEWS[view]]
                wrong = int(np.count_nonzero(from_fpga != expected))

                now = time.perf_counter()
                rate = 1 / max(now - last, 1e-6)
                fps = rate if fps == 0.0 else 0.8 * fps + 0.2 * rate  # smoothed
                last = now

                # Camera on the left, FPGA result on the right, status lines
                # below. Small frames are enlarged by a whole factor so the
                # pixels stay crisp; big ones are shrunk to fit the screen.
                images = np.hstack([small, cv2.cvtColor(from_fpga, cv2.COLOR_GRAY2BGR)])
                zoom = min(640 / width, 600 / height)
                zoom = int(zoom) if zoom >= 1 else zoom
                images = cv2.resize(images, None, fx=zoom, fy=zoom,
                                    interpolation=cv2.INTER_NEAREST if zoom >= 1
                                    else cv2.INTER_AREA)
                bar = np.zeros((48, images.shape[1], 3), np.uint8)
                status = (f"view: {VIEWS[view]}   threshold: {t}   "
                          f"{fps:.1f} fps   {width}x{height}   "
                          f"gray by: {'FPGA' if color else 'laptop'}")
                verdict = ("FPGA output is identical to the software model" if wrong == 0
                           else "FPGA output differs from the software model "
                                f"in {wrong} pixels")
                cv2.putText(bar, status, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(bar, verdict, (10, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if wrong == 0 else (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW, np.vstack([images, bar]))

            key = cv2.waitKey(1) & 0xFF
            open_now = window_is_open()
            if key in (ord("q"), ord("Q"), 27) or (window_seen and not open_now):
                break                           # q, Esc, or the window's X button
            window_seen = open_now
            if ord("1") <= key <= ord("5"):
                view = key - ord("1")
            elif key in (ord("+"), ord("=")):
                t = min(t + 5, 255)
            elif key in (ord("-"), ord("_")):
                t = max(t - 5, 0)
            elif key in (ord("c"), ord("C")):
                color = not color
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Live Sobel edge detector running on the FPGA.")
    parser.add_argument("--port", default=None,
                        help="serial port of the board, for example COM5 "
                             "(default: find it automatically)")
    parser.add_argument("--baud", type=int, default=3_000_000,
                        help="must match the BAUD the FPGA design was built "
                             "with (default 3000000)")
    parser.add_argument("--camera", default="0",
                        help="camera index, or the path of a video file (default 0)")
    parser.add_argument("--width", type=int, default=None,
                        help="width of the frames sent to the FPGA, in pixels "
                             "(default 160, or 320 at 6 Mbaud and up)")
    parser.add_argument("--color", action="store_true",
                        help="send color pixels and let the FPGA do the "
                             "grayscale step too (3 times slower)")
    parser.add_argument("--selftest", action="store_true",
                        help="verify the link and the FPGA's math, then exit")
    args = parser.parse_args()

    width = args.width
    if width is None:
        width = 320 if args.baud >= 6_000_000 else 160
    if not 3 <= width <= MAX_WIDTH:
        sys.exit(f"--width must be between 3 and {MAX_WIDTH}")

    link = connect(args.port, args.baud)
    print(f"FPGA found on {link.port} at {link.baud} baud.")
    try:
        if args.selftest:
            sys.exit(0 if selftest(link) else 1)
        camera = int(args.camera) if args.camera.isdigit() else args.camera
        run_webcam(link, camera, width, args.color)
    finally:
        link.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit("Stopped.")
