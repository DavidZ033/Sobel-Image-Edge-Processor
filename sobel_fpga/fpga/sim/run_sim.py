"""
Simulate the FPGA design and check it against sobel_webcam.py.

This runs the real Verilog from fpga/rtl in a simulator, so the design can be
tested (and changed) without the board or Vivado. The software model is the
judge: every byte the simulated FPGA sends back has to match it.

Setup:  install Icarus Verilog so that iverilog and vvp are on the PATH
        (Windows installer: https://bleyer.org/icarus/), plus
        pip install opencv-python numpy pyserial
Run:    python fpga/sim/run_sim.py
        python fpga/sim/run_sim.py --waves    also write .vcd waveforms to
                                              fpga/sim/build, for GTKWave
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent
RTL = SIM.parent / "rtl"
BUILD = SIM / "build"
sys.path.insert(0, str(SIM.parent.parent))      # where the two .py files live

import sobel_webcam as model                    # noqa: E402
import sobel_fpga as host                       # noqa: E402

VIEWS = model.VIEWS
RESET = ["2000"]
waves = False
results = []


# ---------------------------------------------------------------------------
# Running the simulator.
# ---------------------------------------------------------------------------

def compile_tb(name, top, **parameters):
    """Compile a testbench with the design. Returns the file vvp runs."""
    out = BUILD / f"{name}.vvp"
    command = ["iverilog", "-g2005", "-o", str(out), "-s", top]
    command += [f"-P{top}.{key}={value}" for key, value in parameters.items()]
    command += [str(f) for f in sorted(RTL.glob("*.v"))]
    command += [str(SIM / "xilinx_stubs.v"), str(SIM / f"{top}.v")]
    subprocess.run(command, check=True)
    return out


def simulate(vvp, test, *plusargs):
    """Run one simulation. Returns the bytes that came back and the counters."""
    out = BUILD / f"{test}.out"
    out.unlink(missing_ok=True)                 # never judge a leftover from an earlier run
    command = ["vvp", "-n", vvp.name, f"+out={out.name}", *plusargs]
    if waves:
        command.append(f"+vcd={test}.vcd")
    run = subprocess.run(command, cwd=BUILD, capture_output=True, text=True)
    summary = [line for line in run.stdout.splitlines() if line.startswith("RESULT")]
    if run.returncode != 0 or not summary:
        sys.exit(f"simulation {test} did not finish:\n{run.stdout}{run.stderr}")
    counters = {key: float(value) for key, value in
                (item.split("=") for item in summary[0].split()[1:])}
    reply = bytes(int(line, 16) for line in out.read_text().split()) if out.exists() else b""
    if len(reply) != counters.get("bytes_back", 0):
        sys.exit(f"simulation {test}: the output file does not match the run")
    return reply, counters


def report(text, ok):
    results.append(ok)
    print(("PASS  " if ok else "FAIL  ") + text, flush=True)


def send(data):
    """Script words that send these bytes."""
    return [f"00{value:02x}" for value in data]


def idle(bit_times):
    """Script words that keep the line quiet for at least this long."""
    return [f"1{min(4095, -(-bit_times // 64)):03x}"]


def frame_words(frame_bgr, view, t, color):
    pixels = frame_bgr if color else model.rgb_to_gray(frame_bgr)
    return send(host.make_packet(pixels, view, t))


def replies_match(reply, cases):
    """cases is a list of (frame_bgr, view, threshold) in the order sent."""
    at = 0
    for frame, view, t in cases:
        h, w = frame.shape[:2]
        chunk = reply[at:at + w * h]
        at += w * h
        if len(chunk) != w * h:
            return False
        got = host.unpack_reply(chunk, w, h, view)
        if not np.array_equal(got, model.process(frame, t)[VIEWS[view]]):
            return False
    return at == len(reply)                     # and nothing extra came back


def expected_stream(frame_bgr, view, t):
    """The bytes the FPGA should send for this frame, in the order it sends them."""
    picture = model.process(frame_bgr, t)[VIEWS[view]]
    return (picture if view == 0 else np.roll(picture, (1, 1), axis=(0, 1))).tobytes()


def run_top(vvp, test, words, *plusargs):
    (BUILD / f"{test}.script").write_text("\n".join(words + ["ffff"]) + "\n")
    return simulate(vvp, test, f"+script={test}.script", *plusargs)


def clean(counters):
    """No error of any kind was flagged inside the design or on the wire."""
    return all(counters[key] == 0 for key in
               ("frame_errors", "rx_errors", "overflows", "bad_stop_bits"))


# ---------------------------------------------------------------------------
# The tests.
# ---------------------------------------------------------------------------

def gray_to_bgr(gray):
    return np.repeat(gray[..., None], 3, axis=2)


def test_fifo():
    """sync_fifo.v alone, tiny, with random traffic so that it fills and empties."""
    vvp = compile_tb("fifo", "tb_sync_fifo", DEPTH=8)
    _, counters = simulate(vvp, "fifo")
    report(f"FIFO alone, 8 deep, random traffic: {counters['pops']:.0f} bytes came out "
           f"in order, {counters['refused']:.0f} pushes were turned away while it was full",
           counters["errors"] == 0 and counters["peak"] == 8
           and counters["refused"] > 0 and counters["pops"] == counters["pushes"] > 10_000)


def test_core(rng):
    """sobel_core.v alone, fed a pixel on every clock."""
    vvp = compile_tb("core", "tb_sobel_core")
    frames = [rng.integers(0, 256, (48, 64), dtype=np.uint8) for _ in range(3)]
    frames[1] = rng.choice(np.array([0, 255], np.uint8), (48, 64))    # worst case swings
    pixels = np.concatenate([f.ravel() for f in frames])
    (BUILD / "core.pixels").write_text("\n".join(f"{p:02x}" for p in pixels) + "\n")

    for stall, how in ((0, "a pixel on every clock"), (3, "random pauses between pixels")):
        ok, latencies = True, set()
        for view in range(5):
            reply, counters = simulate(
                vvp, f"core_view{view}_stall{stall}", "+pixels=core.pixels",
                "+width=64", "+height=48", "+frames=3", f"+view={view}",
                "+threshold=100", f"+stall={stall}")
            ok &= replies_match(reply, [(gray_to_bgr(f), view, 100) for f in frames])
            latencies.add(counters["latency"])
        report(f"core alone, {how}: 3 frames back to back, all 5 views match, "
               f"result is always {min(latencies):.0f} clocks behind",
               ok and latencies == {7.0})

    wide = rng.integers(0, 256, (4, host.MAX_WIDTH), dtype=np.uint8)
    (BUILD / "wide.pixels").write_text("\n".join(f"{p:02x}" for p in wide.ravel()) + "\n")
    reply, _ = simulate(vvp, "core_wide", "+pixels=wide.pixels", f"+width={host.MAX_WIDTH}",
                        "+height=4", "+view=3")
    report(f"core alone, a frame as wide as the line buffer ({host.MAX_WIDTH} pixels) matches",
           replies_match(reply, [(gray_to_bgr(wide), 3, 0)]))


def test_views(vvp, rng, baud):
    noise = rng.integers(0, 256, (10, 14, 3), dtype=np.uint8)
    black_or_white = rng.choice(np.array([0, 255], np.uint8), (10, 14, 1))
    extremes = np.repeat(black_or_white, 3, axis=2)
    for name, frame in (("random noise", noise), ("0/255 extremes", extremes)):
        for color in (False, True):
            words, cases = [], []
            for view in range(5):
                words += frame_words(frame, view, 100, color)
                cases.append((frame, view, 100))
            reply, counters = run_top(vvp, f"views_{baud}_{name[:1]}_{int(color)}", words)
            report(f"{baud} baud, {name}, {'color' if color else 'gray'} pixels sent: "
                   "all 5 views match the software model",
                   replies_match(reply, cases) and clean(counters)
                   and counters["frames_done"] == 5)


def test_sizes_and_thresholds(vvp, rng):
    words, cases = [], []
    for w, h in [(1, 1), (2, 2), (3, 3), (5, 2), (2, 5), (7, 3), (3, 7), (21, 4)]:
        frame = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
        for view in range(5):
            words += frame_words(frame, view, 40, color=(w + view) % 2 == 0)
            cases.append((frame, view, 40))
    reply, counters = run_top(vvp, "sizes", words)
    report("frame sizes from 1x1 to 21x4, mixed gray and color: all match",
           replies_match(reply, cases) and clean(counters))

    frame = rng.integers(0, 256, (10, 14, 3), dtype=np.uint8)
    words, cases = [], []
    for t in (0, 1, 127, 254, 255):
        words += frame_words(frame, 4, t, color=False)
        cases.append((frame, 4, t))
    reply, counters = run_top(vvp, "thresholds", words)
    report("edges view at thresholds 0, 1, 127, 254, 255: all match",
           replies_match(reply, cases) and clean(counters))

    every_value = np.arange(256, dtype=np.uint8).reshape(16, 16)
    reply, counters = run_top(vvp, "loopback",
                              frame_words(gray_to_bgr(every_value), 0, 0, color=False))
    report("gray view returns all 256 pixel values unchanged",
           reply == every_value.tobytes() and clean(counters))


def test_clock_mismatch(vvp, rng, baud):
    """The laptop's USB chip and the FPGA never agree exactly on the bit time."""
    frame = rng.integers(0, 256, (10, 14, 3), dtype=np.uint8)
    cases = [(frame, view, 100) for view in range(5)]
    words = [w for view in range(5) for w in frame_words(frame, view, 100, False)]
    nominal = 1e9 / baud
    for percent in (+2.0, -2.0):
        reply, counters = run_top(vvp, f"mismatch_{percent:+.0f}", words,
                                  f"+bit_ns={nominal * (1 - percent / 100):.6f}")
        report(f"laptop clock {abs(percent):.0f}% "
               f"{'fast' if percent > 0 else 'slow'}: still every byte right",
               replies_match(reply, cases) and clean(counters))


def test_back_to_back(vvp, rng):
    """Frames with no pause between them, and the laptop's clock a little fast."""
    frames = [rng.integers(0, 256, (30, 40, 3), dtype=np.uint8) for _ in range(4)]
    words = [w for f in frames for w in frame_words(f, 3, 0, False)]
    reply, counters = run_top(vvp, "stream", words, f"+bit_ns={1e9 / 12e6 * 0.995:.6f}")
    report("4 frames of 40x30 with no pause, laptop clock 0.5% fast: all match "
           f"(up to {counters['fifo_peak']:.0f} bytes waited in the output queue)",
           replies_match(reply, [(f, 3, 0) for f in frames]) and clean(counters)
           and counters["shortest_gap_bits"] > 9.99)


def test_recovery(vvp, rng):
    """Things that should be shrugged off, each followed by a normal frame."""
    frame = rng.integers(0, 256, (8, 12, 3), dtype=np.uint8)
    good = frame_words(frame, 3, 0, False)
    expect = [(frame, 3, 0)]
    packet = host.make_packet(model.rgb_to_gray(frame), 3, 0)

    # Noise, including lone 0xA5 bytes that look like the start of a marker.
    # (A complete A5 5A pair is taken out: that would be a real frame start.)
    noise = rng.integers(0, 256, 400, dtype=np.uint8).tobytes() + b"\xa5\x00\xa5\xa5"
    while b"\xa5\x5a" in noise:
        noise = noise.replace(b"\xa5\x5a", b"\xa5\x00")
    reply, counters = run_top(vvp, "junk", send(noise) + good)
    report("random bytes before a frame, stray marker bytes included, are ignored",
           replies_match(reply, expect) and counters["frames_done"] == 1)

    bad_check = bytearray(packet[:9 + 20])
    bad_check[8] ^= 0x01
    reply, counters = run_top(vvp, "badcheck", send(bad_check) + idle(64) + good)
    report("a header with a wrong check byte is thrown away",
           replies_match(reply, expect) and counters["frame_errors"] == 1)

    too_wide = bytearray(packet[:9])
    too_wide[4:6] = (host.MAX_WIDTH + 1).to_bytes(2, "little")
    too_wide[8] = 0
    for value in too_wide[2:8]:
        too_wide[8] ^= value
    reply, counters = run_top(vvp, "toowide", send(too_wide) + idle(64) + good)
    report("a header asking for a row wider than the line buffer is thrown away",
           replies_match(reply, expect) and counters["frame_errors"] == 1)

    # The header and 40 pixels of a frame, then silence for longer than the
    # timeout (1 ms in this testbench, 200 ms on the board), then a normal
    # frame. The 40 pixels that did arrive still get their 40 answers.
    half = send(packet[:9 + 40])
    answers = expected_stream(frame, 3, 0)[:40]
    reply, counters = run_top(vvp, "timeout", half + idle(13_000) + good)
    report("a frame that stops halfway is abandoned after the timeout",
           reply[:40] == answers and replies_match(reply[40:], expect)
           and counters["frame_errors"] == 1 and counters["frames_done"] == 1)

    reply, counters = run_top(vvp, "reset", half + idle(200) + RESET + good)
    report("pressing reset in the middle of a frame recovers cleanly",
           reply[:40] == answers and replies_match(reply[40:], expect)
           and counters["frames_done"] == 1)

    reply, counters = run_top(vvp, "slowsender", good, "+gap=7")
    report("a sender that pauses between bytes (but under the timeout) is fine",
           replies_match(reply, expect) and clean(counters))


def main():
    global waves
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--waves", action="store_true",
                        help="write a .vcd waveform file for every simulation")
    waves = parser.parse_args().waves

    if not shutil.which("iverilog") or not shutil.which("vvp"):
        sys.exit("Icarus Verilog was not found. Install it and make sure "
                 "iverilog and vvp are on the PATH.")
    BUILD.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    test_fifo()
    test_core(rng)

    fast = compile_tb("top_12M", "tb_sobel_top", BAUD=12_000_000)   # 8 clocks per bit
    test_views(fast, rng, 12_000_000)
    test_sizes_and_thresholds(fast, rng)
    test_clock_mismatch(fast, rng, 12_000_000)
    test_back_to_back(fast, rng)
    test_recovery(fast, rng)

    slow = compile_tb("top_3M", "tb_sobel_top", BAUD=3_000_000)     # 32 clocks per bit
    test_views(slow, rng, 3_000_000)

    print()
    print("All simulations match the software model." if all(results)
          else f"{results.count(False)} of {len(results)} checks FAILED.")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
