#!/usr/bin/env python3
"""Ferrum closed-loop aim test.

Spawns a fullscreen Tkinter window with red dots. Reads the Mac cursor
position via pynput, sends km.move(dx, dy) Ferrum commands to drive the
cursor toward each target with a smooth-pull controller (speed proportional
to remaining distance — fast at first, slows as it approaches). Dots turn
green when hit.

Where the commands go:
    Point this at the hurra-app VCOM (the hurra-bridge PTY), not the raw
    CH343 UART. The device emulates the Ferrum command surface but runs the
    faster Hurra binary path underneath on the wire; the bridge does that
    translation. Because the PTY is an in-memory pipe, baud rate is
    irrelevant here — `--baud` is accepted only so the same invocation works
    if you ever point it straight at a real serial port.

Send rate:
    Sending is decoupled from the GUI event loop. A dedicated paced thread
    emits commands at `--rate` Hz (default 2000) using a high-resolution
    sleep+spin timer, while the GUI loop only does cursor tracking and target
    bookkeeping at a slower cadence. The aim controller outputs a velocity
    (px/sec); the sender converts it to per-tick deltas with a sub-pixel
    accumulator, so total cursor speed is independent of the send rate and
    fractional steps are never lost. The device coalesces sub-1ms commands
    into each 1 kHz HID frame, so rates well above 1 kHz are useful for
    exercising the link and the firmware's injection-accumulation path.

    NOTE: rates above ~1 kHz busy-wait one CPU core for timing precision
    (macOS time.sleep() granularity is too coarse to pace sub-millisecond).

Setup required:
    1. hurra-bridge running, exposing a VCOM/PTY               (command channel)
    2. Teensy USB device port plugged into Mac                 (HID injection out)
    3. A real USB mouse plugged into the Teensy carrier's USB HOST port
       — the firmware is a HID proxy; it won't run main() without an
       upstream device to enumerate.  See src/main.c.

Usage:
    pip install pynput pyserial
    tools/ferrum_aim_test.py /dev/ttys00N            # hurra-bridge PTY
    tools/ferrum_aim_test.py /dev/ttys00N --rate 4000

Keys:
    Space   start the run (cycles through all dots in order)
    Click   add a custom target at the click position
    R       reset all dots to red
    ESC     quit
"""

import argparse
import math
import sys
import threading
import time
import tkinter as tk

try:
    import serial
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")

try:
    from pynput.mouse import Controller as MouseController
except ImportError:
    sys.exit("pynput required:  pip install pynput")


# Aim controller tuning
HIT_THRESHOLD_PX = 15        # within this distance = target hit
TIMEOUT_S        = 3.0       # give up on a target after this long
CONTROL_MS       = 5         # GUI cursor-tracking tick (~200 Hz); NOT the send rate
COOLDOWN_MS      = 200       # idle pause between targets, lets cursor settle

# Closed-loop steering law. The inject→cursor→pynput feedback path lags tens of
# ms, so we steer toward the cursor's PREDICTED position one latency ahead
# (measured velocity × LATENCY_S) rather than its stale measured position. That
# makes it brake on time instead of overshooting, and reverse if it would blow
# past — which lets us run a snappy gain and skip a slow crawl-in.
LATENCY_S        = 0.045     # s; dead-time used for prediction (--latency override)
APPROACH_TAU     = 0.05      # s; speed = predicted_error / TAU, capped
SPEED_MAX        = 2500.0    # px/s cap
SPEED_MIN        = 20.0      # px/s floor to beat sub-pixel stiction (until in range)
ACCEL_MAX        = 80000.0   # px/s^2; gentle slew limit to smooth command spikes
VEL_FILTER_ALPHA = 0.4       # EWMA on the measured cursor velocity (noise control)

# OS pointer-acceleration / sensitivity compensation. macOS runs injected
# deltas through a nonlinear accel curve, so realized pixels per command unit
# is unknown and speed-dependent. Estimate it online from the ratio of observed
# cursor speed to commanded speed and divide it out so the loop steers in true
# screen pixels.
GAIN_INIT       = 1.0        # px observed per command unit (starting guess)
GAIN_MIN        = 0.2
GAIN_MAX        = 8.0
GAIN_ALPHA      = 0.10       # EWMA smoothing for the online estimate
GAIN_CMD_FLOOR  = 50.0       # only sample gain when commanding faster than this (units/s)
GAIN_OBS_FLOOR  = 30.0       # ...and the cursor is actually moving (px/s)

DEFAULT_RATE_HZ = 2000


class Dot:
    __slots__ = ("oval_id", "screen_x", "screen_y", "name", "hit",
                 "time_to_hit_s", "iterations")
    def __init__(self, oval_id, screen_x, screen_y, name):
        self.oval_id       = oval_id
        self.screen_x      = screen_x
        self.screen_y      = screen_y
        self.name          = name
        self.hit           = False
        self.time_to_hit_s = None
        self.iterations    = 0


def send_move(ser, dx, dy):
    """Send one km.move; return True if it went out, False on backpressure/closed."""
    line = f"km.move({dx}, {dy})\r\n".encode("ascii")
    try:
        ser.write(line)
        return True
    except serial.SerialTimeoutException:
        return False  # firmware/bridge backpressure — drop, next tick resends
    except serial.SerialException:
        return False  # port closed mid-shutdown


class AimTest:
    def __init__(self, ser, rate_hz, latency_s=LATENCY_S):
        self.ser     = ser
        self.rate    = rate_hz
        self.latency = latency_s
        self.mouse   = MouseController()

        # Shared state between the GUI control loop and the sender thread.
        self._lock   = threading.Lock()
        self._vx     = 0.0           # commanded velocity, units/sec (written by GUI)
        self._vy     = 0.0
        self._active = threading.Event()   # set while actively pulling a target
        self._stop   = threading.Event()   # set to tear the sender thread down
        self._sent   = 0             # total commands emitted (GIL-atomic int)
        self._sender = threading.Thread(target=self._sender_loop, daemon=True)

        # Controller state (GUI thread only).
        self.gain          = GAIN_INIT  # realized px per command unit (adaptive)
        self.cur_speed     = 0.0        # last commanded speed, px/s (for slew limit)
        self.last_cmd_speed = 0.0       # last commanded speed in units/s (for gain est)
        self.prev_x        = 0.0        # cursor position at previous control tick
        self.prev_y        = 0.0
        self.prev_t        = 0.0        # time of previous control tick
        self.vel_x         = 0.0        # filtered measured cursor velocity, px/s
        self.vel_y         = 0.0

        self.root = tk.Tk()
        self.root.title("Ferrum Aim Test")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#101010")

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.canvas = tk.Canvas(self.root, width=sw, height=sh,
                                bg="#101010", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.status = self.canvas.create_text(
            sw // 2, 32, fill="#cccccc",
            font=("Menlo", 16),
            text="[space] start   [click] add target   [r] reset   [esc] quit"
        )

        self.dots: list[Dot] = []
        self._place_default_grid(sw, sh)

        self.state           = "idle"      # idle | running | between | done
        self.current_idx     = 0
        self.target_start_t  = 0.0
        self.target_sent0    = 0            # self._sent snapshot at target start
        self.run_start_t     = 0.0
        self.run_sent0       = 0

        self.root.bind("<space>",  self.on_space)
        self.root.bind("<Escape>", self.on_escape)
        self.canvas.bind("<Button-1>", self.on_click)
        self.root.bind("r", self.on_reset)
        self.root.bind("R", self.on_reset)

    # ----- dot placement -----

    def _place_default_grid(self, sw, sh):
        # 4 × 2 grid centred on the screen, with some margin
        margin_x = sw // 6
        margin_y = sh // 4
        cols, rows = 4, 2
        for r in range(rows):
            for c in range(cols):
                x = margin_x + c * (sw - 2 * margin_x) // (cols - 1)
                y = margin_y + r * (sh - 2 * margin_y) // (rows - 1)
                self._add_dot(x, y)

    def _add_dot(self, x, y, radius=16):
        name = f"#{len(self.dots) + 1}"
        oid = self.canvas.create_oval(
            x - radius, y - radius, x + radius, y + radius,
            fill="#e02020", outline="#ff8080", width=2,
        )
        self.canvas.create_text(
            x, y + radius + 14,
            fill="#888888", font=("Menlo", 11),
            text=name,
        )
        # Canvas covers full screen on a fullscreen window, so canvas coords
        # match screen coords for cursor comparison.
        self.dots.append(Dot(oid, x, y, name))

    # ----- input handlers -----

    def on_space(self, _evt):
        if self.state in ("idle", "done"):
            self._reset_dots()
            self.state          = "running"
            self.current_idx    = 0
            self.run_start_t    = time.monotonic()
            self.run_sent0      = self._sent
            self._begin_target()

    def on_escape(self, _evt):
        self._shutdown()
        self.root.destroy()

    def on_click(self, evt):
        if self.state == "idle":
            self._add_dot(evt.x, evt.y)

    def on_reset(self, _evt):
        if self.state in ("idle", "done"):
            self._reset_dots()

    # ----- run -----

    def _reset_dots(self):
        for d in self.dots:
            d.hit           = False
            d.time_to_hit_s = None
            d.iterations    = 0
            self.canvas.itemconfig(d.oval_id, fill="#e02020", outline="#ff8080")
        self._set_status("[space] start   [click] add target   [r] reset   [esc] quit")

    def _begin_target(self):
        if self.current_idx >= len(self.dots):
            self._finish_run()
            return
        d = self.dots[self.current_idx]
        self.canvas.itemconfig(d.oval_id, outline="#ffff00", width=3)
        self.target_start_t = time.monotonic()
        self.target_sent0   = self._sent
        # Seed the feedback baseline at the current cursor so the first velocity
        # sample isn't polluted by the inter-target jump, and start from rest.
        self.prev_x, self.prev_y = self.mouse.position
        self.prev_t          = time.monotonic()
        self.vel_x = self.vel_y = 0.0
        self.cur_speed       = 0.0
        self.last_cmd_speed  = 0.0
        self.root.after(0, self._tick)

    def _stop_pull(self):
        """Pause the sender (between targets) and zero the commanded velocity."""
        self._active.clear()
        self.cur_speed      = 0.0
        self.last_cmd_speed = 0.0
        self.vel_x = self.vel_y = 0.0
        with self._lock:
            self._vx = 0.0
            self._vy = 0.0

    def _tick(self):
        if self.state != "running":
            return
        d = self.dots[self.current_idx]

        now = time.monotonic()
        dt = now - self.prev_t
        if dt <= 0.0 or dt > 0.1:        # first tick / stall guard
            dt = CONTROL_MS / 1000.0
        self.prev_t = now

        cx, cy = self.mouse.position

        # Measured cursor velocity (px/s), EWMA-filtered against pynput jitter.
        ivx = (cx - self.prev_x) / dt
        ivy = (cy - self.prev_y) / dt
        self.vel_x = (1.0 - VEL_FILTER_ALPHA) * self.vel_x + VEL_FILTER_ALPHA * ivx
        self.vel_y = (1.0 - VEL_FILTER_ALPHA) * self.vel_y + VEL_FILTER_ALPHA * ivy
        self.prev_x, self.prev_y = cx, cy

        # --- online gain estimate: observed px/s per commanded unit/s ---
        # Tracks the OS pointer-acceleration curve so we steer in true pixels.
        obs_speed = math.hypot(self.vel_x, self.vel_y)
        if self.last_cmd_speed > GAIN_CMD_FLOOR and obs_speed > GAIN_OBS_FLOOR:
            g_inst = obs_speed / self.last_cmd_speed
            self.gain = min(GAIN_MAX, max(GAIN_MIN,
                (1.0 - GAIN_ALPHA) * self.gain + GAIN_ALPHA * g_inst))

        dist = math.hypot(d.screen_x - cx, d.screen_y - cy)

        elapsed = max(now - self.run_start_t, 1e-6)
        achieved = (self._sent - self.run_sent0) / elapsed
        self._set_status(
            f"target {self.current_idx + 1}/{len(self.dots)}  "
            f"dist={dist:6.1f}px  g={self.gain:4.2f}  "
            f"send={achieved:6.0f}/s (target {self.rate}/s)"
        )

        if dist <= HIT_THRESHOLD_PX:
            d.hit           = True
            d.time_to_hit_s = now - self.target_start_t
            d.iterations    = self._sent - self.target_sent0
            self.canvas.itemconfig(d.oval_id, fill="#20c020",
                                              outline="#80ff80", width=2)
            self.current_idx += 1
            self.state = "between"
            self._stop_pull()
            self.root.after(COOLDOWN_MS, self._after_cooldown)
            return

        if now - self.target_start_t > TIMEOUT_S:
            d.iterations = self._sent - self.target_sent0
            self.canvas.itemconfig(d.oval_id, fill="#404040",
                                              outline="#808080", width=2)
            self.current_idx += 1
            self.state = "between"
            self._stop_pull()
            self.root.after(COOLDOWN_MS, self._after_cooldown)
            return

        # Steer toward the cursor's PREDICTED position one latency ahead, so
        # in-flight motion is accounted for: braking happens on time, and if we
        # would overshoot the predicted error flips sign and we reverse.
        ex = d.screen_x - (cx + self.vel_x * self.latency)
        ey = d.screen_y - (cy + self.vel_y * self.latency)
        edist = math.hypot(ex, ey)

        speed = min(SPEED_MAX, edist / APPROACH_TAU)
        if dist > HIT_THRESHOLD_PX:
            speed = max(speed, SPEED_MIN)

        # Gentle slew limit to smooth command-speed spikes (e.g. gain jumps).
        dv_max = ACCEL_MAX * dt
        if speed > self.cur_speed + dv_max:
            speed = self.cur_speed + dv_max
        elif speed < self.cur_speed - dv_max:
            speed = self.cur_speed - dv_max
        self.cur_speed = speed

        # px/s → command units/s via estimated gain, along the predicted error.
        cmd_speed = speed / self.gain
        self.last_cmd_speed = cmd_speed
        if edist > 1e-6:
            vx = ex / edist * cmd_speed
            vy = ey / edist * cmd_speed
        else:
            vx = vy = 0.0
        with self._lock:
            self._vx = vx
            self._vy = vy
        self._active.set()
        self.root.after(CONTROL_MS, self._tick)

    def _after_cooldown(self):
        self.state = "running"
        self._begin_target()

    def _finish_run(self):
        self.state = "done"
        self._stop_pull()
        total = time.monotonic() - self.run_start_t
        hits  = sum(1 for d in self.dots if d.hit)
        avg_t = (
            sum(d.time_to_hit_s for d in self.dots if d.hit) / hits
            if hits else 0.0
        )
        avg_i = (
            sum(d.iterations for d in self.dots if d.hit) / hits
            if hits else 0.0
        )
        achieved = (self._sent - self.run_sent0) / max(total, 1e-6)
        print(f"\n=== Run complete ===")
        print(f"Total time: {total:.2f} s")
        print(f"Hits: {hits}/{len(self.dots)}")
        print(f"Send rate: {achieved:.0f}/s achieved (target {self.rate}/s)")
        print(f"Est. gain: {self.gain:.2f} px/unit (OS accel/sensitivity)")
        if hits:
            print(f"Avg time-to-hit: {avg_t * 1000:.0f} ms")
            print(f"Avg commands:    {avg_i:.1f}")
        print("Per-target:")
        for d in self.dots:
            if d.hit:
                print(f"  {d.name:>4}  {d.time_to_hit_s * 1000:5.0f} ms "
                      f"({d.iterations} cmds)")
            else:
                print(f"  {d.name:>4}  MISS")
        self._set_status(
            f"done — {hits}/{len(self.dots)} hits in {total:.1f}s   "
            f"{achieved:.0f}/s   [space] retry   [r] reset   [esc] quit"
        )

    def _set_status(self, txt):
        self.canvas.itemconfig(self.status, text=txt)

    # ----- sender thread -----

    def _sender_loop(self):
        """Emit km.move paced at --rate Hz, decoupled from the Tkinter loop.

        Commanded velocity (units/sec) is set by the GUI control loop. We
        integrate over the REAL elapsed time between iterations — not the
        nominal period — so the motion stays correct even when the link drains
        slower than --rate and blocking writes stretch the interval. The
        sub-pixel remainder is carried so small steps accumulate instead of
        rounding to zero, and we report what we actually sent back to the
        controller for its gain estimate.
        """
        period = 1.0 / self.rate
        # Sleep until ~spin_margin before the deadline, then busy-spin the rest.
        # macOS time.sleep() can overshoot by ~1 ms, so for sub-ms periods this
        # is effectively a pure spin — accurate, at the cost of one core.
        spin_margin = 0.0015
        fx = fy = 0.0
        last = time.perf_counter()
        next_t = last
        while not self._stop.is_set():
            next_t += period
            while True:
                dt_wait = next_t - time.perf_counter()
                if dt_wait <= 0:
                    break
                if dt_wait > spin_margin:
                    time.sleep(dt_wait - spin_margin)
                # else: busy-spin the remainder for sub-ms precision
            now = time.perf_counter()
            if now - next_t > 0.05:      # fell behind (stall/GC): don't burst
                next_t = now
            dt = now - last
            last = now
            if dt > 0.05:                # cap after a long pause
                dt = 0.05

            if not self._active.is_set():
                fx = fy = 0.0
                continue

            with self._lock:
                vx, vy = self._vx, self._vy
            fx += vx * dt
            fy += vy * dt
            sx = int(fx); fx -= sx
            sy = int(fy); fy -= sy
            if (sx or sy) and send_move(self.ser, sx, sy):
                self._sent += 1

    def _shutdown(self):
        self._stop.set()
        self._active.clear()
        if self._sender.is_alive():
            self._sender.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass

    # ----- runloop -----

    def run(self):
        # Verify the device speaks Ferrum before starting the GUI runloop.
        # NB: no ser.flush() — it blocks indefinitely on the hurra-bridge PTY.
        self.ser.reset_input_buffer()
        self.ser.write(b"km.version()\r\n")
        deadline = time.monotonic() + 0.5
        buf = b""
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b: continue
            buf += b
            if buf.endswith(b"\r\n"): break
        ver = buf.decode("ascii", "replace").rstrip("\r\n")
        if ver != "kmbox: Ferrum":
            sys.exit(f"device handshake failed: got {ver!r}, expected 'kmbox: Ferrum'")
        print(f"Connected: {ver}  (send rate target {self.rate}/s)")
        self._sender.start()
        try:
            self.root.mainloop()
        finally:
            self._shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="hurra-bridge VCOM/PTY, e.g. /dev/ttys003")
    ap.add_argument("--rate", type=int, default=DEFAULT_RATE_HZ,
                    help=f"command send rate in Hz (default {DEFAULT_RATE_HZ}); "
                         "values >1000 busy-wait one core for timing precision")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored on a PTY; only used if pointed at a real serial port")
    ap.add_argument("--latency", type=float, default=LATENCY_S,
                    help=f"feedback dead-time (s) used for prediction "
                         f"(default {LATENCY_S}); raise if it still overshoots, "
                         "lower if it stalls short of the dot")
    args = ap.parse_args()

    if args.rate < 1:
        sys.exit("--rate must be >= 1")
    if args.latency < 0:
        sys.exit("--latency must be >= 0")

    # Short write_timeout: under backpressure we want to DROP a stale command
    # and keep the control loop fresh, never block on a full buffer.
    ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.02)
    AimTest(ser, args.rate, args.latency).run()


if __name__ == "__main__":
    main()
