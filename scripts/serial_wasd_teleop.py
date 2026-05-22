#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import serial


HELP = """
Serial WASD teleop

Controls:
  w: forward
  s: backward
  a: spin left
  d: spin right
  x or space: stop
  + / -: increase/decrease PWM step
  q: quit
""".strip()


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def format_cmd(left: int, right: int) -> str:
    return f"{left},{right}\n"


def main() -> int:
    p = argparse.ArgumentParser(description="WASD teleop over raw serial PWM commands")
    p.add_argument("--port", default="/dev/motor-controller", help="Serial port path")
    p.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    p.add_argument("--pwm", type=int, default=512, help="Default absolute PWM for moves")
    p.add_argument("--pwm-step", type=int, default=20, help="PWM step for +/- keys")
    p.add_argument("--pwm-min", type=int, default=0, help="Minimum allowed absolute PWM")
    p.add_argument("--pwm-max", type=int, default=1023, help="Maximum allowed absolute PWM")
    p.add_argument(
        "--repeat-ms",
        type=int,
        default=100,
        help="Command resend period in ms (must be < node timeout, e.g. 300ms)",
    )
    p.add_argument(
        "--deadman-ms",
        type=int,
        default=500,
        help="How long a keypress stays active without repeats; lower means stricter hold-to-run",
    )
    p.add_argument(
        "--line-mode",
        action="store_true",
        help="Read commands as full lines (w/a/s/d + Enter), useful if raw key mode is unavailable",
    )
    args = p.parse_args()

    if args.repeat_ms <= 0:
        print("--repeat-ms must be > 0", file=sys.stderr)
        return 2
    if args.deadman_ms <= 0:
        print("--deadman-ms must be > 0", file=sys.stderr)
        return 2

    pwm = clamp(args.pwm, args.pwm_min, args.pwm_max)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    print(HELP)
    print(f"Opened {args.port} @ {args.baud}. repeat={args.repeat_ms}ms, pwm={pwm}")

    stdin_is_tty = sys.stdin.isatty()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd) if stdin_is_tty else None

    active = "stop"
    active_until = 0.0
    stop_sent = False

    def command_for(mode: str, mag: int):
        if mode == "fwd":
            return mag, mag
        if mode == "back":
            return -mag, -mag
        if mode == "left":
            return -mag, mag
        if mode == "right":
            return mag, -mag
        return 0, 0

    def send(mode: str):
        left, right = command_for(mode, pwm)
        cmd = format_cmd(left, right)
        ser.write(cmd.encode("ascii"))
        ser.flush()
        return left, right

    try:
        if args.line_mode or not stdin_is_tty:
            if not stdin_is_tty:
                print("stdin is not a TTY; forcing --line-mode")
            print("Line mode: type w/a/s/d/x/space/+/-/q then Enter")
            next_send_t = time.monotonic()
            while True:
                now = time.monotonic()
                if now >= next_send_t and active != "stop" and now <= active_until:
                    left, right = send(active)
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    next_send_t = now + (args.repeat_ms / 1000.0)
                    stop_sent = False
                elif active != "stop" and now > active_until:
                    active = "stop"
                    left, right = send("stop")
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    stop_sent = True

                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    continue
                line = sys.stdin.readline()
                if not line:
                    continue
                c = line.strip().lower()[:1]
                if c == "q":
                    break
                elif c == "w":
                    active = "fwd"
                    active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                    stop_sent = False
                elif c == "s":
                    active = "back"
                    active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                    stop_sent = False
                elif c == "a":
                    active = "left"
                    active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                    stop_sent = False
                elif c == "d":
                    active = "right"
                    active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                    stop_sent = False
                elif c in ("x", ""):
                    active = "stop"
                    active_until = 0.0
                    stop_sent = False
                elif c == "+":
                    pwm = clamp(pwm + args.pwm_step, args.pwm_min, args.pwm_max)
                    print(f"\nPWM={pwm}")
                elif c == "-":
                    pwm = clamp(pwm - args.pwm_step, args.pwm_min, args.pwm_max)
                    print(f"\nPWM={pwm}")
                if active != "stop":
                    left, right = send(active)
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    stop_sent = False
                elif not stop_sent:
                    left, right = send("stop")
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    stop_sent = True
        else:
            tty.setraw(fd)
            next_send_t = 0.0

            while True:
                now = time.monotonic()
                timeout = max(0.0, next_send_t - now)
                rlist, _, _ = select.select([sys.stdin], [], [], timeout)

                if rlist:
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue

                    c = ch.lower()
                    if c == "q":
                        break
                    elif c == "w":
                        active = "fwd"
                        active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                        stop_sent = False
                    elif c == "s":
                        active = "back"
                        active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                        stop_sent = False
                    elif c == "a":
                        active = "left"
                        active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                        stop_sent = False
                    elif c == "d":
                        active = "right"
                        active_until = time.monotonic() + (args.deadman_ms / 1000.0)
                        stop_sent = False
                    elif c in ("x", " "):
                        active = "stop"
                        active_until = 0.0
                        stop_sent = False
                    elif c == "+":
                        pwm = clamp(pwm + args.pwm_step, args.pwm_min, args.pwm_max)
                        print(f"\nPWM={pwm}")
                    elif c == "-":
                        pwm = clamp(pwm - args.pwm_step, args.pwm_min, args.pwm_max)
                        print(f"\nPWM={pwm}")

                    if active != "stop":
                        left, right = send(active)
                        print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                        stop_sent = False
                    elif not stop_sent:
                        left, right = send("stop")
                        print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                        stop_sent = True
                    next_send_t = time.monotonic() + (args.repeat_ms / 1000.0)
                    continue

                now = time.monotonic()
                if active != "stop" and now <= active_until and now >= next_send_t:
                    left, right = send(active)
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    next_send_t = now + (args.repeat_ms / 1000.0)
                    stop_sent = False
                elif active != "stop" and now > active_until and not stop_sent:
                    active = "stop"
                    left, right = send("stop")
                    print(f"\rmode={active:>5} pwm={pwm:4d} cmd={left:5d},{right:5d}   ", end="", flush=True)
                    stop_sent = True

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(format_cmd(0, 0).encode("ascii"))
            ser.flush()
        except Exception:
            pass
        ser.close()
        if old_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nStopped and serial closed")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
