#!/usr/bin/env python3
"""Interactive CLI for the robot's binary serial protocol.

Replaces the firmware's old HumanInterface text mode and the root-level
package_gen.py / package_receive.py scripts, which are no longer needed now
that this talks the binary protocol directly with checksums.

Usage:
    python tools/cli.py                  # auto-detect the MCU
    python tools/cli.py --port COM5       # or specify explicitly
"""
import argparse
import shlex
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import devices
from kinematics import solve_ik_first
from serial_link import COMMAND_MAP, STATUS_MAP, SerialLink

AXES = "abcde"


def parse_tagged(arg):
    """Parse '-a 10 -b 20' into {'a': 10.0, 'b': 20.0}."""
    tokens = shlex.split(arg)
    out = {}
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok.startswith("-") and len(tok) == 2 and tok[1] in AXES and i + 1 < len(tokens):
            out[tok[1]] = float(tokens[i + 1])
            i += 2
        else:
            i += 1
    return out


def _fmt_switches(link):
    """Human-readable limit-switch line, e.g. 'A: open   B: AT LIMIT   C: open'."""
    labels = ("A", "B", "C")
    touched = link.limit_switches
    return "   ".join(
        f"{name}: {'AT LIMIT' if hit else 'open'}" for name, hit in zip(labels, touched)
    )


def _switch_tag(link):
    """Compact monitor column, e.g. 'SW[..C]' (letter = touched, . = open)."""
    labels = ("A", "B", "C")
    return "SW[" + "".join(
        name if hit else "." for name, hit in zip(labels, link.limit_switches)
    ) + "]"


def angles_and_bitmask(tags):
    angles = [0.0] * 5
    bitmask = 0
    for i, ax in enumerate(AXES):
        if ax in tags:
            angles[i] = tags[ax]
            bitmask |= 1 << i
    return angles, bitmask


HELP = """\
Commands:
  moveto -a <deg> -b <deg> -c <deg> -d <deg> -e <deg>   absolute move (a/b/c/d=joints 1-4, e=grip)
  move -a <deg> ...                                     relative move
  goto <x_mm> <y_mm> <z_mm>                     move to an XYZ point (mm) via inverse kinematics
  currentpos -a <deg> ...                      set current position without moving
  position                                      query current joint angles
  grip                                          close the gripper
  release                                       open the gripper
  calibrate                                     move to reference switches / home
  abort                                         emergency stop (see doc/bug-report.md 5.1: incomplete in firmware)
  monitor                                       print the raw 20Hz status stream (Ctrl-C to stop)
  help                                          show this message
  quit / Ctrl-D                                 exit
"""


def run_shell(link):
    print(HELP)
    while True:
        try:
            line = input("robot> ").strip()
        except EOFError:
            print()
            return
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        cmd = cmd.lower()

        if cmd in ("quit", "exit"):
            return
        elif cmd == "help":
            print(HELP)
        elif cmd == "moveto":
            tags = parse_tagged(arg)
            if not tags:
                print("usage: moveto -a <deg> -b <deg> ...")
                continue
            angles, bitmask = angles_and_bitmask(tags)
            ok = link.move_to(angles, bitmask=bitmask)
            print("OK" if ok else "TIMEOUT/FAILED")
        elif cmd == "move":
            tags = parse_tagged(arg)
            if not tags:
                print("usage: move -a <deg> ...")
                continue
            deltas, bitmask = angles_and_bitmask(tags)
            ok = link.move_rel(deltas, bitmask=bitmask)
            print("OK" if ok else "TIMEOUT/FAILED")
        elif cmd == "goto":
            parts = shlex.split(arg)
            if len(parts) != 3:
                print("usage: goto <x_mm> <y_mm> <z_mm>")
                continue
            try:
                x, y, z = (float(p) for p in parts)
            except ValueError:
                print("usage: goto <x_mm> <y_mm> <z_mm> (numbers only)")
                continue
            sol = solve_ik_first(x, y, z)
            if sol is None:
                print("unreachable / no safe IK solution")
                continue
            print("IK: " + ", ".join(f"{a:.2f}" for a in sol))
            ok = link.move_to(sol, bitmask=0x0F)
            print("OK" if ok else "TIMEOUT/FAILED")
        elif cmd == "currentpos":
            tags = parse_tagged(arg)
            angles, bitmask = angles_and_bitmask(tags)
            link.set_current_pos(angles, bitmask=bitmask)
            print("sent")
        elif cmd == "position":
            print(link.query_position())
            print("Limit switches:  " + _fmt_switches(link))
        elif cmd == "grip":
            print("OK" if link.grip_close() else "TIMEOUT/FAILED")
        elif cmd == "release":
            print("OK" if link.grip_open() else "TIMEOUT/FAILED")
        elif cmd == "calibrate":
            print("Calibrating (this can take a while)...")
            print("OK" if link.calibrate() else "TIMEOUT/FAILED")
        elif cmd == "abort":
            link.abort()
            print("sent (see doc/bug-report.md 5.1: firmware abort is currently incomplete)")
        elif cmd == "monitor":
            do_monitor(link)
        else:
            print(f"unknown command: {cmd!r} (type 'help')")


def do_monitor(link):
    def on_packet(proc, stat, args):
        proc_name = COMMAND_MAP.get(proc, f"Unknown({proc})")
        stat_name = STATUS_MAP.get(stat, f"Unknown({stat})")
        args_fmt = ", ".join(f"{a:7.2f}" for a in args)
        print(f"{proc_name:20s} | {stat_name:10s} | [{args_fmt}] | {_switch_tag(link)}")

    link.add_packet_listener(on_packet)
    print("Monitoring... Ctrl-C to stop.")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        link.remove_packet_listener(on_packet)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or devices.find_mcu_port()
    print(f"Connecting to {port} @ {args.baud}...")
    link = SerialLink(port, baudrate=args.baud)
    try:
        run_shell(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
