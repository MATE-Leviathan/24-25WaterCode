#!/usr/bin/env python3
"""Simple serial monitor for Jetson/Linux systems."""

from __future__ import annotations

import argparse
import signal
import sys
from typing import Iterable

import serial
from serial.tools import list_ports


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read and print data from a serial device.",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial device path, e.g. /dev/ttyACM0 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate to use",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Read timeout in seconds",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw bytes repr instead of decoded text",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected serial ports and exit",
    )
    return parser


def iter_ports() -> Iterable[str]:
    for port in sorted(list_ports.comports(), key=lambda item: item.device):
        details = f"{port.device}"
        if port.description and port.description != "n/a":
            details += f" - {port.description}"
        yield details


def list_available_ports() -> int:
    ports = list(iter_ports())
    if not ports:
        print("No serial devices found.")
        return 1

    print("Available serial devices:")
    for port in ports:
        print(f"  {port}")
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.list:
        return list_available_ports()

    running = True

    def handle_signal(signum, frame):  # type: ignore[unused-argument]
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Reading from {args.port} @ {args.baud} baud. Press Ctrl+C to stop.")

    try:
        while running:
            line = ser.readline()
            if not line:
                continue

            if args.raw:
                print(repr(line))
                continue

            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            print(text)
    except serial.SerialException as exc:
        print(f"Serial read error: {exc}", file=sys.stderr)
        return 1
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
