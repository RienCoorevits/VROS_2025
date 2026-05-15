#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Run this script with a Python that has pyserial, "
        "for example ~/.platformio/penv/bin/python."
    ) from exc

try:
    from stream_drawing import (
        available_ports,
        expand_repeated_commands,
        iter_commands,
        open_serial_port,
        prepare_serial_port,
        stream_commands,
    )
except ModuleNotFoundError:
    from scripts.stream_drawing import (
        available_ports,
        expand_repeated_commands,
        iter_commands,
        open_serial_port,
        prepare_serial_port,
        stream_commands,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive serial console with host-side drawFromStream support."
    )
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem14401")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the serial port",
    )
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=300.0,
        help="Seconds to wait for the firmware to acknowledge streamed commands",
    )
    parser.add_argument(
        "--move-window",
        type=int,
        default=6,
        help="Maximum number of streamed move commands to keep in flight",
    )
    parser.add_argument(
        "--quiet-window",
        type=float,
        default=0.6,
        help="Seconds of serial silence before the prompt returns after a raw command",
    )
    parser.add_argument(
        "--drawings-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "drawings",
        help="Directory that drawFromStream looks in by default",
    )
    return parser.parse_args()


def print_available_ports() -> None:
    ports = available_ports()
    if not ports:
        print("No serial ports detected.", file=sys.stderr)
        return

    print("Available serial ports:", file=sys.stderr)
    for port in ports:
        print(f"  {port}", file=sys.stderr)


def read_until_quiet(ser: serial.Serial, quiet_window: float) -> None:
    deadline = time.monotonic() + quiet_window
    while time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
        if not line:
            continue
        print(f"< {line}")
        deadline = time.monotonic() + quiet_window


def resolve_drawing_path(raw_name: str, drawings_dir: Path) -> Path:
    candidate = Path(raw_name).expanduser()
    if candidate.is_absolute():
        return candidate
    if candidate.exists():
        return candidate.resolve()
    return (drawings_dir / candidate).resolve()


def handle_draw_from_stream(
    ser: serial.Serial,
    command: str,
    drawings_dir: Path,
    command_timeout: float,
    move_window: int,
) -> None:
    parts = [part.strip() for part in command.split(",")]
    if len(parts) < 2 or not parts[1]:
        raise ValueError("Usage: drawFromStream,<drawing-file>[,<repeat>]")

    drawing_path = resolve_drawing_path(parts[1], drawings_dir)
    if not drawing_path.is_file():
        raise FileNotFoundError(f"Drawing file not found: {drawing_path}")

    repeat_count = 1
    if len(parts) >= 3 and parts[2]:
        repeat_count = int(parts[2])
    if repeat_count < 1:
        raise ValueError("Repeat count must be at least 1.")

    commands = list(iter_commands(drawing_path))
    if not commands:
        raise ValueError(f"Drawing file contains no streamable commands: {drawing_path}")

    commands = expand_repeated_commands(commands, repeat_count)
    print(f"Streaming {drawing_path} ({repeat_count} lap(s))")
    stream_commands(
        ser,
        commands,
        command_timeout=command_timeout,
        move_window=move_window,
        return_to_origin=False,
    )
    print("Stream complete.")


def print_local_help() -> None:
    print("Local commands:")
    print("  drawFromStream,<drawing-file>[,<repeat>]")
    print("  help")
    print("  quit")
    print("All other input is forwarded to the firmware unchanged.")


def main() -> int:
    args = parse_args()

    if not args.port:
        print("--port is required.", file=sys.stderr)
        print_available_ports()
        return 1
    if args.move_window < 1:
        print("--move-window must be at least 1.", file=sys.stderr)
        return 1
    if args.quiet_window < 0:
        print("--quiet-window must be non-negative.", file=sys.stderr)
        return 1

    with open_serial_port(args.port, args.baud) as ser:
        print(f"Opened {args.port} at {args.baud} baud")
        prepare_serial_port(ser, args.startup_delay)
        print_local_help()

        while True:
            try:
                raw_command = input("serial> ").strip()
            except EOFError:
                print("")
                break
            except KeyboardInterrupt:
                print("")
                break

            if not raw_command:
                continue
            if raw_command in {"quit", "exit"}:
                break
            if raw_command == "help":
                print_local_help()
                continue

            try:
                if raw_command.startswith("drawFromStream,"):
                    handle_draw_from_stream(
                        ser,
                        raw_command,
                        drawings_dir=args.drawings_dir,
                        command_timeout=args.command_timeout,
                        move_window=args.move_window,
                    )
                    continue

                print(f"> {raw_command}")
                ser.write(f"{raw_command}\n".encode("utf-8"))
                ser.flush()
                read_until_quiet(ser, args.quiet_window)
            except Exception as exc:
                print(f"! {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
