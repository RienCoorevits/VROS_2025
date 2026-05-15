#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Run this script with a Python that has pyserial, "
        "for example ~/.platformio/penv/bin/python."
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream a repo-local drawing file to the robot over serial."
    )
    parser.add_argument("drawing_file", type=Path, help="Path to the drawing file to stream")
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
        help="Seconds to wait for the firmware to acknowledge each command",
    )
    parser.add_argument(
        "--return-to-origin",
        action="store_true",
        help="Send returnToOrigin after the drawing file is finished",
    )
    parser.add_argument(
        "--move-window",
        type=int,
        default=6,
        help="Maximum number of streamed move commands to keep in flight",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Repeat the full drawing file this many times",
    )
    return parser.parse_args()


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def iter_commands(drawing_path: Path):
    for raw_line in drawing_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        yield line


def print_available_ports() -> None:
    ports = available_ports()
    if not ports:
        print("No serial ports detected.", file=sys.stderr)
        return

    print("Available serial ports:", file=sys.stderr)
    for port in ports:
        print(f"  {port}", file=sys.stderr)


def open_serial_port(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(port, baud, timeout=0.25, write_timeout=5)
    except serial.SerialException as exc:
        print(f"Could not open serial port {port}: {exc}", file=sys.stderr)
        print_available_ports()
        raise SystemExit(1) from exc


def drain_serial(ser: serial.Serial, quiet: bool = False) -> None:
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
        if line and not quiet:
            print(f"< {line}")


def expect_startup_quiet(ser: serial.Serial, settle_time: float = 0.5) -> None:
    deadline = time.monotonic() + settle_time
    while time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
        if line:
            print(f"< {line}")
            deadline = time.monotonic() + settle_time


def send_and_wait_for_ok(ser: serial.Serial, command: str, timeout: float) -> None:
    print(f"> {command}")
    ser.write(f"{command}\n".encode("utf-8"))
    ser.flush()
    wait_for_ok(ser, command, timeout)


def wait_for_ok(ser: serial.Serial, command: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
        if not line:
            continue
        print(f"< {line}")
        if line.startswith("error\t") or line.startswith("error "):
            raise RuntimeError(f"Firmware rejected command '{command}': {line}")
        if line == "Invalid command" or line.startswith("command unavailable"):
            raise RuntimeError(f"Firmware rejected command '{command}': {line}")
        if line == "ok":
            return

    raise TimeoutError(f"Timed out waiting for 'ok' after sending: {command}")


def is_control_instruction(command: str) -> bool:
    return command.startswith("type\t") or command.startswith("mode\t") or command.startswith("adjustment\t")


def is_move_instruction(command: str) -> bool:
    return command.startswith("move\t")


def expand_repeated_commands(commands: list[str], repeat_count: int) -> list[str]:
    if repeat_count < 1:
        raise ValueError("--repeat must be at least 1.")
    if repeat_count == 1:
        return commands
    return commands * repeat_count


def send_control_batch(ser: serial.Serial, commands: list[str], timeout: float) -> None:
    if not commands:
        return

    for command in commands:
        print(f"> {command}")
        ser.write(f"{command}\n".encode("utf-8"))
    ser.flush()

    for command in commands:
        wait_for_ok(ser, command, timeout)


def drain_in_flight_moves(ser: serial.Serial, in_flight_moves: list[str], timeout: float) -> None:
    if not in_flight_moves:
        return

    ser.flush()
    while in_flight_moves:
        wait_for_ok(ser, in_flight_moves.pop(0), timeout)


def prepare_serial_port(ser: serial.Serial, startup_delay: float) -> None:
    time.sleep(startup_delay)
    drain_serial(ser)
    expect_startup_quiet(ser)


def stream_commands(
    ser: serial.Serial,
    commands: list[str],
    command_timeout: float,
    move_window: int,
    return_to_origin: bool = False,
) -> None:
    pending_control_commands: list[str] = []
    in_flight_moves: list[str] = []
    for command in commands:
        if is_control_instruction(command):
            # The firmware rejects control changes while streamed moves are still queued.
            drain_in_flight_moves(ser, in_flight_moves, command_timeout)
            pending_control_commands.append(command)
            continue

        if pending_control_commands:
            send_control_batch(ser, pending_control_commands, command_timeout)
            pending_control_commands.clear()

        if is_move_instruction(command):
            print(f"> {command}")
            ser.write(f"{command}\n".encode("utf-8"))
            in_flight_moves.append(command)
            if len(in_flight_moves) >= move_window:
                ser.flush()
                wait_for_ok(ser, in_flight_moves.pop(0), command_timeout)
            continue

        drain_in_flight_moves(ser, in_flight_moves, command_timeout)
        send_and_wait_for_ok(ser, command, command_timeout)

    if pending_control_commands:
        send_control_batch(ser, pending_control_commands, command_timeout)

    drain_in_flight_moves(ser, in_flight_moves, command_timeout)

    if return_to_origin:
        send_and_wait_for_ok(ser, "returnToOrigin", command_timeout)


def main() -> int:
    args = parse_args()

    if not args.drawing_file.is_file():
        print(f"Drawing file not found: {args.drawing_file}", file=sys.stderr)
        return 1

    if not args.port:
        print("--port is required.", file=sys.stderr)
        print_available_ports()
        return 1
    if args.move_window < 1:
        print("--move-window must be at least 1.", file=sys.stderr)
        return 1
    if args.repeat < 1:
        print("--repeat must be at least 1.", file=sys.stderr)
        return 1

    commands = list(iter_commands(args.drawing_file))
    if not commands:
        print("Drawing file contains no streamable commands.", file=sys.stderr)
        return 1
    commands = expand_repeated_commands(commands, args.repeat)

    with open_serial_port(args.port, args.baud) as ser:
        print(f"Opened {args.port} at {args.baud} baud")
        prepare_serial_port(ser, args.startup_delay)
        stream_commands(
            ser,
            commands,
            command_timeout=args.command_timeout,
            move_window=args.move_window,
            return_to_origin=args.return_to_origin,
        )

    print("Stream complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
