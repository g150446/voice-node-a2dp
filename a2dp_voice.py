#!/usr/bin/env python3
"""
A2DP Voice Bridge — Mac companion for M5StickC Plus2

Usage:
    python a2dp_voice.py list
    python a2dp_voice.py pair        [--device M5StickC] [--scan-time 15]
    python a2dp_voice.py configure   --port /dev/tty.usbserial-*
    python a2dp_voice.py sink        [--device M5StickC] [--latency 0.1]
    python a2dp_voice.py source      [--device M5StickC] [--latency 0.1]

Dependencies:
    pip install -r requirements.txt   # sounddevice numpy pyserial
    brew install blueutil             # required for 'pair' subcommand

Modes:
    list        Print all audio devices
    pair        Scan, pair, and connect M5Stick over Classic BT (needs blueutil)
    configure   Write this Mac's BT name to M5Stick NVS via USB serial
                (required for SOURCE mode so M5Stick knows which Sink to connect to)
    sink        Mac default mic → M5StickC-SPK BT output (M5Stick plays audio)
    source      M5StickC-MIC BT input → Mac default speakers (hear M5Stick mic)

Pairing workflow:
    1. Switch M5Stick to SINK mode (BtnA) for initial pairing
    2. Grant Bluetooth to terminal: System Settings → Privacy & Security → Bluetooth
    3. python a2dp_voice.py pair
    4. python a2dp_voice.py configure --port /dev/tty.usbserial-*
    5. Press BtnB to switch M5Stick to SOURCE mode
"""

import argparse
import json
import shutil
import subprocess
import sys
import threading
import time
import numpy as np

try:
    import sounddevice as sd
except ImportError:
    sys.exit("sounddevice not installed. Run: pip install sounddevice numpy")


# ---------------------------------------------------------------------------
# Bluetooth pairing via blueutil
# ---------------------------------------------------------------------------

def _require_blueutil() -> str:
    """Return path to blueutil or exit with install instructions."""
    path = shutil.which("blueutil")
    if not path:
        sys.exit(
            "blueutil not found. Install it with:\n"
            "    brew install blueutil\n"
            "Then re-run: python a2dp_voice.py pair"
        )
    return path


def _run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True)


def _get_mac_bt_name() -> str:
    """Return this Mac's Bluetooth device name."""
    r = subprocess.run(
        ["defaults", "read", "/Library/Preferences/com.apple.Bluetooth", "Name"],
        capture_output=True, text=True,
    )
    name = r.stdout.strip()
    if not name:
        r2 = subprocess.run(["scutil", "--get", "ComputerName"], capture_output=True, text=True)
        name = r2.stdout.strip()
    return name or "unknown"


def bt_pair(device_name: str, scan_time: int) -> None:
    """
    Scan for a Classic Bluetooth device matching device_name, then pair and
    connect it.  Requires blueutil (brew install blueutil).
    """
    blueutil = _require_blueutil()

    mac_bt_name = _get_mac_bt_name()
    print(f"This Mac's BT name: {mac_bt_name!r}")
    print(f"(Needed for SOURCE mode — run 'python a2dp_voice.py configure' after pairing)\n")

    # 1. Make sure Bluetooth is on
    r = _run([blueutil, "--power"])
    if r.stdout.strip() == "0":
        print("Bluetooth is off — turning it on...")
        _run([blueutil, "--power", "1"])
        time.sleep(2)

    print(f"Scanning for '{device_name}' ({scan_time}s) — make sure M5Stick is powered on...")
    r = _run([blueutil, "--inquiry", str(scan_time), "--format", "json"])
    if r.returncode != 0:
        sys.exit(f"blueutil inquiry failed: {r.stderr.strip()}")

    try:
        found = json.loads(r.stdout)
    except json.JSONDecodeError:
        sys.exit(f"Could not parse blueutil output:\n{r.stdout}")

    partial_lower = device_name.lower()
    matches = [d for d in found if partial_lower in d.get("name", "").lower()]

    if not matches:
        print(f"\nNo device matching '{device_name}' found. "
              f"({len(found)} device(s) seen during scan)")
        if found:
            print("Devices seen:")
            for d in found:
                print(f"  {d.get('address', '?'):18}  {d.get('name', '(unnamed)')}")
        print()
        print("Troubleshooting:")
        print("  1. Grant Bluetooth permission to Terminal/iTerm2:")
        print("     System Settings → Privacy & Security → Bluetooth → enable your terminal")
        print("  2. For first-time pairing, switch M5Stick to SINK mode (press BtnA),")
        print("     then re-run this command. SINK mode is more reliably discoverable.")
        print("  3. Try a longer scan:  python a2dp_voice.py pair --scan-time 30")
        sys.exit(1)

    if len(matches) > 1:
        print(f"Multiple matches — using first:")
        for d in matches:
            print(f"  {d['address']}  {d['name']}")

    target = matches[0]
    addr   = target["address"]
    name   = target.get("name", addr)
    print(f"\nFound: {name}  [{addr}]")

    # 2. Pair (no-op if already paired)
    print("Pairing...")
    r = _run([blueutil, "--pair", addr])
    if r.returncode != 0:
        msg = (r.stdout + r.stderr).lower()
        if "already" in msg or "existing" in msg:
            print("Already paired.")
        else:
            sys.exit(f"Pairing failed: {r.stderr.strip()}")
    else:
        print("Paired.")
        time.sleep(1)

    # 3. Connect
    print("Connecting...")
    r = _run([blueutil, "--connect", addr])
    if r.returncode != 0:
        sys.exit(f"Connection failed: {r.stderr.strip()}")
    print(f"Connected to {name}.")
    print("\nNext steps:")
    print(f"  SINK mode  → System Settings → Sound → Output → select M5StickC-SPK")
    print(f"  SOURCE mode → python a2dp_voice.py configure --port /dev/tty.usbserial-*")
    print(f"               (writes Mac BT name {mac_bt_name!r} to M5Stick NVS)")


# ---------------------------------------------------------------------------
# Configure: write Mac BT name to M5Stick over serial
# ---------------------------------------------------------------------------

def configure(port: str) -> None:
    """Send Mac's BT name to M5Stick via serial so SOURCE mode can find it."""
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial not installed. Run: pip install pyserial")

    mac_bt_name = _get_mac_bt_name()
    print(f"Mac BT name: {mac_bt_name!r}")
    print(f"Writing to M5Stick on {port} ...")

    try:
        with serial.Serial(port, 115200, timeout=3) as ser:
            time.sleep(1.5)  # wait for M5Stick to be ready after opening port
            cmd = f"SET_REMOTE:{mac_bt_name}\n"
            ser.write(cmd.encode())
            ser.flush()
            response = ser.readline().decode(errors="replace").strip()
    except serial.SerialException as exc:
        sys.exit(f"Serial error: {exc}")

    if response.startswith("OK"):
        print(f"Done — M5Stick confirmed: {response}")
        print("M5Stick will restart into SOURCE mode automatically.")
        print("Then run: python a2dp_voice.py source")
    else:
        print(f"Unexpected response: {response!r}")
        print("Check that M5Stick is running the a2dp_voice sketch and connected via USB.")


# ---------------------------------------------------------------------------
# Audio device discovery
# ---------------------------------------------------------------------------

def list_devices() -> None:
    """Print all audio devices with index, name, and channel counts."""
    devices = sd.query_devices()
    print(f"{'Idx':>4}  {'In':>3}  {'Out':>3}  Name")
    print("-" * 60)
    for i, d in enumerate(devices):
        marker = ""
        if i == sd.default.device[0]:
            marker += " [default-in]"
        if i == sd.default.device[1]:
            marker += " [default-out]"
        print(f"{i:>4}  {d['max_input_channels']:>3}  {d['max_output_channels']:>3}  "
              f"{d['name']}{marker}")


def find_device(partial: str, kind: str) -> int:
    """
    Case-insensitive partial-name search across sd.query_devices().
    kind = 'input'  → requires max_input_channels  > 0
    kind = 'output' → requires max_output_channels > 0
    Raises RuntimeError if not found.
    """
    cap_key = "max_input_channels" if kind == "input" else "max_output_channels"
    partial_lower = partial.lower()
    matches = [
        (i, d)
        for i, d in enumerate(sd.query_devices())
        if partial_lower in d["name"].lower() and d[cap_key] > 0
    ]
    if not matches:
        raise RuntimeError(
            f"No {kind} device matching '{partial}' found.\n"
            "Run 'python a2dp_voice.py list' to see available devices."
        )
    if len(matches) > 1:
        names = ", ".join(f"{i}: {d['name']}" for i, d in matches)
        print(f"Warning: multiple {kind} matches — using first: {names}")
    return matches[0][0]


# ---------------------------------------------------------------------------
# SINK mode: Mac default mic → M5StickC-SPK BT output
# ---------------------------------------------------------------------------

def run_sink(device_name: str, latency: float) -> None:
    """
    Open Mac default mic as input and M5StickC BT as output.
    Forwards audio live so the M5Stick's Hat SPK2 plays it.
    """
    bt_out_idx = find_device(device_name, "output")
    bt_info = sd.query_devices(bt_out_idx)
    sample_rate = int(bt_info["default_samplerate"])
    channels = 2  # A2DP SBC requires stereo

    print(f"SINK mode  →  '{bt_info['name']}' (idx={bt_out_idx})")
    print(f"Sample rate: {sample_rate} Hz  |  Latency: {latency}s")
    print("Speak into your Mac mic — audio plays through M5Stick Hat SPK2")
    print("Press Ctrl+C to stop.\n")

    stop_event = threading.Event()

    def callback(indata: np.ndarray, outdata: np.ndarray,
                 frames: int, time, status) -> None:
        if status:
            print(f"[sink] {status}", file=sys.stderr)
        if indata.shape[1] == 1:
            # Mac mic is mono → duplicate to stereo
            outdata[:] = np.repeat(indata, 2, axis=1)
        else:
            out_ch = outdata.shape[1]
            in_ch  = indata.shape[1]
            if in_ch >= out_ch:
                outdata[:] = indata[:, :out_ch]
            else:
                outdata[:, :in_ch] = indata
                outdata[:, in_ch:] = 0

    try:
        with sd.Stream(
            device=(None, bt_out_idx),   # (input, output)
            samplerate=sample_rate,
            channels=(1, channels),      # 1 in (Mac mic), 2 out (BT)
            dtype="int16",
            latency=latency,
            callback=callback,
        ):
            stop_event.wait()
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as exc:
        sys.exit(f"Stream error: {exc}")


# ---------------------------------------------------------------------------
# SOURCE mode: M5StickC-MIC BT input → Mac default speakers
# ---------------------------------------------------------------------------

def run_source(device_name: str, latency: float) -> None:
    """
    Open M5StickC BT as input and Mac default speakers as output.
    Forwards audio live so you hear the M5Stick's mic through Mac speakers.
    """
    bt_in_idx = find_device(device_name, "input")
    bt_info   = sd.query_devices(bt_in_idx)
    sample_rate = int(bt_info["default_samplerate"])
    channels    = 2  # A2DP SBC stereo

    print(f"SOURCE mode  ←  '{bt_info['name']}' (idx={bt_in_idx})")
    print(f"Sample rate: {sample_rate} Hz  |  Latency: {latency}s")
    print("Speak near the M5Stick — audio plays through Mac speakers")
    print("Press Ctrl+C to stop.\n")

    stop_event = threading.Event()

    def callback(indata: np.ndarray, outdata: np.ndarray,
                 frames: int, time, status) -> None:
        if status:
            print(f"[source] {status}", file=sys.stderr)
        out_ch = outdata.shape[1]
        in_ch  = indata.shape[1]
        if in_ch >= out_ch:
            outdata[:] = indata[:, :out_ch]
        else:
            outdata[:, :in_ch] = indata
            outdata[:, in_ch:] = 0

    try:
        with sd.Stream(
            device=(bt_in_idx, None),    # (input, output)
            samplerate=sample_rate,
            channels=(channels, channels),
            dtype="int16",
            latency=latency,
            callback=callback,
        ):
            stop_event.wait()
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as exc:
        sys.exit(f"Stream error: {exc}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="A2DP Voice Bridge — Mac companion for M5StickC Plus2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="cmd")

    sub.add_parser("list", help="Print all audio devices")

    cfg_p = sub.add_parser("configure", help="Write Mac BT name to M5Stick NVS via USB serial (needed for SOURCE mode)")
    cfg_p.add_argument("--port", required=True,
                       help="Serial port, e.g. /dev/tty.usbserial-*")

    pair_p = sub.add_parser("pair", help="Scan, pair, and connect M5Stick over BT (needs blueutil)")
    pair_p.add_argument("--device",    default="M5StickC",
                        help="Partial BT device name (default: M5StickC)")
    pair_p.add_argument("--scan-time", type=int, default=15,
                        help="Inquiry scan duration in seconds (default: 15)")

    sink_p = sub.add_parser("sink", help="Mac mic → M5Stick Hat SPK2")
    sink_p.add_argument("--device",  default="M5StickC",
                        help="Partial BT device name to search for (default: M5StickC)")
    sink_p.add_argument("--latency", type=float, default=0.1,
                        help="Stream latency in seconds (default: 0.1)")

    src_p = sub.add_parser("source", help="M5Stick mic → Mac speakers")
    src_p.add_argument("--device",  default="M5StickC",
                       help="Partial BT device name to search for (default: M5StickC)")
    src_p.add_argument("--latency", type=float, default=0.1,
                       help="Stream latency in seconds (default: 0.1)")

    args = parser.parse_args()

    if args.cmd == "list":
        list_devices()
    elif args.cmd == "configure":
        configure(args.port)
    elif args.cmd == "pair":
        bt_pair(args.device, args.scan_time)
    elif args.cmd == "sink":
        run_sink(args.device, args.latency)
    elif args.cmd == "source":
        run_source(args.device, args.latency)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
