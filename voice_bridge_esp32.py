#!/usr/bin/env python3
"""
A2DP Voice Bridge — Mac companion for M5StickC Plus2

Usage:
    python voice_bridge_esp32.py list
    python voice_bridge_esp32.py pair        [--device M5StickC] [--scan-time 15]
    python voice_bridge_esp32.py configure   --port /dev/tty.usbserial-*
    python voice_bridge_esp32.py sink        [--device M5StickC] [--latency 0.1]
    python voice_bridge_esp32.py source      [--device M5StickC] [--latency 0.1]
    python voice_bridge_esp32.py transcribe  [--whisper-url http://localhost:9000]

Dependencies:
    pip install -r requirements.txt   # sounddevice numpy pyserial requests
    brew install blueutil             # required for 'pair' subcommand

Modes:
    list        Print all audio devices
    pair        Scan, pair, and connect M5Stick over Classic BT (needs blueutil)
    configure   Write this Mac's BT name to M5Stick NVS via USB serial
                (required for SOURCE mode so M5Stick knows which Sink to connect to)
    sink        Mac default mic → M5StickC-SPK BT output (M5Stick plays audio)
    source      M5StickC-MIC BT input → Mac default speakers (hear M5Stick mic)
    transcribe  Connect to M5StickC via BT serial, stream mic → Whisper STT
"""

import argparse
import glob as globmod
import io
import json
import os
import shutil
import struct
import subprocess
import sys
import queue
import threading
import time
import numpy as np

# Load OPENROUTER_API_KEY from environment (set in ~/.zshrc or shell)
OPENROUTER_API_KEY = os.environ.get("OPENROUTER_API_KEY", "")
if not OPENROUTER_API_KEY:
    # Try reading from ~/.zshrc as fallback
    zshrc = os.path.expanduser("~/.zshrc")
    if os.path.exists(zshrc):
        with open(zshrc) as f:
            for line in f:
                if "OPENROUTER_API_KEY" in line and "=" in line:
                    val = line.split("=", 1)[1].strip().strip("'\"")
                    if val and not val.startswith("$"):
                        OPENROUTER_API_KEY = val
                        break

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
            "Then re-run: python voice_bridge_esp32.py pair"
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


def _get_mac_bt_addr() -> str:
    """Return this Mac's BT MAC address as 'aa:bb:cc:dd:ee:ff'."""
    r = subprocess.run(
        ["system_profiler", "SPBluetoothDataType", "-json"],
        capture_output=True, text=True,
    )
    if r.returncode == 0:
        try:
            data = json.loads(r.stdout)
            bt_info = data.get("SPBluetoothDataType", [{}])[0]
            ctrl = bt_info.get("controller_properties", {})
            addr = ctrl.get("controller_address", "")
            if addr:
                return addr.lower()
        except (json.JSONDecodeError, IndexError, KeyError):
            pass

    # Fallback: blueutil --info
    blueutil = shutil.which("blueutil")
    if blueutil:
        r2 = subprocess.run([blueutil, "--format", "json", "--info"],
                            capture_output=True, text=True)
        if r2.returncode == 0:
            try:
                info = json.loads(r2.stdout)
                addr = info.get("address", "")
                if addr:
                    return addr.lower()
            except (json.JSONDecodeError, KeyError):
                pass

    return ""


def bt_pair(device_name: str, scan_time: int) -> None:
    """
    Scan for a Classic Bluetooth device matching device_name, then pair and
    connect it.  Requires blueutil (brew install blueutil).
    """
    blueutil = _require_blueutil()

    mac_bt_name = _get_mac_bt_name()
    print(f"This Mac's BT name: {mac_bt_name!r}")
    print(f"(Needed for SOURCE mode — run 'python voice_bridge_esp32.py configure' after pairing)\n")

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
        print("  3. Try a longer scan:  python voice_bridge_esp32.py pair --scan-time 30")
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
    print(f"  SOURCE mode → python voice_bridge_esp32.py configure --port /dev/tty.usbserial-*")
    print(f"               (writes Mac BT name {mac_bt_name!r} to M5Stick NVS)")


# ---------------------------------------------------------------------------
# Configure: write Mac BT name to M5Stick over serial
# ---------------------------------------------------------------------------

def configure(port: str) -> None:
    """Send Mac's BT name and MAC address to M5Stick via serial."""
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial not installed. Run: pip install pyserial")

    mac_bt_name = _get_mac_bt_name()
    mac_bt_addr = _get_mac_bt_addr()
    print(f"Mac BT name: {mac_bt_name!r}")
    print(f"Mac BT addr: {mac_bt_addr or '(not detected)'}")
    if not mac_bt_addr:
        sys.exit("ERROR: Could not detect this Mac's BT MAC address.\n"
                 "Install blueutil (brew install blueutil) or check system_profiler.")
    print(f"Writing to M5Stick on {port} ...")

    try:
        with serial.Serial(port, 115200, timeout=3) as ser:
            time.sleep(1.5)  # wait for M5Stick to be ready after opening port

            # 1. Send BT name (backward compatible)
            cmd = f"SET_REMOTE:{mac_bt_name}\n"
            ser.write(cmd.encode())
            ser.flush()
            deadline = time.time() + 5
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue
                print(f"  [{line}]")
                if line.startswith("OK"):
                    break

            # Wait for device to restart after SET_REMOTE
            time.sleep(3)

            # 2. Send BT MAC address
            cmd2 = f"SET_MAC:{mac_bt_addr}\n"
            ser.write(cmd2.encode())
            ser.flush()
            response = ""
            deadline = time.time() + 5
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue
                print(f"  [{line}]")
                if line.startswith("OK"):
                    response = line
                    break
                if line.startswith("ERROR") or line.startswith("Unknown"):
                    response = line
                    break
    except serial.SerialException as exc:
        sys.exit(f"Serial error: {exc}")

    if response.startswith("OK"):
        print(f"Done — M5Stick confirmed: {response}")
        print("M5Stick will restart. Switch to SOURCE mode then run:")
        print("  python voice_bridge_esp32.py source")
    else:
        print(f"No OK response received (last line: {response!r})")
        print("Check that M5Stick is running the voice_bridge_esp32 sketch and connected via USB.")


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
            "Run 'python voice_bridge_esp32.py list' to see available devices."
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
    HFP/SCO provides mono input (1 channel); expand to stereo for output.
    """
    try:
        bt_in_idx = find_device(device_name, "input")
    except RuntimeError:
        sys.exit(
            f"No input device matching '{device_name}' found.\n"
            "Ensure M5Stick is paired and:\n"
            "  1. System Settings → Sound → Input → select M5StickC-MIC\n"
            "  2. Press BtnB on M5Stick to switch to SOURCE mode"
        )
    bt_info     = sd.query_devices(bt_in_idx)
    sample_rate = int(bt_info["default_samplerate"])
    in_channels = bt_info["max_input_channels"]  # 1 for HFP/SCO

    # Mac output: use default device channel count
    mac_out_idx  = sd.default.device[1]
    mac_out_info = sd.query_devices(mac_out_idx)
    out_channels = min(mac_out_info["max_output_channels"], 2)

    print(f"SOURCE mode  ←  '{bt_info['name']}' (idx={bt_in_idx})")
    print(f"Sample rate: {sample_rate} Hz  |  In ch: {in_channels}  Out ch: {out_channels}")
    print(f"Latency: {latency}s")
    print("Speak near the M5Stick — audio plays through Mac speakers")
    print("Press Ctrl+C to stop.\n")

    stop_event = threading.Event()

    def callback(indata: np.ndarray, outdata: np.ndarray,
                 frames: int, time, status) -> None:
        if status:
            print(f"[source] {status}", file=sys.stderr)
        out_ch = outdata.shape[1]
        in_ch  = indata.shape[1]
        if in_ch == 1 and out_ch > 1:
            outdata[:] = np.repeat(indata, out_ch, axis=1)
        elif in_ch >= out_ch:
            outdata[:] = indata[:, :out_ch]
        else:
            outdata[:, :in_ch] = indata
            outdata[:, in_ch:] = 0

    try:
        with sd.Stream(
            device=(bt_in_idx, None),    # (input, output=default)
            samplerate=sample_rate,
            channels=(in_channels, out_channels),
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
# TRANSCRIBE: BT serial (SPP) → Whisper STT server
# ---------------------------------------------------------------------------

FRAME_MAGIC = b"\x55\xAA"
SAMPLE_RATE = 16000  # must match firmware MIC_SAMPLE_RATE


def _bt_addr_for_name(name_hint: str) -> str:
    """Look up paired BT device address by name via blueutil."""
    try:
        out = subprocess.check_output(
            ["blueutil", "--paired"], text=True, timeout=5
        )
        for line in out.split("\n"):
            if name_hint.lower() in line.lower():
                return line.split(",")[0].split(":")[1].strip()
    except Exception:
        pass
    return ""


def _bt_connect(bt_addr: str) -> None:
    """Ensure device is connected at L2CAP level via blueutil."""
    if not bt_addr:
        return
    addr_dash = bt_addr.replace(":", "-")
    try:
        print(f"Connecting BT device {addr_dash}...")
        subprocess.run(
            ["blueutil", "--connect", addr_dash],
            capture_output=True, text=True, timeout=10,
        )
        time.sleep(2)
    except Exception as exc:
        print(f"blueutil connect warning: {exc}")


def _ensure_bt_serial_port(name_hint: str = "M5StickC", bt_addr: str = "") -> str:
    """Find or create the macOS BT serial port via SDP discovery."""
    patterns = [
        f"/dev/tty.*{name_hint}*",
        "/dev/tty.*M5Stick*",
        "/dev/tty.M5StickC-MIC*",
    ]

    # Resolve BT address early so we can ensure connection
    if not bt_addr:
        bt_addr = _bt_addr_for_name(name_hint)

    # Ensure BT is connected before checking ports
    _bt_connect(bt_addr)

    # Check if port exists
    for pat in patterns:
        matches = globmod.glob(pat)
        if matches:
            return matches[0]

    # Port not found — try IOBluetooth SDP discovery
    if bt_addr:
        addr_colon = bt_addr.replace("-", ":")
        print(f"BT serial port not found. Triggering SDP discovery for {addr_colon}...")
        try:
            import objc
            objc.loadBundle(
                "IOBluetooth",
                bundle_path="/System/Library/Frameworks/IOBluetooth.framework",
                module_globals={},
            )
            device_cls = objc.lookUpClass("IOBluetoothDevice")
            device = device_cls.deviceWithAddressString_(addr_colon)
            if device:
                device.openConnection()
                time.sleep(2)
                device.performSDPQuery_(None)
                # Wait for serial port to appear
                for _ in range(10):
                    time.sleep(1)
                    for pat in patterns:
                        matches = globmod.glob(pat)
                        if matches:
                            print(f"Serial port created: {matches[0]}")
                            return matches[0]
                print("Warning: SDP discovery did not create serial port")
        except ImportError:
            print("pyobjc not available. Install: pip install pyobjc-framework-IOBluetooth")
        except Exception as exc:
            print(f"SDP discovery error: {exc}")

    all_bt = globmod.glob("/dev/tty.*")
    avail = [p for p in all_bt if "Bluetooth" not in p and "usbserial" not in p
             and "debug" not in p.lower()]
    raise RuntimeError(
        f"No BT serial port found matching '{name_hint}'.\n"
        f"Available ports: {avail}\n"
        "Make sure M5StickC is in SOURCE mode and paired via BT."
    )


def _verify_spp(ser) -> bool:
    """Send PING and wait for PONG to verify SPP is live."""
    ser.reset_input_buffer()
    ser.write(b"PING\n")
    deadline = time.time() + 3
    buf = b""
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if b"PONG" in buf:
                return True
        time.sleep(0.1)
    return False


def _force_reconnect_spp(name_hint: str = "M5StickC") -> str:
    """Unpair, re-pair, connect to force fresh RFCOMM. Returns port path."""
    bt_addr = _bt_addr_for_name(name_hint)
    if not bt_addr:
        raise RuntimeError(f"No paired BT device matching '{name_hint}'")
    addr_dash = bt_addr.replace(":", "-")
    print(f"Forcing BT reconnect for {addr_dash}...")
    subprocess.run(["blueutil", "--unpair", addr_dash],
                    capture_output=True, timeout=10)
    time.sleep(2)
    subprocess.run(["blueutil", "--pair", addr_dash],
                    capture_output=True, timeout=10)
    time.sleep(3)
    subprocess.run(["blueutil", "--connect", addr_dash],
                    capture_output=True, timeout=10)
    time.sleep(3)
    patterns = ["/dev/tty.*M5Stick*", "/dev/tty.M5StickC-MIC*"]
    for pat in patterns:
        matches = globmod.glob(pat)
        if matches:
            return matches[0]
    raise RuntimeError("BT serial port not found after reconnect")


def _read_spp_data(ser) -> tuple:
    """Read SPP data: either a PCM frame or a command line.
    Returns ('frame', pcm_bytes) or ('cmd', cmd_string) or ('empty', b'').
    """
    b = ser.read(1)
    if not b:
        return ('empty', b'')
    if b == b"\x55":
        b2 = ser.read(1)
        if b2 == b"\xAA":
            hdr = ser.read(2)
            if len(hdr) < 2:
                return ('empty', b'')
            length = struct.unpack(">H", hdr)[0]
            data = b""
            while len(data) < length:
                chunk = ser.read(length - len(data))
                if not chunk:
                    break
                data += chunk
            return ('frame', data)
        return ('empty', b'')
    if b == b'C':
        rest = ser.readline()
        line = (b'C' + rest).decode('utf-8', errors='replace').strip()
        if line.startswith('CMD:'):
            return ('cmd', line)
    return ('empty', b'')


def _pcm_to_wav(pcm_data: bytes, sample_rate: int = SAMPLE_RATE) -> bytes:
    """Wrap raw int16 PCM in a WAV header."""
    import wave
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm_data)
    return buf.getvalue()


def _send_to_whisper(wav_bytes: bytes, whisper_url: str) -> str:
    """POST a WAV file to the Whisper server, return transcribed text."""
    import requests
    try:
        resp = requests.post(
            f"{whisper_url}/transcribe",
            files={"audio_file": ("audio.wav", wav_bytes, "audio/wav")},
            timeout=30,
        )
        resp.raise_for_status()
        result = resp.json()
        # Handle both "text" and "transcription" response keys
        text = result.get("text", "") or result.get("transcription", "")
        return text.strip() if isinstance(text, str) else str(text).strip()
    except Exception as exc:
        return f"[Whisper error: {exc}]"


# TTS audio frame: [0xAA][0x55][len_hi][len_lo][pcm_data...]
TTS_MAGIC = b"\xAA\x55"

def _send_tts_frame(ser, pcm_chunk: bytes) -> None:
    """Send one TTS audio frame over SPP."""
    length = len(pcm_chunk)
    header = TTS_MAGIC + struct.pack(">H", length)
    ser.write(header + pcm_chunk)

def _send_tts_end(ser) -> None:
    """Send TTS end marker (zero-length frame)."""
    ser.write(TTS_MAGIC + b"\x00\x00")

def _downsample_chunk_3to2(samples_24k: np.ndarray) -> np.ndarray:
    """3:2 polyphase downsample 24kHz→16kHz. Input length must be multiple of 3."""
    assert len(samples_24k) % 3 == 0
    s = samples_24k.astype(np.float64).reshape(-1, 3)
    out = np.empty((len(s), 2), dtype=np.float64)
    out[:, 0] = s[:, 0]
    out[:, 1] = (s[:, 1] + s[:, 2]) * 0.5
    return out.reshape(-1).astype(np.int16)

def _text_to_speech_and_send(text: str, ser) -> None:
    """Send text to OpenRouter TTS, stream PCM audio back to M5StickC via SPP."""
    import requests, base64

    if not OPENROUTER_API_KEY:
        print("[TTS] No OPENROUTER_API_KEY found, skipping TTS")
        return

    print(f"[TTS] Generating speech for: {text[:60]}...")
    try:
        resp = requests.post(
            "https://openrouter.ai/api/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                "Content-Type": "application/json",
            },
            json={
                "model": "openai/gpt-audio-mini",
                "modalities": ["text", "audio"],
                "audio": {"voice": "alloy", "format": "pcm16"},
                "messages": [{"role": "user", "content": text}],
                "stream": True,
            },
            stream=True,
            timeout=60,
        )
        resp.raise_for_status()

        q: queue.Queue = queue.Queue(maxsize=16)
        carry_24k: bytearray = bytearray()
        all_pcm_debug: bytearray = bytearray()
        producer_error: list = []

        def _producer() -> None:
            try:
                for line in resp.iter_lines():
                    if not line:
                        continue
                    line_str = line.decode("utf-8")
                    if not line_str.startswith("data: "):
                        continue
                    payload = line_str[6:]
                    if payload == "[DONE]":
                        break
                    try:
                        chunk_json = json.loads(payload)
                        delta = chunk_json.get("choices", [{}])[0].get("delta", {})
                        if "audio" in delta and "data" in delta["audio"]:
                            pcm_raw = base64.b64decode(delta["audio"]["data"])
                            if not pcm_raw:
                                continue
                            all_pcm_debug.extend(pcm_raw)
                            carry_24k.extend(pcm_raw)
                            aligned = len(carry_24k) - (len(carry_24k) % 6)
                            if aligned >= 6:
                                samples_24k = np.frombuffer(bytes(carry_24k[:aligned]), dtype=np.int16)
                                q.put(_downsample_chunk_3to2(samples_24k).tobytes())
                                del carry_24k[:aligned]
                    except (json.JSONDecodeError, KeyError, IndexError):
                        pass
                # Flush leftover carry bytes
                if len(carry_24k) >= 2:
                    pad = (6 - len(carry_24k) % 6) % 6
                    carry_24k.extend(b"\x00" * pad)
                    samples_24k = np.frombuffer(bytes(carry_24k), dtype=np.int16)
                    q.put(_downsample_chunk_3to2(samples_24k).tobytes())
                    carry_24k.clear()
            except Exception as exc:
                producer_error.append(exc)
            finally:
                q.put(None)

        t = threading.Thread(target=_producer, daemon=True, name="tts-producer")
        t.start()
        print(f"[TTS] Streaming from OpenRouter (producer started)...")

        TTS_CHUNK = 512
        BATCH = 4
        send_buf: bytearray = bytearray()
        frame_count = 0
        sent = 0
        end_sent = False

        try:
            while True:
                try:
                    item = q.get(timeout=30)
                except queue.Empty:
                    print("[TTS] Producer timeout — aborting send")
                    break
                if item is None:
                    break
                send_buf.extend(item)
                while len(send_buf) >= TTS_CHUNK:
                    _send_tts_frame(ser, bytes(send_buf[:TTS_CHUNK]))
                    del send_buf[:TTS_CHUNK]
                    sent += TTS_CHUNK
                    frame_count += 1
                    time.sleep(0.01)          # B1 FIX: was 0.06
                    if frame_count % BATCH == 0:
                        while ser.in_waiting:
                            ser.read(ser.in_waiting)
            if send_buf:
                _send_tts_frame(ser, bytes(send_buf))
                sent += len(send_buf)
                frame_count += 1
                time.sleep(0.01)
            _send_tts_end(ser)
            end_sent = True
            print(f"[TTS] Sent {sent} bytes ({frame_count} frames) of 16kHz audio")
        except Exception as exc:
            print(f"[TTS] Consumer error: {exc}")
        finally:
            if not end_sent:
                _send_tts_end(ser)   # always unblock ESP32
            t.join(timeout=10)
            if t.is_alive():
                print("[TTS] Warning: producer thread still alive after join timeout")

        if producer_error:
            print(f"[TTS] Producer error: {producer_error[0]}")

        print(f"[TTS] Collected {len(all_pcm_debug)} bytes from OpenRouter")
        if all_pcm_debug:
            import wave
            debug_path = os.path.join(os.path.dirname(__file__) or ".", "tts_debug.wav")
            with wave.open(debug_path, "wb") as wf:
                wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(24000)
                wf.writeframes(bytes(all_pcm_debug))
            print(f"[TTS] Debug audio saved: {debug_path}")

    except Exception as exc:
        print(f"[TTS] Error: {exc}")
        _send_tts_end(ser)


def run_transcribe(
    whisper_url: str = "http://localhost:9000",
    bt_port: str = "",
    chunk_sec: float = 20.0,
    silence_thresh: float = 0.02,
) -> None:
    """
    Connect to M5StickC via BT serial, wait for button press to start/stop,
    receive mic PCM frames, detect voice activity, send to Whisper STT,
    and send transcription results back to the device for display.
    """
    import requests

    try:
        r = requests.get(f"{whisper_url}/health", timeout=5)
        print(f"Whisper server: {whisper_url} (status={r.status_code})")
    except Exception:
        sys.exit(f"Cannot reach Whisper server at {whisper_url}")

    try:
        import serial as pyserial
    except ImportError:
        sys.exit("pyserial not installed. Run: pip install pyserial")

    if not bt_port:
        bt_port = _ensure_bt_serial_port()
    print(f"Connecting to BT serial: {bt_port}")

    try:
        ser = pyserial.Serial(bt_port, 115200, timeout=1)
    except pyserial.SerialException as exc:
        sys.exit(f"Cannot open {bt_port}: {exc}")

    # Verify SPP is actually connected via PING/PONG
    print("Verifying SPP connection...")
    time.sleep(1)
    if not _verify_spp(ser):
        print("SPP not responding — forcing BT reconnect...")
        ser.close()
        bt_port = _force_reconnect_spp()
        print(f"Reconnected. Opening {bt_port}...")
        ser = pyserial.Serial(bt_port, 115200, timeout=1)
        time.sleep(1)
        if not _verify_spp(ser):
            sys.exit("SPP still not responding after reconnect. "
                     "Try rebooting M5StickC and re-running.")
    print("SPP connection verified.")

    print(f"Chunk duration: {chunk_sec}s | Silence threshold: {silence_thresh}")
    print("Press Ctrl+C to stop.\n")

    recording = False
    pcm_buffer = bytearray()
    max_chunk_bytes = int(chunk_sec * SAMPLE_RATE * 2)
    silence_bytes = int(1.5 * SAMPLE_RATE * 2)
    consecutive_silent = 0

    try:
        while True:
            msg_type, data = _read_spp_data(ser)

            if msg_type == 'cmd':
                if data == 'CMD:START':
                    recording = True
                    pcm_buffer.clear()
                    consecutive_silent = 0
                    print("[REC] Started — listening...")
                elif data == 'CMD:STOP':
                    recording = False
                    # Process remaining buffer
                    if len(pcm_buffer) > SAMPLE_RATE:
                        all_samples = np.frombuffer(bytes(pcm_buffer), dtype=np.int16).astype(np.float32) / 32768.0
                        total_rms = np.sqrt(np.mean(all_samples ** 2))
                        if total_rms > silence_thresh * 0.5:
                            wav = _pcm_to_wav(bytes(pcm_buffer))
                            text = _send_to_whisper(wav, whisper_url)
                            if text and text.strip():
                                print(f">>> {text}")
                                ser.write((text.strip() + "\n").encode('utf-8'))
                                _text_to_speech_and_send(text.strip(), ser)
                    pcm_buffer.clear()
                    print("[REC] Stopped")
                continue

            if msg_type == 'empty' or not recording:
                continue

            if msg_type == 'frame' and data:
                pcm_buffer.extend(data)

                samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                rms = np.sqrt(np.mean(samples ** 2))

                if rms < silence_thresh:
                    consecutive_silent += len(data)
                else:
                    consecutive_silent = 0

                should_send = (
                    len(pcm_buffer) >= max_chunk_bytes
                    or (len(pcm_buffer) > SAMPLE_RATE
                        and consecutive_silent >= silence_bytes)
                )

                if should_send:
                    all_samples = np.frombuffer(bytes(pcm_buffer), dtype=np.int16).astype(np.float32) / 32768.0
                    total_rms = np.sqrt(np.mean(all_samples ** 2))

                    if total_rms > silence_thresh * 0.5:
                        wav = _pcm_to_wav(bytes(pcm_buffer))
                        text = _send_to_whisper(wav, whisper_url)
                        if text and text.strip():
                            print(f">>> {text}")
                            ser.write((text.strip() + "\n").encode('utf-8'))
                            _text_to_speech_and_send(text.strip(), ser)

                    pcm_buffer.clear()
                    consecutive_silent = 0

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


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

    tr_p = sub.add_parser("transcribe", help="M5Stick mic → Whisper STT via BT serial")
    tr_p.add_argument("--whisper-url", default="http://localhost:9000",
                      help="Whisper STT server URL (default: http://localhost:9000)")
    tr_p.add_argument("--bt-port", default="",
                      help="BT serial port (auto-detected if omitted)")
    tr_p.add_argument("--chunk-sec", type=float, default=20.0,
                      help="Max audio chunk duration in seconds (default: 20.0)")
    tr_p.add_argument("--silence-thresh", type=float, default=0.02,
                      help="RMS silence threshold for VAD (default: 0.02)")

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
    elif args.cmd == "transcribe":
        run_transcribe(args.whisper_url, args.bt_port, args.chunk_sec, args.silence_thresh)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
