# Voice Node A2DP — M5StickC Plus2 Bluetooth Audio Bridge

Bluetooth audio bridge between **M5StickC Plus2** (+ Hat SPK2) and a Mac.
Supports bidirectional audio streaming and **real-time speech-to-text** via Whisper.

## Architecture

```
┌──────────────────┐       Bluetooth        ┌──────────────┐
│ M5StickC Plus2   │◄──────────────────────►│  Mac          │
│ + Hat SPK2       │  A2DP (SINK mode)      │              │
│                  │  SPP serial (SOURCE)   │  voice_bridge_esp32.py
│  PDM Mic (16kHz) │────── SPP RFCOMM ─────►│  → Whisper STT│
└──────────────────┘                        └──────────────┘
```

### Modes

| Mode | Direction | Transport | Use Case |
|------|-----------|-----------|----------|
| **SINK** | Mac → M5Stick | A2DP | Play Mac audio through Hat SPK2 speaker |
| **SOURCE** | M5Stick → Mac | BT SPP (serial) | Stream mic audio to Mac for Whisper STT |

Mode is selected on the device via **BtnA** (SINK) or **BtnB** (SOURCE), stored in NVS, and requires a reboot to switch.

## Repository Structure

```
├── voice_bridge_esp32/
│   └── voice_bridge_esp32.ino      # Arduino firmware for M5StickC Plus2
├── voice_bridge_esp32.py            # Python companion script (Mac side)
├── mic_playback_stickc/
│   └── mic_playback_stickc.ino  # Standalone mic→speaker test sketch
├── requirements.txt         # Python dependencies
├── CLAUDE.md                # Build instructions and hardware notes
└── README.md                # This file
```

### Firmware (`voice_bridge_esp32/voice_bridge_esp32.ino`)

Single Arduino sketch with two runtime modes:

- **SINK mode**: Uses `BluetoothA2DPSink` to receive A2DP audio from Mac and play through Hat SPK2 via I2S.
- **SOURCE mode**: Uses `BluetoothSerial` (SPP) to stream 16kHz mono PCM from the built-in PDM microphone over Bluetooth serial to the Mac.

The SPP audio frame protocol:
```
[0x55][0xAA][len_hi][len_lo][pcm_data...]
```
- Magic: `0x55AA` (2 bytes)
- Length: uint16 big-endian (PCM bytes following)
- PCM: int16_t little-endian, 16kHz mono, 512 samples per frame (~32ms)

### Python Companion (`voice_bridge_esp32.py`)

Mac-side CLI tool with these subcommands:

| Command | Description |
|---------|-------------|
| `list` | Print all audio devices |
| `pair` | Scan, pair, and connect M5Stick via Bluetooth |
| `configure` | Write Mac's BT name to M5Stick NVS via USB serial |
| `sink` | Mac mic → M5Stick speaker (A2DP) |
| `source` | M5Stick mic → Mac speakers |
| `transcribe` | **M5Stick mic → Whisper STT via BT serial** |

## Quick Start

### Prerequisites

- **Hardware**: M5StickC Plus2 with Hat SPK2
- **Arduino**: `arduino-cli` with `m5stack:esp32` board package and `M5StickCPlus2` library
- **Mac**: Python 3.10+, `blueutil` (`brew install blueutil`)
- **Whisper**: A Whisper STT server (e.g. [faster-whisper-server](https://github.com/fedirz/faster-whisper-server)) running on `http://localhost:9000`

### 1. Flash Firmware

```bash
arduino-cli compile --fqbn m5stack:esp32:m5stack_stickc_plus2 voice_bridge_esp32/
arduino-cli upload -p /dev/tty.usbserial-* --fqbn m5stack:esp32:m5stack_stickc_plus2 voice_bridge_esp32/
```

### 2. Install Python Dependencies

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

### 3. Pair the Device

Press **BtnB** on M5StickC to enter SOURCE mode (requires reboot). Then:

```bash
python voice_bridge_esp32.py pair --device M5StickC
```

> **Note**: macOS Bluetooth Settings may show the device as "Not Connected" after pairing. This is normal — the `transcribe` command establishes its own SPP connection via IOBluetooth SDP discovery. You don't need the device to show "Connected" in System Settings.

### 4. Run Speech-to-Text

```bash
# Start your Whisper server first, then:
python voice_bridge_esp32.py transcribe --whisper-url http://localhost:9000
```

The script will:
1. Auto-discover the BT serial port (triggers macOS SDP discovery if needed)
2. Connect to M5StickC via SPP
3. Stream mic audio and detect voice activity
4. Send speech segments to the Whisper server
5. Print transcriptions in real-time as `>>> transcribed text`

### Options

```
--whisper-url URL    Whisper server URL (default: http://localhost:9000)
--bt-port PORT       BT serial port (auto-detected if omitted)
--chunk-sec SEC      Max audio chunk duration (default: 3.0s)
--silence-thresh N   RMS silence threshold for VAD (default: 0.02)
```

## Serial Commands

Connect to the USB serial port at 115200 baud to control the device:

| Command | Description |
|---------|-------------|
| `status` | Print current mode, BT state, remote name |
| `sink` | Switch to SINK mode (restarts) |
| `source` | Switch to SOURCE mode (restarts) |
| `SET_REMOTE:<name>` | Store Mac BT name in NVS (for SINK mode) |
| `help` | Print command list |

## Hardware Notes

- **GPIO0** is shared between the PDM microphone (WS/clock) and Hat SPK2 (LRC). They cannot run simultaneously.
- The speaker must be stopped (`Speaker.end()`) before starting the mic (`Mic.begin()`), and vice versa.
- Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC=0
- M5StickC Plus2 BT address: base MAC + 2
