# Voice Bridge ESP32 — WiFi Voice Assistant

WiFi-based voice assistant on **M5StickC Plus2** (+ Hat SPK2) that records speech, sends it to **OpenRouter GPT Audio Mini** via HTTPS, and plays back the AI response through the speaker — all on-device with no companion app needed.

## Architecture

```
┌──────────────────────┐    HTTPS/SSE     ┌─────────────────────┐
│  M5StickC Plus2      │◄───── WiFi ─────►│  OpenRouter API     │
│  + Hat SPK2          │                   │  (gpt-audio-mini)   │
│                      │                   │                     │
│  PDM Mic → WAV+b64 ──┼──── Request ────►│  Audio input        │
│  Hat SPK2 ◄── PCM ───┼──── SSE stream ──│  Audio + text output│
└──────────────────────┘                   └─────────────────────┘
```

### How It Works

1. **Record**: Press **BtnA** to start recording from the built-in PDM microphone (16 kHz, 16-bit mono). Press **BtnA** again to stop, or recording auto-stops when the buffer is full (~3.3 s on SRAM-only devices).
2. **Send**: The firmware base64-encodes the WAV data in 3 KB streaming chunks and sends it to OpenRouter's `/api/v1/chat/completions` endpoint with `stream: true`.
3. **Receive & Play**: The SSE response streams audio chunks (base64 PCM16 at 24 kHz). The firmware decodes, downsamples to 16 kHz, and writes to a **ring buffer**. A dedicated FreeRTOS task drains the ring buffer to the I2S speaker DMA — ensuring smooth playback even during network jitter.

### Streaming Playback Design

The key design challenge was smooth audio playback while receiving data over WiFi. Three approaches were tested:

| Approach | Problem |
|----------|---------|
| M5Unified `playRaw()` | Stores pointer, not copy. Pointer-lifetime and blocking issues cause audio gaps |
| Direct `i2s_channel_write()` | Single-threaded — network pauses drain the DMA buffer causing silence |
| **Ring buffer + I2S task** ✅ | Producer-consumer decoupling eliminates gaps |

**Final architecture**:
- **Producer** (main task, core 1): reads SSE stream → base64 decode → 24→16 kHz downsample → ring buffer write
- **Consumer** (I2S task, core 0): ring buffer read → `i2s_channel_write()` to DMA
- **Ring buffer**: ~78 KB heap allocation (~2.4 s of 16 kHz mono audio), providing large jitter tolerance
- **DMA**: 8 × 256 frames (matches M5Unified defaults), auto-clear enabled

### Memory Budget (M5StickC Plus2, SRAM only)

| Resource | Size | Notes |
|----------|------|-------|
| Total SRAM heap | ~272 KB | PSRAM disabled (causes boot loop on this device) |
| WiFi + SSL overhead | ~42 KB | WiFiClientSecure with mbedTLS |
| Audio recording buffer | ~105 KB | Dynamically sized = largest free block − 4 KB |
| Ring buffer (during playback) | ~78 KB | Allocated after freeing recording buffer |
| I2S DMA | ~8 KB | 8 descriptors × 256 frames × 4 bytes |
| Free during playback | ~8 KB | For SSL cleanup and small allocations |

**Recording time**: ~3.3 s at 16 kHz (105 KB / 2 bytes per sample / 16000 Hz)

## Extending Recording Time with PSRAM

Devices with PSRAM (e.g., **M5Atom Echo S3R** with 8 MB PSRAM) can dramatically increase recording time:

### What Changes with PSRAM

| Parameter | SRAM only (current) | With 8 MB PSRAM |
|-----------|---------------------|-------------------|
| Recording buffer | ~105 KB (~3.3 s) | 1–4 MB (30–120 s) |
| Ring buffer | ~78 KB (~2.4 s) | 256+ KB (8+ s jitter tolerance) |
| Max recording time | ~3.3 s | 30+ s (adjustable) |
| FQBN option | `PSRAM=disabled` | `PSRAM=enabled` (default) |

### How to Adapt the Code

1. **Enable PSRAM**: Change the build FQBN to remove `PSRAM=disabled`:
   ```bash
   arduino-cli compile --fqbn m5stack:esp32s3:m5stack_atoms3r voice_bridge_esp32/
   ```

2. **Use PSRAM for large buffers**: Replace `malloc()` with `ps_malloc()` (or `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`) for `audioBuf` and `ringBuf` in `setup()` and `callOpenRouter()`.

3. **Increase recording time**: The recording length is determined by `audioBufBytes / sizeof(int16_t) / SAMPLE_RATE`. With PSRAM, allocate a larger buffer:
   ```cpp
   audioBufBytes = 960000;  // 960 KB = 30s at 16kHz
   audioBuf = (int16_t*)ps_malloc(audioBufBytes);
   ```

4. **Update I2S pin mapping**: The M5Atom Echo S3R has a different speaker (built-in NS4168, no Hat SPK2). Update `SPK_BCLK`, `SPK_LRC`, `SPK_DOUT` to match the new device's GPIO assignments. Also check if the microphone uses I2S instead of the M5StickC's PDM, and update mic configuration accordingly.

5. **ESP32-S3 I2S differences**: The S3 uses `SOC_I2S_HW_VERSION_2`, so the slot config fields differ slightly (uses `left_align`, `big_endian`, `bit_order_lsb` instead of `msb_right`). The `#if SOC_I2S_HW_VERSION_1` preprocessor guard in `i2sSpkBegin()` already handles this.

## Repository Structure

```
├── voice_bridge_esp32/
│   └── voice_bridge_esp32.ino   # Arduino firmware (WiFi voice assistant)
├── voice_bridge_esp32.py         # Legacy Python companion (Bluetooth mode)
├── hfp_headset/                  # HFP headset experiment
├── requirements.txt              # Python dependencies (legacy)
├── CLAUDE.md                     # Build instructions and hardware notes
└── README.md                     # This file
```

## Build & Upload

### Prerequisites

- **Hardware**: M5StickC Plus2 with Hat SPK2
- **Arduino CLI**: with `m5stack:esp32` board package and `M5StickCPlus2` library
- **WiFi**: 2.4 GHz network (hotspot or router)
- **API Key**: OpenRouter API key with access to `openai/gpt-audio-mini`

### 1. Set WiFi Credentials

Edit the `WIFI_SSID` and `WIFI_PASS` constants near the top of `voice_bridge_esp32.ino`.

### 2. Flash Firmware

```bash
arduino-cli compile --fqbn "m5stack:esp32:m5stack_stickc_plus2:PSRAM=disabled" voice_bridge_esp32/
arduino-cli upload -p /dev/tty.usbserial-* --fqbn "m5stack:esp32:m5stack_stickc_plus2:PSRAM=disabled" voice_bridge_esp32/
```

> **Important**: PSRAM must be disabled for M5StickC Plus2 — enabling it causes an infinite boot loop (`RTCWDT_RTC_RESET`).

### 3. Set API Key

Via USB serial at 115200 baud:
```
SET_KEY:sk-or-v1-your-key-here
```

The key is stored in NVS (non-volatile storage) and persists across reboots.

### 4. Use

- **BtnA**: Start/stop recording. After recording stops, the audio is sent to the API and the response plays automatically.
- The display shows recording progress, status messages, and WiFi state.

## Serial Commands

| Command | Description |
|---------|-------------|
| `status` | Print state, WiFi IP, API key status, free heap |
| `SET_KEY:<key>` | Store OpenRouter API key in NVS |
| `help` | Print command list |

## Hardware Notes

- **GPIO0** is shared between the PDM microphone (WS/clock) and Hat SPK2 (LRC). They cannot run simultaneously — `Mic.end()` must be called before starting the speaker, and vice versa.
- Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC=0
- Hat SPK2 uses NS4168 amplifier with L/R pin grounded
- The I2S driver uses ESP-IDF v5's new API (`driver/i2s_std.h`). The legacy `driver/i2s.h` cannot coexist with M5Unified.
- M5Unified's Speaker is intentionally bypassed during TTS playback. Direct I2S via `i2s_new_channel()` / `i2s_channel_write()` avoids pointer-stability and blocking issues in `playRaw()`.
