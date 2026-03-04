# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

WiFi-based voice assistant firmware for **M5StickC Plus2** (+ Hat SPK2). Records audio from the built-in PDM microphone, sends it to OpenRouter's GPT Audio Mini API via HTTPS, and plays back the AI response through the Hat SPK2 speaker using streaming I2S output.

Main sketch: `voice_bridge_esp32/voice_bridge_esp32.ino`

## Build & Upload

**CRITICAL**: PSRAM must be disabled for M5StickC Plus2 (causes boot loop otherwise).

```bash
# Compile
arduino-cli compile --fqbn "m5stack:esp32:m5stack_stickc_plus2:PSRAM=disabled" voice_bridge_esp32/

# Upload (replace port as needed)
arduino-cli upload -p /dev/tty.usbserial-* --fqbn "m5stack:esp32:m5stack_stickc_plus2:PSRAM=disabled" voice_bridge_esp32/

# Monitor serial output
arduino-cli monitor -p /dev/tty.usbserial-* --config baudrate=115200
```

Required board package: `m5stack:esp32` (install via `arduino-cli core install m5stack:esp32`)
Required library: `M5StickCPlus2`

## Architecture

**State machine:** `IDLE → RECORDING → PROCESSING → PLAYING → IDLE`

- **BtnA**: Start/stop recording. After stop, sends to API and plays response.

**Critical hardware constraint:** The microphone and Hat SPK2 speaker share GPIO0 (LRC pin). They cannot run simultaneously. Always call `Mic.end()` before starting the speaker, and vice versa.

**Hat SPK2 I2S pin mapping:** DOUT=25, BCLK=26, LRC=0

### Streaming Playback

The firmware uses a **ring buffer + FreeRTOS task** architecture for gap-free playback:

1. **Producer** (main task / loopTask on core 1): SSE stream → base64 decode → 24→16 kHz downsample → ring buffer
2. **Consumer** (I2S writer task on core 0): ring buffer → `i2s_channel_write()` → DMA → speaker

M5Unified's `playRaw()` is intentionally bypassed — it stores a pointer (not copy) and its blocking mechanism causes audio gaps during streaming.

### Memory Layout (SRAM only, ~272 KB)

- Recording buffer: ~105 KB (dynamically sized, freed before playback)
- Ring buffer: ~78 KB (allocated from freed recording buffer space)
- I2S DMA: ~8 KB (8 × 256 frames, matching M5Unified defaults)
- WiFi + SSL: ~42 KB
- Free during playback: ~8 KB

### I2S Driver Notes

- Must use ESP-IDF v5 new API (`driver/i2s_std.h`), NOT legacy `driver/i2s.h`
- Legacy driver causes runtime abort: "CONFLICT! driver_ng is not allowed to be used with the legacy driver"
- Slot config matches M5Unified exactly: MONO + SLOT_BOTH, Philips format, 16-bit
- `#if SOC_I2S_HW_VERSION_1` handles ESP32 vs ESP32-S3 slot config differences

### OpenRouter API

- Endpoint: `https://openrouter.ai/api/v1/chat/completions`
- Model: `openai/gpt-audio-mini`
- **Must include `"stream":true`** — without it, returns HTTP 400
- Request: streaming base64 WAV in 3 KB chunks
- Response: SSE with transcript chunks first, then audio data chunks (base64 PCM16 @ 24 kHz)
- API key stored in NVS via `SET_KEY:` serial command
