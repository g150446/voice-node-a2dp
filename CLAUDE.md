# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Arduino firmware for **M5StickC Plus2** that records audio from the built-in microphone and plays it back through the **Hat SPK2** external speaker.

Single sketch: `mic_playback_stickc/mic_playback_stickc.ino`

## Build & Upload

Using Arduino CLI:

```bash
# Compile
arduino-cli compile --fqbn m5stack:esp32:m5stack_stickc_plus2 mic_playback_stickc/

# Upload (replace /dev/tty.usbserial-* with actual port)
arduino-cli upload -p /dev/tty.usbserial-* --fqbn m5stack:esp32:m5stack_stickc_plus2 mic_playback_stickc/

# Monitor serial output
arduino-cli monitor -p /dev/tty.usbserial-* --config baudrate=115200
```

Required board package: `m5stack:esp32` (install via `arduino-cli core install m5stack:esp32`)
Required library: `M5StickCPlus2`

## Architecture

**State machine:** `IDLE → RECORDING → PLAYING → IDLE`

- **BtnA**: Start/stop recording (toggles IDLE ↔ RECORDING)
- **BtnB**: Start/stop playback (toggles IDLE ↔ PLAYING, only if audio recorded)

**Critical hardware constraint:** The microphone and Hat SPK2 speaker share GPIO0 (LRC pin). They cannot run simultaneously. Always call `Speaker.end()` before `Mic.begin()` and vice versa.

**Hat SPK2 I2S pin mapping:** DOUT=25, BCLK=26, LRC=0

**Audio buffer:** Allocated on heap via `heap_caps_malloc` — 48000 × int16_t (~96 KB), providing ~3 seconds at 16 kHz. Buffer is statically sized; recording auto-stops when full.

**Speaker initialization:** Must set `cfg.external_speaker.hat_spk2 = true` and `cfg.internal_spk = false` in `M5.config()` before `StickCP2.begin(cfg)`. This is done once in `setup()`.

**Display:** Runs in landscape (`setRotation(1)`). Three draw functions map to the three states: `drawIdle()`, `drawRecording()`, `drawPlaying()`.
