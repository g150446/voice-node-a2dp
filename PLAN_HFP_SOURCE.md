# HFP SOURCE Mode — Implementation Plan

## Status of existing code

- **SINK mode** (`BluetoothA2DPSink` + manual IDF5 I2S on I2S_NUM_1): works correctly.
- **SOURCE mode** (`BluetoothA2DPSource`): broken on macOS — AVDTP always reaches
  `CONNECTING → DISCONNECTED`. This is a hard macOS limitation; macOS cannot act as
  an A2DP Sink for incoming connections.
- **Fix:** Replace SOURCE mode with HFP (Hands-Free Profile). M5Stick = HF client,
  Mac = Audio Gateway (AG). SCO carries 8 kHz CVSD or 16 kHz mSBC mono PCM.

---

## Files to change

| File | Change |
|------|--------|
| `a2dp_voice/a2dp_voice.ino` | Replace `BluetoothA2DPSource` with HFP client (ESP-IDF raw API). |
| `a2dp_voice.py` | `configure` command sends Mac BT MAC address; `run_source()` handles mono SCO channels. |

---

## Part 1 — Firmware (`a2dp_voice.ino`)

### 1.1 Removed includes / globals

Remove:
```cpp
#include <BluetoothA2DPSource.h>
```

Add:
```cpp
#include "esp_hf_client_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
```

New global in the SOURCE section:
```cpp
static esp_bd_addr_t  hfpPeerAddr;        // Mac BT MAC, read from NVS
static volatile bool  hfpScoConnected = false;
static uint32_t       hfpSampleRate   = 8000; // 8000=CVSD, 16000=mSBC
```

Keep the ring buffer and `ringMux` — they are still used (micTask → HFP send callback).

### 1.2 NVS: store Mac BT MAC address

Add a new NVS key for the 6-byte hardware address:
```cpp
static const char* PREFS_KEY_MAC = "mac_addr"; // 6-byte blob
```

New helper to read it:
```cpp
static bool readMacAddr(esp_bd_addr_t out) {
    Preferences p;
    p.begin(PREFS_NS, true);
    size_t n = p.getBytes(PREFS_KEY_MAC, out, 6);
    p.end();
    return (n == 6);
}
```

New helper to write it (called from serial command handler):
```cpp
static void writeMacAddr(const uint8_t* mac6) {
    Preferences p;
    p.begin(PREFS_NS, false);
    p.putBytes(PREFS_KEY_MAC, mac6, 6);
    p.end();
}
```

### 1.3 Serial command: `SET_MAC:<xx:xx:xx:xx:xx:xx>`

In `handleSerial()`, add a new branch alongside `SET_REMOTE`:
```
SET_MAC:aa:bb:cc:dd:ee:ff
```
Parse the six hex octets, call `writeMacAddr()`, print `OK mac=...`, restart.

The existing `SET_REMOTE:<name>` command can be kept for compatibility or removed
(A2DP source no longer needs it).

### 1.4 BT stack init for SOURCE / HFP mode

`BluetoothA2DPSource` no longer handles BT initialisation in SOURCE mode.
`setupSourceMode()` must do it manually (same pattern used internally by the ESP32
Arduino BT libraries):

```
esp_bt_controller_config_t → esp_bt_controller_init() → esp_bt_controller_enable()
esp_bluedroid_init() → esp_bluedroid_enable()
esp_hf_client_register_callback(hfClientCallback)
esp_hf_client_init()
esp_hf_client_register_data_callback(hfRecvCb, hfSendCb)
esp_bt_dev_set_device_name("M5StickC-MIC")
esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE)
esp_hf_client_connect(hfpPeerAddr)   // initiate SLC to Mac
```

All of this goes into a new function `setupSourceModeHfp()` that replaces the old
`setupSourceMode()`.

### 1.5 HFP event callback (`hfClientCallback`)

```cpp
static void hfClientCallback(esp_hf_client_cb_event_t event,
                              esp_hf_client_cb_param_t* param) {
    switch (event) {

    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        btConnected = (param->conn_stat.state
                       == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED);
        if (!btConnected) { hfpScoConnected = false; audioActive = false; }
        Serial.printf("[HFP] SLC: %d\n", param->conn_stat.state);
        // Once SLC is up, request SCO:
        if (btConnected)
            esp_hf_client_audio_connect(hfpPeerAddr);
        break;

    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        switch (param->audio_stat.state) {
        case ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC:
            hfpSampleRate = 16000;
            /* fall-through */
        case ESP_HF_CLIENT_AUDIO_STATE_CONNECTED:
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED)
                hfpSampleRate = 8000;
            hfpScoConnected = true;
            audioActive     = true;
            StickCP2.Mic.begin();   // start mic at hfpSampleRate
            // micTask already running; it uses hfpSampleRate
            Serial.printf("[HFP] SCO connected, codec=%s\n",
                hfpSampleRate == 16000 ? "mSBC" : "CVSD");
            break;

        case ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED:
            hfpScoConnected = false;
            audioActive     = false;
            StickCP2.Mic.end();
            Serial.println("[HFP] SCO disconnected");
            break;
        }
        break;

    default: break;
    }
}
```

### 1.6 SCO PCM callbacks

```cpp
// Called by BT task when Mac sends PCM to us (earpiece direction — discard).
static void hfRecvCb(const uint8_t* buf, uint32_t size) {
    (void)buf; (void)size;   // M5Stick is mic-only
}

// Called by BT task when it wants mic PCM to send to Mac.
// Size is always 240 bytes (CVSD) or 480 bytes (mSBC) per call.
static void hfSendCb(uint8_t* buf, uint32_t size) {
    uint32_t samples = size / 2;                // int16 samples
    int16_t* out = reinterpret_cast<int16_t*>(buf);
    portENTER_CRITICAL(&ringMux);
    for (uint32_t i = 0; i < samples; i++) {
        if (ringTail != ringHead) {
            out[i] = ringBuf[ringTail];
            ringTail = (ringTail + 1) % RING_BUF_SAMPLES;
        } else {
            out[i] = 0;                          // underrun → silence
        }
    }
    portEXIT_CRITICAL(&ringMux);
}
```

### 1.7 micTask changes

The existing `micTask` uses `StickCP2.Mic.record(chunk, 256, 44100)`.
Change to use `hfpSampleRate` (either 8000 or 16000) so the mic produces
samples at the right rate before SCO connects. Since `hfpSampleRate` is set
in the event callback after SCO connects, the task should start mic recording
only after `hfpScoConnected` is true, or else record at 16000 initially and
restart after codec negotiation.

**Simplest approach:** Start `micTask` from `hfClientCallback` (on SCO connect)
and stop it on SCO disconnect. Do NOT call `StickCP2.Mic.begin()` before SCO.

Revised flow:
1. `setupSourceModeHfp()`: init BT, register callbacks, connect — but do NOT start micTask yet.
2. `hfClientCallback / AUDIO_CONNECTED`: call `StickCP2.Mic.begin()`, then
   `xTaskCreatePinnedToCore(micTask, ...)`.
3. `hfClientCallback / AUDIO_DISCONNECTED`: call `StickCP2.Mic.end()`,
   delete micTask handle (pass handle via global `TaskHandle_t micTaskHandle`).

`micTask` itself: replace hardcoded `44100` with `hfpSampleRate`.

### 1.8 `drawDisplay()` update

Add HFP-specific status line:
```
SOURCE + SCO  [green, orange dot]
SOURCE + SLC  [green, no dot]
SOURCE only   [green, grey "Searching..."]
```

Replace the `audioActive` dot check with a check on `hfpScoConnected` in SOURCE mode.

### 1.9 `setup()` changes

In the SOURCE branch:
```cpp
if (currentMode == MODE_SOURCE) {
    if (!readMacAddr(hfpPeerAddr)) {
        Serial.println("[SOURCE] ERROR: No Mac BT address configured.");
        Serial.println("Run: python a2dp_voice.py configure --port /dev/tty.usbserial-*");
        // Display error and wait for serial command or button press
        drawError("Run configure");
        // Don't call setupSourceModeHfp() — nothing to connect to
    } else {
        setupSourceModeHfp();
    }
}
```

---

## Part 2 — Python (`a2dp_voice.py`)

### 2.1 `configure` command — send MAC address

Currently sends `SET_REMOTE:<name>`. Now also send `SET_MAC:<addr>` with the Mac's
Bluetooth hardware address.

New helper to get Mac BT MAC:
```python
def _get_mac_bt_addr() -> str:
    """Return this Mac's BT MAC address as 'aa:bb:cc:dd:ee:ff'."""
    r = subprocess.run(
        ["system_profiler", "SPBluetoothDataType", "-json"],
        capture_output=True, text=True
    )
    # Parse JSON: controller_properties.controller_address
    # Fallback: blueutil --info (if available)
    ...
    return addr_string   # e.g. "00:11:22:33:44:55"
```

In `configure()`, after writing the name, also write the MAC:
```python
cmd2 = f"SET_MAC:{mac_bt_addr}\n"
ser.write(cmd2.encode())
```

The firmware's serial handler parses `SET_MAC:` and stores the 6 bytes in NVS.

### 2.2 `run_source()` — handle mono SCO channels

HFP SCO exposes the device as mono on macOS (1 input channel, unlike A2DP's 2).
Change:
```python
# Before (A2DP stereo):
channels = 2

# After (HFP mono, with query):
bt_info   = sd.query_devices(bt_in_idx)
in_channels  = bt_info["max_input_channels"]   # 1 for HFP/SCO
```

And in the stream callback, handle mono→stereo expansion for the output:
```python
def callback(indata, outdata, frames, time, status):
    out_ch = outdata.shape[1]
    if indata.shape[1] == 1 and out_ch > 1:
        outdata[:] = np.repeat(indata, out_ch, axis=1)
    else:
        outdata[:, :indata.shape[1]] = indata
```

Stream construction:
```python
sd.Stream(
    device=(bt_in_idx, None),
    samplerate=sample_rate,
    channels=(in_channels, mac_out_channels),
    dtype="int16",
    latency=latency,
    callback=callback,
)
```

### 2.3 New `configure` usage hint in `run_source()`

If `find_device("M5StickC", "input")` fails:
```
No input device matching 'M5StickC' found.
Ensure M5Stick is paired and:
  1. System Settings → Sound → Input → select M5StickC-MIC
  2. Press BtnB on M5Stick to switch to SOURCE mode
```

---

## Part 3 — sdkconfig / Build

`CONFIG_BT_HFP_CLIENT_ENABLE` must be `y`.

Check whether the m5stack:esp32 board package has it enabled by default:
```bash
find ~/.arduino15/packages/m5stack -name sdkconfig.h | xargs grep -l HFP
```

If `CONFIG_BT_HFP_CLIENT_ENABLE` is not set, options:
1. **Local sdkconfig override** via `a2dp_voice/sdkconfig.board` (if supported).
2. **Patch** the board's `sdkconfig.h` or relevant `.cmake` file.
3. **Use `esp_hf_client_api.h` guard** — if not enabled, the compile will fail with
   a clear linker error; document the fix.

This must be verified at compile time before implementing.

---

## Part 4 — Pairing workflow (updated)

```
# 1. Pair (SINK mode, same as before):
python a2dp_voice.py pair

# 2. Write Mac BT MAC to NVS (new: also writes SET_MAC):
python a2dp_voice.py configure --port /dev/cu.usbserial-5B090228551

# 3. Switch M5Stick to SOURCE mode:
#    Press BtnB on the device  OR  send "source" via serial monitor

# 4. On Mac:
#    System Settings → Sound → Input → M5StickC-MIC  (select it)
#    This opens the SCO link.

# 5. Stream to Mac speakers:
python a2dp_voice.py source
```

---

## Part 5 — Risk / unknowns to verify before coding

| Risk | Mitigation |
|------|-----------|
| `CONFIG_BT_HFP_CLIENT_ENABLE` not in m5stack board package | Check sdkconfig.h; may need manual patch |
| Mac BT MAC address parsing from `system_profiler` output | Test on target machine; have blueutil fallback |
| `esp_hf_client_init()` must be called before or after bluedroid enable | Follow ESP-IDF order: bluedroid_enable → hf_client_init |
| `hfSendCb` called from BT task (interrupt-like) — ring buffer access safety | Already using `portENTER_CRITICAL` with `portMUX_TYPE` |
| mSBC wideband negotiation on macOS | macOS supports mSBC since Monterey — test if negotiated |
| `StickCP2.Mic.record()` with 8000 Hz | M5Stack mic PDM clock scaling — verify supported rate |

---

## Implementation order (suggested)

1. Verify `CONFIG_BT_HFP_CLIENT_ENABLE` in board sdkconfig.
2. Implement `SET_MAC` serial command + NVS read/write in firmware.
3. Implement `_get_mac_bt_addr()` and updated `configure()` in Python.
4. Replace `setupSourceMode()` with `setupSourceModeHfp()` (BT stack init + callbacks).
5. Implement `hfClientCallback`, `hfRecvCb`, `hfSendCb`.
6. Update `micTask` to use `hfpSampleRate` and to start/stop via callbacks.
7. Update `drawDisplay()` for HFP state.
8. Fix `run_source()` channel count in Python.
9. Compile, flash, test.
