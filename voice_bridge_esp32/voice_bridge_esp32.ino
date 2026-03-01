/**
 * A2DP Voice Bridge for M5StickC Plus2
 *
 * SINK mode  (BtnA): Mac streams audio → M5Stick plays through Hat SPK2
 * SOURCE mode (BtnB): M5Stick mic streams audio → Mac speakers
 *
 * Modes cannot run in the same boot (BT stack cannot reinit).
 * Mode is stored in NVS; BtnA/BtnB trigger ESP.restart() into new mode.
 *
 * IDF5 note: A2DP_LEGACY_I2S_SUPPORT=false → set_i2s_config/set_pin_config
 * are no-ops. SINK mode drives I2S_NUM_1 directly via driver/i2s_std.h.
 *
 * Hardware: M5StickC PLUS2 + Hat SPK2
 * Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC/WS=0 (shared with mic PDM WS)
 */

#include <M5StickCPlus2.h>
#include <Preferences.h>
#include <BluetoothA2DPSink.h>
#include <BluetoothSerial.h>
#include "driver/i2s_std.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr gpio_num_t SPK_DOUT = GPIO_NUM_25;
static constexpr gpio_num_t SPK_BCLK = GPIO_NUM_26;
static constexpr gpio_num_t SPK_LRC  = GPIO_NUM_0;   // shared with mic PDM WS
static constexpr gpio_num_t MIC_DATA = GPIO_NUM_34;

static constexpr size_t     RING_BUF_SAMPLES = 4096; // SOURCE ring buffer
static const char*          PREFS_NS             = "voice_bridge";
static const char*          PREFS_KEY_MODE        = "mode";
static const char*          PREFS_KEY_REMOTE      = "remote"; // Mac BT name for SOURCE
static const char*          PREFS_KEY_MAC         = "mac_addr"; // Mac BT MAC (6-byte blob)

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
enum Mode { MODE_SINK, MODE_SOURCE };
static Mode currentMode;

static volatile bool btConnected  = false;
static volatile bool audioActive  = false;

// SINK: IDF5 I2S TX channel handle
static i2s_chan_handle_t i2sTxHandle = nullptr;

// SOURCE: mono ring buffer (mic → SPP)
static int16_t          ringBuf[RING_BUF_SAMPLES];
static volatile size_t  ringHead = 0;
static volatile size_t  ringTail = 0;
static portMUX_TYPE     ringMux  = portMUX_INITIALIZER_UNLOCKED;

// Display
static int dispW, dispH;
static unsigned long lastDisplayUpdate = 0;

// SOURCE / SPP globals
static BluetoothSerial  SerialBT;
static volatile bool    sppClientConnected = false;
static volatile bool    isTranscribing     = false;  // toggled by BtnA
static volatile bool    isPlayingTTS       = false;  // true while playing TTS audio
static volatile bool    ttsReady           = false;  // set by micTask, consumed by loop()
static TaskHandle_t     micTaskHandle      = nullptr;
static constexpr uint32_t MIC_SAMPLE_RATE  = 16000; // 16kHz for Whisper
static constexpr uint32_t TTS_SAMPLE_RATE  = 16000; // 16kHz (downsampled from 24kHz)

// Transcription result display (received from Python via SPP)
static String lastTranscription = "";
static SemaphoreHandle_t textMutex = nullptr;

// ---------------------------------------------------------------------------
// NVS mode persistence
// ---------------------------------------------------------------------------
static Mode readMode() {
    Preferences p;
    p.begin(PREFS_NS, true);
    Mode m = (Mode)p.getUChar(PREFS_KEY_MODE, (uint8_t)MODE_SINK);
    p.end();
    return m;
}

static void writeMode(Mode m) {
    Preferences p;
    p.begin(PREFS_NS, false);
    p.putUChar(PREFS_KEY_MODE, (uint8_t)m);
    p.end();  // flush NVS before restart
}

// Returns the Mac BT device name stored in NVS, or "" if not set.
// Set via: python voice_bridge_esp32.py configure --port /dev/tty.usbserial-*
static String readRemoteName() {
    Preferences p;
    p.begin(PREFS_NS, true);
    String name = p.getString(PREFS_KEY_REMOTE, "");
    p.end();
    return name;
}

// Read 6-byte BT MAC address from NVS. Returns true if valid.
static bool readMacAddr(uint8_t out[6]) {
    Preferences p;
    p.begin(PREFS_NS, true);
    size_t n = p.getBytes(PREFS_KEY_MAC, out, 6);
    p.end();
    return (n == 6);
}

static void writeMacAddr(const uint8_t* mac6) {
    Preferences p;
    p.begin(PREFS_NS, false);
    p.putBytes(PREFS_KEY_MAC, mac6, 6);
    p.end();
}

static void requestModeSwitch(Mode next) {
    writeMode(next);
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setTextDatum(middle_center);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    StickCP2.Display.drawString("Restarting...", dispW / 2, dispH / 2);
    delay(800);
    ESP.restart();
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
static void drawDisplay() {
    StickCP2.Display.fillScreen(BLACK);

    // --- Mode banner (top) ---
    StickCP2.Display.setTextDatum(top_center);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    if (currentMode == MODE_SINK) {
        StickCP2.Display.setTextColor(BLUE);
        StickCP2.Display.drawString("SINK", dispW / 2, 4);
    } else {
        StickCP2.Display.setTextColor(GREEN);
        StickCP2.Display.drawString("SOURCE", dispW / 2, 4);
    }

    // --- Status / transcription (middle area) ---
    StickCP2.Display.setFont(&fonts::Font2);
    if (currentMode == MODE_SOURCE) {
        if (!sppClientConnected) {
            StickCP2.Display.setTextColor(DARKGREY);
            StickCP2.Display.setTextDatum(middle_center);
            StickCP2.Display.drawString("Run on Mac:", dispW / 2, dispH / 2 - 10);
            StickCP2.Display.setFont(&fonts::Font0);
            StickCP2.Display.drawString("python voice_bridge_esp32.py transcribe", dispW / 2, dispH / 2 + 10);
        } else if (isPlayingTTS) {
            StickCP2.Display.setTextColor(CYAN);
            StickCP2.Display.setTextDatum(top_center);
            StickCP2.Display.drawString("Playing...", dispW / 2, 30);
            if (textMutex && xSemaphoreTake(textMutex, pdMS_TO_TICKS(10))) {
                if (lastTranscription.length() > 0) {
                    StickCP2.Display.setTextColor(WHITE);
                    StickCP2.Display.setTextDatum(top_left);
                    StickCP2.Display.setTextWrap(true);
                    StickCP2.Display.setCursor(4, 50);
                    String disp = lastTranscription;
                    if (disp.length() > 90) disp = disp.substring(disp.length() - 90);
                    StickCP2.Display.print(disp);
                }
                xSemaphoreGive(textMutex);
            }
        } else if (isTranscribing) {
            // Recording indicator
            StickCP2.Display.setTextColor(RED);
            StickCP2.Display.setTextDatum(top_center);
            StickCP2.Display.drawString("* REC", dispW / 2, 30);
            // Show latest transcription
            if (textMutex && xSemaphoreTake(textMutex, pdMS_TO_TICKS(10))) {
                if (lastTranscription.length() > 0) {
                    StickCP2.Display.setTextColor(WHITE);
                    StickCP2.Display.setTextDatum(top_left);
                    StickCP2.Display.setTextWrap(true);
                    StickCP2.Display.setCursor(4, 50);
                    // Truncate for display (landscape: ~30 chars wide)
                    String disp = lastTranscription;
                    if (disp.length() > 90) disp = disp.substring(disp.length() - 90);
                    StickCP2.Display.print(disp);
                }
                xSemaphoreGive(textMutex);
            }
        } else {
            // Idle — show last transcription if any
            StickCP2.Display.setTextColor(WHITE);
            StickCP2.Display.setTextDatum(top_center);
            StickCP2.Display.drawString("Ready", dispW / 2, 30);
            if (textMutex && xSemaphoreTake(textMutex, pdMS_TO_TICKS(10))) {
                if (lastTranscription.length() > 0) {
                    StickCP2.Display.setTextColor(LIGHTGREY);
                    StickCP2.Display.setTextDatum(top_left);
                    StickCP2.Display.setTextWrap(true);
                    StickCP2.Display.setCursor(4, 50);
                    String disp = lastTranscription;
                    if (disp.length() > 90) disp = disp.substring(disp.length() - 90);
                    StickCP2.Display.print(disp);
                }
                xSemaphoreGive(textMutex);
            }
        }
    } else {
        StickCP2.Display.setTextDatum(middle_center);
        if (btConnected) {
            StickCP2.Display.setTextColor(WHITE);
            StickCP2.Display.drawString("Connected", dispW / 2, dispH / 2);
        } else {
            StickCP2.Display.setTextColor(DARKGREY);
            StickCP2.Display.drawString("Searching...", dispW / 2, dispH / 2);
        }
    }

    // --- Button hints (bottom) ---
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextColor(YELLOW);
    StickCP2.Display.setTextDatum(bottom_left);
    if (currentMode == MODE_SOURCE) {
        StickCP2.Display.drawString(isTranscribing ? "[A]=STOP" : "[A]=REC", 4, dispH - 2);
    }
    StickCP2.Display.setTextDatum(bottom_right);
    StickCP2.Display.drawString(currentMode == MODE_SOURCE ? "[B]=SINK" : "[B]=SRC", dispW - 4, dispH - 2);

    StickCP2.Display.display();
}

// ---------------------------------------------------------------------------
// SINK mode: IDF5 I2S init for Hat SPK2 (I2S_NUM_1)
// ---------------------------------------------------------------------------
static void initI2sSink() {
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chanCfg.dma_desc_num  = 8;
    chanCfg.dma_frame_num = 512;  // 8×512×4B = 16 KB ≈ 93 ms buffer (was 64 → 2 KB, too small)
    chanCfg.auto_clear    = true; // output silence on underrun instead of garbage
    ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, &i2sTxHandle, nullptr));

    i2s_std_config_t stdCfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_BCLK,
            .ws   = SPK_LRC,
            .dout = SPK_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2sTxHandle, &stdCfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2sTxHandle));
}

// A2DP Sink audio callback — receives decoded 44100 Hz stereo int16 PCM
static void sinkDataCallback(const uint8_t *data, uint32_t len) {
    if (!i2sTxHandle) return;
    size_t written = 0;
    // Block up to 100 ms: gives DMA time to drain without dropping frames.
    // With the 16 KB buffer above, blocking is rare in steady state.
    i2s_channel_write(i2sTxHandle, data, len, &written, pdMS_TO_TICKS(100));
    if (written < len) {
        Serial.printf("[SINK] warn: dropped %u/%u bytes\n", len - written, len);
    }
    audioActive = (written > 0);
}

static void setupSinkMode() {
    Serial.println("[SINK] Initialising I2S...");
    initI2sSink();
    Serial.println("[SINK] I2S ready on I2S_NUM_1 (BCLK=26 WS=0 DOUT=25)");

    static BluetoothA2DPSink sink;

    // i2s_output=false: library must NOT init its own I2S; we own I2S_NUM_1
    sink.set_stream_reader(sinkDataCallback, false);

    sink.set_on_connection_state_changed(
        [](esp_a2d_connection_state_t s, void*) {
            btConnected = (s == ESP_A2D_CONNECTION_STATE_CONNECTED);
            if (!btConnected) audioActive = false;
            Serial.printf("[SINK] BT connection: %s\n",
                s == ESP_A2D_CONNECTION_STATE_CONNECTED   ? "CONNECTED" :
                s == ESP_A2D_CONNECTION_STATE_DISCONNECTED? "DISCONNECTED" :
                s == ESP_A2D_CONNECTION_STATE_CONNECTING  ? "CONNECTING" :
                                                            "DISCONNECTING");
        },
        nullptr);

    sink.set_on_audio_state_changed(
        [](esp_a2d_audio_state_t s, void*) {
            audioActive = (s == ESP_A2D_AUDIO_STATE_STARTED);
            Serial.printf("[SINK] Audio: %s\n",
                s == ESP_A2D_AUDIO_STATE_STARTED ? "STARTED" :
                s == ESP_A2D_AUDIO_STATE_STOPPED ? "STOPPED" : "REMOTE_SUSPEND");
        },
        nullptr);

    sink.set_auto_reconnect(false);
    Serial.println("[SINK] Starting BT as 'M5StickC-SPK'...");
    sink.start("M5StickC-SPK");
    Serial.println("[SINK] BT started. Waiting for Mac to connect via Sound settings.");
}

// ---------------------------------------------------------------------------
// SOURCE mode: SPP (Bluetooth Serial) — mic → BT serial stream
// ---------------------------------------------------------------------------

// Frame header: [0x55][0xAA][len_hi][len_lo][pcm_data...]
static constexpr uint8_t  FRAME_MAGIC_0 = 0x55;
static constexpr uint8_t  FRAME_MAGIC_1 = 0xAA;
static constexpr size_t   MIC_CHUNK     = 512;  // samples per read (~32ms @ 16kHz)

// TTS audio frame: [0xAA][0x55][len_hi][len_lo][pcm_data...]  (reversed magic)
static constexpr uint8_t  TTS_MAGIC_0   = 0xAA;
static constexpr uint8_t  TTS_MAGIC_1   = 0x55;

// TTS audio buffer (collected from SPP frames, played after all received)
static int16_t* ttsBuf     = nullptr;
static size_t   ttsBufSize = 0;       // allocated size in bytes
static size_t   ttsBufUsed = 0;       // used bytes
static constexpr size_t TTS_MAX_BYTES = 960000; // ~30s at 16kHz mono (PSRAM)

static void initTTSBuffer() {
    if (!ttsBuf) {
        // Prefer PSRAM for large buffer (~30s); fall back to SRAM (~6s) if unavailable
        ttsBuf = (int16_t*)heap_caps_malloc(TTS_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ttsBuf) {
            ttsBufSize = TTS_MAX_BYTES;
        } else {
            constexpr size_t FALLBACK = 192000;
            ttsBuf = (int16_t*)heap_caps_malloc(FALLBACK, MALLOC_CAP_8BIT);
            ttsBufSize = ttsBuf ? FALLBACK : 0;
            Serial.printf("[TTS] PSRAM unavailable, fallback buffer: %u bytes\n", ttsBufSize);
        }
    }
    ttsBufUsed = 0;
}

static void playTTSBuffer() {
    if (!ttsBuf || ttsBufUsed == 0) {
        Serial.println("[TTS] No data to play");
        return;
    }
    size_t nSamples = ttsBufUsed / sizeof(int16_t);
    Serial.printf("[TTS] Playing %u samples (%u bytes) via Speaker API\n", nSamples, ttsBufUsed);
    StickCP2.Mic.end();
    vTaskDelay(pdMS_TO_TICKS(50));
    bool ok = StickCP2.Speaker.begin();
    Serial.printf("[TTS] Speaker.begin() = %d, isEnabled=%d\n", ok, StickCP2.Speaker.isEnabled());
    StickCP2.Speaker.setVolume(255);
    StickCP2.Speaker.playRaw(ttsBuf, nSamples, TTS_SAMPLE_RATE, false, 1, 0);
    Serial.printf("[TTS] playRaw() called, isPlaying=%d\n", StickCP2.Speaker.isPlaying());
    // Wait for playback to finish
    unsigned long t0 = millis();
    while (StickCP2.Speaker.isPlaying()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    unsigned long elapsed = millis() - t0;
    Serial.printf("[TTS] Playback done in %lu ms\n", elapsed);
    StickCP2.Speaker.stop();
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();
    ttsBufUsed = 0;
}

// Play a 1kHz test tone for 2 seconds through Hat SPK2
static void playTestTone() {
    Serial.println("[TEST] Playing 1kHz tone via Speaker API...");
    StickCP2.Mic.end();
    vTaskDelay(pdMS_TO_TICKS(50));
    StickCP2.Speaker.begin();
    StickCP2.Speaker.setVolume(255);

    // Generate 1kHz mono sine wave at 16kHz, 2 seconds
    static constexpr size_t TONE_SAMPLES = 32000; // 2s at 16kHz
    int16_t* tone = (int16_t*)heap_caps_malloc(TONE_SAMPLES * 2, MALLOC_CAP_8BIT);
    if (tone) {
        for (size_t i = 0; i < TONE_SAMPLES; i++) {
            tone[i] = (int16_t)(sinf(2.0f * M_PI * 1000.0f * i / 16000.0f) * 30000);
        }
        StickCP2.Speaker.playRaw(tone, TONE_SAMPLES, 16000, false, 1, 0);
        while (StickCP2.Speaker.isPlaying()) vTaskDelay(pdMS_TO_TICKS(50));
        StickCP2.Speaker.stop();
        heap_caps_free(tone);
    }
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();
    Serial.println("[TEST] Tone done");
}

// SPP connection events (set by callback, processed by micTask)
static volatile bool sppEventConnect = false;
static volatile bool sppEventDisconnect = false;

// SPP connection callback — runs in BT task, do NOT use Serial here
static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        sppEventConnect = true;
    } else if (event == ESP_SPP_CLOSE_EVT) {
        sppEventDisconnect = true;
    }
}

// Core 0 task: read mic when transcribing, handle SPP text + TTS audio
static void micTask(void*) {
    static int16_t chunk[MIC_CHUNK];
    uint8_t header[4];
    header[0] = FRAME_MAGIC_0;
    header[1] = FRAME_MAGIC_1;
    String incomingText = "";
    unsigned long lastClientCheck = 0;

    while (true) {
        // Process SPP events from callback
        if (sppEventConnect) {
            sppEventConnect = false;
            sppClientConnected = true;
            btConnected = true;
            Serial.println("[SPP] Client connected (callback)");
        }
        if (sppEventDisconnect) {
            sppEventDisconnect = false;
            sppClientConnected = false;
            btConnected = false;
            audioActive = false;
            isTranscribing = false;
            Serial.println("[SPP] Client disconnected (callback)");
        }

        // Fallback: poll hasClient() every 500ms
        if (millis() - lastClientCheck > 500) {
            lastClientCheck = millis();
            bool has = SerialBT.hasClient();
            if (has && !sppClientConnected) {
                sppClientConnected = true;
                btConnected = true;
                Serial.println("[SPP] Client detected (poll)");
            } else if (!has && sppClientConnected) {
                sppClientConnected = false;
                btConnected = false;
                audioActive = false;
                isTranscribing = false;
                Serial.println("[SPP] Client lost (poll)");
            }
        }

        // Read incoming data from Python (text lines or TTS audio frames)
        while (SerialBT.available()) {
            uint8_t b = SerialBT.peek();

            // Check for TTS audio frame: [0xAA][0x55][len_hi][len_lo][pcm...]
            if (b == TTS_MAGIC_0) {
                SerialBT.read(); // consume 0xAA
                // Wait up to 100ms for next byte
                unsigned long w = millis();
                while (!SerialBT.available() && millis() - w < 100) vTaskDelay(1);
                if (SerialBT.available() && SerialBT.peek() == TTS_MAGIC_1) {
                    SerialBT.read(); // consume 0x55
                    // Read length (2 bytes, big-endian)
                    while (SerialBT.available() < 2) vTaskDelay(1);
                    uint8_t hdr[2];
                    SerialBT.readBytes(hdr, 2);
                    uint16_t audioLen = (hdr[0] << 8) | hdr[1];

                    if (audioLen == 0) {
                        // End-of-TTS marker — signal main loop to play
                        Serial.printf("[TTS] Received %u bytes, signaling playback\n", ttsBufUsed);
                        SerialBT.printf("DBG:TTS_PLAY bytes=%u\n", ttsBufUsed);
                        ttsReady = true;
                        // Wait for main loop to finish playback
                        while (ttsReady) vTaskDelay(pdMS_TO_TICKS(50));
                        isPlayingTTS = false;
                        SerialBT.println("DBG:TTS_DONE");
                        continue;
                    }

                    // First TTS frame: init buffer
                    if (!isPlayingTTS) {
                        isPlayingTTS = true;
                        initTTSBuffer();
                        Serial.printf("[TTS] Buffering audio, first frame len=%u\n", audioLen);
                        SerialBT.printf("DBG:TTS_START len=%u\n", audioLen);
                    }

                    // Read frame data into buffer
                    static uint8_t frameBuf[2048];
                    uint16_t remaining = audioLen;
                    while (remaining > 0) {
                        uint16_t toRead = (remaining > sizeof(frameBuf)) ? sizeof(frameBuf) : remaining;
                        uint16_t got = 0;
                        unsigned long deadline = millis() + 2000;
                        while (got < toRead && millis() < deadline) {
                            int avail = SerialBT.available();
                            if (avail > 0) {
                                int rd = SerialBT.readBytes(frameBuf + got,
                                    min((int)(toRead - got), avail));
                                got += rd;
                            } else {
                                vTaskDelay(1);
                            }
                        }
                        if (got > 0 && ttsBuf && ttsBufUsed + got <= ttsBufSize) {
                            memcpy(((uint8_t*)ttsBuf) + ttsBufUsed, frameBuf, got);
                            ttsBufUsed += got;
                        }
                        remaining -= got;
                        if (got == 0) {
                            Serial.printf("[TTS] Timeout, remaining=%u\n", remaining);
                            break;
                        }
                    }
                } else {
                    // Not TTS frame, treat as text char 0xAA
                    if (incomingText.length() < 256) incomingText += (char)0xAA;
                }
                continue;
            }

            // Text data (transcription results or PING)
            char c = (char)SerialBT.read();
            if (c == '\n') {
                if (incomingText == "PING") {
                    SerialBT.println("PONG");
                    incomingText = "";
                    continue;
                }
                if (incomingText.length() > 0) {
                    if (textMutex && xSemaphoreTake(textMutex, pdMS_TO_TICKS(50))) {
                        lastTranscription = incomingText;
                        xSemaphoreGive(textMutex);
                    }
                    Serial.printf("[STT] %s\n", incomingText.c_str());
                    incomingText = "";
                }
            } else {
                if (incomingText.length() < 256) incomingText += c;
            }
        }

        if (!isTranscribing) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (StickCP2.Mic.record(chunk, MIC_CHUNK, MIC_SAMPLE_RATE)) {
            uint16_t byteLen = MIC_CHUNK * sizeof(int16_t);
            header[2] = (byteLen >> 8) & 0xFF;
            header[3] = byteLen & 0xFF;
            SerialBT.write(header, 4);
            SerialBT.write(reinterpret_cast<const uint8_t*>(chunk), byteLen);
        }
        taskYIELD();
    }
}

static void setupSourceMode() {
    Serial.println("[SOURCE] Starting SPP as 'M5StickC-MIC'...");

    textMutex = xSemaphoreCreateMutex();

    SerialBT.register_callback(sppCallback);
    if (!SerialBT.begin("M5StickC-MIC", false /* not master */)) {
        Serial.println("[SOURCE] ERROR: BluetoothSerial.begin() failed");
        return;
    }
    Serial.println("[SOURCE] SPP server started. Waiting for client...");

    // Allocate TTS audio buffer
    initTTSBuffer();
    Serial.printf("[SOURCE] TTS buffer: %u bytes\n", ttsBufSize);

    // Start mic — ensure speaker is off first (GPIO0 shared)
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();
    xTaskCreatePinnedToCore(micTask, "micTask", 8192, nullptr, 5,
                            &micTaskHandle, 0);
    Serial.println("[SOURCE] Mic ready. Press BtnA to start transcription.");
}

// ---------------------------------------------------------------------------
// Serial command handler
// ---------------------------------------------------------------------------
// Commands (send as a line, CR+LF or LF):
//   sink              → switch to SINK mode  (restarts)
//   source            → switch to SOURCE mode (restarts)
//   status            → print current mode, BT state, remote name
//   SET_REMOTE:<name> → store Mac BT name in NVS, restart
//   help              → print command list
static void printStatus() {
    String remote = readRemoteName();
    uint8_t mac[6];
    bool hasMac = readMacAddr(mac);
    char macStr[18] = "(not set)";
    if (hasMac) {
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    Serial.printf("[STATUS] mode=%s bt=%s audio=%s remote=%s spp=%s\n",
        currentMode == MODE_SINK ? "SINK" : "SOURCE",
        btConnected  ? "connected"  : "disconnected",
        audioActive  ? "active"     : "idle",
        remote.length() > 0 ? remote.c_str() : "(any)",
        sppClientConnected ? "connected" : "disconnected");
}

static void printHelp() {
    Serial.println("--- voice_bridge commands ---");
    Serial.println("  sink              switch to SINK mode (restarts)");
    Serial.println("  source            switch to SOURCE mode (restarts)");
    Serial.println("  status            print mode / BT / audio state");
    Serial.println("  SET_REMOTE:<name> store Mac BT name for SOURCE mode");
    Serial.println("  help              this list");
}

static void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.printf("> %s\n", line.c_str());

    if (line.equalsIgnoreCase("sink")) {
        if (currentMode == MODE_SINK) { Serial.println("Already in SINK mode."); return; }
        Serial.println("Switching to SINK mode...");
        requestModeSwitch(MODE_SINK);

    } else if (line.equalsIgnoreCase("source")) {
        if (currentMode == MODE_SOURCE) { Serial.println("Already in SOURCE mode."); return; }
        Serial.println("Switching to SOURCE mode...");
        requestModeSwitch(MODE_SOURCE);

    } else if (line.equalsIgnoreCase("status")) {
        printStatus();

    } else if (line.equalsIgnoreCase("help")) {
        printHelp();

    } else if (line.startsWith("SET_REMOTE:")) {
        String name = line.substring(11);
        Preferences p;
        p.begin(PREFS_NS, false);
        p.putString(PREFS_KEY_REMOTE, name);
        p.end();
        Serial.printf("OK remote=%s — restarting...\n", name.c_str());
        delay(200);
        ESP.restart();

    } else if (line.equalsIgnoreCase("tone")) {
        if (currentMode == MODE_SOURCE) playTestTone();
        else Serial.println("tone only available in SOURCE mode");

    } else {
        Serial.printf("Unknown command: %s  (type 'help')\n", line.c_str());
    }
}

void setup() {
    currentMode = readMode();

    Serial.begin(115200);

    auto cfg = M5.config();
    // Hat SPK2 must be enabled for M5Unified Speaker API (used for TTS playback)
    cfg.external_speaker.hat_spk2 = true;
    cfg.internal_spk              = false;
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);
    dispW = StickCP2.Display.width();
    dispH = StickCP2.Display.height();

    // Ensure both audio peripherals are off before BT init
    StickCP2.Speaker.end();
    StickCP2.Mic.end();

    Serial.printf("\n=== voice_bridge booting in %s mode ===\n",
        currentMode == MODE_SINK ? "SINK" : "SOURCE");
    Serial.println("Type 'help' for commands.");

    drawDisplay();  // show mode BEFORE BT init (takes 2-3 s)

    if (currentMode == MODE_SINK) {
        setupSinkMode();
    } else {
        setupSourceMode();
    }

    Serial.println("=== Boot complete ===");
    printStatus();

    // Startup beep to verify speaker works
    if (currentMode == MODE_SOURCE) {
        StickCP2.Mic.end();
        vTaskDelay(pdMS_TO_TICKS(50));
        StickCP2.Speaker.begin();
        StickCP2.Speaker.setVolume(255);
        StickCP2.Speaker.tone(1000, 200);  // 1kHz, 200ms
        vTaskDelay(pdMS_TO_TICKS(300));
        StickCP2.Speaker.stop();
        StickCP2.Speaker.end();
        StickCP2.Mic.begin();
        Serial.println("[BOOT] Startup beep played");
    }
}

void loop() {
    StickCP2.update();
    handleSerial();

    // Play TTS audio from main loop context (matches working mic_playback pattern)
    if (ttsReady && ttsBuf && ttsBufUsed > 0) {
        playTTSBuffer();
        ttsReady = false;
    }

    if (StickCP2.BtnA.wasPressed()) {
        Serial.println("[BTN] BtnA pressed");
        if (currentMode == MODE_SOURCE) {
            isTranscribing = !isTranscribing;
            audioActive = isTranscribing;
            Serial.printf("[SOURCE] Transcription %s\n",
                isTranscribing ? "STARTED" : "STOPPED");
            // Always try to send — write silently fails if no client
            SerialBT.println(isTranscribing ? "CMD:START" : "CMD:STOP");
        }
    }
    if (StickCP2.BtnB.wasPressed()) {
        Serial.println("[BTN] BtnB pressed");
        if (currentMode == MODE_SOURCE) {
            requestModeSwitch(MODE_SINK);
        } else {
            requestModeSwitch(MODE_SOURCE);
        }
    }

    if (millis() - lastDisplayUpdate > 100) {
        lastDisplayUpdate = millis();
        drawDisplay();
    }

    delay(10);
}
