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
#include <BluetoothA2DPSource.h>
#include "driver/i2s_std.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr gpio_num_t SPK_DOUT = GPIO_NUM_25;
static constexpr gpio_num_t SPK_BCLK = GPIO_NUM_26;
static constexpr gpio_num_t SPK_LRC  = GPIO_NUM_0;   // shared with mic PDM WS
static constexpr gpio_num_t MIC_DATA = GPIO_NUM_34;

static constexpr size_t     RING_BUF_SAMPLES = 4096; // SOURCE ring buffer
static const char*          PREFS_NS             = "a2dp_voice";
static const char*          PREFS_KEY_MODE        = "mode";
static const char*          PREFS_KEY_REMOTE      = "remote"; // Mac BT name for SOURCE

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
enum Mode { MODE_SINK, MODE_SOURCE };
static Mode currentMode;

static volatile bool btConnected  = false;
static volatile bool audioActive  = false;

// SINK: IDF5 I2S TX channel handle
static i2s_chan_handle_t i2sTxHandle = nullptr;

// SOURCE: mono ring buffer (mic → BT callback)
static int16_t          ringBuf[RING_BUF_SAMPLES];
static volatile size_t  ringHead = 0;
static volatile size_t  ringTail = 0;
static portMUX_TYPE     ringMux  = portMUX_INITIALIZER_UNLOCKED;

// Display
static int dispW, dispH;
static unsigned long lastDisplayUpdate = 0;

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
// Set via: python a2dp_voice.py configure --port /dev/tty.usbserial-*
static String readRemoteName() {
    Preferences p;
    p.begin(PREFS_NS, true);
    String name = p.getString(PREFS_KEY_REMOTE, "");
    p.end();
    return name;
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

    // --- BT status (middle) ---
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextDatum(middle_center);
    if (btConnected) {
        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.drawString("Connected", dispW / 2, dispH / 2);
    } else {
        StickCP2.Display.setTextColor(DARKGREY);
        StickCP2.Display.drawString("Searching...", dispW / 2, dispH / 2);
    }

    // --- Audio active indicator ---
    if (audioActive && btConnected) {
        StickCP2.Display.fillCircle(dispW / 2, dispH / 2 + 20, 5, ORANGE);
    }

    // --- Button hints (bottom) ---
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextColor(YELLOW);
    StickCP2.Display.setTextDatum(bottom_left);
    StickCP2.Display.drawString("[A]=SINK", 4, dispH - 2);
    StickCP2.Display.setTextDatum(bottom_right);
    StickCP2.Display.drawString("[B]=SRC", dispW - 4, dispH - 2);

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
// SOURCE mode: mic task + ring buffer
// ---------------------------------------------------------------------------

// Core 0 task: continuously read mic into ring buffer
static void micTask(void*) {
    static int16_t chunk[256];
    while (true) {
        if (StickCP2.Mic.record(chunk, 256, 44100)) {
            portENTER_CRITICAL(&ringMux);
            for (int i = 0; i < 256; i++) {
                size_t next = (ringHead + 1) % RING_BUF_SAMPLES;
                if (next != ringTail) {
                    ringBuf[ringHead] = chunk[i];
                    ringHead = next;
                }
            }
            portEXIT_CRITICAL(&ringMux);
        }
        taskYIELD();
    }
}

// A2DP Source callback — called from BT task (Core 1)
// Frame layout: { int16_t channel1; int16_t channel2; } at 44100 Hz stereo
static int32_t sourceDataCallback(Frame *frames, int32_t frameCount) {
    for (int32_t i = 0; i < frameCount; i++) {
        portENTER_CRITICAL(&ringMux);
        int16_t s = (ringTail != ringHead) ? ringBuf[ringTail] : 0;
        if (ringTail != ringHead) {
            ringTail = (ringTail + 1) % RING_BUF_SAMPLES;
        }
        portEXIT_CRITICAL(&ringMux);
        frames[i].channel1 = s;
        frames[i].channel2 = s;  // mono → stereo
    }
    audioActive = true;
    delay(1);  // prevents watchdog reset during sustained streaming
    return frameCount;
}

static void setupSourceMode() {
    Serial.println("[SOURCE] Starting mic...");
    StickCP2.Mic.begin();  // PDM on GPIO34/GPIO0, uses I2S_NUM_0
    xTaskCreatePinnedToCore(micTask, "micTask", 4096, nullptr, 5, nullptr, 0);
    Serial.println("[SOURCE] Mic running, micTask pinned to Core 0.");

    static BluetoothA2DPSource src;
    src.set_data_callback_in_frames(sourceDataCallback);

    src.set_on_connection_state_changed(
        [](esp_a2d_connection_state_t s, void*) {
            btConnected = (s == ESP_A2D_CONNECTION_STATE_CONNECTED);
            if (!btConnected) audioActive = false;
            Serial.printf("[SOURCE] BT connection: %s\n",
                s == ESP_A2D_CONNECTION_STATE_CONNECTED    ? "CONNECTED" :
                s == ESP_A2D_CONNECTION_STATE_DISCONNECTED ? "DISCONNECTED" :
                s == ESP_A2D_CONNECTION_STATE_CONNECTING   ? "CONNECTING" :
                                                             "DISCONNECTING");
        },
        nullptr);

    src.set_auto_reconnect(false);

    // Read Mac BT name from NVS (set via: python a2dp_voice.py configure --port ...)
    // If empty → pass nullptr → connect to first available A2DP Sink.
    static String remoteName = readRemoteName();
    const char* remote = remoteName.length() > 0 ? remoteName.c_str() : nullptr;

    Serial.printf("[SOURCE] Remote target: %s\n", remote ? remote : "(any A2DP Sink)");

    // Ensure device is discoverable so Mac can find and pair it.
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    Serial.println("[SOURCE] Discoverable. Starting BT as 'M5StickC-MIC'...");

    src.start(remote);  // nullptr → first available A2DP Sink
    Serial.println("[SOURCE] BT started. Scanning for remote sink...");
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
    Serial.printf("[STATUS] mode=%s bt=%s audio=%s remote=%s\n",
        currentMode == MODE_SINK ? "SINK" : "SOURCE",
        btConnected  ? "connected"  : "disconnected",
        audioActive  ? "active"     : "idle",
        remote.length() > 0 ? remote.c_str() : "(any)");
}

static void printHelp() {
    Serial.println("--- a2dp_voice commands ---");
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

    } else {
        Serial.printf("Unknown command: %s  (type 'help')\n", line.c_str());
    }
}

void setup() {
    currentMode = readMode();

    Serial.begin(115200);

    auto cfg = M5.config();
    // Both modes: M5Unified must NOT own I2S_NUM_1
    // SINK: we drive it ourselves via IDF5 API
    // SOURCE: hat is unused, GPIO0 must be free for mic PDM WS
    cfg.external_speaker.hat_spk2 = false;
    cfg.internal_spk              = false;
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);
    dispW = StickCP2.Display.width();
    dispH = StickCP2.Display.height();

    // Ensure both audio peripherals are off before BT init
    StickCP2.Speaker.end();
    StickCP2.Mic.end();

    Serial.printf("\n=== a2dp_voice booting in %s mode ===\n",
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
}

void loop() {
    StickCP2.update();
    handleSerial();

    // BtnA → SINK mode
    if (StickCP2.BtnA.wasClicked() && currentMode != MODE_SINK) {
        requestModeSwitch(MODE_SINK);
    }
    // BtnB → SOURCE mode
    if (StickCP2.BtnB.wasClicked() && currentMode != MODE_SOURCE) {
        requestModeSwitch(MODE_SOURCE);
    }

    if (millis() - lastDisplayUpdate > 100) {
        lastDisplayUpdate = millis();
        drawDisplay();
    }

    delay(10);
}
