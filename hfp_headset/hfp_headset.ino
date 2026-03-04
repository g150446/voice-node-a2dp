/**
 * HFP Headset for M5StickC Plus2
 *
 * Turns the M5StickC Plus2 into a Bluetooth HFP headset for Android.
 * Audio out: Hat SPK2 (I2S STD TX on I2S_NUM_1)
 * Audio in:  built-in PDM microphone (I2S PDM RX on I2S_NUM_0)
 *
 * GPIO0 constraint: mic CLK and speaker WS share GPIO0 — cannot run
 * simultaneously. BtnA toggles SPK ↔ MIC during a call.
 *
 * Hardware: M5StickC PLUS2 + Hat SPK2
 * Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC/WS=0 (shared with mic PDM CLK)
 */

#include <M5StickCPlus2.h>
#include <Preferences.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr gpio_num_t SPK_DOUT = GPIO_NUM_25;
static constexpr gpio_num_t SPK_BCLK = GPIO_NUM_26;
static constexpr gpio_num_t SPK_LRC  = GPIO_NUM_0;   // shared with mic PDM CLK
static constexpr gpio_num_t MIC_CLK  = GPIO_NUM_0;   // shared with speaker LRC
static constexpr gpio_num_t MIC_DATA = GPIO_NUM_34;

static constexpr size_t RING_SIZE = 2048; // must be power of 2

static const char* PREFS_NS      = "hfp_headset";
static const char* PREFS_KEY_MAC = "peer_mac";

static constexpr const char* DEVICE_NAME = "M5-Headset";

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------
enum AppState  { DISCONNECTED, CONNECTING, CONNECTED, RINGING, IN_CALL, CALL_OUTGOING };
enum AudioMode { MODE_SPK, MODE_MIC };

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static volatile AppState  appState     = DISCONNECTED;
static volatile AudioMode audioMode    = MODE_SPK;
static volatile bool      scoConnected = false;
static volatile bool      modeChangeReq = false;
static volatile uint32_t  scoSampleRate = 8000; // CVSD=8000, mSBC=16000

static esp_bd_addr_t peerAddr;
static bool          hasPeerAddr = false;
static char          clipNumber[33] = {0};

// Reconnect
static unsigned long reconnectAt    = 0;
static int           reconnectCount = 0;
static constexpr int MAX_RECONNECT  = 10;

// Volume (0-15)
static int spkVolume = 10;

// Display
static int dispW, dispH;
static unsigned long lastDisplayUpdate = 0;
static volatile bool displayDirty = true;

// Ring buffers — SCO ↔ I2S bridge
static int16_t        spkRing[RING_SIZE];
static volatile size_t spkHead = 0, spkTail = 0;
static portMUX_TYPE    spkMux = portMUX_INITIALIZER_UNLOCKED;

static int16_t        micRing[RING_SIZE];
static volatile size_t micHead = 0, micTail = 0;
static portMUX_TYPE    micMux = portMUX_INITIALIZER_UNLOCKED;

// I2S handles
static i2s_chan_handle_t i2sTxHandle = nullptr; // speaker
static i2s_chan_handle_t i2sRxHandle = nullptr; // mic

// Audio task
static TaskHandle_t audioTaskHandle = nullptr;

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
static bool readPeerAddr(uint8_t out[6]) {
    Preferences p;
    p.begin(PREFS_NS, true);
    size_t n = p.getBytes(PREFS_KEY_MAC, out, 6);
    p.end();
    return (n == 6);
}

static void writePeerAddr(const uint8_t* mac6) {
    Preferences p;
    p.begin(PREFS_NS, false);
    p.putBytes(PREFS_KEY_MAC, mac6, 6);
    p.end();
}

// ---------------------------------------------------------------------------
// Ring buffer helpers
// ---------------------------------------------------------------------------
static inline size_t ringAvail(volatile size_t h, volatile size_t t) {
    return (h - t) & (RING_SIZE - 1);
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
static const char* stateStr() {
    switch (appState) {
        case DISCONNECTED:  return "Disconnected";
        case CONNECTING:    return "Connecting...";
        case CONNECTED:     return "Connected";
        case RINGING:       return "Ringing";
        case IN_CALL:       return "In Call";
        case CALL_OUTGOING: return "Calling...";
        default:            return "?";
    }
}

static uint32_t stateColor() {
    switch (appState) {
        case DISCONNECTED:  return RED;
        case CONNECTING:    return YELLOW;
        case CONNECTED:     return GREEN;
        case RINGING:       return CYAN;
        case IN_CALL:       return GREEN;
        case CALL_OUTGOING: return YELLOW;
        default:            return WHITE;
    }
}

static void drawDisplay() {
    StickCP2.Display.fillScreen(BLACK);

    // --- Top banner ---
    StickCP2.Display.setTextDatum(top_center);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.drawString(DEVICE_NAME, dispW / 2, 2);

    // Connection dot
    uint32_t dotColor = (appState >= CONNECTED) ? GREEN : RED;
    StickCP2.Display.fillCircle(dispW - 10, 12, 5, dotColor);

    // --- State text (center) ---
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    StickCP2.Display.setTextColor(stateColor());
    StickCP2.Display.setTextDatum(middle_center);
    int centerY = dispH / 2 - 5;
    StickCP2.Display.drawString(stateStr(), dispW / 2, centerY);

    // Caller ID (while ringing)
    if (appState == RINGING && clipNumber[0] != 0) {
        StickCP2.Display.setFont(&fonts::Font2);
        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.setTextDatum(top_center);
        StickCP2.Display.drawString(clipNumber, dispW / 2, centerY + 20);
    }

    // Audio mode indicator (during call)
    if (appState == IN_CALL) {
        StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
        uint32_t modeColor = (audioMode == MODE_SPK) ? CYAN : ORANGE;
        StickCP2.Display.setTextColor(modeColor);
        StickCP2.Display.setTextDatum(top_center);
        StickCP2.Display.drawString(audioMode == MODE_SPK ? "SPK" : "MIC",
                                    dispW / 2, centerY + 22);
    }

    // --- Button hints (bottom) ---
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextColor(YELLOW);
    StickCP2.Display.setTextDatum(bottom_left);
    switch (appState) {
        case RINGING:
            StickCP2.Display.drawString("[A]=Answer", 4, dispH - 2);
            break;
        case IN_CALL:
            StickCP2.Display.drawString("[A]=SPK/MIC", 4, dispH - 2);
            break;
        default:
            break;
    }
    StickCP2.Display.setTextDatum(bottom_right);
    switch (appState) {
        case DISCONNECTED:
            if (hasPeerAddr)
                StickCP2.Display.drawString("[B]=Connect", dispW - 4, dispH - 2);
            break;
        case RINGING:
            StickCP2.Display.drawString("[B]=Reject", dispW - 4, dispH - 2);
            break;
        case IN_CALL:
        case CALL_OUTGOING:
            StickCP2.Display.drawString("[B]=Hangup", dispW - 4, dispH - 2);
            break;
        default:
            break;
    }

    StickCP2.Display.display();
    displayDirty = false;
}

// ---------------------------------------------------------------------------
// I2S init / deinit
// ---------------------------------------------------------------------------
static void initSpkI2S(uint32_t sampleRate) {
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chanCfg.dma_desc_num  = 8;
    chanCfg.dma_frame_num = 240;
    chanCfg.auto_clear    = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, &i2sTxHandle, nullptr));

    i2s_std_config_t stdCfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_BCLK,
            .ws   = SPK_LRC,
            .dout = SPK_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2sTxHandle, &stdCfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2sTxHandle));
    Serial.printf("[I2S] SPK TX enabled @ %u Hz\n", sampleRate);
}

static void deinitSpkI2S() {
    if (i2sTxHandle) {
        i2s_channel_disable(i2sTxHandle);
        i2s_del_channel(i2sTxHandle);
        i2sTxHandle = nullptr;
    }
}

static void initMicI2S(uint32_t sampleRate) {
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chanCfg.dma_desc_num  = 8;
    chanCfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, nullptr, &i2sRxHandle));

    i2s_pdm_rx_config_t pdmCfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sampleRate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk  = MIC_CLK,
            .din  = MIC_DATA,
            .invert_flags = { .clk_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(i2sRxHandle, &pdmCfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2sRxHandle));
    Serial.printf("[I2S] MIC PDM RX enabled @ %u Hz\n", sampleRate);
}

static void deinitMicI2S() {
    if (i2sRxHandle) {
        i2s_channel_disable(i2sRxHandle);
        i2s_del_channel(i2sRxHandle);
        i2sRxHandle = nullptr;
    }
}

// ---------------------------------------------------------------------------
// SCO data callbacks (run in BT task context — must be fast)
// ---------------------------------------------------------------------------

// Incoming audio from AG (remote voice) → speaker ring buffer
static void recvCb(const uint8_t *buf, uint32_t len) {
    const int16_t *samples = (const int16_t *)buf;
    size_t n = len / sizeof(int16_t);
    portENTER_CRITICAL(&spkMux);
    for (size_t i = 0; i < n; i++) {
        size_t next = (spkHead + 1) & (RING_SIZE - 1);
        if (next != spkTail) {
            spkRing[spkHead] = samples[i];
            spkHead = next;
        }
    }
    portEXIT_CRITICAL(&spkMux);
}

// Outgoing audio to AG (our voice) ← mic ring buffer
static uint32_t sendCb(uint8_t *buf, uint32_t len) {
    int16_t *samples = (int16_t *)buf;
    size_t n = len / sizeof(int16_t);
    portENTER_CRITICAL(&micMux);
    for (size_t i = 0; i < n; i++) {
        if (micHead != micTail) {
            samples[i] = micRing[micTail];
            micTail = (micTail + 1) & (RING_SIZE - 1);
        } else {
            samples[i] = 0; // silence
        }
    }
    portEXIT_CRITICAL(&micMux);
    return len;
}

// ---------------------------------------------------------------------------
// HFP event callback
// ---------------------------------------------------------------------------
static void hfpCallback(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
            auto s = param->conn_stat.state;
            Serial.printf("[HFP] SLC: %s\n",
                s == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED  ? "DISCONNECTED" :
                s == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING    ? "CONNECTING" :
                s == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED     ? "CONNECTED" :
                s == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED ? "SLC_CONNECTED" :
                s == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTING ? "DISCONNECTING" :
                                                                     "UNKNOWN");
            if (s == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
                appState = CONNECTED;
                reconnectCount = 0;
                memcpy(peerAddr, param->conn_stat.remote_bda, 6);
                hasPeerAddr = true;
                writePeerAddr(peerAddr);
            } else if (s == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING) {
                appState = CONNECTING;
            } else if (s == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
                scoConnected = false;
                if (appState != DISCONNECTED) {
                    appState = DISCONNECTED;
                    audioMode = MODE_SPK;
                    if (hasPeerAddr && reconnectCount < MAX_RECONNECT) {
                        unsigned long backoff = min(30000UL, 2000UL << reconnectCount);
                        reconnectAt = millis() + backoff;
                        reconnectCount++;
                        Serial.printf("[HFP] Reconnect in %lu ms (#%d)\n",
                                      backoff, reconnectCount);
                    }
                }
            }
            displayDirty = true;
            break;
        }

        case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
            auto s = param->audio_stat.state;
            Serial.printf("[HFP] Audio: %s\n",
                s == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED   ? "DISCONNECTED" :
                s == ESP_HF_CLIENT_AUDIO_STATE_CONNECTING     ? "CONNECTING" :
                s == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED      ? "CONNECTED (CVSD)" :
                s == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC ? "CONNECTED (mSBC)" :
                                                                 "UNKNOWN");
            if (s == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
                scoConnected = true;
                scoSampleRate = 8000;
            } else if (s == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                scoConnected = true;
                scoSampleRate = 16000;
            } else {
                scoConnected = false;
            }
            displayDirty = true;
            break;
        }

        case ESP_HF_CLIENT_RING_IND_EVT:
            Serial.println("[HFP] RING");
            if (appState == CONNECTED || appState == RINGING) {
                appState = RINGING;
                displayDirty = true;
            }
            break;

        case ESP_HF_CLIENT_CLIP_EVT:
            if (param->clip.number) {
                strncpy(clipNumber, param->clip.number, sizeof(clipNumber) - 1);
                Serial.printf("[HFP] Caller: %s\n", clipNumber);
                displayDirty = true;
            }
            break;

        case ESP_HF_CLIENT_CIND_CALL_EVT:
            Serial.printf("[HFP] Call: %s\n",
                param->call.status == ESP_HF_CALL_STATUS_NO_CALLS
                    ? "NO_CALLS" : "IN_PROGRESS");
            if (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS) {
                appState = IN_CALL;
                clipNumber[0] = 0;
            } else {
                if (appState == IN_CALL || appState == RINGING ||
                    appState == CALL_OUTGOING) {
                    appState = CONNECTED;
                    audioMode = MODE_SPK;
                }
            }
            displayDirty = true;
            break;

        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
            Serial.printf("[HFP] Call setup: %d\n", param->call_setup.status);
            if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING) {
                appState = RINGING;
            } else if (param->call_setup.status ==
                           ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING ||
                       param->call_setup.status ==
                           ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING) {
                appState = CALL_OUTGOING;
            }
            displayDirty = true;
            break;

        case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
            if (param->volume_control.type == ESP_HF_VOLUME_CONTROL_TARGET_SPK) {
                spkVolume = param->volume_control.volume;
                Serial.printf("[HFP] SPK vol: %d\n", spkVolume);
            } else {
                Serial.printf("[HFP] MIC vol: %d\n",
                              param->volume_control.volume);
            }
            break;

        case ESP_HF_CLIENT_AT_RESPONSE_EVT:
            Serial.printf("[HFP] AT resp: code=%d cme=%d\n",
                          param->at_response.code, param->at_response.cme);
            break;

        default:
            Serial.printf("[HFP] evt=%d\n", event);
            break;
    }
}

// ---------------------------------------------------------------------------
// GAP event callback
// ---------------------------------------------------------------------------
static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.printf("[GAP] Auth OK: %s\n", param->auth_cmpl.device_name);
                memcpy(peerAddr, param->auth_cmpl.bda, 6);
                hasPeerAddr = true;
                writePeerAddr(peerAddr);
            } else {
                Serial.printf("[GAP] Auth FAIL: %d\n", param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            Serial.printf("[GAP] SSP confirm %06lu → accept\n",
                          (unsigned long)param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        case ESP_BT_GAP_PIN_REQ_EVT: {
            Serial.println("[GAP] PIN req → 0000");
            esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
            break;
        }

        case ESP_BT_GAP_MODE_CHG_EVT:
            break; // power mode — ignore

        default:
            Serial.printf("[GAP] evt=%d\n", event);
            break;
    }
}

// ---------------------------------------------------------------------------
// Audio task (Core 0) — bridges ring buffers ↔ I2S
// ---------------------------------------------------------------------------
static void audioTask(void*) {
    static int16_t ioBuf[240];
    AudioMode curMode = MODE_SPK;
    bool i2sUp = false;

    while (true) {
        // Tear down I2S when SCO disconnects
        if (!scoConnected) {
            if (i2sUp) {
                if (curMode == MODE_SPK) deinitSpkI2S();
                else                     deinitMicI2S();
                i2sUp = false;
                Serial.println("[AUDIO] I2S down (SCO lost)");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Init or switch I2S mode
        if (modeChangeReq || !i2sUp) {
            AudioMode target = audioMode;
            if (i2sUp) {
                if (curMode == MODE_SPK) deinitSpkI2S();
                else                     deinitMicI2S();
                i2sUp = false;
                vTaskDelay(pdMS_TO_TICKS(50)); // GPIO0 settle
            }

            uint32_t rate = scoSampleRate;
            if (target == MODE_SPK) initSpkI2S(rate);
            else                    initMicI2S(rate);

            curMode = target;
            i2sUp = true;
            modeChangeReq = false;

            // Flush ring buffers
            portENTER_CRITICAL(&spkMux);
            spkHead = spkTail = 0;
            portEXIT_CRITICAL(&spkMux);
            portENTER_CRITICAL(&micMux);
            micHead = micTail = 0;
            portEXIT_CRITICAL(&micMux);
        }

        if (curMode == MODE_SPK) {
            // spkRing → I2S TX
            size_t avail;
            portENTER_CRITICAL(&spkMux);
            avail = ringAvail(spkHead, spkTail);
            if (avail > 240) avail = 240;
            for (size_t i = 0; i < avail; i++) {
                ioBuf[i] = spkRing[spkTail];
                spkTail = (spkTail + 1) & (RING_SIZE - 1);
            }
            portEXIT_CRITICAL(&spkMux);

            if (avail == 0) {
                // Output silence to keep I2S clock running
                memset(ioBuf, 0, 240 * sizeof(int16_t));
                avail = 240;
            }
            size_t written = 0;
            i2s_channel_write(i2sTxHandle, ioBuf, avail * sizeof(int16_t),
                              &written, pdMS_TO_TICKS(100));
        } else {
            // I2S PDM RX → micRing
            size_t readBytes = 0;
            esp_err_t err = i2s_channel_read(i2sRxHandle, ioBuf,
                                             240 * sizeof(int16_t),
                                             &readBytes, pdMS_TO_TICKS(100));
            if (err == ESP_OK && readBytes > 0) {
                size_t n = readBytes / sizeof(int16_t);
                portENTER_CRITICAL(&micMux);
                for (size_t i = 0; i < n; i++) {
                    size_t next = (micHead + 1) & (RING_SIZE - 1);
                    if (next != micTail) {
                        micRing[micHead] = ioBuf[i];
                        micHead = next;
                    }
                }
                portEXIT_CRITICAL(&micMux);
            }
        }
        taskYIELD();
    }
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
static void printHelp() {
    Serial.println("--- hfp_headset commands ---");
    Serial.println("  status              state info");
    Serial.println("  connect             connect to saved peer");
    Serial.println("  disconnect          disconnect HFP");
    Serial.println("  answer              answer call");
    Serial.println("  hangup              reject/hang up");
    Serial.println("  volume:<0-15>       set SPK volume");
    Serial.println("  SET_MAC:<xx:xx:..>  store peer MAC");
    Serial.println("  help                this list");
}

static void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    Serial.printf("> %s\n", line.c_str());

    if (line.equalsIgnoreCase("status")) {
        Serial.printf("[STATUS] state=%s sco=%s mode=%s rate=%u vol=%d\n",
            stateStr(), scoConnected ? "yes" : "no",
            audioMode == MODE_SPK ? "SPK" : "MIC",
            (unsigned)scoSampleRate, spkVolume);
        if (hasPeerAddr) {
            Serial.printf("[STATUS] peer=%02x:%02x:%02x:%02x:%02x:%02x\n",
                peerAddr[0], peerAddr[1], peerAddr[2],
                peerAddr[3], peerAddr[4], peerAddr[5]);
        }
    } else if (line.equalsIgnoreCase("connect")) {
        if (hasPeerAddr) {
            Serial.println("[CMD] Connecting...");
            esp_hf_client_connect(peerAddr);
        } else {
            Serial.println("[CMD] No peer. Use SET_MAC or pair from phone.");
        }
    } else if (line.equalsIgnoreCase("disconnect")) {
        if (hasPeerAddr) esp_hf_client_disconnect(peerAddr);
    } else if (line.equalsIgnoreCase("answer")) {
        esp_hf_client_answer_call();
    } else if (line.equalsIgnoreCase("hangup")) {
        esp_hf_client_reject_call();
    } else if (line.startsWith("volume:")) {
        int v = line.substring(7).toInt();
        if (v >= 0 && v <= 15) {
            spkVolume = v;
            esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_SPK, v);
            Serial.printf("[CMD] Volume → %d\n", v);
        }
    } else if (line.startsWith("SET_MAC:")) {
        uint8_t mac[6];
        if (sscanf(line.c_str() + 8,
                   "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                   &mac[0], &mac[1], &mac[2],
                   &mac[3], &mac[4], &mac[5]) == 6) {
            writePeerAddr(mac);
            memcpy(peerAddr, mac, 6);
            hasPeerAddr = true;
            Serial.printf("[CMD] MAC saved: %02x:%02x:%02x:%02x:%02x:%02x\n",
                          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        } else {
            Serial.println("[CMD] Bad MAC format. Use xx:xx:xx:xx:xx:xx");
        }
    } else if (line.equalsIgnoreCase("help")) {
        printHelp();
    } else {
        Serial.printf("Unknown: %s (type 'help')\n", line.c_str());
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    hasPeerAddr = readPeerAddr(peerAddr);

    // M5 hardware init — don't let M5Unified init speaker/mic (we own I2S)
    auto cfg = M5.config();
    cfg.external_speaker.hat_spk2 = false;
    cfg.internal_spk              = false;
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);
    dispW = StickCP2.Display.width();
    dispH = StickCP2.Display.height();

    // Release GPIO0 from M5Unified audio drivers
    StickCP2.Speaker.end();
    StickCP2.Mic.end();

    Serial.printf("\n=== HFP Headset booting ===\n");
    if (hasPeerAddr) {
        Serial.printf("[BOOT] Peer: %02x:%02x:%02x:%02x:%02x:%02x\n",
            peerAddr[0], peerAddr[1], peerAddr[2],
            peerAddr[3], peerAddr[4], peerAddr[5]);
    }
    drawDisplay();

    // --- BT controller with SCO over HCI ---
    // Arduino framework may have pre-initialized BT controller with default
    // config (bt_max_sync_conn=0). Tear it down so we can reinit with SCO.
    Serial.println("[BT] Stopping pre-initialized controller...");
    Serial.flush();
    btStop();
    Serial.println("[BT] btStop() done");
    Serial.flush();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.bt_max_sync_conn = 1;
    bt_cfg.bt_sco_datapath  = BTDM_CONTROLLER_SCO_DATA_PATH_HCI;

    esp_err_t err;
    err = esp_bt_controller_init(&bt_cfg);
    Serial.printf("[BT] controller_init: %s\n", esp_err_to_name(err));
    Serial.flush();
    if (err != ESP_OK) { Serial.println("[BT] FATAL: controller init failed"); return; }

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    Serial.printf("[BT] controller_enable: %s\n", esp_err_to_name(err));
    Serial.flush();
    if (err != ESP_OK) { Serial.println("[BT] FATAL: controller enable failed"); return; }

    err = esp_bluedroid_init();
    Serial.printf("[BT] bluedroid_init: %s\n", esp_err_to_name(err));
    Serial.flush();
    if (err != ESP_OK) { Serial.println("[BT] FATAL: bluedroid init failed"); return; }

    err = esp_bluedroid_enable();
    Serial.printf("[BT] bluedroid_enable: %s\n", esp_err_to_name(err));
    Serial.flush();
    if (err != ESP_OK) { Serial.println("[BT] FATAL: bluedroid enable failed"); return; }

    err = esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_HCI);
    Serial.printf("[BT] sco_datapath_set(HCI): %s\n", esp_err_to_name(err));
    Serial.flush();

    // GAP — SSP auto-accept (NoInputNoOutput)
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_bt_gap_register_callback(gapCallback);
    esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin);

    // Class of Device — Audio/Video + Wearable Headset + Audio service
    esp_bt_cod_t cod = {};
    cod.major   = 0x04;  // Audio/Video
    cod.minor   = 0x01;  // Wearable Headset Device
    cod.service = 0x100; // Audio
    err = esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
    Serial.printf("[BT] set_cod(headset): %s\n", esp_err_to_name(err));

    // HFP client
    esp_hf_client_register_callback(hfpCallback);
    err = esp_hf_client_init();
    Serial.printf("[BT] hf_client_init: %s\n", esp_err_to_name(err));
    esp_hf_client_register_data_callback(recvCb, sendCb);

    // Discoverable
    esp_bt_gap_set_device_name(DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    Serial.printf("[BOOT] BT ready — discoverable as '%s'\n", DEVICE_NAME);

    // Auto-connect to saved peer
    if (hasPeerAddr) {
        Serial.println("[BOOT] Auto-connecting...");
        esp_hf_client_connect(peerAddr);
        appState = CONNECTING;
    }

    // Audio task on Core 0
    xTaskCreatePinnedToCore(audioTask, "audioTask", 8192, nullptr, 5,
                            &audioTaskHandle, 0);

    Serial.println("=== Boot complete ===");
    printHelp();
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    StickCP2.update();
    handleSerial();

    // --- BtnA ---
    if (StickCP2.BtnA.wasPressed()) {
        switch (appState) {
            case RINGING:
                Serial.println("[BTN] Answer");
                esp_hf_client_answer_call();
                break;
            case IN_CALL:
                audioMode = (audioMode == MODE_SPK) ? MODE_MIC : MODE_SPK;
                modeChangeReq = true;
                Serial.printf("[BTN] Mode → %s\n",
                              audioMode == MODE_SPK ? "SPK" : "MIC");
                displayDirty = true;
                break;
            default:
                break;
        }
    }

    // --- BtnB ---
    if (StickCP2.BtnB.wasPressed()) {
        switch (appState) {
            case DISCONNECTED:
                if (hasPeerAddr) {
                    Serial.println("[BTN] Reconnect");
                    reconnectCount = 0;
                    reconnectAt = 0;
                    esp_hf_client_connect(peerAddr);
                    appState = CONNECTING;
                    displayDirty = true;
                }
                break;
            case RINGING:
                Serial.println("[BTN] Reject");
                esp_hf_client_reject_call();
                break;
            case IN_CALL:
            case CALL_OUTGOING:
                Serial.println("[BTN] Hangup");
                esp_hf_client_reject_call();
                break;
            default:
                break;
        }
    }

    // --- Auto-reconnect ---
    if (appState == DISCONNECTED && hasPeerAddr &&
        reconnectAt > 0 && millis() >= reconnectAt) {
        reconnectAt = 0;
        Serial.printf("[RECONN] Attempt %d\n", reconnectCount);
        esp_hf_client_connect(peerAddr);
        appState = CONNECTING;
        displayDirty = true;
    }

    // --- Display ---
    if (displayDirty || millis() - lastDisplayUpdate > 500) {
        lastDisplayUpdate = millis();
        drawDisplay();
    }

    delay(10);
}
