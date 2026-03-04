/**
 * Voice Bridge WiFi — M5StickC Plus2
 *
 * Records audio from the built-in PDM microphone, sends it to
 * OpenRouter GPT Audio Mini via WiFi (HTTPS), receives the audio
 * response and plays it through Hat SPK2.
 *
 * BtnA: Start / stop recording (tap to start, tap again to send)
 *
 * Hardware: M5StickC PLUS2 + Hat SPK2
 * Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC/WS=0 (shared with mic PDM WS)
 *
 * Serial commands:
 *   SET_KEY:<key>  Store OpenRouter API key in NVS
 *   status         Print WiFi / API state
 *   help           Command list
 */

#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include "mbedtls/base64.h"
#include <driver/i2s_std.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static const char* WIFI_SSID = "razr";
static const char* WIFI_PASS = "88888888";

static const char* API_HOST = "openrouter.ai";
static const char* API_PATH = "/api/v1/chat/completions";
static const char* DEFAULT_API_KEY =
    "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";

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
static const char* NVS_NS      = "voice_bridge";
static const char* NVS_KEY_API = "api_key";
static String apiKey;

static String readApiKey() {
    Preferences p;
    p.begin(NVS_NS, true);
    String k = p.getString(NVS_KEY_API, "");
    p.end();
    return k;
}
static void writeApiKey(const String& k) {
    Preferences p;
    p.begin(NVS_NS, false);
    p.putString(NVS_KEY_API, k);
    p.end();
}

// ---------------------------------------------------------------------------
// Audio parameters
// ---------------------------------------------------------------------------
static constexpr uint32_t SAMPLE_RATE     = 16000;
static constexpr uint32_t TTS_PLAY_RATE   = 16000;
static constexpr size_t   MIN_AUDIO_BUF   = 96000; // 96KB = 3s at 16kHz
static constexpr size_t   MIC_CHUNK       = 512;

// Shared buffer: recording data, then reused for TTS playback
static int16_t* audioBuf      = nullptr;
static size_t   audioBufBytes = 0;     // actual allocated size
static size_t   maxRecSamples = 0;     // dynamic, based on audioBufBytes
static size_t   recSamples    = 0;     // samples recorded
static size_t   ttsBytes      = 0;     // bytes of 16kHz TTS audio in audioBuf

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State { ST_IDLE, ST_RECORDING, ST_PROCESSING, ST_PLAYING };
static State         state = ST_IDLE;
static int           dispW, dispH;
static unsigned long lastDispMs = 0;
static String        statusMsg;
static String        lastResponse;

// ---------------------------------------------------------------------------
// WAV header (44 bytes, PCM16 mono)
// ---------------------------------------------------------------------------
static void buildWavHeader(uint8_t hdr[44], uint32_t dataBytes, uint32_t sr) {
    uint32_t fileSize   = 36 + dataBytes;
    uint16_t fmt        = 1;
    uint16_t ch         = 1;
    uint32_t byteRate   = sr * 2;
    uint16_t blockAlign = 2;
    uint16_t bps        = 16;
    memcpy(hdr,    "RIFF", 4); memcpy(hdr+4,  &fileSize,   4);
    memcpy(hdr+8,  "WAVE", 4);
    memcpy(hdr+12, "fmt ", 4); uint32_t f16=16; memcpy(hdr+16, &f16, 4);
    memcpy(hdr+20, &fmt,  2);  memcpy(hdr+22, &ch,         2);
    memcpy(hdr+24, &sr,   4);  memcpy(hdr+28, &byteRate,   4);
    memcpy(hdr+32, &blockAlign,2); memcpy(hdr+34, &bps,    2);
    memcpy(hdr+36, "data", 4); memcpy(hdr+40, &dataBytes,  4);
}

// ---------------------------------------------------------------------------
// Base64 helpers
// ---------------------------------------------------------------------------
static inline bool isB64Char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=';
}

// ---------------------------------------------------------------------------
// Direct I2S speaker output with ring buffer for smooth streaming.
// Producer (main task): decodes network data → writes PCM to ring buffer
// Consumer (I2S task): reads ring buffer → writes to I2S DMA
// ---------------------------------------------------------------------------
#define SPK_BCLK     26
#define SPK_LRC       0
#define SPK_DOUT     25

static i2s_chan_handle_t i2s_tx_handle = NULL;

// Ring buffer for producer-consumer decoupling
static uint8_t*  ringBuf  = nullptr;
static size_t    ringSize = 0;
static volatile size_t ringWr = 0;
static volatile size_t ringRd = 0;
static volatile bool   ringEOS = false;    // producer signals end-of-stream
static volatile size_t ringTotalWritten = 0;
static TaskHandle_t    i2sTaskHandle = NULL;

static inline size_t ringAvail() {
    size_t w = ringWr, r = ringRd;
    return (w >= r) ? (w - r) : (ringSize - r + w);
}
static inline size_t ringFree() {
    return ringSize - 1 - ringAvail();
}

// Producer: write PCM bytes to ring buffer. Blocks if full.
static void ringWrite(const int16_t* data, size_t samples) {
    const uint8_t* src = (const uint8_t*)data;
    size_t bytes = samples * sizeof(int16_t);
    while (bytes > 0) {
        size_t fr = ringFree();
        if (fr == 0) { vTaskDelay(1); continue; }
        size_t n = (bytes < fr) ? bytes : fr;
        size_t w = ringWr;
        for (size_t i = 0; i < n; i++) {
            ringBuf[w] = src[i];
            if (++w >= ringSize) w = 0;
        }
        ringWr = w;
        src += n;
        bytes -= n;
    }
}

// Consumer FreeRTOS task: reads ring buffer → writes to I2S DMA
static void i2sWriterTask(void* /*param*/) {
    uint8_t buf[1024];
    while (true) {
        size_t avail = ringAvail();
        if (avail == 0) {
            if (ringEOS) break;
            vTaskDelay(1);
            continue;
        }
        size_t n = (avail < sizeof(buf)) ? avail : sizeof(buf);
        // Ensure we write even number of bytes (16-bit samples)
        n &= ~1;
        if (n == 0) { vTaskDelay(1); continue; }
        size_t r = ringRd;
        for (size_t i = 0; i < n; i++) {
            buf[i] = ringBuf[r];
            if (++r >= ringSize) r = 0;
        }
        ringRd = r;
        size_t written;
        i2s_channel_write(i2s_tx_handle, buf, n, &written, portMAX_DELAY);
        ringTotalWritten += written;
    }
    // Flush DMA with silence
    uint8_t silence[512] = {};
    for (int i = 0; i < 16; i++) {
        size_t w;
        i2s_channel_write(i2s_tx_handle, silence, sizeof(silence), &w, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelete(NULL);
}

static bool i2sSpkBegin(uint32_t sampleRate) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 256;   // Match M5Unified defaults (8×256 = 2048 frames)
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] new_channel failed: %d\n", err);
        return false;
    }

    // Match M5Unified's exact slot config for Hat SPK2 (NS4168)
    i2s_std_config_t std_cfg;
    memset(&std_cfg, 0, sizeof(std_cfg));
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.sample_rate_hz = sampleRate;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_128;
    std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    std_cfg.slot_cfg.ws_width = 16;
    std_cfg.slot_cfg.ws_pol = false;
    std_cfg.slot_cfg.bit_shift = true;
#if SOC_I2S_HW_VERSION_1
    std_cfg.slot_cfg.msb_right = false;
#else
    std_cfg.slot_cfg.left_align = true;
    std_cfg.slot_cfg.big_endian = false;
    std_cfg.slot_cfg.bit_order_lsb = false;
#endif
    std_cfg.gpio_cfg.bclk = (gpio_num_t)SPK_BCLK;
    std_cfg.gpio_cfg.ws   = (gpio_num_t)SPK_LRC;
    std_cfg.gpio_cfg.dout = (gpio_num_t)SPK_DOUT;
    std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;

    err = i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("[I2S] init_std failed: %d\n", err);
        i2s_del_channel(i2s_tx_handle);
        i2s_tx_handle = NULL;
        return false;
    }

    err = i2s_channel_enable(i2s_tx_handle);
    if (err != ESP_OK) {
        Serial.printf("[I2S] enable failed: %d\n", err);
        i2s_del_channel(i2s_tx_handle);
        i2s_tx_handle = NULL;
        return false;
    }
    return true;
}

static void i2sSpkStop() {
    if (!i2s_tx_handle) return;
    i2s_channel_disable(i2s_tx_handle);
    i2s_del_channel(i2s_tx_handle);
    i2s_tx_handle = NULL;
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
static void drawDisplay() {
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextDatum(top_center);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);

    switch (state) {
    case ST_IDLE:
        StickCP2.Display.setTextColor(GREEN);
        StickCP2.Display.drawString("READY", dispW / 2, 4);
        break;
    case ST_RECORDING:
        StickCP2.Display.setTextColor(RED);
        StickCP2.Display.drawString("* REC", dispW / 2, 4);
        break;
    case ST_PROCESSING:
        StickCP2.Display.setTextColor(YELLOW);
        StickCP2.Display.drawString("THINKING", dispW / 2, 4);
        break;
    case ST_PLAYING:
        StickCP2.Display.setTextColor(CYAN);
        StickCP2.Display.drawString("PLAYING", dispW / 2, 4);
        break;
    }

    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextColor(WHITE);

    if (state == ST_IDLE && apiKey.length() == 0) {
        StickCP2.Display.setTextDatum(middle_center);
        StickCP2.Display.setTextColor(RED);
        StickCP2.Display.drawString("No API Key", dispW / 2, dispH / 2 - 10);
        StickCP2.Display.setFont(&fonts::Font0);
        StickCP2.Display.setTextColor(DARKGREY);
        StickCP2.Display.drawString("Serial: SET_KEY:<key>",
                                    dispW / 2, dispH / 2 + 10);
    } else if (state == ST_IDLE) {
        if (lastResponse.length() > 0) {
            StickCP2.Display.setTextDatum(top_left);
            StickCP2.Display.setTextWrap(true);
            StickCP2.Display.setCursor(4, 34);
            String d = lastResponse;
            if (d.length() > 100) d = d.substring(d.length() - 100);
            StickCP2.Display.print(d);
        } else {
            StickCP2.Display.setTextDatum(middle_center);
            StickCP2.Display.drawString("[A] = Talk", dispW / 2, dispH / 2);
        }
    } else if (state == ST_RECORDING) {
        StickCP2.Display.setTextDatum(middle_center);
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1fs / %.0fs",
                 (float)recSamples / SAMPLE_RATE,
                 (float)maxRecSamples / SAMPLE_RATE);
        StickCP2.Display.drawString(buf, dispW / 2, dispH / 2);
    } else if (statusMsg.length() > 0) {
        StickCP2.Display.setTextDatum(middle_center);
        StickCP2.Display.drawString(statusMsg.c_str(), dispW / 2, dispH / 2);
    }

    // WiFi indicator
    StickCP2.Display.setFont(&fonts::Font0);
    StickCP2.Display.setTextDatum(bottom_right);
    StickCP2.Display.setTextColor(WiFi.isConnected() ? GREEN : RED);
    StickCP2.Display.drawString(
        WiFi.isConnected() ? "WiFi OK" : "No WiFi", dispW - 4, dispH - 2);

    // Button hint
    StickCP2.Display.setTextDatum(bottom_left);
    StickCP2.Display.setTextColor(YELLOW);
    if (state == ST_IDLE)
        StickCP2.Display.drawString("[A] Talk", 4, dispH - 2);
    else if (state == ST_RECORDING)
        StickCP2.Display.drawString("[A] Send", 4, dispH - 2);

    StickCP2.Display.display();
}

// ---------------------------------------------------------------------------
// HTTP chunked / content-length body reader
// ---------------------------------------------------------------------------
struct BodyReader {
    WiFiClientSecure* cl;
    int contentLen;     // -1 if chunked
    bool chunked;
    int chunkRem;       // remaining in current chunk
    bool done;
    unsigned long deadline;

    // Read buffer to reduce WiFi read overhead
    static const int RDBUF_SZ = 2048;
    uint8_t rdBuf[RDBUF_SZ];
    int rdPos;
    int rdLen;

    BodyReader(WiFiClientSecure* c, int clen, bool ch, unsigned long dl)
        : cl(c), contentLen(clen), chunked(ch),
          chunkRem(0), done(false), deadline(dl),
          rdPos(0), rdLen(0) {}

    // Buffered low-level read from WiFi client
    int waitByte() {
        if (rdPos < rdLen) return rdBuf[rdPos++];
        // Refill buffer
        while (!cl->available()) {
            if (!cl->connected() || millis() > deadline) {
                done = true; return -1;
            }
            delay(1);
        }
        rdLen = cl->read(rdBuf, min(RDBUF_SZ, (int)cl->available()));
        rdPos = 0;
        if (rdLen <= 0) { done = true; return -1; }
        return rdBuf[rdPos++];
    }

    int readByte() {
        if (done) return -1;

        if (!chunked) {
            if (contentLen == 0) { done = true; return -1; }
            int b = waitByte();
            if (b < 0) { done = true; return -1; }
            if (contentLen > 0) contentLen--;
            return b;
        }

        // Chunked mode: read next chunk header when needed
        if (chunkRem <= 0) {
            String sizeLine;
            while (true) {
                int c = waitByte();
                if (c < 0) return -1;
                if (c == '\n') break;
                if (c != '\r') sizeLine += (char)c;
            }
            if (sizeLine.length() == 0) {
                // Empty line between chunks, try again
                return readByte();
            }
            chunkRem = (int)strtoul(sizeLine.c_str(), nullptr, 16);
            if (chunkRem == 0) { done = true; return -1; }
        }

        int b = waitByte();
        if (b < 0) return -1;
        chunkRem--;

        // After chunk data, consume trailing \r\n
        if (chunkRem == 0) {
            waitByte(); // \r
            waitByte(); // \n
        }
        return b;
    }

    // Parse SSE streaming response, decode base64 audio, downsample 24→16kHz,
    // and write PCM to the ring buffer. A separate I2S task drains the buffer.
    bool parseAudio() {
        const char* dataPattern       = "\"data\":\"";       // 8 chars
        const char* transcriptPattern = "\"transcript\":\""; // 14 chars

        int dpIdx = 0;
        int tpIdx = 0;

        enum { SCAN, READ_B64, READ_TRANSCRIPT } pstate = SCAN;

        char b64Quad[4];
        int  b64Idx = 0;
        uint8_t carry24[6];
        int     carryLen = 0;

        // PCM accumulation before ring write
        static int16_t pcmBuf[512];
        size_t pcmPos = 0;
        size_t totalBytes = 0;

        String transcript;
        bool   escNext = false;

        while (!done) {
            int b = readByte();
            if (b < 0) break;
            char c = (char)b;

            switch (pstate) {
            case SCAN:
                if (c == dataPattern[dpIdx]) {
                    dpIdx++;
                    if (dpIdx == 8) { pstate = READ_B64; dpIdx = 0; tpIdx = 0; }
                } else {
                    dpIdx = (c == dataPattern[0]) ? 1 : 0;
                }
                if (pstate == SCAN) {
                    if (c == transcriptPattern[tpIdx]) {
                        tpIdx++;
                        if (tpIdx == 14) { pstate = READ_TRANSCRIPT; dpIdx = 0; tpIdx = 0; }
                    } else {
                        tpIdx = (c == transcriptPattern[0]) ? 1 : 0;
                    }
                }
                break;

            case READ_B64:
                if (c == '\\') {
                    int nx = readByte();
                    if (nx < 0) break;
                    c = (char)nx;
                }
                if (c == '"') {
                    b64Idx = 0;
                    pstate = SCAN;
                    break;
                }
                if (!isB64Char(c)) break;

                b64Quad[b64Idx++] = c;
                if (b64Idx == 4) {
                    uint8_t decoded[3];
                    size_t outLen = 0;
                    mbedtls_base64_decode(decoded, 3, &outLen,
                        (unsigned char*)b64Quad, 4);
                    if (outLen > 0 && carryLen + (int)outLen <= 6) {
                        memcpy(carry24 + carryLen, decoded, outLen);
                        carryLen += outLen;
                    }
                    // Downsample 24kHz→16kHz: 3 samples in → 2 samples out
                    while (carryLen >= 6) {
                        int16_t* src = (int16_t*)carry24;
                        pcmBuf[pcmPos++] = src[0];
                        pcmBuf[pcmPos++] = (int16_t)(((int32_t)src[1] + src[2]) / 2);
                        memmove(carry24, carry24 + 6, carryLen - 6);
                        carryLen -= 6;

                        if (pcmPos >= 512) {
                            ringWrite(pcmBuf, pcmPos);
                            totalBytes += pcmPos * sizeof(int16_t);
                            pcmPos = 0;
                        }
                    }
                    b64Idx = 0;
                }
                break;

            case READ_TRANSCRIPT:
                if (escNext) {
                    escNext = false;
                    if (c == 'n') transcript += '\n';
                    else transcript += c;
                } else if (c == '\\') {
                    escNext = true;
                } else if (c == '"') {
                    pstate = SCAN;
                } else {
                    if (transcript.length() < 300) transcript += c;
                }
                break;
            }
        }

        // Flush remaining carry
        if (carryLen >= 2 && pcmPos + 1 <= 512) {
            pcmBuf[pcmPos++] = *(int16_t*)carry24;
        }
        // Flush remaining PCM to ring buffer
        if (pcmPos > 0) {
            ringWrite(pcmBuf, pcmPos);
            totalBytes += pcmPos * sizeof(int16_t);
        }

        ttsBytes = totalBytes;
        lastResponse = transcript;
        Serial.printf("[API] TTS: %u bytes (%u samples @ %uHz), Transcript: %s\n",
                      totalBytes, totalBytes / 2, TTS_PLAY_RATE,
                      transcript.c_str());
        return totalBytes > 0;
    }
};

// ---------------------------------------------------------------------------
// OpenRouter API call — streams request body, parses response on the fly
// ---------------------------------------------------------------------------
static bool callOpenRouter() {
    if (recSamples == 0 || !audioBuf) return false;

    size_t pcmBytes = recSamples * sizeof(int16_t);
    size_t wavSize  = 44 + pcmBytes;
    // Exact base64 output length
    size_t b64Len   = ((wavSize + 2) / 3) * 4;

    // JSON fragments around the base64 data
    const char* jsonPre =
        "{\"model\":\"openai/gpt-audio-mini\","
        "\"modalities\":[\"text\",\"audio\"],"
        "\"audio\":{\"voice\":\"alloy\",\"format\":\"pcm16\"},"
        "\"stream\":true,"
        "\"messages\":["
        "{\"role\":\"system\",\"content\":"
        "\"Reply in 1-2 short sentences. Be concise. "
        "Reply in the same language as the user.\"},"
        "{\"role\":\"user\",\"content\":["
        "{\"type\":\"input_audio\",\"input_audio\":{\"data\":\"";
    const char* jsonPost = "\",\"format\":\"wav\"}}]}]}";

    size_t preLen  = strlen(jsonPre);
    size_t postLen = strlen(jsonPost);
    size_t contentLength = preLen + b64Len + postLen;

    Serial.printf("[API] pcm=%u wav=%u b64=%u body=%u\n",
                  pcmBytes, wavSize, b64Len, contentLength);

    // ---- Connect ----
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(90);
    Serial.println("[API] Connecting...");
    if (!client.connect(API_HOST, 443)) {
        Serial.println("[API] HTTPS connect failed");
        return false;
    }

    // ---- Send HTTP headers ----
    client.printf(
        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: Bearer %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        API_PATH, API_HOST, apiKey.c_str(), contentLength);

    // ---- Stream request body ----
    // 1) JSON prefix
    client.print(jsonPre);

    // 2) Base64-encoded WAV, streamed in chunks
    //    WAV = 44-byte header + PCM data
    //    Encode in 3072-byte groups (→ 4096 base64 chars, no padding)
    //    First chunk: 45 bytes (44 hdr + 1 pcm) to align to multiple of 3
    uint8_t wavHdr[44];
    buildWavHeader(wavHdr, pcmBytes, SAMPLE_RATE);

    static char b64Buf[4100];  // static to avoid stack overflow with SSL
    size_t outLen = 0;

    // First 45 bytes: header(44) + audio[0]
    uint8_t first[45];
    memcpy(first, wavHdr, 44);
    first[44] = ((uint8_t*)audioBuf)[0];
    mbedtls_base64_encode((unsigned char*)b64Buf, sizeof(b64Buf),
                          &outLen, first, 45);
    client.write((uint8_t*)b64Buf, outLen);

    // Remaining audio bytes starting at offset 1
    const uint8_t* pcm = (const uint8_t*)audioBuf;
    size_t offset = 1;
    size_t remaining = pcmBytes - 1;
    const size_t ENC_CHUNK = 3072; // multiple of 3

    while (remaining > 0) {
        size_t chunk = (remaining >= ENC_CHUNK) ? ENC_CHUNK : remaining;
        outLen = 0;
        mbedtls_base64_encode((unsigned char*)b64Buf, sizeof(b64Buf),
                              &outLen, pcm + offset, chunk);
        client.write((uint8_t*)b64Buf, outLen);
        offset += chunk;
        remaining -= chunk;
    }

    // 3) JSON suffix
    client.print(jsonPost);
    Serial.println("[API] Request sent, waiting for response...");

    // ---- Read response headers ----
    unsigned long deadline = millis() + 120000; // 2 min total
    while (!client.available() && client.connected() && millis() < deadline)
        delay(10);

    if (!client.available()) {
        Serial.println("[API] Response timeout");
        client.stop();
        return false;
    }

    // Status line
    String statusLine = client.readStringUntil('\n');
    statusLine.trim();
    int httpCode = 0;
    if (statusLine.length() >= 12)
        httpCode = statusLine.substring(9, 12).toInt();
    Serial.printf("[API] %s (code=%d)\n", statusLine.c_str(), httpCode);

    // Headers
    int respContentLen = -1;
    bool isChunked = false;
    while (client.connected()) {
        String hdr = client.readStringUntil('\n');
        hdr.trim();
        if (hdr.length() == 0) break;
        String hdrLow = hdr;
        hdrLow.toLowerCase();
        if (hdrLow.startsWith("content-length:"))
            respContentLen = hdr.substring(16).toInt();
        if (hdrLow.indexOf("chunked") >= 0)
            isChunked = true;
    }

    if (httpCode != 200) {
        // Read and display error
        String err;
        while (client.available() && err.length() < 500)
            err += (char)client.read();
        Serial.printf("[API] Error: %s\n", err.c_str());
        statusMsg = String("HTTP ") + httpCode;
        client.stop();
        return false;
    }

    // ---- Parse response body with streaming playback ----
    recSamples = 0;
    ttsBytes = 0;

    // Free audioBuf to make room for ring buffer + I2S DMA
    free(audioBuf);
    audioBuf = nullptr;

    // Allocate ring buffer from freed memory
    ringSize = ESP.getMaxAllocHeap();
    if (ringSize > 80000) ringSize = 80000;  // cap at 80KB
    if (ringSize < 16000) ringSize = 16000;  // minimum 16KB
    ringBuf = (uint8_t*)malloc(ringSize);
    if (!ringBuf) {
        Serial.println("[API] Ring buffer alloc failed");
        audioBuf = (int16_t*)malloc(audioBufBytes);
        client.stop();
        return false;
    }
    ringWr = ringRd = 0;
    ringEOS = false;
    ringTotalWritten = 0;
    Serial.printf("[API] Ring buffer: %u KB\n", ringSize / 1024);

    // Start I2S speaker (mic already ended by caller)
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!i2sSpkBegin(TTS_PLAY_RATE)) {
        Serial.println("[API] I2S speaker init failed");
        free(ringBuf); ringBuf = nullptr;
        audioBuf = (int16_t*)malloc(audioBufBytes);
        client.stop();
        return false;
    }

    // Start I2S writer task on core 0 (network runs on core 1 via loopTask)
    i2sTaskHandle = NULL;
    xTaskCreatePinnedToCore(i2sWriterTask, "i2s_wr", 4096, NULL, 5, &i2sTaskHandle, 0);
    if (!i2sTaskHandle) {
        Serial.println("[API] I2S task create failed");
        i2sSpkStop();
        free(ringBuf); ringBuf = nullptr;
        audioBuf = (int16_t*)malloc(audioBufBytes);
        client.stop();
        return false;
    }

    state = ST_PLAYING;
    drawDisplay();

    BodyReader reader(&client, respContentLen, isChunked, deadline);
    bool ok = reader.parseAudio();

    // Signal end of stream and wait for I2S task to finish
    ringEOS = true;
    Serial.printf("[API] Stream done, waiting for I2S task (%u bytes in ring)...\n",
                  ringAvail());
    // Wait for task to drain ring buffer and flush DMA
    while (eTaskGetState(i2sTaskHandle) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Stop I2S
    i2sSpkStop();

    Serial.printf("[API] I2S total written: %u bytes\n", ringTotalWritten);

    client.stop();

    // Clean up ring buffer
    free(ringBuf);
    ringBuf = nullptr;
    ringSize = 0;

    // Reallocate audio buffer for next recording
    audioBuf = (int16_t*)malloc(audioBufBytes);
    if (!audioBuf) {
        Serial.println("[WARN] audioBuf realloc failed, retrying smaller");
        audioBufBytes = MIN_AUDIO_BUF;
        audioBuf = (int16_t*)malloc(audioBufBytes);
    }

    return ok;
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
static void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("SET_KEY:")) {
        String k = line.substring(8);
        k.trim();
        writeApiKey(k);
        apiKey = k;
        Serial.printf("OK api_key stored (%u chars)\n", k.length());
    } else if (line.equalsIgnoreCase("status")) {
        Serial.printf("[STATUS] state=%d wifi=%s key=%s heap=%u\n",
            state,
            WiFi.isConnected()
                ? WiFi.localIP().toString().c_str() : "disconnected",
            apiKey.length() > 0 ? "set" : "NOT SET",
            ESP.getFreeHeap());
    } else if (line.equalsIgnoreCase("help")) {
        Serial.println("SET_KEY:<key>  Store OpenRouter API key");
        Serial.println("status         Print state");
        Serial.println("help           This list");
    } else {
        Serial.printf("Unknown: %s (type 'help')\n", line.c_str());
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    auto cfg = M5.config();
    cfg.external_speaker.hat_spk2 = true;
    cfg.internal_spk              = false;
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);
    dispW = StickCP2.Display.width();
    dispH = StickCP2.Display.height();

    StickCP2.Speaker.end();
    StickCP2.Mic.end();

    Serial.println("\n=== Voice Bridge WiFi ===");
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    // API key: NVS first, then compiled-in default
    apiKey = readApiKey();
    if (apiKey.length() == 0 && strlen(DEFAULT_API_KEY) > 0)
        apiKey = DEFAULT_API_KEY;
    Serial.printf("API key: %s\n",
        apiKey.length() > 0 ? "loaded" : "NOT SET (use SET_KEY:<key>)");

    // Allocate shared audio buffer — as large as possible (SRAM only)
    // Try progressively smaller sizes starting from largest contiguous block
    {
        size_t maxBlock = ESP.getMaxAllocHeap();
        audioBufBytes = (maxBlock > 4096) ? (maxBlock - 4096) : MIN_AUDIO_BUF;
        if (audioBufBytes > 200000) audioBufBytes = 200000;
        // Try to allocate, reducing size on failure
        while (audioBufBytes >= MIN_AUDIO_BUF) {
            audioBuf = (int16_t*)malloc(audioBufBytes);
            if (audioBuf) break;
            audioBufBytes -= 4000;
        }
        if (!audioBuf) {
            audioBufBytes = MIN_AUDIO_BUF;
            audioBuf = (int16_t*)malloc(audioBufBytes);
        }
    }
    Serial.printf("Audio buffer: %p (%u KB, %.1fs TTS)\n",
        audioBuf, audioBufBytes / 1024, (float)audioBufBytes / 2.0f / TTS_PLAY_RATE);
    if (!audioBuf) Serial.println("FATAL: audio buffer alloc failed!");
    maxRecSamples = audioBufBytes / sizeof(int16_t);
    Serial.printf("Max recording: %.1fs (%u samples)\n",
        (float)maxRecSamples / SAMPLE_RATE, maxRecSamples);

    // Connect WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("WiFi '%s'...", WIFI_SSID);
    drawDisplay();

    unsigned long t0 = millis();
    while (!WiFi.isConnected() && millis() - t0 < 15000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.isConnected())
        Serial.printf("\nConnected! IP=%s\n",
            WiFi.localIP().toString().c_str());
    else
        Serial.println("\nWiFi FAILED — will retry");

    Serial.printf("Free heap after WiFi: %u bytes\n", ESP.getFreeHeap());

    // Start mic (speaker off — GPIO0 shared)
    StickCP2.Mic.begin();

    // Startup beep
    StickCP2.Mic.end();
    vTaskDelay(pdMS_TO_TICKS(50));
    StickCP2.Speaker.begin();
    StickCP2.Speaker.setVolume(255);
    StickCP2.Speaker.tone(1000, 200);
    vTaskDelay(pdMS_TO_TICKS(300));
    StickCP2.Speaker.stop();
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();

    drawDisplay();
    Serial.println("=== Ready ===");
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    StickCP2.update();
    handleSerial();

    // WiFi auto-reconnect
    static unsigned long lastWifiCheck = 0;
    if (!WiFi.isConnected() && millis() - lastWifiCheck > 10000) {
        lastWifiCheck = millis();
        WiFi.disconnect();
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    // BtnA: start / stop recording
    if (StickCP2.BtnA.wasPressed()) {
        if (state == ST_IDLE) {
            if (apiKey.length() == 0) {
                statusMsg = "No API Key!";
            } else if (!WiFi.isConnected()) {
                statusMsg = "No WiFi!";
            } else {
                recSamples = 0;
                state = ST_RECORDING;
                Serial.println("[REC] Start");
            }
        } else if (state == ST_RECORDING) {
            Serial.printf("[REC] Stop (%u samples, %.1fs)\n",
                recSamples, (float)recSamples / SAMPLE_RATE);
            StickCP2.Mic.end();

            state = ST_PROCESSING;
            statusMsg = "Sending...";
            drawDisplay();

            if (callOpenRouter()) {
                // Audio already played via streaming inside callOpenRouter
            } else {
                if (statusMsg.length() == 0) statusMsg = "No response";
            }
            state = ST_IDLE;
            StickCP2.Mic.begin();
        }
    }

    // Continuously read mic while recording
    if (state == ST_RECORDING && audioBuf) {
        static int16_t chunk[MIC_CHUNK];
        if (StickCP2.Mic.record(chunk, MIC_CHUNK, SAMPLE_RATE)) {
            size_t room = maxRecSamples - recSamples;
            size_t n = min((size_t)MIC_CHUNK, room);
            if (n > 0) {
                memcpy(audioBuf + recSamples, chunk, n * sizeof(int16_t));
                recSamples += n;
            }
            if (recSamples >= maxRecSamples) {
                Serial.println("[REC] Buffer full, auto-send");
                StickCP2.Mic.end();
                state = ST_PROCESSING;
                statusMsg = "Sending...";
                drawDisplay();
                if (callOpenRouter()) {
                    // Audio already played
                }
                state = ST_IDLE;
                StickCP2.Mic.begin();
            }
        }
    }

    // Display refresh
    if (millis() - lastDispMs > 150) {
        lastDispMs = millis();
        drawDisplay();
    }

    delay(5);
}
