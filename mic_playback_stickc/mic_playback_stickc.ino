/**
 * M5StickC Plus2 Mic Recorder with Hat SPK2 Playback
 *
 * Records audio from built-in mic (BtnA), plays back through Hat SPK2 (BtnB).
 * Mic and speaker share GPIO0 so they cannot run simultaneously.
 *
 * Hardware: M5StickC PLUS2 + Hat SPK2
 * Hat SPK2 I2S pins: DOUT=25, BCLK=26, LRC=0
 */

#include <M5StickCPlus2.h>

// Audio config
static constexpr size_t SAMPLE_RATE      = 16000;
static constexpr size_t RECORD_CHUNK     = 256;
static constexpr size_t MAX_SAMPLES      = 48000;  // ~3 seconds at 16kHz
static constexpr size_t BUFFER_BYTES     = MAX_SAMPLES * sizeof(int16_t);

// State machine
enum State { IDLE, RECORDING, PLAYING };
static State state = IDLE;

// Audio buffer
static int16_t *audioBuf       = nullptr;
static size_t recordOffset     = 0;  // next write position (in samples)
static size_t recordedSamples  = 0;  // total recorded samples

// Display dimensions (landscape)
static int dispW, dispH;

void drawIdle() {
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setTextDatum(top_center);

    if (recordedSamples > 0) {
        float secs = (float)recordedSamples / SAMPLE_RATE;

        StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
        StickCP2.Display.drawString("READY", dispW / 2, 10);

        StickCP2.Display.setFont(&fonts::Font2);
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1fs recorded", secs);
        StickCP2.Display.drawString(buf, dispW / 2, 45);

        StickCP2.Display.setTextColor(YELLOW);
        StickCP2.Display.drawString("[A] Record  [B] Play", dispW / 2, 70);
    } else {
        StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
        StickCP2.Display.drawString("MIC RECORDER", dispW / 2, 20);

        StickCP2.Display.setFont(&fonts::Font2);
        StickCP2.Display.setTextColor(YELLOW);
        StickCP2.Display.drawString("[A] Record", dispW / 2, 60);
    }
}

void drawRecording(int16_t *chunk, size_t len) {
    StickCP2.Display.fillScreen(BLACK);

    // Red dot + "REC"
    StickCP2.Display.fillCircle(20, 14, 8, RED);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setTextDatum(top_left);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    StickCP2.Display.drawString("REC", 34, 0);

    // Elapsed / max time
    float elapsed = (float)recordOffset / SAMPLE_RATE;
    float maxSecs = (float)MAX_SAMPLES / SAMPLE_RATE;
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f/%.1fs", elapsed, maxSecs);
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextDatum(top_right);
    StickCP2.Display.drawString(buf, dispW - 4, 4);

    // Waveform in bottom area
    int waveY0 = 35;
    int waveH  = dispH - waveY0;
    int midY   = waveY0 + waveH / 2;

    // Scale chunk across display width
    for (int x = 0; x < dispW - 1; x++) {
        int idx1 = (x * len) / dispW;
        int idx2 = ((x + 1) * len) / dispW;
        int16_t s1 = chunk[idx1];
        int16_t s2 = chunk[idx2];

        // Scale samples to waveform height
        int y1 = midY + (s1 * waveH / 2) / 32768;
        int y2 = midY + (s2 * waveH / 2) / 32768;

        if (y1 > y2) { int tmp = y1; y1 = y2; y2 = tmp; }
        y1 = constrain(y1, waveY0, dispH - 1);
        y2 = constrain(y2, waveY0, dispH - 1);

        StickCP2.Display.drawFastVLine(x, y1, y2 - y1 + 1, GREEN);
    }

    StickCP2.Display.display();
}

void drawPlaying(size_t playedSamples) {
    StickCP2.Display.fillScreen(BLACK);

    // Green play triangle + "PLAY"
    int cx = 20, cy = 14;
    StickCP2.Display.fillTriangle(cx - 8, cy - 8, cx - 8, cy + 8, cx + 8, cy, GREEN);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setTextDatum(top_left);
    StickCP2.Display.setFont(&fonts::FreeSansBold12pt7b);
    StickCP2.Display.drawString("PLAY", 34, 0);

    // Time info
    float elapsed = (float)playedSamples / SAMPLE_RATE;
    float total   = (float)recordedSamples / SAMPLE_RATE;
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f/%.1fs", elapsed, total);
    StickCP2.Display.setFont(&fonts::Font2);
    StickCP2.Display.setTextDatum(top_right);
    StickCP2.Display.drawString(buf, dispW - 4, 4);

    // Progress bar
    int barX = 10, barY = 50, barW = dispW - 20, barH = 20;
    StickCP2.Display.drawRect(barX, barY, barW, barH, WHITE);
    int fillW = (playedSamples * (barW - 2)) / recordedSamples;
    if (fillW > 0) {
        StickCP2.Display.fillRect(barX + 1, barY + 1, fillW, barH - 2, GREEN);
    }

    // Stop hint
    StickCP2.Display.setTextDatum(top_center);
    StickCP2.Display.setTextColor(YELLOW);
    StickCP2.Display.drawString("[B] Stop", dispW / 2, 78);

    StickCP2.Display.display();
}

void startRecording() {
    // Stop speaker if running, start mic
    StickCP2.Speaker.end();
    StickCP2.Mic.begin();

    recordOffset    = 0;
    recordedSamples = 0;
    state           = RECORDING;
}

void stopRecording() {
    StickCP2.Mic.end();
    recordedSamples = recordOffset;
    state           = IDLE;
    drawIdle();
}

void startPlayback() {
    if (recordedSamples == 0) return;

    // Stop mic if running, start speaker
    StickCP2.Mic.end();
    StickCP2.Speaker.begin();
    StickCP2.Speaker.setVolume(255);

    StickCP2.Speaker.playRaw(audioBuf, recordedSamples, SAMPLE_RATE, false, 1, 0);
    state = PLAYING;
}

void stopPlayback() {
    StickCP2.Speaker.stop();
    StickCP2.Speaker.end();
    state = IDLE;
    drawIdle();
}

void setup() {
    auto cfg = M5.config();
    cfg.external_speaker.hat_spk2 = true;
    cfg.internal_spk = false;
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);
    dispW = StickCP2.Display.width();
    dispH = StickCP2.Display.height();

    // Allocate audio buffer
    audioBuf = (int16_t *)heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_8BIT);
    if (!audioBuf) {
        StickCP2.Display.fillScreen(RED);
        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.setTextDatum(middle_center);
        StickCP2.Display.setFont(&fonts::Font2);
        StickCP2.Display.drawString("ALLOC FAILED", dispW / 2, dispH / 2);
        while (true) delay(1000);
    }
    memset(audioBuf, 0, BUFFER_BYTES);

    // Start with both mic and speaker off
    StickCP2.Speaker.end();
    StickCP2.Mic.end();

    drawIdle();
}

void loop() {
    StickCP2.update();

    switch (state) {
        case IDLE:
            if (StickCP2.BtnA.wasClicked()) {
                startRecording();
            } else if (StickCP2.BtnB.wasClicked() && recordedSamples > 0) {
                startPlayback();
            }
            break;

        case RECORDING:
            // Record a chunk
            if (StickCP2.Mic.record(&audioBuf[recordOffset], RECORD_CHUNK, SAMPLE_RATE)) {
                drawRecording(&audioBuf[recordOffset], RECORD_CHUNK);
                recordOffset += RECORD_CHUNK;

                // Auto-stop if buffer full
                if (recordOffset + RECORD_CHUNK > MAX_SAMPLES) {
                    stopRecording();
                }
            }

            if (StickCP2.BtnA.wasClicked()) {
                stopRecording();
            }
            break;

        case PLAYING: {
            // Estimate playback position from elapsed time
            static unsigned long playStartMs = 0;
            if (StickCP2.Speaker.isPlaying()) {
                if (playStartMs == 0) playStartMs = millis();
                size_t elapsedMs = millis() - playStartMs;
                size_t played    = (elapsedMs * SAMPLE_RATE) / 1000;
                if (played > recordedSamples) played = recordedSamples;
                drawPlaying(played);
            } else {
                // Playback finished
                playStartMs = 0;
                stopPlayback();
            }

            if (StickCP2.BtnB.wasClicked()) {
                playStartMs = 0;
                stopPlayback();
            }
            break;
        }
    }

    delay(1);
}
