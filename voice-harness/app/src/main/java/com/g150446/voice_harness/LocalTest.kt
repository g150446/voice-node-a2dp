package com.g150446.voice_harness

import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.AudioTrack
import android.media.MediaRecorder
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Base64
import java.util.concurrent.TimeUnit

class LocalTest {

    @Volatile private var recordingActive = false

    private val httpClient = OkHttpClient.Builder()
        .connectTimeout(30, TimeUnit.SECONDS)
        .readTimeout(60, TimeUnit.SECONDS)
        .build()

    /** Blocking — call on IO thread. Returns raw 16kHz int16 LE PCM bytes. */
    @Suppress("MissingPermission")
    fun record(): ByteArray {
        val sampleRate = 16000
        val minBuf = AudioRecord.getMinBufferSize(
            sampleRate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT
        )
        val bufSize = maxOf(minBuf * 4, 4096)
        val ar = AudioRecord(
            MediaRecorder.AudioSource.MIC, sampleRate,
            AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, bufSize
        )
        if (ar.state != AudioRecord.STATE_INITIALIZED) {
            ar.release()
            throw IllegalStateException("AudioRecord failed to initialize — RECORD_AUDIO permission granted?")
        }
        val out = ByteArrayOutputStream()
        val buf = ByteArray(bufSize)
        try {
            ar.startRecording()
            recordingActive = true
            while (recordingActive) {
                val n = ar.read(buf, 0, buf.size)
                if (n > 0) out.write(buf, 0, n)
            }
        } finally {
            ar.stop()
            ar.release()
        }
        return out.toByteArray()
    }

    fun stopRecording() {
        recordingActive = false
    }

    /** Blocking — call on IO thread. WAV-encodes pcm → OpenRouter → plays 24kHz response on speaker. */
    fun sendAndPlay(
        pcm: ByteArray,
        apiKey: String,
        model: String,
        onLog: (String) -> Unit,
        onState: (String) -> Unit
    ) {
        try {
            val wav = buildWav(pcm)
            val wavBase64 = Base64.getEncoder().encodeToString(wav)

            val body = JSONObject().apply {
                put("model", model)
                put("modalities", JSONArray().apply { put("text"); put("audio") })
                put("audio", JSONObject().apply { put("voice", "alloy"); put("format", "pcm16") })
                put("stream", true)
                put("messages", JSONArray().apply {
                    put(JSONObject().apply {
                        put("role", "system")
                        put("content", "Reply in 1-2 short sentences. Be concise.")
                    })
                    put(JSONObject().apply {
                        put("role", "user")
                        put("content", JSONArray().apply {
                            put(JSONObject().apply {
                                put("type", "input_audio")
                                put("input_audio", JSONObject().apply {
                                    put("data", wavBase64)
                                    put("format", "wav")
                                })
                            })
                        })
                    })
                })
            }.toString()

            val request = Request.Builder()
                .url("https://openrouter.ai/api/v1/chat/completions")
                .addHeader("Authorization", "Bearer $apiKey")
                .addHeader("Content-Type", "application/json")
                .post(body.toRequestBody("application/json".toMediaType()))
                .build()

            onLog("[TEST] Sending ${pcm.size / 2 / 16000}s audio to OpenRouter...")
            val response = httpClient.newCall(request).execute()
            if (!response.isSuccessful) {
                onLog("[TEST] HTTP ${response.code}: ${response.body?.string()?.take(300)}")
                return
            }

            // Collect all 24kHz PCM from SSE stream
            val allPcm = ByteArrayOutputStream()
            response.body?.byteStream()?.bufferedReader()?.use { reader ->
                var line: String?
                while (reader.readLine().also { line = it } != null) {
                    val l = line ?: continue
                    if (!l.startsWith("data: ")) continue
                    val payload = l.removePrefix("data: ")
                    if (payload == "[DONE]") break
                    try {
                        val json = JSONObject(payload)
                        val choices = json.optJSONArray("choices") ?: continue
                        if (choices.length() == 0) continue
                        val delta = choices.getJSONObject(0).optJSONObject("delta") ?: continue
                        val audio = delta.optJSONObject("audio") ?: continue
                        val dataStr = audio.optString("data", "")
                        if (dataStr.isEmpty()) continue
                        allPcm.write(Base64.getDecoder().decode(dataStr))
                    } catch (_: Exception) {}
                }
            }

            val pcmBytes = allPcm.toByteArray()
            if (pcmBytes.isEmpty()) {
                onLog("[TEST] No audio received from OpenRouter")
                return
            }
            onLog("[TEST] Received ${pcmBytes.size} bytes (24kHz PCM), playing...")
            onState("PLAY")
            playPcm24k(pcmBytes)
            onLog("[TEST] Playback done")
        } catch (e: Exception) {
            onLog("[TEST] Error: ${e.message}")
        }
    }

    private fun playPcm24k(pcmBytes: ByteArray) {
        val sampleRate = 24000
        val audioTrack = AudioTrack.Builder()
            .setAudioAttributes(
                AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_MEDIA)
                    .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
                    .build()
            )
            .setAudioFormat(
                AudioFormat.Builder()
                    .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                    .setSampleRate(sampleRate)
                    .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                    .build()
            )
            .setTransferMode(AudioTrack.MODE_STATIC)
            .setBufferSizeInBytes(pcmBytes.size)
            .build()
        try {
            audioTrack.write(pcmBytes, 0, pcmBytes.size)
            audioTrack.play()
            // Wait for playback to finish (duration + 500ms buffer)
            val durationMs = (pcmBytes.size.toLong() * 1000L) / (sampleRate * 2)
            Thread.sleep(durationMs + 500L)
        } finally {
            audioTrack.stop()
            audioTrack.release()
        }
    }

    private fun buildWav(pcm: ByteArray): ByteArray {
        val sampleRate = 16000
        val buf = ByteBuffer.allocate(44 + pcm.size).order(ByteOrder.LITTLE_ENDIAN)
        buf.put("RIFF".toByteArray(Charsets.US_ASCII))
        buf.putInt(36 + pcm.size)
        buf.put("WAVE".toByteArray(Charsets.US_ASCII))
        buf.put("fmt ".toByteArray(Charsets.US_ASCII))
        buf.putInt(16); buf.putShort(1); buf.putShort(1)
        buf.putInt(sampleRate); buf.putInt(sampleRate * 2)
        buf.putShort(2); buf.putShort(16)
        buf.put("data".toByteArray(Charsets.US_ASCII))
        buf.putInt(pcm.size)
        buf.put(pcm)
        return buf.array()
    }
}
