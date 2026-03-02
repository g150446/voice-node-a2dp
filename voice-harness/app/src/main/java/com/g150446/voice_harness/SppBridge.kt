package com.g150446.voice_harness

import android.bluetooth.BluetoothManager
import android.content.Context
import kotlinx.coroutines.delay
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Base64
import java.util.UUID
import java.util.concurrent.TimeUnit
import kotlin.math.sqrt

class SppBridge {

    companion object {
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        private val TTS_MAGIC = byteArrayOf(0xAA.toByte(), 0x55.toByte())
        const val SAMPLE_RATE = 16000
        const val TTS_CHUNK = 512
        const val SILENCE_THRESH = 0.02f
        const val SILENCE_BYTES = 48000   // 1.5s × 16000 × 2
        const val MAX_CHUNK_BYTES = 640000 // 20s × 16000 × 2
    }

    private var socket: android.bluetooth.BluetoothSocket? = null
    private val outLock = Any()   // serialises all writes to the output stream
    private val httpClient = OkHttpClient.Builder()
        .connectTimeout(30, TimeUnit.SECONDS)
        .readTimeout(60, TimeUnit.SECONDS)
        .writeTimeout(30, TimeUnit.SECONDS)
        .build()

    fun connect(context: Context) {
        val bm = context.getSystemService(BluetoothManager::class.java)
        val adapter = bm?.adapter ?: throw IllegalStateException("No Bluetooth adapter available")
        val device = adapter.bondedDevices?.firstOrNull { it.name?.contains("M5") == true }
            ?: throw IllegalStateException("No bonded M5 device found. Pair the M5StickC first.")
        adapter.cancelDiscovery()
        val sock = device.createInsecureRfcommSocketToServiceRecord(SPP_UUID)
        sock.connect()
        socket = sock
        verifyPingPong()
    }

    private fun verifyPingPong() {
        val sock = socket ?: throw IllegalStateException("Not connected")
        sock.outputStream.write("PING\n".toByteArray())
        sock.outputStream.flush()
        val buf = StringBuilder()
        val deadline = System.currentTimeMillis() + 3000L
        while (System.currentTimeMillis() < deadline) {
            val avail = sock.inputStream.available()
            if (avail > 0) {
                val b = ByteArray(avail)
                sock.inputStream.read(b)
                buf.append(String(b))
                if (buf.contains("PONG")) return
            } else {
                Thread.sleep(50)
            }
        }
        throw IllegalStateException("PING/PONG handshake timed out")
    }

    suspend fun runLoop(
        apiKey: String,
        model: String,
        onLog: (String) -> Unit,
        onStateChange: ((String) -> Unit)? = null
    ) = withContext(Dispatchers.IO) {
        val sock = socket ?: throw IllegalStateException("Not connected")
        val inputStream = sock.inputStream
        val outputStream = sock.outputStream
        var recording = false
        val pcmBuffer = ByteArrayOutputStream()
        var consecutiveSilent = 0

        // Keep-alive: send a null byte every 10 s so the BT stack doesn't idle-disconnect
        val keepAliveJob = launch {
            while (isActive) {
                delay(2_000)
                try {
                    synchronized(outLock) { outputStream.write(0); outputStream.flush() }
                } catch (_: Exception) {}
            }
        }

        try {
            while (isActive) {
                val (type, data) = readFrame(inputStream)
                when (type) {
                    "cmd" -> {
                        val cmd = data as String
                        when (cmd) {
                            "CMD:START" -> {
                                recording = true
                                pcmBuffer.reset()
                                consecutiveSilent = 0
                                onLog("[REC] Started — listening...")
                                onStateChange?.invoke("RECORDING")
                            }
                            "CMD:STOP" -> {
                                recording = false
                                val buf = pcmBuffer.toByteArray()
                                if (buf.size > SAMPLE_RATE * 2) {
                                    val totalRms = rms(buf)
                                    if (totalRms > SILENCE_THRESH * 0.5f) {
                                        onLog("[PROC] Processing ${buf.size} bytes...")
                                        onStateChange?.invoke("PROCESSING")
                                        processAndSend(buf, apiKey, model, outputStream, onLog)
                                        onStateChange?.invoke("CONNECTED")
                                    }
                                }
                                pcmBuffer.reset()
                                onLog("[REC] Stopped")
                                onStateChange?.invoke("CONNECTED")
                            }
                        }
                    }
                    "frame" -> {
                        if (recording) {
                            val frameData = data as ByteArray
                            pcmBuffer.write(frameData)
                            val frameRms = rms(frameData)
                            if (frameRms < SILENCE_THRESH) {
                                consecutiveSilent += frameData.size
                            } else {
                                consecutiveSilent = 0
                            }
                            val buf = pcmBuffer.toByteArray()
                            val shouldSend = buf.size >= MAX_CHUNK_BYTES ||
                                    (buf.size > SAMPLE_RATE * 2 && consecutiveSilent >= SILENCE_BYTES)
                            if (shouldSend) {
                                val totalRms = rms(buf)
                                if (totalRms > SILENCE_THRESH * 0.5f) {
                                    onLog("[PROC] Auto-processing ${buf.size} bytes...")
                                    onStateChange?.invoke("PROCESSING")
                                    processAndSend(buf, apiKey, model, outputStream, onLog)
                                    onStateChange?.invoke("RECORDING")
                                }
                                pcmBuffer.reset()
                                consecutiveSilent = 0
                            }
                        }
                    }
                    // "empty" — no data, loop
                }
            }
        } catch (e: IOException) {
            if (isActive) {   // suppress noise when disconnect() closed the socket intentionally
                onLog("[ERR] Connection lost: ${e.message}")
                onStateChange?.invoke("IDLE")
            }
        } finally {
            keepAliveJob.cancel()
        }
    }

    fun readFrame(inputStream: InputStream): Pair<String, Any?> {
        val b = inputStream.read()           // IOException propagates to runLoop's catch
        if (b < 0) throw IOException("stream closed")
        return when (b) {
            0x55 -> {
                val b2 = inputStream.read()
                if (b2 == 0xAA) {
                    val hdr = ByteArray(2)
                    if (inputStream.read(hdr) < 2) return Pair("empty", null)
                    val length = ((hdr[0].toInt() and 0xFF) shl 8) or (hdr[1].toInt() and 0xFF)
                    val data = ByteArray(length)
                    var offset = 0
                    while (offset < length) {
                        val n = inputStream.read(data, offset, length - offset)
                        if (n < 0) break
                        offset += n
                    }
                    Pair("frame", data)
                } else {
                    Pair("empty", null)
                }
            }
            'C'.code -> {
                val rest = StringBuilder()
                var ch = inputStream.read()
                while (ch >= 0 && ch != '\n'.code) {
                    rest.append(ch.toChar())
                    ch = inputStream.read()
                }
                val line = "C$rest".trim()
                if (line.startsWith("CMD:")) Pair("cmd", line) else Pair("empty", null)
            }
            else -> Pair("empty", null)
        }
    }

    fun rms(pcm: ByteArray): Float {
        if (pcm.size < 2) return 0f
        val buf = ByteBuffer.wrap(pcm).order(ByteOrder.LITTLE_ENDIAN)
        val sampleCount = pcm.size / 2
        var sumSq = 0.0
        repeat(sampleCount) {
            val s = buf.short.toDouble() / 32768.0
            sumSq += s * s
        }
        return sqrt(sumSq / sampleCount).toFloat()
    }

    private fun processAndSend(
        pcm: ByteArray,
        apiKey: String,
        model: String,
        out: OutputStream,
        onLog: (String) -> Unit
    ) {
        try {
            val wav = pcmToWav(pcm)
            val wavBase64 = Base64.getEncoder().encodeToString(wav)
            val body = JSONObject().apply {
                put("model", model)
                put("modalities", JSONArray().apply { put("text"); put("audio") })
                put("audio", JSONObject().apply {
                    put("voice", "alloy")
                    put("format", "pcm16")
                })
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

            onLog("[TTS] Sending to OpenRouter (${pcm.size / 2 / SAMPLE_RATE}s audio)...")
            val response = httpClient.newCall(request).execute()
            if (!response.isSuccessful) {
                val errBody = response.body?.string() ?: ""
                onLog("[TTS] HTTP ${response.code}: $errBody")
                sendTtsEnd(out)
                return
            }

            val carry = ByteArrayOutputStream()
            val sendBuf = ByteArrayOutputStream()
            var sentBytes = 0

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

                        val pcmRaw = Base64.getDecoder().decode(dataStr)
                        carry.write(pcmRaw)
                        val carryBytes = carry.toByteArray()
                        val aligned = carryBytes.size - (carryBytes.size % 6)
                        if (aligned >= 6) {
                            val bbuf = ByteBuffer.wrap(carryBytes).order(ByteOrder.LITTLE_ENDIAN)
                            val samples24k = ShortArray(aligned / 2) { bbuf.short }
                            val samples16k = downsample3to2(samples24k)
                            val downBytes = ByteBuffer.allocate(samples16k.size * 2)
                                .order(ByteOrder.LITTLE_ENDIAN)
                                .also { b -> samples16k.forEach { b.putShort(it) } }
                                .array()
                            sendBuf.write(downBytes)
                            carry.reset()
                            if (aligned < carryBytes.size) {
                                carry.write(carryBytes, aligned, carryBytes.size - aligned)
                            }
                            while (sendBuf.size() >= TTS_CHUNK) {
                                val all = sendBuf.toByteArray()
                                sendTtsFrame(out, all.sliceArray(0 until TTS_CHUNK))
                                sentBytes += TTS_CHUNK
                                sendBuf.reset()
                                sendBuf.write(all, TTS_CHUNK, all.size - TTS_CHUNK)
                                Thread.sleep(10)
                            }
                        }
                    } catch (_: Exception) {
                        // skip malformed SSE chunks
                    }
                }
            }

            // flush remaining carry (pad to 6-byte boundary)
            val leftover = carry.toByteArray()
            if (leftover.size >= 2) {
                val pad = (6 - leftover.size % 6) % 6
                val padded = leftover + ByteArray(pad)
                val bbuf = ByteBuffer.wrap(padded).order(ByteOrder.LITTLE_ENDIAN)
                val samples24k = ShortArray(padded.size / 2) { bbuf.short }
                val samples16k = downsample3to2(samples24k)
                val downBytes = ByteBuffer.allocate(samples16k.size * 2)
                    .order(ByteOrder.LITTLE_ENDIAN)
                    .also { b -> samples16k.forEach { b.putShort(it) } }
                    .array()
                sendBuf.write(downBytes)
            }
            if (sendBuf.size() > 0) {
                sendTtsFrame(out, sendBuf.toByteArray())
                sentBytes += sendBuf.size()
            }
            sendTtsEnd(out)
            onLog("[TTS] Sent $sentBytes bytes of 16kHz audio")
        } catch (e: Exception) {
            onLog("[TTS] Error: ${e.message}")
            try { sendTtsEnd(out) } catch (_: Exception) {}
        }
    }

    fun pcmToWav(pcm: ByteArray): ByteArray {
        val dataSize = pcm.size
        val buf = ByteBuffer.allocate(44 + dataSize).order(ByteOrder.LITTLE_ENDIAN)
        buf.put("RIFF".toByteArray(Charsets.US_ASCII))
        buf.putInt(36 + dataSize)
        buf.put("WAVE".toByteArray(Charsets.US_ASCII))
        buf.put("fmt ".toByteArray(Charsets.US_ASCII))
        buf.putInt(16)            // PCM subchunk size
        buf.putShort(1)           // PCM format
        buf.putShort(1)           // mono
        buf.putInt(SAMPLE_RATE)
        buf.putInt(SAMPLE_RATE * 2) // byte rate
        buf.putShort(2)           // block align
        buf.putShort(16)          // bits per sample
        buf.put("data".toByteArray(Charsets.US_ASCII))
        buf.putInt(dataSize)
        buf.put(pcm)
        return buf.array()
    }

    fun downsample3to2(samples24k: ShortArray): ShortArray {
        val groups = samples24k.size / 3
        val out = ShortArray(groups * 2)
        for (i in 0 until groups) {
            out[i * 2] = samples24k[i * 3]
            out[i * 2 + 1] = ((samples24k[i * 3 + 1].toInt() + samples24k[i * 3 + 2].toInt()) / 2).toShort()
        }
        return out
    }

    fun sendTtsFrame(out: OutputStream, chunk: ByteArray) {
        val len = chunk.size
        val frame = ByteArray(4 + chunk.size)
        frame[0] = TTS_MAGIC[0]
        frame[1] = TTS_MAGIC[1]
        frame[2] = ((len shr 8) and 0xFF).toByte()
        frame[3] = (len and 0xFF).toByte()
        chunk.copyInto(frame, 4)
        synchronized(outLock) { out.write(frame); out.flush() }
    }

    fun sendTtsEnd(out: OutputStream) {
        synchronized(outLock) {
            out.write(TTS_MAGIC)
            out.write(0)
            out.write(0)
            out.flush()
        }
    }

    fun disconnect() {
        try { socket?.close() } catch (_: Exception) {}
        socket = null
    }
}
