package com.g150446.voice_harness

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.Binder
import android.os.IBinder
import androidx.core.app.NotificationCompat
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class BtSppService : Service() {

    companion object {
        const val ACTION_CONNECT = "connect"
        const val ACTION_DISCONNECT = "disconnect"
        const val NOTIF_ID = 1
        const val CHANNEL_ID = "bt_spp"
    }

    inner class LocalBinder : Binder() {
        fun getService() = this@BtSppService
    }

    private val binder = LocalBinder()
    private val serviceScope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    private val prefs by lazy { getSharedPreferences("voice_bridge", MODE_PRIVATE) }
    private val sppBridge = SppBridge()
    private val localTest = LocalTest()
    private var connectJob: Job? = null
    private var testJob: Job? = null

    val state = MutableStateFlow(BridgeState.IDLE)
    val logs = MutableStateFlow<List<String>>(emptyList())
    val testState = MutableStateFlow("IDLE")

    override fun onBind(intent: Intent?): IBinder = binder

    override fun onCreate() {
        super.onCreate()
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_CONNECT    -> connect()
            ACTION_DISCONNECT -> disconnect()
        }
        return START_NOT_STICKY
    }

    fun connect() {
        connectJob?.cancel()
        state.value = BridgeState.CONNECTING
        startForeground(NOTIF_ID, buildNotification())
        addLog("[BT] Connecting...")
        connectJob = serviceScope.launch {
            try {
                withContext(Dispatchers.IO) { sppBridge.connect(applicationContext) }
                state.value = BridgeState.CONNECTED
                addLog("[BT] Connected — PING/PONG verified")
                updateNotification()
                sppBridge.runLoop(
                    apiKey = prefs.getString("openrouter_key", "") ?: "",
                    model  = prefs.getString("openrouter_model", "openai/gpt-audio-mini")
                               ?: "openai/gpt-audio-mini",
                    onLog  = { addLog(it) },
                    onStateChange = { s ->
                        state.value = when (s) {
                            "RECORDING"  -> BridgeState.RECORDING
                            "PROCESSING" -> BridgeState.PROCESSING
                            "CONNECTED"  -> BridgeState.CONNECTED
                            else         -> BridgeState.IDLE
                        }
                        updateNotification()
                    }
                )
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                state.value = BridgeState.IDLE
                addLog("[ERR] ${e.message ?: e.javaClass.simpleName}")
                stopForeground(STOP_FOREGROUND_REMOVE)
                stopSelf()
            }
        }
    }

    fun disconnect() {
        connectJob?.cancel()
        connectJob = null
        sppBridge.disconnect()
        state.value = BridgeState.IDLE
        addLog("[BT] Disconnected")
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    fun onTestTap() {
        when (testState.value) {
            "IDLE" -> {
                testJob?.cancel()
                testState.value = "REC"
                addLog("[TEST] Recording — tap again to stop...")
                testJob = serviceScope.launch {
                    val pcm = withContext(Dispatchers.IO) { localTest.record() }
                    if (pcm.size < 3200) {
                        addLog("[TEST] Too short, discarded")
                        testState.value = "IDLE"
                        return@launch
                    }
                    testState.value = "SEND"
                    withContext(Dispatchers.IO) {
                        localTest.sendAndPlay(
                            pcm    = pcm,
                            apiKey = prefs.getString("openrouter_key", "") ?: "",
                            model  = prefs.getString("openrouter_model", "openai/gpt-audio-mini")
                                       ?: "openai/gpt-audio-mini",
                            onLog  = { addLog(it) },
                            onState = { s -> testState.value = s }
                        )
                    }
                    testState.value = "IDLE"
                }
            }
            "REC" -> localTest.stopRecording()
            // SEND / PLAY — ignore taps
        }
    }

    fun addLog(msg: String) {
        logs.value = (logs.value + msg).takeLast(200)
    }

    override fun onDestroy() {
        serviceScope.cancel()
        sppBridge.disconnect()
        localTest.stopRecording()
        super.onDestroy()
    }

    private fun createNotificationChannel() {
        val channel = NotificationChannel(
            CHANNEL_ID,
            "Bluetooth SPP",
            NotificationManager.IMPORTANCE_LOW
        )
        getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
    }

    private fun buildNotification(): Notification {
        val openIntent = PendingIntent.getActivity(
            this, 0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE
        )
        val disconnectIntent = PendingIntent.getService(
            this, 0,
            Intent(this, BtSppService::class.java).apply { action = ACTION_DISCONNECT },
            PendingIntent.FLAG_IMMUTABLE
        )

        val statusText = when (state.value) {
            BridgeState.IDLE       -> "Disconnected"
            BridgeState.CONNECTING -> "Connecting..."
            BridgeState.CONNECTED  -> "Connected"
            BridgeState.RECORDING  -> "Recording"
            BridgeState.PROCESSING -> "Processing..."
        }

        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_launcher_foreground)
            .setContentTitle("Voice Harness")
            .setContentText(statusText)
            .setContentIntent(openIntent)
            .setOngoing(true)
            .apply {
                if (state.value != BridgeState.IDLE)
                    addAction(android.R.drawable.ic_delete, "Disconnect", disconnectIntent)
            }
            .build()
    }

    private fun updateNotification() {
        getSystemService(NotificationManager::class.java).notify(NOTIF_ID, buildNotification())
    }
}
