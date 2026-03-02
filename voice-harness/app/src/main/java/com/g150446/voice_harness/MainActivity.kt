package com.g150446.voice_harness

import android.Manifest
import android.app.Application
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Visibility
import androidx.compose.material.icons.filled.VisibilityOff
import androidx.compose.material3.Button
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.compose.viewModel
import com.g150446.voice_harness.ui.theme.VoiceHarnessTheme
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

enum class BridgeState { IDLE, CONNECTING, CONNECTED, RECORDING, PROCESSING }

class MainViewModel(app: Application) : AndroidViewModel(app) {

    private val _state     = MutableStateFlow(BridgeState.IDLE)
    private val _logs      = MutableStateFlow<List<String>>(emptyList())
    private val _testState = MutableStateFlow("IDLE")

    val state:     StateFlow<BridgeState>  = _state.asStateFlow()
    val logs:      StateFlow<List<String>> = _logs.asStateFlow()
    val testState: StateFlow<String>       = _testState.asStateFlow()

    private var service: BtSppService? = null
    private var isBound = false
    private var collectJobs = listOf<Job>()

    private val serviceConn = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName, binder: IBinder) {
            val svc = (binder as BtSppService.LocalBinder).getService()
            service = svc
            isBound = true
            collectJobs = listOf(
                viewModelScope.launch { svc.state.collect     { _state.value     = it } },
                viewModelScope.launch { svc.logs.collect      { _logs.value      = it } },
                viewModelScope.launch { svc.testState.collect { _testState.value = it } }
            )
        }

        override fun onServiceDisconnected(name: ComponentName) {
            collectJobs.forEach { it.cancel() }
            collectJobs = emptyList()
            service = null
            isBound = false
            _state.value = BridgeState.IDLE
        }
    }

    fun bindService(context: Context) {
        if (!isBound)
            context.bindService(Intent(context, BtSppService::class.java), serviceConn, 0)
    }

    fun unbindService(context: Context) {
        if (isBound) {
            context.unbindService(serviceConn)
            isBound = false
        }
    }

    fun connect(context: Context) {
        ContextCompat.startForegroundService(
            context,
            Intent(context, BtSppService::class.java).apply { action = BtSppService.ACTION_CONNECT }
        )
        if (!isBound)
            context.bindService(
                Intent(context, BtSppService::class.java),
                serviceConn,
                Context.BIND_AUTO_CREATE
            )
    }

    fun disconnect() { service?.disconnect() }
    fun onTestTap()  { service?.onTestTap() }

    fun addLog(msg: String) {
        _logs.value = (_logs.value + msg).takeLast(200)
    }

    override fun onCleared() {
        super.onCleared()
        collectJobs.forEach { it.cancel() }
    }
}

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            VoiceHarnessTheme {
                AppRoot()
            }
        }
    }
}

@Composable
fun AppRoot(vm: MainViewModel = viewModel()) {
    var screen by remember { mutableStateOf("main") }
    val context = LocalContext.current

    // Bind to service when UI is visible; service keeps running in background
    DisposableEffect(Unit) {
        vm.bindService(context)
        onDispose { vm.unbindService(context) }
    }

    // Request POST_NOTIFICATIONS once on Android 13+
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
        val notifPermLauncher = rememberLauncherForActivityResult(
            ActivityResultContracts.RequestPermission()
        ) { /* ignore result — notification is optional */ }
        LaunchedEffect(Unit) {
            if (ContextCompat.checkSelfPermission(context, Manifest.permission.POST_NOTIFICATIONS)
                != PackageManager.PERMISSION_GRANTED
            ) {
                notifPermLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
            }
        }
    }

    val btPermissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { results ->
        if (results.values.all { it }) vm.connect(context)
        else vm.addLog("[ERR] Bluetooth permissions denied")
    }

    val micPermissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) vm.onTestTap()
        else vm.addLog("[TEST] Microphone permission denied")
    }

    fun handleConnect() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val needed = listOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN
            ).filter {
                ContextCompat.checkSelfPermission(context, it) != PackageManager.PERMISSION_GRANTED
            }
            if (needed.isNotEmpty()) {
                btPermissionLauncher.launch(needed.toTypedArray())
                return
            }
        }
        vm.connect(context)
    }

    fun handleTestTap() {
        // Only check permission when starting a new recording
        if (vm.testState.value == "IDLE" &&
            ContextCompat.checkSelfPermission(context, Manifest.permission.RECORD_AUDIO)
            != PackageManager.PERMISSION_GRANTED
        ) {
            micPermissionLauncher.launch(Manifest.permission.RECORD_AUDIO)
            return
        }
        vm.onTestTap()
    }

    when (screen) {
        "settings" -> SettingsScreen(onBack = { screen = "main" })
        else -> MainScreen(
            vm = vm,
            onSettings = { screen = "settings" },
            onConnect = ::handleConnect,
            onDisconnect = { vm.disconnect() },
            onTestTap = ::handleTestTap
        )
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainScreen(
    vm: MainViewModel,
    onSettings: () -> Unit,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    onTestTap: () -> Unit
) {
    val state by vm.state.collectAsState()
    val logs by vm.logs.collectAsState()
    val testState by vm.testState.collectAsState()
    val listState = rememberLazyListState()

    LaunchedEffect(logs.size) {
        if (logs.isNotEmpty()) listState.animateScrollToItem(logs.size - 1)
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Voice Harness") },
                actions = {
                    IconButton(onClick = onSettings) {
                        Icon(Icons.Filled.Settings, contentDescription = "Settings")
                    }
                }
            )
        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            // Status chip
            val (chipLabel, chipColor) = when (state) {
                BridgeState.IDLE        -> "Disconnected" to MaterialTheme.colorScheme.surfaceVariant
                BridgeState.CONNECTING  -> "Connecting..." to MaterialTheme.colorScheme.secondaryContainer
                BridgeState.CONNECTED   -> "Connected" to MaterialTheme.colorScheme.primaryContainer
                BridgeState.RECORDING   -> "Recording" to MaterialTheme.colorScheme.errorContainer
                BridgeState.PROCESSING  -> "Processing" to MaterialTheme.colorScheme.tertiaryContainer
            }
            Surface(
                color = chipColor,
                shape = MaterialTheme.shapes.medium
            ) {
                Text(
                    text = chipLabel,
                    modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp),
                    style = MaterialTheme.typography.labelLarge
                )
            }

            // Connect / Disconnect button
            Button(
                onClick = if (state == BridgeState.IDLE) onConnect else onDisconnect,
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(if (state == BridgeState.IDLE) "Connect" else "Disconnect")
            }

            // Test button (phone mic → OpenRouter → phone speaker)
            val (testLabel, testEnabled) = when (testState) {
                "REC"  -> "Stop & Send" to true
                "SEND" -> "Processing..." to false
                "PLAY" -> "Playing..." to false
                else   -> "Test Voice (no M5)" to true
            }
            OutlinedButton(
                onClick = onTestTap,
                enabled = testEnabled,
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(testLabel)
            }

            // Scrollable log
            Surface(
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f),
                color = MaterialTheme.colorScheme.surfaceVariant,
                shape = MaterialTheme.shapes.medium
            ) {
                LazyColumn(
                    state = listState,
                    modifier = Modifier.padding(8.dp),
                    verticalArrangement = Arrangement.spacedBy(2.dp)
                ) {
                    items(logs) { line ->
                        Text(
                            text = line,
                            style = MaterialTheme.typography.bodySmall,
                            fontFamily = FontFamily.Monospace
                        )
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(onBack: () -> Unit) {
    val context = LocalContext.current
    val prefs = remember { context.getSharedPreferences("voice_bridge", Context.MODE_PRIVATE) }
    var apiKey by remember { mutableStateOf(prefs.getString("openrouter_key", "") ?: "") }
    var model by remember {
        mutableStateOf(
            prefs.getString("openrouter_model", "openai/gpt-audio-mini")
                ?: "openai/gpt-audio-mini"
        )
    }
    var showKey by remember { mutableStateOf(false) }
    var saved by remember { mutableStateOf(false) }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Settings") },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = "Back")
                    }
                }
            )
        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            OutlinedTextField(
                value = apiKey,
                onValueChange = { apiKey = it; saved = false },
                label = { Text("OpenRouter API Key") },
                modifier = Modifier.fillMaxWidth(),
                visualTransformation = if (showKey) VisualTransformation.None else PasswordVisualTransformation(),
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
                trailingIcon = {
                    IconButton(onClick = { showKey = !showKey }) {
                        Icon(
                            if (showKey) Icons.Filled.VisibilityOff else Icons.Filled.Visibility,
                            contentDescription = if (showKey) "Hide key" else "Show key"
                        )
                    }
                }
            )

            OutlinedTextField(
                value = model,
                onValueChange = { model = it; saved = false },
                label = { Text("Model") },
                modifier = Modifier.fillMaxWidth()
            )

            Button(
                onClick = {
                    prefs.edit()
                        .putString("openrouter_key", apiKey)
                        .putString("openrouter_model", model)
                        .apply()
                    saved = true
                },
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(if (saved) "Saved ✓" else "Save")
            }
        }
    }
}
