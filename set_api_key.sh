#!/bin/bash
# Send OPENROUTER_API_KEY to ESP32 via serial

set -e

PORT=$(ls /dev/tty.usbserial-* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
    echo "Error: No USB serial device found"
    exit 1
fi

if [ -z "$OPENROUTER_API_KEY" ]; then
    echo "Error: OPENROUTER_API_KEY is not set"
    exit 1
fi

echo "Sending API key to $PORT..."
stty -f "$PORT" 115200 raw -echo
echo "SET_KEY:${OPENROUTER_API_KEY}" > "$PORT"
sleep 1
# Read response
timeout 2 cat "$PORT" 2>/dev/null || true
echo ""
echo "Done."
