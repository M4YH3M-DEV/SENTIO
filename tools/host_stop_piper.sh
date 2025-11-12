#!/bin/bash
# Stop Piper TTS HTTP server

set -e

PID_FILE="/tmp/piper_server.pid"

if [ ! -f "$PID_FILE" ]; then
    echo "Piper server not running (no PID file)"
    exit 0
fi

PID=$(cat "$PID_FILE")

if kill -0 "$PID" 2>/dev/null; then
    echo "Stopping Piper server (PID: $PID)..."
    kill "$PID"
    
    # Wait for graceful shutdown
    for i in {1..10}; do
        if ! kill -0 "$PID" 2>/dev/null; then
            echo "✓ Piper server stopped"
            rm "$PID_FILE"
            exit 0
        fi
        sleep 1
    done
    
    # Force kill if necessary
    echo "Force killing Piper server..."
    kill -9 "$PID" || true
    rm "$PID_FILE"
    echo "✓ Piper server stopped"
else
    echo "PID $PID not running, cleaning up..."
    rm "$PID_FILE"
fi
