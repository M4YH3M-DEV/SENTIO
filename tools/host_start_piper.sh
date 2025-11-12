#!/bin/bash
# Start Piper TTS HTTP server

set -e

# Configuration
VENV_DIR="${HOME}/venvs/piper"
PORT=5000
HOST="127.0.0.1"
LOG_FILE="/tmp/piper_server.log"
PID_FILE="/tmp/piper_server.pid"

# Check if venv exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Piper venv not found at $VENV_DIR"
    echo "Run: ./tools/install_piper.sh first"
    exit 1
fi

# Activate venv
source "$VENV_DIR/bin/activate"

# Check if already running
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if kill -0 "$OLD_PID" 2>/dev/null; then
        echo "Piper server already running (PID: $OLD_PID)"
        echo "URL: http://$HOST:$PORT"
        exit 0
    else
        rm "$PID_FILE"
    fi
fi

echo "Starting Piper TTS server..."
echo "  Host: $HOST"
echo "  Port: $PORT"
echo "  Log: $LOG_FILE"

# Start Piper HTTP server
nohup python3 -m piper.server \
    --host "$HOST" \
    --port "$PORT" \
    --cuda \
    > "$LOG_FILE" 2>&1 &

NEW_PID=$!
echo "$NEW_PID" > "$PID_FILE"

echo "Server started (PID: $NEW_PID)"

# Wait for server to start
sleep 2

# Check if server is responding
for i in {1..10}; do
    if curl -s "http://$HOST:$PORT/api/voices" > /dev/null 2>&1; then
        echo "✓ Piper server is ready"
        echo "  API: http://$HOST:$PORT/api/voices"
        echo "  Synthesize: POST http://$HOST:$PORT/api/synthesize"
        exit 0
    fi
    
    if [ $i -lt 10 ]; then
        echo "Waiting for server to start... ($i/10)"
        sleep 1
    fi
done

echo "⚠ Server started but not responding to health check"
echo "Check log: tail -f $LOG_FILE"
exit 1
