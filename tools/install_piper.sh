#!/bin/bash
# Install Piper TTS on host system

set -e

echo "=== Installing Piper TTS ==="

# Check if Python venv exists
VENV_DIR="${HOME}/venvs/piper"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating Python virtual environment at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

# Activate venv
source "$VENV_DIR/bin/activate"

echo "Installing Piper TTS via pip..."
pip install --upgrade pip setuptools wheel
pip install piper-tts

echo "Downloading English model (if not present)..."
mkdir -p ~/.local/share/piper

# Download a lightweight English model
if [ ! -f ~/.local/share/piper/en_US-lessac-medium.onnx ]; then
    echo "Downloading en_US-lessac-medium model..."
    cd ~/.local/share/piper
    
    # Download model and JSON
    wget -q https://github.com/rhasspy/piper/releases/download/2023.11.14-1/en_US-lessac-medium.onnx
    wget -q https://github.com/rhasspy/piper/releases/download/2023.11.14-1/en_US-lessac-medium.onnx.json
    
    echo "Model downloaded successfully"
fi

# Verify installation
echo "Verifying Piper installation..."
python3 -c "import piper_tts; print('✓ Piper TTS installed successfully')"

echo "=== Piper TTS Installation Complete ==="
echo "Virtual environment: $VENV_DIR"
echo "To activate: source $VENV_DIR/bin/activate"
