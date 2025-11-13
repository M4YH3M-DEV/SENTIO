#!/bin/bash
echo "🧹 Cleaning up SENTIO repository..."

# Remove build artifacts
git rm -r --cached sentio/firmware/esp32_servo_bridge/.pio/ 2>/dev/null
git rm -r --cached sentio_ros2_ws/build/ 2>/dev/null
git rm -r --cached sentio_ros2_ws/install/ 2>/dev/null
git rm -r --cached sentio_ros2_ws/log/ 2>/dev/null

# Remove Python cache
find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
find . -name "*.pyc" -delete 2>/dev/null

# Update .gitignore
cat >> .gitignore << 'EOF'

# Build artifacts
.pio/
build/
install/
log/

# Python
__pycache__/
*.pyc
*.egg-info/
.eggs/

# IDE
.vscode/
.idea/
*.swp

# OS
.DS_Store
EOF

# Add placeholder tests
for dir in $(find sentio_ros2_ws/src -type d -name tests); do
  touch "$dir/__init__.py" 2>/dev/null
done

# Add environment example
cat > .env.example << 'EOF'
ROS_DISTRO=rolling
SENTIO_MODELS_DIR=/root/aether_sentio_ws/models
WHISPER_MODEL=base
WHISPER_DEVICE=cpu
EOF

# Commit
git add -A
git commit -m "chore: clean up repository - remove build artifacts, add environment template"

echo "✅ Repository cleanup complete!"