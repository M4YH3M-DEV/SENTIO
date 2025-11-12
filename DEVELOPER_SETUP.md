# Developer Setup Guide

Complete guide to setting up SENTIO development environment.

## Prerequisites

- **OS**: Ubuntu 22.04 or 24.04 (tested)
- **CPU**: 4+ cores recommended
- **RAM**: 8GB minimum (16GB recommended)
- **Disk**: 50GB free
- **Network**: Stable internet for package downloads

## Installation

### 1. Install System Dependencies

Update package manager
sudo apt update && sudo apt upgrade -y

Install base tools
sudo apt install -y
git
curl
wget
build-essential
libssl-dev
libffi-dev
python3-dev
python3.12
python3-pip
python3-venv

Install ROS 2 Rolling
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-rolling-desktop

Install colcon
sudo apt install -y python3-colcon-common-extensions

Install Node.js 20 via nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install 20

text

### 2. Clone Repository

Clone SENTIO
git clone https://github.com/devsora/sentio.git
cd sentio

Set git config for commits
git config user.name "Your Name"
git config user.email "your.email@example.com"

text

### 3. Setup Python Environment

Create virtual environment
python3.12 -m venv venv
source venv/bin/activate

Upgrade pip
pip install --upgrade pip setuptools wheel

Install development dependencies
pip install -r requirements-dev.txt

text

### 4. Setup ROS 2 Workspace

Navigate to workspace
cd sentio_ros2_ws

Install dependencies
source /opt/ros/rolling/setup.bash
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y

Build packages
colcon build --symlink-install

Source workspace
source install/setup.bash

(Add to ~/.bashrc for automatic sourcing)
echo "source ~/sentio/sentio_ros2_ws/install/setup.bash" >> ~/.bashrc

text

### 5. Setup Node.js UI

Navigate to UI
cd ../ui

Install dependencies
npm ci

Verify installation
npm run type-check

text

### 6. Setup Firmware Development

Install PlatformIO
pip install platformio

Verify installation
platformio --version

Install ESP32 platform
platformio platform install espressif32

text

## Verify Installation

### Python & ROS 2

Check Python
python3 --version # Should be 3.12.x

Check ROS 2
source /opt/ros/rolling/setup.bash
ros2 --version # Should show Rolling

Check colcon
colcon --version

text

### Build & Test

Run CI locally
cd sentio
./ci/test-runner.sh

Expected output: All tests pass (or warnings for optional components)
text

### Development Tools

Check black
black --version

Check pytest
pytest --version

Check ESLint
npm run lint

Check TypeScript
npm run type-check

text

## Development Workflow

### Daily Workflow

1. Start your terminal session
cd ~/sentio

2. Activate Python environment
source venv/bin/activate

3. Source ROS 2
source /opt/ros/rolling/setup.bash
source sentio_ros2_ws/install/setup.bash

4. Switch to feature branch
git checkout feature/my-feature

5. Make code changes, then:
6. Format code
black sentio_ros2_ws/ ui/
isort sentio_ros2_ws/ ui/

7. Run tests
pytest sentio_ros2_ws/src -v
npm test --prefix ui

8. Build ROS packages
cd sentio_ros2_ws && colcon build

9. Commit changes
git commit -m "feat(component): description"

text

### Running Individual Components

#### Perception Node

source /opt/ros/rolling/setup.bash
source sentio_ros2_ws/install/setup.bash

ros2 run sentio_perception emotion_recognition_node

text

#### Motion Control

source /opt/ros/rolling/setup.bash
source sentio_ros2_ws/install/setup.bash

ros2 run sentio_motion servo_bridge_node --simulate

text

#### Demo Runner

source /opt/ros/rolling/setup.bash
source sentio_ros2_ws/install/setup.bash

ros2 run sentio_demo demo_runner_node

text

#### Web UI

cd ui
npm run dev

Open http://localhost:3000
text

### Running Tests

Python unit tests
pytest sentio_ros2_ws/src -v --cov

ROS 2 integration tests
cd sentio_ros2_ws
colcon test --event-handlers console_direct+

Node.js tests
cd ui
npm test

Firmware tests
cd sentio/tools
python hardware_hil_test.py --port /dev/ttyUSB0

All tests locally (same as CI)
./ci/test-runner.sh

text

## IDE/Editor Setup

### VS Code

1. Install extensions:
   - Python
   - Pylance
   - ROS
   - ESLint
   - Prettier
   - Thunder Client

2. Create `.vscode/settings.json`:

{
"[python]": {
"editor.defaultFormatter": "ms-python.black-formatter",
"editor.formatOnSave": true,
"editor.rulers":
},
"[typescript]": {
"editor.defaultFormatter": "esbenp.prettier-vscode",
"editor.formatOnSave": true
},
"python.linting.enabled": true,
"python.linting.flake8Enabled": true,
"python.linting.flake8Args": ["--max-line-length=100"],
"editor.codeActionsOnSave": {
"source.fixAll.eslint": true
}
}

text

### PyCharm

1. Install PyCharm Community or Professional
2. Open project
3. Configure Python interpreter: `/path/to/sentio/venv/bin/python`
4. Set code style: Editor → Code Style → Scheme → Set to "Black"
5. Enable ROS plugin: Settings → Plugins → Search "ROS"

## Troubleshooting

### Issue: `source: command not found`

**Cause**: Using shell that doesn't support source (e.g., zsh)
**Fix**: Use `source` or `.` explicitly

source venv/bin/activate
. venv/bin/activate # Alternative

text

### Issue: ROS 2 command not found

**Cause**: ROS 2 not sourced
**Fix**: Source ROS 2 setup

source /opt/ros/rolling/setup.bash

text

### Issue: Python dependencies conflict

**Cause**: Virtual environment not used
**Fix**: Ensure venv is activated

source venv/bin/activate
which python # Should show /path/to/sentio/venv/bin/python

text

### Issue: colcon build fails with "package not found"

**Cause**: ROS 2 dependency not resolved
**Fix**: Run rosdep and rebuild

rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

text

### Issue: USB port permission denied

**Cause**: User not in dialout group
**Fix**: Add user to group

sudo usermod -a -G dialout $USER

Logout and login required
text

## Performance Optimization

### Speed up builds

Use all available cores
colcon build --symlink-install -j 4 # Adjust to your CPU cores

Skip tests during development
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

text

### Speed up npm

cd ui
npm ci --prefer-offline --no-audit

text

## Advanced Setup

### Docker Development Environment

Build Docker image
docker-compose -f ci/docker-compose.yml build

Run in container
docker-compose -f ci/docker-compose.yml up

text

### Remote Development

SSH into remote machine
ssh user@remote-host

Follow setup guide above on remote
Then use VS Code Remote SSH extension
text

## Next Steps

1. **Read ONBOARDING.md** for first task suggestions
2. **Check CONTRIBUTING.md** for development guidelines
3. **Review CI_README.md** for CI/CD details
4. **Run local tests** to verify setup
5. **Create feature branch** and start coding

## Getting Help

- **Documentation**: See individual README.md files
- **GitHub Issues**: Report bugs or request features
- **Email**: dev@devsora.tech
- **Community**: Discussions on GitHub

---

**Last Updated**: 2025-11-13