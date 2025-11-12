# SENTIO Developer Onboarding

Welcome to the SENTIO project! This guide will help you get productive quickly.

## First Day Checklist

- [ ] Complete DEVELOPER_SETUP.md
- [ ] Clone repository and verify build
- [ ] Join development Slack/Discord
- [ ] Create GitHub account and get access
- [ ] Set up IDE/editor
- [ ] Run local CI pipeline
- [ ] Try building firmware (optional)

## Understanding the Project

### Architecture Overview

SENTIO Robot System
│
├── Phase 1-5: Core Infrastructure & Perception
│ ├── drivers/ → Sensor I/O (cameras, IMU, rangefinder)
│ ├── perception/ → Emotion detection from images/audio
│ ├── fusion/ → AETHER engine (explainable reasoning)
│ └── ...
│
├── Phase 6-8: Decision Making & Action
│ ├── policy/ → Behavior selection engine
│ ├── motion/ → Servo control & kinematics
│ ├── demo/ → Choreography sequences
│ └── ...
│
├── Phase 9: User Interface
│ └── ui/ → Next.js HUD (real-time dashboard)
│
├── Phase 10: Hardware & Firmware
│ ├── firmware/ → ESP32 servo bridge
│ ├── hardware/ → PCB, BOM, assembly guides
│ └── tools/ → Hardware testing scripts
│
└── Phase 11: CI/CD & Testing (you are here!)
├── .github/workflows/ → GitHub Actions
├── ci/ → Local testing infrastructure
└── docs/ → Developer documentation

text

### Key Components

**ROS 2 Packages**:
- `sentio_core` - Core message definitions
- `sentio_perception` - Vision/audio emotion detection
- `sentio_fusion` - Decision reasoning (AETHER)
- `sentio_policy` - Behavior mapping
- `sentio_motion` - Servo/motion control
- `sentio_demo` - Demo choreography
- `sentio_tts_bridge` - Text-to-speech integration

**Frontend**:
- Next.js web UI in `ui/`
- Real-time ROS topic monitoring
- Live emotion visualization
- Demo control interface

**Firmware**:
- ESP32 servo bridge in `sentio/firmware/`
- JSON-over-serial protocol
- PCA9685 PWM driver

## How to Contribute

### Good First Issues

1. **Fix a documentation typo**
   - Branch: `fix/docs-typo`
   - Time: 5-10 minutes
   - Run: `git grep "typo" README.md`

2. **Add a unit test for existing code**
   - Branch: `test/add-coverage-foo`
   - Time: 30 minutes
   - Location: `sentio_ros2_ws/src/*/tests/`

3. **Improve error messages**
   - Branch: `improve/error-messages`
   - Time: 30 minutes
   - Search: `raise Exception("Fix me!")`

4. **Add new emotion gesture to choreography**
   - Branch: `feat/gestures-anxiety`
   - Time: 1 hour
   - Location: `sentio_ros2_ws/src/sentio_demo/choreography/`

### Medium Tasks

- Add new topic type to perception pipeline
- Implement gesture interpolation improvement
- Add performance monitoring to fusion engine
- Create new demo sequence

### Larger Features

- New emotion detection model
- Alternative backend for TTS
- New hardware integration (LED strip, microphone)
- Optimize motion planning

## Running Your First Test

1. Setup (one-time)
cd ~/sentio
source venv/bin/activate
source /opt/ros/rolling/setup.bash
source sentio_ros2_ws/install/setup.bash

2. Run tests
cd sentio_ros2_ws/src/sentio_perception
pytest tests/ -v

Expected: See test discovery and results
text

## Making Your First PR

### Step 1: Create Branch

git checkout develop
git pull origin develop
git checkout -b feat/first-contribution

text

### Step 2: Make Change

Example: Fix a typo in documentation

Edit file
nano DEVELOPER_SETUP.md

Verify change looks good
git diff DEVELOPER_SETUP.md

text

### Step 3: Test Locally

Format
black .
isort .

Run tests
pytest sentio_ros2_ws/src -v

text

### Step 4: Commit

git add DEVELOPER_SETUP.md
git commit -m "docs: fix typo in setup guide"

text

### Step 5: Push & PR

git push origin feat/first-contribution

text

Then go to GitHub and create PR. CI will automatically run.

### Step 6: Iterate

- Wait for review comments
- Make requested changes
- Push new commits (don't amend or force push)
- Respond to comments
- Once approved, maintainer merges PR

## Code Review Guidelines

When your PR is reviewed:

1. **Read comments carefully** - try to understand the feedback
2. **Ask questions** - if something is unclear
3. **Make changes promptly** - shows good faith
4. **Respond to all comments** - acknowledge feedback
5. **Push new commits** - don't amend (easier to see changes)

Example response:
Thanks for the feedback! I've updated the function to use list
comprehension instead of a loop. Also added a type hint for the
return value as suggested.

text

## Common Workflows

### Adding a New Feature

1. Create branch from develop
git checkout -b feature/my-feature

2. Make changes with tests
Edit files...
Add tests in tests/ directory
Update docs
3. Run full CI locally
./ci/test-runner.sh

4. If tests pass:
git add .
git commit -m "feat(component): description"
git push origin feature/my-feature

5. Create PR on GitHub
text

### Fixing a Bug

1. Create branch from develop
git checkout -b fix/issue-123

2. Write regression test
(Test that fails with current code, passes with fix)
pytest tests/ -v

3. Implement fix
(Make test pass)
4. Verify no other tests broken
./ci/test-runner.sh

5. Commit with reference to issue
git commit -m "fix: resolve issue #123

Fixes issue where servos would not respond to commands
when watchdog timer was active. The timer now correctly
checks for recent commands before triggering shutdown.

Fixes #123"

git push origin fix/issue-123

text

### Updating Documentation

1. Edit markdown files
nano DEVELOPER_SETUP.md

2. Preview (if using VS Code)
Ctrl+Shift+V # Markdown preview

3. Commit
git commit -m "docs: update setup instructions for Python 3.12"

4. Push (no formal review needed for doc-only PRs)
text

## Debugging

### Python Debugging

Add breakpoint in code
import pdb; pdb.set_trace()

Run with pytest
pytest tests/test_foo.py -v -s # -s shows print output

Or use IDE debugger (VS Code: F5)
text

### ROS 2 Debugging

Run node with verbose output
ros2 run sentio_perception emotion_recognition_node --ros-args --log-level info

Monitor topic
ros2 topic echo /affect

Check node status
ros2 node list
ros2 node info /emotion_recognition_node

text

### UI Debugging

cd ui
npm run dev # Starts dev server with hot reload

Browser DevTools: F12
React DevTools: Chrome extension
Redux DevTools: If using Redux
text

## Performance Tips

### Speed Up Builds

Use symlink-install (faster rebuilds)
colcon build --symlink-install

Parallel build
colcon build --symlink-install -j 4

text

### Speed Up Tests

Run specific test file (faster than full suite)
pytest sentio_ros2_ws/src/sentio_perception/tests/test_emotion.py -v

Run in parallel
pytest -n auto

text

### Speed Up npm

Use npm ci (reproducible)
npm ci --prefer-offline

Check package-lock.json into git
git add ui/package-lock.json

text

## Getting Unstuck

### Issue: Tests fail locally but pass on CI

**Solution**: 
1. Check Python version matches (`python3 --version`)
2. Clear build cache: `rm -rf sentio_ros2_ws/build install`
3. Rebuild: `colcon build --symlink-install`
4. Ask for help in GitHub issue

### Issue: Can't build firmware

**Solution**:
1. Install PlatformIO: `pip install platformio`
2. Update platform: `platformio platform update espressif32`
3. Clean build: `rm -rf .pio && platformio run`

### Issue: Weird import errors

**Solution**:
1. Ensure venv is activated: `which python`
2. Reinstall dependencies: `pip install -r requirements-dev.txt`
3. Rebuild: `colcon build`

## Questions?

- **GitHub Issues**: Create issue with label `question`
- **Documentation**: Check CONTRIBUTING.md or DEVELOPER_SETUP.md
- **Slack/Discord**: Ask in #development channel
- **Email**: dev@devsora.tech

## Next Steps

1. ✅ **Complete setup** (DEVELOPER_SETUP.md)
2. 🏃 **Run tests locally** (`./ci/test-runner.sh`)
3. 🌳 **Create feature branch** (`git checkout -b feature/...`)
4. ✍️ **Make small change** (documentation or test)
5. 🚀 **Create your first PR**
6. 🎉 **Welcome to the team!**

---

**Last Updated**: 2025-11-13
**Questions?** Email: info@devsora.com