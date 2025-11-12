# SENTIO Demo Runner Package

Choreography engine for coordinated robot demonstrations and performances.

## Features

- YAML-based choreography sequences
- Timed action scheduling (gestures, TTS, LEDs)
- Pause/resume/stop playback control
- Variable substitution for customization
- Service-based control interface
- Execution logging and status reporting

## Building

### Prerequisites

pip install pyyaml

text

### Build

cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/sentio/ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --packages-select sentio_demo --symlink-install
source install/setup.bash

text

## Running

### Launch Demo Runner

ros2 launch sentio_demo demo_launch.py sequence:=greet_sequence auto_start:=true

text

### Run Default Demo

ros2 run sentio_demo demo_runner_node

text

### Monitor Demo Status

ros2 topic echo /demo/status

text

## Choreography Sequences

Sequences are defined in YAML format in the `choreography/` directory.

### Supported Actions

- **gesture**: Physical gesture/pose
- **tts**: Text-to-speech
- **led**: LED control
- **wait**: Pause

### Example Sequence

name: "Hello"
steps:

action: "gesture"
gesture: "greet"
duration_ms: 1000
led:
color: "cyan"
pattern: "steady"

action: "tts"
text: "Hello there!"
emotion: "happy"
delay_ms: 300

action: "wait"
duration_ms: 2000

text

## Testing

### Unit Tests

cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/sentio/ros2_ws/src/sentio_demo
python -m pytest tests/ -v

text

### Manual Test

Terminal 1: Launch all systems (drivers, perception, fusion, policy, motion)
ros2 launch sentio_drivers all_drivers_launch.py
ros2 launch sentio_perception all_perception_launch.py
ros2 run sentio_fusion fusion_node
ros2 run sentio_policy policy_engine_node
ros2 run sentio_motion servo_bridge_node --simulate

Terminal 2: Start demo runner
ros2 run sentio_demo demo_runner_node

Terminal 3: Trigger demo
ros2 topic pub /trigger_demo std_msgs/String '{data: "greet_sequence"}' -1

Monitor status
ros2 topic echo /demo/status

text

## CLI Tool

python tools/run_demo.py --sequence showcase_full --simulate

text

## Integration Examples

### Via Python

import rclpy
from sentio_demo.choreography_engine import ChoreographyEngine
from sentio_demo.sequence_parser import SequenceParser
from sentio_demo.demo_controller import DemoController

parser = SequenceParser()
parser.load_sequence("choreography/greet_sequence.yaml")

choreography = ChoreographyEngine()
controller = DemoController(choreography, parser)

Start demo
controller.start_demo("greet_sequence", variables={"visitor_name": "Alice"})

text

### Via rosbridge (HTTP REST)

See `examples/rosbridge_trigger_demo.py` and `examples/curl_trigger_demo.sh`

## Topics

### Publications

- `/tts_text` (std_msgs/String) - TTS commands
- `/behavior_cmd` (std_msgs/String) - Behavior/gesture commands
- `/demo/status` (std_msgs/String) - JSON status

## Services (Future)

- `/demo/control` - Start/stop/pause/resume
- `/demo/load_sequence` - Load new sequence dynamically

## Choreography Directory

Sequences are stored in `choreography/` with examples:
- `greet_sequence.yaml` - Simple greeting
- `showcase_full.yaml` - Full capability showcase
- `nod_sequence.yaml` - Simple nod gesture

## Creating Custom Sequences

1. Create YAML file in `choreography/`
2. Define steps with actions
3. Optionally add variable placeholders
4. Run via launcher or CLI

See `choreography/README.md` for detailed format.

## License

Proprietary - DevSora Deep-Tech Research