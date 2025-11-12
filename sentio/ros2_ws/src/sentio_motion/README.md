# SENTIO Motion Control Package

Motion and actuation layer for SENTIO robot. Controls servos, gestures, and physical expressions via ESP32 firmware.

## Features

- JSON-based servo command protocol
- Multi-frame gesture sequences with keyframe interpolation
- Smooth motion trajectories with velocity limiting
- IMU-based fall detection and emergency stop
- Supports both ESP32 serial and PCA9685 I2C control
- Comprehensive safety monitoring
- Simulation mode for development/testing

## Building

### Prerequisites

sudo apt-get install -y python3-pip
pip install pyserial pyyaml

text

### Build

cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/sentio/ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --packages-select sentio_motion --symlink-install
source install/setup.bash

text

## Running

### Hardware Mode

Connect ESP32 via USB and run:

ros2 run sentio_motion servo_bridge_node

text

### Simulation Mode

ros2 run sentio_motion servo_bridge_node --simulate

text

### Launch with Configuration

ros2 launch sentio_motion motion_launch.py simulate:=true

text

## Testing

### Unit Tests

cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/sentio/ros2_ws/src/sentio_motion
python -m pytest tests/ -v

text

### Manual Command Test

Publish a behavior command
ros2 topic pub /behavior_cmd std_msgs/String '{data: "{"gesture": "nod", "led": {"color": "blue", "pattern": "pulse"}, "tts": {"text": ""}}"}' -1

Monitor status
ros2 topic echo /motion/status

text

## Serial Protocol

### JSON Frame Format

#### Servo Move Command

{
"type": "servo_move",
"positions": {
"0": 90.0,
"1": 95.5,
"2": 85.0
},
"duration_ms": 500,
"velocity_limit": 60.0,
"smoothing": true
}

text

#### LED Control Command

{
"type": "led_control",
"color": ,
"pattern": "pulse",
"intensity": 0.8
}

text

#### Response

{
"status": "ok",
"positions": {"0": 90, "1": 95, "2": 85},
"duration_ms": 500
}

text

Or on error:

{
"status": "error",
"msg": "Servo 99 out of range"
}

text

## Configuration Files

- `config/kinematics.yaml` - Servo mappings and gesture definitions
- `config/servo_map.yaml` - Hardware channel assignments
- `config/safety_limits.yaml` - Safety thresholds and limits

## Topics

### Subscriptions

- `/behavior_cmd` (std_msgs/String) - Behavior JSON command
- `/motion/override` (std_msgs/String) - Emergency override ("estop", "clear_estop")
- `/imu/data` (sensor_msgs/Imu) - IMU data for safety monitoring

### Publications

- `/motion/status` (std_msgs/String) - JSON status with command count and errors
- `/motion/feedback` (std_msgs/String) - Servo feedback and telemetry

## Safety Features

- **Velocity Clamping**: Automatic duration extension if motion exceeds velocity limits
- **Fall Detection**: IMU-based detection activates emergency stop
- **Torque Limits**: Servo torque capped to prevent damage
- **Emergency Stop**: User-triggered via `/motion/override` topic

## Firmware

See `sentio/firmware/esp32_servo_bridge/` for ESP32 firmware implementation.

### Building Firmware

cd sentio/firmware/esp32_servo_bridge
platformio run -e esp32doit-devkit-v1

text

### Uploading

platformio run -e esp32doit-devkit-v1 -t upload

text

## Troubleshooting

### No Connection to ESP32

Check serial port
ls /dev/tty*

Check permissions
sudo usermod -a -G dialout $USER

Then log out and back in
Try different port
ros2 run sentio_motion servo_bridge_node --ros-args -p serial_port:=/dev/ttyUSB1

text

### Servos Not Moving in Hardware Mode

1. Check power supply to servos
2. Verify ESP32 firmware is uploaded
3. Test with simulation mode first
4. Check serial port connectivity

### Tests Failing

Ensure dependencies are installed:

pip install pytest pyserial pyyaml

text

## License

Proprietary - DevSora Deep-Tech Research