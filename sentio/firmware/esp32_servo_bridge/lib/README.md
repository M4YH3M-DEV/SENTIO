# SENTIO ESP32 Servo Bridge Firmware

Production-grade firmware for controlling servo motors via PCA9685 PWM driver over JSON-based serial protocol.

## Features

- **JSON-over-Serial Protocol**: Simple, human-readable command interface
- **PCA9685 Driver Support**: Control up to 16 PWM channels (5 servos configured)
- **Watchdog Timer**: Auto-shutdown on command timeout
- **Emergency Stop**: Hardware E-stop button with debouncing
- **Telemetry**: Complete status reporting and diagnostics
- **Safe Defaults**: Neutral servo positions on startup and emergency conditions
- **OTA Ready**: Framework for wireless firmware updates

## Building

### Prerequisites

- PlatformIO Core (`pip install platformio`)
- ESP32 dev board (tested: ESP32 DOIT DevKit v1)
- USB-to-serial cable
- ~5 minutes

### Build

cd sentio/firmware/esp32_servo_bridge

Build for ESP32 DOIT DevKit v1
platformio run -e esp32doit-devkit-v1

Build for production (optimized)
platformio run -e esp32doit-devkit-v1-prod

Expected output:
Compiling .pio/build/esp32doit-devkit-v1/...
Linking .pio/build/esp32doit-devkit-v1/firmware.elf
Building .pio/build/esp32doit-devkit-v1/firmware.bin
✓ Build completed successfully
text

## Flashing

### 1. Connect Hardware

ESP32 USB Serial Adapter
GND ----- GND
TX (GPIO1) ----- RX
RX (GPIO3) ----- TX
5V ----- 5V (or 3.3V)

text

Press and hold BOOT button, then press RESET button to enter flash mode.

### 2. Flash Firmware

Auto-detect and flash
platformio run -e esp32doit-devkit-v1 -t upload

Expected output:
Looking for upload port...
Serial port /dev/ttyUSB0
Uploading .pio/build/esp32doit-devkit-v1/firmware.bin
...
Wrote 323632 bytes to file offset 0x1000 in 8.24 seconds (312 kbit/s)
Hash of data verified.
Hard resetting via RTS pin...
text

### 3. Verify Serial Connection

platformio device monitor -e esp32doit-devkit-v1

Expected output:
{"status":"init","message":"SENTIO Servo Bridge v1.0.0 initializing"}
{"status":"ok","message":"Servo bridge ready","servo_count":5}
text

Press Ctrl+C to exit.

## Serial Protocol

### Command Format

All commands are JSON strings terminated with newline (`\n`):

{"type":"servo_move","positions":{"0":90,"1":45}}\n

text

### Response Format

{"status":"ok","msg":"Servo move executed","cmd_count":1,"error_count":0}

text

### Commands

#### Servo Move

Move servo(s) to specified angle (0-180 degrees):

{
"type": "servo_move",
"positions": {
"0": 90,
"1": 45,
"2": 135
}
}

text

Response:
{"status":"ok","servo":0,"degrees":90,"pwm":308}

text

#### LED Control

Control RGB LED strip (placeholder):

{
"type": "led_control",
"led": {
"color": "blue",
"pattern": "pulse",
"intensity": 0.8
}
}

text

#### Emergency Stop

Activate emergency stop (moves all servos to neutral):

{"type":"estop"}

text

Response:
{"status":"ok","msg":"Emergency stop activated"}

text

#### Clear Emergency Stop

Clear emergency stop condition:

{"type":"clear_estop"}

text

#### Status Report

Request full system status:

{"type":"status"}

text

Response:
{
"status": "ok",
"servo_count": 5,
"command_count": 42,
"error_count": 0,
"emergency_stop": false,
"pca9685_online": true,
"uptime_ms": 123456,
"servos": [
{"id": 0, "position_us": 1500, "degrees": 90},
{"id": 1, "position_us": 1000, "degrees": 0},
{"id": 2, "position_us": 2000, "degrees": 180}
]
}

text

## Testing

### Manual Serial Test

Open serial monitor
platformio device monitor

Send commands:
{"type":"servo_move","positions":{"0":45}}\n
{"type":"servo_move","positions":{"0":135}}\n
{"type":"status"}\n

text

### Automated Test Script

python ../../tools/servo_sweep_test.py --port /dev/ttyUSB0

text

Expected behavior: Servo 0 sweeps from 0° to 180° and back.

### Hardware-in-Loop Test

python ../../tools/hardware_hil_test.py --port /dev/ttyUSB0 --verbose

text

Tests:
- Serial communication
- PCA9685 initialization
- All 5 servos
- E-stop functionality
- Watchdog timer
- Error recovery

## Pin Configuration

### ESP32 Pinout

GPIO 21 - I2C SDA
GPIO 22 - I2C SCL
GPIO 35 - E-STOP (input, active low)
GPIO 2 - Status LED

text

### PCA9685 Channels

Channel 0 - Servo 0 (Neck Pan)
Channel 1 - Servo 1 (Neck Tilt)
Channel 2 - Servo 2 (Torso Rotate)
Channel 3 - Servo 3 (Left Shoulder)
Channel 4 - Servo 4 (Right Shoulder)

text

### I2C Address

PCA9685 Address: 0x40 (default)
I2C Frequency: 400 kHz

text

## Specifications

### Servo Control

- **Frequency**: 50 Hz (standard servo)
- **Min Pulse Width**: 1000 µs (0°)
- **Max Pulse Width**: 2000 µs (180°)
- **Resolution**: 12-bit (4096 levels)
- **Command Timeout**: 5 seconds (triggers watchdog)

### Safety

- **Watchdog**: Auto-safe-shutdown after 5 seconds no command
- **E-stop**: Hardware button with 50 ms debounce
- **Neutral Position**: 1500 µs (90°) on startup and emergency
- **Emergency Action**: All servos to neutral, command rejected

### Performance

- **Baud Rate**: 115200
- **Command Processing**: <10 ms
- **Response Time**: <5 ms
- **Memory Usage**: ~45 KB / 320 KB available

## Troubleshooting

### No Serial Output

Check USB cable is connected

Verify CH340 driver installed (Windows/Mac)

Confirm baud rate is 115200

Try: platformio device list

text

### PCA9685 Not Found

Verify I2C connections (SDA=GPIO21, SCL=GPIO22)

Check 0.1µF capacitors on I2C lines

Scan I2C: i2cdetect -y 1 (from RPi)

Address should appear as 0x40

text

### Servos Not Moving

Check power supply voltage (5V ± 0.5V)

Verify servo connectors are secure

Test servo independently with servo tester

Send status command to check positions

text

### Watchdog Timeout

Ensure commands sent every 5 seconds

Check serial connection stability

Monitor for serial errors or dropped frames

Reduce network latency if over IP

text

## OTA Updates

OTA (Over-The-Air) firmware updates support is compiled in but requires WiFi configuration:

// Future: WiFi OTA setup
// ArduinoOTA.setHostname("sentio-servo-bridge");
// ArduinoOTA.begin();

text

## Power Management

### Recommended PSU

- **Voltage**: 5V DC
- **Current**: 2A minimum (5 servos at full load)
- **Quality**: Good filtering recommended

### Power Distribution

PSU 5V --┬--> PCA9685 VCC
├--> ESP32 5V (through AMS1117 3.3V reg)
└--> Servo Power Rail

GND ----┬--> PCA9685 GND
├--> ESP32 GND
└--> Servo GND

text

## Maintenance

### Regular Checks

- Serial communication stable
- No persistent errors
- Servo response times consistent
- Temperature within 0-85°C

### Log Analysis

Monitor error rate
grep "error" /tmp/servo_bridge.log | wc -l

Check temperature trends
grep "temperature" /tmp/servo_bridge.log | tail -20

text

## Support & Contact

For issues, technical questions, or firmware modifications:

- Email: dev@devsora.tech
- Documentation: See firmware/README.md
- Issues: Report with serial output and command sequence

## License

Proprietary - DevSora Deep-Tech Research