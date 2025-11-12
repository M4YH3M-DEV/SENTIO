# SENTIO Firmware Documentation

Complete firmware implementation for SENTIO robot servo control system.

## Quick Start

### 1. Install PlatformIO

pip install platformio

Verify installation
platformio --version

text

### 2. Build Firmware

cd sentio/firmware/esp32_servo_bridge

Development build
platformio run -e esp32doit-devkit-v1

Production build (optimized)
platformio run -e esp32doit-devkit-v1-prod

text

### 3. Flash ESP32

Connect ESP32 via USB
Press BOOT button, hold, press RESET, release BOOT
platformio run -e esp32doit-devkit-v1 -t upload

text

### 4. Verify

platformio device monitor -e esp32doit-devkit-v1 -b 115200

text

Expected output:
{"status":"init","message":"SENTIO Servo Bridge v1.0.0 initializing"}
{"status":"ok","message":"Servo bridge ready","servo_count":5}

text

## Project Structure

esp32_servo_bridge/
├── platformio.ini # Build configuration
├── src/
│ └── main.cpp # Main firmware
└── lib/
└── PCA9685.h # PWM driver library

text

## Build Configurations

### Development (`esp32doit-devkit-v1`)

- Full debug symbols
- Debugging enabled
- OTA support enabled
- All serial messages enabled
- Slow optimization (faster debug)

Build command:
platformio run -e esp32doit-devkit-v1

text

Output binary: `.pio/build/esp32doit-devkit-v1/firmware.bin` (~400KB)

### Production (`esp32doit-devkit-v1-prod`)

- Optimized for speed and size
- Debug symbols stripped
- OTA support enabled
- Serial messages reduced
- Full optimization (-O3)

Build command:
platformio run -e esp32doit-devkit-v1-prod

text

Output binary: `.pio/build/esp32doit-devkit-v1-prod/firmware.bin` (~300KB)

## Flash Procedure

### Method 1: Automatic (Recommended)

cd sentio/firmware/esp32_servo_bridge
platformio run -e esp32doit-devkit-v1 -t upload

text

PlatformIO will auto-detect port and flash.

### Method 2: Manual

Find serial port
platformio device list

Flash specific port
platformio run -e esp32doit-devkit-v1 -t upload --upload-port=/dev/ttyUSB0

text

### Method 3: Manual with esptool

Download esptool
pip install esptool

Erase and flash
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash
0x1000 .pio/build/esp32doit-devkit-v1/firmware.bin

text

## Serial Interface

### Connection

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### Monitor

Using PlatformIO
platformio device monitor -e esp32doit-devkit-v1

Using screen
screen /dev/ttyUSB0 115200

Using minicom
minicom -D /dev/ttyUSB0 -b 115200

text

## Compilation Notes

### Source Files

- `src/main.cpp`: ~800 lines
  - Initialization: ~100 lines
  - Main loop: ~50 lines
  - Serial handling: ~150 lines
  - Servo control: ~200 lines
  - System management: ~300 lines

### Dependencies

- `ArduinoJson` (6.20.0)
  - For JSON serialization/deserialization
  - ~50KB binary size impact

- `Adafruit PCA9685` (1.4.1)
  - For PWM driver control
  - ~10KB binary size impact

- Arduino Core for ESP32 (built-in)

### Memory Usage

ESP32 Memory Map:
Total Flash: 4 MB
Program: ~350 KB (dev), ~280 KB (prod)
Filesystem: ~2 MB (for OTA)

Total RAM: 520 KB
Used: ~100 KB (runtime)
Free: ~420 KB

text

## Testing

### Unit Tests (Firmware)

No unit tests in firmware (embedded environment). Use HIL tests instead.

### Hardware-in-the-Loop (HIL) Tests

From repository root
python tools/hardware_hil_test.py --port /dev/ttyUSB0 --verbose

text

Tests:
- Serial communication
- PCA9685 detection
- Servo movement
- E-stop functionality
- Response time

### Manual Testing

Connect serial monitor
platformio device monitor

Test commands:
{"type":"status"}\n
{"type":"servo_move","positions":{"0":90}}\n
{"type":"servo_move","positions":{"0":45,"1":135}}\n
{"type":"estop"}\n
{"type":"clear_estop"}\n

text

## Debugging

### Enable Debug Output

Add to `platformio.ini`:
build_flags = -DCORE_DEBUG_LEVEL=5 # Max verbosity

text

### Connect Debugger (Optional)

With FTDI debugger
platformio debug -e esp32doit-devkit-v1

text

### Monitor Serial Output with Timestamps

platformio device monitor --filter=time --filter=colorize

text

## OTA Updates (Future)

Framework for OTA updates is included but not fully configured.

To enable WiFi OTA:

1. Configure WiFi credentials in code
2. Uncomment OTA functions in `main.cpp`
3. Rebuild and flash
4. Upload new firmware over network

platformio run -e esp32doit-devkit-v1 -t upload --upload-port=192.168.1.100

text

## Troubleshooting

### Build Errors

**Error**: "esp32/Arduino.h not found"
Solution: Update platform
platformio platform update espressif32

text

**Error**: "No module named 'platformio'"
Solution: Install PlatformIO
pip install platformio

text

### Upload Errors

**Error**: "Could not connect to ttyUSB0"
Solution 1: Check permissions
sudo usermod -a -G dialout $USER

(Logout and login required)
Solution 2: Try different port
platformio device list
platformio run -t upload --upload-port=/dev/ttyUSB1

text

**Error**: "Invalid header" during upload
Solution: Device not in boot mode
Hold BOOT button, press RESET, release BOOT
Try upload again
text

### Runtime Errors

**No serial output**:
1. Check USB cable connection
2. Verify CH340 driver (Windows/Mac)
3. Try different USB port
4. Erase and reflash

**Servo won't move**:
1. Check 5V power supply
2. Verify PCA9685 detected (`type:status` command)
3. Test with servo tester
4. Check signal cable connection

**I2C communication fails**:
1. Verify pull-up resistors (10kΩ on SDA/SCL)
2. Check I2C address (should be 0x40)
3. Reduce I2C frequency in code
4. Verify wire connections

## Performance Specifications

| Metric | Value |
|--------|-------|
| Startup Time | 2 seconds |
| Serial Response | < 10 ms |
| Command Processing | < 5 ms |
| Max Servo Speed | 90°/0.17s (MG996R) |
| Watchdog Timeout | 5 seconds |
| Uptime | > 72 hours |
| Temperature Range | 0-85°C |

## Version History

### v1.0.0 (2025-11-13)
- Initial release
- PCA9685 support
- 5 servo channels
- JSON serial protocol
- Watchdog timer
- E-stop functionality
- Telemetry reporting

## Support

For issues or questions:

- Check this README
- Review firmware/src/main.cpp comments
- Run HIL tests: `python tools/hardware_hil_test.py --port /dev/ttyUSB0`
- Contact: dev@devsora.tech

## License

Proprietary - DevSora Deep-Tech Research