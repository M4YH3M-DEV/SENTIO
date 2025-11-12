# SENTIO Hardware Documentation

Complete hardware specification, assembly, and integration guide for SENTIO robot.

## Contents

1. **BOM.csv** - Bill of Materials with suppliers and costs
2. **power_management.md** - Power distribution and safety
3. **assembly_guide.md** - Step-by-step assembly instructions
4. **mechanical/** - Mechanical specifications and mounting diagrams
5. **electrical/** - Electrical schematics and safety checklist

## Quick Reference

### Hardware Specifications

| Item | Spec | Reference |
|------|------|-----------|
| Microcontroller | ESP32 DOIT DevKit v1 | Module |
| PWM Driver | PCA9685 16-channel | I2C 0x40 |
| Servos | MG996R (5x) | TowerPro |
| Power Supply | 5V 2A minimum | USB or PSU |
| Logic Voltage | 3.3V (regulated) | AMS1117 |
| Serial | 115200 baud | Debug/Control |
| I2C | 400 kHz, 3.3V logic | Servo Driver |

### Pin Configuration

ESP32 DOIT V1

GPIO21 (I2C SDA) → PCA9685 SDA
GPIO22 (I2C SCL) → PCA9685 SCL
GPIO35 (Input) → E-STOP Button
GPIO 2 (Output) → Status LED
GND → Common Ground
3V3 → Regulated Logic
5V (via reg) → Servo Power

text

### Power Distribution

PSU 5V ──→ [2A Fuse] ──┬──→ PCA9685 VCC
├──→ AMS1117 Input
└──→ Servo Power Rail

PSU GND ──────────────┬──→ PCA9685 GND
├──→ AMS1117 GND
└──→ Servo GND

text

## Assembly Process

### Time Estimate
- PCB soldering: 30-45 minutes
- Servo connections: 15-20 minutes
- Mechanical assembly: 30-45 minutes
- Cable management: 15 minutes
- Testing & verification: 30 minutes
- **Total: 2-3 hours**

### Skill Level
- Soldering: Intermediate (if DIY PCB)
- Mechanical: Beginner
- Electrical: Beginner

### Tools Required
- Soldering iron + solder
- Multimeter
- Wire strippers
- Screwdrivers (M3, flathead)
- Helping hands / soldering stand
- USB cable (for ESP32)

## Build Checklist

- [ ] Parts received and verified against BOM
- [ ] PCB assembled and tested (if DIY)
- [ ] ESP32 powered on
- [ ] I2C communication to PCA9685 working
- [ ] Servos individually tested
- [ ] E-stop button responds
- [ ] Watchdog timer functional
- [ ] Serial communication at 115200 baud
- [ ] HIL tests passing
- [ ] Mechanical assembly complete
- [ ] All cables secured with strain relief
- [ ] System tested with full load

## Integration Checklist

- [ ] ROS 2 serial bridge configured
- [ ] Topic `/behavior_cmd` receives correctly
- [ ] Topic `/tts_text` receives correctly
- [ ] `/motion/status` publishing correctly
- [ ] E-stop controllable via ROS
- [ ] Full system smoke test completed

## Safety Checklist

- [ ] E-stop button installed and accessible
- [ ] Power supply properly fused
- [ ] All connections double-checked for polarity
- [ ] Mechanical parts secured and stress-tested
- [ ] Moving parts guarded
- [ ] Thermal management adequate
- [ ] No exposed high-current traces
- [ ] All GND connections verified

## Verification Steps

### 1. Power On

Check ESP32 LED
Check 5V and 3.3V rails with multimeter
Measure idle current (< 100mA)
text

### 2. Serial Communication

platformio device monitor --port /dev/ttyUSB0

Expected:
{"status":"init",...}
{"status":"ok",...,"servo_count":5}
text

### 3. Servo Testing

python tools/servo_sweep_test.py --port /dev/ttyUSB0 --all

text

### 4. Hardware-in-Loop

python tools/hardware_hil_test.py --port /dev/ttyUSB0 --verbose

text

### 5. System Integration

Start ROS 2 motion bridge
Send behavior commands via /behavior_cmd
Verify servo response
Check motion/status publishing
text

## Troubleshooting

### Power Issues
- Check PSU voltage (4.8-5.2V)
- Measure regulator output (3.3V ±0.1V)
- Check current draw (idle < 150mA)

### Communication Issues
- Verify baud rate (115200)
- Check pull-up resistors on I2C
- Measure I2C voltage (should be 3.3V)

### Servo Issues
- Test individual servos with servo tester
- Verify power connections
- Check signal wire connectivity
- Review servo limits in firmware

See hardware/electrical/safety_checklist.md for comprehensive safety verification.

## Maintenance

### Monthly
- Visual inspection of connections
- Check servo responsiveness
- Verify E-stop functionality
- Monitor thermal conditions

### Quarterly
- Clean connectors with isopropyl alcohol
- Reflow solder joints if needed
- Update firmware if available
- Review error logs

### Annually
- Replace any worn connectors
- Upgrade PSU if needed
- Full system recalibration
- Replace any failed servos

## Performance Metrics

| Measurement | Value | Acceptable Range |
|-------------|-------|-------------------|
| Startup Time | 2.0s | < 5s |
| Idle Current | 80mA | < 150mA |
| Servo Response | 50ms | < 100ms |
| 5V Stability | 5.0V | 4.8-5.2V |
| Servo Sweep | 0.17s | ≤ 0.20s |

## Contact & Support

- Technical Support: dev@devsora.tech
- Documentation: See individual .md files
- Issues: Include serial output and error logs

## License

Proprietary - DevSora Deep-Tech Research