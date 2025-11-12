# SENTIO Hardware Assembly Guide

## Overview

This guide covers complete assembly of SENTIO robot hardware from PCB level to mechanical integration.

**Estimated Time**: 2-3 hours
**Skill Level**: Intermediate (soldering required)
**Tools Required**: Soldering iron, multimeter, screwdrivers, wire strippers

## Phase 1: Power Distribution Board (if DIY)

### 1.1 Prepare PCB

- [ ] Inspect PCB for defects
- [ ] Clean with isopropyl alcohol
- [ ] Dry completely
- [ ] Use solder mask where available

### 1.2 Solder Components

**Order**: Lowest profile first to highest

1. **Resistors** (10kΩ pull-ups)
   - Clean holes
   - Insert resistor
   - Solder both sides
   - Trim leads

2. **Capacitors** (100nF, 470µF)
   - Polarity: Check markings
   - Solder carefully
   - Electrolytic polarity: + to regulated side

3. **AMS1117 Regulator**
   - Align tab with silkscreen
   - Tack one pin first
   - Align, solder other pins
   - Solder back tab for thermal contact

4. **Headers**
   - Start with one pin
   - Align square, solder all pins
   - Use helping hands if needed

### 1.3 Verify

Multimeter test points:

5V Rail to GND: Should read 5.0V

3.3V Rail to GND: Should read 3.3V

No shorts between rails

text

## Phase 2: ESP32 & PCA9685 Assembly

### 2.1 ESP32 Dev Board

**As-purchased**: Already assembled

**Modifications** (optional):
- [ ] Add pull-up resistors if not included
- [ ] Solder pin headers if not installed
- [ ] Add decoupling capacitors (0.1µF on VCC)

### 2.2 PCA9685 Module

**As-purchased**: Usually pre-assembled

**Verification**:
Power module with 5V only (no I2C connections)

Check for LED indicators (may have status LED)

Verify I2C address: Measure resistor ladder on A0-A5

text

### 2.3 I2C Connections

ESP32 (3.3V logic) PCA9685 (5V supply)
│ │
GPIO21 (SDA) ──[10kΩ]────┬─→ SDA (with pull-up to 3.3V)
GPIO22 (SCL) ──[10kΩ]────┬─→ SCL (with pull-up to 3.3V)
GND ─────────────────────┴─→ GND (common)

text

**Cable**: 3-conductor 22AWG wire, soldered or DuPont connectors

## Phase 3: Servo Motor Integration

### 3.1 Servo Connection

**Each Servo**:
Signal (orange/yellow) → PCA9685 CHx Output
Power (red) ────────→ +5V Rail
Ground (brown/black) → GND Rail

text

**Mapping**:
PCA9685 CH0 → Servo 0 (Neck Pan)
PCA9685 CH1 → Servo 1 (Neck Tilt)
PCA9685 CH2 → Servo 2 (Torso)
PCA9685 CH3 → Servo 3 (Left Shoulder)
PCA9685 CH4 → Servo 4 (Right Shoulder)

text

### 3.2 Cable Routing

- **Signal**: Individual 22AWG wires, group loosely
- **Power**: Use twisted pair (red/black) for current capacity
- **Length**: Keep < 1 meter if possible
- **Shielding**: Not required for this voltage/frequency

### 3.3 Servo Verification

Test each servo individually
platformio device monitor

Send: {"type":"servo_move","positions":{"0":90}}
Expected: Servo 0 moves to neutral position
text

## Phase 4: Mechanical Assembly

### 4.1 Mounting Frame

**Materials**:
- Aluminum extrusion (20x20mm profile) OR
- 3D-printed plastic brackets
- M3 mounting bolts and nuts

**Assembly**:
1. Cut extrusion to length (see specifications)
2. Drill mounting holes (Ø3.2mm for M3)
3. Tap threads or use nuts
4. Assemble frame skeleton

### 4.2 Servo Mounting

**Horizontal Servo (Neck Pan - Servo 0)**:
┌─────────────────┐
│ Servo Motor │
│ ┌─────────────┤ ← Mount with servo horn facing up
│ │ Horn │
│ │ ┌──────────┐ ← Mechanical linkage
│ │ │ Link │
│ │ │ to Head │
└──┴──┴──────────┘

text

Mounting holes: 2× M3 on servo bottom

**Vertical Servo (Neck Tilt - Servo 1)**:
text
 ▲ Motor axis
 │
┌───┴───┐
│Servo │ ← Mount to frame side
│ │
└───┬───┘
│
→ To mechanical linkage

text

### 4.3 Mechanical Linkage

**Materials**:
- Aluminum rods (Ø6mm)
- Ball joints (M3 threaded)
- Bearing blocks if needed

**Design Principles**:
- Keep linkages short (< 50mm) for stiffness
- Use ball joints for universal motion
- Provide mechanical advantage where needed

**Assembly**:
Insert rod into servo horn hole

Secure with set screw (M2x3)

Connect other end to moving member

Use lock-nut for security

No gaps, minimal play in joints

text

### 4.4 LED Integration

**Options**:

1. **RGB LED Strip** (WS2812B)
   - Data pin: GPIO33 (or available GPIO)
   - Power: +5V
   - Ground: GND
   - Location: Mounted on head front, facing outward

2. **Discrete LEDs**
   - Use PCA9685 PWM channels or GPIO
   - Current limiting resistor: 220Ω for 5V
   - Mount: LED holder on servo bracket

### 4.5 IMU Mounting

**MPU6050 or similar**:
- Mount on main chassis
- Oriented: Z-axis vertical
- Secure with double-sided tape or velcro
- I2C connection: Parallel with PCA9685

## Phase 5: Cable Management

### 5.1 Cable Organization

Power Rail (Red 22AWG) ──┬─→ Servo 0 Power
├─→ Servo 1 Power
├─→ Servo 2 Power
├─→ Servo 3 Power
└─→ Servo 4 Power

GND Rail (Black 22AWG) ──┬─→ Servo 0 GND
├─→ Servo 1 GND
├─→ Servo 2 GND
├─→ Servo 3 GND
└─→ Servo 4 GND

Signal Wires (22AWG)────→ Individual to PCA9685 channels

text

### 5.2 Strain Relief

- Secure bundles with zip ties (≤ 50mm intervals)
- Avoid sharp bends (min 25mm radius)
- Protect from moving parts with tape/tubing
- Label each cable at both ends

### 5.3 Connector Selection

**Recommended**:
- Servo connectors: JR standard (already on servos)
- Power: Barrel jack or Anderson PowerPole
- I2C: DuPont 2.54mm headers (for modularity)

## Phase 6: E-Stop Wiring

### 6.1 Button Installation

**Hardware**:
- 16mm momentary push button (NO contact)
- IP67 rated recommended
- Red button for visibility

**Installation**:
Mounting hole: Ø16mm on front panel
Button terminals: Two-pin connector

Circuit:
PSU +5V ──┐
├─→ Button ──→ [10kΩ pull-down] ──→ GND
│
GPIO35 (ESP32) ← Connected to button input

text

### 6.2 Firmware Integration

#define ESTOP_PIN 35
// Button pressed (LOW) → Emergency stop
// Button released (HIGH) → Normal operation

text

### 6.3 Testing

Test E-stop functionality
Press button

Check firmware detects event

Verify all servos move to neutral

Release button

Resume normal operation

text

## Phase 7: Environmental Sealing

### 7.1 Enclosure (if outdoor use)

- Material: ABS or polycarbonate
- Rating: IP54 minimum
- Mounting: DIN rail or wall mount

### 7.2 Cable Glands

- Use M16 or M20 cable glands
- Seal with silicone or RTV
- Strain relief inside enclosure

### 7.3 Thermal Management

- Ensure ventilation for AMS1117 regulator
- Add small heatsink if dissipating > 1W
- Monitor temperature with ADC

## Phase 8: Final Verification

### 8.1 Power Test

Without servos connected
Apply 5V power

Check ESP32 LED (should power on)

Measure 3.3V rail (should be ~3.3V)

Check for shorts (all currents reasonable)

text

### 8.2 Serial Communication

platformio device monitor --baud 115200

Expected output:
{"status":"init","message":"SENTIO Servo Bridge v1.0.0 initializing"}
{"status":"ok","message":"Servo bridge ready","servo_count":5}

text

### 8.3 Servo Movement

Send test command (one servo at a time)
{"type":"servo_move","positions":{"0":0}}\n
{"type":"servo_move","positions":{"0":90}}\n
{"type":"servo_move","positions":{"0":180}}\n

Verify:
Servo moves smoothly

Response received for each command

Servo reaches target angle

text

### 8.4 All Servos

Send comprehensive test
{"type":"servo_move","positions":{"0":90,"1":90,"2":90,"3":90,"4":90}}\n

Check:
All servos respond

No current spikes (5V rail stable)

Response time < 100ms

text

### 8.5 E-Stop Test

Press E-stop button
Verify:
All servos move to neutral (1500µs)

Serial output shows error

Button press is detected

Release button
Send new command - should be ignored until reset
(Or implement clear_estop command)
text

## Troubleshooting

### Servo Won't Move
- [ ] Check power (5V present)
- [ ] Verify signal wire connected
- [ ] Test with servo tester
- [ ] Check PCA9685 initialization

### ESP32 Won't Boot
- [ ] Check 3.3V regulator output
- [ ] Verify power supply voltage (4.8-5.2V)
- [ ] Try boot button + reset
- [ ] Check for shorts

### I2C Communication Fails
- [ ] Verify pull-up resistors present
- [ ] Measure I2C voltage (should be 3.3V)
- [ ] Use I2C scanner tool
- [ ] Check address (should be 0x40)

### Voltage Sags During Servo Movement
- [ ] Upgrade power supply (2A → 3A+)
- [ ] Reduce servo speed
- [ ] Add more capacitance
- [ ] Check power distribution resistance

## Safety Checklist

- [ ] E-stop button installed and tested
- [ ] Power supply properly rated (2A+)
- [ ] All GND connections verified
- [ ] No exposed high-current traces
- [ ] Thermal management adequate
- [ ] Cable strain relief installed
- [ ] Mechanical joints secured with lock nuts
- [ ] Servo torque rated for load
- [ ] Watchdog timeout active in firmware

## Testing Checklist

- [ ] Serial communication: 115200 baud
- [ ] PCA9685 detected on I2C
- [ ] All 5 servos individually tested
- [ ] E-stop button triggers safe shutdown
- [ ] Watchdog timeout works
- [ ] Power draw measured and acceptable
- [ ] Servo sweep test passes (see tools/servo_sweep_test.py)
- [ ] HIL test passes (see tools/hardware_hil_test.py)

## Expected Performance

| Parameter | Value |
|-----------|-------|
| Startup time | < 2 seconds |
| Serial response | < 10 ms |
| Servo movement | 0-180° in 0.25s (fastest) |
| Power stability | ±0.25V at 5V rail |
| Temperature | < 50°C under load |
| Uptime | > 72 hours continuous |

## Next Steps

1. Mount servo-controlled components (head, arms)
2. Integrate cameras and sensors
3. Connect to ROS via serial bridge
4. Run full system integration test
5. Deploy to production

See ../README.md for integration with ROS ecosystem.