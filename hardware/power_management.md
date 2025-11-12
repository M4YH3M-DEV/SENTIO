# SENTIO Power Management & Distribution Guide

## Overview

SENTIO operates on a single 5V power rail serving:
- ESP32 microcontroller (regulated to 3.3V)
- PCA9685 PWM driver
- Servo motors (up to 5x MG996R)

## Power Budget

### Typical Operating Conditions

| Component | Idle (mA) | Active (mA) | Peak (mA) |
|-----------|-----------|------------|----------|
| ESP32 | 30 | 80 | 160 |
| PCA9685 | 5 | 20 | 50 |
| Servo (x1) | 10 | 200 | 500 |
| Servo (x5 total) | 50 | 1000 | 2500 |
| **Total** | **85** | **1100** | **2710** |

### Recommended PSU

- **Continuous Rating**: 2.0A @ 5V minimum
- **Peak Rating**: 3.0A @ 5V recommended
- **Quality**: Regulated with good filtering
- **Connector**: Micro-USB or barrel jack (5.5mm OD, 2.1mm ID)

**Recommended Models**:
- Anker PowerPort+ (5V/2.4A)
- Belkin USB-C PD (5V/2A)
- Mean Well RSP-75 (75W, 5V/15A - for permanent installations)

## Wiring Schematic

┌─────────────────────────────────────┐
│ 5V Power Supply (2A) │
│ Input: AC 100-240V │
│ Output: 5V DC │
└─────────────────────────────────────┘
│
▼▼
┌─────┴─┬───────┐
│ +5V │ GND │
│ │ │
├───────┼───────┤
│ │ │
▼ ▼ ▼
┌──────────────────────────────┐
│ Power Distribution Board │
│ (or breadboard with rails) │
│ │
│ +5V Rail │
│ │ │
│ ├─── PCA9685 (VCC) │
│ ├─── AMS1117 Input │
│ └─── Servo Power Rail │
│ │
│ GND Rail │
│ │ │
│ ├─── PCA9685 (GND) │
│ ├─── AMS1117 Ground │
│ └─── Servo Ground Rail │
└──────────────────────────────┘
│
┌──────┴──────┬─────────────────┐
│ │ │
▼ ▼ ▼
┌─────────┐ ┌─────────┐ ┌─────────────┐
│ ESP32 │ │ PCA9685 │ │ Servo Rail │
│(via 3.3V│ │ │ │ (5V Direct) │
│ Reg) │ │ PWM │ │ │
│ │ │ Driver │ │ Servo x5 │
└─────────┘ └─────────┘ └─────────────┘

text

## Component Details

### 5V Power Supply Selection

**USB Power Adapter (2A)**
- Pros: Readily available, compact, safe
- Cons: Limited current for simultaneous servo movement
- Use case: Testing and development

**Bench Power Supply (3-5A)**
- Pros: Adjustable, current monitoring, overcurrent protection
- Cons: Expensive, requires AC power
- Use case: Lab testing

**Industrial PSU (Mean Well, 15A)**
- Pros: High reliability, integrated protection, long lifespan
- Cons: Larger, expensive
- Use case: Production systems

### Linear Regulator (AMS1117-3.3V)

**Specs**:
- Input: 5.5V to 15V (use 5V nominal)
- Output: 3.3V regulated
- Current: 1A max
- Package: TO-220

**Connections**:
5V Rail ──┐
├─► AMS1117 VIN
│
=== (100nF input cap)
│
┌─────────┘
│
├─► AMS1117 ADJ (or GND if fixed)
│
├─► AMS1117 VOUT ──┬─► ESP32 3V3
└─► (470µF cap)
└─► GND

text

**Load**: ESP32 ~50mA typical, well within 1A limit.

### PCA9685 PWM Driver

**Specifications**:
- I2C Address: 0x40 (default, configurable via A5-A0 pins)
- Logic Voltage: 3.3V (via ESP32)
- Servo Voltage: 5V (independent power rail)
- Frequency: 50 Hz (configurable)
- Output Current: ~25mA per channel max (100mA total)

**I2C Connections**:
ESP32 GPIO21 (SDA) ──┐
├──┬─► PCA9685 SDA (with pull-up to 3.3V)
ESP32 GPIO22 (SCL) ──┤ │
└──┴─► PCA9685 SCL (with pull-up to 3.3V)

text

**Pull-up Resistors**:
- 10kΩ SDA to 3.3V
- 10kΩ SCL to 3.3V

### Servo Motor Power Rail

**Connection**:
5V Rail ─────────┬─► Servo 0 Power
├─► Servo 1 Power
├─► Servo 2 Power
├─► Servo 3 Power
└─► Servo 4 Power

GND Rail ────────┬─► Servo 0 GND
├─► Servo 1 GND
├─► Servo 2 GND
├─► Servo 3 GND
└─► Servo 4 GND

text

**High-Current Routing**:
- Use 18-20 AWG wire for power distribution
- Keep power and ground separate (no sharing with logic)
- Add 470µF capacitor at servo power rail (near motors)

### Capacitor Placement

**Purpose**: Suppress voltage transients when servos start/stop.

**Placement**:
5V PSU ──┬─► [100nF] ──┬─► ESP32
│ │
├─► [100nF] ──┬─► PCA9685
│ │
└─► [470µF] ──┬─► Servo Rail (close to motors)
│
GND

text

## Ground Referencing

**Critical**: All grounds must be common (star-grounded).

PSU GND ──────────┬─────────┬─────────┬─────────┐
│ │ │ │
ESP32 GND PCA9685 GND Servo GND │
│ │ │ │
└─────────┴─────────┴─────────┘
(Common GND Rail)

text

**Key Rules**:
1. Never separate GND into different paths
2. Use single thick wire from PSU GND to distribution point
3. All components reference same GND
4. Ground plane recommended for PCB designs

## External Power Distribution Board Design

For production, a dedicated board is recommended:

### Components
- Barrel jack or USB connector
- Dual 470µF capacitors (input filtering)
- AMS1117 regulator
- Power rail distribution headers
- Fuse holder (2A)

### Layout
text
    USB/Barrel
     │
    ▼▼
   [Fuse 2A]
     │
    ▼▼
┌────────────┐
│ Capacitor  │ (470µF × 2, parallel)
│ Bank       │
└────────────┘
     │
    ▼▼
┌────────────────┐
│ AMS1117 + Caps │ (3.3V output)
└────────────────┘
     │
┌────┴────────────┐
│                 │
▼                 ▼
+5V Rail +3.3V Rail
├─ Pin Header ├─ Pin Header
│ (power) │ (3.3V)
│
GND Rail
├─ Pin Header (GND)

text

### PCB Considerations
- 4-layer board recommended (power/GND planes)
- Track width: 1mm+ for 5V distribution
- Via count: Multiple vias between layers
- Ground plane: Continuous GND on Layer 2
- Power plane: Continuous 5V on Layer 3

## Safety Considerations

### Overcurrent Protection
- Install 2A fuse on PSU output
- PCA9685 has internal current limiting (but check with datasheet)
- Monitor ESP32 ADC for PSU voltage sag

### Thermal Management
- AMS1117: Can dissipate ~1W (monitor temperature)
- Add small heatsink if regulation current > 500mA sustained
- Ensure adequate ventilation in enclosure

### ESD Protection
- Use ferrite bead on I2C lines (optional but recommended)
- Keep ESP32 in ESD-safe bag until final assembly

### High-Inrush Current
- When all 5 servos start simultaneously, PSU must handle 2.5A peak
- Use PSU with soft-start or current limiting
- Consider inrush current: t_start = L * I_peak / V_supply

## Testing Procedure

### 1. No-Load Test
Connect PSU, multimeter across 5V rail
Measure voltage: Should be 5.0V ± 0.25V
Measure current: Should be <100mA (ESP32 + PCA9685 only)
text

### 2. Single Servo Test
Connect one servo to Servo 0
Send move command: {"type":"servo_move","positions":{"0":90}}
Measure current: Should peak to ~300mA, settle to ~50mA
Measure 5V voltage: Should not drop below 4.8V
text

### 3. All Servos Idle
Connect all 5 servos
Check idle current: ~150-200mA
Check 5V rail voltage: 4.95V or higher
text

### 4. All Servos Active
Send command: {"type":"servo_move","positions":{"0":0,"1":0,"2":0,"3":0,"4":0}}
Measure peak current: Should be 2.0-2.5A
Voltage should not drop below 4.8V
Current should settle within 5 seconds
text

## Troubleshooting

### Low Voltage at ESP32
**Symptom**: Resets during servo movement
**Cause**: Voltage sag due to inadequate PSU current
**Fix**: Upgrade to 3A+ PSU, reduce servo load, increase capacitance

### Heat on AMS1117
**Symptom**: Regulator gets hot (>60°C)
**Cause**: High continuous current or low input voltage
**Fix**: Reduce load, add heatsink, check PSU output

### Servo Jitter
**Symptom**: Servos vibrate during movement
**Cause**: Noise on 5V rail
**Fix**: Add ferrite bead to servo power line, increase capacitance

### I2C Communication Failures
**Symptom**: "PCA9685 not found" error
**Cause**: Pull-up resistors missing or weak
**Fix**: Add 10kΩ pull-ups to 3.3V on SDA/SCL

## Production Recommendations

1. **Use dedicated PSU**: Not USB bus power (improper filtering)
2. **Implement monitoring**: ADC voltage measurement in firmware
3. **Add telemetry**: Current sense for diagnostics
4. **Thermal control**: Monitor regulator temperature
5. **Redundancy**: Consider dual PSU for fail-over

## References

- AMS1117 Datasheet: [Manufacturer]
- PCA9685 Datasheet: [NXP]
- MG996R Servo Specs: [TowerPro]
- IEC 60938-1: Power supply safety standards