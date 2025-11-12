# SENTIO Hardware Safety Checklist

## Pre-Assembly Safety Review

- [ ] All components reviewed for compatibility
- [ ] BOM parts verified with correct specifications
- [ ] Power supply rated for peak current (2.5A minimum)
- [ ] No counterfeit or damaged components in BOM
- [ ] Relevant datasheets available for reference

## Assembly Safety

- [ ] Work area clean and dry
- [ ] Soldering iron properly grounded
- [ ] Solder fume extraction active
- [ ] No beverages near electronics
- [ ] Proper PPE (safety glasses, no loose clothing)
- [ ] ESD mat used during assembly
- [ ] Components handled with care

## Electrical Safety

- [ ] All power connections use appropriate gauge wire
- [ ] Fuse installed on PSU positive output (2A)
- [ ] No reversed polarity on PSU
- [ ] All GND connections common and secure
- [ ] I2C pull-up resistors present
- [ ] No exposed high-voltage traces
- [ ] Capacitors properly rated for voltage

## Mechanical Safety

- [ ] All moving parts guarded or enclosed
- [ ] E-stop button accessible and clearly marked
- [ ] Servo torque verified for load application
- [ ] Mounting bolts secured with lock-nuts
- [ ] Cable strain relief prevents sharp bends
- [ ] No mechanical interference with electrical
- [ ] All mechanical joints stress-tested

## Thermal Safety

- [ ] AMS1117 regulator has adequate cooling
- [ ] Temperature sensor monitored via firmware
- [ ] Enclosure allows airflow
- [ ] High-current traces sized appropriately
- [ ] No components near hot surfaces

## Emergency Stop System

- [ ] E-stop button wired to GPIO35
- [ ] Button rated for switching servo current
- [ ] E-stop triggers all servos to neutral
- [ ] E-stop cannot be disabled without reset
- [ ] Button LED indicator for status
- [ ] E-stop wiring isolated from signal lines

## Firmware Safety Features

- [ ] Watchdog timer active (5-second timeout)
- [ ] Command timeout triggers safe shutdown
- [ ] Servo commands validated (0-180° range)
- [ ] Current limiting in firmware (if available)
- [ ] Serial buffer overflow protection
- [ ] CRC or checksum for critical commands

## Testing Procedures Completed

- [ ] No-load power test (< 100mA idle)
- [ ] Single servo load test (< 500mA per servo)
- [ ] All servos load test (< 2.5A peak)
- [ ] Watchdog timer verification
- [ ] E-stop functionality test
- [ ] Power supply regulation test
- [ ] Temperature monitoring test

## Environmental Conditions

- [ ] Operating temperature: 0-50°C verified
- [ ] Humidity: < 85% non-condensing
- [ ] EMI shielding adequate (if needed)
- [ ] Grounding to building earth (if AC powered)
- [ ] Lightning protection (if outdoor)

## Documentation

- [ ] Schematics labeled and dated
- [ ] Component values verified on PCB vs schematic
- [ ] Firmware version recorded
- [ ] Assembly photos documented
- [ ] Modifications noted
- [ ] Maintenance log initiated

## Operator Safety

- [ ] Safety manual available
- [ ] Operator training completed
- [ ] Emergency procedures posted
- [ ] E-stop button location marked
- [ ] Power disconnect location known
- [ ] Troubleshooting guide available

## Long-Term Maintenance

- [ ] Inspection schedule established (monthly)
- [ ] Thermal imaging performed at startup
- [ ] Power measurements logged
- [ ] Cable condition monitored
- [ ] Servo response times tracked
- [ ] Error logs reviewed weekly

## Sign-Off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Electrical Engineer | ___________ | ______ | ___________ |
| Mechanical Engineer | ___________ | ______ | ___________ |
| Safety Officer | ___________ | ______ | ___________ |
| QA / Testing | ___________ | ______ | ___________ |

---

## Critical Hazards & Mitigations

### Hazard: High-Current Surge During Servo Start
**Consequence**: Voltage sag, ESP32 reset, system instability
**Mitigation**: 
- Use 2.5A+ PSU with good current capacity
- Add 470µF capacitor bank
- Implement firmware servo current limiting
- Monitor voltage in firmware

### Hazard: E-Stop Button Failure
**Consequence**: Inability to stop servos in emergency
**Mitigation**:
- Use redundant E-stop circuits (if critical)
- Test E-stop monthly
- Firmware watchdog timeout as backup
- Clear visual and audio indication

### Hazard: Thermal Runaway
**Consequence**: Component damage, fire risk (unlikely but possible)
**Mitigation**:
- Monitor AMS1117 temperature
- Ensure adequate heatsink
- Design for natural convection
- Implement thermal shutdown

### Hazard: Mechanical Entanglement
**Consequence**: Injury from moving servos
**Mitigation**:
- Guard all moving parts
- Limit servo torque where possible
- Use weak-link mechanical design
- Operator training mandatory

---

**Last Updated**: 2025-11-13
**Next Review**: 2025-12-13