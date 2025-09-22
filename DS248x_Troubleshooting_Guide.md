# DS2482-800 Troubleshooting Guide

## Checksum Error Patterns and Solutions

### Common Error Patterns

1. **Specific Sensors Always Fail**
   - **Symptoms:** Same sensors consistently show "Scratch pad checksum invalid!"
   - **Cause:** Hardware issues with those specific sensors or their wiring
   - **Solutions:**
     - Check sensor connections and solder joints
     - Verify sensor is still functional (test with separate hardware)
     - Inspect cable for damage, especially at connector points
     - Check for water ingress in sensor connections

2. **Intermittent Failures on Multiple Sensors**
   - **Symptoms:** Different sensors fail randomly
   - **Cause:** Bus timing, power supply, or EMI issues
   - **Solutions:**
     - Reduce I2C frequency to 50kHz or lower
     - Add better power supply filtering
     - Use shielded cables for 1-Wire buses
     - Check for EMI sources (pumps, motors, inverters)

3. **Channel-Specific Failures**
   - **Symptoms:** All sensors on certain channels fail
   - **Cause:** DS2482-800 channel hardware issues or bus loading
   - **Solutions:**
     - Redistribute sensors across channels
     - Check channel-specific wiring
     - Verify DS2482-800 module integrity

## Hardware Diagnostics

### Power Supply Issues
```
Symptoms: Random failures, all-zeros or all-ones patterns
Solutions:
- Measure 3.3V/5V rails under load
- Add 100µF + 10µF capacitors near DS2482-800
- Use separate power supply for sensor buses if needed
- Check for voltage drops during pump/motor operation
```

### I2C Bus Issues
```
Symptoms: Communication timeouts, inconsistent responses
Solutions:
- Add 4.7kΩ pull-up resistors on SDA/SCL if missing
- Reduce I2C frequency: 400kHz → 100kHz → 50kHz
- Shorten I2C cable lengths
- Use twisted pair cables for I2C
- Add 100pF capacitors across pull-up resistors for noise filtering
```

### 1-Wire Bus Issues
```
Symptoms: Checksum errors, sensor timeouts
Solutions:
- Enable strong pullup for cables >3 meters
- Add 120Ω termination resistor at end of long runs
- Use Cat5/Cat6 cable (twisted pairs) for 1-Wire
- Keep 1-Wire cables away from power cables
- Star topology is better than daisy-chain for multiple sensors
```

## Configuration Recommendations

### For Problematic Installations
```yaml
i2c:
  frequency: 50kHz        # Very low for maximum reliability

ds248x:
  active_pullup: true     # Always enable
  strong_pullup: true     # Essential for long cables
  bus_sleep: false        # Disable for maximum stability
  update_interval: 60s    # Reduce bus stress

sensor:
  - platform: ds248x
    resolution: 11         # Lower resolution = faster conversion
    filters:
      - filter_out: nan
      - median:
          window_size: 7   # Aggressive filtering
          send_every: 3    # Reduce update frequency
      - delta: 1.0         # Only send significant changes
```

### Diagnostic Configuration
```yaml
logger:
  level: DEBUG
  logs:
    ds248x: DEBUG
    sensor: INFO
```

## Physical Installation Best Practices

### Wiring Guidelines
1. **I2C Bus:**
   - Maximum 3 meters total length
   - Use twisted pair (Cat5/Cat6)
   - Add pull-up resistors at controller end
   - Avoid running parallel to power cables

2. **1-Wire Sensors:**
   - Maximum 100 meters per channel with strong pullup
   - Use solid core wire for permanent installations
   - Star topology preferred over daisy-chain
   - Waterproof all connections in marine/RV environments

3. **Power Distribution:**
   - Dedicated clean 3.3V/5V supply for DS2482-800
   - Local decoupling capacitors
   - Separate analog and digital grounds if possible

### Environmental Considerations
1. **RV/Marine Installations:**
   - Use marine-grade connectors
   - Apply dielectric grease to connections
   - Protect from vibration and temperature cycling
   - Route cables through grommets and strain reliefs

2. **EMI Mitigation:**
   - Keep sensor cables away from:
     - Inverter/battery charger cables
     - Pump and motor power lines
     - RF transmitters (radio/cell boosters)
   - Use shielded cables in high-EMI environments
   - Ground shields at one end only

## Diagnostic Commands

### Enable Enhanced Logging
Add to your ESPHome configuration:
```yaml
logger:
  level: DEBUG
  logs:
    ds248x: VERBOSE
```

### Monitor Success Rates
The diagnostic version logs sensor statistics every 5 minutes:
- Total reads per sensor
- Failed reads and checksum errors
- Success rate percentages
- Time since last successful read

### Identify Problem Sensors
Look for patterns in the logs:
- Sensors with <80% success rate need attention
- Consistent failures on specific channels indicate wiring issues
- All-zeros/all-ones patterns suggest power or connection problems

## Quick Fix Checklist

1. **Immediate Actions:**
   - [ ] Reduce I2C frequency to 50kHz
   - [ ] Enable strong pullup
   - [ ] Disable bus sleep
   - [ ] Add median filters to problem sensors

2. **Hardware Inspection:**
   - [ ] Check all connections for corrosion/looseness
   - [ ] Verify power supply voltage and stability
   - [ ] Measure resistance of long 1-Wire cables
   - [ ] Look for water ingress in junction boxes

3. **Advanced Diagnostics:**
   - [ ] Use oscilloscope to check I2C signal quality
   - [ ] Test sensors individually with DS2482-100
   - [ ] Swap sensors between channels to isolate problems
   - [ ] Monitor supply voltage during pump/motor operation

## When to Replace Hardware

Replace DS2482-800 module if:
- Multiple channels fail simultaneously
- I2C communication is unreliable even at 50kHz
- Config register reads return inconsistent values

Replace sensors if:
- Consistent checksum errors on one sensor across channels
- Sensor works initially but fails after temperature cycling
- Physical damage visible on sensor or cable

## Support Resources

- ESPHome DS248x Component Documentation
- DS2482-800 Datasheet (Maxim/Analog Devices)
- 1-Wire Design Guide (Application Note 148)
- ESPHome Community Forum for specific installation help