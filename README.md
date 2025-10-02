# Dual Gimbal Motor Controller with AS5048 Magnetic Encoders

A comprehensive solution for controlling two gimbal motors using a Raspberry Pi Pico and FOC driver boards with the SimpleFOC library and AS5048 absolute magnetic encoders.

## Features

- **Dual Motor Control**: Independent control of two gimbal motors (pitch and yaw axes)
- **FOC Control**: Field-Oriented Control for smooth, efficient motor operation
- **AS5048 Magnetic Encoders**: Absolute position sensing with 14-bit resolution
- **Real-time Feedback**: Continuous position and velocity monitoring
- **Serial Interface**: Easy debugging and control via serial commands
- **Gimbal-Optimized**: Parameters tuned specifically for gimbal applications
- **No Homing Required**: Absolute position sensing eliminates startup homing

## Hardware Requirements

### Raspberry Pi Pico
- 1x Raspberry Pi Pico (or Pico W)

### FOC Driver Boards
- 2x FOC driver boards (e.g., SimpleFOC compatible drivers)
- 12V power supply for motors

### Motors
- 2x Gimbal motors with AS5048 magnetic encoders
- Typical specifications: 7 pole pairs

### AS5048 Magnetic Encoders
- 2x AS5048A (SPI) or AS5048B (I²C) magnetic position sensors
- 14-bit resolution (16,384 counts per revolution)
- Absolute position sensing
- PWM output interface

## Wiring

### AS5048 Magnetic Encoder Connections

#### AS5048 Pin Configuration
```
AS5048 Pin    Pico Pin    Function
VCC      ->   3.3V       Power (3.3V)
GND      ->   GND        Ground
PWM      ->   GPIO 8     PWM output (Motor 1)
PWM      ->   GPIO 9     PWM output (Motor 2)
CS       ->   Not used   (PWM mode)
CLK      ->   Not used   (PWM mode)
DO       ->   Not used   (PWM mode)
```

#### Motor 1 (Pitch Axis)
```
Pico Pin    FOC Driver 1    AS5048
GPIO 2  -> PWM A
GPIO 3  -> PWM B  
GPIO 4  -> PWM C
GPIO 8  -> PWM Output (AS5048 Sensor 1)
```

#### Motor 2 (Yaw Axis)
```
Pico Pin    FOC Driver 2    AS5048
GPIO 5  -> PWM A
GPIO 6  -> PWM B
GPIO 7  -> PWM C
GPIO 9  -> PWM Output (AS5048 Sensor 2)
```

#### Power Connections
- 12V supply to both FOC drivers
- 3.3V supply to both AS5048 encoders
- GND connections between Pico, drivers, and encoders
- 3.3V/5V supply to Pico (via USB or external supply)

## Software Setup

### Prerequisites
- PlatformIO installed
- SimpleFOC library (automatically installed via platformio.ini)

### Building and Uploading
```bash
# Build the project
pio run

# Upload to Pico
pio run --target upload

# Monitor serial output
pio device monitor
```

## Usage

### Serial Commands

Once the system is running, you can control the motors via serial commands:

#### Basic Commands
- `set1 <angle>` - Set motor 1 (pitch) target angle in degrees
- `set2 <angle>` - Set motor 2 (yaw) target angle in degrees
- `home1` - Home motor 1 to 0 degrees
- `home2` - Home motor 2 to 0 degrees
- `home` - Home both motors to 0 degrees
- `stop` - Stop both motors
- `status` - Show detailed status
- `help` - Show available commands

#### Examples
```
set1 45.5    # Move motor 1 to 45.5 degrees
set2 -30     # Move motor 2 to -30 degrees
home         # Home both motors to 0 degrees
status       # Check current positions
```

### Real-time Monitoring

The system provides continuous status updates showing:
- Current angles for both motors
- Target angles
- Motor velocities
- Enable/disable status
- AS5048 sensor readings

## Configuration

### AS5048 Encoder Parameters

The code is configured for AS5048 magnetic encoders:

```cpp
// AS5048 configuration
MagneticSensorPWM sensor1 = MagneticSensorPWM(8, 4, 904);
// PWM pin: GPIO 8
// Pulse count: 4 (for 4 pulses per revolution)
// Max angle: 904 (for 360° range)
```

### Motor Parameters

```cpp
// Motor specifications
BLDCMotor motor1 = BLDCMotor(7);  // 7 pole pairs

// Gimbal-optimized settings
motor1.voltage_sensor_align = 1.0;  // Low voltage for alignment
motor1.velocity_limit = 50.0;       // 50 rad/s max velocity
motor1.voltage_limit = 3.0;         // 3V max voltage
```

### Pin Configuration

If you need to change pin assignments, modify these lines in `main.cpp`:

```cpp
// Motor 1 pins
BLDCDriver3PWM driver1 = BLDCDriver3PWM(2, 3, 4, 1);
MagneticSensorPWM sensor1 = MagneticSensorPWM(8, 4, 904);

// Motor 2 pins  
BLDCDriver3PWM driver2 = BLDCDriver3PWM(5, 6, 7, 1);
MagneticSensorPWM sensor2 = MagneticSensorPWM(9, 4, 904);
```

## AS5048 Advantages

### Absolute Position Sensing
- **No homing required**: Knows exact position on startup
- **Power-loss recovery**: Maintains position reference
- **Multi-turn capability**: Can track multiple revolutions

### High Precision
- **14-bit resolution**: 16,384 counts per revolution
- **Sub-degree accuracy**: Better than 0.022° resolution
- **Low noise**: Magnetic sensing is very stable

### Robust Operation
- **No mechanical wear**: Contactless magnetic sensing
- **Temperature stable**: Consistent performance across temperature
- **EMI resistant**: Less susceptible to electrical interference

### Simplified Wiring
- **Single wire PWM**: Only one signal wire per encoder
- **No differential signals**: Standard digital I/O
- **Lower pin count**: Reduces wiring complexity

## Troubleshooting

### Common Issues

1. **Motors not moving**
   - Check power supply connections
   - Verify AS5048 PWM connections
   - Check serial output for initialization errors

2. **Erratic movement**
   - Verify AS5048 magnet positioning
   - Check magnet distance (1-3mm from sensor)
   - Ensure magnet is centered over sensor

3. **Position drift**
   - Check magnet mounting stability
   - Verify AS5048 power supply (3.3V stable)
   - Ensure no magnetic interference

4. **Serial communication issues**
   - Ensure baud rate is set to 115200
   - Check USB connection
   - Try different USB cable/port

### Debug Information

The system provides detailed debug output:
- Initialization status for each motor and AS5048
- Real-time position and velocity data
- AS5048 sensor readings
- Error messages for troubleshooting

## Advanced Configuration

### Tuning for Different Motors

If using different motors, adjust these parameters:

```cpp
// For different pole pairs
BLDCMotor motor1 = BLDCMotor(YOUR_POLE_PAIRS);

// Adjust voltage limits for your power supply
driver1.voltage_power_supply = YOUR_VOLTAGE;
motor1.voltage_limit = YOUR_LIMIT;
```

### AS5048 Alternative Interfaces

If you prefer SPI or I²C instead of PWM:

```cpp
// For SPI interface (AS5048A)
MagneticSensorSPI sensor1 = MagneticSensorSPI(CS_PIN, SPI_BUS);

// For I²C interface (AS5048B)
MagneticSensorI2C sensor1 = MagneticSensorI2C(I2C_ADDRESS, I2C_BUS);
```

### Adding More Features

The code structure makes it easy to add:
- PID tuning parameters
- Different control modes
- Safety limits
- External sensor integration
- Multi-turn position tracking

## Safety Notes

- Always ensure proper power supply ratings
- Check all connections before powering on
- Start with low voltage limits and increase gradually
- Keep hands clear of moving parts during testing
- Ensure AS5048 magnets are properly secured
- Maintain proper magnet-to-sensor distance

## License

This project is open source. Feel free to modify and distribute according to your needs.