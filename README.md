# blimp-control

**blimp-control** is a Python library for a custom embedded flight controller board. The board includes an accelerometer, gyroscope, magnetometer, and 4 brushed DC motors. This library is designed for production use, fully documented using Sphinx, and built with modularity and extensibility in mind.

## Features

- Modular design for sensors (Accelerometer, Gyroscope, Magnetometer) and actuators (Motor).
- Clear, class-based API.
- Simulation of I²C and GPIO operations for development off-hardware.
- Comprehensive Sphinx documentation.
- Python 3.10+ type hinting.
- PEP8 compliant.

## Installation

To install the library, you can use pip with a `setup.py` or `pyproject.toml` (once created):

```bash
# Placeholder for actual installation command
# pip install .
```

## Usage Examples

Here's a quick example of how to use the `blimpcontrol` library:

```python
from blimpcontrol import Motor, Accelerometer, Gyroscope, Magnetometer

# Initialize a motor on GPIO pins 17 and 18
# Assumes pin 17 is used for PWM if no dedicated pwm_pin is provided
motor1 = Motor(pin1=17, pin2=18)

# Initialize an accelerometer on I2C bus 1, address 0x68
accel = Accelerometer(bus=1, addr=0x68) # BMI270 default address

# Initialize a gyroscope (often same chip as accelerometer)
gyro = Gyroscope(bus=1, addr=0x68) # BMI270 default address

# Initialize a magnetometer on I2C bus 1, address 0x10 (example for BMM350)
# Note: BMM350 typical addresses can vary, e.g. 0x10, 0x11, 0x12, 0x13
# Or DFRobot module default 0x13
mag = Magnetometer(bus=1, addr=0x13)


print("--- Motor Control ---")
motor1.spin_forward(150, ramp="quadratic")
# Simulate some action
import time
time.sleep(1)
motor1.stop()
time.sleep(0.5)

print("\n--- Sensor Readings ---")
accel_data = accel.get_xyz()
print(f"Accelerometer: X={accel_data[0]}, Y={accel_data[1]}, Z={accel_data[2]}")

gyro_data = gyro.get_xyz()
print(f"Gyroscope: X={gyro_data[0]}, Y={gyro_data[1]}, Z={gyro_data[2]}")

mag_data = mag.get_xyz()
print(f"Magnetometer: X={mag_data[0]}, Y={mag_data[1]}, Z={mag_data[2]}")

# Clean up GPIO resources used by the motor
motor1.cleanup()

print("\nExample finished.")
```

## Documentation

The full documentation is built with Sphinx and can be found in the `docs/build/html` directory after running `make html` inside `docs/`.

```bash
cd docs
make html
# Open docs/build/html/index.html in your browser
```

## Modules

- `blimpcontrol.accelerometer`: Provides the `Accelerometer` class.
- `blimpcontrol.gyroscope`: Provides the `Gyroscope` class.
- `blimpcontrol.magnetometer`: Provides the `Magnetometer` class.
- `blimpcontrol.motor`: Provides the `Motor` class.
- `blimpcontrol.utils`: Utility functions.
- `blimpcontrol.interfaces`: Abstract base classes (e.g., `SensorInterface`).

Refer to the Sphinx documentation for detailed API references.

## Contributing

Contributions are welcome! Please ensure your code follows PEP8 and includes type hints and docstrings.

## License

This project is licensed under the MIT License - see the LICENSE file for details (not created yet, but assuming MIT).
