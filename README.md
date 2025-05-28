
**blimp-utils** is a Python library designed specifically for the custom embedded flight controller board for the Falcon Flight project. The board includes an accelerometer, gyroscope, magnetometer, and motor drivers for up to 4 brushed DC motors. This library is designed for production use, fully documented using Sphinx, and built with modularity and extensibility in mind.

## Features

- Modular design for sensors (Accelerometer, Gyroscope, Magnetometer) and actuators (Motor).
- Clear, class-based API.
- Simulation of I²C and GPIO operations for development off-hardware.
- Comprehensive Sphinx documentation.
- Python 3.9+ type hinting.
- PEP8 compliant.

## Installation

To install the library, run the following command in a terminal:

```bash
git clone https://github.com/Ballistyxx/blimp-utils.git
cd blimp-utils
sudo pip3 install .
```

## Usage Examples

Here's a quick example of how to use the `blimp-utils` library:

```python
from blimputils import Motor, Accelerometer, Gyroscope, Magnetometer

# Initialize a motor on GPIO pins 13 and 21
# Assumes pin 13 is used for PWM if no dedicated pwm_pin is provided
motor1 = Motor(pin1=13, pin2=21)

# Initialize an accelerometer on I2C bus 1, address 0x68
accel = Accelerometer(bus=1, addr=0x68) # BMI270 default address

# Initialize a gyroscope (same chip as accelerometer)
gyro = Gyroscope(bus=1, addr=0x68) # BMI270 default address

# Initialize a magnetometer on I2C bus 1, address 0x14
mag = Magnetometer(bus=1, addr=0x14)


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
pip install -U sphinx
git clone https://github.com/Ballistyxx/blimp-utils.git
cd blimp-utils/docs
make clean
make html
# Open docs/build/html/index.html in your browser
```

Documentation can also be found at https://ballistyxx.github.io/blimp-utils/

## Modules

- `blimputils.accelerometer`: Provides the `Accelerometer` class.
- `blimputils.gyroscope`: Provides the `Gyroscope` class.
- `blimputils.magnetometer`: Provides the `Magnetometer` class.
- `blimputils.motor`: Provides the `Motor` class.
- `blimputils.utils`: Utility functions.
- `blimputils.interfaces`: Abstract base classes (e.g., `SensorInterface`).

Refer to the Sphinx documentation for detailed API references.

## Contributing

Contributions are welcome! Please ensure your code follows PEP8 and includes type hints and docstrings.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details
