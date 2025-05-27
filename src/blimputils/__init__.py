"""
blimp-utils - A Python library for the Falcon Flight embedded controller board.


:license: The MIT License (MIT)
:author: Eli Ferrara 
:email: eli.ferrara256@gmail.com
:version: V1.0.0
:date: 2025-05-27
:url: https://github.com/Ballistyxx/blimp-utils
"""
from typing import Union, Dict, Tuple # Adjusted imports

from .accelerometer import Accelerometer
from .gyroscope import Gyroscope
from .magnetometer import Magnetometer
from .motor import Motor

# Default sensor addresses and bus (assuming I2C)
DEFAULT_ACCEL_ADDR = 0x68
DEFAULT_GYRO_ADDR = 0x68
DEFAULT_MAG_ADDR = 0x14 # Corrected BMM350 default address
DEFAULT_I2C_BUS = 1     # Usually 1, can be adjusted to 0

# Default motor pin configurations (using BCM numbering)
DEFAULT_MOTOR_PINS: Dict[str, Dict[str, int]] = {
    "motor1": {"pin1": 17, "pin2": 18, "pwm_pin": None}, # pwm_pin can be None or an int
    "motor2": {"pin1": 27, "pin2": 22, "pwm_pin": None},
    "motor3": {"pin1": 23, "pin2": 24, "pwm_pin": None},
    "motor4": {"pin1": 5,  "pin2": 6,  "pwm_pin": None},
}

# Global instances for sensors and motors
# Type hints for clarity
accelerometer_instance: Union[Accelerometer, None] = None
gyroscope_instance: Union[Gyroscope, None] = None
magnetometer_instance: Union[Magnetometer, None] = None
motors: Dict[str, Motor] = {}

def init(
    i2c_bus: int = DEFAULT_I2C_BUS,
    accel_addr: Union[int, None] = DEFAULT_ACCEL_ADDR, # Allow None to skip init
    gyro_addr: Union[int, None] = DEFAULT_GYRO_ADDR,   # Allow None to skip init
    mag_addr: Union[int, None] = DEFAULT_MAG_ADDR,     # Allow None to skip init
    motor_pins_config: Union[Dict[str, Dict[str, Union[int, None]]], None] = None,
) -> Tuple[Union[Accelerometer, None], Union[Gyroscope, None], Union[Magnetometer, None], Dict[str, Motor]]:
    """
    Initializes the sensors and motors.

    :param i2c_bus: The I2C bus number for sensors.
    :param accel_addr: I2C address for the accelerometer. If None, accelerometer is not initialized.
    :param gyro_addr: I2C address for the gyroscope. If None, gyroscope is not initialized.
    :param mag_addr: I2C address for the magnetometer. If None, magnetometer is not initialized.
    :param motor_pins_config: Configuration for motor pins. 
                              Example: {"motor1": {"pin1": 17, "pin2": 18, "pwm_pin": None}}
                              If None, uses DEFAULT_MOTOR_PINS.
    :return: A tuple containing the initialized (or None) sensor instances and a dictionary of motor instances.
    :raises IOError: If sensor initialization fails.
    """
    global accelerometer_instance, gyroscope_instance, magnetometer_instance, motors

    # Initialize Sensors
    if accel_addr is not None:
        try:
            accelerometer_instance = Accelerometer(bus=i2c_bus, addr=accel_addr)
        except IOError as e:
            print(f"Failed to initialize accelerometer: {e}")
            accelerometer_instance = None # Ensure it's None on failure
    else:
        accelerometer_instance = None
        print("Accelerometer initialization skipped (address is None).")

    if gyro_addr is not None:
        try:
            gyroscope_instance = Gyroscope(bus=i2c_bus, addr=gyro_addr)
        except IOError as e:
            print(f"Failed to initialize gyroscope: {e}")
            gyroscope_instance = None
    else:
        gyroscope_instance = None
        print("Gyroscope initialization skipped (address is None).")

    if mag_addr is not None:
        try:
            magnetometer_instance = Magnetometer(bus=i2c_bus, addr=mag_addr)
        except IOError as e:
            print(f"Failed to initialize magnetometer: {e}")
            magnetometer_instance = None
    else:
        magnetometer_instance = None
        print("Magnetometer initialization skipped (address is None).")

    # Initialize Motors
    current_motor_pins = motor_pins_config if motor_pins_config is not None else DEFAULT_MOTOR_PINS
    motors = {}
    for name, pins in current_motor_pins.items():
        try:
            # Ensure pwm_pin is correctly passed as Union[int, None]
            pwm_pin_val = pins.get("pwm_pin") # Will be int or None
            motors[name] = Motor(pin1=pins["pin1"], pin2=pins["pin2"], pwm_pin=pwm_pin_val)
        except Exception as e:
            print(f"Failed to initialize motor {name}: {e}")
            # Optionally, decide if this should raise an error or just skip the motor

    return accelerometer_instance, gyroscope_instance, magnetometer_instance, motors

def cleanup():
    """
    Cleans up resources used by sensors and motors.
    """
    global accelerometer_instance, gyroscope_instance, magnetometer_instance, motors

    if accelerometer_instance:
        accelerometer_instance.close()
    if gyroscope_instance:
        gyroscope_instance.close()
    if magnetometer_instance:
        magnetometer_instance.close()
    
    for motor_name, motor_obj in motors.items():
        try:
            motor_obj.cleanup()
        except Exception as e:
            print(f"Error cleaning up motor {motor_name}: {e}")
    
    print("blimputils cleanup complete.")

#__all__ = ['Accelerometer', 'Gyroscope', 'Magnetometer', 'Motor', 'init', 'cleanup',
#           'DEFAULT_ACCEL_ADDR', 'DEFAULT_GYRO_ADDR', 'DEFAULT_MAG_ADDR', 'DEFAULT_I2C_BUS', 'DEFAULT_MOTOR_PINS']
