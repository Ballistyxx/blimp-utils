"""
blimp-utils - A Python library for the Falcon Flight embedded controller board.
"""
from typing import Union, Dict, Tuple, Any
import time

from .accelerometer import Accelerometer
from .gyroscope import Gyroscope
from .magnetometer import Magnetometer
from .motor import Motor # Assuming Motor class is in .motor
from .utils import *

# Default sensor addresses and bus
DEFAULT_ACCEL_ADDR = 0x68
DEFAULT_GYRO_ADDR = 0x68
DEFAULT_MAG_ADDR = 0x14
DEFAULT_I2C_BUS = 1     # Usually 1, can be adjusted to 0

# Default motor pin configurations (using BCM numbering)
DEFAULT_MOTOR_PINS: Dict[str, Dict[str, int]] = {
    "motor1": {"pin1": 17, "pin2": 18, "pwm_pin": None}, # pwm_pin can be None or an int
    "motor2": {"pin1": 27, "pin2": 22, "pwm_pin": None},
    "motor3": {"pin1": 23, "pin2": 24, "pwm_pin": None},
    "motor4": {"pin1": 5,  "pin2": 6,  "pwm_pin": None},
}

PWR_CTRL = 0x7D

# Global instances for sensors and motors
accelerometer_instance: Union[Accelerometer, None] = None
gyroscope_instance: Union[Gyroscope, None] = None
magnetometer_instance: Union[Magnetometer, None] = None
# motors: Dict[str, Motor] = {} # This will be populated by the main init, not the Motor.init from motor.py

# --- Motor Proxy and __getattr__ for dynamic motor access --- 
class _MotorProxy:
    def __init__(self, motor_name: str):
        self.motor_name = motor_name
        self._instance = None

    def _get_instance(self) -> Union[Motor, None]:
        if self._instance is None:
            # Attempt to get from Motor.instances if Motor.init() was called
            if hasattr(Motor, 'instances') and isinstance(Motor.instances, dict):
                self._instance = Motor.instances.get(self.motor_name)
        return self._instance

    def __getattr__(self, name: str) -> Any:
        instance = self._get_instance()
        if instance:
            # This will now correctly fetch the 'spin' method (and others) 
            # from the actual Motor instance.
            return getattr(instance, name)
        # If trying to access an attribute before init, or if it's not on the motor object
        raise AttributeError(f"Motor '{self.motor_name}' is not initialized or attribute '{name}' not found.")

    def __bool__(self) -> bool:
        return self._get_instance() is not None

# Expose motor1, motor2, motor3, motor4 at the package level
# These will be instances of _MotorProxy
motor1 = _MotorProxy("motor1")
motor2 = _MotorProxy("motor2")
motor3 = _MotorProxy("motor3")
motor4 = _MotorProxy("motor4")

# The main init function for the blimputils package
# This is DIFFERENT from Motor.init() in motor.py
# This init handles sensors and can also call Motor.init() if needed by design.
# For the current test_motor_functionality.py, it seems Motor.init() is called directly.
# If you want this main init to also initialize motors from motor.py, you would add:
# Motor.init() # Call the static method from the motor module

def init(
    i2c_bus: int = DEFAULT_I2C_BUS,
    accel_addr: Union[int, None] = DEFAULT_ACCEL_ADDR,
    gyro_addr: Union[int, None] = DEFAULT_GYRO_ADDR,
    mag_addr: Union[int, None] = DEFAULT_MAG_ADDR,
    # motor_pins_config is for a different motor implementation if used by this init
    # motor_pins_config: Union[Dict[str, Dict[str, Union[int, None]]], None] = None,
) -> Tuple[Union[Accelerometer, None], Union[Gyroscope, None], Union[Magnetometer, None], Dict[str, Motor]]:
    """
    Initializes the sensors. Motors are initialized separately by calling Motor.init()
    from the motor module, which populates Motor.instances for the proxies.
    """
    global accelerometer_instance, gyroscope_instance, magnetometer_instance
    # Removed `motors` from global as it's not used by this simplified init for motors

    # Initialize Sensors (copied from your existing __init__.py)
    if accel_addr is not None:
        try:
            accelerometer_instance = Accelerometer(bus=i2c_bus, addr=accel_addr)
        except IOError as e:
            print(f"Failed to initialize accelerometer: {e}")
            accelerometer_instance = None
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

    if accelerometer_instance or gyroscope_instance:
        pwr_ctrl_val = 0x00
        if accelerometer_instance: pwr_ctrl_val |= 0x04
        if gyroscope_instance: pwr_ctrl_val |= 0x02
        bus_to_use = None
        addr_to_use = None
        if accelerometer_instance and accelerometer_instance.i2c_bus:
            bus_to_use = accelerometer_instance.i2c_bus
            addr_to_use = accelerometer_instance.addr
        elif gyroscope_instance and gyroscope_instance.i2c_bus:
            bus_to_use = gyroscope_instance.i2c_bus
            addr_to_use = gyroscope_instance.addr
        if bus_to_use and addr_to_use is not None and pwr_ctrl_val != 0x00:
            try:
                bus_to_use.write_byte_data(addr_to_use, PWR_CTRL, pwr_ctrl_val)
                time.sleep(0.002)
            except Exception as e:
                print(f"Error writing to PWR_CTRL: {e}")

    if mag_addr is not None:
        try:
            magnetometer_instance = Magnetometer(bus=i2c_bus, addr=mag_addr)
        except IOError as e:
            print(f"Failed to initialize magnetometer: {e}")
            magnetometer_instance = None
    else:
        magnetometer_instance = None
        print("Magnetometer initialization skipped (address is None).")

    # Motors are expected to be initialized by calling blimputils.Motor.init() directly.
    # The _MotorProxy instances (motor1, motor2, etc.) will pick up their instances from Motor.instances.
    # No motor initialization logic here for the pigpio motors.
    
    # Return sensor instances and an empty dict for motors from this specific init function
    # as the pigpio motors are handled by Motor.instances and proxies.
    return accelerometer_instance, gyroscope_instance, magnetometer_instance, {}

def cleanup():
    """
    Cleans up resources. Calls Motor.cleanup() for pigpio motors.
    """
    global accelerometer_instance, gyroscope_instance, magnetometer_instance

    # Cleanup sensors initialized by this package's init
    if accelerometer_instance and hasattr(accelerometer_instance, 'close'):
        accelerometer_instance.close()
    if gyroscope_instance and hasattr(gyroscope_instance, 'close'):
        gyroscope_instance.close()
    if magnetometer_instance and hasattr(magnetometer_instance, 'close'):
        magnetometer_instance.close()
    
    # Call the static cleanup method from the motor module
    # This will handle stopping motors in Motor.instances and cleaning up pigpio/GPIO
    Motor.cleanup()
    
    print("blimputils cleanup complete.")

# __all__ can be defined to control `from blimputils import *` behavior
__all__ = [
    'Accelerometer', 'Gyroscope', 'Magnetometer', 'Motor',
    'init', 'cleanup',
    'motor1', 'motor2', 'motor3', 'motor4',
    'DEFAULT_ACCEL_ADDR', 'DEFAULT_GYRO_ADDR', 'DEFAULT_MAG_ADDR', 'DEFAULT_I2C_BUS',
    'DEFAULT_MOTOR_PINS', 'PWR_CTRL' # Add other utilities from .utils if needed
]
