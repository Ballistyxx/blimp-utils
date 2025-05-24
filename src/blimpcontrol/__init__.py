"""
blimpcontrol - A Python library for the Falcon Flight controller board.
"""
from typing import Union # Add Union

from .accelerometer import Accelerometer
from .gyroscope import Gyroscope
from .magnetometer import Magnetometer
from .motor import Motor, IS_RASPBERRY_PI

# Default sensor addresses and bus (assuming I2C)
# These might vary based on your specific hardware configuration.
# For BMI270, the typical addresses are 0x68 or 0x69.
# For BMM350, the typical address is 0x10, 0x12, 0x13 or 0x14 (check datasheet/config).
DEFAULT_ACCEL_ADDR = 0x68
DEFAULT_GYRO_ADDR = 0x68  # Often same as accelerometer if it's an IMU like BMI270
DEFAULT_MAG_ADDR = 0x10 # Example for BMM350
DEFAULT_I2C_BUS = 1     # Common on Raspberry Pi

# Default motor pin configurations (using BCM numbering)
# These are examples and MUST be configured based on your board's wiring.
# Each motor driver (e.g., DRV8871) needs two control pins.
DEFAULT_MOTOR_PINS = {
    "motor1": {"pin1": 17, "pin2": 18}, # Example pins for Motor 1
    "motor2": {"pin1": 27, "pin2": 22}, # Example pins for Motor 2
    "motor3": {"pin1": 23, "pin2": 24}, # Example pins for Motor 3
    "motor4": {"pin1": 5, "pin2": 6},   # Example pins for Motor 4
}

# Global instances for sensors and motors
accelerometer_instance: Union[Accelerometer, None] = None
gyroscope_instance: Union[Gyroscope, None] = None
magnetometer_instance: Union[Magnetometer, None] = None
motors: dict[str, Motor] = {}

def init(
    i2c_bus: int = DEFAULT_I2C_BUS,
    accel_addr: int = DEFAULT_ACCEL_ADDR,
    gyro_addr: int = DEFAULT_GYRO_ADDR,
    mag_addr: int = DEFAULT_MAG_ADDR,
    motor_pins: Union[dict[str, dict[str, int]], None] = None,
    gpio_warnings: bool = False
) -> tuple[Accelerometer, Gyroscope, Magnetometer, dict[str, Motor]]:
    """
    Initializes the blimpcontrol library, setting up sensors and motors.

    This function configures the GPIO mode (BCM), disables warnings,
    and initializes the accelerometer, gyroscope, magnetometer, and up to 4 motors
    with specified or default I2C addresses and GPIO pins.

    :param i2c_bus: The I2C bus number for the sensors. Defaults to `DEFAULT_I2C_BUS`.
    :type i2c_bus: int
    :param accel_addr: The I2C address for the accelerometer. Defaults to `DEFAULT_ACCEL_ADDR`.
    :type accel_addr: int
    :param gyro_addr: The I2C address for the gyroscope. Defaults to `DEFAULT_GYRO_ADDR`.
    :type gyro_addr: int
    :param mag_addr: The I2C address for the magnetometer. Defaults to `DEFAULT_MAG_ADDR`.
    :type mag_addr: int
    :param motor_pins: A dictionary defining the pins for each motor.
                       If None, uses `DEFAULT_MOTOR_PINS`.
                       Example: ``{"motor1": {"pin1": 17, "pin2": 18}, ...}``
    :type motor_pins: dict[str, dict[str, int]], optional
    :param gpio_warnings: Whether to enable GPIO warnings. Defaults to False.
    :type gpio_warnings: bool
    :return: A tuple containing the initialized (Accelerometer, Gyroscope, Magnetometer, motors_dict).
    :rtype: tuple[Accelerometer, Gyroscope, Magnetometer, dict[str, Motor]]
    :raises ImportError: if RPi.GPIO is not available and not being mocked.
    :raises ValueError: if motor_pins configuration is invalid.

    :example:
    >>> from blimpcontrol import init, Accelerometer, Gyroscope, Magnetometer, Motor
    >>> accel, gyro, mag, all_motors = init()
    >>> motor1 = all_motors["motor1"]
    >>> motor1.spin_forward(100)
    >>> print(accel.get_xyz())
    """
    global accelerometer_instance, gyroscope_instance, magnetometer_instance, motors

    # Initialize GPIO settings
    if IS_RASPBERRY_PI:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(gpio_warnings)
        print("RPi.GPIO initialized, mode BCM, warnings " + ("enabled" if gpio_warnings else "disabled"))
    else:
        # MockGPIO is already instantiated in motor.py if RPi.GPIO is not found
        from .motor import GPIO as MockGPIO_instance # Access the instance
        MockGPIO_instance.setmode("bcm") # Using string as per MockGPIO
        MockGPIO_instance.setwarnings(gpio_warnings)
        print("MockGPIO initialized, mode BCM, warnings " + ("enabled" if gpio_warnings else "disabled"))


    # Initialize Sensors
    accelerometer_instance = Accelerometer(bus=i2c_bus, addr=accel_addr)
    gyroscope_instance = Gyroscope(bus=i2c_bus, addr=gyro_addr)
    magnetometer_instance = Magnetometer(bus=i2c_bus, addr=mag_addr)

    # Initialize Motors
    current_motor_pins = motor_pins if motor_pins is not None else DEFAULT_MOTOR_PINS
    motors.clear() # Clear any previous motor instances
    for name, pins in current_motor_pins.items():
        if "pin1" not in pins or "pin2" not in pins:
            raise ValueError(f"Motor '{name}' configuration is missing 'pin1' or 'pin2'.")
        motors[name] = Motor(pin1=pins["pin1"], pin2=pins["pin2"])
        print(f"Initialized {name} with pin1={pins['pin1']}, pin2={pins['pin2']}")

    return accelerometer_instance, gyroscope_instance, magnetometer_instance, motors

def get_accelerometer() -> Accelerometer:
    """Returns the initialized Accelerometer instance."""
    if accelerometer_instance is None:
        raise RuntimeError("Accelerometer not initialized. Call blimpcontrol.init() first.")
    return accelerometer_instance

def get_gyroscope() -> Gyroscope:
    """Returns the initialized Gyroscope instance."""
    if gyroscope_instance is None:
        raise RuntimeError("Gyroscope not initialized. Call blimpcontrol.init() first.")
    return gyroscope_instance

def get_magnetometer() -> Magnetometer:
    """Returns the initialized Magnetometer instance."""
    if magnetometer_instance is None:
        raise RuntimeError("Magnetometer not initialized. Call blimpcontrol.init() first.")
    return magnetometer_instance

def get_motor(name: str) -> Motor:
    """
    Returns a specific initialized Motor instance by name.

    :param name: The name of the motor (e.g., "motor1").
    :type name: str
    :return: The Motor instance.
    :rtype: Motor
    :raises KeyError: if the motor name is not found.
    :raises RuntimeError: if motors are not initialized via blimpcontrol.init().
    """
    if not motors:
        raise RuntimeError("Motors not initialized. Call blimpcontrol.init() first.")
    if name not in motors:
        raise KeyError(f"Motor with name '{name}' not found. Available motors: {list(motors.keys())}")
    return motors[name]

def get_all_motors() -> dict[str, Motor]:
    """Returns the dictionary of all initialized motor instances."""
    if not motors:
        raise RuntimeError("Motors not initialized. Call blimpcontrol.init() first.")
    return motors

__version__ = "1.0.0"
