"""
Utility functions for the blimpcontrol library.
"""

def example_util_function():
    """
    An example utility function.

    :return: A string message.
    :rtype: str
    """
    return "This is a utility function."

# Add any other common utility functions here, for example,
# functions for I2C communication, data conversion, etc.
# if they are shared across multiple sensor/actuator modules.

# For example, a mock I2C read function if not using a library like smbus2
def mock_i2c_read_byte(bus: int, addr: int, register: int) -> int:
    """
    Mocks an I2C byte read for simulation purposes.

    :param bus: I2C bus number.
    :type bus: int
    :param addr: I2C device address.
    :type addr: int
    :param register: Register to read from.
    :type register: int
    :return: A simulated byte value (e.g., 0-255).
    :rtype: int
    """
    print(f"Simulated I2C read: bus={bus}, addr={hex(addr)}, reg={hex(register)}")
    # Return a pseudo-random value based on inputs for some variation
    return (bus + addr + register) % 256

def mock_i2c_write_byte(bus: int, addr: int, register: int, value: int) -> None:
    """
    Mocks an I2C byte write for simulation purposes.

    :param bus: I2C bus number.
    :type bus: int
    :param addr: I2C device address.
    :type addr: int
    :param register: Register to write to.
    :type register: int
    :param value: Byte value to write.
    :type value: int
    """
    print(f"Simulated I2C write: bus={bus}, addr={hex(addr)}, reg={hex(register)}, value={hex(value)}")

