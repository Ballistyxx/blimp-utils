"""
Accelerometer module for the blimpcontrol library.
"""

from typing import List

class Accelerometer:
    """
    A class to represent the accelerometer.

    :param bus: The I2C bus number.
    :type bus: int
    :param addr: The I2C address of the accelerometer.
    :type addr: int
    """
    def __init__(self, bus: int, addr: int):
        """
        Initialize the accelerometer.

        :param bus: The I2C bus number.
        :type bus: int
        :param addr: The I2C address of the accelerometer.
        :type addr: int
        :raises TypeError: if bus or addr are not integers.
        """
        if not isinstance(bus, int):
            raise TypeError("I2C bus must be an integer")
        if not isinstance(addr, int):
            raise TypeError("I2C address must be an integer")
        self.bus = bus
        self.addr = addr
        # In a real implementation, initialize I2C communication here
        # from smbus2 import SMBus
        # self.i2c_bus = SMBus(self.bus)
        print(f"Accelerometer initialized on I2C bus {self.bus}, address {hex(self.addr)}")

    def _read_sensor(self, register: int) -> int:
        """
        Simulates reading a raw sensor value from a given register.

        In a real implementation, this would perform an I2C read.
        For now, it returns a placeholder value.

        :param register: The register to read from.
        :type register: int
        :return: The raw sensor value.
        :rtype: int
        """
        # Placeholder for I2C read: self.i2c_bus.read_byte_data(self.addr, register)
        # Returning a simulated value
        if register == 0x0C: # ACC_X_7_0 as a placeholder for X
            return 100
        elif register == 0x0E: # ACC_Y_7_0 as a placeholder for Y
            return 150
        elif register == 0x10: # ACC_Z_7_0 as a placeholder for Z
            return 200
        return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis acceleration value.

        :return: The raw X-axis acceleration.
        :rtype: int
        :example: ``accel.get_x()``
        """
        # In a real BMI270, you'd read ACC_X_7_0 and ACC_X_15_8
        # For simplicity, returning a single register read
        return self._read_sensor(0x0C) # Placeholder for X-axis register

    def get_y(self) -> int:
        """
        Get the raw Y-axis acceleration value.

        :return: The raw Y-axis acceleration.
        :rtype: int
        :example: ``accel.get_y()``
        """
        return self._read_sensor(0x0E) # Placeholder for Y-axis register

    def get_z(self) -> int:
        """
        Get the raw Z-axis acceleration value.

        :return: The raw Z-axis acceleration.
        :rtype: int
        :example: ``accel.get_z()``
        """
        return self._read_sensor(0x10) # Placeholder for Z-axis register

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis acceleration values.

        :return: A list containing [X, Y, Z] acceleration values.
        :rtype: list[int]
        :example: ``accel.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]
