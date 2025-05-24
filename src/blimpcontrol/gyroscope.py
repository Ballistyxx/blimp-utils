"""
Gyroscope module for the blimpcontrol library.
"""

from typing import List

class Gyroscope:
    """
    A class to represent the gyroscope.

    :param bus: The I2C bus number.
    :type bus: int
    :param addr: The I2C address of the gyroscope.
    :type addr: int
    """
    def __init__(self, bus: int, addr: int):
        """
        Initialize the gyroscope.

        :param bus: The I2C bus number.
        :type bus: int
        :param addr: The I2C address of the gyroscope.
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
        print(f"Gyroscope initialized on I2C bus {self.bus}, address {hex(self.addr)}")

    def _read_sensor(self, register: int) -> int:
        """
        Simulates reading a raw sensor value from a given register.

        :param register: The register to read from.
        :type register: int
        :return: The raw sensor value.
        :rtype: int
        """
        # Placeholder for I2C read
        if register == 0x12: # GYR_X_7_0
            return 50
        elif register == 0x14: # GYR_Y_7_0
            return 75
        elif register == 0x16: # GYR_Z_7_0
            return 125
        return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis gyroscope value.

        :return: The raw X-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_x()``
        """
        return self._read_sensor(0x12) # Placeholder for X-axis register

    def get_y(self) -> int:
        """
        Get the raw Y-axis gyroscope value.

        :return: The raw Y-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_y()``
        """
        return self._read_sensor(0x14) # Placeholder for Y-axis register

    def get_z(self) -> int:
        """
        Get the raw Z-axis gyroscope value.

        :return: The raw Z-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_z()``
        """
        return self._read_sensor(0x16) # Placeholder for Z-axis register

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis gyroscope values.

        :return: A list containing [X, Y, Z] gyroscope values.
        :rtype: list[int]
        :example: ``gyro.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]
