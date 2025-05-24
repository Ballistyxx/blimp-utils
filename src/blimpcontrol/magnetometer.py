"""
Magnetometer module for the blimpcontrol library.
"""

from typing import List

class Magnetometer:
    """
    A class to represent the magnetometer.

    :param bus: The I2C bus number.
    :type bus: int
    :param addr: The I2C address of the magnetometer.
    :type addr: int
    """
    def __init__(self, bus: int, addr: int):
        """
        Initialize the magnetometer.

        :param bus: The I2C bus number.
        :type bus: int
        :param addr: The I2C address of the magnetometer.
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
        print(f"Magnetometer initialized on I2C bus {self.bus}, address {hex(self.addr)}")

    def _read_sensor(self, register_low: int, register_high: int) -> int:
        """
        Simulates reading a 16-bit raw sensor value from given low and high registers.

        In a real implementation, this would perform I2C reads.
        For now, it returns a placeholder value.

        :param register_low: The low byte register.
        :type register_low: int
        :param register_high: The high byte register.
        :type register_high: int
        :return: The raw 16-bit sensor value.
        :rtype: int
        """
        # Placeholder for I2C read:
        # low_byte = self.i2c_bus.read_byte_data(self.addr, register_low)
        # high_byte = self.i2c_bus.read_byte_data(self.addr, register_high)
        # value = (high_byte << 8) | low_byte
        # Simulate some values
        if register_low == 0x04: # Placeholder for BMM350 X LSB
            return 250
        elif register_low == 0x06: # Placeholder for BMM350 Y LSB
            return -300
        elif register_low == 0x08: # Placeholder for BMM350 Z LSB
            return 500
        return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis magnetometer value.

        :return: The raw X-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_x()``
        """
        # Placeholder registers for BMM350 X-axis data
        return self._read_sensor(0x04, 0x05)

    def get_y(self) -> int:
        """
        Get the raw Y-axis magnetometer value.

        :return: The raw Y-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_y()``
        """
        # Placeholder registers for BMM350 Y-axis data
        return self._read_sensor(0x06, 0x07)

    def get_z(self) -> int:
        """
        Get the raw Z-axis magnetometer value.

        :return: The raw Z-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_z()``
        """
        # Placeholder registers for BMM350 Z-axis data
        return self._read_sensor(0x08, 0x09)

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis magnetometer values.

        :return: A list containing [X, Y, Z] magnetometer values.
        :rtype: list[int]
        :example: ``mag.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]
