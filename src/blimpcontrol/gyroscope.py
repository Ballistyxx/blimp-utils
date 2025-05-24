"""
Gyroscope module for the blimpcontrol library.
"""

from typing import List
from smbus2 import SMBus # Use smbus2 for I2C communication

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
        :raises IOError: if I2C bus cannot be opened.
        """
        if not isinstance(bus, int):
            raise TypeError("I2C bus must be an integer")
        if not isinstance(addr, int):
            raise TypeError("I2C address must be an integer")
        self.bus_num = bus
        self.addr = addr
        try:
            self.i2c_bus = SMBus(self.bus_num)
        except Exception as e:
            raise IOError(f"Failed to open I2C bus {self.bus_num} for gyroscope: {e}")
        print(f"Gyroscope initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")

    def _read_sensor_word(self, register_low: int, little_endian: bool = True) -> int:
        """
        Reads a 16-bit word (two bytes) from the sensor starting at register_low.

        :param register_low: The starting register (LSB).
        :type register_low: int
        :param little_endian: True if data is little-endian, False for big-endian.
        :type little_endian: bool
        :return: The 16-bit value read from the sensor.
        :rtype: int
        """
        try:
            low_byte = self.i2c_bus.read_byte_data(self.addr, register_low)
            high_byte = self.i2c_bus.read_byte_data(self.addr, register_low + 1)
            if little_endian:
                val = (high_byte << 8) | low_byte
            else:
                val = (low_byte << 8) | high_byte
            
            # Convert to signed 16-bit integer
            if val & (1 << 15): # Check if MSB is set (negative number)
                val -= (1 << 16)
            return val
        except Exception as e:
            print(f"Error reading word from gyroscope: addr {hex(self.addr)}, reg_low {hex(register_low)}: {e}")
            return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis gyroscope value.
        Placeholder for specific gyroscope (e.g., BMI270: GYR_X_LSB 0x12, GYR_X_MSB 0x13).

        :return: The raw X-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_x()``
        """
        # Example for BMI270: GYR_X_LSB (0x12), GYR_X_MSB (0x13)
        return self._read_sensor_word(0x12, little_endian=True)

    def get_y(self) -> int:
        """
        Get the raw Y-axis gyroscope value.
        Placeholder for specific gyroscope (e.g., BMI270: GYR_Y_LSB 0x14, GYR_Y_MSB 0x15).

        :return: The raw Y-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_y()``
        """
        return self._read_sensor_word(0x14, little_endian=True)

    def get_z(self) -> int:
        """
        Get the raw Z-axis gyroscope value.
        Placeholder for specific gyroscope (e.g., BMI270: GYR_Z_LSB 0x16, GYR_Z_MSB 0x17).

        :return: The raw Z-axis gyroscope value.
        :rtype: int
        :example: ``gyro.get_z()``
        """
        return self._read_sensor_word(0x16, little_endian=True)

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis gyroscope values.

        :return: A list containing [X, Y, Z] gyroscope values.
        :rtype: list[int]
        :example: ``gyro.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]

    def close(self) -> None:
        """
        Close the I2C bus connection.
        """
        if hasattr(self, 'i2c_bus') and self.i2c_bus:
            self.i2c_bus.close()
            print(f"I2C bus {self.bus_num} closed for gyroscope {hex(self.addr)}.")
