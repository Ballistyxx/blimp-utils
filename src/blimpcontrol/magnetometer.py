"""
Magnetometer module for the blimpcontrol library.
"""

from typing import List
from smbus2 import SMBus # Use smbus2 for I2C communication

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
            raise IOError(f"Failed to open I2C bus {self.bus_num} for magnetometer: {e}")
        print(f"Magnetometer initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")

    def _read_sensor_word(self, register_low: int, little_endian: bool = True) -> int:
        """
        Reads a 16-bit word (two bytes) from the sensor starting at register_low.
        This is a common pattern for many I2C sensors.

        :param register_low: The starting register (LSB).
        :type register_low: int
        :param little_endian: True if data is little-endian, False for big-endian.
        :type little_endian: bool
        :return: The 16-bit value read from the sensor.
        :rtype: int
        """
        try:
            low_byte = self.i2c_bus.read_byte_data(self.addr, register_low)
            high_byte = self.i2c_bus.read_byte_data(self.addr, register_low + 1) # Assuming MSB is next register
            
            if little_endian:
                val = (high_byte << 8) | low_byte
            else:
                val = (low_byte << 8) | high_byte

            # Convert to signed 16-bit integer if necessary (sensor dependent)
            # Example: if sensor returns two's complement for a 16-bit value
            if val & (1 << 15): # Check if MSB is set (negative number)
                val -= (1 << 16)
            return val
        except Exception as e:
            print(f"Error reading word from magnetometer: addr {hex(self.addr)}, reg_low {hex(register_low)}: {e}")
            return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis magnetometer value.
        Placeholder: Adjust registers for your specific magnetometer (e.g., BMM350).
        For BMM350, X LSB is 0x04, X MSB is 0x05.

        :return: The raw X-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_x()``
        """
        # Example for BMM350: X_LSB (0x04), X_MSB (0x05)
        return self._read_sensor_word(0x04, little_endian=True)

    def get_y(self) -> int:
        """
        Get the raw Y-axis magnetometer value.
        Placeholder: Adjust registers for your specific magnetometer (e.g., BMM350).
        For BMM350, Y LSB is 0x06, Y MSB is 0x07.

        :return: The raw Y-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_y()``
        """
        return self._read_sensor_word(0x06, little_endian=True)

    def get_z(self) -> int:
        """
        Get the raw Z-axis magnetometer value.
        Placeholder: Adjust registers for your specific magnetometer (e.g., BMM350).
        For BMM350, Z LSB is 0x08, Z MSB is 0x09.

        :return: The raw Z-axis magnetometer value.
        :rtype: int
        :example: ``mag.get_z()``
        """
        return self._read_sensor_word(0x08, little_endian=True)

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis magnetometer values.

        :return: A list containing [X, Y, Z] magnetometer values.
        :rtype: list[int]
        :example: ``mag.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]

    def close(self) -> None:
        """
        Close the I2C bus connection.
        """
        if hasattr(self, 'i2c_bus') and self.i2c_bus:
            self.i2c_bus.close()
            print(f"I2C bus {self.bus_num} closed for magnetometer {hex(self.addr)}.")
