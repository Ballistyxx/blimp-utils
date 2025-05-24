"""
Accelerometer module for the blimpcontrol library.
"""

from typing import List
from smbus2 import SMBus

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
            raise IOError(f"Failed to open I2C bus {self.bus_num}: {e}")
        print(f"Accelerometer initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")

    def _read_sensor_byte(self, register: int) -> int:
        """
        Reads a single byte from a given register.

        :param register: The register to read from.
        :type register: int
        :return: The byte value read from the sensor.
        :rtype: int
        """
        try:
            return self.i2c_bus.read_byte_data(self.addr, register)
        except Exception as e:
            print(f"Error reading byte from accelerometer: addr {hex(self.addr)}, reg {hex(register)}: {e}")
            # Depending on desired error handling, could raise exception or return default
            return 0 

    def _read_sensor_word(self, register_low: int, little_endian: bool = True) -> int:
        """
        Reads a 16-bit word (two bytes) from the sensor starting at register_low.
        Assumes LSB is at register_low and MSB is at register_low + 1 if little_endian is True.

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
                return (high_byte << 8) | low_byte
            else:
                return (low_byte << 8) | high_byte
        except Exception as e:
            print(f"Error reading word from accelerometer: addr {hex(self.addr)}, reg_low {hex(register_low)}: {e}")
            return 0

    def get_x(self) -> int:
        """
        Get the raw X-axis acceleration value.
        This is a placeholder and needs to be adapted to your specific accelerometer's registers.
        For BMI270, ACC_X_LSB is 0x0C and ACC_X_MSB is 0x0D.

        :return: The raw X-axis acceleration.
        :rtype: int
        :example: ``accel.get_x()``
        """
        # Example for BMI270: ACC_X_LSB (0x0C), ACC_X_MSB (0x0D)
        # Data is 16-bit signed, little-endian
        raw_x = self._read_sensor_word(0x0C, little_endian=True)
        # Convert to signed if necessary (e.g., if sensor returns two's complement)
        if raw_x & (1 << 15): # Check if MSB is set (negative number)
            raw_x -= (1 << 16)
        return raw_x

    def get_y(self) -> int:
        """
        Get the raw Y-axis acceleration value.
        Placeholder for specific accelerometer (e.g., BMI270: ACC_Y_LSB 0x0E, ACC_Y_MSB 0x0F).

        :return: The raw Y-axis acceleration.
        :rtype: int
        :example: ``accel.get_y()``
        """
        raw_y = self._read_sensor_word(0x0E, little_endian=True)
        if raw_y & (1 << 15):
            raw_y -= (1 << 16)
        return raw_y

    def get_z(self) -> int:
        """
        Get the raw Z-axis acceleration value.
        Placeholder for specific accelerometer (e.g., BMI270: ACC_Z_LSB 0x10, ACC_Z_MSB 0x11).

        :return: The raw Z-axis acceleration.
        :rtype: int
        :example: ``accel.get_z()``
        """
        raw_z = self._read_sensor_word(0x10, little_endian=True)
        if raw_z & (1 << 15):
            raw_z -= (1 << 16)
        return raw_z

    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis acceleration values.

        :return: A list containing [X, Y, Z] acceleration values.
        :rtype: list[int]
        :example: ``accel.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]

    def close(self) -> None:
        """
        Close the I2C bus connection.
        """
        if hasattr(self, 'i2c_bus') and self.i2c_bus:
            self.i2c_bus.close()
            print(f"I2C bus {self.bus_num} closed for accelerometer {hex(self.addr)}.")
