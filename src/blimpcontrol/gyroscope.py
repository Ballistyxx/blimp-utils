"""
Gyroscope module for the blimpcontrol library, specifically for BMI270.
"""

import time
from typing import List
from smbus2 import SMBus # Use smbus2 for I2C communication

# BMI270 specific imports
try:
    from .config_file import bmi270_config_file
    from .registers import (
        CHIP_ID_ADDRESS, CMD, PWR_CONF, PWR_CTRL,
        INIT_CTRL, INIT_ADDR_0, INIT_ADDR_1, INIT_DATA,
        INTERNAL_STATUS,
        GYR_CONF, GYR_RANGE,
        GYR_X_7_0, GYR_Y_7_0, GYR_Z_7_0 # LSB registers for raw data
    )
    from .definitions import (
        GYR_ODR_100, GYR_ODR_200, GYR_ODR_400, GYR_ODR_800, GYR_ODR_1600, GYR_ODR_3200, # ODR options
        GYR_BWP_NORMAL, GYR_BWP_OSR2, GYR_BWP_OSR4, # Bandwidth options
        GYR_RANGE_2000, GYR_RANGE_1000, GYR_RANGE_500, GYR_RANGE_250, GYR_RANGE_125 # Range options
    )
except ImportError as e:
    print("ERROR: Could not import BMI270 specific files (config_file.py, registers.py, definitions.py).")
    print("Please ensure these files are present in the 'src/blimpcontrol/' directory and contain the necessary variables.")
    print(f"Import error: {e}")
    raise

BMI270_CHIP_ID = 0x24  # Standard BMI270 Chip ID
I2C_PRIM_ADDR = 0x68   # Default I2C primary address for BMI270

# Gyroscope range settings and their corresponding DPS values and sensitivities
_GYR_RANGE_MAP = {
    GYR_RANGE_2000: {"dps": 2000.0, "sensitivity_lsb_dps": 16.4},
    GYR_RANGE_1000: {"dps": 1000.0, "sensitivity_lsb_dps": 32.8},
    GYR_RANGE_500:  {"dps": 500.0,  "sensitivity_lsb_dps": 65.6},
    GYR_RANGE_250:  {"dps": 250.0,  "sensitivity_lsb_dps": 131.2},
    GYR_RANGE_125:  {"dps": 125.0,  "sensitivity_lsb_dps": 262.4}
}

class Gyroscope:
    """
    A class to represent the BMI270 gyroscope.

    :param bus: The I2C bus number (e.g., 1 for Raspberry Pi).
    :type bus: int
    :param addr: The I2C address of the BMI270 (typically 0x68 or 0x69).
    :type addr: int
    :param default_odr: Default Output Data Rate for the gyroscope, e.g., GYR_ODR_100.
    :type default_odr: int
    :param default_bwp: Default Bandwidth Parameter for the gyroscope, e.g., GYR_BWP_NORMAL.
    :type default_bwp: int
    :param default_range: Default range setting for the gyroscope, e.g., GYR_RANGE_2000.
    :type default_range: int
    :raises TypeError: if bus or addr are not integers.
    :raises IOError: if I2C bus cannot be opened or BMI270 initialization fails.
    """
    def __init__(self, bus: int, addr: int = I2C_PRIM_ADDR, 
                 default_odr: int = GYR_ODR_100,
                 default_bwp: int = GYR_BWP_NORMAL,
                 default_range: int = GYR_RANGE_2000):
        if not isinstance(bus, int):
            raise TypeError("I2C bus must be an integer")
        if not isinstance(addr, int):
            raise TypeError("I2C address must be an integer")

        self.bus_num = bus
        self.addr = addr
        self.i2c_bus = None  # Initialize to None

        self.default_gyr_odr = default_odr
        self.default_gyr_bwp = default_bwp
        self.default_gyr_range_setting = default_range
        
        self.current_dps_val = 0.0
        self.raw_to_dps_factor = 0.0

        try:
            self.i2c_bus = SMBus(self.bus_num)
        except Exception as e:
            raise IOError(f"Failed to open I2C bus {self.bus_num} for gyroscope: {e}")

        try:
            self._check_chip_id()
            self._load_config_file()
            self._configure_sensor(odr=self.default_gyr_odr, 
                                   bwp=self.default_gyr_bwp, 
                                   gyr_range_setting=self.default_gyr_range_setting)
            print(f"Gyroscope (BMI270) initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")
        except Exception as e:
            if self.i2c_bus:
                self.i2c_bus.close()
            raise IOError(f"Failed to initialize BMI270 gyroscope: {e}")

    def _write_register(self, register: int, value: int):
        """Helper to write a byte to a register."""
        self.i2c_bus.write_byte_data(self.addr, register, value)

    def _read_register(self, register: int) -> int:
        """Helper to read a byte from a register."""
        return self.i2c_bus.read_byte_data(self.addr, register)

    def _check_chip_id(self):
        """Verify that the device at the address is a BMI270."""
        chip_id = self._read_register(CHIP_ID_ADDRESS)
        if chip_id != BMI270_CHIP_ID:
            raise RuntimeError(f"BMI270 not found at address {hex(self.addr)}. Found Chip ID: {hex(chip_id)}")
        print(f"BMI270 Chip ID {hex(chip_id)} verified for gyroscope.")

    def _load_config_file(self):
        """Loads the configuration firmware into the BMI270."""
        print("Loading BMI270 configuration file for gyroscope...")
        self._write_register(PWR_CONF, 0x00)  # Disable Advanced Power Save (APS)
        time.sleep(0.00045)  # Wait 450us

        self._write_register(INIT_CTRL, 0x00)  # Prepare for config loading

        # Set initial address for config loading (0x0000)
        self._write_register(INIT_ADDR_0, 0x00)  # LSB of address
        self._write_register(INIT_ADDR_1, 0x00)  # MSB of address

        # Write the config file in bursts to INIT_DATA register
        # SMBus write_i2c_block_data typically has a max length (e.g., 32 bytes)
        max_burst_len = 32 
        for i in range(0, len(bmi270_config_file), max_burst_len):
            chunk = bmi270_config_file[i:i + max_burst_len]
            self.i2c_bus.write_i2c_block_data(self.addr, INIT_DATA, chunk)
            # A small delay might be needed if issues arise, but typically not for burst writes.

        self._write_register(INIT_CTRL, 0x01)  # Finalize config load
        
        time.sleep(0.150)  # Wait for initialization to complete (conservative, matches accelerometer)

        internal_status = self._read_register(INTERNAL_STATUS)
        # Check bit 0 (init_ok) of INTERNAL_STATUS (0x21)
        if not (internal_status & 0x01): # init_err is bit 1, init_ok is bit 0.
            # Bit 0 = 1 means initialization is OK.
            raise RuntimeError(f"BMI270 configuration loading failed for gyroscope. INTERNAL_STATUS: {hex(internal_status)}")
        print("BMI270 configuration file loaded successfully for gyroscope.")

    def _configure_sensor(self, odr: int, bwp: int, gyr_range_setting: int):
        """
        Configures the gyroscope's ODR, BWP, range, and enables it.

        :param odr: Output Data Rate setting (e.g., GYR_ODR_100).
        :type odr: int
        :param bwp: Bandwidth Parameter setting (e.g., GYR_BWP_NORMAL).
        :type bwp: int
        :param gyr_range_setting: Gyroscope range setting (e.g., GYR_RANGE_2000).
        :type gyr_range_setting: int
        """
        print(f"Configuring BMI270 gyroscope: ODR={hex(odr)}, BWP={hex(bwp)}, Range={hex(gyr_range_setting)}")
        
        # Enable Gyroscope
        # Read current PWR_CTRL, enable gyro (bit 1), keep other bits (e.g. acc_en bit 2)
        # This is safer if accelerometer might also be active.
        # However, to mirror accelerometer.py's direct write:
        # self._write_register(PWR_CTRL, 0x02) # Enable Gyro (0b00000010)
        # For now, let's enable only gyro, assuming it might be used standalone or
        # a higher-level IMU class would manage PWR_CTRL for combined operation.
        # If accelerometer.py writes 0x04, this write of 0x02 would disable accel.
        # A common approach is to enable both if both are expected:
        # self._write_register(PWR_CTRL, 0x06) # Enable Accel and Gyro
        # For now, strictly enabling Gyro:
        current_pwr_ctrl = self._read_register(PWR_CTRL)
        self._write_register(PWR_CTRL, current_pwr_ctrl | 0x02) # Set bit 1 (gyr_en)
        time.sleep(0.001) # Short delay after power mode change

        # Configure Gyroscope ODR and BWP
        # GYR_CONF (0x42): bits [3:0] odr, bits [5:4] bwp, bit 6 noise_perf, bit 7 filter_perf
        # Assuming standard performance (bits 7:6 = 00)
        gyr_conf_val = (odr & 0x0F) | ((bwp & 0x03) << 4)
        self._write_register(GYR_CONF, gyr_conf_val)
        time.sleep(0.001)

        # Configure Gyroscope Range
        self._write_register(GYR_RANGE, gyr_range_setting & 0x07) # Range is bits [2:0]
        time.sleep(0.001)

        # Update scaling factors
        self.current_gyr_range_setting = gyr_range_setting
        range_info = _GYR_RANGE_MAP.get(self.current_gyr_range_setting)
        if range_info:
            self.current_dps_val = range_info["dps"]
            # Sensitivity: LSB/dps. To get dps from raw: raw / sensitivity
            # Or: raw * (dps_range / 32768.0)
            self.raw_to_dps_factor = self.current_dps_val / 32768.0 
        else:
            raise ValueError(f"Invalid gyroscope range setting: {hex(gyr_range_setting)}")
        
        print(f"Gyroscope configured. Range: +/-{self.current_dps_val} dps. Factor: {self.raw_to_dps_factor}")

    def _read_sensor_word_signed(self, register_lsb: int) -> int:
        """
        Reads a 16-bit signed word (LSB first) from two consecutive registers.

        :param register_lsb: The starting register (LSB).
        :type register_lsb: int
        :return: The 16-bit signed value.
        :rtype: int
        """
        try:
            low_byte = self._read_register(register_lsb)
            high_byte = self._read_register(register_lsb + 1)
            val = (high_byte << 8) | low_byte
            if val & (1 << 15):  # Check if MSB is set (negative number)
                val -= (1 << 16)  # Convert to signed 16-bit
            return val
        except Exception as e:
            print(f"Error reading signed word from gyroscope: addr {hex(self.addr)}, reg_lsb {hex(register_lsb)}: {e}")
            return 0 # Return 0 on error, or raise exception

    def get_x_raw(self) -> int:
        """
        Get the raw X-axis gyroscope value from BMI270.
        Uses GYR_X_7_0 (LSB) and GYR_X_15_8 (MSB) registers.

        :return: The raw X-axis gyroscope value.
        :rtype: int
        :example: ``raw_x = gyro.get_x_raw()``
        """
        return self._read_sensor_word_signed(GYR_X_7_0)

    def get_y_raw(self) -> int:
        """
        Get the raw Y-axis gyroscope value from BMI270.
        Uses GYR_Y_7_0 (LSB) and GYR_Y_15_8 (MSB) registers.

        :return: The raw Y-axis gyroscope value.
        :rtype: int
        :example: ``raw_y = gyro.get_y_raw()``
        """
        return self._read_sensor_word_signed(GYR_Y_7_0)

    def get_z_raw(self) -> int:
        """
        Get the raw Z-axis gyroscope value from BMI270.
        Uses GYR_Z_7_0 (LSB) and GYR_Z_15_8 (MSB) registers.

        :return: The raw Z-axis gyroscope value.
        :rtype: int
        :example: ``raw_z = gyro.get_z_raw()``
        """
        return self._read_sensor_word_signed(GYR_Z_7_0)

    def get_xyz_raw(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis gyroscope values.

        :return: A list containing [X, Y, Z] raw gyroscope values.
        :rtype: list[int]
        :example: ``raw_xyz = gyro.get_xyz_raw()``
        """
        return [self.get_x_raw(), self.get_y_raw(), self.get_z_raw()]

    def get_x(self) -> float:
        """
        Get the X-axis gyroscope value scaled to degrees per second (dps).

        :return: The X-axis gyroscope value in dps.
        :rtype: float
        :example: ``dps_x = gyro.get_x()``
        """
        return self.get_x_raw() * self.raw_to_dps_factor

    def get_y(self) -> float:
        """
        Get the Y-axis gyroscope value scaled to degrees per second (dps).

        :return: The Y-axis gyroscope value in dps.
        :rtype: float
        :example: ``dps_y = gyro.get_y()``
        """
        return self.get_y_raw() * self.raw_to_dps_factor

    def get_z(self) -> float:
        """
        Get the Z-axis gyroscope value scaled to degrees per second (dps).

        :return: The Z-axis gyroscope value in dps.
        :rtype: float
        :example: ``dps_z = gyro.get_z()``
        """
        return self.get_z_raw() * self.raw_to_dps_factor

    def get_xyz(self) -> List[float]:
        """
        Get the X, Y, and Z-axis gyroscope values scaled to degrees per second (dps).

        :return: A list containing [X, Y, Z] gyroscope values in dps.
        :rtype: list[float]
        :example: ``dps_xyz = gyro.get_xyz()``
        """
        return [self.get_x(), self.get_y(), self.get_z()]

    def close(self) -> None:
        """
        Close the I2C bus connection.
        Also attempts to disable the gyroscope if it was enabled by this instance.
        """
        if hasattr(self, 'i2c_bus') and self.i2c_bus:
            try:
                # Attempt to disable gyroscope: clear bit 1 of PWR_CTRL
                if self._read_register(CHIP_ID_ADDRESS) == BMI270_CHIP_ID: # Check if still connected
                    current_pwr_ctrl = self._read_register(PWR_CTRL)
                    self._write_register(PWR_CTRL, current_pwr_ctrl & ~0x02) # Clear gyr_en bit
            except Exception as e:
                print(f"Warning: Could not disable gyroscope on close: {e}")
            finally:
                self.i2c_bus.close()
                print(f"I2C bus {self.bus_num} closed for gyroscope {hex(self.addr)}.")

# Example usage (for testing, typically not part of the library file):
if __name__ == '__main__':
    try:
        # Assuming I2C bus 1, default address 0x68
        # You might need to change the bus number depending on your Raspberry Pi version and setup
        gyro = Gyroscope(bus=1, addr=0x68) 
        print("Gyroscope (BMI270) initialized successfully.")
        
        for _ in range(10):
            raw_xyz = gyro.get_xyz_raw()
            scaled_xyz = gyro.get_xyz()
            print(f"Raw Gyro (X,Y,Z): {raw_xyz}")
            print(f"Scaled Gyro (X,Y,Z) [dps]: {scaled_xyz[0]:.2f}, {scaled_xyz[1]:.2f}, {scaled_xyz[2]:.2f}")
            time.sleep(0.5)
            
    except IOError as e:
        print(f"I/O error: {e}")
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if 'gyro' in locals() and gyro:
            gyro.close()
