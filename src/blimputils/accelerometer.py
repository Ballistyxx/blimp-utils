"""
Accelerometer module for the blimputils library, specifically for BMI270.
"""

import time
from typing import List
from smbus2 import SMBus, i2c_msg

# Attempt to import BMI270 specific configurations and registers.
# These files (config_file.py, registers.py) are expected to be
# in the same directory as this file, or accessible via PYTHONPATH.
try:
    from .config_file import bmi270_config_file # Changed from .bmi270_config
    from .registers import (                     # Changed from .bmi270_registers
        CHIP_ID_ADDRESS, # BMI270_CHIP_ID, # BMI270_CHIP_ID is not in registers.py, it should be defined or imported elsewhere if needed
        CMD, PWR_CONF, PWR_CTRL,
        INIT_CTRL, INIT_ADDR_0, INIT_ADDR_1, INIT_DATA,
        INTERNAL_STATUS,
        ACC_CONF, ACC_RANGE,
        ACC_X_7_0, ACC_Y_7_0, ACC_Z_7_0, # LSB registers for raw data
        # Constants for configuration values from definitions.py might be needed here
        # ACC_ODR_100, ACC_BWP_NORMAL, ACC_RANGE_2G # These are in definitions.py
    )
    # It seems BMI270_CHIP_ID and other constants like ACC_ODR_100 might be better placed
    # in registers.py or a new bmi270_defs.py if they are specific to the chip's interface
    # For now, let's define BMI270_CHIP_ID here if it's static, or ensure it's in registers.py
    BMI270_CHIP_ID = 0x24 # Standard BMI270 Chip ID
    
    # Let's assume they will be imported from a definitions module or defined if not available from registers.py
    # If your registers.py doesn't have them, you might need to add them or import from definitions.py
    from .definitions import ACC_ODR_100, ACC_BWP_NORMAL, ACC_RANGE_2G

except ImportError as e:
    print("ERROR: Could not import config_file.py or registers.py, or specific constants from them.")
    print("Please ensure these files are present in the 'src/blimputils/' directory and contain the necessary variables.")
    print(f"Import error: {e}")
    raise

class Accelerometer:
    """
    A class to represent the BMI270 accelerometer.

    :param bus: The I2C bus number (e.g., 1 for Raspberry Pi).
    :type bus: int
    :param addr: The I2C address of the accelerometer (typically 0x68 or 0x69 for BMI270).
    :type addr: int
    """
    def __init__(self, bus: int, addr: int = 0x68): # Default BMI270 address
        if not isinstance(bus, int):
            raise TypeError("I2C bus must be an integer")
        if not isinstance(addr, int):
            raise TypeError("I2C address must be an integer")

        self.bus_num = bus
        self.addr = addr
        self.i2c_bus = None # Initialize to None

        try:
            self.i2c_bus = SMBus(self.bus_num)
        except Exception as e:
            raise IOError(f"Failed to open I2C bus {self.bus_num}: {e}")

        try:
            self._check_chip_id()
            self._load_config_file()
            self._configure_sensor()
            # Default G range for scaling, can be changed by re-configuring
            self.current_g_range = 2.0
            self.raw_to_g_factor = self.current_g_range / 32768.0

            print(f"Accelerometer (BMI270) initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")
        except Exception as e:
            if self.i2c_bus:
                self.i2c_bus.close()
            raise IOError(f"Failed to initialize BMI270 accelerometer: {e}")

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
            raise IOError(
                f"BMI270 not found at address {hex(self.addr)} on bus {self.bus_num}. "
                f"Chip ID: {hex(chip_id)}, Expected: {hex(BMI270_CHIP_ID)}"
            )
        print(f"BMI270 Chip ID {hex(chip_id)} verified.")

    def _load_config_file(self):
        """Loads the configuration firmware into the BMI270."""
        print("Loading BMI270 configuration file...")
        # Check if initialization is already done (e.g. after soft reset)
        # A status of 0x01 indicates POR detected and init sequence not running & no errors.
        # If already initialized, no need to reload config.
        # However, a simple power cycle would require re-init.
        # For robustness, we typically load it unless a specific state indicates it's loaded.
        # The reference BMI270.py checks if INTERNAL_STATUS is 0x01 and skips if so.
        # Let's assume we always try to load unless it's a warm start scenario not covered here.

        self._write_register(PWR_CONF, 0x00)  # Disable advanced power save during init
        time.sleep(0.00045)  # 450 microseconds delay

        self._write_register(INIT_CTRL, 0x00) # Prepare for config load

        # Load the configuration file in 32-byte chunks
        # The bmi270_config_file is 8192 bytes (256 chunks of 32 bytes)
        for i in range(256):
            chunk = bmi270_config_file[i*32 : (i+1)*32]
            
            # Set the address for the configuration data chunk
            self._write_register(INIT_ADDR_0, 0x00) # LSB of address for this chunk
            self._write_register(INIT_ADDR_1, i)    # MSB of address for this chunk
            
            # Write the 32-byte chunk to INIT_DATA register
            # smbus2 write_i2c_block_data expects a list of ints (bytes)
            write_msg = i2c_msg.write(self.addr, [INIT_DATA] + list(chunk))
            self.i2c_bus.i2c_rdwr(write_msg)
            time.sleep(0.000020) # 20 microseconds delay between writes

        self._write_register(INIT_CTRL, 0x01) # Finalize config load
        time.sleep(0.020) # 20 milliseconds delay for initialization to complete

        # Verify initialization
        status = self._read_register(INTERNAL_STATUS)
        # Expected status after successful init is 0x01 (POR detected, no errors, init not running)
        if status == 0x01:
            print("BMI270 configuration loaded successfully.")
        else:
            # Check for error bits (bits 2 and 3 of INTERNAL_STATUS)
            if status & 0x0C: # 0b00001100
                raise IOError(f"BMI270 configuration load failed: Error bits set in INTERNAL_STATUS (0x{status:02x})")
            elif status & 0x02: # 0b00000010 (init_running)
                 raise IOError(f"BMI270 configuration load failed: Stuck in init (INTERNAL_STATUS 0x{status:02x})")
            else:
                raise IOError(f"BMI270 configuration load failed: INTERNAL_STATUS is 0x{status:02x}, expected 0x01.")


    def _configure_sensor(self):
        """Configures accelerometer parameters (Power, ODR, Range)."""
        print("Configuring BMI270 accelerometer...")
        # 1. Set advanced power save mode (adv_power_save = 0 for normal/performance)
        # PWR_CONF register (0x7C), bit 0 is adv_power_save.
        # Set to 0x00 to ensure adv_power_save = 0. Other bits are reserved or for FIFO.
        self._write_register(PWR_CONF, 0x00)
        time.sleep(0.001) # Small delay

        # 2. Configure Accelerometer settings (ACC_CONF register 0x40)
        # Bits 0-3: acc_odr (Output Data Rate)
        # Bits 4-6: acc_bwp (Bandwidth Parameter)
        # Bit 7: acc_filter_perf (Accelerometer filter performance)
        # For 100Hz ODR, Normal BWP, Filter Performance Enabled:
        # ACC_ODR_100 (0x08) | (ACC_BWP_NORMAL (0x02) << 4) | (1 << 7)
        # 0x08 | 0x20 | 0x80 = 0xA8
        acc_conf_val = ACC_ODR_100 | (ACC_BWP_NORMAL << 4) | (1 << 7)
        self._write_register(ACC_CONF, acc_conf_val) # e.g., 0xA8
        time.sleep(0.001)

        # 3. Configure Accelerometer Range (ACC_RANGE register 0x41)
        # Bits 0-1: acc_range
        # ACC_RANGE_2G (0x00), ACC_RANGE_4G (0x01), ACC_RANGE_8G (0x02), ACC_RANGE_16G (0x03)
        self._write_register(ACC_RANGE, ACC_RANGE_2G) # Set to +/- 2G range
        self.current_g_range = 2.0 # Store for scaling
        self.raw_to_g_factor = self.current_g_range / 32768.0
        time.sleep(0.001)

        # 4. Enable Accelerometer (PWR_CTRL register 0x7D)
        # Bit 2: acc_en. Set to 1 to enable.
        # Other bits: aux_en (0), gyr_en (1), temp_en (3)
        # To enable only accelerometer: 0b00000100 = 0x04
        # Read current PWR_CTRL, set bit 2, write back to preserve other settings if any.
        # However, BMI270.py reference often writes 0x0E (acc,gyr,temp) then 0x02 to PWR_CONF.
        # For a dedicated accelerometer class, just enabling accelerometer is fine.
        pwr_ctrl_val = self._read_register(PWR_CTRL)
        self._write_register(PWR_CTRL, pwr_ctrl_val | 0x04) # Set acc_en bit
        time.sleep(0.050) # Allow sensor to stabilize (datasheet recommends 2ms after acc_en)

        print(f"BMI270 accelerometer configured: ODR=100Hz, Range=+/-{self.current_g_range}G.")

    def _read_sensor_word_signed(self, register_low: int) -> int:
        """
        Reads a 16-bit signed word (two bytes, little-endian) from the sensor.
        LSB is at register_low, MSB is at register_low + 1.
        """
        try:
            # Read 2 bytes starting from register_low
            read_msg = i2c_msg.write(self.addr, [register_low])
            read_msg_data = i2c_msg.read(self.addr, 2)
            self.i2c_bus.i2c_rdwr(read_msg, read_msg_data)
            
            low_byte = list(read_msg_data)[0]
            high_byte = list(read_msg_data)[1]
            
            value = (high_byte << 8) | low_byte
            
            # Convert to signed 16-bit integer
            if value & (1 << 15):  # Check if MSB (sign bit) is set
                value -= (1 << 16) # Compute two's complement
            return value
        except Exception as e:
            print(f"Error reading word from accelerometer: addr {hex(self.addr)}, reg_low {hex(register_low)}: {e}")
            return 0 # Or raise exception

    def get_x_raw(self) -> int:
        """
        Get the raw X-axis acceleration value.
        :return: The raw X-axis acceleration (16-bit signed integer).
        :example: ``raw_x = accel.get_x_raw()``
        """
        return self._read_sensor_word_signed(ACC_X_7_0)

    def get_y_raw(self) -> int:
        """
        Get the raw Y-axis acceleration value.
        :return: The raw Y-axis acceleration (16-bit signed integer).
        :example: ``raw_y = accel.get_y_raw()``
        """
        return self._read_sensor_word_signed(ACC_Y_7_0)

    def get_z_raw(self) -> int:
        """
        Get the raw Z-axis acceleration value.
        :return: The raw Z-axis acceleration (16-bit signed integer).
        :example: ``raw_z = accel.get_z_raw()``
        """
        return self._read_sensor_word_signed(ACC_Z_7_0)

    def get_xyz_raw(self) -> List[int]:
        """
        Get all three axes raw acceleration data.
        :return: A list containing [X, Y, Z] raw acceleration values.
        :rtype: List[int]
        :example: ``raw_xyz = accel.get_xyz_raw()``
        """
        # Reading all 6 data bytes at once might be more efficient if sensor supports it
        # and smbus2 block read is used correctly.
        # For now, individual reads are clear and map to existing methods.
        # BMI270 data registers are contiguous (0x0C to 0x11 for ACC_X/Y/Z LSB/MSB)
        try:
            data_bytes = self.i2c_bus.read_i2c_block_data(self.addr, ACC_X_7_0, 6)
            
            ax = (data_bytes[1] << 8) | data_bytes[0]
            ay = (data_bytes[3] << 8) | data_bytes[2]
            az = (data_bytes[5] << 8) | data_bytes[4]

            # Convert to signed
            if ax & (1 << 15): ax -= (1 << 16)
            if ay & (1 << 15): ay -= (1 << 16)
            if az & (1 << 15): az -= (1 << 16)
            
            return [ax, ay, az]
        except Exception as e:
            print(f"Error reading block data from accelerometer: {e}")
            # Fallback to individual reads if block read fails, or re-raise
            return [self.get_x_raw(), self.get_y_raw(), self.get_z_raw()]


    def get_xyz(self) -> List[float]:
        """
        Get all three axes acceleration data, scaled to G's.
        :return: A list containing [X, Y, Z] acceleration in G's.
        :rtype: List[float]
        :example: ``g_xyz = accel.get_xyz()``
        """
        raw = self.get_xyz_raw()
        return [val * self.raw_to_g_factor for val in raw]

    def get_x(self) -> float:
        """
        Get X-axis acceleration data, scaled to G's.
        :return: The X-axis acceleration in G's.
        :rtype: float
        :example: ``g_x = accel.get_x()``
        """
        raw = self.get_x_raw()
        return raw * self.raw_to_g_factor
    
    def get_y(self) -> float:
        """
        Get Y-axis acceleration data, scaled to G's.
        :return: The Y-axis acceleration in G's.
        :rtype: float
        :example: ``g_y = accel.get_y()``
        """
        raw = self.get_y_raw()
        return raw * self.raw_to_g_factor

    def get_z(self) -> float:
        """
        Get Z-axis acceleration data, scaled to G's.
        :return: The Z-axis acceleration in G's.
        :rtype: float
        :example: ``g_z = accel.get_z()``
        """
        raw = self.get_z_raw()
        return raw * self.raw_to_g_factor

    def close(self):
        """
        Clean up and close the I2C bus connection.
        It's important to call this when done with the sensor.
        """
        if self.i2c_bus:
            try:
                # Optional: Put sensor in a low power/suspend mode if applicable
                # For BMI270, disabling acc via PWR_CTRL could be an option:
                # current_pwr_ctrl = self._read_register(PWR_CTRL)
                # self._write_register(PWR_CTRL, current_pwr_ctrl & ~0x04) # Clear acc_en bit
                pass
            except Exception as e:
                print(f"Error during accelerometer shutdown: {e}")
            finally:
                self.i2c_bus.close()
                self.i2c_bus = None
                print(f"Accelerometer I2C bus {self.bus_num} closed.")

# Example usage (optional, for testing directly)
if __name__ == '__main__':
    try:
        # Ensure bmi270_config.py and bmi270_registers.py are in src/blimputils/
        # or that src/blimputils is in PYTHONPATH for the imports to work.
        # If running this directly, you might need to adjust sys.path or run as a module.
        # Example: python -m blimputils.accelerometer (if __init__.py makes it a package)
        
        # For direct script execution, relative imports might fail.
        # A temporary workaround for direct testing if files are in same dir:
        # import sys
        # import os
        # sys.path.append(os.path.dirname(__file__)) # Add current dir to path
        # from bmi270_config import bmi270_config_file # Now this might work
        # from bmi270_registers import *
        
        print("Attempting to initialize BMI270 Accelerometer...")
        # Default I2C bus for Raspberry Pi is 1. Address 0x68 or 0x69.
        accel = Accelerometer(bus=1, addr=0x68) 
        print("BMI270 Accelerometer initialized successfully.")

        for i in range(10):
            raw_data = accel.get_xyz_raw()
            scaled_data = accel.get_xyz()
            print(f"Raw Data: X={raw_data[0]}, Y={raw_data[1]}, Z={raw_data[2]}")
            print(f"Scaled Data (G): X={scaled_data[0]:.2f}, Y={scaled_data[1]:.2f}, Z={scaled_data[2]:.2f}")
            time.sleep(0.5)

    except ImportError:
        print("ImportError: Could not run example. Ensure BMI270 config/register/registers.py files are accessible.")
        print("Try copying them to src/blimputils/ and run as part of the package.")
    except IOError as e:
        print(f"IOError: {e}")
        print("Please check I2C connection, address, and that the sensor is a BMI270.")
        print("Ensure 'i2cdetect -y 1' shows the device at the specified address.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if 'accel' in locals() and accel.i2c_bus:
            accel.close()