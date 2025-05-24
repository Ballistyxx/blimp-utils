"""
Magnetometer module for the blimpcontrol library, specifically for BMM350.
"""

import time
from typing import List, Tuple
from smbus2 import SMBus

# BMM350 Default I2C Address
BMM350_I2C_ADDR_DEFAULT = 0x14

# BMM350 Chip ID
BMM350_CHIP_ID_VAL = 0x33

# Register Addresses
BMM350_REG_CHIP_ID = 0x00
BMM350_REG_ERR_REG = 0x02
BMM350_REG_PMU_CMD_AGGR_SET = 0x04
BMM350_REG_PMU_CMD_AXIS_EN = 0x05
BMM350_REG_PMU_CMD = 0x06
BMM350_REG_MAG_X_XLSB = 0x31 # XLSB (bits 3:0), LSB (bits 11:4), MSB (bits 19:12)
BMM350_REG_MAG_Y_XLSB = 0x34
BMM350_REG_MAG_Z_XLSB = 0x37
BMM350_REG_OTP_CMD_REG = 0x50
BMM350_REG_OTP_DATA_MSB_REG = 0x52 # OTP data is read as 16 words (32 bytes)
BMM350_REG_OTP_DATA_LSB_REG = 0x53
BMM350_REG_OTP_STATUS_REG = 0x55
BMM350_REG_CTRL_USER = 0x61

# Commands
BMM350_CMD_SOFTRESET = 0xB6
BMM350_PMU_CMD_NM = 0x01      # Normal Mode
BMM350_PMU_CMD_SUS = 0x00     # Suspend Mode
BMM350_OTP_CMD_READ_PAGE0 = 0x0A # Read OTP Page 0 (first 16 bytes)
BMM350_OTP_CMD_READ_PAGE1 = 0x0B # Read OTP Page 1 (next 16 bytes)

# Settings
BMM350_AXIS_EN_XYZ = 0x07 # Enable X, Y, Z axes
BMM350_AXIS_DIS_XYZ = 0x00 # Disable X, Y, Z axes

# ODR settings (bits 3:0 of PMU_CMD_AGGR_SET)
BMM350_ODR_100HZ = 0x04
BMM350_ODR_50HZ  = 0x05
BMM350_ODR_25HZ  = 0x06
BMM350_ODR_12_5HZ= 0x07 # Default

# AVG settings (bits 5:4 of PMU_CMD_AGGR_SET)
BMM350_AVG_NO_AVG = 0x00 # No averaging
BMM350_AVG_2 = 0x01      # 2 times averaging
BMM350_AVG_4 = 0x02      # 4 times averaging
BMM350_AVG_8 = 0x03      # 8 times averaging

class Magnetometer:
    """
    A class to represent the BMM350 magnetometer.

    :param bus: The I2C bus number (e.g., 1 for Raspberry Pi).
    :type bus: int
    :param addr: The I2C address of the BMM350 (default 0x14).
    :type addr: int
    :param odr: Output Data Rate, e.g., BMM350_ODR_100HZ.
    :type odr: int
    :param avg: Averaging setting, e.g., BMM350_AVG_NO_AVG.
    :type avg: int
    :raises TypeError: if bus or addr are not integers.
    :raises IOError: if I2C bus cannot be opened or BMM350 initialization fails.
    """
    def __init__(self, bus: int, addr: int = BMM350_I2C_ADDR_DEFAULT, 
                 odr: int = BMM350_ODR_100HZ, avg: int = BMM350_AVG_NO_AVG):
        if not isinstance(bus, int):
            raise TypeError("I2C bus must be an integer")
        if not isinstance(addr, int):
            raise TypeError("I2C address must be an integer")

        self.bus_num = bus
        self.addr = addr
        self.i2c_bus = None

        # Compensation coefficients (will be populated from OTP)
        self._sens_x_comp = 1.0
        self._sens_y_comp = 1.0
        self._sens_z_comp = 1.0
        self._cross_x_y_comp = 0.0
        self._cross_x_z_comp = 0.0
        self._cross_y_x_comp = 0.0
        self._cross_y_z_comp = 0.0
        self._cross_z_x_comp = 0.0
        self._cross_z_y_comp = 0.0
        self._t_offset_x_comp = 0.0
        self._t_offset_y_comp = 0.0
        self._t_offset_z_comp = 0.0

        try:
            self.i2c_bus = SMBus(self.bus_num)
        except Exception as e:
            raise IOError(f"Failed to open I2C bus {self.bus_num} for magnetometer: {e}")

        try:
            self._check_chip_id()
            self._soft_reset()
            self._read_and_parse_otp()
            self._set_power_mode(BMM350_PMU_CMD_NM) # Set to Normal Mode
            self._configure_odr_avg(odr, avg)
            self._enable_axes(True, True, True)
            print(f"Magnetometer (BMM350) initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")
        except Exception as e:
            if self.i2c_bus:
                self.i2c_bus.close()
            raise IOError(f"Failed to initialize BMM350 magnetometer: {e}")

    def _write_register(self, register: int, value: int):
        self.i2c_bus.write_byte_data(self.addr, register, value)

    def _read_register(self, register: int) -> int:
        return self.i2c_bus.read_byte_data(self.addr, register)

    def _read_block_data(self, register: int, length: int) -> List[int]:
        return self.i2c_bus.read_i2c_block_data(self.addr, register, length)

    def _check_chip_id(self):
        chip_id = self._read_register(BMM350_REG_CHIP_ID)
        if chip_id != BMM350_CHIP_ID_VAL:
            raise RuntimeError(f"BMM350 not found at address {hex(self.addr)}. Found Chip ID: {hex(chip_id)}")
        print(f"BMM350 Chip ID {hex(chip_id)} verified.")

    def _soft_reset(self):
        print("Performing BMM350 soft reset...")
        self._write_register(BMM350_REG_CTRL_USER, BMM350_CMD_SOFTRESET)
        time.sleep(0.025) # Datasheet: 24ms for soft reset
        # After reset, PMU_CMD_STATUS_0 should be 0x10 (POR detected)
        status = self._read_register(0x07) # PMU_CMD_STATUS_0
        if status != 0x10:
            print(f"Warning: BMM350 status after reset is {hex(status)}, expected 0x10.")
        print("BMM350 soft reset complete.")

    def _set_power_mode(self, mode: int):
        self._write_register(BMM350_REG_PMU_CMD, mode)
        if mode == BMM350_PMU_CMD_NM:
            time.sleep(0.038) # Suspend to Normal delay
        elif mode == BMM350_PMU_CMD_SUS:
            time.sleep(0.006) # Go to Suspend delay

    def _configure_odr_avg(self, odr: int, avg: int):
        val = (odr & 0x0F) | ((avg & 0x03) << 4)
        self._write_register(BMM350_REG_PMU_CMD_AGGR_SET, val)
        time.sleep(0.001)

    def _enable_axes(self, en_x: bool, en_y: bool, en_z: bool):
        val = 0
        if en_x: val |= 0x01
        if en_y: val |= 0x02
        if en_z: val |= 0x04
        self._write_register(BMM350_REG_PMU_CMD_AXIS_EN, val)
        time.sleep(0.001)

    def _unpack_otp_s12(self, lsb: int, msb: int) -> int:
        val = (msb << 8) | lsb
        val &= 0x0FFF # 12-bit value
        if val & (1 << 11): # Sign bit for 12-bit
            val -= (1 << 12)
        return val

    def _unpack_otp_u10(self, lsb: int, msb: int) -> int:
        val = (msb << 8) | lsb
        return val & 0x03FF # 10-bit unsigned
    
    def _unpack_otp_s8(self, byte_val: int) -> int:
        val = byte_val
        if val & (1 << 7):
            val -= (1 << 8)
        return val

    def _read_and_parse_otp(self):
        print("Reading BMM350 OTP data...")
        otp_data_u8 = [0] * 32

        # Read Page 0 (16 bytes)
        self._write_register(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_READ_PAGE0)
        time.sleep(0.001) # Wait for OTP read to prepare
        for i in range(8): # Read 8 words (16 bytes)
            lsb = self._read_register(BMM350_REG_OTP_DATA_LSB_REG)
            msb = self._read_register(BMM350_REG_OTP_DATA_MSB_REG)
            otp_data_u8[i*2] = lsb
            otp_data_u8[i*2+1] = msb
            time.sleep(0.0001) # Short delay between reads
        
        # Read Page 1 (16 bytes)
        self._write_register(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_READ_PAGE1)
        time.sleep(0.001)
        for i in range(8): # Read 8 words (16 bytes)
            lsb = self._read_register(BMM350_REG_OTP_DATA_LSB_REG)
            msb = self._read_register(BMM350_REG_OTP_DATA_MSB_REG)
            otp_data_u8[16 + i*2] = lsb
            otp_data_u8[16 + i*2+1] = msb
            time.sleep(0.0001)

        # Check OTP status - bit 0 should be 1 (OTP_OK)
        otp_status = self._read_register(BMM350_REG_OTP_STATUS_REG)
        if not (otp_status & 0x01):
            raise RuntimeError(f"BMM350 OTP read failed or data invalid. Status: {hex(otp_status)}")
        
        print("BMM350 OTP data read successfully. Parsing...")

        # Parse OTP data into compensation coefficients
        # Addresses are byte indices into otp_data_u8
        sens_x = self._unpack_otp_s12(otp_data_u8[0], otp_data_u8[1])
        sens_y = self._unpack_otp_s12(otp_data_u8[2], otp_data_u8[3])
        sens_z = self._unpack_otp_s12(otp_data_u8[4], otp_data_u8[5])

        cross_x_y = self._unpack_otp_s8(otp_data_u8[6])
        cross_x_z = self._unpack_otp_s8(otp_data_u8[7])
        cross_y_x = self._unpack_otp_s8(otp_data_u8[8])
        cross_y_z = self._unpack_otp_s8(otp_data_u8[9])
        cross_z_x = self._unpack_otp_s8(otp_data_u8[10])
        cross_z_y = self._unpack_otp_s8(otp_data_u8[11])

        # t_dep_x/y/z are not used in the simplified float compensation used here
        # t_dep_x = self._unpack_otp_u10(otp_data_u8[12], otp_data_u8[13])
        # t_dep_y = self._unpack_otp_u10(otp_data_u8[14], otp_data_u8[15])
        # t_dep_z = self._unpack_otp_u10(otp_data_u8[16], otp_data_u8[17])

        t_comp_off_x = self._unpack_otp_s12(otp_data_u8[18], otp_data_u8[19])
        t_comp_off_y = self._unpack_otp_s12(otp_data_u8[20], otp_data_u8[21])
        t_comp_off_z = self._unpack_otp_s12(otp_data_u8[22], otp_data_u8[23])

        # Store compensated factors (division by 131072.0 for sens/cross, 256.0 for t_offset)
        self._sens_x_comp = sens_x / 131072.0
        self._sens_y_comp = sens_y / 131072.0
        self._sens_z_comp = sens_z / 131072.0
        self._cross_x_y_comp = cross_x_y / 131072.0
        self._cross_x_z_comp = cross_x_z / 131072.0
        self._cross_y_x_comp = cross_y_x / 131072.0
        self._cross_y_z_comp = cross_y_z / 131072.0
        self._cross_z_x_comp = cross_z_x / 131072.0
        self._cross_z_y_comp = cross_z_y / 131072.0
        self._t_offset_x_comp = t_comp_off_x / 256.0
        self._t_offset_y_comp = t_comp_off_y / 256.0
        self._t_offset_z_comp = t_comp_off_z / 256.0
        print("BMM350 OTP data parsed and compensation factors calculated.")

    def _unpack_s20(self, xlsb_byte: int, lsb_byte: int, msb_byte: int) -> int:
        """Unpacks 3 bytes into a 20-bit signed integer."""
        # XLSB contains lower 4 bits (0-3) of the 20-bit data
        # LSB contains middle 8 bits (4-11)
        # MSB contains upper 8 bits (12-19)
        val = (msb_byte << 12) | (lsb_byte << 4) | (xlsb_byte & 0x0F)
        if val & (1 << 19):  # Check 20th bit (sign bit)
            val -= (1 << 20) # Convert to signed 20-bit
        return val

    def get_xyz_raw(self) -> Tuple[int, int, int]:
        """
        Get the raw X, Y, and Z-axis magnetometer values (20-bit signed).

        :return: A tuple containing (X_raw, Y_raw, Z_raw) magnetometer values.
        :rtype: tuple[int, int, int]
        :example: ``raw_x, raw_y, raw_z = mag.get_xyz_raw()``
        """
        try:
            # Read 9 bytes: X(xlsb,lsb,msb), Y(xlsb,lsb,msb), Z(xlsb,lsb,msb)
            data = self._read_block_data(BMM350_REG_MAG_X_XLSB, 9)
            
            raw_x = self._unpack_s20(data[0], data[1], data[2])
            raw_y = self._unpack_s20(data[3], data[4], data[5])
            raw_z = self._unpack_s20(data[6], data[7], data[8])
            
            return raw_x, raw_y, raw_z
        except Exception as e:
            print(f"Error reading raw XYZ from magnetometer: {e}")
            return 0, 0, 0

    def get_x_raw(self) -> int:
        """Get the raw X-axis magnetometer value."""
        return self.get_xyz_raw()[0]

    def get_y_raw(self) -> int:
        """Get the raw Y-axis magnetometer value."""
        return self.get_xyz_raw()[1]

    def get_z_raw(self) -> int:
        """Get the raw Z-axis magnetometer value."""
        return self.get_xyz_raw()[2]

    def get_xyz(self) -> Tuple[float, float, float]:
        """
        Get the compensated X, Y, and Z-axis magnetometer values in microTesla (uT).

        :return: A tuple containing (X_uT, Y_uT, Z_uT) magnetometer values.
        :rtype: tuple[float, float, float]
        :example: ``x_uT, y_uT, z_uT = mag.get_xyz()``
        """
        raw_x, raw_y, raw_z = self.get_xyz_raw()

        # Apply compensation
        # comp_val = raw_main_axis * sens_main + raw_cross_axis1 * cross1 + raw_cross_axis2 * cross2 + t_offset
        comp_x = (raw_x * self._sens_x_comp + 
                    raw_y * self._cross_x_y_comp + 
                    raw_z * self._cross_x_z_comp + 
                    self._t_offset_x_comp)
        
        comp_y = (raw_y * self._sens_y_comp + 
                    raw_x * self._cross_y_x_comp + 
                    raw_z * self._cross_y_z_comp + 
                    self._t_offset_y_comp)

        comp_z = (raw_z * self._sens_z_comp + 
                    raw_x * self._cross_z_x_comp + 
                    raw_y * self._cross_z_y_comp + 
                    self._t_offset_z_comp)
        
        return comp_x, comp_y, comp_z

    def get_x(self) -> float:
        """Get the compensated X-axis magnetometer value in uT."""
        return self.get_xyz()[0]

    def get_y(self) -> float:
        """Get the compensated Y-axis magnetometer value in uT."""
        return self.get_xyz()[1]

    def get_z(self) -> float:
        """Get the compensated Z-axis magnetometer value in uT."""
        return self.get_xyz()[2]

    def close(self) -> None:
        """
        Close the I2C bus connection and attempt to put sensor in suspend mode.
        """
        if hasattr(self, 'i2c_bus') and self.i2c_bus:
            try:
                print("Putting BMM350 into suspend mode...")
                self._set_power_mode(BMM350_PMU_CMD_SUS)
            except Exception as e:
                print(f"Warning: Could not set BMM350 to suspend mode on close: {e}")
            finally:
                self.i2c_bus.close()
                print(f"I2C bus {self.bus_num} closed for magnetometer {hex(self.addr)}.")

# Example usage:
if __name__ == '__main__':
    try:
        mag = Magnetometer(bus=1) # Assumes BMM350 on I2C bus 1, address 0x14
        print("BMM350 Magnetometer initialized successfully.")
        
        for i in range(20):
            raw_x, raw_y, raw_z = mag.get_xyz_raw()
            print(f"Raw Data: X={raw_x}, Y={raw_y}, Z={raw_z}")
            
            comp_x, comp_y, comp_z = mag.get_xyz()
            print(f"Compensated Data (uT): X={comp_x:.2f}, Y={comp_y:.2f}, Z={comp_z:.2f}")
            
            time.sleep(0.5)
            
    except IOError as e:
        print(f"I/O error: {e}")
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if 'mag' in locals() and mag:
            mag.close()
