"""
Magnetometer module for the blimpcontrol library, specifically for BMM350.
"""

import time
from typing import List, Tuple
from smbus2 import SMBus
from .magnetometer_vars import *

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
        self.i2c_bus = None # Initialize to None

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
            time.sleep(BMM350_START_UP_TIME_FROM_POR) # Use constant from magnetometer_vars.py
        except FileNotFoundError:
            # This error occurs if the I2C bus device file (e.g., /dev/i2c-1) doesn't exist.
            raise IOError(
                f"Failed to open I2C bus {self.bus_num}. "
                "Ensure I2C is enabled on your Raspberry Pi (e.g., via raspi-config) "
                f"and that bus number '{self.bus_num}' is correct."
            )
        except Exception as e:
            # Catch other potential errors from SMBus constructor
            raise IOError(f"Error opening I2C bus {self.bus_num} for BMM350 magnetometer: {e}")

        try:
            # Removed initial time.sleep(0.002) as BMM350_START_UP_TIME_FROM_POR is used after SMBus open.
            self._perform_full_init_sequence(odr, avg)
            print(f"Magnetometer (BMM350) initialized on I2C bus {self.bus_num}, address {hex(self.addr)}")
        
        except (IOError, RuntimeError) as e: # Catch specific errors from init sequence
            if self.i2c_bus:
                try:
                    self.i2c_bus.close()
                except Exception as close_e:
                    print(f"Additionally, failed to close I2C bus during error handling: {close_e}")
            # Re-raise the original error with context
            # The detailed_msg formatting is already handled in the previous version for IOError
            if isinstance(e, RuntimeError) or "BMM350 not found" in str(e) or "OTP read failed" in str(e):
                 raise IOError(f"Failed to initialize BMM350 magnetometer due to a runtime configuration issue: {e}") from e
            else: # For other IOErrors like Errno 121
                detailed_msg = (
                    f"I2C communication error with BMM350 at address {hex(self.addr)} on bus {self.bus_num}. "
                    f"Original error: {type(e).__name__}: {e}.\n"
                    "This often indicates a hardware issue. Please check the following:\n"
                    "1. Wiring: Ensure SDA, SCL, VCC, and GND are correctly and securely connected.\n"
                    "2. Power: Verify the sensor is receiving the correct voltage and is powered on.\n"
                    f"3. I2C Address: Confirm the sensor's address is {hex(self.addr)}.\n"
                    "4. Pull-up Resistors: Check if necessary.\n"
                    f"5. I2C Bus Health: Run `sudo i2cdetect -y {self.bus_num}`."
                )
                raise IOError(detailed_msg) from e

        except Exception as e: # Catch any other unexpected errors
            if self.i2c_bus:
                try:
                    self.i2c_bus.close()
                except Exception as close_e:
                    print(f"Additionally, failed to close I2C bus during error handling: {close_e}")
            raise IOError(f"An unexpected error occurred during BMM350 magnetometer initialization: {e}") from e

    def _perform_full_init_sequence(self, odr: int, avg: int):
        """
        Performs the full BMM350 initialization sequence based on DFRobot's library.
        """
        # 1. Software reset
        print("Performing BMM350 soft reset...")
        self._write_register(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)
        time.sleep(BMM350_SOFT_RESET_DELAY) # Use constant
        print("BMM350 soft reset command sent.")

        # 1.1 Verify POR (Power-On Reset) status and check for boot-up errors
        print("Verifying Power-On Reset status and checking for boot errors...")
        try:
            pmu_status = self._read_register(BMM350_REG_PMU_CMD_STATUS_0)
            # Bit 4 (0x10) indicates POR_DETECTED in PMU_CMD_STATUS_0 register
            if not (pmu_status & (1 << BMM350_POR_DETECTED_POS)): # Assuming BMM350_POR_DETECTED_POS = 4
                raise RuntimeError(
                    f"BMM350 Power-On Reset not detected after soft reset. "
                    f"PMU_CMD_STATUS_0: {hex(pmu_status)}. Expected bit 4 (0x10) to be set."
                )
            print(f"BMM350 POR detected (PMU_CMD_STATUS_0: {hex(pmu_status)}).")

            err_reg_val = self._read_register(BMM350_REG_ERR_REG)
            if err_reg_val & BMM350_BOOT_UP_ERROR_MSK: # Check boot_up_error bit
                raise RuntimeError(
                    f"BMM350 boot up error detected. ERR_REG: {hex(err_reg_val)}"
                )
            print(f"BMM350 ERR_REG check passed (ERR_REG: {hex(err_reg_val)}).")

        except IOError as e:
            raise IOError(f"Failed to read status/error registers after soft reset: {e}") from e
        
        # 2. Check Chip ID (with retries)
        self._check_chip_id() # This will raise RuntimeError if it fails

        # 3. Disable OTP / Power off OTP (as per DFRobot)
        print("Disabling OTP access...")
        self._write_register(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP) # Write 0x00 to 0x50
        time.sleep(0.001) # DFRobot lib doesn't specify a named delay here, 1ms is a common short delay

        # 4. Magnetic Reset
        self._magnetic_reset()

        # 5. Set to Suspend Mode (before OTP read)
        print("Setting to Suspend Mode before OTP read...")
        self._set_power_mode(BMM350_PMU_CMD_SUS)

        # 6. Read and Parse OTP
        self._read_and_parse_otp() # This will raise RuntimeError on OTP failure

        # 7. Set to Normal Mode
        print("Setting to Normal Mode...")
        self._set_power_mode(BMM350_PMU_CMD_NM)

        # 8. Configure ODR and AVG
        print(f"Configuring ODR={hex(odr)}, AVG={hex(avg)}...")
        self._configure_odr_avg(odr, avg) # Uses BMM350_UPD_OAE_DELAY internally

        # 9. Enable Axes
        print("Enabling X, Y, Z axes...")
        self._enable_axes(True, True, True) # Uses BMM350_UPD_OAE_DELAY internally
        

    def _magnetic_reset(self):
        """
        Performs the magnetic reset sequence.
        Ensures device is in suspend mode before issuing BR/FGR commands.
        """
        print("Performing BMM350 magnetic reset...")
        
        print("Setting to Suspend Mode for magnetic reset...")
        self._set_power_mode(BMM350_PMU_CMD_SUS) 

        self._write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)
        time.sleep(BMM350_BR_DELAY) # Use constant
        self._write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)
        time.sleep(BMM350_FGR_DELAY) # Use constant
        print("BMM350 magnetic reset complete.")

    def _write_register(self, register: int, value: int):
        self.i2c_bus.write_byte_data(self.addr, register, value)

    def _read_register(self, register: int) -> int:
        return self.i2c_bus.read_byte_data(self.addr, register)

    def _read_block_data(self, register: int, length: int) -> List[int]:
        return self.i2c_bus.read_i2c_block_data(self.addr, register, length)

    def _check_chip_id(self):
        retries = 3
        last_chip_id_read = -1 # Variable to store the last read chip ID
        for i in range(retries):
            try:
                chip_id = self._read_register(BMM350_REG_CHIP_ID)
                last_chip_id_read = chip_id # Store the read chip_id
                if chip_id == BMM350_CHIP_ID_VAL:
                    print(f"BMM350 Chip ID {hex(chip_id)} verified.")
                    return
                else:
                    print(f"Attempt {i+1}/{retries}: Incorrect Chip ID read: {hex(chip_id)}, expected {hex(BMM350_CHIP_ID_VAL)}.")
            except IOError as e:
                print(f"Attempt {i+1}/{retries}: Error reading chip ID: {e}")
                last_chip_id_read = -1 # Indicate error during read for this attempt
            
            if i < retries - 1: # Don't sleep after the last attempt
                time.sleep(BMM350_SOFT_RESET_DELAY / 2) # Use a fraction of soft reset delay between retries

        # If all retries fail, raise the error with the last read chip_id
        error_message_detail = f"Found Chip ID: {hex(last_chip_id_read) if last_chip_id_read != -1 else 'Error during read'}"
        raise RuntimeError(f"BMM350 not found at address {hex(self.addr)}. {error_message_detail} after {retries} attempts.")

    def _wait_pmu_busy(self, timeout_us: int = 100000): # Increased timeout to 100ms
        """
        Waits for the PMU to not be busy by checking the PMU_CMD_BUSY bit.
        Raises RuntimeError if timeout is reached.
        """
        print("Waiting for PMU to not be busy...")
        start_time = time.perf_counter()
        while (time.perf_counter() - start_time) * 1_000_000 < timeout_us:
            try:
                status = self._read_register(BMM350_REG_PMU_CMD_STATUS_0)
                if not (status & BMM350_PMU_CMD_BUSY_MSK): # Check if PMU_CMD_BUSY_POS (bit 0) is 0
                    print(f"PMU not busy (PMU_CMD_STATUS_0: {hex(status)}).")
                    return
            except IOError as e:
                # If we get an IOError while waiting, something is wrong.
                raise IOError(f"Error reading PMU_CMD_STATUS_0 while waiting for PMU busy flag: {e}") from e
            time.sleep(0.00001) # Poll every 10us

        last_status = self._read_register(BMM350_REG_PMU_CMD_STATUS_0) # Read one last time for error message
        raise RuntimeError(f"Timeout waiting for PMU to not be busy. Last PMU_CMD_STATUS_0: {hex(last_status)}")

    def _set_power_mode(self, mode: int):
        """
        Sets the power mode of the BMM350 and waits for the command to complete.
        Uses appropriate delays based on the DFRobot library.
        """
        current_mode_is_normal = False
        try:
            # Check if current mode is normal to apply correct delay for suspend
            # This is a simplification; DFRobot's logic is more complex if tracking previous state precisely
            pmu_status = self._read_register(BMM350_REG_PMU_CMD_STATUS_0)
            if pmu_status & (1 << BMM350_PWR_MODE_IS_NORMAL_POS): # Assuming BMM350_PWR_MODE_IS_NORMAL_POS = 3
                 current_mode_is_normal = True
        except IOError:
            print("Warning: Could not read PMU status before setting power mode.")


        print(f"Setting power mode to {hex(mode)}...")
        self._write_register(BMM350_REG_PMU_CMD, mode)

        if mode == BMM350_PMU_CMD_NM:
            # Delay when transitioning to Normal Mode (typically from Suspend)
            time.sleep(BMM350_SUSPEND_TO_NORMAL_DELAY)
        elif mode == BMM350_PMU_CMD_SUS:
            # Delay when transitioning to Suspend Mode
            # DFRobot uses BMM350_GOTO_SUSPEND_DELAY if previous was normal,
            # or a shorter delay if it was already in some form of low power.
            # For simplicity here, using the general suspend delay.
            time.sleep(BMM350_GOTO_SUSPEND_DELAY)
        # Other modes (Forced, Fast Forced) would have different delays based on ODR/AVG.
        # For this initialization, NM and SUS are primary.

        self._wait_pmu_busy() # Wait for the PMU command to complete
        print(f"Power mode {hex(mode)} set and PMU not busy.")


    def _configure_odr_avg(self, odr: int, avg: int):
        val = (odr & BMM350_ODR_MSK) | ((avg << BMM350_AVG_POS) & BMM350_AVG_MSK)
        self._write_register(BMM350_REG_PMU_CMD_AGGR_SET, val)
        time.sleep(BMM350_UPD_OAE_DELAY) # Use constant

    def _enable_axes(self, en_x: bool, en_y: bool, en_z: bool):
        val = 0
        if en_x: val |= BMM350_EN_X_MSK
        if en_y: val |= BMM350_EN_Y_MSK
        if en_z: val |= BMM350_EN_Z_MSK
        self._write_register(BMM350_REG_PMU_CMD_AXIS_EN, val)
        time.sleep(BMM350_UPD_OAE_DELAY) # Use constant

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
        otp_data_u8 = [0] # Changed to single byte read for OTP, per DFRobot's latest example

        # Read Page 0 (16 bytes)
        self._write_register(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_READ_PAGE0)
        time.sleep(0.001) # Wait for OTP read to prepare
        for i in range(8): # Read 8 words (16 bytes)
            lsb = self._read_register(BMM350_REG_OTP_DATA_LSB_REG)
            msb = self._read_register(BMM350_REG_OTP_DATA_MSB_REG)
            otp_data_u8.append(lsb)
            otp_data_u8.append(msb)
            time.sleep(0.0001) # Short delay between reads
        
        # Read Page 1 (16 bytes)
        self._write_register(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_READ_PAGE1)
        time.sleep(0.001)
        for i in range(8): # Read 8 words (16 bytes)
            lsb = self._read_register(BMM350_REG_OTP_DATA_LSB_REG)
            msb = self._read_register(BMM350_REG_OTP_DATA_MSB_REG)
            otp_data_u8.append(lsb)
            otp_data_u8.append(msb)
            time.sleep(0.0001)

        # Check OTP status - bit 0 should be 1 (OTP_OK)
        otp_status = self._read_register(BMM350_REG_OTP_STATUS_REG)
        if not (otp_status & (1 << BMM350_OTP_STATUS_OTP_OK_POS)): # Check OTP_OK bit
            raise RuntimeError(f"BMM350 OTP read failed or data invalid. Status: {hex(otp_status)}")
        
        print("BMM350 OTP data read successfully. Parsing...")

        # Parse OTP data into compensation coefficients
        # Addresses are byte indices into otp_data_u8
        # These are based on DFRobot library's parsing logic for BMM350 OTP map
        # Ensure these match the specific BMM350 OTP layout if issues arise.
        
        # Example: (adjust indices if necessary based on actual OTP map used by DFRobot)
        # self._sens_x_comp = self._unpack_otp_s12(otp_data_u8[0], otp_data_u8[1]) / 1024.0
        # self._sens_y_comp = self._unpack_otp_s12(otp_data_u8[2], otp_data_u8[3]) / 1024.0
        # self._sens_z_comp = self._unpack_otp_s12(otp_data_u8[4], otp_data_u8[5]) / 1024.0
        # ... and so on for other coefficients.
        # For now, keeping them as 1.0 or 0.0 until OTP parsing is fully verified.
        # The DFRobot library has a detailed _parse_otp_data method.
        # This part needs careful implementation matching DFRobot's _parse_otp_data if compensation is critical.
        print("OTP parsing placeholder: using default compensation values.")
        pass # Placeholder for detailed OTP parsing


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
