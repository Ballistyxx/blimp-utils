# -*- coding: utf-8 -*-
"""
magnetometer.py
======================

Module for the BMM350 magnetometer. Provides class infrastructure and
implementation of underlying methods for interacting with the sensor.


:license: The MIT License (MIT)
:author: [Eli Ferrara](eli.ferrara256@gmail.com)
:version: V1.0.0
:date: 2025-05-26
:url: https://github.com/Ballistyxx/blimp-utils
"""

import serial
import time
import os
import math
import numpy as np # Added for matrix multiplication
from .magnetometer_vars import *

# Define BMM350_FLOAT_DATA_ERROR if not already defined elsewhere (e.g. in magnetometer_vars.py)
# This is a common way to handle sensor errors.
if 'BMM350_FLOAT_DATA_ERROR' not in globals():
    BMM350_FLOAT_DATA_ERROR = float('nan')

# Calibration constants derived from user's calibration process
_CALIBRATION_HARD_IRON = np.array([-31820.3708, 23670.7932, -852.7357])

_USER_PROVIDED_SOFT_IRON = np.array([
    [7824.9209, 199.3796, -923.4261],
    [199.3796, 7430.4929, 47.6604],
    [-923.4261, 47.6604, 5784.2916]
])

# Calculate the inverse of the user-provided soft iron matrix, as it's likely M_scaled^(-1/2)
# and we need M_scaled^(1/2) to transform to a unit sphere.
try:
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.linalg.inv(_USER_PROVIDED_SOFT_IRON)
except np.linalg.LinAlgError:
    print("Error: Provided soft iron matrix is singular. Using identity matrix instead. Calibration will be incorrect.")
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.eye(3)


_LOCAL_EARTH_FIELD_UT = 45.0 # Default placeholder - USER SHOULD UPDATE THIS

# --------------------------------------------
class bmm350_dut_offset_coef:
    """
    BMM350 magnetometer DUT offset coefficient structure.

    :param t_offs: Temperature offset.
    :type t_offs: float
    :param offset_x: X-axis offset.
    :type offset_x: float
    :param offset_y: Y-axis offset.
    :type offset_y: float
    :param offset_z: Z-axis offset.
    :type offset_z: float
    """
    def __init__(self, t_offs: float, offset_x: float, offset_y: float, offset_z: float):
        self.t_offs = t_offs
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

class bmm350_dut_sensit_coef:
    """
    BMM350 magnetometer DUT sensitivity coefficient structure.

    :param t_sens: Temperature sensitivity.
    :type t_sens: float
    :param sens_x: X-axis sensitivity.
    :type sens_x: float
    :param sens_y: Y-axis sensitivity.
    :type sens_y: float
    :param sens_z: Z-axis sensitivity.
    :type sens_z: float
    """
    def __init__(self, t_sens: float, sens_x: float, sens_y: float, sens_z: float):
        self.t_sens = t_sens
        self.sens_x = sens_x
        self.sens_y = sens_y
        self.sens_z = sens_z

class bmm350_dut_tco:
    """
    BMM350 magnetometer DUT TCO (Temperature Coefficient of Offset) structure.

    :param tco_x: TCO for X-axis.
    :type tco_x: float
    :param tco_y: TCO for Y-axis.
    :type tco_y: float
    :param tco_z: TCO for Z-axis.
    :type tco_z: float
    """
    def __init__(self, tco_x: float, tco_y: float, tco_z: float):
        self.tco_x = tco_x
        self.tco_y = tco_y
        self.tco_z = tco_z
class bmm350_dut_tcs:
    """
    BMM350 magnetometer DUT TCS (Temperature Coefficient of Sensitivity) structure.

    :param tcs_x: TCS for X-axis.
    :type tcs_x: float
    :param tcs_y: TCS for Y-axis.
    :type tcs_y: float
    :param tcs_z: TCS for Z-axis.
    :type tcs_z: float
    """
    def __init__(self, tcs_x: float, tcs_y: float, tcs_z: float):
        self.tcs_x = tcs_x
        self.tcs_y = tcs_y
        self.tcs_z = tcs_z

class bmm350_cross_axis:
    """
    BMM350 magnetometer cross-axis compensation structure.

    :param cross_x_y: Cross-axis sensitivity from Y to X.
    :type cross_x_y: float
    :param cross_y_x: Cross-axis sensitivity from X to Y.
    :type cross_y_x: float
    :param cross_z_x: Cross-axis sensitivity from X to Z.
    :type cross_z_x: float
    :param cross_z_y: Cross-axis sensitivity from Y to Z.
    :type cross_z_y: float
    """
    def __init__(self, cross_x_y: float, cross_y_x: float, cross_z_x: float, cross_z_y: float):
        self.cross_x_y = cross_x_y
        self.cross_y_x = cross_y_x
        self.cross_z_x = cross_z_x
        self.cross_z_y = cross_z_y

class bmm350_mag_compensate:
    """
    BMM350 magnetometer compensation data structure.
    Aggregates various coefficient structures for sensor compensation.
    """
    def __init__(self, dut_offset_coef: bmm350_dut_offset_coef, dut_sensit_coef: bmm350_dut_sensit_coef, dut_tco: bmm350_dut_tco, dut_tcs: bmm350_dut_tcs, dut_t0: float, cross_axis: bmm350_cross_axis):
        self.dut_offset_coef = dut_offset_coef
        self.dut_sensit_coef = dut_sensit_coef
        self.dut_tco = dut_tco
        self.dut_tcs = dut_tcs
        self.dut_t0 = dut_t0
        self.cross_axis = cross_axis

class bmm350_dev:
  """
  BMM350 device structure.
  Holds device-specific information like chip ID, OTP data, and compensation settings.
  """
  def __init__(self, mag_comp: bmm350_mag_compensate):
    self.chipID   = 0
    self.otp_data = [0] * 32
    self.var_id = 0
    self.mag_comp = mag_comp
    self.power_mode = 0
    self.axis_en = 0

# Create instances of the required classes with example values
dut_offset_coef = bmm350_dut_offset_coef(t_offs=0, offset_x=0, offset_y=0, offset_z=0)
dut_sensit_coef = bmm350_dut_sensit_coef(t_sens=0, sens_x=0, sens_y=0, sens_z=0)
dut_tco = bmm350_dut_tco(tco_x=0, tco_y=0, tco_z=0)
dut_tcs = bmm350_dut_tcs(tcs_x=0, tcs_y=0, tcs_z=0)
cross_axis = bmm350_cross_axis(cross_x_y=0, cross_y_x=0, cross_z_x=0, cross_z_y=0)
mag_comp = bmm350_mag_compensate(
    dut_offset_coef=dut_offset_coef,
    dut_sensit_coef=dut_sensit_coef,
    dut_tco=dut_tco,
    dut_tcs=dut_tcs,
    dut_t0=0,
    cross_axis=cross_axis
)
# The bmm350_mag_compensate object now contains all the data defined above.
bmm350_sensor = bmm350_dev(mag_comp)


# Uncompensated geomagnetic and temperature data
class BMM350RawMagData:
  """
  Structure to hold uncompensated (raw) magnetometer and temperature data.
  """
  def __init__(self):
    self.raw_x_data = 0
    self.raw_y_data = 0
    self.raw_z_data = 0
    self.raw_t_data = 0
_raw_mag_data = BMM350RawMagData()

class BMM350MagData:
  """
  Structure to hold compensated magnetometer data (X, Y, Z) and temperature.
  """
  def __init__(self):
    self.x  = 0
    self.y  = 0
    self.z  = 0
    self.temperature  = 0
_mag_data = BMM350MagData()

class bmm350_pmu_cmd_status_0:
    """
    BMM350 PMU (Power Management Unit) command status 0 register structure.
    Reflects the status of PMU commands and sensor state.
    """
    def __init__(self, pmu_cmd_busy, odr_ovwr, avr_ovwr, pwr_mode_is_normal, cmd_is_illegal, pmu_cmd_value):
        self.pmu_cmd_busy = pmu_cmd_busy
        self.odr_ovwr = odr_ovwr
        self.avr_ovwr = avr_ovwr
        self.pwr_mode_is_normal = pwr_mode_is_normal
        self.cmd_is_illegal = cmd_is_illegal
        self.pmu_cmd_value = pmu_cmd_value
pmu_cmd_stat_0 = bmm350_pmu_cmd_status_0(pmu_cmd_busy=0, odr_ovwr=0, avr_ovwr=0, pwr_mode_is_normal=0, cmd_is_illegal=0, pmu_cmd_value=0)

# --------------------------------------------
class magnetometer_bmm350(object):
  """
  Main class for interacting with the BMM350 magnetometer.
  Handles sensor initialization, configuration, and data acquisition.
  """
  I2C_MODE                       = 1
  I3C_MODE                       = 2
  __thresholdMode = 2
  threshold = 0


  def __init__(self, bus):
    """
    Initialize the magnetometer sensor base class.

    :param bus: I2C or I3C bus identifier. If 0, I3C mode is assumed, otherwise I2C.
    :type bus: int
    """
    if bus != 0:
      self.__i2c_i3c = self.I2C_MODE
    else:
      self.__i2c_i3c = self.I3C_MODE

  def BMM350_SET_BITS(self, reg_data, bitname_msk, bitname_pos, data):
    return (reg_data & ~bitname_msk) | ((data << bitname_pos) & bitname_msk)

  def BMM350_GET_BITS(self, reg_data, mask, pos):
    return (reg_data & mask) >> pos
 
  def BMM350_GET_BITS_POS_0(self, reg_data, mask):
    return reg_data & mask

  def BMM350_SET_BITS_POS_0(self, reg_data, mask, data):
    return ((reg_data & ~(mask)) | (data & mask))  



  # brief This internal API converts the raw data from the IC data registers to signed integer
  def fix_sign(self, inval, number_of_bits):
    power = 0
    if number_of_bits == BMM350_SIGNED_8_BIT:
      power = 128; # 2^7
    elif number_of_bits == BMM350_SIGNED_12_BIT:
      power = 2048 # 2^11
    elif number_of_bits == BMM350_SIGNED_16_BIT:
      power = 32768 # 2^15
    elif number_of_bits == BMM350_SIGNED_21_BIT:
      power = 1048576 # 2^20
    elif number_of_bits == BMM350_SIGNED_24_BIT:
      power = 8388608 # 2^23
    else:
      power = 0
    if inval >= power:
      inval = inval - (power * 2)
    return inval

  # brief This internal API is used to update magnetometer offset and sensitivity data.
  def update_mag_off_sens(self):
    off_x_lsb_msb = bmm350_sensor.otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF
    off_y_lsb_msb = ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) + (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & BMM350_LSB_MASK)
    off_z_lsb_msb = (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) + (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Z] & BMM350_LSB_MASK)
    t_off = bmm350_sensor.otp_data[BMM350_TEMP_OFF_SENS] & BMM350_LSB_MASK

    bmm350_sensor.mag_comp.dut_offset_coef.offset_x = self.fix_sign(off_x_lsb_msb, BMM350_SIGNED_12_BIT)
    bmm350_sensor.mag_comp.dut_offset_coef.offset_y = self.fix_sign(off_y_lsb_msb, BMM350_SIGNED_12_BIT)
    bmm350_sensor.mag_comp.dut_offset_coef.offset_z = self.fix_sign(off_z_lsb_msb, BMM350_SIGNED_12_BIT)
    bmm350_sensor.mag_comp.dut_offset_coef.t_offs = self.fix_sign(t_off, BMM350_SIGNED_8_BIT) / 5.0

    sens_x = (bmm350_sensor.otp_data[BMM350_MAG_SENS_X] & BMM350_MSB_MASK) >> 8
    sens_y = (bmm350_sensor.otp_data[BMM350_MAG_SENS_Y] & BMM350_LSB_MASK)
    sens_z = (bmm350_sensor.otp_data[BMM350_MAG_SENS_Z] & BMM350_MSB_MASK) >> 8
    t_sens = (bmm350_sensor.otp_data[BMM350_TEMP_OFF_SENS] & BMM350_MSB_MASK) >> 8

    bmm350_sensor.mag_comp.dut_sensit_coef.sens_x = self.fix_sign(sens_x, BMM350_SIGNED_8_BIT) / 256.0
    bmm350_sensor.mag_comp.dut_sensit_coef.sens_y = (self.fix_sign(sens_y, BMM350_SIGNED_8_BIT) / 256.0) + BMM350_SENS_CORR_Y
    bmm350_sensor.mag_comp.dut_sensit_coef.sens_z = self.fix_sign(sens_z, BMM350_SIGNED_8_BIT) / 256.0
    bmm350_sensor.mag_comp.dut_sensit_coef.t_sens = self.fix_sign(t_sens, BMM350_SIGNED_8_BIT) / 512.0

    tco_x = (bmm350_sensor.otp_data[BMM350_MAG_TCO_X] & BMM350_LSB_MASK)
    tco_y = (bmm350_sensor.otp_data[BMM350_MAG_TCO_Y] & BMM350_LSB_MASK)
    tco_z = (bmm350_sensor.otp_data[BMM350_MAG_TCO_Z] & BMM350_LSB_MASK)

    bmm350_sensor.mag_comp.dut_tco.tco_x = self.fix_sign(tco_x, BMM350_SIGNED_8_BIT) / 32.0
    bmm350_sensor.mag_comp.dut_tco.tco_y = self.fix_sign(tco_y, BMM350_SIGNED_8_BIT) / 32.0
    bmm350_sensor.mag_comp.dut_tco.tco_z = self.fix_sign(tco_z, BMM350_SIGNED_8_BIT) / 32.0

    tcs_x = (bmm350_sensor.otp_data[BMM350_MAG_TCS_X] & BMM350_MSB_MASK) >> 8
    tcs_y = (bmm350_sensor.otp_data[BMM350_MAG_TCS_Y] & BMM350_MSB_MASK) >> 8
    tcs_z = (bmm350_sensor.otp_data[BMM350_MAG_TCS_Z] & BMM350_MSB_MASK) >> 8

    bmm350_sensor.mag_comp.dut_tcs.tcs_x = self.fix_sign(tcs_x, BMM350_SIGNED_8_BIT) / 16384.0
    bmm350_sensor.mag_comp.dut_tcs.tcs_y = self.fix_sign(tcs_y, BMM350_SIGNED_8_BIT) / 16384.0
    bmm350_sensor.mag_comp.dut_tcs.tcs_z = (self.fix_sign(tcs_z, BMM350_SIGNED_8_BIT) / 16384.0) - BMM350_TCS_CORR_Z

    bmm350_sensor.mag_comp.dut_t0 = (self.fix_sign(bmm350_sensor.otp_data[BMM350_MAG_DUT_T_0], BMM350_SIGNED_16_BIT) / 512.0) + 23.0

    cross_x_y = (bmm350_sensor.otp_data[BMM350_CROSS_X_Y] & BMM350_LSB_MASK)
    cross_y_x = (bmm350_sensor.otp_data[BMM350_CROSS_Y_X] & BMM350_MSB_MASK) >> 8
    cross_z_x = (bmm350_sensor.otp_data[BMM350_CROSS_Z_X] & BMM350_LSB_MASK)
    cross_z_y = (bmm350_sensor.otp_data[BMM350_CROSS_Z_Y] & BMM350_MSB_MASK) >> 8

    bmm350_sensor.mag_comp.cross_axis.cross_x_y = self.fix_sign(cross_x_y, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_y_x = self.fix_sign(cross_y_x, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_z_x = self.fix_sign(cross_z_x, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_z_y = self.fix_sign(cross_z_y, BMM350_SIGNED_8_BIT) / 800.0


  def bmm350_set_powermode(self, powermode):
    last_pwr_mode = self.read_reg(BMM350_REG_PMU_CMD, 1)
    if (last_pwr_mode[0] == BMM350_PMU_CMD_NM) or (last_pwr_mode[0] == BMM350_PMU_CMD_UPD_OAE):
      self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUS)
      time.sleep(BMM350_GOTO_SUSPEND_DELAY)
    # Array to store suspend to forced mode delay
    sus_to_forced_mode =[BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY, BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY, 
                          BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY, BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY]
    # Array to store suspend to forced mode fast delay
    sus_to_forced_mode_fast = [BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY, BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY,
                               BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY, BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY]
    # Set PMU command configuration to desired power mode
    self.write_reg(BMM350_REG_PMU_CMD, powermode)
    # Get average configuration
    get_avg = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    # Mask the average value
    avg = ((get_avg[0] & BMM350_AVG_MSK) >> BMM350_AVG_POS)
    delay_us = 0
    # Check if desired power mode is normal mode
    if powermode == BMM350_NORMAL_MODE:
      delay_us = BMM350_SUSPEND_TO_NORMAL_DELAY
    # Check if desired power mode is forced mode
    if powermode == BMM350_FORCED_MODE:
      delay_us = sus_to_forced_mode[avg]; # Store delay based on averaging mode
    # Check if desired power mode is forced mode fast
    if powermode == BMM350_FORCED_MODE_FAST:
        delay_us = sus_to_forced_mode_fast[avg] # Store delay based on averaging mode
    # Perform delay based on power mode
    time.sleep(delay_us)
    bmm350_sensor.power_mode = powermode


  def bmm350_magnetic_reset_and_wait(self):
    # 1. Read PMU CMD status
    reg_data = self.read_reg(BMM350_REG_PMU_CMD_STATUS_0, 1)
    pmu_cmd_stat_0.pmu_cmd_busy = self.BMM350_GET_BITS_POS_0(reg_data[0], BMM350_PMU_CMD_BUSY_MSK)
    pmu_cmd_stat_0.odr_ovwr = self.BMM350_GET_BITS(reg_data[0], BMM350_ODR_OVWR_MSK, BMM350_ODR_OVWR_POS)
    pmu_cmd_stat_0.avr_ovwr = self.BMM350_GET_BITS(reg_data[0], BMM350_AVG_OVWR_MSK, BMM350_AVG_OVWR_POS)
    pmu_cmd_stat_0.pwr_mode_is_normal = self.BMM350_GET_BITS(reg_data[0], BMM350_PWR_MODE_IS_NORMAL_MSK, BMM350_PWR_MODE_IS_NORMAL_POS)
    pmu_cmd_stat_0.cmd_is_illegal = self.BMM350_GET_BITS(reg_data[0], BMM350_CMD_IS_ILLEGAL_MSK, BMM350_CMD_IS_ILLEGAL_POS)
    pmu_cmd_stat_0.pmu_cmd_value = self.BMM350_GET_BITS(reg_data[0], BMM350_PMU_CMD_VALUE_MSK, BMM350_PMU_CMD_VALUE_POS)
    # 2. Check whether the power mode is normal before magnetic reset
    restore_normal = BMM350_DISABLE
    if pmu_cmd_stat_0.pwr_mode_is_normal == BMM350_ENABLE:
      restore_normal = BMM350_ENABLE
    # Reset can only be triggered in suspend
    self.bmm350_set_powermode(BMM350_SUSPEND_MODE)
    # Set BR to PMU_CMD register
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)
    time.sleep(BMM350_BR_DELAY)
    # Set FGR to PMU_CMD register
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)
    time.sleep(BMM350_FGR_DELAY)
    if restore_normal == BMM350_ENABLE:
      self.bmm350_set_powermode(BMM350_NORMAL_MODE)


  def sensor_init(self):
    """
    Initialize the BMM350 sensor.

    Checks the chip ID, downloads OTP compensation data from the sensor's
    One-Time Programmable memory, updates internal compensation parameters,
    and performs a magnetic reset.

    :return: Initialization status.
    :rtype: int
    :retval BMM350_OK: Initialization successful.
    :retval BMM350_CHIP_ID_ERROR: Chip ID mismatch, initialization failed.
    """
    # _magData = [0, 0, 0]
    # rslt = self._get_compensated_mag_xyz_data(_magData)
    rslt = BMM350_OK
    # Specifies that all axes are enabled
    bmm350_sensor.axis_en = BMM350_EN_XYZ_MSK
    time.sleep(BMM350_START_UP_TIME_FROM_POR)
    # 1. Software reset
    self.write_reg(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)
    time.sleep(BMM350_SOFT_RESET_DELAY)
    # 2. Read chip ID
    reg_date = self.read_reg(BMM350_REG_CHIP_ID, 1)
    bmm350_sensor.chipID = reg_date[0]
    if reg_date[0] == BMM350_CHIP_ID:
      # 3. Download OTP compensation data
      for indx in range(BMM350_OTP_DATA_LENGTH):
        # 3.1 Set the OTP address register -- > Each address corresponds to a different OTP value (total OTP data is 32 bytes)
        otp_cmd = BMM350_OTP_CMD_DIR_READ | (indx & BMM350_OTP_WORD_ADDR_MSK)
        self.write_reg(BMM350_REG_OTP_CMD_REG, otp_cmd)
        while (True):
          time.sleep(0.0003)
          # 3.2 The OTP status was read
          otp_status = self.read_reg(BMM350_REG_OTP_STATUS_REG, 1)
          otp_err = otp_status[0] & BMM350_OTP_STATUS_ERROR_MSK
          if otp_err != BMM350_OTP_STATUS_NO_ERROR:
            break
          if (otp_status[0] & BMM350_OTP_STATUS_CMD_DONE):
            break
        # 3.3 Gets 16 bytes of OTP data from the OTP address specified above
        OTP_MSB_data = self.read_reg(BMM350_REG_OTP_DATA_MSB_REG, 1)
        OTP_LSB_data = self.read_reg(BMM350_REG_OTP_DATA_LSB_REG, 1)
        OTP_data = ((OTP_MSB_data[0] << 8) | OTP_LSB_data[0]) & 0xFFFF
        bmm350_sensor.otp_data[indx] = OTP_data
        bmm350_sensor.var_id = (bmm350_sensor.otp_data[30] & 0x7f00) >> 9
        # 3.4 Update the magnetometer offset and sensitivity data
        self.update_mag_off_sens()
      # 4. Disable OTP
      self.write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP)

      # 5. Magnetic reset
      self.bmm350_magnetic_reset_and_wait()
    else:
      # The chip id verification failed and initialization failed. Procedure
      rslt = BMM350_CHIP_ID_ERROR
    return rslt


  def get_chip_id(self):
    """
    Read the chip ID from the sensor.

    :return: The chip ID value.
    :rtype: int
    """
    chip_id = self.read_reg(BMM350_REG_CHIP_ID, 1)
    return chip_id[0]


  def soft_reset(self):
    """
    Perform a soft reset of the sensor.

    This command resets the sensor to its default state and places it into
    suspend mode. OTP data is reloaded, and a magnetic reset is performed.
    The user needs to manually set the desired operation mode afterwards.
    """
    self.write_reg(BMM350_REG_CMD, BMM350_CMD_SOFTRESET) # Software reset
    time.sleep(BMM350_SOFT_RESET_DELAY)
    self.write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP) # Disable OTP
    self.bmm350_magnetic_reset_and_wait() # magnetic reset
    self.bmm350_set_powermode(BMM350_SUSPEND_MODE)


  def set_operation_mode(self, modes):
    """
    Set the sensor's operation mode.

    :param modes: The desired operation mode.
    :type modes: int
    :options modes:
        * ``BMM350_SUSPEND_MODE``: Suspend mode. Minimal current consumption. All registers are accessible.
        * ``BMM350_NORMAL_MODE``: Normal mode. Continuous data acquisition based on ODR.
        * ``BMM350_FORCED_MODE``: Forced mode. Sensor performs a single measurement and returns to suspend mode.
        * ``BMM350_FORCED_MODE_FAST``: Forced mode fast. Similar to forced mode, but optimized for faster single measurements, allowing ODR up to 200Hz.
    """
    self.bmm350_set_powermode(modes)


  def get_operation_mode(self):
    """
    Get the current operation mode of the sensor.

    :return: A string describing the current operation mode (e.g., "bmm350 is normal mode!").
    :rtype: str
    """
    result = ""
    if bmm350_sensor.power_mode == BMM350_SUSPEND_MODE:
       result = "bmm350 is suspend mode!"
    elif bmm350_sensor.power_mode == BMM350_NORMAL_MODE:
       result = "bmm350 is normal mode!"
    elif bmm350_sensor.power_mode == BMM350_FORCED_MODE:
       result = "bmm350 is forced mode!"
    elif bmm350_sensor.power_mode == BMM350_FORCED_MODE_FAST:
       result = "bmm350 is forced_fast mode!"
    else:
       result = "error mode!"
    return result


  def set_rate(self, rates):
    """
    Set the data output rate (ODR) for geomagnetic data.

    The ODR determines how frequently new data is available from the sensor
    when in normal mode.

    :param rates: The desired data rate identifier.
    :type rates: int
    :options rates:
        * ``BMM350_DATA_RATE_1_5625HZ`` (1.5625 Hz)
        * ``BMM350_DATA_RATE_3_125HZ`` (3.125 Hz)
        * ``BMM350_DATA_RATE_6_25HZ`` (6.25 Hz)
        * ``BMM350_DATA_RATE_12_5HZ`` (12.5 Hz, default rate)
        * ``BMM350_DATA_RATE_25HZ`` (25 Hz)
        * ``BMM350_DATA_RATE_50HZ`` (50 Hz)
        * ``BMM350_DATA_RATE_100HZ`` (100 Hz)
        * ``BMM350_DATA_RATE_200HZ`` (200 Hz)
        * ``BMM350_DATA_RATE_400HZ`` (400 Hz)
    """
    # self.bmm350_set_powermode(BMM350_NORMAL_MODE)
    avg_odr_reg = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    avg_reg = self.BMM350_GET_BITS(avg_odr_reg[0], BMM350_AVG_MSK, BMM350_AVG_POS)
    reg_data = (rates & BMM350_ODR_MSK)
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_AVG_MSK, BMM350_AVG_POS, avg_reg)
    self.write_reg(BMM350_REG_PMU_CMD_AGGR_SET, reg_data)
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)
    time.sleep(BMM350_UPD_OAE_DELAY)


  def get_rate(self):
    """
    Get the configured data output rate (ODR).

    :return: The data rate in Hertz (Hz).
    :rtype: float
    """
    avg_odr_reg = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    odr_reg = self.BMM350_GET_BITS(avg_odr_reg[0], BMM350_ODR_MSK, BMM350_ODR_POS)
    if odr_reg == BMM350_ODR_1_5625HZ:
      result = 1.5625
    elif odr_reg == BMM350_ODR_3_125HZ:
      result = 3.125
    elif odr_reg == BMM350_ODR_6_25HZ:
      result = 6.25
    elif odr_reg == BMM350_ODR_12_5HZ:
      result = 12.5
    elif odr_reg == BMM350_ODR_25HZ:
      result = 25
    elif odr_reg == BMM350_ODR_50HZ:
      result = 50
    elif odr_reg == BMM350_ODR_100HZ:
      result = 100
    elif odr_reg == BMM350_ODR_200HZ:
      result = 200
    elif odr_reg == BMM350_ODR_400HZ:
      result = 400
    return result


  def set_preset_mode(self, avg, odr = BMM350_DATA_RATE_12_5HZ):
    """
    Set a preset mode for sensor configuration, combining averaging and ODR.

    This simplifies sensor setup by providing predefined configurations for
    different use cases.

    :param avg: Averaging configuration identifier. This determines the number of
                raw samples averaged for one output data point.
    :type avg: int
    :param odr: Output Data Rate identifier, defaults to ``BMM350_DATA_RATE_12_5HZ``.
    :type odr: int, optional
    :options avg:
        * ``BMM350_AVERAGING_NO_AVG`` (or similar, check definitions for actual names)
        * ``BMM350_AVERAGING_2_AVG``
        * ``BMM350_AVERAGING_4_AVG``
        * ``BMM350_AVERAGING_8_AVG``
    :options odr:
        * ``BMM350_DATA_RATE_1_5625HZ`` (1.5625 Hz)
        * ``BMM350_DATA_RATE_3_125HZ`` (3.125 Hz)
        * ``BMM350_DATA_RATE_6_25HZ`` (6.25 Hz)
        * ``BMM350_DATA_RATE_12_5HZ`` (12.5 Hz, default rate)
        * ``BMM350_DATA_RATE_25HZ`` (25 Hz)
        * ``BMM350_DATA_RATE_50HZ`` (50 Hz)
        * ``BMM350_DATA_RATE_100HZ`` (100 Hz)
        * ``BMM350_DATA_RATE_200HZ`` (200 Hz)
        * ``BMM350_DATA_RATE_400HZ`` (400 Hz)
    """
    reg_data = (odr & BMM350_ODR_MSK)
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_AVG_MSK, BMM350_AVG_POS, avg)
    self.write_reg(BMM350_REG_PMU_CMD_AGGR_SET, reg_data)
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)  


  def self_test(self):
    """
    Perform a sensor self-test.

    Checks the enabled axes based on the PMU_CMD_AXIS_EN register.

    :return: A string indicating the self-test result (e.g., "x y z aix test success").
    :rtype: str
    """
    axis_en = self.read_reg(BMM350_REG_PMU_CMD_AXIS_EN, 1)
    en_x = self.BMM350_GET_BITS(axis_en[0], BMM350_EN_X_MSK, BMM350_EN_X_POS)
    en_y = self.BMM350_GET_BITS(axis_en[0], BMM350_EN_Y_MSK, BMM350_EN_Y_POS)
    en_z = self.BMM350_GET_BITS(axis_en[0], BMM350_EN_Z_MSK, BMM350_EN_Z_POS)
    str1 = ""
    if en_x & 0x01:
      str1 += "x "
    if en_y & 0x01:
      str1 += "y "
    if en_z & 0x01:
      str1 += "z "
    if axis_en == 0:
      str1 = "xyz aix self test fail"
    else:
      str1 += "aix test success"
    return str1


  def set_measurement_XYZ(self, en_x = BMM350_X_EN, en_y = BMM350_Y_EN, en_z = BMM350_Z_EN):
    """
    Enable or disable measurements for X, Y, and Z axes.

    By default, all axes are enabled. Disabling an axis will result in incorrect
    or zero data for that axis. If all axes are disabled, the sensor might
    enter a specific low-power state or stop measurements.

    :param en_x: Enable state for X-axis. Defaults to ``BMM350_X_EN``.
    :type en_x: int, optional
    :options en_x: ``BMM350_X_EN`` (enable), ``BMM350_X_DIS`` (disable)
    :param en_y: Enable state for Y-axis. Defaults to ``BMM350_Y_EN``.
    :type en_y: int, optional
    :options en_y: ``BMM350_Y_EN`` (enable), ``BMM350_Y_DIS`` (disable)
    :param en_z: Enable state for Z-axis. Defaults to ``BMM350_Z_EN``.
    :type en_z: int, optional
    :options en_z: ``BMM350_Z_EN`` (enable), ``BMM350_Z_DIS`` (disable)
    """
    if en_x == BMM350_X_DIS and en_y == BMM350_Y_DIS and en_z == BMM350_Z_DIS:
      bmm350_sensor.axis_en = BMM350_DISABLE
    else:
      axis_en = self.read_reg(BMM350_REG_PMU_CMD_AXIS_EN, 1)
      data = self.BMM350_SET_BITS(axis_en[0], BMM350_EN_X_MSK, BMM350_EN_X_POS, en_x)
      data = self.BMM350_SET_BITS(data, BMM350_EN_Y_MSK, BMM350_EN_Y_POS, en_y)
      data = self.BMM350_SET_BITS(data, BMM350_EN_Z_MSK, BMM350_EN_Z_POS, en_z)
      bmm350_sensor.axis_en = data


  def get_measurement_state_XYZ(self):
    """
    Get the measurement enable status for X, Y, and Z axes.

    :return: A string describing the enable status of each axis (e.g., "The x axis is enable! The y axis is enable! The z axis is enable! ").
    :rtype: str
    """
    axis_en = bmm350_sensor.axis_en
    en_x = self.BMM350_GET_BITS(axis_en, BMM350_EN_X_MSK, BMM350_EN_X_POS)
    en_y = self.BMM350_GET_BITS(axis_en, BMM350_EN_Y_MSK, BMM350_EN_Y_POS)
    en_z = self.BMM350_GET_BITS(axis_en, BMM350_EN_Z_MSK, BMM350_EN_Z_POS)
    result = ""
    result += "The x axis is enable! " if en_x == 1 else "The x axis is disable! "
    result += "The y axis is enable! " if en_y == 1 else "The y axis is disable! "
    result += "The z axis is enable! " if en_z == 1 else "The z axis is disable! "
    return result

  def get_raw_magnetic_data_for_calibration(self):
    """
    Get raw X, Y, Z magnetic data before OTP and any software compensations.

    This method reads the raw 24-bit sensor values for each axis directly
    from the data registers and applies sign correction. It is intended for
    use during calibration procedures.

    :return: List containing raw [x, y, z] data as integers, or None if the read fails.
    :rtype: list[int] or None
    """
    # 1. Read raw data registers for X, Y, Z axes
    # BMM350_MAG_TEMP_DATA_LEN is 12 (Xlsb,Xmsb,Xmmsb, Ylsb,Ymsb,Ymmsb, Zlsb,Zmsb,Zmmsb, Tlsb,Tmsb,Tmmsb)
    # We need the first 9 bytes for X, Y, Z.
    mag_data_regs = self.read_reg(BMM350_REG_MAG_X_XLSB, 9)
    if mag_data_regs is None or len(mag_data_regs) < 9:
        # Handle error case if read_reg fails or returns insufficient data
        print("Failed to read raw magnetometer data for calibration.")
        return None

    raw_mag_x_u = mag_data_regs[0] + (mag_data_regs[1] << 8) + (mag_data_regs[2] << 16)
    raw_mag_y_u = mag_data_regs[3] + (mag_data_regs[4] << 8) + (mag_data_regs[5] << 16)
    raw_mag_z_u = mag_data_regs[6] + (mag_data_regs[7] << 8) + (mag_data_regs[8] << 16)

    # Apply sign correction using the existing fix_sign method
    # BMM350_SIGNED_24_BIT is used in get_geomagnetic_data for these raw values
    raw_x = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    raw_y = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    raw_z = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    
    return [raw_x, raw_y, raw_z]

  def get_geomagnetic_data(self):
    """
    Get calibrated geomagnetic data (X, Y, Z) in microTeslas (µT).

    This method reads raw sensor data, applies hard and soft iron calibrations
    using pre-defined calibration matrices (`_CALIBRATION_HARD_IRON`,
    `_CALIBRATION_SOFT_IRON_TRANSFORM`), and scales the result to physical units (µT)
    using `_LOCAL_EARTH_FIELD_UT`. It also calculates and updates the sensor temperature.

    :return: List containing calibrated [X_µT, Y_µT, Z_µT].
              Returns a list of ``BMM350_FLOAT_DATA_ERROR`` (e.g., `float('nan')`)
              for each component on failure to read sensor data.
    :rtype: list[float]
    """
    mag_and_temp_regs = self.read_reg(BMM350_REG_MAG_X_XLSB, BMM350_MAG_TEMP_DATA_LEN)
    if mag_and_temp_regs is None or len(mag_and_temp_regs) < BMM350_MAG_TEMP_DATA_LEN:
        print("Failed to read sensor data.")
        # Ensure _mag_data is accessible here if it's a global or instance variable
        # For safety, directly return error values if _mag_data might not be defined/accessible.
        # _mag_data.x = _mag_data.y = _mag_data.z = BMM350_FLOAT_DATA_ERROR
        # _mag_data.temperature = BMM350_FLOAT_DATA_ERROR
        return [BMM350_FLOAT_DATA_ERROR] * 3

    raw_mag_x_u = mag_and_temp_regs[0] + (mag_and_temp_regs[1] << 8) + (mag_and_temp_regs[2] << 16)
    raw_mag_y_u = mag_and_temp_regs[3] + (mag_and_temp_regs[4] << 8) + (mag_and_temp_regs[5] << 16)
    raw_mag_z_u = mag_and_temp_regs[6] + (mag_and_temp_regs[7] << 8) + (mag_and_temp_regs[8] << 16)
    raw_temp_u  = mag_and_temp_regs[9] + (mag_and_temp_regs[10] << 8) + (mag_and_temp_regs[11] << 16)

    raw_x_lsb = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    raw_y_lsb = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    raw_z_lsb = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    
    # Access _raw_mag_data correctly (e.g., self._raw_mag_data or global _raw_mag_data)
    # Assuming _raw_mag_data is a global instance for now as per original structure hints
    _raw_mag_data.raw_t_data = self.fix_sign(raw_temp_u, BMM350_SIGNED_24_BIT)
    _raw_mag_data.raw_x_data = raw_x_lsb
    _raw_mag_data.raw_y_data = raw_y_lsb
    _raw_mag_data.raw_z_data = raw_z_lsb

    raw_lsb_vec = np.array([raw_x_lsb, raw_y_lsb, raw_z_lsb])
    hard_iron_corrected_vec = raw_lsb_vec - _CALIBRATION_HARD_IRON
    
    # Use the inverted soft iron matrix
    unit_sphere_vec = _CALIBRATION_SOFT_IRON_TRANSFORM @ hard_iron_corrected_vec
    geomagnetic_ut_vec = _LOCAL_EARTH_FIELD_UT * unit_sphere_vec

    # Access _mag_data correctly
    _mag_data.x = geomagnetic_ut_vec[0]
    _mag_data.y = geomagnetic_ut_vec[1]
    _mag_data.z = geomagnetic_ut_vec[2]

    # Temperature Calculation (preserved)
    bxy_sens = 14.55; bz_sens = 9.0; temp_sens = 0.00204
    ina_xy_gain_trgt = 19.46; ina_z_gain_trgt = 31.0
    adc_gain = 1 / 1.5; lut_gain = 0.714607238769531
    lsb_to_degc_for_temp = 1 / (temp_sens * adc_gain * lut_gain * 1048576)
    temp_out_data = _raw_mag_data.raw_t_data * lsb_to_degc_for_temp
    temp_adjusted = temp_out_data - (25.49 if temp_out_data > 0 else (-25.49 if temp_out_data < 0 else 0))
    _mag_data.temperature = (1 + bmm350_sensor.mag_comp.dut_sensit_coef.t_sens) * temp_adjusted + bmm350_sensor.mag_comp.dut_offset_coef.t_offs

    return [geomagnetic_ut_vec[0], geomagnetic_ut_vec[1], geomagnetic_ut_vec[2]]

  def get_compass_degree(self):
    """
    Calculate the compass heading (yaw) in degrees.

    This calculation assumes the sensor's XY plane is horizontal (parallel to the ground).
    The heading is the angle in the XY plane, measured counter-clockwise from the
    sensor's positive X-axis to the magnetic field vector.

    If the sensor's X-axis is aligned with the vehicle's forward direction, this
    function returns the magnetic heading of the vehicle.

    .. note:: This calculation is not tilt-compensated. For accurate yaw when the
              sensor is tilted, data from an accelerometer would be required to
              correct for the tilt.

    :return: Compass heading in degrees (0.0 to 360.0).
              Returns ``BMM350_FLOAT_DATA_ERROR`` (e.g., `float('nan')`) if
              magnetometer data is invalid.
    :rtype: float
    """
    magData = self.get_geomagnetic_data() # Returns [X, Y, Z] in uT

    if magData is None or any(math.isnan(comp) for comp in magData):
        print("Warning: Invalid magnetometer data received for compass calculation.")
        return BMM350_FLOAT_DATA_ERROR # Or a suitable error value like float('nan')

    # Standard formula for yaw (heading) from magnetometer X and Y components
    # atan2(Y, X) gives the angle in radians from the positive X-axis to the point (X,Y)
    yaw_radians = math.atan2(magData[1], magData[0]) # magData[1] is Y, magData[0] is X

    # Convert radians to degrees
    yaw_degrees = math.degrees(yaw_radians)

    # Normalize to 0-360 degrees
    # math.degrees(atan2(y,x)) returns angle in (-180, 180]
    if yaw_degrees < 0:
        yaw_degrees += 360.0
    
    return yaw_degrees

  def set_data_ready_pin(self, modes, polarity):
    """
    Configure the data ready (DRDY) interrupt pin.

    Enables or disables the DRDY interrupt and sets its active polarity.
    When enabled, the DRDY pin changes state when new sensor data is available.

    :param modes: Enable or disable the DRDY interrupt.
    :type modes: int
    :options modes:
        * ``BMM350_ENABLE_INTERRUPT``: Enable DRDY interrupt.
        * ``BMM350_DISABLE_INTERRUPT``: Disable DRDY interrupt.
    :param polarity: Polarity of the DRDY pin.
    :type polarity: int
    :options polarity:
        * ``BMM350_ACTIVE_HIGH``: DRDY pin is active high (e.g., low normally, high on interrupt).
        * ``BMM350_ACTIVE_LOW``: DRDY pin is active low (e.g., high normally, low on interrupt).
    """
    # 1. Gets and sets the interrupt control configuration
    reg_data = self.read_reg(BMM350_REG_INT_CTRL, 1)
    reg_data[0] = self.BMM350_SET_BITS_POS_0(reg_data[0], BMM350_INT_MODE_MSK, BMM350_INT_MODE_PULSED)
    reg_data[0] = self.BMM350_SET_BITS(reg_data[0], BMM350_INT_POL_MSK, BMM350_INT_POL_POS, polarity)
    reg_data[0] = self.BMM350_SET_BITS(reg_data[0], BMM350_INT_OD_MSK, BMM350_INT_OD_POS, BMM350_INT_OD_PUSHPULL) 
    reg_data[0] = self.BMM350_SET_BITS(reg_data[0], BMM350_INT_OUTPUT_EN_MSK, BMM350_INT_OUTPUT_EN_POS, BMM350_MAP_TO_PIN)
    reg_data[0] = self.BMM350_SET_BITS(reg_data[0], BMM350_DRDY_DATA_REG_EN_MSK, BMM350_DRDY_DATA_REG_EN_POS, modes)
    # 2. Update interrupt control configuration
    self.write_reg(BMM350_REG_INT_CTRL, reg_data[0]) 


  def get_data_ready_state(self):
    """
    Get the data ready status from the interrupt status register.

    Indicates whether new measurement data is available.

    :return: True if data is ready, False otherwise.
    :rtype: bool
    """
    int_status_reg = self.read_reg(BMM350_REG_INT_STATUS, 1) 
    drdy_status = self.BMM350_GET_BITS(int_status_reg[0], BMM350_DRDY_DATA_REG_MSK, BMM350_DRDY_DATA_REG_POS)
    if drdy_status & 0x01:
      return True
    else:
      return False


  def set_threshold_interrupt(self, modes, threshold, polarity):
    """
    Configure the threshold interrupt.

    An interrupt is triggered when the geomagnetic value of any enabled channel
    crosses a defined threshold. The interrupt can be configured for low threshold
    (triggers if value < threshold) or high threshold (triggers if value > threshold).

    .. note:: The actual threshold used by the sensor might be scaled.
              The original comment mentioned "default to expand 16 times".
              Verify this behavior with the sensor datasheet.

    :param modes: Threshold interrupt mode.
    :type modes: int
    :options modes:
        * ``LOW_THRESHOLD_INTERRUPT``: Low threshold interrupt mode.
        * ``HIGH_THRESHOLD_INTERRUPT``: High threshold interrupt mode.
    :param threshold: The threshold value. The interpretation of this value
                      (raw LSB or scaled µT) depends on the sensor's behavior.
    :type threshold: int or float
    :param polarity: Polarity of the interrupt pin when a threshold event occurs.
    :type polarity: int
    :options polarity:
        * ``POLARITY_HIGH``: Active high.
        * ``POLARITY_LOW``: Active low.
    """
    if modes == LOW_THRESHOLD_INTERRUPT:
      self.__thresholdMode = LOW_THRESHOLD_INTERRUPT
      self.set_data_ready_pin(BMM350_ENABLE_INTERRUPT, polarity)
      self.threshold = threshold
    else:
      self.__thresholdMode = HIGH_THRESHOLD_INTERRUPT
      self.set_data_ready_pin(BMM350_ENABLE_INTERRUPT, polarity)
      self.threshold = threshold
    

  def get_threshold_data(self):
    """
    Get the geomagnetic data that triggered a threshold interrupt.

    If a threshold interrupt has occurred (check with ``get_data_ready_state()``
    if DRDY pin is mapped to threshold events, or a specific threshold status register),
    this function returns the data for each axis.

    :return: A list of three elements [x, y, z]. If an axis's data triggered
             the interrupt (based on the configured threshold mode and value),
             its value is returned. Otherwise, the corresponding element in the
             list will be ``NO_DATA``.
    :rtype: list
    """
    Data = [NO_DATA] * 3
    state = self.get_data_ready_state()
    if state == True:
      magData = self.get_geomagnetic_data()
      if self.__thresholdMode == LOW_THRESHOLD_INTERRUPT:
        if magData[0] < self.threshold*16:
          Data[0] = magData[0]
        if magData[1] < self.threshold*16:
          Data[1] = magData[1]
        if magData[2] < self.threshold*16:
          Data[2] = magData[2]
      elif self.__thresholdMode == HIGH_THRESHOLD_INTERRUPT:
        if magData[0] < self.threshold*16:
          Data[0] = magData[0]
        if magData[1] < self.threshold*16:
          Data[1] = magData[1]
        if magData[2] < self.threshold*16:
          Data[2] = magData[2]
    return Data

# I2C interface
class magnetometer_bmm350_I2C(magnetometer_bmm350):
  """
  I2C interface implementation for the BMM350 magnetometer.
  Inherits from :class:`magnetometer_bmm350` and provides I2C communication methods.
  """
  def __init__(self, bus, addr):
    """
    Initialize the I2C interface for the BMM350 sensor.

    :param bus: I2C bus number (e.g., 1 for Raspberry Pi).
    :type bus: int
    :param addr: I2C address of the BMM350 sensor.
    :type addr: int
    """
    self.bus = bus
    self.__addr = addr
    if self.is_raspberrypi():
      import smbus
      self.i2cbus = smbus.SMBus(bus)
    else:
      self.test_platform()
    super(magnetometer_bmm350_I2C, self).__init__(self.bus)

  def is_raspberrypi(self):
    import io
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower(): return True
    except Exception: pass
    return False

  def test_platform(self):
    import re
    import platform
    import subprocess
    where = platform.system()
    if where == "Linux":
        p = subprocess.Popen(['i2cdetect', '-l'], stdout=subprocess.PIPE,)
        for i in range(0, 25):
            line = str(p.stdout.readline())
            s = re.search("i2c-tiny-usb", line)
            if s:
                line = re.split(r'\W+', line)
                bus = int(line[2])
        import smbus
        self.i2cbus = smbus.SMBus(bus)
    elif where == "Windows":
        from i2c_mp_usb import I2C_MP_USB as SMBus
        self.i2cbus = SMBus()
    else:
        print("Platform not supported")  



  def write_reg(self, reg, data):
    """
    Write a byte of data to a specified register via I2C.

    :param reg: Register address to write to.
    :type reg: int
    :param data: Byte of data to write.
    :type data: int
    """
    while 1:
      try:
        self.i2cbus.write_byte_data(self.__addr, reg, data)
        return
      except:
        print("please check connect w!")
        time.sleep(1)
        return
  
  def read_reg(self, reg, length): # Changed 'len' to 'length'
    """
    Read a specified number of bytes from a register via I2C.

    Accounts for dummy bytes if required by the BMM350 I2C protocol.

    :param reg: Register address to read from.
    :type reg: int
    :param length: Number of bytes to read (excluding dummy bytes).
    :type length: int
    :return: A list of bytes read from the register.
    :rtype: list[int]
    """
    while True:
      try:
        # Read data from I2C bus
        temp_buf = self.i2cbus.read_i2c_block_data(self.__addr, reg, length + BMM350_DUMMY_BYTES)
        # Copy data after dummy byte indices
        reg_data = temp_buf[BMM350_DUMMY_BYTES:]
        return reg_data  # Assuming this function is part of a larger method
      except Exception as e:
        time.sleep(1)
        print("please check connect r!")
