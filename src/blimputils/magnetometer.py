# -*- coding: utf-8 -*-
"""
magnetometer.py
======================

Module for the Bosch-Sensortec BMM350 magnetometer. Provides class infrastructure and
implementation of underlying methods for interacting with the sensor.
"""

import time
import os
import math
import numpy as np # Added for matrix multiplication
from .magnetometer_vars import *

# This is a common way to handle sensor errors.
if 'BMM350_FLOAT_DATA_ERROR' not in globals():
    BMM350_FLOAT_DATA_ERROR = float('nan')

# Calibration constants derived from user's calibration process
#_CALIBRATION_HARD_IRON = np.array([-31820.3708, 23670.7932, -852.7357])
_CALIBRATION_HARD_IRON = np.array([-11437.7170, 14531.7774, -4471.1151])
'''
_USER_PROVIDED_SOFT_IRON = np.array([
    [7824.9209, 199.3796, -923.4261],
    [199.3796, 7430.4929, 47.6604],
    [-923.4261, 47.6604, 5784.2916]
])
'''
_USER_PROVIDED_SOFT_IRON = np.array([
   [9085.6920, 595.7422, -1201.8323],
   [595.7422, 6615.2679, 298.3144],
   [-1201.8323, 298.3144, 6489.1692]
])


# Calculate the inverse of the user-provided soft iron matrix, as it's likely M_scaled^(-1/2)
# and we need M_scaled^(1/2) to transform to a unit sphere.
try:
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.linalg.inv(_USER_PROVIDED_SOFT_IRON)
except np.linalg.LinAlgError:
    print("Error: Provided soft iron matrix is singular. Using identity matrix instead. Calibration will be incorrect.")
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.eye(3)


_LOCAL_EARTH_FIELD_UT = 45.0 # Default placeholder - USER CAN UPDATE THIS

# --------------------------------------------
'''!
  bmm350 magnetometer dut offset coefficient structure
'''
class bmm350_dut_offset_coef:
    """
    BMM350 magnetometer DUT offset coefficient structure.

    Stores temperature and XYZ offset coefficients.

    :param t_offs: Temperature offset coefficient.
    :type t_offs: float
    :param offset_x: X-axis offset coefficient.
    :type offset_x: float
    :param offset_y: Y-axis offset coefficient.
    :type offset_y: float
    :param offset_z: Z-axis offset coefficient.
    :type offset_z: float
    """
    def __init__(self, t_offs: float, offset_x: float, offset_y: float, offset_z: float):
        self.t_offs = t_offs
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

'''!
  bmm350 magnetometer dut sensitivity coefficient structure
'''
class bmm350_dut_sensit_coef:
    """
    BMM350 magnetometer DUT sensitivity coefficient structure.

    Stores temperature and XYZ sensitivity coefficients.

    :param t_sens: Temperature sensitivity coefficient.
    :type t_sens: float
    :param sens_x: X-axis sensitivity coefficient.
    :type sens_x: float
    :param sens_y: Y-axis sensitivity coefficient.
    :type sens_y: float
    :param sens_z: Z-axis sensitivity coefficient.
    :type sens_z: float
    """
    def __init__(self, t_sens: float, sens_x: float, sens_y: float, sens_z: float):
        self.t_sens = t_sens
        self.sens_x = sens_x
        self.sens_y = sens_y
        self.sens_z = sens_z

'''!
  bmm350 magnetometer dut tco structure
'''
class bmm350_dut_tco:
    """
    BMM350 magnetometer DUT TCO (Temperature Coefficient of Offset) structure.

    Stores TCO coefficients for X, Y, and Z axes.

    :param tco_x: X-axis TCO coefficient.
    :type tco_x: float
    :param tco_y: Y-axis TCO coefficient.
    :type tco_y: float
    :param tco_z: Z-axis TCO coefficient.
    :type tco_z: float
    """
    def __init__(self, tco_x: float, tco_y: float, tco_z: float):
        self.tco_x = tco_x
        self.tco_y = tco_y
        self.tco_z = tco_z
'''!
  bmm350 magnetometer dut tcs structure
'''
class bmm350_dut_tcs:
    """
    BMM350 magnetometer DUT TCS (Temperature Coefficient of Sensitivity) structure.

    Stores TCS coefficients for X, Y, and Z axes.

    :param tcs_x: X-axis TCS coefficient.
    :type tcs_x: float
    :param tcs_y: Y-axis TCS coefficient.
    :type tcs_y: float
    :param tcs_z: Z-axis TCS coefficient.
    :type tcs_z: float
    """
    def __init__(self, tcs_x: float, tcs_y: float, tcs_z: float):
        self.tcs_x = tcs_x
        self.tcs_y = tcs_y
        self.tcs_z = tcs_z

'''!
  bmm350 magnetometer cross axis compensation structure
'''
class bmm350_cross_axis:
    """
    BMM350 magnetometer cross-axis compensation structure.

    Stores cross-axis sensitivity coefficients.

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

'''!
  bmm350 magnetometer compensate structure
'''
class bmm350_mag_compensate:
    """
    BMM350 magnetometer compensation data structure.

    Holds all compensation coefficients required for sensor data correction.

    :param dut_offset_coef: DUT offset coefficients.
    :type dut_offset_coef: bmm350_dut_offset_coef
    :param dut_sensit_coef: DUT sensitivity coefficients.
    :type dut_sensit_coef: bmm350_dut_sensit_coef
    :param dut_tco: DUT TCO coefficients.
    :type dut_tco: bmm350_dut_tco
    :param dut_tcs: DUT TCS coefficients.
    :type dut_tcs: bmm350_dut_tcs
    :param dut_t0: DUT T0 (reference temperature).
    :type dut_t0: float
    :param cross_axis: Cross-axis compensation coefficients.
    :type cross_axis: bmm350_cross_axis
    """
    def __init__(self, dut_offset_coef: bmm350_dut_offset_coef, dut_sensit_coef: bmm350_dut_sensit_coef, dut_tco: bmm350_dut_tco, dut_tcs: bmm350_dut_tcs, dut_t0: float, cross_axis: bmm350_cross_axis):
        self.dut_offset_coef = dut_offset_coef
        self.dut_sensit_coef = dut_sensit_coef
        self.dut_tco = dut_tco
        self.dut_tcs = dut_tcs
        self.dut_t0 = dut_t0
        self.cross_axis = cross_axis

'''!
  bmm350 device structure
'''
class bmm350_dev:
  """
  BMM350 device structure.

  Represents the BMM350 sensor device, including its chip ID, OTP data,
  compensation data, power mode, and enabled axes.

  :param mag_comp: Magnetometer compensation data.
  :type mag_comp: bmm350_mag_compensate
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
  Structure to hold uncompensated (raw) BMM350 magnetometer and temperature data.

  Used internally to store raw ADC counts before any compensation or scaling.
  """
  def __init__(self):
    self.raw_x_data = 0
    self.raw_y_data = 0
    self.raw_z_data = 0
    self.raw_t_data = 0
_raw_mag_data = BMM350RawMagData()

class BMM350MagData:
  """
  Structure to hold compensated BMM350 magnetometer and temperature data.

  Stores the final X, Y, Z magnetic field values (typically in µT) and
  the temperature (typically in °C) after all compensations.
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

    Decodes the bits of the PMU_CMD_STATUS_0 register.

    :param pmu_cmd_busy: PMU command busy status.
    :type pmu_cmd_busy: int
    :param odr_ovwr: ODR (Output Data Rate) overwrite status.
    :type odr_ovwr: int
    :param avr_ovwr: Averaging overwrite status.
    :type avr_ovwr: int
    :param pwr_mode_is_normal: Power mode is normal status.
    :type pwr_mode_is_normal: int
    :param cmd_is_illegal: Command is illegal status.
    :type cmd_is_illegal: int
    :param pmu_cmd_value: PMU command value.
    :type pmu_cmd_value: int
    """
    def __init__(self, pmu_cmd_busy, odr_ovwr, avr_ovwr, pwr_mode_is_normal, cmd_is_illegal, pmu_cmd_value):
        self.pmu_cmd_busy = pmu_cmd_busy
        self.odr_ovwr = odr_ovwr
        self.avr_ovwr = avr_ovwr
        self.pwr_mode_is_normal = pwr_mode_is_normal
        self.cmd_is_illegal = cmd_is_illegal
        self.pmu_cmd_value = pmu_cmd_value
pmu_cmd_stat_0 = bmm350_pmu_cmd_status_0(pmu_cmd_busy=0, odr_ovwr=0, avr_ovwr=0, pwr_mode_is_normal=0, cmd_is_illegal=0, pmu_cmd_value=0)

# I2C interface
class Magnetometer(object):
  """
  Bosch BMM350 Magnetometer sensor class.

  Provides methods to initialize, configure, and read data from the BMM350 sensor
  using I2C communication. This class handles low-level register interactions,
  OTP data loading, power mode management, and data compensation.

  :param bus: The I2C bus number (e.g., 1 for Raspberry Pi's /dev/i2c-1).
  :type bus: int
  :param addr: The I2C address of the BMM350 sensor.
               Defaults to ``BMM350_I2C_ADDRESS_PRIMARY`` if not provided,
               but typically specified during instantiation.
  :type addr: int
  """
  I2C_MODE = 1
  def __init__(self, bus=1, addr=0x14):
    """
    Initialize the BMM350 sensor.

    Sets up I2C communication based on the detected platform (Raspberry Pi or other Linux/Windows).

    :param bus: The I2C bus number.
    :type bus: int
    :param addr: The I2C address of the BMM350 sensor.
    :type addr: int
    """
    self.__i2c_i3c = self.I2C_MODE
    self.bus = bus
    self.__addr = addr
    if self.is_raspberrypi():
      import smbus
      self.i2cbus = smbus.SMBus(bus)
    else:
      self.test_platform()

  def BMM350_SET_BITS(self, reg_data, bitname_msk, bitname_pos, data):
    """
    Set specific bits in a register value.

    :param reg_data: The original register value.
    :type reg_data: int
    :param bitname_msk: The mask for the bits to be set.
    :type bitname_msk: int
    :param bitname_pos: The starting position of the bits.
    :type bitname_pos: int
    :param data: The data to write to the specified bits.
    :type data: int
    :return: The modified register value.
    :rtype: int
    """
    return (reg_data & ~bitname_msk) | ((data << bitname_pos) & bitname_msk)

  def BMM350_GET_BITS(self, reg_data, mask, pos):
    """
    Get specific bits from a register value.

    :param reg_data: The register value.
    :type reg_data: int
    :param mask: The mask for the bits to be retrieved.
    :type mask: int
    :param pos: The starting position of the bits.
    :type pos: int
    :return: The value of the specified bits.
    :rtype: int
    """
    return (reg_data & mask) >> pos
 
  def BMM350_GET_BITS_POS_0(self, reg_data, mask):
    """
    Get specific bits from a register value, assuming bit position 0.

    :param reg_data: The register value.
    :type reg_data: int
    :param mask: The mask for the bits to be retrieved.
    :type mask: int
    :return: The value of the specified bits.
    :rtype: int
    """
    return reg_data & mask

  def BMM350_SET_BITS_POS_0(self, reg_data, mask, data):
    """
    Set specific bits in a register value, assuming bit position 0.

    :param reg_data: The original register value.
    :type reg_data: int
    :param mask: The mask for the bits to be set.
    :type mask: int
    :param data: The data to write to the specified bits.
    :type data: int
    :return: The modified register value.
    :rtype: int
    """
    return ((reg_data & ~(mask)) | (data & mask))  

  # This internal API converts the raw data from the IC data registers to signed integer
  def fix_sign(self, inval, number_of_bits):
    """
    Convert raw data from IC data registers to a signed integer.

    This internal API handles the conversion of unsigned ADC values to
    signed integer representations based on the number of bits.

    :param inval: The input unsigned integer value.
    :type inval: int
    :param number_of_bits: The number of bits representing the signed integer.
    :type number_of_bits: int
    :options number_of_bits:
        * ``BMM350_SIGNED_8_BIT``
        * ``BMM350_SIGNED_12_BIT``
        * ``BMM350_SIGNED_16_BIT``
        * ``BMM350_SIGNED_21_BIT``
        * ``BMM350_SIGNED_24_BIT``
    :return: The signed integer value.
    :rtype: int
    """
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

  # This internal API is used to update magnetometer offset and sensitivity data.
  def update_mag_off_sens(self):
    """
    Update magnetometer offset and sensitivity data from OTP values.

    This internal API reads the OTP (One-Time Programmable) memory data
    and populates the compensation structures (`bmm350_sensor.mag_comp`)
    with offset, sensitivity, TCO, TCS, T0, and cross-axis coefficients.
    """
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
    """
    Set the power mode of the BMM350 sensor.

    Transitions the sensor to the specified power mode (Suspend, Normal, Forced, Forced Fast).
    Includes necessary delays for mode transitions.

    :param powermode: The desired power mode.
    :type powermode: int
    :options powermode:
        * ``BMM350_SUSPEND_MODE``
        * ``BMM350_NORMAL_MODE``
        * ``BMM350_FORCED_MODE``
        * ``BMM350_FORCED_MODE_FAST``
    """
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
    """
    Perform a magnetic reset sequence and wait for completion.

    This sequence involves transitioning to suspend mode, issuing
    magnetic reset commands (BR and FGR), and then optionally
    restoring the previous normal mode if it was active.
    """
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

    Performs a software reset, verifies the chip ID, downloads OTP compensation data,
    updates internal compensation parameters, and performs a magnetic reset.

    :return: Status of the initialization.
    :rtype: int
    :retval BMM350_OK: Initialization successful.
    :retval BMM350_CHIP_ID_ERROR: Chip ID mismatch, initialization failed.
    """
  
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
      # The chip id verification failed and initialization failed.
      rslt = BMM350_CHIP_ID_ERROR
    return rslt


  def get_chip_id(self):
    """
    Read the chip ID from the BMM350 sensor.

    :return: The chip ID value.
    :rtype: int
    """
    chip_id = self.read_reg(BMM350_REG_CHIP_ID, 1)
    return chip_id[0]


  def soft_reset(self):
    """
    Perform a soft reset on the BMM350 sensor.

    Restores the sensor to suspend mode after the soft reset.
    This involves issuing a soft reset command, disabling OTP,
    performing a magnetic reset, and finally setting the power mode to suspend.
    The user needs to manually set the desired operation mode afterwards.
    """
    self.write_reg(BMM350_REG_CMD, BMM350_CMD_SOFTRESET) # Software reset
    time.sleep(BMM350_SOFT_RESET_DELAY)
    self.write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP) # Disable OTP
    self.bmm350_magnetic_reset_and_wait() # magnetic reset
    self.bmm350_set_powermode(BMM350_SUSPEND_MODE)


  def set_operation_mode(self, modes):
    """
    Set the sensor operation mode.

    Allows switching between suspend, normal, forced, and forced-fast modes.

    :param modes: The desired operation mode.
    :type modes: int
    :options modes:
        * ``BMM350_SUSPEND_MODE``: Suspend mode. Minimal current consumption.
                                   All registers are accessible. Default after power-on.
        * ``BMM350_NORMAL_MODE``: Normal mode. Continuous measurement at the configured ODR.
        * ``BMM350_FORCED_MODE``: Forced mode. Single measurement, then returns to suspend mode.
        * ``BMM350_FORCED_MODE_FAST``: Forced mode fast. Single measurement with faster ODR capability (up to 200Hz), then returns to suspend mode.
    """
    self.bmm350_set_powermode(modes)


  def get_operation_mode(self):
    """
    Get the current sensor operation mode.

    :return: A string describing the current operation mode.
    :rtype: str
    :retval "bmm350 is suspend mode!": Sensor is in suspend mode.
    :retval "bmm350 is normal mode!": Sensor is in normal mode.
    :retval "bmm350 is forced mode!": Sensor is in forced mode.
    :retval "bmm350 is forced_fast mode!": Sensor is in forced-fast mode.
    :retval "error mode!": Unknown or error in power mode status.
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

    :return: The current ODR in Hertz (Hz).
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
    Set a preset measurement mode combining averaging and ODR.

    This simplifies sensor configuration for common use cases by setting
    the number of averages and the output data rate.

    :param avg: The averaging setting (number of measurements to average).
    :type avg: int
    :options avg:
        * ``BMM350_AVERAGING_NO_AVG`` (No averaging)
        * ``BMM350_AVERAGING_2_AVG`` (2 averages)
        * ``BMM350_AVERAGING_4_AVG`` (4 averages)
        * ``BMM350_AVERAGING_8_AVG`` (8 averages)
    :param odr: The desired data output rate. Defaults to ``BMM350_DATA_RATE_12_5HZ``.
    :type odr: int, optional
    :options odr:
        * ``BMM350_DATA_RATE_1_5625HZ``
        * ``BMM350_DATA_RATE_3_125HZ``
        * ``BMM350_DATA_RATE_6_25HZ``
        * ``BMM350_DATA_RATE_12_5HZ``
        * ``BMM350_DATA_RATE_25HZ``
        * ``BMM350_DATA_RATE_50HZ``
        * ``BMM350_DATA_RATE_100HZ``
        * ``BMM350_DATA_RATE_200HZ``
        * ``BMM350_DATA_RATE_400HZ``
    """
    reg_data = (odr & BMM350_ODR_MSK)
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_AVG_MSK, BMM350_AVG_POS, avg)
    self.write_reg(BMM350_REG_PMU_CMD_AGGR_SET, reg_data)
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)  


  def self_test(self):
    """
    Perform a sensor self-test by checking enabled axes.

    This method reads the axis enable register and reports which axes (X, Y, Z)
    are currently enabled for measurement. It does not perform a full
    functional self-test of the sensor hardware.

    :return: A string indicating the self-test result based on enabled axes.
    :rtype: str
    :retval "x y z aix test success": If X, Y, and Z axes are enabled.
    :retval "<axes> aix test success": If a subset of axes are enabled (e.g., "x y ").
    :retval "xyz aix self test fail": If no axes are enabled.
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

    Allows individual control over which magnetic axes are active.
    Disabling an axis will result in invalid data for that axis.

    :param en_x: Enable state for the X-axis. Defaults to ``BMM350_X_EN``.
    :type en_x: int, optional
    :options en_x:
        * ``BMM350_X_EN``: Enable X-axis measurement.
        * ``BMM350_X_DIS``: Disable X-axis measurement.
    :param en_y: Enable state for the Y-axis. Defaults to ``BMM350_Y_EN``.
    :type en_y: int, optional
    :options en_y:
        * ``BMM350_Y_EN``: Enable Y-axis measurement.
        * ``BMM350_Y_DIS``: Disable Y-axis measurement.
    :param en_z: Enable state for the Z-axis. Defaults to ``BMM350_Z_EN``.
    :type en_z: int, optional
    :options en_z:
        * ``BMM350_Z_EN``: Enable Z-axis measurement.
        * ``BMM350_Z_DIS``: Disable Z-axis measurement.
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
    Get the current enable status for X, Y, and Z axes measurements.

    :return: A string describing the enable status of each axis.
    :rtype: str
    :exretval "The x axis is enable! The y axis is enable! The z axis is enable! "
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
    Get raw X, Y, Z magnetic data before OTP and software compensations.

    This method reads the raw 24-bit ADC values for the X, Y, and Z magnetic axes.
    It is intended for use during sensor calibration procedures to collect
    uncompensated data.

    :return: A list containing the raw signed integer values for [X, Y, Z].
             Returns ``None`` if the read operation fails or returns insufficient data.
    :rtype: list[int] or None
    """
    # 1. Read raw data registers for X, Y, Z axes
    # BMM350_MAG_TEMP_DATA_LEN is 12 (Xlsb,Xmsb,Xmmsb, Ylsb,Ymsb,Ymmsb, Zlsb,Zmsb,Zmmsb, Tlsb,Tmsb,Tmmsb)
    mag_data_regs = self.read_reg(BMM350_REG_MAG_X_XLSB, 9)
    if mag_data_regs is None or len(mag_data_regs) < 9:
        # Handle error case if read_reg fails or returns insufficient data
        print("Failed to read raw magnetometer data for calibration.")
        return None

    raw_mag_x_u = mag_data_regs[0] + (mag_data_regs[1] << 8) + (mag_data_regs[2] << 16)
    raw_mag_y_u = mag_data_regs[3] + (mag_data_regs[4] << 8) + (mag_data_regs[5] << 16)
    raw_mag_z_u = mag_data_regs[6] + (mag_data_regs[7] << 8) + (mag_data_regs[8] << 16)

    # Apply sign correction using the existing fix_sign method
    # BMM350_SIGNED_24_BIT is used in get_xyz for these raw values
    raw_x = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    raw_y = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    raw_z = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    
    return [raw_x, raw_y, raw_z]

  def get_xyz(self):
    """
    Get calibrated geomagnetic data (X, Y, Z) in microTeslas (µT) and update sensor temperature.

    This method performs the following steps:
    1. Reads raw 24-bit data for X, Y, Z magnetic axes and temperature.
    2. Applies sign correction to the raw data.
    3. Stores raw LSB values in `_raw_mag_data`.
    4. Applies user-defined hard iron calibration offsets (`_CALIBRATION_HARD_IRON`).
    5. Applies user-defined soft iron calibration transformation (`_CALIBRATION_SOFT_IRON_TRANSFORM`).
    6. Scales the corrected vector to physical units (µT) using `_LOCAL_EARTH_FIELD_UT`.
    7. Stores the final calibrated X, Y, Z values in `_mag_data`.
    8. Calculates and updates the sensor temperature in `_mag_data.temperature` using OTP compensation values.

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
        # _mag_data.x = _mag_data.y = _mag_data.z = _mag_data.temperature = BMM350_FLOAT_DATA_ERROR
        return [BMM350_FLOAT_DATA_ERROR] * 4

    raw_mag_x_u = mag_and_temp_regs[0] + (mag_and_temp_regs[1] << 8) + (mag_and_temp_regs[2] << 16)
    raw_mag_y_u = mag_and_temp_regs[3] + (mag_and_temp_regs[4] << 8) + (mag_and_temp_regs[5] << 16)
    raw_mag_z_u = mag_and_temp_regs[6] + (mag_and_temp_regs[7] << 8) + (mag_and_temp_regs[8] << 16)
    raw_temp_u  = mag_and_temp_regs[9] + (mag_and_temp_regs[10] << 8) + (mag_and_temp_regs[11] << 16)

    raw_x_lsb = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    raw_y_lsb = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    raw_z_lsb = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    
    # Access _raw_mag_data correctly (e.g., self._raw_mag_data or global _raw_mag_data)
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

  def get_x(self):
    """
    Get the calibrated X-axis geomagnetic data.

    This is a convenience method that calls `get_xyz` and returns only the X component.

    :return: Calibrated X-axis magnetic field strength in microTeslas (µT).
             Returns ``BMM350_FLOAT_DATA_ERROR`` if data retrieval fails.
    :rtype: float
    """
    # Ensure _mag_data is updated by calling get_xyz if it hasn't been called recently
    # or if a fresh read is desired. For simplicity, we can call it here, though
    # it might be redundant if get_xyz was just called.
    # Alternatively, rely on _mag_data.x being up-to-date from a previous call.
    self.get_xyz() # This updates _mag_data.x
    return _mag_data.x

  def get_y(self):
    """
    Get the calibrated Y-axis geomagnetic data.

    This is a convenience method that calls `get_xyz` and returns only the Y component.

    :return: Calibrated Y-axis magnetic field strength in microTeslas (µT).
             Returns ``BMM350_FLOAT_DATA_ERROR`` if data retrieval fails.
    :rtype: float
    """
    self.get_xyz()
    return _mag_data.y

  def get_z(self):
    """
    Get the calibrated Z-axis geomagnetic data.

    This is a convenience method that calls `get_xyz` and returns only the Z component.

    :return: Calibrated Z-axis magnetic field strength in microTeslas (µT).
             Returns ``BMM350_FLOAT_DATA_ERROR`` if data retrieval fails.
    :rtype: float
    """
    self.get_xyz()
    return _mag_data.z

  def get_t(self):
    """
    Get the sensor temperature.

    This is a convenience method that calls `get_xyz` (which also calculates temperature)
    and returns the temperature.

    :return: Sensor temperature in degrees Celsius (°C).
             Returns ``BMM350_FLOAT_DATA_ERROR`` if data retrieval fails.
    :rtype: float
    """
    self.get_xyz()
    return _mag_data.temperature

  def get_compass_degree(self):
    """
    Get compass degree (yaw/heading) assuming the sensor's XY plane is horizontal.

    Calculates the yaw angle based on the calibrated X and Y geomagnetic components.
    The angle is measured counter-clockwise from the sensor's positive X-axis
    to the projection of the magnetic field vector in the XY plane.

    .. note::
        This calculation is **not** tilt-compensated; To derive True North heading, apply your real-world 
        declination constant post-function call. 
    .. warning::
        This function is still in development, due to the complexity associated with hard-iron, soft-iron, and tilt 
        compensation calibration. 

    :return: Compass degree (0.0 to 360.0 degrees).
             Returns ``BMM350_FLOAT_DATA_ERROR`` (e.g., `float('nan')`) if
             magnetometer data is invalid.
    :rtype: float
    """
    magData = self.get_xyz()

    if magData is None or any(math.isnan(comp) for comp in magData):
        print("Warning: Invalid magnetometer data received for compass calculation.")
        return BMM350_FLOAT_DATA_ERROR
    
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
    Configure the Data Ready (DRDY) interrupt pin.

    Enables or disables the DRDY interrupt and sets its polarity.
    When enabled, the DRDY pin signals when new data is available.

    :param modes: Enable or disable the DRDY interrupt.
    :type modes: int
    :options modes:
        * ``BMM350_ENABLE_INTERRUPT``: Enable DRDY interrupt.
        * ``BMM350_DISABLE_INTERRUPT``: Disable DRDY interrupt.
    :param polarity: Polarity of the DRDY interrupt pin.
    :type polarity: int
    :options polarity:
        * ``BMM350_ACTIVE_HIGH``: Active high polarity. Pin goes high on interrupt.
        * ``BMM350_ACTIVE_LOW``: Active low polarity. Pin goes low on interrupt.
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

    Checks if the DRDY (Data Ready) bit is set, indicating new data is available.

    :return: ``True`` if data is ready, ``False`` otherwise.
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

    Sets up an interrupt that triggers when the geomagnetic value of a channel
    crosses a defined threshold (either low or high).
    The threshold value is effectively scaled by 16 internally.

    :param modes: The type of threshold interrupt.
    :type modes: int
    :options modes:
        * ``LOW_THRESHOLD_INTERRUPT``: Interrupt when data is below the threshold.
        * ``HIGH_THRESHOLD_INTERRUPT``: Interrupt when data is above the threshold (Note: current implementation logic for HIGH seems same as LOW, review needed).
    :param threshold: The threshold value. The actual trigger threshold will be `threshold * 16`.
    :type threshold: int or float
    :param polarity: Polarity of the interrupt pin.
    :type polarity: int
    :options polarity:
        * ``POLARITY_HIGH`` (or ``BMM350_ACTIVE_HIGH``): Active high.
        * ``POLARITY_LOW`` (or ``BMM350_ACTIVE_LOW``): Active low.
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
    Get the data that caused a threshold interrupt.

    If a threshold interrupt has occurred (checked via `get_data_ready_state`),
    this method returns the geomagnetic data components (X, Y, Z) that crossed
    the configured threshold.

    :return: A list of three elements. Each element is the magnetic data for an axis (X, Y, Z)
             if it triggered the interrupt, otherwise it's ``NO_DATA``.
             Example: `[mag_x, NO_DATA, mag_z]` if X and Z triggered the interrupt.
    :rtype: list[float or int]
    """
    Data = [NO_DATA] * 3
    state = self.get_data_ready_state()
    if state == True:
      magData = self.get_xyz()
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

  def is_raspberrypi(self):
    """
    Check if the current platform is a Raspberry Pi.

    :return: ``True`` if a Raspberry Pi is detected, ``False`` otherwise.
    :rtype: bool
    """
    import io
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower(): return True
    except Exception: pass
    return False

  def test_platform(self):
    """
    Detects the platform (Linux or Windows) and initializes the I2C bus accordingly.

    For Linux, it attempts to find an `i2c-tiny-usb` adapter if a standard bus is not found.
    For Windows, it uses `i2c_mp_usb`.
    Prints a message if the platform is not supported.
    """
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
    Writes a byte of data to a specified register.

    Handles I2C communication errors by printing a message and retrying after a delay.

    :param reg: The register address to write to.
    :type reg: int
    :param data: The byte of data to write.
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
  
  def read_reg(self, reg ,len):
    """
    Reads a specified number of bytes from a register.

    Handles I2C communication, including dummy bytes if required by the BMM350 protocol.
    Retries on exception.

    :param reg: The register address to read from.
    :type reg: int
    :param len: The number of bytes to read (excluding dummy bytes).
    :type len: int
    :return: A list of bytes read from the register.
    :rtype: list[int]
    """
    while True:
      try:
        # Read data from I2C bus
        temp_buf = self.i2cbus.read_i2c_block_data(self.__addr, reg, len + BMM350_DUMMY_BYTES)
        # Copy data after dummy byte indices
        reg_data = temp_buf[BMM350_DUMMY_BYTES:]
        return reg_data  # Assuming this function is part of a larger method
      except Exception as e:
        time.sleep(1)
        print("please check connect r!")
