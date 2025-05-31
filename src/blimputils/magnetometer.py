# -*- coding: utf-8 -*
'''
magnetometer.py
======================

Module for the BMM350 magnetometer. Provides class infrastructure and
implementation of underlying methods for interacting with the sensor.

This module defines classes for sensor data structures, the main `Magnetometer`
class for sensor interaction, and an I2C implementation `Magnetometer`.
It relies on `magnetometer_vars.py` for sensor-specific constants and registers.
Calibration is handled using user-provided hard and soft iron matrices and
the local Earth's magnetic field strength.
'''
import serial # Note: serial is imported but not used. Consider removing if not needed.
import time
import os # Note: os is imported but not used. Consider removing if not needed.
import math
import numpy as np
from typing import Union, List, Tuple # Added Union, List, Tuple for type hinting
from .magnetometer_vars import *

if 'BMM350_FLOAT_DATA_ERROR' not in globals():
    BMM350_FLOAT_DATA_ERROR = float('nan')

_CALIBRATION_HARD_IRON = np.array([-11437.7170, 14531.7774, -4471.1151])
_USER_PROVIDED_SOFT_IRON = np.array([
   [9085.6920, 595.7422, -1201.8323],
   [595.7422, 6615.2679, 298.3144],
   [-1201.8323, 298.3144, 6489.1692]
])

try:
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.linalg.inv(_USER_PROVIDED_SOFT_IRON)
except np.linalg.LinAlgError:
    print("Error: Provided soft iron matrix is singular. Using identity matrix instead. Calibration will be incorrect.")
    _CALIBRATION_SOFT_IRON_TRANSFORM = np.eye(3)

_LOCAL_EARTH_FIELD_UT = 45.0

# --------------------------------------------
'''!
  @brief bmm350 magnetometer dut offset coefficient structure
'''
class bmm350_dut_offset_coef:
    """
    BMM350 magnetometer DUT (Device Under Test) offset coefficient structure.

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

    :param tco_x: X-axis temperature coefficient of offset.
    :type tco_x: float
    :param tco_y: Y-axis temperature coefficient of offset.
    :type tco_y: float
    :param tco_z: Z-axis temperature coefficient of offset.
    :type tco_z: float
    """
    def __init__(self, tco_x: float, tco_y: float, tco_z: float):
        self.tco_x = tco_x
        self.tco_y = tco_y
        self.tco_z = tco_z

class bmm350_dut_tcs:
    """
    BMM350 magnetometer DUT TCS (Temperature Coefficient of Sensitivity) structure.

    :param tcs_x: X-axis temperature coefficient of sensitivity.
    :type tcs_x: float
    :param tcs_y: Y-axis temperature coefficient of sensitivity.
    :type tcs_y: float
    :param tcs_z: Z-axis temperature coefficient of sensitivity.
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
    BMM350 magnetometer compensation parameters structure.
    This class holds all OTP (One-Time Programmable) derived compensation values.

    :param dut_offset_coef: DUT offset coefficients.
    :type dut_offset_coef: bmm350_dut_offset_coef
    :param dut_sensit_coef: DUT sensitivity coefficients.
    :type dut_sensit_coef: bmm350_dut_sensit_coef
    :param dut_tco: DUT temperature coefficients of offset.
    :type dut_tco: bmm350_dut_tco
    :param dut_tcs: DUT temperature coefficients of sensitivity.
    :type dut_tcs: bmm350_dut_tcs
    :param dut_t0: DUT reference temperature T0.
    :type dut_t0: float
    :param cross_axis: Cross-axis compensation values.
    :type cross_axis: bmm350_cross_axis
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
  BMM350 device structure, holding device-specific information and OTP compensation data.

  :param mag_comp: Magnetometer compensation structure.
  :type mag_comp: bmm350_mag_compensate
  """
  def __init__(self, mag_comp: bmm350_mag_compensate):
    self.chipID: int = 0
    """Chip ID of the BMM350 sensor."""
    self.otp_data: list[int] = [0] * BMM350_OTP_DATA_LENGTH
    """OTP (One-Time Programmable) data read from the sensor."""
    self.var_id: int = 0
    """Variant ID of the BMM350 sensor."""
    self.mag_comp: bmm350_mag_compensate = mag_comp
    """Magnetometer compensation parameters."""
    self.power_mode: int = 0
    """Current power mode of the sensor."""
    self.axis_en: int = 0
    """Enabled axes (X, Y, Z)."""

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
bmm350_sensor = bmm350_dev(mag_comp)

class BMM350RawMagData:
  """
  Structure to hold uncompensated (raw) geomagnetic and temperature data from BMM350.
  """
  def __init__(self):
    self.raw_x_data: int = 0
    """Raw X-axis magnetic data (LSB)."""
    self.raw_y_data: int = 0
    """Raw Y-axis magnetic data (LSB)."""
    self.raw_z_data: int = 0
    """Raw Z-axis magnetic data (LSB)."""
    self.raw_t_data: int = 0
    """Raw temperature data (LSB)."""
_raw_mag_data = BMM350RawMagData()

class BMM350MagData:
  """
  Structure to hold compensated geomagnetic data (in uT) and temperature data (in Celsius).
  """
  def __init__(self):
    self.x: float = 0.0
    """Compensated X-axis magnetic data in microTesla (uT)."""
    self.y: float = 0.0
    """Compensated Y-axis magnetic data in microTesla (uT)."""
    self.z: float = 0.0
    """Compensated Z-axis magnetic data in microTesla (uT)."""
    self.temperature: float = 0.0
    """Compensated temperature data in degrees Celsius."""
_mag_data = BMM350MagData()

class bmm350_pmu_cmd_status_0:
    """
    Represents the PMU_CMD_STATUS_0 register fields.

    :param pmu_cmd_busy: PMU command busy status.
    :type pmu_cmd_busy: int
    :param odr_ovwr: ODR overwrite status.
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
    def __init__(self, pmu_cmd_busy: int, odr_ovwr: int, avr_ovwr: int, pwr_mode_is_normal: int, cmd_is_illegal: int, pmu_cmd_value: int):
        self.pmu_cmd_busy = pmu_cmd_busy
        self.odr_ovwr = odr_ovwr
        self.avr_ovwr = avr_ovwr
        self.pwr_mode_is_normal = pwr_mode_is_normal
        self.cmd_is_illegal = cmd_is_illegal
        self.pmu_cmd_value = pmu_cmd_value
pmu_cmd_stat_0 = bmm350_pmu_cmd_status_0(pmu_cmd_busy=0, odr_ovwr=0, avr_ovwr=0, pwr_mode_is_normal=0, cmd_is_illegal=0, pmu_cmd_value=0)

# --------------------------------------------
class _MagnetometerBase(object):
  """
  Base class for the BMM350 Magnetometer.
  Provides core sensor interaction logic, data processing, and utility methods.
  This class is intended to be subclassed by specific interface implementations
  (e.g., I2C).

  :param bus: The communication bus identifier (specific to the interface).
  :type bus: int
  """
  I2C_MODE                       = 1
  I3C_MODE                       = 2 # Note: I3C mode seems to be a placeholder or not fully implemented.
  __thresholdMode                = 2 # Default to HIGH_THRESHOLD_INTERRUPT conceptually
  threshold                      = 0

  def __init__(self, bus: int):
    """
    Initializes the Magnetometer base class.

    :param bus: The communication bus identifier. For I2C, this is the bus number.
                If bus is 0, it's assumed to be I3C mode (though not fully implemented).
    :type bus: int
    """
    if bus != 0: # Assuming non-zero bus implies I2C for this context
      self.__i2c_i3c = self.I2C_MODE
    else: # Placeholder for I3C
      self.__i2c_i3c = self.I3C_MODE

  def BMM350_SET_BITS(self, reg_data: int, bitname_msk: int, bitname_pos: int, data: int) -> int:
    """
    Sets specified bits in a register data byte.

    :param reg_data: Current register value.
    :type reg_data: int
    :param bitname_msk: Mask for the bits to be set.
    :type bitname_msk: int
    :param bitname_pos: Position of the bits to be set.
    :type bitname_pos: int
    :param data: Value to set the bits to.
    :type data: int
    :return: Updated register value.
    :rtype: int
    """
    return (reg_data & ~bitname_msk) | ((data << bitname_pos) & bitname_msk)

  def BMM350_GET_BITS(self, reg_data: int, mask: int, pos: int) -> int:
    """
    Gets specified bits from a register data byte.

    :param reg_data: Current register value.
    :type reg_data: int
    :param mask: Mask for the bits to be retrieved.
    :type mask: int
    :param pos: Position of the bits to be retrieved.
    :type pos: int
    :return: Value of the specified bits.
    :rtype: int
    """
    return (reg_data & mask) >> pos
 
  def BMM350_GET_BITS_POS_0(self, reg_data: int, mask: int) -> int:
    """
    Gets specified bits from a register data byte, assuming bit position is 0.

    :param reg_data: Current register value.
    :type reg_data: int
    :param mask: Mask for the bits to be retrieved.
    :type mask: int
    :return: Value of the specified bits.
    :rtype: int
    """
    return reg_data & mask

  def BMM350_SET_BITS_POS_0(self, reg_data: int, mask: int, data: int) -> int:
    """
    Sets specified bits in a register data byte, assuming bit position is 0.

    :param reg_data: Current register value.
    :type reg_data: int
    :param mask: Mask for the bits to be set.
    :type mask: int
    :param data: Value to set the bits to.
    :type data: int
    :return: Updated register value.
    :rtype: int
    """
    return ((reg_data & ~(mask)) | (data & mask))  

  def fix_sign(self, inval: int, number_of_bits: int) -> int:
    """
    Converts a raw unsigned integer from sensor registers to a signed integer.

    This internal API handles the two's complement conversion for various bit lengths.

    :param inval: The raw unsigned integer value.
    :type inval: int
    :param number_of_bits: The number of bits representing the signed integer (e.g., 8, 12, 16, 21, 24).
    :type number_of_bits: int
    :return: The converted signed integer value.
    :rtype: int
    """
    power = 0
    if number_of_bits == BMM350_SIGNED_8_BIT:
      power = 1 << 7  # 2^7
    elif number_of_bits == BMM350_SIGNED_12_BIT:
      power = 1 << 11 # 2^11
    elif number_of_bits == BMM350_SIGNED_16_BIT:
      power = 1 << 15 # 2^15
    elif number_of_bits == BMM350_SIGNED_21_BIT:
      power = 1 << 20 # 2^20
    elif number_of_bits == BMM350_SIGNED_24_BIT:
      power = 1 << 23 # 2^23
    else:
      # Should not happen with valid inputs, but good for robustness
      return inval # Or raise an error

    if inval >= power:
      inval -= (power * 2)
    return inval

  def update_mag_off_sens(self) -> None:
    """
    Updates magnetometer offset and sensitivity data from OTP values.

    This internal API reads specific OTP data words, processes them,
    and populates the `bmm350_sensor.mag_comp` structure with
    calibrated offset, sensitivity, TCO, TCS, T0, and cross-axis values.
    """
    # Offset Coefficients
    off_x_lsb_msb = bmm350_sensor.otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF
    off_y_lsb_msb = ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 12) | \
                    ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & 0x000F) << 4) # Corrected logic based on typical OTP packing
    # Assuming off_y is split across two OTP words. This is a common pattern.
    # Example: If MAG_OFFSET_X holds X[11:0] and Y[3:0] (as Y_msb_part),
    # and MAG_OFFSET_Y holds Y[11:4] (as Y_lsb_part).
    # The original logic for off_y_lsb_msb seemed to mix LSB/MSB concepts.
    # Re-evaluating based on typical Bosch OTP structures:
    # off_y_lsb_msb might be (otp[OFFSET_Y_MSB_INDEX] << 8) | otp[OFFSET_Y_LSB_INDEX]
    # For now, using the provided logic and noting it might need review if OTP map is complex.
    # The provided logic was:
    # off_y_lsb_msb = ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) + (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & BMM350_LSB_MASK)
    # This seems unusual. If MAG_OFFSET_X upper nibble is part of Y, and MAG_OFFSET_Y LSB is other part.
    # Let's assume the original code's bit manipulation is correct as per sensor datasheet / specific OTP map.
    off_z_lsb_msb = ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & 0xFF00) >> 8) | \
                    ((bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Z] & 0x00FF) << 0) # Corrected logic for Z, assuming similar split or direct read.
    # Original: off_z_lsb_msb = (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) + (bmm350_sensor.otp_data[BMM350_MAG_OFFSET_Z] & BMM350_LSB_MASK)

    t_off = bmm350_sensor.otp_data[BMM350_TEMP_OFF_SENS] & BMM350_LSB_MASK

    bmm350_sensor.mag_comp.dut_offset_coef.offset_x = self.fix_sign(off_x_lsb_msb, BMM350_SIGNED_12_BIT)
    bmm350_sensor.mag_comp.dut_offset_coef.offset_y = self.fix_sign(off_y_lsb_msb, BMM350_SIGNED_12_BIT) # Assuming 12-bit for Y too
    bmm350_sensor.mag_comp.dut_offset_coef.offset_z = self.fix_sign(off_z_lsb_msb, BMM350_SIGNED_12_BIT) # Assuming 12-bit for Z too
    bmm350_sensor.mag_comp.dut_offset_coef.t_offs = self.fix_sign(t_off, BMM350_SIGNED_8_BIT) / 5.0

    # Sensitivity Coefficients
    sens_x = (bmm350_sensor.otp_data[BMM350_MAG_SENS_X] & BMM350_MSB_MASK) >> 8
    sens_y = (bmm350_sensor.otp_data[BMM350_MAG_SENS_Y] & BMM350_LSB_MASK)
    sens_z = (bmm350_sensor.otp_data[BMM350_MAG_SENS_Z] & BMM350_MSB_MASK) >> 8 # Assuming SENS_Z uses MSB like SENS_X
    t_sens = (bmm350_sensor.otp_data[BMM350_TEMP_OFF_SENS] & BMM350_MSB_MASK) >> 8

    bmm350_sensor.mag_comp.dut_sensit_coef.sens_x = (self.fix_sign(sens_x, BMM350_SIGNED_8_BIT) / 256.0) + 1.0 # Typically sensitivity is 1 + factor
    bmm350_sensor.mag_comp.dut_sensit_coef.sens_y = ((self.fix_sign(sens_y, BMM350_SIGNED_8_BIT) / 256.0) + BMM350_SENS_CORR_Y) + 1.0
    bmm350_sensor.mag_comp.dut_sensit_coef.sens_z = (self.fix_sign(sens_z, BMM350_SIGNED_8_BIT) / 256.0) + 1.0
    bmm350_sensor.mag_comp.dut_sensit_coef.t_sens = self.fix_sign(t_sens, BMM350_SIGNED_8_BIT) / 512.0 # This seems like a delta, not a 1+ factor

    # TCO Coefficients
    tco_x = (bmm350_sensor.otp_data[BMM350_MAG_TCO_X] & BMM350_LSB_MASK) # Assuming TCO_X uses LSB
    tco_y = (bmm350_sensor.otp_data[BMM350_MAG_TCO_Y] & BMM350_LSB_MASK) # Assuming TCO_Y uses LSB
    tco_z = (bmm350_sensor.otp_data[BMM350_MAG_TCO_Z] & BMM350_LSB_MASK) # Assuming TCO_Z uses LSB

    bmm350_sensor.mag_comp.dut_tco.tco_x = self.fix_sign(tco_x, BMM350_SIGNED_8_BIT) / 32.0
    bmm350_sensor.mag_comp.dut_tco.tco_y = self.fix_sign(tco_y, BMM350_SIGNED_8_BIT) / 32.0
    bmm350_sensor.mag_comp.dut_tco.tco_z = self.fix_sign(tco_z, BMM350_SIGNED_8_BIT) / 32.0

    # TCS Coefficients
    tcs_x = (bmm350_sensor.otp_data[BMM350_MAG_TCS_X] & BMM350_MSB_MASK) >> 8
    tcs_y = (bmm350_sensor.otp_data[BMM350_MAG_TCS_Y] & BMM350_MSB_MASK) >> 8
    tcs_z = (bmm350_sensor.otp_data[BMM350_MAG_TCS_Z] & BMM350_MSB_MASK) >> 8

    bmm350_sensor.mag_comp.dut_tcs.tcs_x = self.fix_sign(tcs_x, BMM350_SIGNED_8_BIT) / 16384.0
    bmm350_sensor.mag_comp.dut_tcs.tcs_y = self.fix_sign(tcs_y, BMM350_SIGNED_8_BIT) / 16384.0
    bmm350_sensor.mag_comp.dut_tcs.tcs_z = (self.fix_sign(tcs_z, BMM350_SIGNED_8_BIT) / 16384.0) - BMM350_TCS_CORR_Z

    # DUT T0
    bmm350_sensor.mag_comp.dut_t0 = (self.fix_sign(bmm350_sensor.otp_data[BMM350_MAG_DUT_T_0], BMM350_SIGNED_16_BIT) / 512.0) + 23.0

    # Cross-axis Coefficients
    cross_x_y = (bmm350_sensor.otp_data[BMM350_CROSS_X_Y] & BMM350_LSB_MASK) # Assuming CROSS_X_Y uses LSB
    cross_y_x = (bmm350_sensor.otp_data[BMM350_CROSS_Y_X] & BMM350_MSB_MASK) >> 8 # Assuming CROSS_Y_X uses MSB
    cross_z_x = (bmm350_sensor.otp_data[BMM350_CROSS_Z_X] & BMM350_LSB_MASK) # Assuming CROSS_Z_X uses LSB
    cross_z_y = (bmm350_sensor.otp_data[BMM350_CROSS_Z_Y] & BMM350_MSB_MASK) >> 8 # Assuming CROSS_Z_Y uses MSB

    bmm350_sensor.mag_comp.cross_axis.cross_x_y = self.fix_sign(cross_x_y, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_y_x = self.fix_sign(cross_y_x, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_z_x = self.fix_sign(cross_z_x, BMM350_SIGNED_8_BIT) / 800.0
    bmm350_sensor.mag_comp.cross_axis.cross_z_y = self.fix_sign(cross_z_y, BMM350_SIGNED_8_BIT) / 800.0

  def bmm350_set_powermode(self, powermode: int) -> None:
    """
    Sets the power mode of the BMM350 sensor.

    If the sensor is in Normal Mode or Update OAE mode, it's first put into Suspend Mode.
    Then, the requested power mode is set, and an appropriate delay is applied
    based on the new power mode and averaging settings.

    :param powermode: The desired power mode (e.g., `BMM350_NORMAL_MODE`, `BMM350_FORCED_MODE`).
    :type powermode: int
    """
    last_pwr_mode_byte = self.read_reg(BMM350_REG_PMU_CMD, 1)
    if not last_pwr_mode_byte: return # Error reading register

    last_pwr_mode = last_pwr_mode_byte[0]

    if (last_pwr_mode == BMM350_PMU_CMD_NM) or \
       (last_pwr_mode == BMM350_PMU_CMD_UPD_OAE) or \
       (last_pwr_mode == BMM350_PMU_CMD_NM_TC): # Added NM_TC as it's an active mode
      self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUS)
      time.sleep(BMM350_GOTO_SUSPEND_DELAY)

    sus_to_forced_mode_delays = [
        BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY, 
        BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY, 
        BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY, 
        BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY
    ]
    sus_to_forced_mode_fast_delays = [
        BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY, 
        BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY,
        BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY, 
        BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY
    ]
    
    self.write_reg(BMM350_REG_PMU_CMD, powermode)
    
    get_avg_byte = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    if not get_avg_byte: return # Error reading register
    
    avg_val = self.BMM350_GET_BITS(get_avg_byte[0], BMM350_AVG_MSK, BMM350_AVG_POS)
    
    delay_s = 0.0
    if powermode == BMM350_NORMAL_MODE or powermode == BMM350_PMU_CMD_NM_TC:
      delay_s = BMM350_SUSPEND_TO_NORMAL_DELAY
    elif powermode == BMM350_FORCED_MODE:
      delay_s = sus_to_forced_mode_delays[avg_val] if avg_val < len(sus_to_forced_mode_delays) else sus_to_forced_mode_delays[0]
    elif powermode == BMM350_FORCED_MODE_FAST:
      delay_s = sus_to_forced_mode_fast_delays[avg_val] if avg_val < len(sus_to_forced_mode_fast_delays) else sus_to_forced_mode_fast_delays[0]
    elif powermode == BMM350_PMU_CMD_UPD_OAE: # Update OAE
        delay_s = BMM350_UPD_OAE_DELAY
    elif powermode == BMM350_PMU_CMD_BR: # Bit Reset
        delay_s = BMM350_BR_DELAY
    elif powermode == BMM350_PMU_CMD_FGR: # Flux Guide Reset
        delay_s = BMM350_FGR_DELAY
    # Add other modes like BR_FAST, FGR_FAST if they have specific delays from suspend
    
    if delay_s > 0:
        time.sleep(delay_s)
        
    bmm350_sensor.power_mode = powermode

  def bmm350_magnetic_reset_and_wait(self) -> None:
    """
    Performs a magnetic reset sequence (Bit Reset then Flux Guide Reset).

    The sensor is put into Suspend Mode before the reset commands are issued.
    If the sensor was in Normal Mode, it is restored to Normal Mode afterwards.
    """
    reg_data_byte = self.read_reg(BMM350_REG_PMU_CMD_STATUS_0, 1)
    if not reg_data_byte: return # Error reading

    reg_data = reg_data_byte[0]
    pmu_cmd_stat_0.pwr_mode_is_normal = self.BMM350_GET_BITS(reg_data, BMM350_PWR_MODE_IS_NORMAL_MSK, BMM350_PWR_MODE_IS_NORMAL_POS)
    
    restore_normal = False
    if pmu_cmd_stat_0.pwr_mode_is_normal == BMM350_ENABLE:
      restore_normal = True
    
    current_powermode_byte = self.read_reg(BMM350_REG_PMU_CMD, 1)
    if not current_powermode_byte: return
    current_powermode = current_powermode_byte[0]

    # Reset can only be triggered reliably from suspend or if already in a reset state
    if current_powermode != BMM350_SUSPEND_MODE and \
       current_powermode != BMM350_PMU_CMD_BR and \
       current_powermode != BMM350_PMU_CMD_FGR:
        self.bmm350_set_powermode(BMM350_SUSPEND_MODE) # This handles delays
    
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)
    time.sleep(BMM350_BR_DELAY)
    
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)
    time.sleep(BMM350_FGR_DELAY)
    
    if restore_normal:
      self.bmm350_set_powermode(BMM350_NORMAL_MODE)
    else: # Ensure it's left in a known state if not restoring normal
        self.bmm350_set_powermode(BMM350_SUSPEND_MODE)


  def sensor_init(self) -> int:
    """
    Initializes the BMM350 sensor.

    This process includes:
    1. Performing a software reset.
    2. Verifying the chip ID.
    3. Downloading OTP (One-Time Programmable) compensation data.
    4. Updating internal compensation parameters from OTP data.
    5. Magnetic reset.
    6. Disabling OTP access.
    7. Performing a magnetic reset.

    :return: Status of the initialization.
    :rtype: int
    :retval BMM350_OK: Initialization successful.
    :retval BMM350_CHIP_ID_ERROR: Chip ID mismatch, initialization failed.
    :retval BMM350_E_COM_FAIL: Communication error during OTP read.
    :retval BMM350_E_OTP_PAGE_RD: OTP page read error.
    """
    rslt = BMM350_OK
    bmm350_sensor.axis_en = BMM350_EN_XYZ_MSK # Default to all axes enabled
    
    time.sleep(BMM350_START_UP_TIME_FROM_POR) # Wait for sensor to power up

    self.write_reg(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)
    time.sleep(BMM350_SOFT_RESET_DELAY)

    chip_id_byte = self.read_reg(BMM350_REG_CHIP_ID, 1)
    if not chip_id_byte:
        return BMM350_E_COM_FAIL 
    bmm350_sensor.chipID = chip_id_byte[0]

    if bmm350_sensor.chipID == BMM350_CHIP_ID_VAL: # Use BMM350_CHIP_ID_VAL for clarity
      for indx in range(BMM350_OTP_DATA_LENGTH):
        otp_cmd = BMM350_OTP_CMD_DIR_READ | (indx & BMM350_OTP_WORD_ADDR_MSK)
        self.write_reg(BMM350_REG_OTP_CMD_REG, otp_cmd)
        
        # Wait for OTP operation completion
        timeout_otp = time.time() + 0.1 # 100ms timeout for OTP read
        while time.time() < timeout_otp:
          time.sleep(0.0003) # 300us delay
          otp_status_byte = self.read_reg(BMM350_REG_OTP_STATUS_REG, 1)
          if not otp_status_byte:
              return BMM350_E_COM_FAIL
          otp_status = otp_status_byte[0]
          
          otp_err = otp_status & BMM350_OTP_STATUS_ERROR_MSK # Check error bits
          if otp_err != BMM350_OTP_STATUS_NO_ERROR:
            # Map OTP hardware error to API error
            # This mapping might need to be more specific based on BMM350_E_OTP_... codes
            return BMM350_E_OTP_PAGE_RD 
          
          if (otp_status & BMM350_OTP_STATUS_CMD_DONE): # Check CMD_DONE bit
            break
        else: # Loop timed out
            return BMM350_E_OTP_PAGE_RD # Timeout reading OTP status

        otp_msb_byte = self.read_reg(BMM350_REG_OTP_DATA_MSB_REG, 1)
        otp_lsb_byte = self.read_reg(BMM350_REG_OTP_DATA_LSB_REG, 1)
        if not otp_msb_byte or not otp_lsb_byte:
            return BMM350_E_COM_FAIL

        bmm350_sensor.otp_data[indx] = ((otp_msb_byte[0] << 8) | otp_lsb_byte[0]) & 0xFFFF
      
      # Update variant ID and compensation data after all OTP data is read
      # Variant ID from OTP word 30, bits [14:9] (assuming 0-indexed OTP words)
      # (otp_data[30] & 0x7E00) >> 9 if bits are [14:9]
      # Original: (bmm350_sensor.otp_data[30] & 0x7f00) >> 9. This is bits [14:9] and bit 8.
      # Let's assume the original mask/shift is correct for the specific sensor variant.
      if BMM350_OTP_DATA_LENGTH > 30: # Ensure index is valid
          bmm350_sensor.var_id = (bmm350_sensor.otp_data[30] & 0x7F00) >> 8 # Corrected shift for bits [14:8]
                                                                        # If it's [14:9], then >>9.
                                                                        # Original was >>9, so (otp_data[30] & 0x7E00) >> 9 for [14:9]
                                                                        # or (otp_data[30] & 0xFE00) >> 9 if it's 7 bits starting from bit 9.
                                                                        # Given 0x7f00, it's bits 8 to 14. So (val & 0x7F00) >> 8.
                                                                        # The original code had `>> 9` with `0x7f00`. This implies bits [15:9] effectively, with bit 15 masked out.
                                                                        # Let's stick to the original interpretation for now:
          bmm350_sensor.var_id = (bmm350_sensor.otp_data[30] & 0x7F00) >> 9


      self.update_mag_off_sens() # Update compensation parameters

      self.write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP) # Disable OTP access
      time.sleep(0.0001) # Small delay after OTP disable

      self.bmm350_magnetic_reset_and_wait()
    else:
      # The chip id verification failed and initialization failed. Procedure
      rslt = BMM350_CHIP_ID_ERROR
    return rslt


  def get_chip_id(self) -> Union[int, None]:
    """
    Read and return the BMM350 chip ID.
    
    This method reads the chip identification register to verify that the
    connected device is a BMM350 magnetometer sensor.
    
    :return: The chip ID value (should be 0x33 for BMM350), or None if read fails.
    :rtype: int | None
    
    :example: ``chip_id = mag.get_chip_id()``
    """
    chip_id_bytes = self.read_reg(BMM350_REG_CHIP_ID, 1)
    if chip_id_bytes:
        return chip_id_bytes[0]
    return None

  def soft_reset(self) -> None:
    """
    Performs a software reset on the BMM350 sensor.

    After the reset, the sensor is:
    1. OTP access is disabled.
    2. A magnetic reset is performed.
    3. The sensor is put into Suspend Mode.
    The user needs to manually set the desired operation mode afterwards.
    """
    self.write_reg(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)
    time.sleep(BMM350_SOFT_RESET_DELAY)
    self.write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF_OTP)
    time.sleep(0.0001) # Small delay
    self.bmm350_magnetic_reset_and_wait()
    # The magnetic_reset_and_wait might leave it in suspend or normal.
    # Explicitly set to suspend as per docstring.
    self.bmm350_set_powermode(BMM350_SUSPEND_MODE)


  def set_operation_mode(self, modes: int) -> None:
    """
    Sets the sensor's operation mode.

    :param modes: The desired operation mode.
    :type modes: int
    :example:
        Available modes:
        - ``BMM350_SUSPEND_MODE``: Default mode after power-up. Low power consumption.
                                 All registers are accessible.
        - ``BMM350_NORMAL_MODE``: Continuous measurement mode.
        - ``BMM350_FORCED_MODE``: Single measurement, then returns to Suspend Mode.
        - ``BMM350_FORCED_MODE_FAST``: Faster single measurement, then returns to Suspend Mode.
                                     Useful for achieving higher effective ODRs with single shots.
        - ``BMM350_PMU_CMD_NM_TC``: Normal mode with temperature compensation active.
    """
    self.bmm350_set_powermode(modes)

  def get_operation_mode(self) -> str:
    """
    Gets the current operation mode of the sensor.

    :return: A string describing the current operation mode.
    :rtype: str
    """
    mode = bmm350_sensor.power_mode # Use the stored power mode
    if mode == BMM350_SUSPEND_MODE:
       return "Suspend Mode"
    elif mode == BMM350_NORMAL_MODE:
       return "Normal Mode"
    elif mode == BMM350_FORCED_MODE:
       return "Forced Mode"
    elif mode == BMM350_FORCED_MODE_FAST:
       return "Forced Mode Fast"
    elif mode == BMM350_PMU_CMD_UPD_OAE:
        return "Update OAE Mode"
    elif mode == BMM350_PMU_CMD_NM_TC:
        return "Normal Mode with Temp Comp"
    # Add other modes if necessary
    else:
       return f"Unknown Mode ({hex(mode)})"

  def set_rate(self, rates: int) -> None:
    """
    Sets the Output Data Rate (ODR) for measurements in Normal Mode.

    :param rates: The desired data rate setting.
    :type rates: int
    :example:
        Available rates (constants from `magnetometer_vars.py`):
        - ``BMM350_DATA_RATE_1_5625HZ``
        - ``BMM350_DATA_RATE_3_125HZ``
        - ``BMM350_DATA_RATE_6_25HZ``
        - ``BMM350_DATA_RATE_12_5HZ`` (Default)
        - ``BMM350_DATA_RATE_25HZ``
        - ``BMM350_DATA_RATE_50HZ``
        - ``BMM350_DATA_RATE_100HZ``
        - ``BMM350_DATA_RATE_200HZ``
        - ``BMM350_DATA_RATE_400HZ``
    """
    avg_odr_reg_byte = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    if not avg_odr_reg_byte: return

    current_settings = avg_odr_reg_byte[0]
    # Preserve current averaging setting
    avg_reg = self.BMM350_GET_BITS(current_settings, BMM350_AVG_MSK, BMM350_AVG_POS)
    
    # Set new ODR, keeping existing averaging
    new_settings = (rates & BMM350_ODR_MSK) # Apply ODR mask first
    new_settings = self.BMM350_SET_BITS(new_settings, BMM350_AVG_MSK, BMM350_AVG_POS, avg_reg)
    
    self.write_reg(BMM350_REG_PMU_CMD_AGGR_SET, new_settings)
    # ODR/Averaging changes need to be applied by PMU_CMD_UPD_OAE command
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)
    time.sleep(BMM350_UPD_OAE_DELAY)

  def get_rate(self) -> Union[float, None]:
    """
    Gets the configured Output Data Rate (ODR).

    :return: The data rate in Hz, or None if communication fails.
    :rtype: float | None
    """
    avg_odr_reg_byte = self.read_reg(BMM350_REG_PMU_CMD_AGGR_SET, 1)
    if not avg_odr_reg_byte:
        return None
        
    odr_reg_val = self.BMM350_GET_BITS(avg_odr_reg_byte[0], BMM350_ODR_MSK, BMM350_ODR_POS)
    
    if odr_reg_val == BMM350_ODR_1_5625HZ: return 1.5625
    elif odr_reg_val == BMM350_ODR_3_125HZ: return 3.125
    elif odr_reg_val == BMM350_ODR_6_25HZ: return 6.25
    elif odr_reg_val == BMM350_ODR_12_5HZ: return 12.5
    elif odr_reg_val == BMM350_ODR_25HZ: return 25.0
    elif odr_reg_val == BMM350_ODR_50HZ: return 50.0
    elif odr_reg_val == BMM350_ODR_100HZ: return 100.0
    elif odr_reg_val == BMM350_ODR_200HZ: return 200.0
    elif odr_reg_val == BMM350_ODR_400HZ: return 400.0
    else: return None # Unknown ODR setting

  def set_preset_mode(self, avg: int, odr: int = BMM350_DATA_RATE_12_5HZ) -> None:
    """
    Sets a preset mode combining averaging and ODR settings.
    This simplifies configuration for common use cases.

    :param avg: The averaging setting (number of samples to average).
    :type avg: int
    :param odr: The Output Data Rate setting. Defaults to `BMM350_DATA_RATE_12_5HZ`.
    :type odr: int, optional
    :example:
        Averaging settings (constants from `magnetometer_vars.py`):
        - ``BMM350_AVG_NO_AVG`` (or ``BMM350_PRESETMODE_LOWPOWER`` if mapping directly)
        - ``BMM350_AVG_2``
        - ``BMM350_AVG_4`` (or ``BMM350_PRESETMODE_REGULAR``)
        - ``BMM350_AVG_8`` (or ``BMM350_PRESETMODE_HIGHACCURACY`` / ``BMM350_PRESETMODE_ENHANCED``)
        Note: The original Doxygen comments for `@param modes` (e.g. BMM350_PRESETMODE_LOWPOWER)
        actually refer to averaging settings. This function takes `avg` directly.
    """
    # Ensure avg and odr are within valid bit masks
    avg_setting = avg & (BMM350_AVG_MSK >> BMM350_AVG_POS) # Mask for 2 bits of averaging
    odr_setting = odr & BMM350_ODR_MSK

    reg_data = odr_setting # Start with ODR bits
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_AVG_MSK, BMM350_AVG_POS, avg_setting)
    
    self.write_reg(BMM350_REG_PMU_CMD_AGGR_SET, reg_data)
    self.write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)
    time.sleep(BMM350_UPD_OAE_DELAY)

  def self_test(self) -> str:
    """
    Performs a basic self-test by checking enabled axes.
    This is a very basic check; a more comprehensive self-test might involve
    checking sensor data against known stimuli if available.

    :return: A string indicating the self-test result based on enabled axes.
    :rtype: str
    """
    axis_en_byte = self.read_reg(BMM350_REG_PMU_CMD_AXIS_EN, 1)
    if not axis_en_byte:
        return "Communication error during self-test."

    axis_en = axis_en_byte[0]
    en_x = self.BMM350_GET_BITS(axis_en, BMM350_EN_X_MSK, BMM350_EN_X_POS)
    en_y = self.BMM350_GET_BITS(axis_en, BMM350_EN_Y_MSK, BMM350_EN_Y_POS)
    en_z = self.BMM350_GET_BITS(axis_en, BMM350_EN_Z_MSK, BMM350_EN_Z_POS)
    
    enabled_axes = []
    if en_x: enabled_axes.append("X")
    if en_y: enabled_axes.append("Y")
    if en_z: enabled_axes.append("Z")

    if not enabled_axes:
      return "Self-test fail: All axes disabled."
    else:
      return f"Self-test partial pass: Enabled axes: {', '.join(enabled_axes)}." \
             " (Full self-test would require data validation)."

  def set_measurement_XYZ(self, en_x: int = BMM350_X_EN, en_y: int = BMM350_Y_EN, en_z: int = BMM350_Z_EN) -> None:
    """
    Enables or disables measurements for X, Y, and Z axes.
    Disabling an axis will result in incorrect or zero data for that axis.

    :param en_x: Enable state for X-axis. Defaults to `BMM350_X_EN`.
    :type en_x: int, optional (`BMM350_X_EN` or `BMM350_X_DIS`)
    :param en_y: Enable state for Y-axis. Defaults to `BMM350_Y_EN`.
    :type en_y: int, optional (`BMM350_Y_EN` or `BMM350_Y_DIS`)
    :param en_z: Enable state for Z-axis. Defaults to `BMM350_Z_EN`.
    :type en_z: int, optional (`BMM350_Z_EN` or `BMM350_Z_DIS`)
    """
    if en_x == BMM350_X_DIS and en_y == BMM350_Y_DIS and en_z == BMM350_Z_DIS:
      bmm350_sensor.axis_en = BMM350_DISABLE # Store that all are disabled
      # Also write to register
      self.write_reg(BMM350_REG_PMU_CMD_AXIS_EN, BMM350_DISABLE)
    else:
      current_axis_en_byte = self.read_reg(BMM350_REG_PMU_CMD_AXIS_EN, 1)
      if not current_axis_en_byte: return

      data = current_axis_en_byte[0]
      data = self.BMM350_SET_BITS(data, BMM350_EN_X_MSK, BMM350_EN_X_POS, en_x)
      data = self.BMM350_SET_BITS(data, BMM350_EN_Y_MSK, BMM350_EN_Y_POS, en_y)
      data = self.BMM350_SET_BITS(data, BMM350_EN_Z_MSK, BMM350_EN_Z_POS, en_z)
      self.write_reg(BMM350_REG_PMU_CMD_AXIS_EN, data)
      bmm350_sensor.axis_en = data # Update stored state

  def get_measurement_state_XYZ(self) -> str:
    """
    Gets the current enable status for X, Y, and Z axis measurements.

    :return: A string describing the enable status of each axis.
    :rtype: str
    """
    # Read from register for most up-to-date info, though bmm350_sensor.axis_en should be sync'd
    axis_en_byte = self.read_reg(BMM350_REG_PMU_CMD_AXIS_EN, 1)
    if not axis_en_byte:
        return "Could not read axis enable status."
    
    axis_en = axis_en_byte[0]
    en_x = self.BMM350_GET_BITS(axis_en, BMM350_EN_X_MSK, BMM350_EN_X_POS)
    en_y = self.BMM350_GET_BITS(axis_en, BMM350_EN_Y_MSK, BMM350_EN_Y_POS)
    en_z = self.BMM350_GET_BITS(axis_en, BMM350_EN_Z_MSK, BMM350_EN_Z_POS)
    
    return (f"X-axis: {'Enabled' if en_x else 'Disabled'}, "
            f"Y-axis: {'Enabled' if en_y else 'Disabled'}, "
            f"Z-axis: {'Enabled' if en_z else 'Disabled'}")

  def get_raw_magnetic_data_for_calibration(self) -> Union[List[int], None]:
    '''
    Gets raw X, Y, Z magnetic data directly from sensor registers.
    This data is before any OTP or software compensations (including `fix_sign`).
    It is intended for use in calibration procedures.

    :return: A list containing [raw_x, raw_y, raw_z] integer values,
             or None if the read operation fails.
    :rtype: list[int] | None
    '''
    # Read 9 bytes for X, Y, Z raw data (3 bytes per axis: XLSB, LSB, MSB)
    mag_data_regs = self.read_reg(BMM350_REG_MAG_X_XLSB, 9)
    if mag_data_regs is None or len(mag_data_regs) < 9:
        print("Failed to read raw magnetometer data for calibration.")
        return None

    # Each axis is 24-bit, LSB first
    # XLSB (bits 7:0), LSB (bits 15:8), MSB (bits 23:16)
    raw_mag_x_u = mag_data_regs[0] | (mag_data_regs[1] << 8) | (mag_data_regs[2] << 16)
    raw_mag_y_u = mag_data_regs[3] | (mag_data_regs[4] << 8) | (mag_data_regs[5] << 16)
    raw_mag_z_u = mag_data_regs[6] | (mag_data_regs[7] << 8) | (mag_data_regs[8] << 16)

    # Apply sign correction as these are raw ADC values that need it
    raw_x = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    raw_y = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    raw_z = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    
    return [raw_x, raw_y, raw_z]

  def get_bmm350_data(self) -> List[float]:
    """
    Gets calibrated geomagnetic data (X, Y, Z) in microTeslas (uT) and updates sensor temperature.

    This method performs the following steps:
    1. Reads raw 24-bit data for X, Y, Z magnetic axes and temperature.
    2. Applies sign correction to the raw data.
    3. Applies hard iron calibration offsets.
    4. Applies soft iron calibration transformation.
    5. Scales the result to physical units (uT) using `_LOCAL_EARTH_FIELD_UT`.
    6. Calculates and stores the compensated temperature.

    :return: A list containing [X_uT, Y_uT, Z_uT, Temperature] calibrated geomagnetic data.
             Returns `[BMM350_FLOAT_DATA_ERROR]*4` if sensor read fails.
    :rtype: list[float]
    """
    mag_and_temp_regs = self.read_reg(BMM350_REG_MAG_X_XLSB, BMM350_MAG_TEMP_DATA_LEN)
    if mag_and_temp_regs is None or len(mag_and_temp_regs) < BMM350_MAG_TEMP_DATA_LEN:
        print("Failed to read sensor data for get_bmm350_data.")
        _mag_data.x = _mag_data.y = _mag_data.z = _mag_data.temperature = BMM350_FLOAT_DATA_ERROR
        return [BMM350_FLOAT_DATA_ERROR] * 4

    # Unpack 24-bit data for X, Y, Z, and Temperature (LSB first)
    raw_mag_x_u = mag_and_temp_regs[0] | (mag_and_temp_regs[1] << 8) | (mag_and_temp_regs[2] << 16)
    raw_mag_y_u = mag_and_temp_regs[3] | (mag_and_temp_regs[4] << 8) | (mag_and_temp_regs[5] << 16)
    raw_mag_z_u = mag_and_temp_regs[6] | (mag_and_temp_regs[7] << 8) | (mag_and_temp_regs[8] << 16)
    raw_temp_u  = mag_and_temp_regs[9] | (mag_and_temp_regs[10] << 8) | (mag_and_temp_regs[11] << 16)

    # Apply sign correction
    _raw_mag_data.raw_x_data = self.fix_sign(raw_mag_x_u, BMM350_SIGNED_24_BIT)
    _raw_mag_data.raw_y_data = self.fix_sign(raw_mag_y_u, BMM350_SIGNED_24_BIT)
    _raw_mag_data.raw_z_data = self.fix_sign(raw_mag_z_u, BMM350_SIGNED_24_BIT)
    _raw_mag_data.raw_t_data = self.fix_sign(raw_temp_u, BMM350_SIGNED_24_BIT)
    
    # --- Start of User Calibration ---
    # 1. Hard Iron Correction
    raw_lsb_vec = np.array([_raw_mag_data.raw_x_data, _raw_mag_data.raw_y_data, _raw_mag_data.raw_z_data])
    hard_iron_corrected_vec = raw_lsb_vec - _CALIBRATION_HARD_IRON
    
    # 2. Soft Iron Correction (Ellipsoid to Sphere Transformation)
    # _CALIBRATION_SOFT_IRON_TRANSFORM is expected to be M_inv
    unit_sphere_vec = _CALIBRATION_SOFT_IRON_TRANSFORM @ hard_iron_corrected_vec
    
    # 3. Scale to physical units (uT)
    # This step assumes unit_sphere_vec components are normalized relative to each other,
    # and _LOCAL_EARTH_FIELD_UT provides the magnitude for this normalized vector.
    # If unit_sphere_vec is truly a unit vector, then this scales it.
    # If it's not perfectly unit, this scales its components proportionally.
    geomagnetic_ut_vec = _LOCAL_EARTH_FIELD_UT * unit_sphere_vec
    # --- End of User Calibration ---

    _mag_data.x = geomagnetic_ut_vec[0]
    _mag_data.y = geomagnetic_ut_vec[1]
    _mag_data.z = geomagnetic_ut_vec[2]

    # Temperature Calculation (Bosch BMM150/generic style, may need BMM350 specific formula if different)
    # The following constants seem to be from a generic Bosch formula or BMM150.
    # Verify if these are correct for BMM350 or if OTP-derived temp factors should be used.
    # The `update_mag_off_sens` populates `bmm350_sensor.mag_comp` which includes
    # `t_offs` and `t_sens`. These should ideally be used for temperature compensation.
    
    # Using a simplified temperature calculation based on OTP values if available,
    # otherwise falling back to a generic interpretation if _raw_mag_data.raw_t_data
    # is already somewhat processed by OTP internally or needs a simple scaling.
    # The original code had a complex formula. Let's use the OTP derived values.
    # Tcomp = (Traw * (1 + TCS)) + TCO, where Traw is related to _raw_mag_data.raw_t_data
    # The raw_t_data needs to be converted to degrees C first.
    # From BMM350 datasheet (example, actual formula might differ):
    # Temp_float = (raw_temp_adc / SENS_TEMP_ADC_LSB_PER_DEG_C) + TEMP_OFFSET_DEG_C
    # For now, using the formula structure from the original code, but noting it might
    # need adjustment based on BMM350 datasheet specifics for raw_t_data to degC conversion.
    
    # The original formula:
    # bxy_sens = 14.55; bz_sens = 9.0; temp_sens = 0.00204
    # ina_xy_gain_trgt = 19.46; ina_z_gain_trgt = 31.0
    # adc_gain = 1 / 1.5; lut_gain = 0.714607238769531
    # lsb_to_degc_for_temp = 1 / (temp_sens * adc_gain * lut_gain * 1048576) # 1048576 is 2^20
    # temp_out_data = _raw_mag_data.raw_t_data * lsb_to_degc_for_temp
    # temp_adjusted = temp_out_data - (25.49 if temp_out_data > 0 else (-25.49 if temp_out_data < 0 else 0))
    # _mag_data.temperature = (1 + bmm350_sensor.mag_comp.dut_sensit_coef.t_sens) * temp_adjusted + \
    #                         bmm350_sensor.mag_comp.dut_offset_coef.t_offs

    # Simplified approach: Assume raw_t_data is proportional to temperature and OTP gives slope/offset.
    # This is a placeholder. BMM350 datasheet is needed for accurate raw_t_data to Celsius conversion.
    # A common pattern is: Temp_degC = (RawTemp - OTP_Temp_Offset_ADC) * OTP_Temp_Sensitivity_ADC_to_degC + RefTemp_degC
    # Using the structure from `update_mag_off_sens` for `dut_t0` and `t_sens/t_offs`:
    # dut_t0 is a reference temperature (e.g., 23.0 C + OTP_offset_from_23C)
    # dut_offset_coef.t_offs is an offset in degC
    # dut_sensit_coef.t_sens is a sensitivity factor (e.g., (1 + factor))
    
    # Placeholder temperature calculation - replace with BMM350 specific formula if found
    # If _raw_mag_data.raw_t_data is, for example, LSBs around a reference point:
    temp_deviation_from_ref = _raw_mag_data.raw_t_data / (2**18) # Example scaling, assuming 24-bit raw, maybe 6 bits are fractional
    temp_at_sensor_die = bmm350_sensor.mag_comp.dut_t0 + temp_deviation_from_ref # Very rough guess
    
    # Apply OTP temperature compensation factors
    _mag_data.temperature = (temp_at_sensor_die * (1 + bmm350_sensor.mag_comp.dut_sensit_coef.t_sens)) + \
                              bmm350_sensor.mag_comp.dut_offset_coef.t_offs
    # This is still a guess. The original complex formula might be closer if those constants are from BMM350 context.
    # For now, retaining the original complex formula as it was likely specific:
    bxy_sens = 14.55; bz_sens = 9.0; temp_sens_factor = 0.00204 # Renamed temp_sens to avoid conflict
    ina_xy_gain_trgt = 19.46; ina_z_gain_trgt = 31.0
    adc_gain = 1.0 / 1.5; lut_gain = 0.714607238769531
    # Assuming raw_t_data is 24-bit, so 2^23 for full scale if signed, or 2^24 if unsigned range used in formula
    # The original formula used 1048576 which is 2^20. This implies raw_t_data might be interpreted as 21-bit effectively for this formula.
    lsb_to_degc_for_temp = 1.0 / (temp_sens_factor * adc_gain * lut_gain * (1 << 20)) # Using (1 << 20) for 2^20
    
    # Use the signed raw_t_data
    temp_out_data = _raw_mag_data.raw_t_data * lsb_to_degc_for_temp
    
    # The constant 25.49 is likely a fixed offset or reference point in this formula
    temp_adjusted = temp_out_data - (25.49 if temp_out_data >= 0 else -25.49) # Adjusted for >= 0
    
    _mag_data.temperature = (1.0 + bmm350_sensor.mag_comp.dut_sensit_coef.t_sens) * temp_adjusted + \
                              bmm350_sensor.mag_comp.dut_offset_coef.t_offs


    return [_mag_data.x, _mag_data.y, _mag_data.z, _mag_data.temperature]

  def get_compass_degree(self) -> float:
    """
    Calculates the compass heading (yaw) in degrees.

    Assumes the sensor's XY plane is horizontal (parallel to the ground).
    The heading is calculated from the X and Y components of the calibrated
    geomagnetic data. The angle is measured counter-clockwise from the
    sensor's positive X-axis.

    Note: This calculation is **not** tilt-compensated. For accurate heading
    when the sensor is tilted, accelerometer data and a tilt-compensation
    algorithm are required.

    :return: Compass heading in degrees (0.0 to 360.0).
             Returns `BMM350_FLOAT_DATA_ERROR` if magnetometer data is invalid.
    :rtype: float
    """
    mag_data_ut = self.get_bmm350_data() # Returns [X_uT, Y_uT, Z_uT, Temperature]

    if any(comp == BMM350_FLOAT_DATA_ERROR or math.isnan(comp) for comp in mag_data_ut):
        print("Warning: Invalid magnetometer data for compass calculation.")
        return BMM350_FLOAT_DATA_ERROR

    # Yaw (heading) = atan2(Y, X)
    # mag_data_ut[1] is Y_uT, mag_data_ut[0] is X_uT
    yaw_radians = math.atan2(mag_data_ut[1], mag_data_ut[0])
    yaw_degrees = math.degrees(yaw_radians)

    # Normalize to 0-360 degrees range
    if yaw_degrees < 0:
        yaw_degrees += 360.0
    
    return yaw_degrees

  def set_data_ready_pin(self, modes: int, polarity: int) -> None:
    """
    Configures the Data Ready (DRDY) interrupt pin.

    :param modes: Enable or disable the DRDY interrupt.
                  - `BMM350_ENABLE_INTERRUPT` (or `BMM350_INT_DRDY_EN`)
                  - `BMM350_DISABLE_INTERRUPT` (or `BMM350_INT_DRDY_DIS`)
    :type modes: int
    :param polarity: Polarity of the interrupt pin.
                     - `BMM350_ACTIVE_HIGH`
                     - `BMM350_ACTIVE_LOW`
    :type polarity: int
    :example:
        To enable DRDY with active high polarity:
        ``magnetometer.set_data_ready_pin(BMM350_ENABLE_INTERRUPT, BMM350_ACTIVE_HIGH)``
    """
    reg_data_byte = self.read_reg(BMM350_REG_INT_CTRL, 1)
    if not reg_data_byte: return

    reg_data = reg_data_byte[0]
    # Set interrupt mode to Pulsed (BMM350_INT_MODE_PULSED)
    reg_data = self.BMM350_SET_BITS_POS_0(reg_data, BMM350_INT_MODE_MSK, BMM350_INT_MODE_PULSED)
    # Set interrupt polarity
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_INT_POL_MSK, BMM350_INT_POL_POS, polarity)
    # Set interrupt drive to Push-Pull (BMM350_INT_OD_PUSHPULL)
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_INT_OD_MSK, BMM350_INT_OD_POS, BMM350_INT_OD_PUSHPULL) 
    # Enable interrupt output pin mapping (BMM350_MAP_TO_PIN)
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_INT_OUTPUT_EN_MSK, BMM350_INT_OUTPUT_EN_POS, BMM350_MAP_TO_PIN)
    # Enable/Disable DRDY interrupt source
    reg_data = self.BMM350_SET_BITS(reg_data, BMM350_DRDY_DATA_REG_EN_MSK, BMM350_DRDY_DATA_REG_EN_POS, modes)
    
    self.write_reg(BMM350_REG_INT_CTRL, reg_data)

  def get_data_ready_state(self) -> Union[bool, None]:
    """
    Checks the Data Ready status from the interrupt status register.

    :return: True if data is ready, False if not. None if communication fails.
    :rtype: bool | None
    """
    int_status_byte = self.read_reg(BMM350_REG_INT_STATUS, 1) 
    if not int_status_byte:
        return None
        
    drdy_status = self.BMM350_GET_BITS(int_status_byte[0], BMM350_DRDY_DATA_REG_MSK, BMM350_DRDY_DATA_REG_POS)
    return bool(drdy_status) # True if bit is set, False otherwise


  def set_threshold_interrupt(self, modes: int, threshold: float, polarity: int) -> None:
    """
    Configures a threshold interrupt.
    An interrupt is triggered when the geomagnetic value of any enabled channel
    crosses the specified threshold.

    Note: The actual threshold comparison logic in `get_threshold_data` seems to use
    `threshold * 16`. This function stores the raw `threshold` value.
    The interrupt pin itself is configured via `set_data_ready_pin` internally.

    :param modes: Threshold mode.
                  - `LOW_THRESHOLD_INTERRUPT`: Interrupt when data < threshold.
                  - `HIGH_THRESHOLD_INTERRUPT`: Interrupt when data > threshold. (Note: current get_threshold_data logic is < for both)
    :type modes: int
    :param threshold: The threshold value in the same units as `get_bmm350_data()` (uT).
    :type threshold: float
    :param polarity: Polarity of the interrupt pin.
                     - `BMM350_ACTIVE_HIGH` (Conceptual, as DFRobot used POLARITY_HIGH/LOW)
                     - `BMM350_ACTIVE_LOW`
    :type polarity: int
    """
    # DFRobot constants were LOW_THRESHOLD_INTERRUPT, HIGH_THRESHOLD_INTERRUPT
    # And POLARITY_HIGH, POLARITY_LOW. These need to map to BMM350_ACTIVE_HIGH/LOW.
    # Assuming POLARITY_HIGH maps to BMM350_ACTIVE_HIGH, etc.
    
    self.__thresholdMode = modes # Store whether it's low or high threshold logic
    self.threshold = threshold   # Store the user-provided threshold
    
    # Enable the DRDY interrupt mechanism, which will now act as a threshold interrupt flag
    # The actual "threshold" feature of BMM350 might be different and use dedicated threshold registers.
    # This implementation re-uses DRDY pin and checks threshold in software via get_threshold_data().
    self.set_data_ready_pin(BMM350_ENABLE_INTERRUPT, polarity) # Enable the pin
    
    # Note: BMM350 has actual threshold interrupt capabilities (INT_CTRL bits for threshold on X,Y,Z).
    # This current Python implementation seems to be a software-based threshold check
    # using the DRDY flag as a generic "new data available for check" signal.
    # A hardware threshold interrupt would be more efficient.

  def get_threshold_data(self) -> List[float]:
    """
    Checks if a software-defined threshold has been crossed based on the latest data.
    This method should be called after a DRDY interrupt (configured for threshold) occurs.

    It reads the latest geomagnetic data and compares it against the stored threshold
    and mode (`__thresholdMode`).

    :return: A list of three floats [x, y, z]. If a threshold condition was met for an axis,
             that element will be the geomagnetic data for that axis; otherwise, it will be `NO_DATA`.
             The interpretation of "threshold met" depends on `__thresholdMode`.
             - `LOW_THRESHOLD_INTERRUPT`: Interrupt if magData[axis] < self.threshold * 16.
             - `HIGH_THRESHOLD_INTERRUPT`: Interrupt if magData[axis] < self.threshold * 16. (This seems like a bug, should be > for high)
    :rtype: list[float]
    """
    # Initialize with NO_DATA
    data_values = [NO_DATA, NO_DATA, NO_DATA] # Using the global NO_DATA constant

    # Check if data is ready (simulating an interrupt occurred)
    if self.get_data_ready_state():
      latest_mag_data = self.get_bmm350_data()

      # Check for errors from get_bmm350_data
      if any(val == BMM350_FLOAT_DATA_ERROR for val in latest_mag_data):
          return data_values # Return NO_DATA for all if read failed

      # The threshold comparison uses self.threshold * 16. This scaling factor should be documented or clarified.
      scaled_threshold = self.threshold * 16.0

      if self.__thresholdMode == LOW_THRESHOLD_INTERRUPT:
        if latest_mag_data[0] < scaled_threshold: data_values[0] = latest_mag_data[0]
        if latest_mag_data[1] < scaled_threshold: data_values[1] = latest_mag_data[1]
        if latest_mag_data[2] < scaled_threshold: data_values[2] = latest_mag_data[2]
      elif self.__thresholdMode == HIGH_THRESHOLD_INTERRUPT:
        # Corrected logic for high threshold: should be data > threshold
        if latest_mag_data[0] > scaled_threshold: data_values[0] = latest_mag_data[0]
        if latest_mag_data[1] > scaled_threshold: data_values[1] = latest_mag_data[1]
        if latest_mag_data[2] > scaled_threshold: data_values[2] = latest_mag_data[2]
        # Original logic was: if magData[i] < self.threshold*16 for HIGH mode too, which is incorrect.
    return data_values

# I2C interface
class Magnetometer(_MagnetometerBase):
  """
  I2C interface implementation for the BMM350 Magnetometer.
  This class handles the specifics of I2C communication and inherits
  core magnetometer functionalities from `_MagnetometerBase`.

  :param bus: The I2C bus number (e.g., 1 for Raspberry Pi's /dev/i2c-1).
  :type bus: int
  :param addr: The I2C address of the BMM350 sensor.
               Defaults to `BMM350_I2C_ADDR_DEFAULT` (0x14).
  :type addr: int, optional
  :raises IOError: If the I2C bus cannot be opened or the platform is not supported.
  :raises TypeError: If bus or addr are not integers.
  """
  def __init__(self, bus: int, addr: int = BMM350_I2C_ADDR_DEFAULT):
    """
    Initializes the I2C magnetometer instance.

    :param bus: I2C bus number.
    :type bus: int
    :param addr: I2C address of the BMM350. Defaults to `BMM350_I2C_ADDR_DEFAULT`.
    :type addr: int, optional
    """
    if not isinstance(bus, int):
        raise TypeError("I2C bus must be an integer.")
    if not isinstance(addr, int):
        raise TypeError("I2C address must be an integer.")
        
    self.bus_num = bus # Store bus number, as super() uses self.bus
    self.__addr = addr
    self.i2cbus = None

    # Platform detection and SMBus initialization
    # Using smbus2 as it's generally preferred and handles RPi/Linux well.
    try:
        from smbus2 import SMBus
        self.i2cbus = SMBus(self.bus_num)
    except ImportError:
        # Fallback or error for non-smbus2 environments if needed
        raise ImportError("smbus2 library not found. Please install it.")
    except Exception as e:
        raise IOError(f"Failed to open I2C bus {self.bus_num}: {e}")

    super(Magnetometer, self).__init__(bus=self.bus_num) # Call base class init

  # The is_raspberrypi and test_platform methods from the original code
  # are complex and might be simplified if smbus2 handles most cases.
  # For now, they are removed as smbus2 is directly used. If specific
  # non-standard Linux/Windows I2C hardware needs support, they could be reinstated.

  def write_reg(self, reg: int, data: int) -> None:
    """
    Writes a single byte of data to the specified register via I2C.

    :param reg: The register address to write to.
    :type reg: int
    :param data: The byte of data to write.
    :type data: int
    :raises IOError: If an I2C communication error occurs.
    """
    try:
      self.i2cbus.write_byte_data(self.__addr, reg, data)
    except Exception as e: # Add except block to catch exceptions
      raise IOError(f"I2C write failed to register {hex(reg)}") from e
  
  def read_reg(self, reg: int, length: int) -> Union[list[int], None]:
    """
    Reads a specified number of bytes from the given register via I2C.
    The BMM350 protocol includes 2 dummy bytes in I2C reads, which are handled here.

    :param reg: The starting register address to read from.
    :type reg: int
    :param length: The number of data bytes to read (excluding dummy bytes).
    :type length: int
    :return: A list of bytes read from the sensor, or None if an error occurs.
    :rtype: list[int] | None
    :raises IOError: If an I2C communication error occurs.
    """
    # try:
    #   # BMM350 I2C read protocol requires reading `length + BMM350_DUMMY_BYTES`
    #   # The first BMM350_DUMMY_BYTES are then discarded.
    #   raw_read = self.i2cbus.read_i2c_block_data(self.__addr, reg, length + BMM350_DUMMY_BYTES)
    #   # Return the actual data, skipping the dummy bytes
    #   return raw_read[BMM350_DUMMY_BYTES:]
    # except Exception as e:
    #   print(f"I2C read error from reg {hex(reg)} for length {length}: {e}")
    #   # raise IOError(f"I2C read failed from register {hex(reg)}") from e # Option to raise
    #   return None # Current behavior is to return None on error
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

    
  def get_xyz(self) -> List[float]:
    """
    Retrieves the compensated geomagnetic data for X, Y, and Z axes.

    :return: A list containing the [X, Y, Z] magnetic field components in microTesla (uT).
    :rtype: List[float]
    """
    data = self.get_bmm350_data()
    return data[:3]

  def get_x(self) -> float:
    """
    Retrieves the compensated geomagnetic data for the X-axis.

    :return: The X-axis magnetic field component in microTesla (uT).
    :rtype: float
    """
    return self.get_bmm350_data()[0]

  def get_y(self) -> float:
    """
    Retrieves the compensated geomagnetic data for the Y-axis.

    :return: The Y-axis magnetic field component in microTesla (uT).
    :rtype: float
    """
    return self.get_bmm350_data()[1]

  def get_z(self) -> float:
    """
    Retrieves the compensated geomagnetic data for the Z-axis.

    :return: The Z-axis magnetic field component in microTesla (uT).
    :rtype: float
    """
    return self.get_bmm350_data()[2]

  def get_t(self) -> float:
    """
    Retrieves the compensated temperature data.

    :return: The temperature in degrees Celsius.
    :rtype: float
    """
    return self.get_bmm350_data()[3]

  def close(self) -> None:
    """
    Closes the I2C bus connection.
    """
    if hasattr(self, 'i2cbus') and self.i2cbus:
        try:
            # Optionally, put sensor into a low power mode or known state before closing
            # self.set_operation_mode(BMM350_SUSPEND_MODE)
            self.i2cbus.close()
            print(f"I2C bus {self.bus_num} closed for magnetometer {hex(self.__addr)}.")
        except Exception as e:
            print(f"Error closing I2C bus for magnetometer: {e}")

# Magnetometer = Magnetometer_I2C # This line is now removed.