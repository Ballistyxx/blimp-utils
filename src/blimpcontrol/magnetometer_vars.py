# Chip id of BMM350
BMM350_CHIP_ID                              = 0x33
BMM350_I2C_ADDR_DEFAULT                     = 0x14
# Variant ID of BMM350
BMM350_MIN_VAR                              = 0x10

# Sensor interface success code
BMM350_INTF_RET_SUCCESS                     = 0

# API success code
BMM350_OK                                   = 0

# API error codes
BMM350_E_NULL_PTR                           = -1
BMM350_E_COM_FAIL                           = -2
BMM350_E_DEV_NOT_FOUND                      = -3
BMM350_E_INVALID_CONFIG                     = -4
BMM350_E_BAD_PAD_DRIVE                      = -5
BMM350_E_RESET_UNFINISHED                   = -6
BMM350_E_INVALID_INPUT                      = -7
BMM350_E_SELF_TEST_INVALID_AXIS             = -8
BMM350_E_OTP_BOOT                           = -9
BMM350_E_OTP_PAGE_RD                        = -10
BMM350_E_OTP_PAGE_PRG                       = -11
BMM350_E_OTP_SIGN                           = -12
BMM350_E_OTP_INV_CMD                        = -13
BMM350_E_OTP_UNDEFINED                      = -14
BMM350_E_ALL_AXIS_DISABLED                  = -15
BMM350_E_PMU_CMD_VALUE                      = -16

BMM350_NO_ERROR                             = 0

# Sensor delay time settings in microseconds
BMM350_SOFT_RESET_DELAY                     = 24000/1000000
BMM350_MAGNETIC_RESET_DELAY                 = 40000/1000000
BMM350_START_UP_TIME_FROM_POR               = 3000/1000000
BMM350_GOTO_SUSPEND_DELAY                   = 6000/1000000
BMM350_SUSPEND_TO_NORMAL_DELAY              = 38000/1000000
BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY       = 15000/1000000
BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY        = 17000/1000000
BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY        = 20000/1000000
BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY        = 28000/1000000
BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY  = 4000/1000000
BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY   = 5000/1000000
BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY   = 9000/1000000
BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY   = 16000/1000000
BMM350_UPD_OAE_DELAY                        = 1000/1000000
BMM350_BR_DELAY                             = 14000/1000000
BMM350_FGR_DELAY                            = 18000/1000000

# Length macros
BMM350_OTP_DATA_LENGTH                      = 32
BMM350_READ_BUFFER_LENGTH                   = 127
BMM350_MAG_TEMP_DATA_LEN                    = 12

# Averaging macros
BMM350_AVG_NO_AVG                           = 0x0
BMM350_AVG_2                                = 0x1
BMM350_AVG_4                                = 0x2
BMM350_AVG_8                                = 0x3

# ODR
BMM350_ODR_400HZ                            = 0x2
BMM350_ODR_200HZ                            = 0x3
BMM350_ODR_100HZ                            = 0x4
BMM350_ODR_50HZ                             = 0x5
BMM350_ODR_25HZ                             = 0x6
BMM350_ODR_12_5HZ                           = 0x7   # default rate
BMM350_ODR_6_25HZ                           = 0x8
BMM350_ODR_3_125HZ                          = 0x9
BMM350_ODR_1_5625HZ                         = 0xA

# Power modes
BMM350_PMU_CMD_SUS                          = 0x00
BMM350_PMU_CMD_NM                           = 0x01
BMM350_PMU_CMD_UPD_OAE                      = 0x02
BMM350_PMU_CMD_FM                           = 0x03
BMM350_PMU_CMD_FM_FAST                      = 0x04
BMM350_PMU_CMD_FGR                          = 0x05
BMM350_PMU_CMD_FGR_FAST                     = 0x06
BMM350_PMU_CMD_BR                           = 0x07
BMM350_PMU_CMD_BR_FAST                      = 0x08
BMM350_PMU_CMD_NM_TC                        = 0x09

BMM350_PMU_STATUS_0                         = 0x0

BMM350_DISABLE                              = 0x0
BMM350_ENABLE                               = 0x1
BMM350_MAP_TO_PIN                           = BMM350_ENABLE

BMM350_CMD_NOP                              = 0x0
BMM350_CMD_SOFTRESET                        = 0xB6

BMM350_TARGET_PAGE_PAGE0                    = 0x0
BMM350_TARGET_PAGE_PAGE1                    = 0x1

BMM350_INT_MODE_LATCHED                     = 0x1
BMM350_INT_MODE_PULSED                      = 0x0

BMM350_INT_POL_ACTIVE_HIGH                  = 0x1
BMM350_INT_POL_ACTIVE_LOW                   = 0x0

BMM350_INT_OD_PUSHPULL                      = 0x1
BMM350_INT_OD_OPENDRAIN                     = 0x0

BMM350_INT_OUTPUT_EN_OFF                    = 0x0
BMM350_INT_OUTPUT_EN_ON                     = 0x1

BMM350_INT_DRDY_EN                          = 0x1
BMM350_INT_DRDY_DIS                         = 0x0

BMM350_MR_MR1K8                             = 0x0
BMM350_MR_MR2K1                             = 0x1
BMM350_MR_MR1K5                             = 0x2
BMM350_MR_MR0K6                             = 0x3

BMM350_SEL_DTB1X_PAD_PAD_INT                = 0x0
BMM350_SEL_DTB1X_PAD_PAD_BYP                = 0x1

BMM350_TMR_TST_HIZ_VTMR_VTMR_ON             = 0x0
BMM350_TMR_TST_HIZ_VTMR_VTMR_HIZ            = 0x1

BMM350_LSB_MASK                             = 0x00FF
BMM350_MSB_MASK                             = 0xFF00

# Pad drive strength
BMM350_PAD_DRIVE_WEAKEST                    = 0
BMM350_PAD_DRIVE_STRONGEST                  = 7

# I2C Register Addresses

# Register to set I2C address to LOW
BMM350_I2C_ADSEL_SET_LOW                    = 0x14

# Register to set I2C address to HIGH
BMM350_I2C_ADSEL_SET_HIGH                   = 0x15

BMM350_DUMMY_BYTES                          = 2

# Register Addresses

BMM350_REG_CHIP_ID                          = 0x00
BMM350_REG_REV_ID                           = 0x01
BMM350_REG_ERR_REG                          = 0x02
BMM350_REG_PAD_CTRL                         = 0x03
BMM350_REG_PMU_CMD_AGGR_SET                 = 0x04
BMM350_REG_PMU_CMD_AXIS_EN                  = 0x05
BMM350_REG_PMU_CMD                          = 0x06
BMM350_REG_PMU_CMD_STATUS_0                 = 0x07
BMM350_REG_PMU_CMD_STATUS_1                 = 0x08
BMM350_REG_I3C_ERR                          = 0x09
BMM350_REG_I2C_WDT_SET                      = 0x0A
BMM350_REG_TRSDCR_REV_ID                    = 0x0D
BMM350_REG_TC_SYNC_TU                       = 0x21
BMM350_REG_TC_SYNC_ODR                      = 0x22
BMM350_REG_TC_SYNC_TPH_1                    = 0x23
BMM350_REG_TC_SYNC_TPH_2                    = 0x24
BMM350_REG_TC_SYNC_DT                       = 0x25
BMM350_REG_TC_SYNC_ST_0                     = 0x26
BMM350_REG_TC_SYNC_ST_1                     = 0x27
BMM350_REG_TC_SYNC_ST_2                     = 0x28
BMM350_REG_TC_SYNC_STATUS                   = 0x29
BMM350_REG_INT_CTRL                         = 0x2E
BMM350_REG_INT_CTRL_IBI                     = 0x2F
BMM350_REG_INT_STATUS                       = 0x30
BMM350_REG_MAG_X_XLSB                       = 0x31
BMM350_REG_MAG_X_LSB                        = 0x32
BMM350_REG_MAG_X_MSB                        = 0x33
BMM350_REG_MAG_Y_XLSB                       = 0x34
BMM350_REG_MAG_Y_LSB                        = 0x35
BMM350_REG_MAG_Y_MSB                        = 0x36
BMM350_REG_MAG_Z_XLSB                       = 0x37
BMM350_REG_MAG_Z_LSB                        = 0x38
BMM350_REG_MAG_Z_MSB                        = 0x39
BMM350_REG_TEMP_XLSB                        = 0x3A
BMM350_REG_TEMP_LSB                         = 0x3B
BMM350_REG_TEMP_MSB                         = 0x3C
BMM350_REG_SENSORTIME_XLSB                  = 0x3D
BMM350_REG_SENSORTIME_LSB                   = 0x3E
BMM350_REG_SENSORTIME_MSB                   = 0x3F
BMM350_REG_OTP_CMD_REG                      = 0x50
BMM350_REG_OTP_DATA_MSB_REG                 = 0x52
BMM350_REG_OTP_DATA_LSB_REG                 = 0x53
BMM350_REG_OTP_STATUS_REG                   = 0x55
BMM350_REG_TMR_SELFTEST_USER                = 0x60
BMM350_REG_CTRL_USER                        = 0x61
BMM350_REG_CMD                              = 0x7E

# Macros for OVWR
BMM350_REG_OVWR_VALUE_ANA_0                 = 0x3A
BMM350_REG_OVWR_EN_ANA_0                    = 0x3B

# Chip ID Value (used for verification)
BMM350_CHIP_ID_VAL                          = BMM350_CHIP_ID # Alias for BMM350_CHIP_ID

# OTP Commands
BMM350_OTP_CMD_READ_PAGE0                   = 0x0A
BMM350_OTP_CMD_READ_PAGE1                   = 0x0B
BMM350_OTP_CMD_PWR_OFF_OTP                  = 0x00 # Command to disable OTP access

# OTP Status Register bits
BMM350_OTP_STATUS_OTP_OK_POS                = 0 # Bit position for OTP_OK
BMM350_OTP_STATUS_OTP_BUSY_POS              = 1 # Bit position for OTP_BUSY
BMM350_OTP_STATUS_OTP_ERR_POS               = 2 # Bit position for OTP_ERR

# PMU_CMD_STATUS_0 bits
BMM350_POR_DETECTED_POS                     = 4 # Bit position for POR_DETECTED in PMU_CMD_STATUS_0

# Macros for bit masking

BMM350_CHIP_ID_OTP_MSK                      = 0xf
BMM350_CHIP_ID_OTP_POS                      = 0x0
BMM350_CHIP_ID_FIXED_MSK                    = 0xf0
BMM350_CHIP_ID_FIXED_POS                    = 0x4
BMM350_REV_ID_MAJOR_MSK                     = 0xf0
BMM350_REV_ID_MAJOR_POS                     = 0x4
BMM350_REV_ID_MINOR_MSK                     = 0xf
BMM350_REV_ID_MINOR_POS                     = 0x0
BMM350_PMU_CMD_ERROR_MSK                    = 0x1
BMM350_PMU_CMD_ERROR_POS                    = 0x0
BMM350_BOOT_UP_ERROR_MSK                    = 0x2
BMM350_BOOT_UP_ERROR_POS                    = 0x1
BMM350_DRV_MSK                              = 0x7
BMM350_DRV_POS                              = 0x0
BMM350_AVG_MSK                              = 0x30
BMM350_AVG_POS                              = 0x4
BMM350_ODR_MSK                              = 0xf
BMM350_ODR_POS                              = 0x0
BMM350_PMU_CMD_MSK                          = 0xf
BMM350_PMU_CMD_POS                          = 0x0
BMM350_EN_X_MSK                             = 0x01
BMM350_EN_X_POS                             = 0x0
BMM350_EN_Y_MSK                             = 0x02
BMM350_EN_Y_POS                             = 0x1
BMM350_EN_Z_MSK                             = 0x04
BMM350_EN_Z_POS                             = 0x2
BMM350_EN_XYZ_MSK                           = 0x7
BMM350_EN_XYZ_POS                           = 0x0
BMM350_PMU_CMD_BUSY_MSK                     = 0x1
BMM350_PMU_CMD_BUSY_POS                     = 0x0
BMM350_ODR_OVWR_MSK                         = 0x2
BMM350_ODR_OVWR_POS                         = 0x1
BMM350_AVG_OVWR_MSK                         = 0x4
BMM350_AVG_OVWR_POS                         = 0x2
BMM350_PWR_MODE_IS_NORMAL_MSK               = 0x8
BMM350_PWR_MODE_IS_NORMAL_POS               = 0x3
BMM350_CMD_IS_ILLEGAL_MSK                   = 0x10
BMM350_CMD_IS_ILLEGAL_POS                   = 0x4
BMM350_PMU_CMD_VALUE_MSK                    = 0xE0
BMM350_PMU_CMD_VALUE_POS                    = 0x5
BMM350_PMU_ODR_S_MSK                        = 0xf
BMM350_PMU_ODR_S_POS                        = 0x0
BMM350_PMU_AVG_S_MSK                        = 0x30
BMM350_PMU_AVG_S_POS                        = 0x4
BMM350_I3C_ERROR_0_MSK                      = 0x1
BMM350_I3C_ERROR_0_POS                      = 0x0
BMM350_I3C_ERROR_3_MSK                      = 0x8
BMM350_I3C_ERROR_3_POS                      = 0x3
BMM350_I2C_WDT_EN_MSK                       = 0x1
BMM350_I2C_WDT_EN_POS                       = 0x0
BMM350_I2C_WDT_SEL_MSK                      = 0x2

BMM350_I2C_WDT_SEL_POS                      = 0x1
BMM350_TRSDCR_REV_ID_OTP_MSK                = 0x3
BMM350_TRSDCR_REV_ID_OTP_POS                = 0x0
BMM350_TRSDCR_REV_ID_FIXED_MSK              = 0xfc
BMM350_TRSDCR_REV_ID_FIXED_POS              = 0x2
BMM350_PAGING_EN_MSK                        = 0x80
BMM350_PAGING_EN_POS                        = 0x7
BMM350_DRDY_DATA_REG_MSK                    = 0x4
BMM350_DRDY_DATA_REG_POS                    = 0x2
BMM350_INT_MODE_MSK                         = 0x1
BMM350_INT_MODE_POS                         = 0x0
BMM350_INT_POL_MSK                          = 0x2
BMM350_INT_POL_POS                          = 0x1
BMM350_INT_OD_MSK                           = 0x4
BMM350_INT_OD_POS                           = 0x2
BMM350_INT_OUTPUT_EN_MSK                    = 0x8
BMM350_INT_OUTPUT_EN_POS                    = 0x3
BMM350_DRDY_DATA_REG_EN_MSK                 = 0x80
BMM350_DRDY_DATA_REG_EN_POS                 = 0x7
BMM350_DRDY_INT_MAP_TO_IBI_MSK              = 0x1
BMM350_DRDY_INT_MAP_TO_IBI_POS              = 0x0
BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_MSK   = 0x10
BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_POS   = 0x4
BMM350_TC_SYNC_TU_MSK                       = 0xff
BMM350_TC_SYNC_ODR_MSK                      = 0xff
BMM350_TC_SYNC_TPH_1_MSK                    = 0xff
BMM350_TC_SYNC_TPH_2_MSK                    = 0xff
BMM350_TC_SYNC_DT_MSK                       = 0xff
BMM350_TC_SYNC_ST_0_MSK                     = 0xff
BMM350_TC_SYNC_ST_1_MSK                     = 0xff
BMM350_TC_SYNC_ST_2_MSK                     = 0xff
BMM350_CFG_FORCE_SOSC_EN_MSK                = 0x4
BMM350_CFG_FORCE_SOSC_EN_POS                = 0x2
BMM350_ST_IGEN_EN_MSK                       = 0x1
BMM350_ST_IGEN_EN_POS                       = 0x0
BMM350_ST_N_MSK                             = 0x2
BMM350_ST_N_POS                             = 0x1
BMM350_ST_P_MSK                             = 0x4
BMM350_ST_P_POS                             = 0x2
BMM350_IST_EN_X_MSK                         = 0x8
BMM350_IST_EN_X_POS                         = 0x3
BMM350_IST_EN_Y_MSK                         = 0x10
BMM350_IST_EN_Y_POS                         = 0x4
BMM350_CFG_SENS_TIM_AON_MSK                 = 0x1
BMM350_CFG_SENS_TIM_AON_POS                 = 0x0
BMM350_DATA_X_7_0_MSK                       = 0xff
BMM350_DATA_X_7_0_POS                       = 0x0
BMM350_DATA_X_15_8_MSK                      = 0xff
BMM350_DATA_X_15_8_POS                      = 0x0
BMM350_DATA_X_23_16_MSK                     = 0xff
BMM350_DATA_X_23_16_POS                     = 0x0
BMM350_DATA_Y_7_0_MSK                       = 0xff
BMM350_DATA_Y_7_0_POS                       = 0x0
BMM350_DATA_Y_15_8_MSK                      = 0xff
BMM350_DATA_Y_15_8_POS                      = 0x0
BMM350_DATA_Y_23_16_MSK                     = 0xff
BMM350_DATA_Y_23_16_POS                     = 0x0
BMM350_DATA_Z_7_0_MSK                       = 0xff
BMM350_DATA_Z_7_0_POS                       = 0x0
BMM350_DATA_Z_15_8_MSK                      = 0xff
BMM350_DATA_Z_15_8_POS                      = 0x0
BMM350_DATA_Z_23_16_MSK                     = 0xff
BMM350_DATA_Z_23_16_POS                     = 0x0
BMM350_DATA_T_7_0_MSK                       = 0xff
BMM350_DATA_T_7_0_POS                       = 0x0
BMM350_DATA_T_15_8_MSK                      = 0xff
BMM350_DATA_T_15_8_POS                      = 0x0
BMM350_DATA_T_23_16_MSK                     = 0xff
BMM350_DATA_T_23_16_POS                     = 0x0
BMM350_DATA_ST_7_0_MSK                      = 0xff
BMM350_DATA_ST_7_0_POS                      = 0x0
BMM350_DATA_ST_15_8_MSK                     = 0xff
BMM350_DATA_ST_15_8_POS                     = 0x0
BMM350_DATA_ST_23_16_MSK                    = 0xff
BMM350_DATA_ST_23_16_POS                    = 0x0
BMM350_SIGN_INVERT_T_MSK                    = 0x10
BMM350_SIGN_INVERT_T_POS                    = 0x4
BMM350_SIGN_INVERT_X_MSK                    = 0x20
BMM350_SIGN_INVERT_X_POS                    = 0x5
BMM350_SIGN_INVERT_Y_MSK                    = 0x40
BMM350_SIGN_INVERT_Y_POS                    = 0x6
BMM350_SIGN_INVERT_Z_MSK                    = 0x80
BMM350_SIGN_INVERT_Z_POS                    = 0x7
BMM350_DIS_BR_NM_MSK                        = 0x1
BMM350_DIS_BR_NM_POS                        = 0x0
BMM350_DIS_FGR_NM_MSK                       = 0x2
BMM350_DIS_FGR_NM_POS                       = 0x1
BMM350_DIS_CRST_AT_ALL_MSK                  = 0x4
BMM350_DIS_CRST_AT_ALL_POS                  = 0x2
BMM350_DIS_BR_FM_MSK                        = 0x8
BMM350_DIS_BR_FM_POS                        = 0x3
BMM350_FRC_EN_BUFF_MSK                      = 0x1
BMM350_FRC_EN_BUFF_POS                      = 0x0
BMM350_FRC_INA_EN1_MSK                      = 0x2
BMM350_FRC_INA_EN1_POS                      = 0x1
BMM350_FRC_INA_EN2_MSK                      = 0x4
BMM350_FRC_INA_EN2_POS                      = 0x2
BMM350_FRC_ADC_EN_MSK                       = 0x8
BMM350_FRC_ADC_EN_POS                       = 0x3
BMM350_FRC_INA_RST_MSK                      = 0x10
BMM350_FRC_INA_RST_POS                      = 0x4
BMM350_FRC_ADC_RST_MSK                      = 0x20
BMM350_FRC_ADC_RST_POS                      = 0x5
BMM350_FRC_INA_XSEL_MSK                     = 0x1
BMM350_FRC_INA_XSEL_POS                     = 0x0
BMM350_FRC_INA_YSEL_MSK                     = 0x2
BMM350_FRC_INA_YSEL_POS                     = 0x1
BMM350_FRC_INA_ZSEL_MSK                     = 0x4
BMM350_FRC_INA_ZSEL_POS                     = 0x2
BMM350_FRC_ADC_TEMP_EN_MSK                  = 0x8
BMM350_FRC_ADC_TEMP_EN_POS                  = 0x3
BMM350_FRC_TSENS_EN_MSK                     = 0x10
BMM350_FRC_TSENS_EN_POS                     = 0x4
BMM350_DSENS_FM_MSK                         = 0x20
BMM350_DSENS_FM_POS                         = 0x5
BMM350_DSENS_SEL_MSK                        = 0x40
BMM350_DSENS_SEL_POS                        = 0x6
BMM350_DSENS_SHORT_MSK                      = 0x80
BMM350_DSENS_SHORT_POS                      = 0x7
BMM350_ERR_MISS_BR_DONE_MSK                 = 0x1
BMM350_ERR_MISS_BR_DONE_POS                 = 0x0
BMM350_ERR_MISS_FGR_DONE_MSK                = 0x2
BMM350_ERR_MISS_FGR_DONE_POS                = 0x1
BMM350_TST_CHAIN_LN_MODE_MSK                = 0x1
BMM350_TST_CHAIN_LN_MODE_POS                = 0x0
BMM350_TST_CHAIN_LP_MODE_MSK                = 0x2
BMM350_TST_CHAIN_LP_MODE_POS                = 0x1
BMM350_EN_OVWR_TMR_IF_MSK                   = 0x1
BMM350_EN_OVWR_TMR_IF_POS                   = 0x0
BMM350_TMR_CKTRIGB_MSK                      = 0x2
BMM350_TMR_CKTRIGB_POS                      = 0x1
BMM350_TMR_DO_BR_MSK                        = 0x4
BMM350_TMR_DO_BR_POS                        = 0x2
BMM350_TMR_DO_FGR_MSK                       = 0x18
BMM350_TMR_DO_FGR_POS                       = 0x3
BMM350_TMR_EN_OSC_MSK                       = 0x80
BMM350_TMR_EN_OSC_POS                       = 0x7
BMM350_VCM_TRIM_X_MSK                       = 0x1f
BMM350_VCM_TRIM_X_POS                       = 0x0
BMM350_VCM_TRIM_Y_MSK                       = 0x1f
BMM350_VCM_TRIM_Y_POS                       = 0x0
BMM350_VCM_TRIM_Z_MSK                       = 0x1f
BMM350_VCM_TRIM_Z_POS                       = 0x0
BMM350_VCM_TRIM_DSENS_MSK                   = 0x1f
BMM350_VCM_TRIM_DSENS_POS                   = 0x0
BMM350_TWLB_MSK                             = 0x30
BMM350_TWLB_POS                             = 0x4
BMM350_PRG_PLS_TIM_MSK                      = 0x30
BMM350_PRG_PLS_TIM_POS                      = 0x4
BMM350_OTP_OVWR_EN_MSK                      = 0x1
BMM350_OTP_OVWR_EN_POS                      = 0x0
BMM350_OTP_MEM_CLK_MSK                      = 0x2
BMM350_OTP_MEM_CLK_POS                      = 0x1
BMM350_OTP_MEM_CS_MSK                       = 0x4
BMM350_OTP_MEM_CS_POS                       = 0x2
BMM350_OTP_MEM_PGM_MSK                      = 0x8
BMM350_OTP_MEM_PGM_POS                      = 0x3
BMM350_OTP_MEM_RE_MSK                       = 0x10
BMM350_OTP_MEM_RE_POS                       = 0x4
BMM350_SAMPLE_RDATA_PLS_MSK                 = 0x80
BMM350_SAMPLE_RDATA_PLS_POS                 = 0x7
BMM350_CFG_FW_MSK                           = 0x1
BMM350_CFG_FW_POS                           = 0x0
BMM350_EN_BR_X_MSK                          = 0x2
BMM350_EN_BR_X_POS                          = 0x1
BMM350_EN_BR_Y_MSK                          = 0x4
BMM350_EN_BR_Y_POS                          = 0x2
BMM350_EN_BR_Z_MSK                          = 0x8
BMM350_EN_BR_Z_POS                          = 0x3
BMM350_CFG_PAUSE_TIME_MSK                   = 0x30
BMM350_CFG_PAUSE_TIME_POS                   = 0x4
BMM350_CFG_FGR_PLS_DUR_MSK                  = 0xf
BMM350_CFG_FGR_PLS_DUR_POS                  = 0x0
BMM350_CFG_BR_Z_ORDER_MSK                   = 0x10
BMM350_CFG_BR_Z_ORDER_POS                   = 0x4
BMM350_CFG_BR_XY_CHOP_MSK                   = 0x20
BMM350_CFG_BR_XY_CHOP_POS                   = 0x5
BMM350_CFG_BR_PLS_DUR_MSK                   = 0xc0
BMM350_CFG_BR_PLS_DUR_POS                   = 0x6
BMM350_ENABLE_BR_FGR_TEST_MSK               = 0x1
BMM350_ENABLE_BR_FGR_TEST_POS               = 0x0
BMM350_SEL_AXIS_MSK                         = 0xe
BMM350_SEL_AXIS_POS                         = 0x1
BMM350_TMR_CFG_TEST_CLK_EN_MSK              = 0x10
BMM350_TMR_CFG_TEST_CLK_EN_POS              = 0x4
BMM350_TEST_VAL_BITS_7DOWNTO0_MSK           = 0xff
BMM350_TEST_VAL_BITS_7DOWNTO0_POS           = 0x0
BMM350_TEST_VAL_BITS_8_MSK                  = 0x1
BMM350_TEST_VAL_BITS_8_POS                  = 0x0
BMM350_TEST_P_SAMPLE_MSK                    = 0x2
BMM350_TEST_P_SAMPLE_POS                    = 0x1
BMM350_TEST_N_SAMPLE_MSK                    = 0x4
BMM350_TEST_N_SAMPLE_POS                    = 0x2
BMM350_TEST_APPLY_TO_REM_MSK                = 0x10
BMM350_TEST_APPLY_TO_REM_POS                = 0x4
BMM350_UFO_TRM_OSC_RANGE_MSK                = 0xf
BMM350_UFO_TRM_OSC_RANGE_POS                = 0x0
BMM350_ISO_CHIP_ID_MSK                      = 0x78
BMM350_ISO_CHIP_ID_POS                      = 0x3
BMM350_ISO_I2C_DEV_ID_MSK                   = 0x80
BMM350_ISO_I2C_DEV_ID_POS                   = 0x7
BMM350_I3C_FREQ_BITS_1DOWNTO0_MSK           = 0xc
BMM350_I3C_FREQ_BITS_1DOWNTO0_POS           = 0x2
BMM350_I3C_IBI_MDB_SEL_MSK                  = 0x10
BMM350_I3C_IBI_MDB_SEL_POS                  = 0x4
BMM350_TC_ASYNC_EN_MSK                      = 0x20
BMM350_TC_ASYNC_EN_POS                      = 0x5
BMM350_TC_SYNC_EN_MSK                       = 0x40
BMM350_TC_SYNC_EN_POS                       = 0x6
BMM350_I3C_SCL_GATING_EN_MSK                = 0x80
BMM350_I3C_SCL_GATING_EN_POS                = 0x7
BMM350_I3C_INACCURACY_BITS_6DOWNTO0_MSK     = 0x7f
BMM350_I3C_INACCURACY_BITS_6DOWNTO0_POS     = 0x0
BMM350_EST_EN_X_MSK                         = 0x1
BMM350_EST_EN_X_POS                         = 0x0
BMM350_EST_EN_Y_MSK                         = 0x2
BMM350_EST_EN_Y_POS                         = 0x1
BMM350_CRST_DIS_MSK                         = 0x4
BMM350_CRST_DIS_POS                         = 0x2
BMM350_BR_TFALL_MSK                         = 0x7
BMM350_BR_TFALL_POS                         = 0x0
BMM350_BR_TRISE_MSK                         = 0x70
BMM350_BR_TRISE_POS                         = 0x4
BMM350_TMR_SOFT_START_DIS_MSK               = 0x80
BMM350_TMR_SOFT_START_DIS_POS               = 0x7
BMM350_FOSC_LOW_RANGE_MSK                   = 0x80
BMM350_FOSC_LOW_RANGE_POS                   = 0x7
BMM350_VCRST_TRIM_FG_MSK                    = 0x3f
BMM350_VCRST_TRIM_FG_POS                    = 0x0
BMM350_VCRST_TRIM_BR_MSK                    = 0x3f
BMM350_VCRST_TRIM_BR_POS                    = 0x0
BMM350_BG_TRIM_VRP_MSK                      = 0xc0
BMM350_BG_TRIM_VRP_POS                      = 0x6
BMM350_BG_TRIM_TC_MSK                       = 0xf
BMM350_BG_TRIM_TC_POS                       = 0x0
BMM350_BG_TRIM_VRA_MSK                      = 0xf0
BMM350_BG_TRIM_VRA_POS                      = 0x4
BMM350_BG_TRIM_VRD_MSK                      = 0xf
BMM350_BG_TRIM_VRD_POS                      = 0x0
BMM350_OVWR_REF_IB_EN_MSK                   = 0x10
BMM350_OVWR_REF_IB_EN_POS                   = 0x4
BMM350_OVWR_VDDA_EN_MSK                     = 0x20
BMM350_OVWR_VDDA_EN_POS                     = 0x5
BMM350_OVWR_VDDP_EN_MSK                     = 0x40
BMM350_OVWR_VDDP_EN_POS                     = 0x6
BMM350_OVWR_VDDS_EN_MSK                     = 0x80
BMM350_OVWR_VDDS_EN_POS                     = 0x7
BMM350_REF_IB_EN_MSK                        = 0x10
BMM350_REF_IB_EN_POS                        = 0x4
BMM350_VDDA_EN_MSK                          = 0x20
BMM350_VDDA_EN_POS                          = 0x5
BMM350_VDDP_EN_MSK                          = 0x40
BMM350_VDDP_EN_POS                          = 0x6
BMM350_VDDS_EN_MSK                          = 0x80
BMM350_VDDS_EN_POS                          = 0x7
BMM350_OVWR_OTP_PROG_VDD_SW_EN_MSK          = 0x8
BMM350_OVWR_OTP_PROG_VDD_SW_EN_POS          = 0x3
BMM350_OVWR_EN_MFE_BG_FILT_BYPASS_MSK       = 0x10
BMM350_OVWR_EN_MFE_BG_FILT_BYPASS_POS       = 0x4
BMM350_OTP_PROG_VDD_SW_EN_MSK               = 0x8
BMM350_OTP_PROG_VDD_SW_EN_POS               = 0x3
BMM350_CP_COMP_CRST_EN_TM_MSK               = 0x10
BMM350_CP_COMP_CRST_EN_TM_POS               = 0x4
BMM350_CP_COMP_VDD_EN_TM_MSK                = 0x20
BMM350_CP_COMP_VDD_EN_TM_POS                = 0x5
BMM350_CP_INTREFS_EN_TM_MSK                 = 0x40
BMM350_CP_INTREFS_EN_TM_POS                 = 0x6
BMM350_ADC_LOCAL_CHOP_EN_MSK                = 0x20
BMM350_ADC_LOCAL_CHOP_EN_POS                = 0x5
BMM350_INA_MODE_MSK                         = 0x40
BMM350_INA_MODE_POS                         = 0x6
BMM350_VDDD_EXT_EN_MSK                      = 0x20
BMM350_VDDD_EXT_EN_POS                      = 0x5
BMM350_VDDP_EXT_EN_MSK                      = 0x80
BMM350_VDDP_EXT_EN_POS                      = 0x7
BMM350_ADC_DSENS_EN_MSK                     = 0x10
BMM350_ADC_DSENS_EN_POS                     = 0x4
BMM350_DSENS_EN_MSK                         = 0x20
BMM350_DSENS_EN_POS                         = 0x5
BMM350_OTP_TM_CLVWR_EN_MSK                  = 0x40
BMM350_OTP_TM_CLVWR_EN_POS                  = 0x6
BMM350_OTP_VDDP_DIS_MSK                     = 0x80
BMM350_OTP_VDDP_DIS_POS                     = 0x7
BMM350_FORCE_HIGH_VREF_IREF_OK_MSK          = 0x10
BMM350_FORCE_HIGH_VREF_IREF_OK_POS          = 0x4
BMM350_FORCE_HIGH_FOSC_OK_MSK               = 0x20
BMM350_FORCE_HIGH_FOSC_OK_POS               = 0x5
BMM350_FORCE_HIGH_MFE_BG_RDY_MSK            = 0x40
BMM350_FORCE_HIGH_MFE_BG_RDY_POS            = 0x6
BMM350_FORCE_HIGH_MFE_VTMR_RDY_MSK          = 0x80
BMM350_FORCE_HIGH_MFE_VTMR_RDY_POS          = 0x7
BMM350_ERR_END_OF_RECHARGE_MSK              = 0x1
BMM350_ERR_END_OF_RECHARGE_POS              = 0x0
BMM350_ERR_END_OF_DISCHARGE_MSK             = 0x2
BMM350_ERR_END_OF_DISCHARGE_POS             = 0x1
BMM350_CP_TMX_DIGTP_SEL_MSK                 = 0x7
BMM350_CP_TMX_DIGTP_SEL_POS                 = 0x0
BMM350_CP_CPOSC_EN_TM_MSK                   = 0x80
BMM350_CP_CPOSC_EN_TM_POS                   = 0x7
BMM350_TST_ATM1_CFG_MSK                     = 0x3f
BMM350_TST_ATM1_CFG_POS                     = 0x0
BMM350_TST_TB1_EN_MSK                       = 0x80
BMM350_TST_TB1_EN_POS                       = 0x7
BMM350_TST_ATM2_CFG_MSK                     = 0x1f
BMM350_TST_ATM2_CFG_POS                     = 0x0
BMM350_TST_TB2_EN_MSK                       = 0x80
BMM350_TST_TB2_EN_POS                       = 0x7
BMM350_REG_DTB1X_SEL_MSK                    = 0x7f
BMM350_REG_DTB1X_SEL_POS                    = 0x0
BMM350_SEL_DTB1X_PAD_MSK                    = 0x80
BMM350_SEL_DTB1X_PAD_POS                    = 0x7
BMM350_REG_DTB2X_SEL_MSK                    = 0x7f
BMM350_REG_DTB2X_SEL_POS                    = 0x0
BMM350_TMR_TST_CFG_MSK                      = 0x7f
BMM350_TMR_TST_CFG_POS                      = 0x0
BMM350_TMR_TST_HIZ_VTMR_MSK                 = 0x80
BMM350_TMR_TST_HIZ_VTMR_POS                 = 0x7

# OTP MACROS
BMM350_OTP_CMD_DIR_READ                     = 0x20
BMM350_OTP_CMD_DIR_PRGM_1B                  = 0x40
BMM350_OTP_CMD_DIR_PRGM                     = 0x60
BMM350_OTP_CMD_PWR_OFF_OTP                  = 0x80
BMM350_OTP_CMD_EXT_READ                     = 0xA0
BMM350_OTP_CMD_EXT_PRGM                     = 0xE0
BMM350_OTP_CMD_MSK                          = 0xE0
BMM350_OTP_WORD_ADDR_MSK                    = 0x1F

BMM350_OTP_STATUS_ERROR_MSK                 = 0xE0
BMM350_OTP_STATUS_NO_ERROR                  = 0x00
BMM350_OTP_STATUS_BOOT_ERR                  = 0x20
BMM350_OTP_STATUS_PAGE_RD_ERR               = 0x40
BMM350_OTP_STATUS_PAGE_PRG_ERR              = 0x60
BMM350_OTP_STATUS_SIGN_ERR                  = 0x80
BMM350_OTP_STATUS_INV_CMD_ERR               = 0xA0
BMM350_OTP_STATUS_CMD_DONE                  = 0x01

# OTP indices
BMM350_TEMP_OFF_SENS                        = 0x0D

BMM350_MAG_OFFSET_X                         = 0x0E
BMM350_MAG_OFFSET_Y                         = 0x0F
BMM350_MAG_OFFSET_Z                         = 0x10

BMM350_MAG_SENS_X                           = 0x10
BMM350_MAG_SENS_Y                           = 0x11
BMM350_MAG_SENS_Z                           = 0x11

BMM350_MAG_TCO_X                            = 0x12
BMM350_MAG_TCO_Y                            = 0x13
BMM350_MAG_TCO_Z                            = 0x14

BMM350_MAG_TCS_X                            = 0x12
BMM350_MAG_TCS_Y                            = 0x13
BMM350_MAG_TCS_Z                            = 0x14

BMM350_MAG_DUT_T_0                          = 0x18

BMM350_CROSS_X_Y                            = 0x15
BMM350_CROSS_Y_X                            = 0x15
BMM350_CROSS_Z_X                            = 0x16
BMM350_CROSS_Z_Y                            = 0x16

BMM350_SENS_CORR_Y                          = 0.01
BMM350_TCS_CORR_Z                           = 0.000

# Signed bit macros
BMM350_SIGNED_8_BIT                         = 8
BMM350_SIGNED_12_BIT                        = 12
BMM350_SIGNED_16_BIT                        = 16
BMM350_SIGNED_21_BIT                        = 21
BMM350_SIGNED_24_BIT                        = 24

# Self-test macros
BMM350_SELF_TEST_DISABLE                    = 0x00
BMM350_SELF_TEST_POS_X                      = 0x0D
BMM350_SELF_TEST_NEG_X                      = 0x0B
BMM350_SELF_TEST_POS_Y                      = 0x15
BMM350_SELF_TEST_NEG_Y                      = 0x13

BMM350_X_FM_XP_UST_MAX_LIMIT                = 150
BMM350_X_FM_XP_UST_MIN_LIMIT                = 50

BMM350_X_FM_XN_UST_MAX_LIMIT                = -50
BMM350_X_FM_XN_UST_MIN_LIMIT                = -150

BMM350_Y_FM_YP_UST_MAX_LIMIT                = 150
BMM350_Y_FM_YP_UST_MIN_LIMIT                = 50

BMM350_Y_FM_YN_UST_MAX_LIMIT                = -50
BMM350_Y_FM_YN_UST_MIN_LIMIT                = -150

# PMU command status 0 macros
BMM350_PMU_CMD_STATUS_0_SUS                 = 0x00
BMM350_PMU_CMD_STATUS_0_NM                  = 0x01
BMM350_PMU_CMD_STATUS_0_UPD_OAE             = 0x02
BMM350_PMU_CMD_STATUS_0_FM                  = 0x03
BMM350_PMU_CMD_STATUS_0_FM_FAST             = 0x04
BMM350_PMU_CMD_STATUS_0_FGR                 = 0x05
BMM350_PMU_CMD_STATUS_0_FGR_FAST            = 0x06
BMM350_PMU_CMD_STATUS_0_BR                  = 0x07
BMM350_PMU_CMD_STATUS_0_BR_FAST             = 0x07


# PRESET MODE DEFINITIONS
BMM350_PRESETMODE_LOWPOWER                = 0x01
BMM350_PRESETMODE_REGULAR                 = 0x02
BMM350_PRESETMODE_HIGHACCURACY            = 0x03
BMM350_PRESETMODE_ENHANCED                = 0x04

LOW_THRESHOLD_INTERRUPT          = 0
HIGH_THRESHOLD_INTERRUPT         = 1
INTERRUPT_X_ENABLE               = 0
INTERRUPT_Y_ENABLE               = 0
INTERRUPT_Z_ENABLE               = 0
INTERRUPT_X_DISABLE              = 1
INTERRUPT_Y_DISABLE              = 1
INTERRUPT_Z_DISABLE              = 1
ENABLE_INTERRUPT_PIN             = 1
DISABLE_INTERRUPT_PIN            = 0
NO_DATA                          = -32768

# -------------------------------------------
BMM350_CHIP_ID_ERROR             = -1


# -------------------------------------------

BMM350_DISABLE_INTERRUPT = BMM350_DISABLE
BMM350_ENABLE_INTERRUPT = BMM350_ENABLE

BMM350_SUSPEND_MODE = BMM350_PMU_CMD_SUS
BMM350_NORMAL_MODE = BMM350_PMU_CMD_NM
BMM350_FORCED_MODE = BMM350_PMU_CMD_FM
BMM350_FORCED_MODE_FAST = BMM350_PMU_CMD_FM_FAST

BMM350_DATA_RATE_400HZ    = BMM350_ODR_400HZ
BMM350_DATA_RATE_200HZ    = BMM350_ODR_200HZ
BMM350_DATA_RATE_100HZ    = BMM350_ODR_100HZ
BMM350_DATA_RATE_50HZ     = BMM350_ODR_50HZ
BMM350_DATA_RATE_25HZ     = BMM350_ODR_25HZ
BMM350_DATA_RATE_12_5HZ   = BMM350_ODR_12_5HZ
BMM350_DATA_RATE_6_25HZ   = BMM350_ODR_6_25HZ
BMM350_DATA_RATE_3_125HZ  = BMM350_ODR_3_125HZ
BMM350_DATA_RATE_1_5625HZ = BMM350_ODR_1_5625HZ

BMM350_FLUXGUIDE_9MS = BMM350_PMU_CMD_FGR
BMM350_FLUXGUIDE_FAST = BMM350_PMU_CMD_FGR_FAST
BMM350_BITRESET_9MS = BMM350_PMU_CMD_BR
BMM350_BITRESET_FAST = BMM350_PMU_CMD_BR_FAST
BMM350_NOMAGRESET = 127

BMM350_INTR_DISABLE = BMM350_DISABLE
BMM350_INTR_ENABLE = BMM350_ENABLE

BMM350_UNMAP_FROM_PIN = BMM350_DISABLE
BMM350_MAP_TO_PIN = BMM350_ENABLE

BMM350_PULSED = BMM350_INT_MODE_PULSED
BMM350_LATCHED = BMM350_INT_MODE_LATCHED

BMM350_ACTIVE_LOW = BMM350_INT_POL_ACTIVE_LOW
BMM350_ACTIVE_HIGH = BMM350_INT_POL_ACTIVE_HIGH

BMM350_INTR_OPEN_DRAIN = BMM350_INT_OD_OPENDRAIN
BMM350_INTR_PUSH_PULL = BMM350_INT_OD_PUSHPULL

BMM350_IBI_DISABLE = BMM350_DISABLE
BMM350_IBI_ENABLE = BMM350_ENABLE

BMM350_NOCLEAR_ON_IBI = BMM350_DISABLE
BMM350_CLEAR_ON_IBI = BMM350_ENABLE

BMM350_I2C_WDT_DIS = BMM350_DISABLE
BMM350_I2C_WDT_EN = BMM350_ENABLE

BMM350_I2C_WDT_SEL_SHORT = BMM350_DISABLE
BMM350_I2C_WDT_SEL_LONG = BMM350_ENABLE

BMM350_NO_AVERAGING = BMM350_AVG_NO_AVG
BMM350_AVERAGING_2 = BMM350_AVG_2
BMM350_AVERAGING_4 = BMM350_AVG_4
BMM350_AVERAGING_8 = BMM350_AVG_8

BMM350_ST_IGEN_DIS = BMM350_DISABLE
BMM350_ST_IGEN_EN = BMM350_ENABLE

BMM350_ST_N_DIS = BMM350_DISABLE
BMM350_ST_N_EN = BMM350_ENABLE

BMM350_ST_P_DIS = BMM350_DISABLE
BMM350_ST_P_EN = BMM350_ENABLE

BMM350_IST_X_DIS = BMM350_DISABLE
BMM350_IST_X_EN = BMM350_ENABLE

BMM350_IST_Y_DIS = BMM350_DISABLE
BMM350_IST_Y_EN = BMM350_ENABLE

BMM350_CFG_SENS_TIM_AON_DIS = BMM350_DISABLE
BMM350_CFG_SENS_TIM_AON_EN = BMM350_ENABLE

BMM350_X_DIS = BMM350_DISABLE
BMM350_X_EN = BMM350_ENABLE

BMM350_Y_DIS = BMM350_DISABLE
BMM350_Y_EN = BMM350_ENABLE

BMM350_Z_DIS = BMM350_DISABLE
BMM350_Z_EN = BMM350_ENABLE

PI        = 3.141592653
M_PI	    = 3.14159265358979323846