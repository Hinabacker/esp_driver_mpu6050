#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

// MPU6050 Register Addresses

#define MPU6050_REGISTER_XG_OFFS_TC         (0x00)
#define MPU6050_REGISTER_YG_OFFS_TC         (0x01)
#define MPU6050_REGISTER_ZG_OFFS_TC         (0x02)
#define MPU6050_REGISTER_X_FINE_GAIN        (0x03)
#define MPU6050_REGISTER_Y_FINE_GAIN        (0x04)
#define MPU6050_REGISTER_Z_FINE_GAIN        (0x05)
#define MPU6050_REGISTER_XA_OFFS_H          (0x06)
#define MPU6050_REGISTER_XA_OFFS_L_TC       (0x07)
#define MPU6050_REGISTER_YA_OFFS_H          (0x08)
#define MPU6050_REGISTER_YA_OFFS_L_TC       (0x09)
#define MPU6050_REGISTER_ZA_OFFS_H          (0x0A)
#define MPU6050_REGISTER_ZA_OFFS_L_TC       (0x0B)
#define MPU6050_REGISTER_SELF_TEST_X        (0x0D)
#define MPU6050_REGISTER_SELF_TEST_Y        (0x0E)
#define MPU6050_REGISTER_SELF_TEST_Z        (0x0F)
#define MPU6050_REGISTER_SELF_TEST_A        (0x10)
#define MPU6050_REGISTER_XG_OFFS_USRH       (0x13)
#define MPU6050_REGISTER_XG_OFFS_USRL       (0x14)
#define MPU6050_REGISTER_YG_OFFS_USRH       (0x15)
#define MPU6050_REGISTER_YG_OFFS_USRL       (0x16)
#define MPU6050_REGISTER_ZG_OFFS_USRH       (0x17)
#define MPU6050_REGISTER_ZG_OFFS_USRL       (0x18)
#define MPU6050_REGISTER_SMPLRT_DIV         (0x19)
#define MPU6050_REGISTER_CONFIG             (0x1A)
#define MPU6050_REGISTER_GYRO_CONFIG        (0x1B)
#define MPU6050_REGISTER_ACCEL_CONFIG       (0x1C)
#define MPU6050_REGISTER_FF_THR             (0x1D)
#define MPU6050_REGISTER_FF_DUR             (0x1E)
#define MPU6050_REGISTER_MOT_THR            (0x1F)
#define MPU6050_REGISTER_MOT_DUR            (0x20)
#define MPU6050_REGISTER_ZRMOT_THR          (0x21)
#define MPU6050_REGISTER_ZRMOT_DUR          (0x22)
#define MPU6050_REGISTER_FIFO_EN            (0x23)
#define MPU6050_REGISTER_I2C_MST_CTRL       (0x24)
#define MPU6050_REGISTER_I2C_SLV0_ADDR      (0x25)
#define MPU6050_REGISTER_I2C_SLV0_REG       (0x26)
#define MPU6050_REGISTER_I2C_SLV0_CTRL      (0x27)
#define MPU6050_REGISTER_I2C_SLV1_ADDR      (0x28)
#define MPU6050_REGISTER_I2C_SLV1_REG       (0x29)
#define MPU6050_REGISTER_I2C_SLV1_CTRL      (0x2A)
#define MPU6050_REGISTER_I2C_SLV2_ADDR      (0x2B)
#define MPU6050_REGISTER_I2C_SLV2_REG       (0x2C)
#define MPU6050_REGISTER_I2C_SLV2_CTRL      (0x2D)
#define MPU6050_REGISTER_I2C_SLV3_ADDR      (0x2E)
#define MPU6050_REGISTER_I2C_SLV3_REG       (0x2F)
#define MPU6050_REGISTER_I2C_SLV3_CTRL      (0x30)
#define MPU6050_REGISTER_I2C_SLV4_ADDR      (0x31)
#define MPU6050_REGISTER_I2C_SLV4_REG       (0x32)
#define MPU6050_REGISTER_I2C_SLV4_DO        (0x33)
#define MPU6050_REGISTER_I2C_SLV4_CTRL      (0x34)
#define MPU6050_REGISTER_I2C_SLV4_DI        (0x35)
#define MPU6050_REGISTER_I2C_MST_STATUS     (0x36)
#define MPU6050_REGISTER_INT_PIN_CFG        (0x37)
#define MPU6050_REGISTER_INT_ENABLE         (0x38)
#define MPU6050_REGISTER_DMP_INT_STATUS     (0x39)
#define MPU6050_REGISTER_INT_STATUS         (0x3A)
#define MPU6050_REGISTER_ACCEL_XOUT_H       (0x3B)
#define MPU6050_REGISTER_ACCEL_XOUT_L       (0x3C)
#define MPU6050_REGISTER_ACCEL_YOUT_H       (0x3D)
#define MPU6050_REGISTER_ACCEL_YOUT_L       (0x3E)
#define MPU6050_REGISTER_ACCEL_ZOUT_H       (0x3F)
#define MPU6050_REGISTER_ACCEL_ZOUT_L       (0x40)
#define MPU6050_REGISTER_TEMP_OUT_H         (0x41)
#define MPU6050_REGISTER_TEMP_OUT_L         (0x42)
#define MPU6050_REGISTER_GYRO_XOUT_H        (0x43)
#define MPU6050_REGISTER_GYRO_XOUT_L        (0x44)
#define MPU6050_REGISTER_GYRO_YOUT_H        (0x45)
#define MPU6050_REGISTER_GYRO_YOUT_L        (0x46)
#define MPU6050_REGISTER_GYRO_ZOUT_H        (0x47)
#define MPU6050_REGISTER_GYRO_ZOUT_L        (0x48)
#define MPU6050_REGISTER_EXT_SENS_DATA_00   (0x49)
#define MPU6050_REGISTER_EXT_SENS_DATA_01   (0x4A)
#define MPU6050_REGISTER_EXT_SENS_DATA_02   (0x4B)
#define MPU6050_REGISTER_EXT_SENS_DATA_03   (0x4C)
#define MPU6050_REGISTER_EXT_SENS_DATA_04   (0x4D)
#define MPU6050_REGISTER_EXT_SENS_DATA_05   (0x4E)
#define MPU6050_REGISTER_EXT_SENS_DATA_06   (0x4F)
#define MPU6050_REGISTER_EXT_SENS_DATA_07   (0x50)
#define MPU6050_REGISTER_EXT_SENS_DATA_08   (0x51)
#define MPU6050_REGISTER_EXT_SENS_DATA_09   (0x52)
#define MPU6050_REGISTER_EXT_SENS_DATA_10   (0x53)
#define MPU6050_REGISTER_EXT_SENS_DATA_11   (0x54)
#define MPU6050_REGISTER_EXT_SENS_DATA_12   (0x55)
#define MPU6050_REGISTER_EXT_SENS_DATA_13   (0x56)
#define MPU6050_REGISTER_EXT_SENS_DATA_14   (0x57)
#define MPU6050_REGISTER_EXT_SENS_DATA_15   (0x58)
#define MPU6050_REGISTER_EXT_SENS_DATA_16   (0x59)
#define MPU6050_REGISTER_EXT_SENS_DATA_17   (0x5A)
#define MPU6050_REGISTER_EXT_SENS_DATA_18   (0x5B)
#define MPU6050_REGISTER_EXT_SENS_DATA_19   (0x5C)
#define MPU6050_REGISTER_EXT_SENS_DATA_20   (0x5D)
#define MPU6050_REGISTER_EXT_SENS_DATA_21   (0x5E)
#define MPU6050_REGISTER_EXT_SENS_DATA_22   (0x5F)
#define MPU6050_REGISTER_EXT_SENS_DATA_23   (0x60)
#define MPU6050_REGISTER_MOT_DETECT_STATUS  (0x61)
#define MPU6050_REGISTER_I2C_SLV0_DO        (0x63)
#define MPU6050_REGISTER_I2C_SLV1_DO        (0x64)
#define MPU6050_REGISTER_I2C_SLV2_DO        (0x65)
#define MPU6050_REGISTER_I2C_SLV3_DO        (0x66)
#define MPU6050_REGISTER_I2C_MST_DELAY_CTRL (0x67)
#define MPU6050_REGISTER_SIGNAL_PATH_RESET  (0x68)
#define MPU6050_REGISTER_MOT_DETECT_CTRL    (0x69)
#define MPU6050_REGISTER_USER_CTRL          (0x6A)
#define MPU6050_REGISTER_PWR_MGMT_1         (0x6B)
#define MPU6050_REGISTER_PWR_MGMT_2         (0x6C)
#define MPU6050_REGISTER_BANK_SEL           (0x6D)
#define MPU6050_REGISTER_MEM_START_ADDR     (0x6E)
#define MPU6050_REGISTER_MEM_R_W            (0x6F)
#define MPU6050_REGISTER_DMP_CFG_1          (0x70)
#define MPU6050_REGISTER_DMP_CFG_2          (0x71)
#define MPU6050_REGISTER_FIFO_COUNTH        (0x72)
#define MPU6050_REGISTER_FIFO_COUNTL        (0x73)
#define MPU6050_REGISTER_FIFO_R_W           (0x74)
#define MPU6050_REGISTER_WHO_AM_I           (0x75)

// MPU6050 Digital Low Pass Filter (DLPF) Configuration Values
#define MPU6050_DLPF_BW_256     (0x00)  // Bandwidth 256Hz
#define MPU6050_DLPF_BW_188     (0x01)  // Bandwidth 188Hz
#define MPU6050_DLPF_BW_98      (0x02)  // Bandwidth 98Hz
#define MPU6050_DLPF_BW_42      (0x03)  // Bandwidth 42Hz
#define MPU6050_DLPF_BW_20      (0x04)  // Bandwidth 20Hz
#define MPU6050_DLPF_BW_10      (0x05)  // Bandwidth 10Hz
#define MPU6050_DLPF_BW_5       (0x06)  // Bandwidth 5Hz

// MPU6050 Digital High Pass Filter (DHPF) Configuration Values
#define MPU6050_DHPF_RESET      (0x00)  // Reset filter
#define MPU6050_DHPF_5HZ        (0x01)  // 5Hz cutoff frequency
#define MPU6050_DHPF_2HZ        (0x02)  // 2Hz cutoff frequency
#define MPU6050_DHPF_1HZ        (0x03)  // 1Hz cutoff frequency
#define MPU6050_DHPF_0_5HZ      (0x04)  // 0.5Hz cutoff frequency

// MPU6050 Decrement Values for register configurations
#define MPU6050_DETECT_DECREMENT_RESET (0x0)  // Reset the decrement counter or disable the decrement functionality
#define MPU6050_DETECT_DECREMENT_1     (0x1)  // Decrement by 1
#define MPU6050_DETECT_DECREMENT_2     (0x2)  // Decrement by 2
#define MPU6050_DETECT_DECREMENT_4     (0x3)  // Decrement by 4

// External Synchronization configuration
#define MPU6050_EXT_SYNC_DISABLED     (0x0)  // No external synchronization
#define MPU6050_EXT_SYNC_TEMP_OUT_L   (0x1)  // Sync with temperature data (Low byte)
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  (0x2)  // Sync with Gyroscope X axis data (Low byte)
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  (0x3)  // Sync with Gyroscope Y axis data (Low byte)
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  (0x4)  // Sync with Gyroscope Z axis data (Low byte)
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L (0x5)  // Sync with Accelerometer X axis data (Low byte)
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L (0x6)  // Sync with Accelerometer Y axis data (Low byte)
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L (0x7)  // Sync with Accelerometer Z axis data (Low byte)



// MPU6050 Clock Division Values
#define MPU6050_CLOCK_DIV_348       (0x0)  // Clock division by 348
#define MPU6050_CLOCK_DIV_333       (0x1)  // Clock division by 333
#define MPU6050_CLOCK_DIV_320       (0x2)  // Clock division by 320
#define MPU6050_CLOCK_DIV_308       (0x3)  // Clock division by 308
#define MPU6050_CLOCK_DIV_296       (0x4)  // Clock division by 296
#define MPU6050_CLOCK_DIV_286       (0x5)  // Clock division by 286
#define MPU6050_CLOCK_DIV_276       (0x6)  // Clock division by 276
#define MPU6050_CLOCK_DIV_267       (0x7)  // Clock division by 267
#define MPU6050_CLOCK_DIV_258       (0x8)  // Clock division by 258
#define MPU6050_CLOCK_DIV_500       (0x9)  // Clock division by 500
#define MPU6050_CLOCK_DIV_471       (0xA)  // Clock division by 471
#define MPU6050_CLOCK_DIV_444       (0xB)  // Clock division by 444
#define MPU6050_CLOCK_DIV_421       (0xC)  // Clock division by 421
#define MPU6050_CLOCK_DIV_400       (0xD)  // Clock division by 400
#define MPU6050_CLOCK_DIV_381       (0xE)  // Clock division by 381
#define MPU6050_CLOCK_DIV_364       (0xF)  // Clock division by 364

// Self-test configuration for Accelerometer (XA, YA, ZA) and Gyroscope (XG, YG, ZG)
#define MPU6050_SELF_TEST_XA_1_BIT    (0x07)   // Bit position for XA self-test (part 1)
#define MPU6050_SELF_TEST_XA_1_LENGTH (0x03)   // Bit length for XA self-test (part 1)
#define MPU6050_SELF_TEST_XA_2_BIT    (0x05)   // Bit position for XA self-test (part 2)
#define MPU6050_SELF_TEST_XA_2_LENGTH (0x02)   // Bit length for XA self-test (part 2)

#define MPU6050_SELF_TEST_YA_1_BIT    (0x07)   // Bit position for YA self-test (part 1)
#define MPU6050_SELF_TEST_YA_1_LENGTH (0x03)   // Bit length for YA self-test (part 1)
#define MPU6050_SELF_TEST_YA_2_BIT    (0x03)   // Bit position for YA self-test (part 2)
#define MPU6050_SELF_TEST_YA_2_LENGTH (0x02)   // Bit length for YA self-test (part 2)

#define MPU6050_SELF_TEST_ZA_1_BIT    (0x07)   // Bit position for ZA self-test (part 1)
#define MPU6050_SELF_TEST_ZA_1_LENGTH (0x03)   // Bit length for ZA self-test (part 1)
#define MPU6050_SELF_TEST_ZA_2_BIT    (0x01)   // Bit position for ZA self-test (part 2)
#define MPU6050_SELF_TEST_ZA_2_LENGTH (0x02)   // Bit length for ZA self-test (part 2)

#define MPU6050_SELF_TEST_XG_1_BIT    (0x04)   // Bit position for XG self-test (part 1)
#define MPU6050_SELF_TEST_XG_1_LENGTH (0x05)   // Bit length for XG self-test (part 1)

#define MPU6050_SELF_TEST_YG_1_BIT    (0x04)   // Bit position for YG self-test (part 1)
#define MPU6050_SELF_TEST_YG_1_LENGTH (0x05)   // Bit length for YG self-test (part 1)

#define MPU6050_SELF_TEST_ZG_1_BIT    (0x04)   // Bit position for ZG self-test (part 1)
#define MPU6050_SELF_TEST_ZG_1_LENGTH (0x05)   // Bit length for ZG self-test (part 1)



// Bit and length defines for CONFIG register
// Configure the digital filters and synchronization options for the sensor

#define MPU6050_CFG_EXT_SYNC_SET_BIT    (3)        // Bit position for External Sync setting
#define MPU6050_CFG_EXT_SYNC_SET_MASK   (7 << MPU6050_CFG_EXT_SYNC_SET_BIT) // Mask for the 3 bits for EXT_SYNC setting

#define MPU6050_CFG_DLPF_CFG_BIT        (0)        // Bit position for Digital Low-Pass Filter configuration
#define MPU6050_CFG_DLPF_CFG_MASK       (7 << MPU6050_CFG_DLPF_CFG_BIT)   // Mask for DLPF configuration (3 bits for DLPF setting)

// Bit and length defines for GYRO_CONFIG register:
// The GYRO_CONFIG register sets the full-scale range for the gyroscope measurements.
#define MPU6050_GCONFIG_FS_SEL_BIT      (3)           // Bit position for Gyroscope Full-Scale selection
#define MPU6050_GCONFIG_FS_SEL_MASK     (3 << MPU6050_GCONFIG_FS_SEL_BIT) // Mask for Gyroscope full-scale range (2 bits)


// Bit and length defines for ACCEL_CONFIG register:
// The ACCEL_CONFIG register configures the accelerometer full-scale range, self-test, and high-pass filter.
#define MPU6050_ACONFIG_XA_ST_BIT       (7)    // Bit position for Accelerometer X-axis self-test
#define MPU6050_ACONFIG_YA_ST_BIT       (6)    // Bit position for Accelerometer Y-axis self-test
#define MPU6050_ACONFIG_ZA_ST_BIT       (5)    // Bit position for Accelerometer Z-axis self-test

#define MPU6050_ACONFIG_AFS_SEL_BIT     (3)    // Bit position for Accelerometer Full-Scale selection
#define MPU6050_ACONFIG_AFS_SEL_MASK    (3 << MPU6050_ACONFIG_AFS_SEL_BIT) // Mask for Full-Scale Range (2 bits)

#define MPU6050_ACONFIG_ACCEL_HPF_BIT   (0)    // Bit position for Accelerometer High-Pass Filter selection
#define MPU6050_ACONFIG_ACCEL_HPF_MASK  (7 << MPU6050_ACONFIG_ACCEL_HPF_BIT) // Mask for Accelerometer High-Pass Filter (3 bits)


// Bit and length defines for FIFO_EN register:
// The FIFO_EN register enables/disables the data streams (e.g., temperature, accelerometer, gyroscope) to be stored in the FIFO buffer.
#define MPU6050_TEMP_FIFO_EN_BIT    (7)    // Bit position for enabling the temperature data stream in FIFO
#define MPU6050_XG_FIFO_EN_BIT      (6)    // Bit position for enabling the X-axis gyroscope data stream in FIFO
#define MPU6050_YG_FIFO_EN_BIT      (5)    // Bit position for enabling the Y-axis gyroscope data stream in FIFO
#define MPU6050_ZG_FIFO_EN_BIT      (4)    // Bit position for enabling the Z-axis gyroscope data stream in FIFO
#define MPU6050_ACCEL_FIFO_EN_BIT   (3)    // Bit position for enabling the accelerometer data stream in FIFO
#define MPU6050_SLV2_FIFO_EN_BIT    (2)    // Bit position for enabling the slave 2 data stream in FIFO
#define MPU6050_SLV1_FIFO_EN_BIT    (1)    // Bit position for enabling the slave 1 data stream in FIFO
#define MPU6050_SLV0_FIFO_EN_BIT    (0)    // Bit position for enabling the slave 0 data stream in FIFO


// Bit and length defines for I2C_MST_CTRL register
// The I2C_MST_CTRL register configures the I2C Master control functionality.
#define MPU6050_MULT_MST_EN_BIT     (7)    // Bit position for enabling multiple I2C masters
#define MPU6050_WAIT_FOR_ES_BIT     (6)    // Bit position for enabling I2C master to wait for external signals
#define MPU6050_SLV_3_FIFO_EN_BIT   (5)    // Bit position for enabling slave 3 FIFO in I2C master mode
#define MPU6050_I2C_MST_P_NSR_BIT   (4)    // Bit position for I2C master to support non-stop read or write
#define MPU6050_I2C_MST_CLK_BIT     (0)    // Bit position for configuring the I2C master clock divider
#define MPU6050_I2C_MST_CLK_MASK    (7 << MPU6050_I2C_MST_CLK_BIT) // Mask for the clock divider setting (3 bits)


// Bit and length defines for I2C_SLV* register
// These are for configuring the slave devices in I2C master mode. There are multiple I2C slave registers (SLV0 to SLV4).
#define MPU6050_I2C_SLV_RW_BIT          (7)    // Bit position for read/write operation (1 for write, 0 for read)
#define MPU6050_I2C_SLV_ADDR_BIT        (6)    // Bit position for slave device address (7-bit address)
#define MPU6050_I2C_SLV_ADDR_LENGTH     (7)    // Length of the slave address (7 bits)
#define MPU6050_I2C_SLV_EN_BIT          (7)    // Bit position for enabling the I2C slave device
#define MPU6050_I2C_SLV_BYTE_SW_BIT     (6)    // Bit position for enabling byte swap on I2C slave data
#define MPU6050_I2C_SLV_REG_DIS_BIT     (5)    // Bit position for disabling register address for I2C slave
#define MPU6050_I2C_SLV_GRP_BIT         (4)    // Bit position for enabling the group for the I2C slave
#define MPU6050_I2C_SLV_LEN_BIT         (0)    // Bit position for configuring the length of data to be read/written from/to I2C slave
#define MPU6050_I2C_SLV_LEN_MASK        (7 << MPU6050_I2C_SLV_LEN_BIT) // Mask for the length of the data from/to I2C slave (3 bits)


// Bit and length defines for I2C_SLV4 register:
// These are for configuring the 4th I2C slave device in master mode (similar to SLV0 to SLV3).
#define MPU6050_I2C_SLV4_RW_BIT         (7)    // Bit position for read/write operation on I2C slave 4
#define MPU6050_I2C_SLV4_ADDR_BIT       (6)    // Bit position for slave 4 address (7-bit address)
#define MPU6050_I2C_SLV4_ADDR_LENGTH    (7)    // Length of slave 4 address (7 bits)
#define MPU6050_I2C_SLV4_EN_BIT         (7)    // Bit position for enabling I2C slave 4
#define MPU6050_I2C_SLV4_INT_EN_BIT     (6)    // Bit position for enabling interrupt for slave 4
#define MPU6050_I2C_SLV4_REG_DIS_BIT    (5)    // Bit position for disabling register address for I2C slave 4
#define MPU6050_I2C_SLV4_MST_DLY_BIT    (4)    // Bit position for setting master delay for slave 4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH (5)    // Length for master delay configuration (5 bits)

// Bit and length defines for I2C_MST_STATUS register:
// This register contains the status of the I2C master and any potential issues with the I2C communication.
#define MPU6050_MST_PASS_THROUGH_BIT    (7)    // Bit position for Pass-Through mode, indicating if the I2C master is in pass-through mode
#define MPU6050_MST_I2C_SLV4_DONE_BIT   (6)    // Bit position indicating if the I2C slave 4 transaction is completed
#define MPU6050_MST_I2C_LOST_ARB_BIT    (5)    // Bit position for detecting if I2C arbitration is lost (used when multiple masters are involved)
#define MPU6050_MST_I2C_SLV4_NACK_BIT   (4)    // Bit position for indicating a NACK (No Acknowledgment) response from slave 4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   (3)    // Bit position for indicating a NACK response from slave 3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   (2)    // Bit position for indicating a NACK response from slave 2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   (1)    // Bit position for indicating a NACK response from slave 1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   (0)    // Bit position for indicating a NACK response from slave 0

// Bit and length defines for INT_PIN_CFG register:
// This register controls the configuration of the interrupt pin (INT).
#define MPU6050_INTCFG_INT_LEVEL_BIT       (7)  // Bit position to set interrupt level (active high/low)
#define MPU6050_INTCFG_INT_OPEN_BIT        (6)  // Bit position to configure interrupt pin as open-drain or push-pull
#define MPU6050_INTCFG_LATCH_INT_EN_BIT    (5)  // Bit position to enable latch interrupt function
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT    (4)  // Bit position to configure interrupt status register clearing behavior
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT (3)  // Bit position to set FSYNC interrupt level (active high/low)
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT    (2)  // Bit position to enable FSYNC interrupt
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT   (1)  // Bit position to enable the I2C bypass mode (bypassing the MPU6050's internal I2C master)
#define MPU6050_INTCFG_CLKOUT_EN_BIT       (0)  // Bit position to enable the CLKOUT (clock output) pin

// Bit and length defines for INT_ENABLE and INT_STATUS registers:
// These registers enable or reflect the status of specific interrupts triggered by various conditions.
#define MPU6050_INTERRUPT_FF_BIT          (7)   // Bit position for Free-Fall interrupt
#define MPU6050_INTERRUPT_MOT_BIT         (6)   // Bit position for Motion detection interrupt
#define MPU6050_INTERRUPT_ZMOT_BIT        (5)   // Bit position for Zero Motion detection interrupt
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT  (4)   // Bit position for FIFO overflow interrupt
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT (3)   // Bit position for I2C master interrupt
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT (2)   // Bit position for PLL ready interrupt (when the PLL is locked)
#define MPU6050_INTERRUPT_DMP_INT_BIT     (1)   // Bit position for Digital Motion Processor interrupt (when DMP is triggered)
#define MPU6050_INTERRUPT_DATA_RDY_BIT    (0)   // Bit position for Data Ready interrupt (when new sensor data is available)

// Bit and length defines for MOT_DETECT_STATUS register:
// This register contains status flags indicating motion detection on each axis.
#define MPU6050_MOTION_MOT_XNEG_BIT     (7)    // Bit position for negative X-axis motion detection
#define MPU6050_MOTION_MOT_XPOS_BIT     (6)    // Bit position for positive X-axis motion detection
#define MPU6050_MOTION_MOT_YNEG_BIT     (5)    // Bit position for negative Y-axis motion detection
#define MPU6050_MOTION_MOT_YPOS_BIT     (4)    // Bit position for positive Y-axis motion detection
#define MPU6050_MOTION_MOT_ZNEG_BIT     (3)    // Bit position for negative Z-axis motion detection
#define MPU6050_MOTION_MOT_ZPOS_BIT     (2)    // Bit position for positive Z-axis motion detection
#define MPU6050_MOTION_MOT_ZRMOT_BIT    (0)    // Bit position for detecting zero-motion event on the Z-axis

// Bit and length defines for I2C_MST_DELAY_CTRL register:
// This register controls the delays for the I2C slave devices in the master mode.
#define MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT (7)  // Bit position for enabling the shadow registers for delay setup (used for synchronization)
#define MPU6050_DLYCTRL_I2C_SLV4_DLY_EN_BIT (4)  // Bit position for enabling delay for I2C slave 4 transactions
#define MPU6050_DLYCTRL_I2C_SLV3_DLY_EN_BIT (3)  // Bit position for enabling delay for I2C slave 3 transactions
#define MPU6050_DLYCTRL_I2C_SLV2_DLY_EN_BIT (2)  // Bit position for enabling delay for I2C slave 2 transactions
#define MPU6050_DLYCTRL_I2C_SLV1_DLY_EN_BIT (1)  // Bit position for enabling delay for I2C slave 1 transactions
#define MPU6050_DLYCTRL_I2C_SLV0_DLY_EN_BIT (0)  // Bit position for enabling delay for I2C slave 0 transactions

// Bit and length defines for SIGNAL_PATH_RESET register:
// This register is used to reset the sensor signal paths (Gyroscope, Accelerometer, and Temperature).
#define MPU6050_PATHRESET_GYRO_RESET_BIT  (2)    // Bit position for resetting the Gyroscope signal path
#define MPU6050_PATHRESET_ACCEL_RESET_BIT (1)    // Bit position for resetting the Accelerometer signal path
#define MPU6050_PATHRESET_TEMP_RESET_BIT  (0)    // Bit position for resetting the Temperature sensor signal path

// Bit and length defines for MOT_DETECT_CTRL register:
// This register configures the motion detection behavior, including acceleration delay and thresholds.
#define MPU6050_DETECT_ACCEL_DELAY_BIT    (4)    // Bit position to configure the acceleration delay (how long to wait before motion is detected)
#define MPU6050_DETECT_ACCEL_DELAY_MASK   (3 << MPU6050_DETECT_ACCEL_DELAY_BIT)  // Mask for the acceleration delay configuration
#define MPU6050_DETECT_FF_COUNT_BIT       (2)    // Bit position to configure the free-fall detection count (number of samples for free-fall event)
#define MPU6050_DETECT_FF_COUNT_MASK      (3 << MPU6050_DETECT_FF_COUNT_BIT)    // Mask for the free-fall detection count
#define MPU6050_DETECT_MOT_COUNT_BIT      (0)    // Bit position to configure the motion detection count (number of samples for motion detection)
#define MPU6050_DETECT_MOT_COUNT_MASK     (3 << MPU6050_DETECT_MOT_COUNT_BIT)    // Mask for the motion detection count

// Bit and length defines for USER_CTRL register:
// This register controls various features, such as enabling the DMP, FIFO, I2C master, and resets for the different subsystems.
#define MPU6050_USERCTRL_DMP_EN_BIT         (7)    // Bit position to enable the Digital Motion Processor (DMP)
#define MPU6050_USERCTRL_FIFO_EN_BIT        (6)    // Bit position to enable the FIFO (First-In-First-Out buffer)
#define MPU6050_USERCTRL_I2C_MST_EN_BIT     (5)    // Bit position to enable the I2C master interface
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT     (4)    // Bit position to disable the I2C interface (useful when bypassing I2C)
#define MPU6050_USERCTRL_DMP_RESET_BIT      (3)    // Bit position to reset the Digital Motion Processor (DMP)
#define MPU6050_USERCTRL_FIFO_RESET_BIT     (2)    // Bit position to reset the FIFO buffer
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT  (1)    // Bit position to reset the I2C master interface
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT (0)    // Bit position to reset the signal conditioning (used for gyro and accelerometer)


// Bit and length defines for PWR_MGMT_1 register:
// This register is used to configure power management settings, including sleep modes and clock source selection.
#define MPU6050_PWR1_DEVICE_RESET_BIT (7)  // Bit position for resetting the MPU6050 device
#define MPU6050_PWR1_SLEEP_BIT        (6)  // Bit position to enable the sleep mode, reducing power consumption
#define MPU6050_PWR1_CYCLE_BIT        (5)  // Bit position to enable cycling sleep mode (wakes up periodically)
#define MPU6050_PWR1_TEMP_DIS_BIT     (3)  // Bit position to disable the temperature sensor
#define MPU6050_PWR1_CLKSEL_BIT       (0)  // Bit position to select the clock source for the MPU6050
#define MPU6050_PWR1_CLKSEL_MASK      (7 << MPU6050_PWR1_CLKSEL_BIT)  // Mask for selecting the clock source, with 3 possible values

// Bit and length defines for PWR_MGMT_2 register:
// This register allows enabling/disabling various sensors and controls the wake-up behavior.
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT    (6)  // Bit position to control the low-power wake-up behavior
#define MPU6050_PWR2_LP_WAKE_CTRL_MASK   (3 << MPU6050_PWR2_LP_WAKE_CTRL_BIT)  // Mask to configure low-power wake-up control (2-bit field)
#define MPU6050_PWR2_STBY_XA_BIT         (5)  // Bit position to put the X-axis accelerometer in standby mode
#define MPU6050_PWR2_STBY_YA_BIT         (4)  // Bit position to put the Y-axis accelerometer in standby mode
#define MPU6050_PWR2_STBY_ZA_BIT         (3)  // Bit position to put the Z-axis accelerometer in standby mode
#define MPU6050_PWR2_STBY_XG_BIT         (2)  // Bit position to put the X-axis gyroscope in standby mode
#define MPU6050_PWR2_STBY_YG_BIT         (1)  // Bit position to put the Y-axis gyroscope in standby mode
#define MPU6050_PWR2_STBY_ZG_BIT         (0)  // Bit position to put the Z-axis gyroscope in standby mode

// Bit and length defines for WHO_AM_I register:
// This register returns a unique identifier for the MPU6050 device. The ID can be used to verify that the sensor is present and correctly connected.
#define MPU6050_WHO_AM_I_BIT        (1)  // Bit position for the WHO_AM_I register (returns device ID)
#define MPU6050_WHO_AM_I_MASK       (0x3f << MPU6050_WHO_AM_I_BIT)  // Mask to extract the device ID (6 bits of data)

// Undocumented bits and lengths (These are not part of the standard register documentation but might still be used internally):
#define MPU6050_TC_PWR_MODE_BIT     (7)  // Bit position to control power mode for internal temperature compensation (undocumented)
#define MPU6050_TC_OFFSET_BIT       (6)  // Bit position for controlling the temperature compensation offset (undocumented)
#define MPU6050_TC_OFFSET_LENGTH    (6)  // Length of the offset field for temperature compensation (undocumented)
#define MPU6050_TC_OTP_BNK_VLD_BIT  (0)  // Bit position for verifying the validity of the OTP (One-Time Programmable) bank (undocumented)
#define MPU6050_DMPINT_5_BIT        (5)  // Bit position for Digital Motion Processor interrupt 5 (undocumented)
#define MPU6050_DMPINT_4_BIT        (4)  // Bit position for Digital Motion Processor interrupt 4 (undocumented)
#define MPU6050_DMPINT_3_BIT        (3)  // Bit position for Digital Motion Processor interrupt 3 (undocumented)
#define MPU6050_DMPINT_2_BIT        (2)  // Bit position for Digital Motion Processor interrupt 2 (undocumented)
#define MPU6050_DMPINT_1_BIT        (1)  // Bit position for Digital Motion Processor interrupt 1 (undocumented)
#define MPU6050_DMPINT_0_BIT        (0)  // Bit position for Digital Motion Processor interrupt 0 (undocumented)



#endif