#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h" 
#include "driver/i2c_master.h"

#define MPU6050_I2C_ADDRESS         0x68u /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1       0x69u /*!< I2C address with AD0 pin high */

// Raw Acceeleration data
typedef struct
{
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6050_raw_acceleration_t;

 //Raw rotation data
typedef struct
{
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6050_raw_rotation_t;

// MPU6050 acceleration data, g
typedef struct
{
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6050_acceleration_t;

 // MPU6050 rotation data, °/s
typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_rotation_t;

 // Axes
typedef enum {
    MPU6050_X_AXIS = 0,
    MPU6050_Y_AXIS,
    MPU6050_Z_AXIS,
} axis_info_t;

 // Clock sources
typedef enum {
    MPU6050_CLOCK_INTERNAL = 0,
    MPU6050_CLOCK_PLL_X, 
    MPU6050_CLOCK_PLL_Y, 
    MPU6050_CLOCK_PLL_Z,  
    MPU6050_CLOCK_EXT_32768HZ, 
    MPU6050_CLOCK_EXT_19_2MHZ,
    MPU6050_CLOCK_RESERVED,
    MPU6050_CLOCK_STOP 
} clock_source_t;


 // Scale ranges for gyroscope
typedef enum {
    MPU6050_GYRO_RANGE_250 = 0,
    MPU6050_GYRO_RANGE_500 = 1,
    MPU6050_GYRO_RANGE_1000 = 2,
    MPU6050_GYRO_RANGE_2000 = 3,
} gyro_range_t;

 // Scale ranges for accelerometer
typedef enum {
    MPU6050_ACCEL_RANGE_2 = 0,
    MPU6050_ACCEL_RANGE_4 = 1,
    MPU6050_ACCEL_RANGE_8 = 2,
    MPU6050_ACCEL_RANGE_16 = 3,
} accel_range_t;

typedef struct {
    uint16_t i2c_address;  // I2C address of the device
    i2c_port_t i2c_port;  // I2C port (I2C_NUM_0, I2C_NUM_1)
    uint32_t counter;
    float dt;
    struct timeval *timer;
    
    struct {
        gyro_range_t gyro;   // Gyroscope range
        accel_range_t accel; // Accelerometer range
    } ranges;
} mod_dev_t;

typedef void *mpu6050_handle_t;

esp_err_t i2c_bus_init(void);
esp_err_t i2c_free_desc(i2c_port_t port);

esp_err_t i2c_sensor_mpu6050_init(mod_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
esp_err_t mpu6050_wake_up(mod_dev_t *dev);
esp_err_t set_clock_source(mod_dev_t *dev, clock_source_t clock_source);

esp_err_t mpu6050_get_raw_acceleration(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acceleration_t *const acce_value);
esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity);

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity);
esp_err_t mpu6050_get_raw_gyro(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_rotation_t *const gyro_value);

#endif 