#include <stdio.h>
#include "mpu6050.h"
#include "mpu6050_regs.h"
#include "driver/i2c.h"

#include <sys/time.h>
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_MASTER_SCL_IO    22  // GPIO number for SCL
#define I2C_MASTER_SDA_IO    21  // GPIO number for SDA
#define I2C_MASTER_PORT      I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ   100000  // Frequency of I2C clock (100 kHz)
#define I2C_MASTER_TIMEOUT   1000  // Timeout for I2C operations

//#define MPU6050_I2C_ADDR 0x68

#define ACCE_START_ADD 0x3B
#define GYRO_START_ADD 0x43

#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define CHECK(X) do { esp_err_t __; if ((__ = X) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "mpu6050";
static mpu6050_handle_t mpu6050 = NULL;


esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;


    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "I2C parameter config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "I2C driver installation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
    
}


esp_err_t i2c_free_desc(i2c_port_t port)
{
    esp_err_t err = i2c_driver_delete(port);
    return err;
}

static esp_err_t mpu6050_port_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mod_dev_t *sens = (mod_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, ACK_CHECK_EN);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, ACK_VAL);
    assert(ESP_OK == ret);
    ret = i2c_master_read_byte(cmd, data_buf + data_len, NACK_VAL);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6050_port_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mod_dev_t *dev = (mod_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, ACK_CHECK_EN);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t i2c_address)
{
    mod_dev_t *dev = (mod_dev_t *) calloc(1, sizeof(mod_dev_t));
    dev->i2c_port = port;
    dev->i2c_address = i2c_address << 1;
    dev->counter = 0;
    dev->dt = 0;
    dev->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6050_handle_t) dev;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const accel_range_t acce_fs, const gyro_range_t gyro_fs)
{
    esp_err_t ret;
    uint8_t gyro_config = gyro_fs << 3;
    uint8_t accel_config = acce_fs << 3;

    ret = mpu6050_port_write(sensor, MPU6050_REGISTER_GYRO_CONFIG, &gyro_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_port_write(sensor, MPU6050_REGISTER_ACCEL_CONFIG, &accel_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 configured: Accel Range=%d, Gyro Range=%d", acce_fs, gyro_fs);
    return ESP_OK;
}


esp_err_t i2c_sensor_mpu6050_init(mod_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    esp_err_t ret;

    // Create the MPU6050 sensor instance
    mpu6050_handle_t mpu6050 = mpu6050_create(I2C_MASTER_PORT, MPU6050_I2C_ADDRESS);

    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create returned NULL");
        return ESP_FAIL;  // Exit the function if sensor creation failed
    }

    // Configure the sensor with accelerometer and gyroscope ranges
    ret = mpu6050_config(mpu6050, MPU6050_ACCEL_RANGE_2, MPU6050_GYRO_RANGE_250);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050: %s", esp_err_to_name(ret));
        return ret;  // Exit if configuration fails
    }

    // Wake up the MPU6050
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;  // Exit if wake-up fails
    }

    ESP_LOGI(TAG, "MPU6050 initialized and configured successfully");
    return ESP_OK;
}

esp_err_t mpu6050_wake_up(mod_dev_t *dev)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_port_read(dev, MPU6050_REGISTER_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_port_write(dev, MPU6050_REGISTER_PWR_MGMT_1, &tmp, 1);
    return ret;
}


esp_err_t set_clock_source(mod_dev_t *dev, clock_source_t clock_source) {
    uint8_t pwr_mgmt_1_value = 0;

    // Read current value of the PWR_MGMT_1 register
    esp_err_t ret = mpu6050_port_read(dev, MPU6050_REGISTER_PWR_MGMT_1, &pwr_mgmt_1_value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to read PWR_MGMT_1 register");
        return ret;
    }

    // Clear the existing clock source bits (CLKS bits 0 and 1)
    pwr_mgmt_1_value &= ~(0x03);  // Mask out bits 0 and 1

    // Set the new clock source (CLKS bits 0 and 1)
    pwr_mgmt_1_value |= (clock_source & 0x03);  // Set new clock source

    // Write the updated value back to the PWR_MGMT_1 register
    return mpu6050_port_write(dev, MPU6050_REGISTER_PWR_MGMT_1, pwr_mgmt_1_value, 1);
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_port_read(sensor, MPU6050_REGISTER_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case MPU6050_ACCEL_RANGE_2:
        *acce_sensitivity = 16384;
        break;

    case MPU6050_ACCEL_RANGE_4:
        *acce_sensitivity = 8192;
        break;

    case MPU6050_ACCEL_RANGE_8:
        *acce_sensitivity = 4096;
        break;

    case MPU6050_ACCEL_RANGE_16:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acceleration_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    uint8_t raw_data[6];

    mod_dev_t *mpu6050 = (mod_dev_t *)sensor;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_acceleration(I2C_MASTER_PORT, raw_data, sizeof(raw_data));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_acce_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t raw_acce_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t raw_acce_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    acce_value->acce_x = raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce_z / acce_sensitivity;
    return ESP_OK;
}


esp_err_t mpu6050_get_raw_acceleration(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ACCE_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 /portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_port_read(sensor, MPU6050_REGISTER_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case MPU6050_GYRO_RANGE_250:
        *gyro_sensitivity = 131;
        break;

    case MPU6050_GYRO_RANGE_500:
        *gyro_sensitivity = 65.5;
        break;

    case MPU6050_GYRO_RANGE_1000:
        *gyro_sensitivity = 32.8;
        break;

    case MPU6050_GYRO_RANGE_2000:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, GYRO_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 /portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_rotation_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    uint8_t raw_data[6];

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }

    mod_dev_t *mpu6050 = (mod_dev_t *)sensor;
    ret = mpu6050_get_raw_gyro(I2C_MASTER_PORT, raw_data, sizeof(raw_data));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_gyro_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t raw_gyro_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t raw_gyro_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    gyro_value->gyro_x = raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

// Function to change accelerometer range
esp_err_t change_accel_range(mpu6050_handle_t sensor, accel_range_t new_range) {
    uint8_t accel_config = new_range << 3; // Shift new_range to bits 3 and 4
    esp_err_t ret = mpu6050_port_write(sensor, MPU6050_REGISTER_ACCEL_CONFIG, &accel_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer range: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Accelerometer range updated to: ±%dg", 2 << new_range);
    return ESP_OK;
}

// Function to change gyroscope range
esp_err_t change_gyro_range(mpu6050_handle_t sensor, gyro_range_t new_range) {
    uint8_t gyro_config = new_range << 3;
    esp_err_t ret = mpu6050_port_write(sensor, MPU6050_REGISTER_GYRO_CONFIG, &gyro_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope range: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Gyroscope range updated to: %d", new_range);
    return ESP_OK;
}