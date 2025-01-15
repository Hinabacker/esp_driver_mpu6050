#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>
#include <mpu6050_regs.h>

#define CONFIG_EXAMPLE_SDA_GPIO 21
#define CONFIG_EXAMPLE_SCL_GPIO 22
#define I2C_MASTER_PORT I2C_NUM_0 

#define BUFF_SIZE 6

static const char *TAG = "mpu6050_test";
mod_dev_t dev = { 0 };
esp_err_t ret;
static mpu6050_handle_t mpu6050 = NULL;

void disp_buf(uint8_t* buf, int len)
{
    int i;
    int16_t temp;
    for(i = 0; i < len; i++) {
        temp = ((buf[i] << 8) + buf[i +1]);
        printf("%d ", temp);
        i++;
    }
    printf("\n");
}

void mpu6050_test(void *pvParameters)
{
    ret = i2c_sensor_mpu6050_init(&dev, MPU6050_I2C_ADDRESS, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    uint8_t* data_rd = (uint8_t*) malloc(BUFF_SIZE);

    while (1)
    {
        mpu6050_acceleration_t accel_values;
        mpu6050_rotation_t gyro_values;

        ret = mpu6050_get_raw_acceleration(I2C_MASTER_PORT, data_rd, BUFF_SIZE);
        // printf("MPU6050 Acce Value\n");
        // if (ret == ESP_OK) {
        //     disp_buf(data_rd, BUFF_SIZE);
        // }

        ret = mpu6050_get_raw_gyro(I2C_MASTER_PORT, data_rd, BUFF_SIZE);
        // printf("MPU6050 raw gyro Value\n");
        // if (ret == ESP_OK) {
        //     disp_buf(data_rd, BUFF_SIZE);
        // }

        esp_err_t ret_a = mpu6050_get_acce(mpu6050, &accel_values);
        if (ret_a == ESP_OK) {
            printf("Accelerometer Values (g) :\n");
            printf("X: %.2f, Y: %.2f, Z: %.2f\n", accel_values.acce_x, accel_values.acce_y, accel_values.acce_z);
        } 
        else {
            printf("Failed to get accel values: %s\n", esp_err_to_name(ret));
        }

        esp_err_t ret_g = mpu6050_get_gyro(mpu6050, &gyro_values);
        if (ret_g == ESP_OK) {
            printf("Gyro Values (in dps):\n");
            printf("X: %.2f, Y: %.2f, Z: %.2f\n", gyro_values.gyro_x, gyro_values.gyro_y, gyro_values.gyro_z);
        } 
        else {
            printf("Failed to get gyro values: %s\n", esp_err_to_name(ret));
        }
        
        printf("%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",
            gyro_values.gyro_x, gyro_values.gyro_y, gyro_values.gyro_z,
            accel_values.acce_x, accel_values.acce_y, accel_values.acce_z);


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ret = i2c_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}