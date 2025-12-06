#include "I2CInterface.hpp"
#include "MPU6050.hpp"
#include <esp_log.h>

extern "C"
{
    void app_main()
    {
        static const char *TAG = "MAIN";
        ESP_LOGI(TAG, "starting");

        I2CInterface i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);
        if (i2c.init() != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c init failed");
            return;
        }

        MPU6050 imu(i2c, 0x68);
        if (imu.init() == ESP_OK)
        {
            uint8_t id;
            if (imu.whoami(id) == ESP_OK)
                ESP_LOGI(TAG, "MPU WHO_AM_I = 0x%02X", id);

            float ax, ay, az, gx, gy, gz;
            float temp;
            if (imu.readAccelerometer(ax, ay, az) == ESP_OK)
                ESP_LOGI(TAG, "Accel: [%.2f, %.2f, %.2f] g", ax, ay, az);
            if (imu.readGyroscope(gx, gy, gz) == ESP_OK)
                ESP_LOGI(TAG, "Gyro: [%.2f, %.2f, %.2f] °/s", gx, gy, gz);
            if (imu.readTemperatureC(temp) == ESP_OK)
                ESP_LOGI(TAG, "Temp: %.2f C", temp);
        }
        else
            ESP_LOGW(TAG, "MPU init failed");

        i2c.deinit();
    }
}