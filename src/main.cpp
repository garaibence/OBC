#include "I2CInterface.hpp"
#include "MPU6050.hpp"
#include "BMP180.hpp"
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
        BMP180 bmp(i2c, 0x77);

        if (imu.init() == ESP_OK && bmp.init() == ESP_OK)
        {
            bmp.setOversampling(Oversampling::OSS_3);
            float ax, ay, az, gx, gy, gz, temp_mpu;
            float temp_bmp, pressure, altitude;
            while (true)
            {
                if (imu.readAccelerometer(ax, ay, az) == ESP_OK)
                {
                    printf(">ax:%f§m/s²\n", ax);
                    printf(">ay:%f§m/s²\n", ay);
                    printf(">az:%f§m/s²\n", az);
                }

                if (imu.readGyroscope(gx, gy, gz) == ESP_OK)
                {
                    printf(">gx:%f§°/s\n", gx);
                    printf(">gy:%f§°/s\n", gy);
                    printf(">gz:%f§°/s\n", gz);
                }

                if (imu.readTemperatureC(temp_mpu) == ESP_OK)
                    printf(">temp_mpu:%f§°C\n", temp_mpu);

                if (bmp.readTemperature(temp_bmp) == ESP_OK)
                    printf(">temp_bmp:%f§°C\n", temp_bmp);

                if (bmp.readPressure(pressure) == ESP_OK)
                    printf(">pressure:%f§Pa\n", pressure);

                if (bmp.readAltitude(altitude) == ESP_OK)
                    printf(">altitude:%f§m\n", altitude);
            }
        }

        i2c.deinit();
    }
}