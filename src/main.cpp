#include "I2CInterface.hpp"
#include "MPU6050.hpp"
#include "BMP180.hpp"
#include "QMC5883P.hpp"
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
        if (imu.init() != ESP_OK)
        {
            ESP_LOGE(TAG, "MPU6050 init failed");
            return;
        }
        imu.setAccelRange(AccelRange::RANGE_16G);
        imu.setDlpfBandwidth(DlpfBandwidth::BW_5HZ);
        imu.setGyroRange(GyroRange::RANGE_1000_DEG);
        imu.setI2CBypass(true);

        QMC5883P qmc(i2c, 0x2C);
        if (qmc.init() != ESP_OK)
        {
            ESP_LOGE(TAG, "QMC init failed");
            return;
        }

        BMP180 bmp(i2c, 0x77);
        if (bmp.init() != ESP_OK)
        {
            ESP_LOGE(TAG, "BMP init failed");
            return;
        }
        bmp.setOversampling(Oversampling::OSS_3);

        float ax, ay, az, gx, gy, gz, temp_mpu;
        float x, y, z;
        float temp_bmp, pressure, altitude;


        
        float avg_pres;
        bmp.readPressure(avg_pres);
        for (size_t i = 1; i < 50; i++)
        {
            float temp;
            bmp.readPressure(temp);
            avg_pres = (avg_pres + temp) / 2.0f;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        

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
            
            if (qmc.readMagneto(x, y, z) == ESP_OK)
            {
                printf(">mag_x:%f§µT\n", x);
                printf(">mag_y:%f§µT\n", y);
                printf(">mag_z:%f§µT\n", z);
            }
            
            if (bmp.readTemperature(temp_bmp) == ESP_OK)
                printf(">temp_bmp:%f§°C\n", temp_bmp);

            if (bmp.readPressure(pressure) == ESP_OK)
                printf(">pressure:%f§Pa\n", pressure);
            
            if (bmp.readAltitude(altitude, avg_pres) == ESP_OK)
                printf(">altitude:%f§m\n", altitude);

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        i2c.deinit();
    }
}