#include "SD.hpp"
#include "esp_log.h"
#include <cstdio>

static const char *TAG = "SD";

SD::SD(SPIInterface &spi, const char *mount_point, int cs_pin)
    : mount_point(mount_point), cs_pin(cs_pin), spi(spi), card(nullptr), is_mounted(false) {}

SD::~SD()
{
    unmount();
}

esp_err_t SD::mount(bool format_if_failed)
{
    if (is_mounted)
        return ESP_OK;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = format_if_failed;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = (int)spi.get_host_id();

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)cs_pin;

    slot_config.host_id = (spi_host_device_t)host.slot;

    esp_err_t err = esp_vfs_fat_sdspi_mount(mount_point.c_str(), &host, &slot_config, &mount_config, &card);

    if (err == ESP_OK)
    {
        is_mounted = true;
        ESP_LOGI(TAG, "SD Card mounted at %s", mount_point.c_str());
    }
    else
        ESP_LOGE(TAG, "Failed to mount SD Card: %s", esp_err_to_name(err));
    return err;
}

esp_err_t SD::unmount()
{
    if (!is_mounted)
        return ESP_OK;

    esp_err_t err = esp_vfs_fat_sdcard_unmount(mount_point.c_str(), card);
    if (err == ESP_OK)
    {
        is_mounted = false;
        card = nullptr;
        ESP_LOGI(TAG, "SD card unmounted");
    }
    else
        ESP_LOGE(TAG, "Failed to unmount SD card");
    return err;
}

esp_err_t SD::write_file(const char *path, const char *data)
{
    std::string full_path = mount_point + path;
    FILE *f = fopen(full_path.c_str(), "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Opened file was NULL");
        return ESP_FAIL;
    }

    fprintf(f, "%s", data);
    fclose(f);
    return ESP_OK;
}

std::string SD::read_file(const char *path)
{
    std::string full_path = mount_point + path;
    FILE *f = fopen(full_path.c_str(), "r");
    if (f == NULL)
        return "";

    char line[128];
    std::string result = "";
    if (fgets(line, sizeof(line), f))
    {
        result = line;
        size_t pos = result.find_last_of("\n");
        if (pos != std::string::npos)
            result.erase(pos);
    }
    fclose(f);
    return result;
}

void SD::print_info()
{
    if (is_mounted && card)
    {
        sdmmc_card_print_info(stdout, card);
    }
}