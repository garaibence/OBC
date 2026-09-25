#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <sdmmc_cmd.h>

class SD
{
public:
    SD(spi_host_device_t host, gpio_num_t mosi_gpio, gpio_num_t miso_gpio,
       gpio_num_t clk_gpio, gpio_num_t cs_gpio,
       const char *mount_point = "/sdcard",
       int max_freq_khz = 20000,
       uint32_t bus_max_transfer_sz = 4000,
       bool format_if_mount_failed = false,
       int max_open_files = 5,
       size_t allocation_unit_size = 16 * 1024);
    ~SD();

    SD(const SD &) = delete;
    SD &operator=(const SD &) = delete;

    esp_err_t mount();
    esp_err_t unmount();
    bool isMounted() const { return mounted; }

    esp_err_t writeFile(const char *relative_path, const void *data, size_t len,
                        bool append = false);
    esp_err_t writeTextFile(const char *relative_path, const char *text, bool append = false);

    esp_err_t readFile(const char *relative_path, void *buffer, size_t buffer_len,
                       size_t *out_len = nullptr);

    esp_err_t deleteFile(const char *relative_path);
    esp_err_t renameFile(const char *old_relative_path, const char *new_relative_path);
    bool fileExists(const char *relative_path) const;

    void printCardInfo(FILE *stream = stdout) const;

    const char *getMountPoint() const { return mount_point; }
    sdmmc_card_t *getCard() { return card; }

private:
    spi_host_device_t host_id;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t clk;
    gpio_num_t cs;
    char mount_point[32];
    int max_freq_khz;
    uint32_t bus_max_transfer_sz;
    bool format_if_mount_failed;
    int max_open_files;
    size_t allocation_unit_size;

    sdmmc_card_t *card;
    bool bus_initialized;
    bool mounted;

    esp_err_t buildPath(const char *relative_path, char *out, size_t out_size) const;
};