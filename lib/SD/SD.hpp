#pragma once

#include <string>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "SPIInterface.hpp"

class SD {
public:
    SD(SPIInterface& spi, const char* mount_point, int cs_pin);
    ~SD();

    esp_err_t mount(bool format_if_failed = false);
    esp_err_t unmount();

    esp_err_t write_file(const char* path, const char* data);
    std::string read_file(const char* path);
    
    void print_info();

private:
    std::string mount_point;
    int cs_pin;
    SPIInterface& spi;
    sdmmc_card_t* card;
    bool is_mounted;
};