#include "SD.hpp"
#include <cstring>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <driver/sdspi_host.h>

static const char *TAG = "SD";

SD::SD(spi_host_device_t host, gpio_num_t mosi_gpio, gpio_num_t miso_gpio,
       gpio_num_t clk_gpio, gpio_num_t cs_gpio, const char *mount_point_in,
       int max_freq_khz, uint32_t bus_max_transfer_sz, bool format_if_mount_failed,
       int max_open_files, size_t allocation_unit_size)
    : host_id(host), mosi(mosi_gpio), miso(miso_gpio), clk(clk_gpio), cs(cs_gpio),
      max_freq_khz(max_freq_khz), bus_max_transfer_sz(bus_max_transfer_sz),
      format_if_mount_failed(format_if_mount_failed), max_open_files(max_open_files),
      allocation_unit_size(allocation_unit_size), card(nullptr), bus_initialized(false),
      mounted(false)
{
    strncpy(mount_point, mount_point_in, sizeof(mount_point) - 1);
    mount_point[sizeof(mount_point) - 1] = '\0';
}

SD::~SD()
{
    unmount();
}

esp_err_t SD::mount()
{
    if (mounted)
        return ESP_OK;

    spi_bus_config_t bus_cfg{};
    bus_cfg.mosi_io_num = mosi;
    bus_cfg.miso_io_num = miso;
    bus_cfg.sclk_io_num = clk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = bus_max_transfer_sz;

    esp_err_t err = spi_bus_initialize(host_id, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
        return err;
    }
    bus_initialized = true;

    sdmmc_host_t sdmmc_host = SDSPI_HOST_DEFAULT();
    sdmmc_host.slot = host_id;
    sdmmc_host.max_freq_khz = max_freq_khz;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs;
    slot_config.host_id = host_id;

    esp_vfs_fat_sdmmc_mount_config_t mount_config{};
    mount_config.format_if_mount_failed = format_if_mount_failed;
    mount_config.max_files = max_open_files;
    mount_config.allocation_unit_size = allocation_unit_size;

    ESP_LOGI(TAG, "Mounting filesystem at %s", mount_point);
    err = esp_vfs_fat_sdspi_mount(mount_point, &sdmmc_host, &slot_config, &mount_config, &card);
    if (err != ESP_OK)
    {
        if (err == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. Set format_if_mount_failed to "
                          "true if you want the card auto-formatted.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). Check wiring/pull-ups.",
                     esp_err_to_name(err));
        }
        spi_bus_free(host_id);
        bus_initialized = false;
        card = nullptr;
        return err;
    }

    mounted = true;
    ESP_LOGI(TAG, "Filesystem mounted");
    return ESP_OK;
}

esp_err_t SD::unmount()
{
    if (!mounted)
    {
        if (bus_initialized)
        {
            spi_bus_free(host_id);
            bus_initialized = false;
        }
        return ESP_OK;
    }

    esp_err_t err = esp_vfs_fat_sdcard_unmount(mount_point, card);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unmount filesystem: %s", esp_err_to_name(err));
    }
    card = nullptr;

    if (bus_initialized)
    {
        esp_err_t free_err = spi_bus_free(host_id);
        if (free_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(free_err));
            if (err == ESP_OK)
                err = free_err;
        }
        bus_initialized = false;
    }

    mounted = false;
    ESP_LOGI(TAG, "Card unmounted");
    return err;
}

esp_err_t SD::buildPath(const char *relative_path, char *out, size_t out_size) const
{
    if (relative_path == nullptr || out == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (relative_path[0] == '/')
        relative_path++;

    int written = snprintf(out, out_size, "%s/%s", mount_point, relative_path);
    if (written < 0 || static_cast<size_t>(written) >= out_size)
    {
        ESP_LOGE(TAG, "Path too long: %s/%s", mount_point, relative_path);
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t SD::writeFile(const char *relative_path, const void *data, size_t len, bool append)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "writeFile: card not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr && len > 0)
        return ESP_ERR_INVALID_ARG;

    char full_path[64];
    esp_err_t err = buildPath(relative_path, full_path, sizeof(full_path));
    if (err != ESP_OK)
        return err;

    FILE *f = fopen(full_path, append ? "ab" : "wb");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open %s for writing", full_path);
        return ESP_FAIL;
    }

    size_t written = (len > 0) ? fwrite(data, 1, len, f) : 0;
    fclose(f);

    if (written != len)
    {
        ESP_LOGE(TAG, "Short write to %s: wrote %u of %u bytes", full_path,
                 static_cast<unsigned>(written), static_cast<unsigned>(len));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SD::writeTextFile(const char *relative_path, const char *text, bool append)
{
    if (text == nullptr)
        return ESP_ERR_INVALID_ARG;
    return writeFile(relative_path, text, strlen(text), append);
}

esp_err_t SD::readFile(const char *relative_path, void *buffer, size_t buffer_len, size_t *out_len)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "readFile: card not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    if (buffer == nullptr && buffer_len > 0)
        return ESP_ERR_INVALID_ARG;

    char full_path[64];
    esp_err_t err = buildPath(relative_path, full_path, sizeof(full_path));
    if (err != ESP_OK)
        return err;

    FILE *f = fopen(full_path, "rb");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open %s for reading", full_path);
        return ESP_ERR_NOT_FOUND;
    }

    size_t read = fread(buffer, 1, buffer_len, f);
    fclose(f);

    if (out_len != nullptr)
        *out_len = read;
    return ESP_OK;
}

esp_err_t SD::deleteFile(const char *relative_path)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "deleteFile: card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[64];
    esp_err_t err = buildPath(relative_path, full_path, sizeof(full_path));
    if (err != ESP_OK)
        return err;

    if (unlink(full_path) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete %s", full_path);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SD::renameFile(const char *old_relative_path, const char *new_relative_path)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "renameFile: card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char old_path[64];
    char new_path[64];
    esp_err_t err = buildPath(old_relative_path, old_path, sizeof(old_path));
    if (err != ESP_OK)
        return err;
    err = buildPath(new_relative_path, new_path, sizeof(new_path));
    if (err != ESP_OK)
        return err;

    struct stat st;
    if (stat(new_path, &st) == 0)
    {
        unlink(new_path);
    }

    if (rename(old_path, new_path) != 0)
    {
        ESP_LOGE(TAG, "Failed to rename %s to %s", old_path, new_path);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool SD::fileExists(const char *relative_path) const
{
    if (!mounted)
        return false;

    char full_path[64];
    if (buildPath(relative_path, full_path, sizeof(full_path)) != ESP_OK)
        return false;

    struct stat st;
    return stat(full_path, &st) == 0;
}

void SD::printCardInfo(FILE *stream) const
{
    if (!mounted || card == nullptr)
    {
        ESP_LOGW(TAG, "printCardInfo: card not mounted");
        return;
    }
    sdmmc_card_print_info(stream, card);
}