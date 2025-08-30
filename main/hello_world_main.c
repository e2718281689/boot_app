#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

// --- 日志 TAG ---
static const char *TAG = "SAFE_BOOT";

// --- TF卡上的固件文件名 ---
#define OTA_FIRMWARE_PATH "/sdcard/spi_lcd_st7789.bin"

// --- 刷写缓冲区大小 ---
#define OTA_BUFFER_SIZE 4096

// --- TF卡 SPI 引脚配置 (请根据你的硬件连接修改) ---

#define SPI_HOST_ID   SPI2_HOST // 使用 VSPI
/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK       (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_MOSI       (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MISO       (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_RST        (-1)
#define EXAMPLE_LCD_GPIO_DC         (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS         (GPIO_NUM_3)

#define EXAMPLE_TF_GPIO_CS         (GPIO_NUM_5)

#define GPIO_OUTPUT_IO    (GPIO_NUM_9)
#define GPIO_OUTPUT_PIN_SEL  (1ULL << GPIO_OUTPUT_IO)

#define MOUNT_POINT "/sdcard"
#define EXAMPLE_LCD_SPI_NUM         (SPI2_HOST)
/**
 * @brief 初始化 SPI 总线并挂载 TF 卡
 *
 * @return esp_err_t ESP_OK 表示成功
 */
static esp_err_t mount_sdcard_spi(void)
{
    esp_err_t ret = ESP_OK;

    //初始化spi总线
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = EXAMPLE_LCD_GPIO_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };
    spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO);



    // 1. 配置 GPIO11 为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 2. 先置低电平
    gpio_set_level(GPIO_OUTPUT_IO, 0);


    //挂载tf卡
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = EXAMPLE_LCD_SPI_NUM;   // 重要：告诉 SD 驱动用我们刚初始化好的总线

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = EXAMPLE_TF_GPIO_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ SD 卡挂载失败，继续运行：%s", esp_err_to_name(ret));
        ret = ESP_FAIL; // ★ 把错误码复位成 OK
    } else {
        ESP_LOGI(TAG, "SD 卡已就绪");
        sdmmc_card_print_info(stdout, card);
    }

    return ret;

}

/**
 * @brief 从 TF 卡读取固件并刷写到 app0 分区
 *
 * @return true 如果成功刷写并准备重启
 * @return false 如果没有找到固件或刷写失败
 */
static bool flash_main_app_from_sd(void)
{
    FILE *f = fopen(OTA_FIRMWARE_PATH, "rb");
    if (f == NULL) {
        ESP_LOGI(TAG, "No firmware file '%s' found on TF card. Booting existing app.", OTA_FIRMWARE_PATH);
        return false;
    }

    ESP_LOGI(TAG, "Found new firmware on TF card. Starting update process...");

    // 修正 #2: 使用 ESP_PARTITION_SUBTYPE_ANY 来查找分区，主要依赖分区名 "app0"
    const esp_partition_t *update_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, "app0");

    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Target partition 'app0' not found!");
        fclose(f);
        return false;
    }

    ESP_LOGI(TAG, "Target partition: %s, size: %lu bytes", update_partition->label, (unsigned long)update_partition->size);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        fclose(f);
        return false;
    }

    char *ota_buffer = (char *)malloc(OTA_BUFFER_SIZE);
    if (!ota_buffer) {
        ESP_LOGE(TAG, "Failed to allocate OTA buffer");
        esp_ota_abort(ota_handle);
        fclose(f);
        return false;
    }
    
    int bytes_read = 0;
    while ((bytes_read = fread(ota_buffer, 1, OTA_BUFFER_SIZE, f)) > 0) {
        err = esp_ota_write(ota_handle, (const void *)ota_buffer, bytes_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            free(ota_buffer);
            fclose(f);
            return false;
        }
    }
    free(ota_buffer);
    fclose(f);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return false;
    }
    
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Firmware update successful! Restarting to launch the new application.");
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Safe Bootloader Started (IDF v5.4).");

    if (mount_sdcard_spi() == ESP_OK) {
        if (flash_main_app_from_sd()) {
            esp_restart();
        }
    }

    ESP_LOGI(TAG, "No update performed. Attempting to boot existing application...");
    
    // 修正 #3: 同样使用 ESP_PARTITION_SUBTYPE_ANY 来查找启动分区
    const esp_partition_t *boot_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, "app0");
        
    if (boot_partition != NULL) {
        esp_ota_set_boot_partition(boot_partition);
    } else {
        ESP_LOGE(TAG, "No bootable app partition (app0) found. Halting.");
        while(1);
    }
    
    esp_restart();
}
