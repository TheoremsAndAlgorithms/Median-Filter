#include "I2C.h"

#include "esp_log.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_FREQ_HZ  100000

#define I2C_SCAN_ADDR_START 0x03
#define I2C_SCAN_ADDR_END   0x77
#define I2C_TIMEOUT         1000  // ms

static const char *TAG = "I2C";

static i2c_master_bus_handle_t _hBus = NULL;

void I2C_Init(void)
{
    i2c_master_bus_config_t config = 
    {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = I2C_SDA,
        .scl_io_num                   = I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&config, &_hBus));
}

void I2C_Scan(void)
{
    if(_hBus == NULL)
    {
        ESP_LOGE(TAG, "I2C bus is not initialized.");
        return;
    }

    uint8_t cnt = 0;

    ESP_LOGI(TAG, "Scanning for devices.");

    for(uint8_t addr = I2C_SCAN_ADDR_START; addr <= I2C_SCAN_ADDR_END; addr++)
    {
        esp_err_t err = i2c_master_probe(_hBus, addr, I2C_TIMEOUT);

        if(!err)
        {
            ESP_LOGI(TAG, "Found device at address 0x%02X.", addr);
            cnt++;
        }
    }

    ESP_LOGI(TAG, "Scan finished. Number of found devices is %u.", cnt);
}

i2c_master_dev_handle_t I2C_AddDev(uint8_t addr)
{
    i2c_device_config_t config = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t hDev = NULL;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(_hBus, &config, &hDev));

    return hDev;
}

void I2C_WriteReg(i2c_master_dev_handle_t hDev, uint8_t reg, uint8_t val)
{
    uint8_t data[] = { reg, val };
    ESP_ERROR_CHECK(i2c_master_transmit(hDev, data, sizeof(data), I2C_TIMEOUT));
}

uint8_t I2C_ReadReg(i2c_master_dev_handle_t hDev, uint8_t reg)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(hDev, &reg, sizeof(uint8_t), &val, sizeof(uint8_t), I2C_TIMEOUT));

    return val;
}

void I2C_ReadData(i2c_master_dev_handle_t hDev, uint8_t reg, uint8_t *pData, size_t len)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(hDev, &reg, sizeof(uint8_t), pData, len, I2C_TIMEOUT));
}
