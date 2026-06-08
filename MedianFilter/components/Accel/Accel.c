// ADXL345 accelerometer

#include "Accel.h"
#include "I2C.h"

#include "esp_err.h"
#include "esp_log.h"

#define ACCEL_I2C_ADDR 0x53

#define REG_DEVID       0x00
#define VAL_DEVID       0xE5

#define REG_BW_RATE     0x2C
#define VAL_BW_RATE     0x0A /* Output data rate = 100 Hz */

#define REG_POWER_CTL   0x2D
#define VAL_POWER_CTL   0x08 /* Measurement mode. */

#define REG_DATA_FORMAT 0x31
#define VAL_DATA_FORMAT 0x08 /* Full resolution = 1.*/

#define REG_DATAX0      0x32

static const char *TAG = "Accel";

static i2c_master_dev_handle_t _hDev = NULL;

static inline void WriteReg(uint8_t reg, uint8_t value)
{
    I2C_WriteReg(_hDev, reg, value);
}

void Accel_Init(void)
{
    _hDev = I2C_AddDev(ACCEL_I2C_ADDR);

    if(NULL == _hDev)
    {
        ESP_LOGE(TAG, "Failed to add ADXL345 I2C device.");
        return;
    }

    uint8_t devid = I2C_ReadReg(_hDev, REG_DEVID);

    if(devid != VAL_DEVID)
    {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X", devid);
        return;
    }

    WriteReg(REG_BW_RATE, VAL_BW_RATE);

    WriteReg(REG_DATA_FORMAT, VAL_DATA_FORMAT);

    WriteReg(REG_POWER_CTL, VAL_POWER_CTL);
}

void Accel_ReadRaw(int16_t *pAccel)
{
    I2C_ReadData(_hDev, REG_DATAX0, (uint8_t *)pAccel, sizeof(int16_t) * 3);
}