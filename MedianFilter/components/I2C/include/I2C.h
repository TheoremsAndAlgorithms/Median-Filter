#pragma once

#include "driver/i2c_master.h"

void I2C_Init(void);

void I2C_Scan(void);

i2c_master_dev_handle_t I2C_AddDev(uint8_t addr);

void I2C_WriteReg(i2c_master_dev_handle_t hDev, uint8_t reg, uint8_t val);

uint8_t I2C_ReadReg(i2c_master_dev_handle_t hDev, uint8_t reg);

void I2C_ReadData(i2c_master_dev_handle_t hDev, uint8_t reg, uint8_t *pData, size_t len);