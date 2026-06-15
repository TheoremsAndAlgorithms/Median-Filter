#pragma once

#include <stdint.h>

void Accel_Init(void);

void Accel_ReadRaw(int16_t *pAccel);

float Accel_GetAcc_g(void);