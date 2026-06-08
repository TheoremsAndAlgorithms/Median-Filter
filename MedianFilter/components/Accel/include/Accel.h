#pragma once

#include <stdint.h>

typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;

void Accel_Init(void);

void Accel_ReadRaw(accel_t *pAccel);