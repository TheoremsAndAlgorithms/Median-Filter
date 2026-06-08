#include "I2C.h"
#include "LCD.h"
#include "Accel.h"

#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define DELAY 500000 /* uS */

#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

const char *TAG = "main";

void app_main(void)
{
    I2C_Init();
    I2C_Scan();

    // LCD_Init();

    Accel_Init();

    uint32_t cnt = 0;

    while(true)
    {
        cnt++;

        accel_t accel = {0};

        Accel_ReadRaw(&accel);

        float x_squared = accel.x * accel.x;
        float y_squared = accel.y * accel.y;
        float z_squared = accel.z * accel.z;

        float alpha = RAD_TO_DEG(atan2f(accel.x, sqrtf(y_squared + z_squared)));
        float beta  = RAD_TO_DEG(atan2f(accel.y, sqrtf(x_squared + z_squared)));
        float gamma = RAD_TO_DEG(atan2f(accel.z, sqrtf(x_squared + y_squared)));

        ESP_LOGI(TAG, "cnt: %lu, x: %d, y: %d, z: %d", cnt, accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "alpha: %.2f, beta: %.2f, gamma: %.2f\n", alpha, beta, gamma);

        esp_rom_delay_us(DELAY);
    }
}