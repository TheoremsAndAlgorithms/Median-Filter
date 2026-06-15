#include "I2C.h"
#include "LCD.h"
#include "Accel.h"

#include "esp_rom_sys.h"
#include "esp_random.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define WIN_LEN 32

#define DELAY 20000 /* us */

#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

const char *TAG = "main";

typedef enum
{
    ALPHA = 0,
    BETA,
    GAMMA,
    ANGLE_COUNT
} angle_t;

typedef enum
{
    X = 0,
    Y,
    Z,
    ELEM_COUNT
} elem_t;

typedef struct
{
    uint32_t normSq;
    int16_t  elem[ELEM_COUNT];
} vec_t;

typedef struct node node_t;

struct node
{
    vec_t   vec;

    node_t *pNext;
    node_t *pPrev;
};

typedef struct
{
    node_t win[WIN_LEN];

    uint8_t idx;
    uint8_t cnt;

    node_t *pMin;
    node_t *pMed;
} mmf_t;

esp_err_t MMF_Init(mmf_t *pMmf)
{
    if(pMmf == NULL)
    {
        ESP_LOGE(TAG, "MMF_Init fail: INVALID ARGUMENT");
        return ESP_ERR_INVALID_ARG;
    }

    memset(pMmf, 0, sizeof(mmf_t));

    node_t *pNode = &pMmf->win[0];

    pNode->pNext = pNode;
    pNode->pPrev = pNode;

    pMmf->pMin = pNode;
    pMmf->pMed = pNode;

    return ESP_OK;
}

esp_err_t MMF_Update(mmf_t *pMmf, int16_t smp[ELEM_COUNT])
{
    if(pMmf == NULL || smp == NULL)
    {
        ESP_LOGE(TAG, "MMF_Update fail: INVALID ARGUMENT");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t normSq = 0;

    for(elem_t el = X; el < ELEM_COUNT; el++)
    {
        normSq += smp[el] * smp[el];
    }

    node_t *pNode = &pMmf->win[pMmf->idx];

    if(pMmf->cnt < WIN_LEN)
    {
        if(pMmf->cnt % 2 == 0)
        {
            pMmf->pMed = pMmf->pMed->pPrev;
        }

        pMmf->cnt++;
    }
    else
    {
        if(pNode == pMmf->pMed || pNode->vec.normSq > pMmf->pMed->vec.normSq)
        {
            pMmf->pMed = pMmf->pMed->pPrev;
        }
        else if(pNode == pMmf->pMin)
        {
            pMmf->pMin = pNode->pNext;
        }

        pNode->pNext->pPrev = pNode->pPrev;
        pNode->pPrev->pNext = pNode->pNext;
    }

    memcpy(pNode->vec.elem, smp, sizeof(pNode->vec.elem));
    pNode->vec.normSq = normSq;

    uint8_t i;
    node_t *pItr = pMmf->pMin;

    for(i = 0; i < pMmf->cnt - 1; i++)
    {
        if(normSq < pItr->vec.normSq)
        {
            break;
        }

        pItr = pItr->pNext;
    }

    pItr->pPrev->pNext = pNode;
    pNode->pPrev       = pItr->pPrev;
    pItr->pPrev        = pNode;
    pNode->pNext       = pItr;

    if(i >= pMmf->cnt / 2)
    {
        pMmf->pMed = pMmf->pMed->pNext;
    }
    else if(i == 0)
    {
        pMmf->pMin = pNode;
    }

    pMmf->idx++;
    pMmf->idx %= WIN_LEN;

    return ESP_OK;
}

esp_err_t MMF_GetMedian(mmf_t *pMmf, int16_t smp[ELEM_COUNT])
{
    if(pMmf == NULL || smp == NULL)
    {
        ESP_LOGE(TAG, "MMF_GetMedian fail: INVALID ARGUMENT");
        return ESP_ERR_INVALID_ARG;
    }

    if(pMmf->cnt == 0)
    {
        ESP_LOGE(TAG, "MMF_GetMedian fail: INVALID COUNT");
        return ESP_ERR_NOT_ALLOWED;
    }

    uint8_t idx = pMmf->idx;

    for(uint8_t i = 0; i < pMmf->cnt; i++)
    {
        idx = idx > 0 ? idx - 1 : WIN_LEN - 1;

        node_t *pNode = &pMmf->win[idx];

        if(pNode->vec.normSq == pMmf->pMed->vec.normSq)
        {
            memcpy(smp, pNode->vec.elem, sizeof(pNode->vec.elem));
            break;
        }
    }

    return ESP_OK;
}

esp_err_t MMF_Test(void)
{
    mmf_t mmf;
    esp_err_t err = MMF_Init(&mmf);

    if(err)
    {
        return err;
    }

    typedef struct
    {
        uint32_t age;
        vec_t    vec;
    } test_t;

    test_t test[WIN_LEN] = {0};

    for(uint16_t i = 0; i < 2 * WIN_LEN; i++)
    {
        uint8_t idx = i % WIN_LEN;
        uint8_t cnt = i < WIN_LEN ? i + 1 : WIN_LEN;

        test[idx].age        = i;
        test[idx].vec.normSq = 0;

        for(elem_t el = X; el < ELEM_COUNT; el++)
        {
            int16_t min = -10;
            int16_t max =  10;

            test[idx].vec.elem[el] = min + (esp_random() % (max - min));
            test[idx].vec.normSq  += test[idx].vec.elem[el] * test[idx].vec.elem[el];
        }

        err = MMF_Update(&mmf, test[idx].vec.elem);

        if(err)
        {
            return err;
        }

        test_t sorted[WIN_LEN];
        memcpy(sorted, test, cnt * sizeof(test_t));

        // Insertion sort
        for(int32_t j = 1; j < cnt; j++)
        {
            test_t key = sorted[j];
            int32_t k = j - 1;

            while(k >= 0 && (sorted[k].vec.normSq > key.vec.normSq))
            {
                sorted[k + 1] = sorted[k];
                k--;
            }

            sorted[k + 1] = key;
        }

        uint32_t expectedNormSq = sorted[cnt / 2].vec.normSq;
        test_t *pExpected       = NULL;

        for(uint8_t j = 0; j < cnt; j++)
        {
            if(test[j].vec.normSq == expectedNormSq)
            {
                if(pExpected == NULL || test[j].age > pExpected->age)
                {
                    pExpected = &test[j];
                }
            }
        }

        int16_t actual[ELEM_COUNT];

        err = MMF_GetMedian(&mmf, actual);

        if(err)
        {
            return err;
        }

        if(actual[X] != pExpected->vec.elem[X] || actual[Y] != pExpected->vec.elem[Y] || actual[Z] != pExpected->vec.elem[Z] || expectedNormSq != pExpected->vec.normSq)
        {
            ESP_LOGE(TAG, "MMF test failed at iteration %u", i);
            ESP_LOGE(TAG, "Expected: norm = %lu, vector = (%d, %d, %d)", pExpected->vec.normSq, pExpected->vec.elem[X], pExpected->vec.elem[Y], pExpected->vec.elem[Z]);
            ESP_LOGE(TAG, "Obtained: norm = %lu, vector = (%d, %d, %d)", expectedNormSq, actual[X], actual[Y], actual[Z]);

            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err = MMF_Test();

    if(err)
    {
        return;
    }

    I2C_Init();
    LCD_Init();
    Accel_Init();

    mmf_t mmf;
    err = MMF_Init(&mmf);

    uint32_t cnt = 0;

    while(true)
    {
        cnt++;

        int16_t inAccel[ELEM_COUNT];
        Accel_ReadRaw(inAccel);

        err = MMF_Update(&mmf, inAccel);

        if(err)
        {
            return;
        }

        if(cnt % 25 == 0)
        {
            int16_t outAccel[ELEM_COUNT];

            err = MMF_GetMedian(&mmf, outAccel);

            if(err)
            {
                return;
            }

            if(outAccel[X] == 0 && outAccel[Y] == 0 && outAccel[Z] == 0)
            {
                ESP_LOGW(TAG, "Can not compute atan2f because all accelerometer values are zero.");
                esp_rom_delay_us(DELAY);

                continue;
            }

            uint32_t squared[] =
            {
                outAccel[X] * outAccel[X],
                outAccel[Y] * outAccel[Y],
                outAccel[Z] * outAccel[Z]
            };

            float angle[] =
            {
                RAD_TO_DEG(atan2f(outAccel[X], sqrtf(squared[Y] + squared[Z]))),
                RAD_TO_DEG(atan2f(outAccel[Y], sqrtf(squared[X] + squared[Z]))),
                RAD_TO_DEG(atan2f(outAccel[Z], sqrtf(squared[X] + squared[Y])))
            };

            char str[17];

            LCD_SetCursor(0, 0);
            snprintf(str, sizeof(str), "%s%.1f\xDF ", angle[ALPHA] < 0 ? "-" : " ", fabsf(angle[ALPHA]));
            LCD_Print(str);

            LCD_SetCursor(0, 10);
            snprintf(str, sizeof(str), "%s%.1f\xDF%s", angle[BETA] < 0 ? "-" : " ", fabsf(angle[BETA]), fabsf(angle[BETA]) < 10 ? " " : "");
            LCD_Print(str);

            LCD_SetCursor(1, 5);
            snprintf(str, sizeof(str), "%s%.1f\xDF ", angle[GAMMA] < 0 ? "-" : " ", fabsf(angle[GAMMA]));
            LCD_Print(str);
        }

        esp_rom_delay_us(DELAY);
    }
}
