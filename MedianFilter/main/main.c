#include "I2C.h"
#include "Accel.h"

#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define WIN_LEN 20

#define DELAY 20000 /* us */

const char *TAG = "main";

typedef struct node node_t;

struct node
{
    float   val;

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

esp_err_t MMF_Update(mmf_t *pMmf, float val)
{
    if(pMmf == NULL)
    {
        ESP_LOGE(TAG, "MMF_Update fail: INVALID ARGUMENT");
        return ESP_ERR_INVALID_ARG;
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
        if(pNode == pMmf->pMed || pNode->val > pMmf->pMed->val)
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

    pNode->val = val;

    uint8_t i;
    node_t *pItr = pMmf->pMin;

    for(i = 0; i < pMmf->cnt - 1; i++)
    {
        if(val < pItr->val)
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

esp_err_t MMF_GetMedian(mmf_t *pMmf, float *pVal)
{
    if(pMmf == NULL || pVal == NULL)
    {
        ESP_LOGE(TAG, "MMF_GetMedian fail: INVALID ARGUMENT");
        return ESP_ERR_INVALID_ARG;
    }

    if(pMmf->cnt == 0)
    {
        ESP_LOGE(TAG, "MMF_GetMedian fail: INVALID COUNT");
        return ESP_ERR_NOT_ALLOWED;
    }

    *pVal = pMmf->pMed->val;

    if(pMmf->cnt % 2 == 0)
    {
        *pVal += pMmf->pMed->pPrev->val;
        *pVal /= 2.0f;
    }

    return ESP_OK;
}

void app_main(void)
{
    I2C_Init();
    Accel_Init();

    mmf_t mmf;
    esp_err_t err = MMF_Init(&mmf);

    if(err)
    {
        return;
    }

    uint8_t cnt = 0;

    struct main
    {
        float win[WIN_LEN];

        uint8_t idx;
        uint8_t cnt;

        float med;
    } test = {0};

    while(cnt < 3 * WIN_LEN)
    {
        cnt++;

        float val = Accel_GetAcc_g();

        err = MMF_Update(&mmf, val);

        if(err)
        {
            return;
        }

        float med;

        err = MMF_GetMedian(&mmf, &med);

        if(err)
        {
            return;
        }

        test.win[test.idx] = val;

        test.idx++;
        test.idx %= WIN_LEN;

        test.cnt++;

        if(test.cnt > WIN_LEN)
        {
            test.cnt = WIN_LEN;
        }

        // Insertion sort
        float sorted[WIN_LEN];
        memcpy(sorted, test.win, test.cnt * sizeof(float));

        for(int i = 1; i < test.cnt; i++)
        {
            float key = sorted[i];
            int j = i - 1;

            while(j >= 0 && sorted[j] > key)
            {
                sorted[j + 1] = sorted[j];
                j = j - 1;
            }

            sorted[j + 1] = key;
        }

        test.med = sorted[test.cnt / 2];
        if(test.cnt % 2 == 0)
        {
            test.med += sorted[test.cnt / 2 - 1];
            test.med /= 2.0f;
        }

        ESP_LOGI(TAG, "%u. input = %.3f, (expected, actual) = (%.3f, %.3f)\n", test.cnt, val, test.med, med);

        if(med != test.med)
        {
            ESP_LOGE(TAG, "Incorrect median");
            break;
        }

        esp_rom_delay_us(DELAY);
    }
}
