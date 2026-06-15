#include "I2C.h"
#include "Accel.h"

#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define WIN_LEN 9

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
        pMmf->cnt++;
    }
    else
    {
        if(pNode == pMmf->pMin)
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

    if(i == 0)
    {
        pMmf->pMin = pNode;
    }

    pMmf->idx++;
    pMmf->idx %= WIN_LEN;

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

    while(cnt < 2 * WIN_LEN)
    {
        cnt++;

        float val = Accel_GetAcc_g();

        err = MMF_Update(&mmf, val);

        if(err)
        {
            return;
        }

        node_t *pNode = mmf.pMin;

        uint8_t len = cnt < WIN_LEN ? cnt : WIN_LEN;

        for(uint8_t i = 0; i < len; i++)
        {
            if(i > 0) // Make sure there are at least two populated nodes.
            {
                if(pNode->pPrev->val > pNode->val)
                {
                    ESP_LOGE(TAG, "Incorrect node order");
                    return;
                }
            }

            printf("%.3f ", pNode->val);

            pNode = pNode->pNext;
        }
        printf("\n\n");

        esp_rom_delay_us(DELAY);
    }
}
