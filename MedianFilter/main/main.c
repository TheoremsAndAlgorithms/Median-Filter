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

#define WIN_LEN 24

#if WIN_LEN < 1
# error "Insufficient window length"
#endif

#define DELAY 500000 /* uS */

#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

const char *TAG = "main";

typedef enum
{
    X = 0,
    Y,
    Z,
    ELEM_COUNT
} elem_t;

typedef struct
{
    int16_t  elem[ELEM_COUNT];
    uint32_t normSq; // L2 norm squared
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
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t normSq = 0;
    for(elem_t el = X; el < ELEM_COUNT; el++)
    {
        normSq += smp[el] * smp[el];
    }

    node_t *pNode = &pMmf->win[pMmf->idx]; // pNode now points to the oldest node if the window is full, otherwise to the next non-populated node.

    if(pMmf->cnt < WIN_LEN) // If the window is not yet full, move the median to the previous node if the count is transitioning from even to odd
    {
        if(pMmf->cnt % 2 == 0)
        {
            pMmf->pMed = pMmf->pMed->pPrev;
        }
        pMmf->cnt++;
    }
    else
    {
        if(pNode == pMmf->pMed || pNode->vec.normSq > pMmf->pMed->vec.normSq) // If the oldest node is th median node or the oldest's node norm is greater than the median node's norm, set the previous node as the median.
        {
            pMmf->pMed = pMmf->pMed->pPrev;
        }
        else if(pNode == pMmf->pMin) // Or if the oldest node is the minimum node, set the next node as the minimum node...
        {
            pMmf->pMin = pNode->pNext;
        }

        // Detatch the oldest node from the list.
        pNode->pNext->pPrev = pNode->pPrev;
        pNode->pPrev->pNext = pNode->pNext;
    }

    uint8_t i;
    node_t *pItr = pMmf->pMin;

    // Loop through the linked list,
    // Break out of the loop when the the latest node's norm squared is smaller or equal than the pIt node's norm squared. 
    // Also, break out if cnt limit is reached. -1 is because of detaching the node from the list when the window is full and because counter is already incremented if the window is not full.

    for(i = 0; i < pMmf->cnt - 1; i++)
    {
        if(normSq < pItr->vec.normSq)
        {
            break;
        }
        pItr = pItr->pNext;
    }

    // Insert the new node between the left node with respect to the pItr and the pItr.
    // Remember, the pItr norm squared is bigger than the new nodes norm squared and the left node from the pItr has a norm squared smaller or equal then the norm squared of the latest node.
    // Safe for single node list as well, because in that case pItr will be the only node in the list and the new node will be inserted before or after it, which is the same thing because of circularity.
    pItr->pPrev->pNext = pNode;
    pNode->pPrev = pItr->pPrev;
    pItr->pPrev  = pNode;
    pNode->pNext = pItr;

    // Handle special cases. If the counter is zero, than set the newest node as the minimum node. Also if the counter is greater than or equal to trunc(SIZE / 2), increment the medians position. Note that trunc(SIZE / 2) is exectly the middle element when the number of elements is odd.
    if(i >= pMmf->cnt / 2)
    {
        pMmf->pMed = pMmf->pMed->pNext;
    }
    else if(i == 0)
    {
        pMmf->pMin = pNode;
    }

    // Update the window parameters
    memcpy(pNode->vec.elem, smp, sizeof(pNode->vec.elem));
    pNode->vec.normSq = normSq;

    pMmf->idx++;
    pMmf->idx %= WIN_LEN;

    return ESP_OK;
}

esp_err_t MMF_GetMedian(mmf_t *pMmf, int16_t smp[ELEM_COUNT])
{
    // Napomena, ako je samo vrednost u pitanju, a ne struktura, onda uradis return (pMed->val + pMed->pPrev->val) / 2; u GetMedian funkciji
    if(pMmf == NULL || smp == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if(pMmf->cnt == 0 || pMmf->pMed == NULL)
    {
        return ESP_FAIL;
    }

    uint8_t idx = pMmf->idx; // index of the oldest node because idx was previously incremented. First decrementation will get us the newest node

    for(uint8_t i = 0; i < pMmf->cnt; i++)
    {
        idx = idx > 0 ? idx - 1 : WIN_LEN - 1;

        node_t *pNode = &pMmf->win[idx];

        if(pNode->vec.normSq == pMmf->pMed->vec.normSq) // Find the youngest median vector
        {
            memcpy(smp, pNode->vec.elem, sizeof(pNode->vec.elem));
            break;
        }
    }

    return ESP_OK;
}

void app_main(void)
{
    I2C_Init();
    I2C_Scan();

    // LCD_Init();

    Accel_Init();

    mmf_t mmf;
    MMF_Init(&mmf);

    uint32_t cnt = 0;

    while(true)
    {
        cnt++;

        int16_t inAccel[ELEM_COUNT]  = {0};
        int16_t outAccel[ELEM_COUNT] = {0};

        Accel_ReadRaw(inAccel);

        MMF_Update(&mmf, inAccel);
        MMF_GetMedian(&mmf, outAccel);

        float xSq = outAccel[X] * outAccel[X];
        float ySq = outAccel[Y] * outAccel[Y];
        float zSq = outAccel[Z] * outAccel[Z];

        float alpha = RAD_TO_DEG(atan2f(outAccel[X], sqrtf(ySq + zSq)));
        float beta  = RAD_TO_DEG(atan2f(outAccel[Y], sqrtf(xSq + zSq)));
        float gamma = RAD_TO_DEG(atan2f(outAccel[Z], sqrtf(xSq + ySq)));

        ESP_LOGI(TAG, "cnt: %lu, x: %d, y: %d, z: %d", cnt, outAccel[X], outAccel[Y], outAccel[Z]);
        ESP_LOGI(TAG, "alpha: %.2f, beta: %.2f, gamma: %.2f\n", alpha, beta, gamma);

        esp_rom_delay_us(DELAY);
    }
}