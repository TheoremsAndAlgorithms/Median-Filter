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

#define WIN_LEN 32

#if WIN_LEN < 1
# error "Insufficient window length"
#endif

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

    node_t *pNode = &pMmf->win[pMmf->idx]; // pNode now points to the oldest node if the window is full, otherwise it points to the next non-populated node.

    // If the window is not full, move the median to the previous node if the counter is transitioning from even to odd.
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
        /* 
            Handle special cases.
            If the oldest node is the median node or the oldest's node norm is greater than the median node's norm, set the previous node as the median.
            Otherwise, if the oldest node is the minimum node, set the next node as the minimum node.
        */
        if(pNode == pMmf->pMed || pNode->vec.normSq > pMmf->pMed->vec.normSq)
        {
            pMmf->pMed = pMmf->pMed->pPrev;
        }
        else if(pNode == pMmf->pMin)
        {
            pMmf->pMin = pNode->pNext;
        }

        // Detatch the oldest node from the list.
        pNode->pNext->pPrev = pNode->pPrev;
        pNode->pPrev->pNext = pNode->pNext;
    }

    // Populate the node with the new vector and its squared norm. pNode is now the newest node in the list.
    memcpy(pNode->vec.elem, smp, sizeof(pNode->vec.elem));
    pNode->vec.normSq = normSq;

    /*
        Loop through the list.
        Break out of the loop when the the newest node's squared norm is smaller than the iterator's squared norm. 
        Also, break out if the counter limit is reached. 
        The minus one in (i < pMmf->cnt - 1) is because of detaching the node from the list when the window is full and because the counter is already incremented if the window is not full.
    */
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

    /*
        Insert the new node between the left node with respect to the pItr and the pItr
        Note, the pItr squared norm is bigger than the newest nodes squared norm and the left node from the pItr has a squared norm smaller or equal to the squared norm of the newest node.
        This is safe for a single node list as well, because in that case pItr will be the only node in the list and the new node will be inserted before or after it, which is the same thing because of circularity.
    */
    pItr->pPrev->pNext = pNode;
    pNode->pPrev       = pItr->pPrev;
    pItr->pPrev        = pNode;
    pNode->pNext       = pItr;

    /*
        Handle special cases.
        If the i-counter is greater than or equal to (pMmf->cnt / 2), increment the median to its correct position.
        The median is right alligned if (pMmf->cnt / 2) is even.
        Note that (pMmf->cnt / 2) is alwaus the median's position in the list, regardless if pMmf->cnt is even or odd.
    */
    if(i >= pMmf->cnt / 2)
    {
        pMmf->pMed = pMmf->pMed->pNext;
    }
    else if(i == 0)
    {
        pMmf->pMin = pNode;
    }

    // Update the window parameters
    pMmf->idx++;
    pMmf->idx %= WIN_LEN;

    return ESP_OK;
}

esp_err_t MMF_GetMedian(mmf_t *pMmf, int16_t smp[ELEM_COUNT])
{
    if(pMmf == NULL || smp == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if(pMmf->cnt == 0)
    {
        return ESP_ERR_NOT_ALLOWED;
    }

    // idx represents the index of the oldest node and decrementing will get us the newest node.
    uint8_t idx = pMmf->idx;

    for(uint8_t i = 0; i < pMmf->cnt; i++)
    {
        idx = idx > 0 ? idx - 1 : WIN_LEN - 1;

        node_t *pNode = &pMmf->win[idx];

        if(pNode->vec.normSq == pMmf->pMed->vec.normSq) // Find the youngest vector with the squared norm equal to the median's squared norm.
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
    LCD_Init();
    Accel_Init();

    mmf_t mmf;
    MMF_Init(&mmf);

    uint32_t cnt = 0;

    while(true)
    {
        cnt++;

        int16_t inAccel[ELEM_COUNT];
        Accel_ReadRaw(inAccel);
        
        MMF_Update(&mmf, inAccel);

        if(cnt % 25 == 0) // Print the angles on the LCD display
        {
            int16_t outAccel[ELEM_COUNT];
            MMF_GetMedian(&mmf, outAccel);

            uint32_t squared[] =
            {
                outAccel[X] * outAccel[X],
                outAccel[Y] * outAccel[Y],
                outAccel[Z] * outAccel[Z]
            };

            if(outAccel[X] == 0 && outAccel[Y] == 0 && outAccel[Z] == 0)
            {
                ESP_LOGE(TAG, "Cannot compute atan2f because all accelerometer values are zero.");
                esp_rom_delay_us(DELAY);
                continue;
            }

            float angle[] =                                                      // Every angle is in range of [-90, 90] degrees
            {
                RAD_TO_DEG(atan2f(outAccel[X], sqrtf(squared[Y] + squared[Z]))), // alpha, angle between the x axis and the horizontal plane
                RAD_TO_DEG(atan2f(outAccel[Y], sqrtf(squared[X] + squared[Z]))), // beta,  angle between the y axis and the horizontal plane
                RAD_TO_DEG(atan2f(outAccel[Z], sqrtf(squared[X] + squared[Y])))  // gamma, angle between the z axis and the horizontal plane 
            };

            char str[17]; // 16 characters + null terminator

            LCD_SetCursor(0, 0);
            snprintf(str, sizeof(str), "%s%.1f\xDF ", angle[ALPHA] < 0 ? "-" : " ", fabsf(angle[ALPHA]));
            LCD_Print(str);

            LCD_SetCursor(0, 10);
            snprintf(str, sizeof(str), "%s%.1f\xDF%s", angle[BETA] < 0 ? "-" : " ", fabsf(angle[BETA]), fabsf(angle[BETA]) < 10 ? " " : "");
            LCD_Print(str);

            LCD_SetCursor(1, 5);
            snprintf(str, sizeof(str), "%s%.1f\xDF ", angle[GAMMA] < 0 ? "-" : " ", fabsf(angle[GAMMA]));
            LCD_Print(str);

            ESP_LOGI(TAG, "cnt: %lu, x: %d, y: %d, z: %d, window is %s", cnt, outAccel[X], outAccel[Y], outAccel[Z], cnt < WIN_LEN ? "not full" : "full");
            ESP_LOGI(TAG, "alpha: %.2f, beta: %.2f, gamma: %.2f\n", angle[ALPHA], angle[BETA], angle[GAMMA]);
        }

        esp_rom_delay_us(DELAY);
    }
}
