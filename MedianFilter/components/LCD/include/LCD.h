#pragma once

#include <stdint.h>

void LCD_Init(void);

void LCD_Clear(void);

void LCD_SetCursor(uint8_t col, uint8_t row);

void LCD_Print(const char *str);
