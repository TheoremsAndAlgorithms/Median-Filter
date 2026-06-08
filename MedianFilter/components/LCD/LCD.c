#include "LCD.h"
#include "I2C.h"
#include "esp_rom_sys.h"

#define LCD_I2C_ADDR 0x27

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

static i2c_master_dev_handle_t _hDev = NULL;

static void WritePcf(uint8_t data)
{
    data |= LCD_BACKLIGHT;
    ESP_ERROR_CHECK(i2c_master_transmit(_hDev, &data, sizeof(uint8_t), 1000));
}

static void PulseEnable(uint8_t data)
{
    WritePcf(data | LCD_ENABLE);
    esp_rom_delay_us(100);

    WritePcf(data & ~LCD_ENABLE);
    esp_rom_delay_us(50);
}

static void WriteNibble(uint8_t nibble, uint8_t flags)
{
    uint8_t data = (nibble & 0xF0) | flags;

    WritePcf(data);
    PulseEnable(data);
}

static void Send(uint8_t value, uint8_t flags)
{
    WriteNibble(value & 0xF0, flags);
    WriteNibble((value << 4) & 0xF0, flags);
}

static void SendCmd(uint8_t cmd)
{
    Send(cmd, 0);
}

static void LcdData(uint8_t data)
{
    Send(data, LCD_RS);
}

void LCD_Init(void)
{
    _hDev = I2C_AddDev(LCD_I2C_ADDR);

    WriteNibble(0x30, 0);
    esp_rom_delay_us(5000);

    WriteNibble(0x30, 0);
    esp_rom_delay_us(150);

    WriteNibble(0x30, 0);
    esp_rom_delay_us(150);

    WriteNibble(0x20, 0); // 4-bit mode

    SendCmd(0x28); // 4-bit, 2 lines, 5x8 font
    SendCmd(0x08); // display off
    SendCmd(0x01); // clear display
    esp_rom_delay_us(2000);

    SendCmd(0x06); // entry mode: cursor moves right
    SendCmd(0x0C); // display on, cursor off, blink off
}

void LCD_Clear(void)
{
    SendCmd(0x01);
    esp_rom_delay_us(2000);
}

void LCD_SetCursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = { 0x00, 0x40 };
    SendCmd(0x80 | (col + row_offsets[row]));
}

void LCD_Print(const char *str)
{
    while(*str) 
    {
        LcdData((uint8_t)*str++);
    }
}
