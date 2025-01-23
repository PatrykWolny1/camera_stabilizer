#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"

#define SSD1306_SPI &hspi1          // Replace with your SPI instance
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Control pins
#define SSD1306_DC_GPIO GPIOB
#define SSD1306_DC_PIN GPIO_PIN_2
#define SSD1306_RESET_GPIO GPIOB
#define SSD1306_RESET_PIN GPIO_PIN_1

// Command/Data Mode
#define SSD1306_COMMAND 0
#define SSD1306_DATA 1

// Function prototypes
void SSD1306_Init(void);
void SSD1306_Write(uint8_t data, uint8_t cmd);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c, uint8_t color);
void SSD1306_DrawFloat(uint8_t x, uint8_t y, float number, uint8_t decimal_places, uint8_t color);
void SSD1306_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color);
void SSD1306_DrawNumber(uint8_t x, uint8_t y, int number, uint8_t color);

#endif
