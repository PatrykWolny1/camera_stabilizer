#include "ssd1306.h"
#include "font.h"
#include <string.h> // For memset
#include <stdio.h>

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8]; // Display buffer

void SSD1306_Reset(void) {
    HAL_GPIO_WritePin(SSD1306_RESET_GPIO, SSD1306_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SSD1306_RESET_GPIO, SSD1306_RESET_PIN, GPIO_PIN_SET);
}

void SSD1306_Write(uint8_t data, uint8_t cmd) {
    HAL_GPIO_WritePin(SSD1306_DC_GPIO, SSD1306_DC_PIN, cmd == SSD1306_DATA ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_SPI_Transmit(SSD1306_SPI, &data, 1, HAL_MAX_DELAY);
}

void SSD1306_Init(void) {
	uint8_t init_cmd = 0xAA;
	if (HAL_SPI_Transmit(SSD1306_SPI, &init_cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("SPI transmission error\r\n");
	}

    // Reset the OLED (if the RESET pin is connected)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Replace GPIOB and PIN_2 with your RESET pin
    HAL_Delay(10);                                        // Wait 10ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    // Initialization commands for SSD1306
    SSD1306_Write(0xAE, SSD1306_COMMAND); // Display OFF
    SSD1306_Write(0x20, SSD1306_COMMAND); // Set Memory Addressing Mode
    SSD1306_Write(0x00, SSD1306_COMMAND); // Horizontal Addressing Mode
    SSD1306_Write(0xB0, SSD1306_COMMAND); // Set Page Start Address
    SSD1306_Write(0xC8, SSD1306_COMMAND); // COM Output Scan Direction
    SSD1306_Write(0x00, SSD1306_COMMAND); // Set Low Column Address
    SSD1306_Write(0x10, SSD1306_COMMAND); // Set High Column Address
    SSD1306_Write(0x40, SSD1306_COMMAND); // Set Start Line Address
    SSD1306_Write(0x81, SSD1306_COMMAND); // Set Contrast Control
    SSD1306_Write(0xFF, SSD1306_COMMAND); // Max contrast
    SSD1306_Write(0xA1, SSD1306_COMMAND); // Set Segment Re-map
    SSD1306_Write(0xA6, SSD1306_COMMAND); // Set Normal Display
    SSD1306_Write(0xA8, SSD1306_COMMAND); // Set Multiplex Ratio
    SSD1306_Write(0x3F, SSD1306_COMMAND); // 1/64 Duty
    SSD1306_Write(0xA4, SSD1306_COMMAND); // Disable Entire Display ON
    SSD1306_Write(0xD3, SSD1306_COMMAND); // Set Display Offset
    SSD1306_Write(0x00, SSD1306_COMMAND); // No offset
    SSD1306_Write(0xD5, SSD1306_COMMAND); // Set Display Clock Divide Ratio
    SSD1306_Write(0x80, SSD1306_COMMAND); // Default clock ratio
    SSD1306_Write(0xD9, SSD1306_COMMAND); // Set Pre-charge Period
    SSD1306_Write(0xF1, SSD1306_COMMAND); // Default pre-charge
    SSD1306_Write(0xDA, SSD1306_COMMAND); // Set COM Pins Hardware Configuration
    SSD1306_Write(0x12, SSD1306_COMMAND); // Alternative COM pins
    SSD1306_Write(0xDB, SSD1306_COMMAND); // Set VCOMH Deselect Level
    SSD1306_Write(0x40, SSD1306_COMMAND); // Default VCOMH
    SSD1306_Write(0x8D, SSD1306_COMMAND); // Enable charge pump
    SSD1306_Write(0x14, SSD1306_COMMAND);
    SSD1306_Write(0xAF, SSD1306_COMMAND); // Display ON

    SSD1306_Clear();
    SSD1306_UpdateScreen();

    HAL_Delay(10);                                        // Wait 10ms
}

void SSD1306_Clear(void) {
    memset(SSD1306_Buffer, 0x00, sizeof(SSD1306_Buffer));
}

void SSD1306_ClearRegion(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            SSD1306_DrawPixel(x + i, y + j, 0); // Draw black pixels
        }
    }
}

void SSD1306_UpdateScreen(void) {
    for (uint8_t page = 0; page < 8; page++) {
        SSD1306_Write(0xB0 + page, SSD1306_COMMAND);
        SSD1306_Write(0x00, SSD1306_COMMAND);
        SSD1306_Write(0x10, SSD1306_COMMAND);
        HAL_GPIO_WritePin(SSD1306_DC_GPIO, SSD1306_DC_PIN, GPIO_PIN_SET);
        HAL_SPI_Transmit(SSD1306_SPI, &SSD1306_Buffer[page * SSD1306_WIDTH], SSD1306_WIDTH, HAL_MAX_DELAY);
    }
}

void SSD1306_UpdateScreen_Vertical(void) {
    for (uint8_t col = 0; col < SSD1306_WIDTH; col++) {
        // Set the column address
        SSD1306_Write(0x00 + (col & 0x0F), SSD1306_COMMAND);   // Lower nibble of column
        SSD1306_Write(0x10 + (col >> 4), SSD1306_COMMAND);     // Higher nibble of column

        // Write data for all 8 pages of this column
        for (uint8_t page = 0; page < 8; page++) {
            SSD1306_Write(0xB0 + page, SSD1306_COMMAND);       // Set page address
            HAL_GPIO_WritePin(SSD1306_DC_GPIO, SSD1306_DC_PIN, GPIO_PIN_SET); // Set DC high for data
            HAL_SPI_Transmit(SSD1306_SPI, &SSD1306_Buffer[page * SSD1306_WIDTH + col], 1, HAL_MAX_DELAY);
        }
    }
}


void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (color) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_DrawChar(uint8_t x, uint8_t y, char c, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (c < 32 || c > 126) c = '?'; // Replace unsupported characters with '?'

    for (uint8_t i = 0; i < 5; i++) { // Each character is 5 pixels wide
        uint8_t line = Font5x7[(c - 32) * 5 + i];
        for (uint8_t j = 0; j < 8; j++) { // Each character is 7 pixels tall
            if (line & (1 << j)) {
                SSD1306_DrawPixel(x + i, y + j, color);
            } else {
                SSD1306_DrawPixel(x + i, y + j, !color);
            }
        }
    }
}

void SSD1306_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color) {
    while (*str) {
        SSD1306_DrawChar(x, y, *str, color);
        x += 6; // Move to the next character (5 pixels + 1 space)
        str++;
        if (x + 5 >= SSD1306_WIDTH) break; // Stop if the string goes off-screen
    }
}

void SSD1306_DrawNumber(uint8_t x, uint8_t y, int number, uint8_t color) {
    char buffer[12]; // Buffer to hold the number as a string
    snprintf(buffer, sizeof(buffer), "%d", number); // Convert number to string
    SSD1306_DrawString(x, y, buffer, color);
}

void SSD1306_DrawFloat(uint8_t x, uint8_t y, float number, uint8_t decimal_places, uint8_t color) {
    char buffer[20]; // Buffer to hold the converted float as a string
    snprintf(buffer, sizeof(buffer), "%.*f", decimal_places, number); // Convert float to string
    SSD1306_DrawString(x, y, buffer, color); // Use the existing string drawing function
}


