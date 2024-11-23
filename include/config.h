#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>


// SPI Defines
#define SPI_PORT    spi0
#define PIN_CS      17
#define PIN_SCK     18
#define PIN_MOSI    19
#define PIN_DC      13  
#define PIN_RESET   14
#define PIN_BL      15

// I2C Defines
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// UART Defines
#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Screen Constants
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 300
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

#define SERIAL_CLK_DIV 1.f
#define UNIT_LSB 16

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Initialization sequence for the ST7789 LCD
extern const uint8_t st7789_init_seq[];

#endif // CONFIG_H
