#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "config.h"

// Function declarations
void spi_init_internal(PIO pio, uint sm);
void i2c_init_internal();
void uart_init_internal();
int wifi_init_internal();

#endif // PERIPHERALS_H
