#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "hardware/pio.h"
#include "st7789_lcd.pio.h"
#include "config.h"

// Function declarations
void lcd_init(PIO pio, uint sm, const uint8_t *init_seq);
void lcd_set_dc_cs(bool dc, bool cs);
void st7789_start_pixels(PIO pio, uint sm);
void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count);

#endif // LCD_H