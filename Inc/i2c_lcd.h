#include "stm32f1xx_hal.h"

#define LCD_CLEAR 0x01
#define LCD_CURSOR_HOME 0x02
#define LCD_CURSOR_HOME_PTR 0x80
#define LCD_CURSOR_NEW_LINE_OFFSET 0x40
#define LCD_CURSOR_HIDE 0x0c
#define LCD_CURSOR_UNDERLINE 0x0e
#define LCD_CURSOR_BLINK 0x0f

#define LCD_4BIT_MODE 0x28

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_error_handler (void); // lcd error handler 
