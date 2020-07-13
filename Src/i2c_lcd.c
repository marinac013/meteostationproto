/** Put this in the src folder **/

#include "i2c_lcd.h"

#define SLAVE_ADDRESS_LCD 0x3F<<1 // change this according to ur setup

extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	
	do
  {
    if(HAL_I2C_Master_Transmit_IT (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4) != HAL_OK)
    {
      lcd_error_handler();
    }
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
  }
  while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	HAL_Delay(1);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	
	do
	{
		if(HAL_I2C_Master_Transmit_IT (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4) != HAL_OK)
    {
      lcd_error_handler();
    }
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	}
  while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	HAL_Delay(1);
}

void lcd_init (void)
{
	lcd_send_cmd (LCD_CURSOR_HOME);
	lcd_send_cmd (LCD_4BIT_MODE);
	lcd_send_cmd (LCD_CURSOR_HIDE);
	lcd_send_cmd (LCD_CURSOR_HOME_PTR);
	lcd_send_cmd(LCD_CLEAR);
	HAL_Delay(1);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_error_handler (void)
{
	return;
}

