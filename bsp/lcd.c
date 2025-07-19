#include "lcd.h"    

static void write_4_bits(uint8_t value);
static void lcd_enable(void);


void lcd_send_command(uint8_t cmd)
{
    /*RS = 0*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(cmd >> 4);
    write_4_bits(cmd & 0x0F);

}

void lcd_print_char(uint8_t data)
{
    /*RS = 1*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(data >> 4); /*higher nibble*/
    write_4_bits(data & 0x0F); /*lower nibble*/

}


void lcd_print_string(char* str)
{
    while(*str){
        lcd_print_char(*str++);
    }
}

void lcd_init()
{
    //1. configure the GPIO pins for LCD 
    GPIO_Handle_t lcd_signal;
    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_OUT_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_2MHZ;
    lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal); 

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal); 

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET); //RS = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET); //RW = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET); //EN = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET); //D4 = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET); //D5 = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET); //D6 = 0
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET); //D7 = 0

    //2. do the lcd initialization 
    mdelay(40); //wait for 40ms

    /*RS = 0 for LCD command*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /*R/nW = 0 Writing to LCD*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET); 

    write_4_bits(0x03);

    mdelay(5);

    write_4_bits(0x03);

    udelay(150);

    write_4_bits(0x03);
    write_4_bits(0x02);
    
    //function set command
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);
    //display on, cursor on, blink on
    lcd_send_command(LCD_CMD_DON_CURON);
    //display clear command
    lcd_display_clear(); //clear display
    //entry mode set command
    lcd_send_command(LCD_CMD_INCADD); 

}

void lcd_display_clear(void)
{
	//Display clear
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	/*
	 * check page number 24 of datasheet.
	 * display clear command execution wait time is around 2ms
	 */

	mdelay(2);
}

/*Cursor returns to home position */
void lcd_display_return_home(void)
{

	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * check page number 24 of datasheet.
	 * return home command execution wait time is around 2ms
	 */
	mdelay(2);
}

/**
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--;
    switch(row){
        case 1:
            lcd_send_command(0x80 | column); //0x80 is the command for first line
            break;
        case 2:
            lcd_send_command(0xC0 | column); //0xC0 is the command for second line
            break;
        default:
            break;
    }
}



static void write_4_bits(uint8_t value){
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, (value >> 0) & 0x01);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, (value >> 1) & 0x01);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, (value >> 2) & 0x01);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, (value >> 3) & 0x01);
    lcd_enable();   
}

static void lcd_enable()
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET); //EN = 1
    udelay(10); 
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET); //EN = 0
    udelay(100); 
}

void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

void udelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}
