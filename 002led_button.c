#include <string.h>
#include <stdio.h>
#include "stm32f103xx.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;
	
    GpioLed.pGPIOx = GPIOC;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_10MHZ;
    GpioLed.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_OUT_OD;
    GPIO_Init(&GpioLed);
	
		GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    GpioBtn.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_IN_PUPD;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_Init(&GpioBtn);

    while (1)
    {
			if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5) == LOW)
			{
				delay();
				GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
			}
			else
			{
				GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, LOW);
			}
    }
    return 0;
}
