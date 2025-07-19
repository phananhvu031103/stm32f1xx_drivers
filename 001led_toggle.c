#include <string.h>
#include <stdio.h>
#include "stm32f103xx.h"


void delay(void)
{
    for (volatile uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_10MHZ;
    GpioLed.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_OUT_OD;
    GPIO_Init(&GpioLed);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        delay();
    }
    return 0;
}
