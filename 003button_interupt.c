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
	
    memset(&GpioLed,0,sizeof(GpioLed));
	  memset(&GpioBtn,0,sizeof(GpioBtn));

    GpioLed.pGPIOx = GPIOC;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_10MHZ;
    GpioLed.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_OUT_PP;
    GPIO_Init(&GpioLed);
	
		GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_ITR;
    GpioBtn.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_IT_RT;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_Init(&GpioBtn);

		GPIO_WriteToOutputPin(GPIOC,GPIO_PIN_NO_13,GPIO_PIN_RESET);
    //IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    
		while (1);

}

void EXTI9_5_IRQHandler(void)
{
  delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
  GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
}