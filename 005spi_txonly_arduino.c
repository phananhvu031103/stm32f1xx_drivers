#include <string.h>
#include <stdio.h>
#include "stm32f103xx.h"

//PA7 - MOSI
//PA6 - MISO
//PA5 - SCK
//PA4 - NSS
//ALT FUNCTION MODE
void delay(void)
{
    for (uint32_t i = 0; i < 500000; i++);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_IN_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&GpioBtn);
}

void SPI1_GPIOInits(){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinModeConf = GPIO_MODE_AF_OUT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_50MHZ;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	// //MISO
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	// GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
}

void SPI1_Inits(){
	SPI_Handle_t SPI1Handle;	
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI1Handle);
}

int main()
{
	const char* user_data = "Hello World";

	//Config button
	GPIO_ButtonInit();
	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	//This function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();

	SPI_SSOEConfig(SPI1, ENABLE);
	
	while(1){
		// SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));
		// for (volatile int i = 0; i < 100000; i++);

		//wait for button press
		//while (!GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7)); 
		//delay();

		//This function is used to enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		//first send length information	
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI1, &data_len, 1);
		
		//delay();
		//This function to send data to SPI1 peripheral
		SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

		//confirm SPI is not busy	
		while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG)); 

		//Disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);
	}
	return 0;
}



