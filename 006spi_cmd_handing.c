#include <string.h>
#include <stdio.h>
#include "stm32f103xx.h"

//PA7 - MOSI
//PA6 - MISO
//PA5 - SCK
//PA4 - NSS
//ALT FUNCTION MODE

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led
#define LED_PIN  9


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

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
    if (ackbyte == 0xF5){
        return 1; //ACK
    }else
        return 0; //NACK
}

int main()
{
	const char* user_data = "Hello World";
    uint8_t dummy_write = 0xff;
    uint8_t dummy_read;

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
		while (!GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7)); 

        //to avoid button debouncing
		delay();

		//This function is used to enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

        uint8_t commandcode = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];

        //1. CMD_LED_CTRL <pin no 1> <value 1>
        SPI_SendData(SPI1, &commandcode, 1);
       
        //do dummy read to clear the RXNE flag  
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        //send some dummy bits (1 byte) to fetch the response from slave
        SPI_SendData(SPI1, &dummy_write, 1);

        //read the ack byte from slave  
        SPI_ReceiveData(SPI1, &ackbyte, 1);
		
        if (SPI_VerifyResponse(ackbyte)){
            //send arguments to slave
            args[0] = LED_PIN; //pin number
            args[1] = LED_ON; //value to be set
            SPI_SendData(SPI1, args, 2);
        }
        //end of COMMAND_LED_CTRL

        //2. CMD_SENOSR_READ   <analog pin number(1) >
        //wait for button press
		while (!GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7)); 

        //to avoid button debouncing
		delay();

        //This function is used to enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

        commandcode = COMMAND_SENSOR_READ;

        //send the command code to the slave CMD_SENSOR_READ <pin no 1>
        SPI_SendData(SPI1, &commandcode, 1);
        
        //do dummy read to clear the RXNE flag  
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        //send some dummy bits (1 byte) to fetch the response from slave
        SPI_SendData(SPI1, &dummy_write, 1);

        //read the ack byte from slave  
        SPI_ReceiveData(SPI1, &ackbyte, 1);
        
        if (SPI_VerifyResponse(ackbyte))
        {
            args[0] = ANALOG_PIN0; 

           //send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

            //insert some delay to allow slave to process the request and send the response 
            delay();

            //send some dummy bits (1 byte) to fetch the response from slave
            SPI_SendData(SPI1, &dummy_write, 1);
            uint8_t analog_read;
            SPI_ReceiveData(SPI1, &analog_read, 1);
            //printf("COMMAND_SENSOR_READ %d\n",analog_read);
        }




        
		//Disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);
	}
	return 0;
}



