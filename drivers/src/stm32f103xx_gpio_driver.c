/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Mar 24, 2025
 *      Author: PHAN ANH VU
 */

#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx.h"
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function Init Port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init_Input(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
	{
		//configure the input and the input type of the pin and pull up pull down settings
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
		pGPIOHandle->pGPIOx->CRL |= temp;

		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
		pGPIOHandle->pGPIOx->CRL |= temp;
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 8)
	{
		//configure the input and the input type of the pin and pull up pull down settings
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)); 
		pGPIOHandle->pGPIOx->CRH |= temp;

		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)); 
		pGPIOHandle->pGPIOx->CRH |= temp;
	}

	if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf == GPIO_MODE_IN_PUPD)
	{
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->ODR |= temp;
	}
}


void GPIO_Init_Output(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
	{
		//configure the output speed configure the output type
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
		pGPIOHandle->pGPIOx->CRL |= temp;

		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
		pGPIOHandle->pGPIOx->CRL |= temp;
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 8)
	{
		//configure the output speed configure the output type
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ));
		pGPIOHandle->pGPIOx->CRH |= temp;

		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)); 
		pGPIOHandle->pGPIOx->CRH |= temp;
	}
}

/*Peripheral side*/
void GPIO_Init_EXTI(GPIO_Handle_t * pGPIOHandle)
{
	// Step 0: Set pin as input with pull-up/pull-down
	uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t pos = pinNumber < 8 ? pinNumber : pinNumber % 8;
	volatile uint32_t *cr_reg = pinNumber < 8 ? &pGPIOHandle->pGPIOx->CRL : &pGPIOHandle->pGPIOx->CRH;

	// MODE = 00 (input), CNF = 10 (input pull-up/pull-down)
	*cr_reg &= ~(0xF << (pos * 4)); // Clear MODE + CNF
	*cr_reg |=  (0x2 << (pos * 4)); // CNF = 10, MODE = 00

	uint32_t temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->ODR |= temp;

	//this part will code later . ( interrupt mode)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf == GPIO_MODE_IT_FT )
	{
		//1. configure the FTSR
		EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf == GPIO_MODE_IT_RT )
	{
		//1 . configure the RTSR
		EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinModeConf == GPIO_MODE_IT_RFT )
	{
		//1. configure both FTSR and RTSR
		EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. configure the GPIO port selection in AFIO_EXTICR[]
	uint8_t chooseEXTIReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
	uint8_t chooseEXTILine = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	AFIO_PCLK_EN(); //enable the clock for AFIO	
	//select the port code for the EXTI line
	AFIO->EXTICR[chooseEXTIReg] &= ~(0xF << (chooseEXTILine * 4));
	AFIO->EXTICR[chooseEXTIReg] |= (portcode << (chooseEXTILine * 4)); 

	//3 . enable the exti interrupt delivery using IMR
	EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}



void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	 //enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	if ((temp == GPIO_MODE_OUTPUT) || (temp == GPIO_MODE_ALTFN))
	{
		GPIO_Init_Output(pGPIOHandle);
	}
	else if (temp == GPIO_MODE_INPUT)
	{
		GPIO_Init_Input(pGPIOHandle);
	}
		//configure the alt functionality
	else if (temp == GPIO_MODE_INPUT_ITR)
	{
		GPIO_Init_EXTI(pGPIOHandle);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;	
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Processor side
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else{
		if (IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	
	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*( NVIC_PR_BASEADDR + iprx ) |=  ((uint8_t)IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_IRQHandling(uint8_t PinNumber)//da viet trong file startup code
{
	//1. check which pin has triggered the interrupt
	//2. clear the exti pr register corresponding to the pin number
	 if(EXTI->PR & (1 << PinNumber))
   {
       /* Clear pin */
       EXTI->PR |= (1 << PinNumber);
   }
}


