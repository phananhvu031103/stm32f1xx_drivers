/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Mar 24, 2025
 *      Author: PHAN ANH VU
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinModeConf;		/*!< possible values from @GPIO_PIN_MODES_CONF >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< only for INPUT MODE*/
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 	/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible input modes
 */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_INPUT_ITR			2
#define GPIO_MODE_ALTFN				3		
/*
 * @GPIO_PIN_MODES_CONF
 * GPIO pin possible input configuration modes
 */
#define GPIO_MODE_IN_ANALOG			0
#define GPIO_MODE_IN_FLOAT 			1
#define GPIO_MODE_IN_PUPD 			2

/*
 * @GPIO_PIN_MODES_CONF
 * GPIO pin possible output modes  GPIO_PIN_MODES_CONF 
 */
#define GPIO_MODE_OUT_PP			0 	//Output Push-pull
#define GPIO_MODE_OUT_OD			1 	//Output Open-drain
#define GPIO_MODE_AF_OUT_PP			2	//Alternate function push-pull	
#define GPIO_MODE_AF_OUT_OD			3	//Alternate function open-drain
#define GPIO_MODE_IT_FT     		5
#define GPIO_MODE_IT_RT     		6
#define GPIO_MODE_IT_RFT    		7
/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_OUT_SPEED_10MHZ		1
#define GPIO_OUT_SPEED_2MHZ			2
#define GPIO_OUT_SPEED_50MHZ		3

/*
 * GPIO pin pull up AND pull down configuration macros
 * @GPIO_PinPuPdControl
 */
#define GPIO_PIN_PU					0
#define GPIO_PIN_PD					1
#define GPIO_PIN_NO_PUPD			2	

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
