/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: Apr 13, 2025
 *      Author: PHAN ANH VU
 */

#ifndef INC_STM32F103XX_I2C_DRIVER_H_
#define INC_STM32F103XX_I2C_DRIVER_H_

#include "stm32f103xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint32_t I2C_DeviceAddress;
    uint32_t I2C_ACKControl;
    uint32_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
    I2C_RegDef_t    *pI2Cx;
    I2C_Config_t    I2C_Config;
    uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;


/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM            100000 // Standard mode speed 100kHz
#define I2C_SCL_SPEED_FM4K          400000 // Fast mode speed 200kHz
#define I2C_SCL_SPEED_FM2K          200000 // Fast mode speed 2MHz

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE              1
#define I2C_ACK_DISABLE             0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_CYCLE_2         0
#define I2C_FM_DUTY_CYCLE_16_9      1

/*
 * Possible I2C application events
 */
#define I2C_FLAG_SB                 (1 << I2C_SR1_SB)	    // Start bit flag
#define I2C_FLAG_ADDR               (1 << I2C_SR1_ADDR)	    // Address bit flag
#define I2C_FLAG_BTF                (1 << I2C_SR1_BTF)	    // Buffer transfer flag
#define I2C_FLAG_STOPF              (1 << I2C_SR1_STOPF)	    // Stop bit flag
#define I2C_FLAG_RXNE               (1 << I2C_SR1_RXNE)	    // Receive not empty flag
#define I2C_FLAG_TXE                (1 << I2C_SR1_TXE)	    // Transmit empty flag
#define I2C_FLAG_BERR               (1 << I2C_SR1_BERR)	    // Bus error flag
#define I2C_FLAG_ARLO               (1 << I2C_SR1_ARLO)	    // Arbitration lost flag
#define I2C_FLAG_AF                 (1 << I2C_SR1_AF)	    // Acknowledge failure flag
#define I2C_FLAG_OVR                (1 << I2C_SR1_OVR)	    // Overrun/Underrun flag
#define I2C_FLAG_TIMEOUT            (1 << I2C_SR1_TIMEOUT)  // Timeout flag
#define I2C_FLAG_ADD10              (1 << I2C_SR1_ADD10)	// 10-bit address flag

#define I2C_DISABLE_SR  	        RESET
#define I2C_ENABLE_SR   	        SET

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	    0
#define I2C_EV_RX_CMPLT  	 	    1
#define I2C_EV_STOP       		    2
#define I2C_ERROR_BERR 	 		    3
#define I2C_ERROR_ARLO  		    4
#define I2C_ERROR_AF    		    5
#define I2C_ERROR_OVR   		    6
#define I2C_ERROR_TIMEOUT 		    7
#define I2C_EV_DATA_REQ             8
#define I2C_EV_DATA_RCV             9


/******************************************************************************************
 *                              APIs supported by this driver
 *       For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive    
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive in interrupt mode  
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);   
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);    

/*
 * IRQ Configuration and ISR handling      
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);  

/*
 * Other Peripheral Control APIs        
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);    
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */
