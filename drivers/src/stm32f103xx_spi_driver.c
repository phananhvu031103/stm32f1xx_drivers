/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Apr 4, 2025
 *      Author: PHAN ANH VU
 */

#include "stm32f103xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
		}
        else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
        else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle){

    //peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    //first lets configure the SPI_CR1 register
    uint32_t tempreg = 0;
    //1. configure the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
    
    //2. configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

    // 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set DR is empty
		//TXE flag is set when the data register is empty and ready to accept new data
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len -= 2;
			pTxBuffer += 2;	
		}
		else
		{
			//8 bit DFF
			pSPIx->DR =  *((uint8_t *)pTxBuffer);
			Len--;
			pTxBuffer++;	
		}	
	}
	while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG)); // Wait while BUSY == 1
}



/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */

 void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
 {
	 while (Len > 0)
	 {
		 //1. wait until RXNE is set DR is not empty
		 //RXNE flag is set when the data register is not empty and ready to accept new data
		 while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET );
 
		 //2. check the DFF bit in CR1
		 if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		 {
			 //16 bit DFF
			 //1. load the data from the DR to the Rx buffer
			 *((uint16_t *)pTxBuffer) = pSPIx->DR;
			 Len -= 2;
			 pTxBuffer += 2;	
		 }
		 else
		 {
			 //8 bit DFF
			 *((uint8_t *)pTxBuffer) = pSPIx->DR;
			 Len--;
			 pTxBuffer++;	
		 }	
	 }
	 while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));// Wait while BSY == 1
 }
 
 

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*( NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		//1. Save the TX buffer address and length information in some global variable	
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		
		//2. Mark the SPI state as busy in communication so that 
		// no other communication is possible before the current communication is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;	

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);	
	}
	
	return state;
	//4. Data transmission will be handled in the ISR code	
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the TX buffer address and length information in some global variable	
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		
		//2. Mark the SPI state as busy in communication so that 
		// no other communication is possible before the current communication is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;	

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);	
	}
	
	return state;
	//4. Data transmission will be handled in the ISR code	
}

static void spi_txe_interrupt_handle(SPI_Handle_t * pSPIHandle)
{
	//check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		pSPIHandle->pTxBuffer += 2;	
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =  *((uint8_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;	
	}	

	if (!pSPIHandle->TxLen) //TxLen is Zero, so close the spi communication
	{
		//TxLen is Zero, so close the spi communication
		//TX is over
		//this prevents interupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);	

	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t * pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load the data from the DR to the Rx buffer
		*((uint16_t *)pSPIHandle->pTxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer += 2;	
	}
	else
	{
		//8 bit DFF
		*((uint8_t *)pSPIHandle->pTxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pTxBuffer++;	
	}
	
	if (!pSPIHandle->RxLen) //TxLen is Zero, so close the spi communication
	{
		//RxLen is Zero, so close the spi communication
		//RX is over
		//this prevents interupts from setting up of RXNE flag

		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);	
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t * pSPIHandle)
{
	uint8_t temp;	
	//clear ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR; //read the data register to clear the ovr flag
		temp = pSPIHandle->pSPIx->SR; //read the status register to clear the ovr flag
	}
	(void)temp; //to avoid compiler warning
	//inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);	
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)

{
	uint8_t temp1, temp2;
	//check for TXE flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);	

	if (temp1 && temp2)
	{
		//handle OVR error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}	
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}	

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}	

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR; //read the data register to clear the ovr flag
	temp = pSPIx->SR; //read the status register to clear the ovr flag
	(void)temp; //to avoid compiler warning
}	

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	//This is a weak implementation. The user application may override this function.
}

