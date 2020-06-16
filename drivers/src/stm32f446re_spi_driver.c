/*
 * stm32f446re_spi_driver.c
 *
 *  Created on: 26-Dec-2019
 *      Author: atharva
 */

#include "stm32f446re_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{if(EnorDi == ENABLE)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_EN();
	}else if (pSPIx == SPI2)
	{
		SPI2_PCLK_EN();
	}else if (pSPIx == SPI3)
	{
		SPI3_PCLK_EN();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_PCLK_EN();
	}
}
else
{ if(pSPIx==SPI1)
{
	SPI1_PCLK_DI();
}else if(pSPIx==SPI2)
{
	SPI2_PCLK_DI();
}else if(pSPIx==SPI3)
{
	SPI3_PCLK_DI();
}else if(pSPIx==SPI4)
{
	SPI4_PCLK_DI();
}

}

}




//INIT AND DEINIT

 void SPI_Init(SPI_Handle_t *pSPIHandle)
 {
	 uint32_t temp=0;
	 SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);
	 temp|= pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;
	 if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	 {temp&=~(1<<SPI_CR1_BIDIMODE);

	 }else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	 {
		 temp|=(1<<SPI_CR1_BIDIMODE);
	 }else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RX)
	 {temp&=~(1<<15);
	 temp|=(1<<10);
		 }
	 temp|=pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;
	 temp|=pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;
	 temp|=pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;
	 temp|=pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;
	 temp|=pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM;
	 pSPIHandle->pSPIx->CR1=temp;
 }
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
{

}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{if(pSPIx->SR & FlagName)
{
	return FLAG_SET;
}
	return FLAG_RESET;
}
//DATA SEND AND RECEIVE

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTXBuffer,uint32_t len)
{
	while(len>0)
{
	while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);
	//check the dff bit
	if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16 BIT
		pSPIx->DR=*((uint16_t*)pTXBuffer);
		len--;
		len--;
		(uint16_t*)pTXBuffer++;//2 bytes of data is sent,so two decrements
	}else
	{//8bit
		pSPIx->DR=(*pTXBuffer);
				len--;
				pTXBuffer++;
				 }
}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRXBuffer,uint32_t len)
{
	while(len > 0)
			{
				//1. wait until RXNE is set
				while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

				//2. check the DFF bit in CR1
				if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
				{
					//16 bit DFF
					//1. load the data from DR to Rxbuffer address
					 *((uint16_t*)pRXBuffer) = pSPIx->DR ;
					len--;
					len--;
					(uint16_t*)pRXBuffer++;
				}else
				{
					//8 bit DFF
					*(pRXBuffer) = pSPIx->DR ;
					len--;
					pRXBuffer++;
				}
			}
}

//irq config

void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(NVIC_PR_BASEADDR+(iprx*4))|=(IRQPriority<<(8*iprx_section));

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1|=1<<SPI_CR1_SPE;
	}else
	{
		pSPIx->CR1&=~(1<<SPI_CR1_SPE);
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			pSPIx->CR1|=1<<SPI_CR1_SSI;
		}else
		{
			pSPIx->CR1&=~(1<<SPI_CR1_SSI);
		}
}
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			pSPIx->CR2|=1<<SPI_CR2_SSOE;
		}else
		{
			pSPIx->CR2&=~(1<<SPI_CR2_SSOE);
		}
}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTXBuffer,uint32_t len)
{uint8_t state=pSPIHandle->TxState;
if(state!=SPI_BUSY_IN_RX)
{pSPIHandle->pTXBuffer=pTXBuffer;
pSPIHandle->TxLen=len;
pSPIHandle->TxState=SPI_BUSY_IN_TX;
//ENABLE TXEIE CONTROL BIT TO GET INTERRUPT WHEN TXE FLAG IS SET IN SR
pSPIHandle->pSPIx->CR2|= (1<<SPI_CR2_TXEIE);

}
return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRXBuffer,uint32_t len)
{
	uint8_t state=pSPIHandle->RxState;
	if(state!=SPI_BUSY_IN_TX)
	{pSPIHandle->pRXBuffer=pRXBuffer;
	pSPIHandle->RxLen=len;
	pSPIHandle->RxState=SPI_BUSY_IN_RX;
	//ENABLE TXEIE CONTROL BIT TO GET INTERRUPT WHEN TXE FLAG IS SET IN SR
	pSPIHandle->pSPIx->CR2|= (1<<SPI_CR2_RXNEIE);

	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t*pSPIHandle)
{//LETS CHECK FOR TXE
uint8_t temp1,temp2;
temp1=pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);
if(temp1 && temp2)
{
	//handle txe
	spi_txe_interrupt_handle(pSPIHandle);

}
//check for rxne
temp1=pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);
if(temp1 && temp2)
{
	//handle txe
	spi_rxne_interrupt_handle(pSPIHandle);

}
//check for overrun error
temp1=pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);
if(temp1 && temp2)
{
	//handle txe
	spi_ovr_err_interrupt_handle(pSPIHandle);

}

}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 BIT
			pSPIHandle->pSPIx->DR=*((uint16_t*)pSPIHandle->pTXBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTXBuffer++;//2 bytes of data is sent,so two decrements
		}else
		{//8bit
			pSPIHandle->pSPIx->DR=(*pSPIHandle->pTXBuffer);
			pSPIHandle->TxLen--;
					pSPIHandle->pTXBuffer++;
					 }
	if(!pSPIHandle->TxLen)
	{

		//TX len is 0, so close spi transmission
		 SPI_CLOSE_TRANSMISSION(pSPIHandle);}
	SPI_Application_event_call_back(pSPIHandle,SPI_EVENT_TX_CMPLT);
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
					{
						//16 bit DFF
						//1. load the data from DR to Rxbuffer address
						 *((uint16_t*)pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->DR ;
						pSPIHandle->RxLen--;
						pSPIHandle->RxLen--;
						(uint16_t*)pSPIHandle->pRXBuffer++;
					}else
					{
						//8 bit DFF
						*(pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->DR ;
						pSPIHandle->RxLen--;
						pSPIHandle->pRXBuffer++;
					}
	if(!pSPIHandle->RxLen)
		{

			//TX len is 0, so close spi transmission
		SPI_CLOSE_RECEPTION(pSPIHandle);}
		SPI_Application_event_call_back(pSPIHandle,SPI_EVENT_RX_CMPLT);

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
//CLEAR OVR FLAG
	uint8_t temp;
	if(pSPIHandle->TxState!=SPI_BUSY_IN_TX)
	{
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}
	SPI_Application_event_call_back(pSPIHandle,SPI_EVENT_OVR_CMPLT);
}
void SPI_Clear_OVR_Flag(SPI_RegDef_t *pSPIx)
{
uint8_t temp;
temp=pSPIx->DR;
temp=pSPIx->SR;
(void)temp;
}
void SPI_CLOSE_TRANSMISSION(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_TXEIE);
			pSPIHandle->pTXBuffer=NULL;
			pSPIHandle->TxLen=0;
			pSPIHandle->TxState=SPI_READY;
}
void SPI_CLOSE_RECEPTION(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_RXNEIE);
			pSPIHandle->pRXBuffer=NULL;
			pSPIHandle->RxLen=0;
			pSPIHandle->RxState=SPI_READY;
}

void SPI_Application_event_call_back(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}

