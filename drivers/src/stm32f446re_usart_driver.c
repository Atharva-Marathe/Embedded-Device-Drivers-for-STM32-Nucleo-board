/*
 * stm32f446re_usart_driver.c
 *
 *  Created on: 09-Apr-2020
 *      Author: atharva
 */


#include "stm32f446re_usart_driver.h"
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx==USART1)
									{
										USART1_PCLK_DI() ;
									}
									else if(pUSARTx==USART2)
									{
										USART2_PCLK_DI();
									}
									else if (pUSARTx == USART3)
											{
												USART3_PCLK_DI();
											}else if (pUSARTx == UART4)
											{
												UART4_PCLK_DI();
											}//TODO
	}

}

void USART_Init(USART_Handle_t *pUSARTHandle)
{ uint32_t temp=0;

//USART TX AND RX ENGINES ACC TO USART MODE CONFIGURATIONS
if(pUSARTHandle->USART_Config.USART_Mode==USART_MODE_ONLY_RX)
{
	temp|=(1<<USART_CR1_RE );
}
else if(pUSARTHandle->USART_Config.USART_Mode==USART_MODE_ONLY_TX)
{
	temp|=(1<<USART_CR1_TE );
}
else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
{
	//Implement the code to enable the both Transmitter and Receiver bit fields
	temp |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
}

temp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

//configuring parity bits

if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_EN_EVEN)
{
	//enable parity control
	temp|=(1<<USART_CR1_PCE);
}
else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
{
	//Implement the code to enable the parity control
    temp |= ( 1 << USART_CR1_PCE);

    //Implement the code to enable ODD parity
    temp |= ( 1 << USART_CR1_PS);

}

//configuring the CR1 REgister
pUSARTHandle->pUSARTx->CR1=temp;


//cr2 configuration
temp=0;
if(pUSARTHandle->USART_Config.USART_NoOfStopBits==USART_STOPBITS_0_5)
{temp|=(1<<USART_CR2_STOP);

}
else if(pUSARTHandle->USART_Config.USART_NoOfStopBits==USART_STOPBITS_2)
{
	temp|= (2<<USART_CR2_STOP);
}
else if(pUSARTHandle->USART_Config.USART_NoOfStopBits==USART_STOPBITS_1_5)
{temp|=(3<<USART_CR2_STOP);

}
//program the CR2 Register
pUSARTHandle->pUSARTx->CR2 = temp;
//configuring CR3 Register

temp=0;
if(pUSARTHandle->USART_Config.USART_HWFlowControl==USART_HW_FLOW_CTRL_CTS)
{
	temp|= (1<<USART_CR3_CTSE);
}
else if(pUSARTHandle->USART_Config.USART_HWFlowControl==USART_HW_FLOW_CTRL_RTS)
{
	temp|=(1<<USART_CR3_RTSE);
}
else if(pUSARTHandle->USART_Config.USART_HWFlowControl==USART_HW_FLOW_CTRL_CTS_RTS)
{
	temp|= (1<<USART_CR3_CTSE);
	temp|=(1<<USART_CR3_RTSE);

}
pUSARTHandle->pUSARTx->CR3 = temp;

USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);


}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}


void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
  uint16_t *pdata;

  for(uint32_t i=0; i<Len;i++)
  {   while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

    //check usart word length for 8 or 9 bits

  if(pUSARTHandle->USART_Config.USART_WordLength ==USART_WORDLEN_9BITS)
  {

	  pdata=(uint16_t*)pTxBuffer;
	  pUSARTHandle->pUSARTx->DR =(*pdata & (uint16_t)0x1ff);
	  //check for parity

	  if(pUSARTHandle->USART_Config.USART_ParityControl ==USART_PARITY_DISABLE)
	  {
		  pTxBuffer++;
		  pTxBuffer++;
	  }
	  else
	  {
		  pTxBuffer++;
	  }
  }

	  else
	  {
		  pUSARTHandle->pUSARTx->DR =(*pdata & (uint16_t)0xff);
		  pTxBuffer++;
	  }
  }

  while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));

  }



void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}

}



