/*
 * stm32f446re_rcc_driver.c
 *
 *  Created on: 13-May-2020
 *      Author: atharva
 */
#include "stm32f446re_rcc_driver.h"
#include "stm32f446re.h"
uint16_t AHB1_PRESCALER[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PRESCALAR[4] = { 2, 4 , 8, 16};

uint32_t RCC_GETPCLK(void)

{
	uint32_t pclk1,sysclk;
	uint8_t clksrc,temp,ahb1,apb1;
	clksrc=((RCC->CFGR>>2)& 0x3);
	if(clksrc==0)
	{sysclk=16000000;
		}
	else if(clksrc==1)
	{
		sysclk=8000000;
	}else if(clksrc==2)
	{
		sysclk=RCC_GETPLLOUTPUTCLK();
	}
	temp=((RCC->CFGR>>4)&0xF);//FOR AHB1 Prescaler
	if(temp<8)
	{
		ahb1=1;

	}else{
		ahb1=AHB1_PRESCALER[temp-8];
	}

	//abp1
	temp=((RCC->CFGR>>10)&0x7);
	if(temp<4)
	{
		apb1=1;
	}else
	{
		apb1=APB1_PRESCALAR[temp-4];
	}
	pclk1=(sysclk/ahb1/apb1);
	return  pclk1;
}
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB1_PRESCALER[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PRESCALAR[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

