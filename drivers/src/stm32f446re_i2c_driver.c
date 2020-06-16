/*
 * stm32f446re_i2c_driver.c
 *
 *  Created on: 09-Jan-2020
 *      Author: atharva
 */

#include "stm32f446re_i2c_driver.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1|=(1<< I2C_CR1_START);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{SlaveAddr=SlaveAddr<<1;
SlaveAddr&=~(1);
pI2Cx->DR=SlaveAddr;

}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr=SlaveAddr<<1;
	SlaveAddr|=(1);
	pI2Cx->DR=SlaveAddr;

}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyread=pI2Cx->SR1;
	 dummyread=pI2Cx->SR2;

	(void)dummyread;

}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1|=(1<<I2C_CR1_STOP);
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{if(EnorDi == ENABLE)
{
	if(pI2Cx == I2C1)
	{
		I2C1_PCLK_EN();
	}else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_EN();
	}else if (pI2Cx == I2C3)
	{
		I2C3_PCLK_EN();
	}
}
else
{
	if(pI2Cx==I2C1)
	{
		I2C1_PCLK_DI();
	}else if(pI2Cx==I2C2)
	{
		I2C2_PCLK_DI();
	}else if(pI2Cx==I2C3)
	{
		I2C3_PCLK_DI();
	}
}

}
uint16_t RCC_GETPLLOUTPUTCLK()
{
    return 0;
}
uint16_t AHB1_PRESCALER[8]={2,4,8,16,64,128,256,512};
uint8_t APB1_PRESCALER[4]={2,4,8,16};

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
		apb1=APB1_PRESCALER[temp-4];
	}
	pclk1=(sysclk/ahb1/apb1);
	return  pclk1;
}
/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{uint32_t temp=0;
temp|=pI2CHandle->I2C_Config.I2C_ACKControl<<10;//ack control
pI2CHandle->pI2Cx->CR1=temp;
temp=0;
temp|=RCC_GETPCLK()/1000000U;
pI2CHandle->pI2Cx->CR2=temp & 0x3F;
temp=0;
temp|=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
temp|=(1<<14);
pI2CHandle->pI2Cx->OAR1=temp;

uint16_t ccr_value=0;
temp=0;
if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM)
{
	ccr_value=(RCC_GETPCLK()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	temp|=(ccr_value & 0xfff);
	}
else
{
	temp|=(1<<15);
	temp|=(pI2CHandle->I2C_Config.I2C_FMDutyCycle<<14);
	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_2)
	{
		ccr_value=(RCC_GETPCLK()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	}else
	{
		ccr_value=(RCC_GETPCLK()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
	}
	temp|=(ccr_value & 0xfff);
}
pI2CHandle->pI2Cx->CCR=temp;
if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM)
{//STANDARD MODE

temp=(RCC_GETPCLK()/1000000U) +1;

	}
else
{
	temp=((RCC_GETPCLK()*300)/1000000000U)+1;
}
pI2CHandle->pI2Cx->TRISE=temp & 0x3f;
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{if(pI2Cx->SR1 & FlagName)
{
	return FLAG_SET;
}
	return FLAG_RESET;
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//generate start condition
	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	 //CONFIRM START GENERATION,UNTIL SB IS CLEARED SCL  WILL BE STRETCHED
	 while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));
     //send addr of slave with r/nw bit set to 0
	 I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);
	 //confirm address phase is completed by checking addr flag inSR1
	 while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));
	 //CLEAR ADDR FLAG
	 I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	 //send data
	 while(Len>0)
	 {while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	 pI2CHandle->pI2Cx->DR=*pTxbuffer;
	 pTxbuffer++;
	 Len--;
	 }
	 //lenght becomes 0 wait for TXE=1 and BTF=1 before generating stop condition
	 //when TXE=1 ,BTF=1 means both SR and DR are empty
	 while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	 while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));
	 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
//CONFIRM START GENERATION,UNTIL SB IS CLEARED SCL  WILL BE STRETCHED
while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));

I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));

//for 1 byte
if(Len==1)
{//disable acking
	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);//clear addr flag
	//wait for RXNE to turn 1

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));
	//generate stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	//read data into buffer

	*pRxBuffer=pI2CHandle->pI2Cx->DR;


}

if(Len>1)
{I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

//read data until len becomes 0
for(uint32_t i=Len; i>0;i--)
{
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));
	if(i==2)
	{//clear ack bit
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
	*pRxBuffer=pI2CHandle->pI2Cx->DR;
	pRxBuffer++;
}


}

I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);


}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{

	if(EnorDi==I2C_ACK_ENABLE)
	{
		pI2Cx->CR1|=(1<<I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1&=~(1<<I2C_CR1_ACK);
	}
}
