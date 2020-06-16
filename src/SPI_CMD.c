/*
 * SPI_CMD.c
 *
 *  Created on: 01-Jan-2020
 *      Author: atharva
 */


//PB12 AS NSS
//PB13 AS SCK
//PB14 AS MISO
//PB15 AS MOSI
#include "stm32f446re.h"
#include<string.h>
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
void delay()
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);

}
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MEDIUM;
	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//mosi
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//miso
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//nss
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
		GPIO_Init(&SPIPins);
}
void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx=SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM=SPI_SSM_HW;
	SPI_Init(&SPI2Handle);
	}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}
uint8_t SPI_Verify_Response(uint8_t ackbyte)
{
	if(ackbyte==0xF5)
	{
		return 1;
	}
	return 0;
}
int main(void)
{ uint8_t dummybyte=0xff;
uint8_t dummyread=0xff;
	char user[]="helloworld";
  GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Init();
	//SPI_SSIConfig(SPI2,ENABLE);//NSS IS INTERNALLY HIGH SO THAT MODF ERRORIS AVOIDED
	//enable the SPI2 Peripheral
	SPI_SSOEConfig(SPI2,ENABLE);
	while(1)
		{
			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();
	        SPI_PeripheralControl(SPI2,ENABLE);
	        //
	        SPI_SendData(SPI2,(uint8_t*)user,strlen(user));
           uint8_t commandcode=COMMAND_LED_CTRL;
           uint8_t ackbyte;
           uint8_t arg[2];

           SPI_SendData(SPI2,&commandcode,1);
           //DO dummy read to clear RXNE
           SPI_ReceiveData(SPI2,&dummyread,1);
           //send dummy bytes to fetch response from slave
           SPI_SendData(SPI2,&dummybyte,1);
           //read the ack byte
           SPI_ReceiveData(SPI2,&ackbyte,1);

          if( SPI_Verify_Response(ackbyte))
          {
        	arg[0]=LED_ON;
        	arg[1]=LED_OFF;
        	SPI_SendData(SPI2,arg,2);
          }

          while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

          			//to avoid button de-bouncing related issues 200ms of delay
          delay();

         commandcode=COMMAND_SENSOR_READ;
         //Send code
         SPI_SendData(SPI2,&commandcode,1);
                    //DO dummy read to clear RXNE
                    SPI_ReceiveData(SPI2,&dummyread,1);
                    //send dummy bytes to fetch response from slave
                    SPI_SendData(SPI2,&dummybyte,1);
                    //read the ack byte
                    SPI_ReceiveData(SPI2,&ackbyte,1);

                   if( SPI_Verify_Response(ackbyte))
                   {
                 	arg[0]=ANALOG_PIN0;

                 	SPI_SendData(SPI2,arg,1);
                 	//DO dummy read
                 	                   SPI_ReceiveData(SPI2,&dummyread,1);
                 	                   //give delay for slave to respond
                 	                   delay();
                 	                SPI_SendData(SPI2,&dummybyte,1);
                 	                   uint8_t analogread;
                 	                   SPI_ReceiveData(SPI2,&analogread,1);
                   }

	       while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
	        SPI_PeripheralControl(SPI2,DISABLE);
		}
	return 0;
}


