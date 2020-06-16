/*
 * SPITX.c
 *
 *  Created on: 30-Dec-2019
 *      Author: atharva
 */
//PB12 AS NSS
//PB13 AS SCK
//PB14 AS MISO
//PB15 AS MOSI
#include "stm32f446re.h"
#include<string.h>
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MEDIUM;
	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//mosi
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//miso
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	//GPIO_Init(&SPIPins);
	//nss
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
		//GPIO_Init(&SPIPins);
}
void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx=SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM=SPI_SSM_SW;
	SPI_Init(&SPI2Handle);
	}
int main(void)
{   char user[]="hello world";
	SPI2_GPIOInits();

	SPI2_Init();
	SPI_SSIConfig(SPI2,ENABLE);//NSS IS INTERNALLY HIGH SO THAT MODF ERRORIS AVOIDED
	//enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2,ENABLE);
	SPI_SendData(SPI2,(uint8_t*)user,strlen(user));
	while(1);
	return 0;}
