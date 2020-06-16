/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: 26-Dec-2019
 *      Author: atharva
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_
#include "stm32f446re.h"
//configuration structure for spix peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

//handle structure for SPIX Peripheral
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTXBuffer;
	uint8_t *pRXBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;



/* APIS SUPPORTED BY DRIVER
 *
 *
 *
 *
 */
//PERIPHERAL CLOCK SETUP
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//INIT AND DEINIT

 void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


//DATA SEND AND RECEIVE

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTXBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRXBuffer,uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTXBuffer,uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRXBuffer,uint32_t len);

//irq config

void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t*pHandle);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
//DEVICE MODES
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_Clear_OVR_Flag(SPI_RegDef_t *pSPIx);
void SPI_CLOSE_TRANSMISSION(SPI_Handle_t *pHandle);
void SPI_CLOSE_RECEPTION(SPI_Handle_t *pHandle);

void SPI_Application_event_call_back(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE  0
//BUS CONFIG

#define SPI_BUS_CONFIG_FD            1
#define SPI_BUS_CONFIG_HD            2
#define SPI_BUS_CONFIG_SIMPLEX_RX    3


//CLOCK SPEED
#define SPI_SCLK_SPEED_DIV2      0
#define SPI_SCLK_SPEED_DIV4      1
#define SPI_SCLK_SPEED_DIV8      2
#define SPI_SCLK_SPEED_DIV16     3
#define SPI_SCLK_SPEED_DIV32     4
#define SPI_SCLK_SPEED_DIV64     5
#define SPI_SCLK_SPEED_DIV128    6
#define SPI_SCLK_SPEED_DIV256    7
//DFF
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1
//CPOL
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0
//CPHA
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0
//SSM
#define SPI_SSM_SW  0
#define SPI_SSM_HW  1

#define SPI_TXE_FLAG (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

#define SPI_READY  0
#define SPI_BUSY_IN_RX  1
#define SPI_BUSY_IN_TX 2


#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_CMPLT 3
#define SPI_EVENT_CRC_CMPLT 4






#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
