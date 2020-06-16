/*
 * 001ledtoggle.c
 *
 *  Created on: 05-Oct-2019
 *      Author: atharva
 */
#include "stm32f446re.h"
//#include  "stm32f446re_gpio_driver.h"
//#include "stm32f446re_gpio_driver.c"
void delay()
{
	for(uint32_t i=0;i<500000;i++);
}
int main()
{
	GPIO_Handle_t  GPIOLED;
	GPIOLED.pGPIOx=GPIOA;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MEDIUM;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOLED);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}

