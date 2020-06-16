/*
 * led.c
 *
 *  Created on: 21-Apr-2020
 *      Author: atharva
 */
#include "stm32f446re.h"
//#include  "stm32f446re_gpio_driver.h"
//#include "stm32f446re_gpio_driver.c"

void delayMs(int n);

int main(void) {
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */

    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */

    while(1) {
        GPIOA->ODR |=  0x00000020;  /* turn on LED */
        delayMs(500);
        GPIOA->ODR &= ~0x00000020;  /* turn off LED */
        delayMs(500);
    }
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 3195; i++) ;
}
/*void delay()
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
	//GPIO_PeriClockControl(GPIOA,ENABLE);
	RCC->AHB1ENR|=1<<0;
	GPIO_Init(&GPIOLED);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}*/

