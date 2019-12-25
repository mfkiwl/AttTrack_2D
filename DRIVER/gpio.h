#ifndef _GPIO_H__
#define _GPIO_H__

#include "stm32h7xx_hal.h"

#define KEY_PORT		      GPIOB
#define KEY_CLOCK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define KEY_AUTO_PIN	    GPIO_PIN_0
#define KEY_UP_PIN		    GPIO_PIN_1
#define KEY_DOWN_PIN	    GPIO_PIN_2
/********************************************************************************************************/ 
#ifdef __cplusplus
extern "C"{
#endif
 
	
void Gpio_Init(void);

	
#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
