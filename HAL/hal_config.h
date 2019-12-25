#ifndef _HAL_CONFIG_H__
#define _HAL_CONFIG_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "hal_typedef.h"	

	
extern SYSTICKDriver_st hal_systick;
extern GPIODriver_st    hal_gpio;
extern FDCANDriver_st   hal_fdcan;
extern SPIDriver_st     hal_spi;  
extern USARTDriver_st   hal_usart;  
extern TIMERDriver_st   hal_timer;

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
