#ifndef _SPI_H__
#define _SPI_H__


#include <stdint.h>
#include "stm32h7xx_hal.h"


#ifdef __cplusplus
extern "C"{
#endif
 
	

void Spi_Init(void);
uint8_t Spi_SendMessage(uint8_t id, uint8_t *pRequest, uint8_t *pResponse, uint16_t Size);
	
#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
