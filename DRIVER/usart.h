#ifndef _USART_H
#define _USART_H

#include "hal_typedef.h"

#ifdef __cplusplus
extern "C"{
#endif
	
	
void Usart_Init(void);
	
void Usart_SendMessage(uint8_t *pstr);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
