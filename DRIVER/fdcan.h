#ifndef _FDCAN_H__
#define _FDCAN_H__

#include "stdint.h"
#include "stm32h7xx_hal.h" 

#ifdef __cplusplus
extern "C"{
#endif
 
//FDCAN1接收RX0中断使能
#define FDCAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.


	
void Fdcan_Init(void);
void Fdcan_SendMessage(uint32_t id, uint8_t *msg, uint16_t len);
void FdcanExt_SendMessage(uint32_t id, uint8_t *msg, uint16_t len);
void Fdcan_ReadMessage(uint8_t *buf);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
