#ifndef __CAN_PROTOCOL_H__
#define __CAN_PROTOCOL_H__

#include "stm32h7xx_hal.h" 

#ifdef __cplusplus
extern "C"{
#endif

//大臂、小臂、挖斗传感器can ID
#define FIRMWARE_VERSION	    0x270219
//#define BOOM_COBID            0x535  // 0x532
//#define STICK_COBID           0x539   //0x537
//#define DOGBONE_COBID         0x536   //0x556
	
#define BOOM_COBID            0x535  // 0x532
#define STICK_COBID           0x537   //0x537
#define DOGBONE_COBID         0x532  //0x556	
	
	
void FDCAN_ProtocolProcess(FDCAN_RxHeaderTypeDef RxHeader, uint8_t m_RxData[]);


  
  
  
  
#ifdef __cplusplus
} // extern "C"
#endif


#endif













