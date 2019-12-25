#ifndef __SYSTEM_H__
#define __SYSTEM_H__


#include <stdint.h>
#include "stm32h7xx_hal.h"

/****************************************************************/
#ifdef __cplusplus
extern "C" {
#endif           	/*__cplusplus*/
/****************************************************************/	
#define AD7682    1
#define SCA3300   2
	
#define AD7682_X_ERROR    0x0100
#define AD7682_Y_ERROR    0x0200
#define AD7682_Z_ERROR    0x0400
#define SCA3300_T_ERROR   0x0001
#define SCA3300_X_ERROR   0x0002
#define SCA3300_Y_ERROR   0x0004
#define SCA3300_Z_ERROR   0x0008
	
//#define AD7682_SAMPLE_TIMES   10
//#define SCA3300_SAMPLE_TIMES  10
	
#define AD7682_CS_LOW()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)	
#define AD7682_CS_HIGH()	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)	
#define SCA3300_CS_LOW()	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)	
#define SCA3300_CS_HIGH()	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)	
	
#define AD7682_REQUEST_CHANNEL0   0  //0xFF36   //0xFC30  //0xFC35  //0xFC30  //0x7C37  //0xFC30  //0xFC08   (GAsensor:0xFC30)
#define AD7682_REQUEST_CHANNEL1   1  //0xFCB0  //0x7CB7  //0xFCB0  //0xFC88
#define AD7682_REQUEST_CHANNEL2   2  //0xFD30  //0x7D37  //0xFD30  //0xFD08


//------------SCA3300 REQUEST----------------------------
// Standard requests
#define REQ_READ_ACC_X    0x040000F7 
#define REQ_READ_ACC_Y    0x080000FD
#define REQ_READ_ACC_Z    0x0C0000FB
#define REQ_READ_TEMP     0x140000EF
#define REQ_READ_STAT_SUM 0x180000E5
#define	REQ_READ_WHOAMI   0x40000091	
#define REQ_READ_STO      0x100000E9	
	
// Special requests
#define REQ_SW_RESET      0xB4002098
#define	REQ_CHANGE_MODE1  0xB400001F   //3g
#define	REQ_CHANGE_MODE2  0xB4000102   // 6g
#define	REQ_CHANGE_MODE3  0xB4000225   //1.5g
#define	REQ_CHANGE_MODE4  0xB4000338   //1.5g

// Frame field masks
#define OPCODE_FIELD_MASK 0xFC000000
#define RS_FIELD_MASK     0x03000000
#define DATA_FIELD_MASK   0x00FFFF00
#define CRC_FIELD_MASK    0x000000FF

/****************************************************************/	
void system_init(void);	
//uint16_t AD7682_SendRequest(uint16_t Request);
//uint32_t SCA3300_SendRequest(uint32_t Request);	
void AD7682_Sampling(void);
void SCA3300_Sampling(void);
/****************************************************************/
#ifdef __cplutplus
}
#endif				/*__cplusplus*/
/****************************************************************/
#endif

