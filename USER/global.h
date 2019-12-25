#ifndef _GLOBAL_PARAMETER_H__
#define _GLOBAL_PARAMETER_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "AttTrack_lib.h"


#ifdef __cplusplus
extern "C"{
#endif

#define	VERSION_YEAR   18
#define	VERSION_MONTH  12
#define	VERSION_DAY    30
#define	SENSOR_TYPE    0x02 
	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
/********************************************************************************************************/  
typedef enum {FALSE = 0, TRUE = !FALSE} TestStatus;

typedef struct 
{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3s16; 

typedef struct 
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
} Vector3u16;

typedef struct 
{
  int32_t x;
  int32_t y;
  int32_t z;
}Vector3s32;

typedef struct 
{
  uint32_t x;
  uint32_t y;
  uint32_t z;
}Vector3u32;

typedef struct 
{
  float x;
  float y;
  float z;
}Vector3f;

/********************************************************************************************************/  
extern float G_Dt;
extern uint8_t  USART_RX_BUF[USART_REC_LEN];
extern uint8_t test[8];	
	
extern char  DebugMessage[100];		
extern uint8_t FdcanSend_buff[8];
//extern uint8_t ADC7682_DataReady;
//extern uint8_t SCA3300_DataReady;
	
//extern int32_t Sampling_Rate_X_Sum;
//extern int32_t Sampling_Rate_Y_Sum;
//extern int32_t Sampling_Rate_Z_Sum;
//extern int32_t Sampling_Acc_X_Sum;
//extern int32_t Sampling_Acc_Y_Sum;
//extern int32_t Sampling_Acc_Z_Sum;
//extern int32_t Sampling_Temp_Sum;
extern Vector3u16 Result_Acc_Count;
extern uint16_t Result_Temp_Count;
extern uint16_t Result_Gyro_Count;

extern Vector3u32 Sampling_Gyro_Sum;
extern Vector3s32 Sampling_Acc_Sum;
extern int32_t Sampling_Temp_Sum;

extern Vector3u16 Result_Gyro;
extern Vector3s16 Result_Acc;
extern uint32_t Result_Delt_T;
extern int16_t Result_Temp;
extern Vector3f Send_Acc;
extern Vector3f Send_Gyro;
//-----------------------------------------------
extern Vector3s16 Receive_BoomGyro;
extern Vector3s16 Receive_BoomAcc;  
extern uint32_t Receive_BoomTime_Pre;
extern uint32_t Receive_BoomTime;
extern uint32_t Receive_BoomDelt_T;

extern Vector3s16 Receive_StickGyro;
extern Vector3s16 Receive_StickAcc;
extern uint32_t Receive_StickTime_Pre;
extern uint32_t Receive_StickTime;
extern uint32_t Receive_StickDelt_T;

extern Vector3s16 Receive_DogboneGyro;
extern Vector3s16 Receive_DogboneAcc;  
extern uint32_t Receive_DogboneTime_Pre;
extern uint32_t Receive_DogboneTime;
extern uint32_t Receive_DogboneDelt_T;
//-----------------------------------------------
extern IMUdata_t BodyImuDataRaw;
extern IMUdata_t BoomImuDataRaw;
extern IMUdata_t StickImuDataRaw;
extern IMUdata_t DogBoneImuDataRaw;
extern double BodyRollPitchResult[2];
extern double BoomPitchResult;
extern double StickPitchResult;
extern double DogBonePitchResult;


extern uint16_t Error_Status;
extern uint8_t  Button_Status;
/********************************************************************************************************/  


#ifdef __cplusplus
} // extern "C"
#endif



#endif

//------------------End of File----------------------------
