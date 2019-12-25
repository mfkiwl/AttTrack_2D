#include "global.h"


// Updated with the fast loop
float G_Dt = 0.001f;

uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
    

uint8_t test[8] = {0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x08,0x09};

/********************************************************************************************************/  

char  DebugMessage[100] = {0};
uint8_t FdcanSend_buff[8] = {0};


//int32_t Sampling_Rate_X_Sum = 0;
//int32_t Sampling_Rate_Y_Sum = 0;
//int32_t Sampling_Rate_Z_Sum = 0;
//int32_t Sampling_Acc_X_Sum = 0;
//int32_t Sampling_Acc_Y_Sum = 0;
//int32_t Sampling_Acc_Z_Sum = 0;
//int32_t Sampling_Temp_Sum = 0;

Vector3u16 Result_Acc_Count = {0}; 
uint16_t Result_Temp_Count = 0;
uint16_t Result_Gyro_Count = 0;

Vector3u32 Sampling_Gyro_Sum = {0};
Vector3s32 Sampling_Acc_Sum = {0};
int32_t Sampling_Temp_Sum = 0;

Vector3u16 Result_Gyro = {0};
Vector3s16 Result_Acc = {0};     
uint32_t Result_Delt_T = 0;
int16_t Result_Temp = 0;
Vector3f Send_Acc = {0};
Vector3f Send_Gyro = {0};
//-----------------------------------------------
Vector3s16 Receive_BoomGyro = {0};
Vector3s16 Receive_BoomAcc = {0}; 
uint32_t Receive_BoomTime_Pre = 0;
uint32_t Receive_BoomTime = 0;
uint32_t Receive_BoomDelt_T = 0;

Vector3s16 Receive_StickGyro = {0};
Vector3s16 Receive_StickAcc = {0}; 
uint32_t Receive_StickTime_Pre = 0;
uint32_t Receive_StickTime = 0;
uint32_t Receive_StickDelt_T = 0;

Vector3s16 Receive_DogboneGyro = {0};
Vector3s16 Receive_DogboneAcc = {0};
uint32_t Receive_DogboneTime_Pre = 0;
uint32_t Receive_DogboneTime = 0;
uint32_t Receive_DogboneDelt_T = 0;
//-----------------------------------------------
IMUdata_t BodyImuDataRaw = {0};
IMUdata_t BoomImuDataRaw = {0};
IMUdata_t StickImuDataRaw = {0};
IMUdata_t DogBoneImuDataRaw = {0};
double BodyRollPitchResult[2] = {0}; 
double BoomPitchResult = 0;
double StickPitchResult = 0;
double DogBonePitchResult = 0;
//-----------------------------------------------
uint16_t Error_Status = 0;
uint8_t  Button_Status = 0;



