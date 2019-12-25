#include "can_protocol.h"
#include "global.h"
#include "scheduler.h" 
#include "hal.h"



void FDCAN_ProtocolProcess(FDCAN_RxHeaderTypeDef FDCAN_RxHeader, uint8_t m_RxData[])
{
	uint8_t Txdata[8] = {0};
//	uint8_t *prxData = pRxData;  //函数调用如何传递数组数据

	if (FDCAN_RxHeader.Identifier == 0x257)         // query firmware version, LENGTH = 1;
	{
		if ((0x0A ==  m_RxData[0]) && (0x00010000 == FDCAN_RxHeader.DataLength))   
		{
				Txdata[0] = (uint8_t)FIRMWARE_VERSION; 
				Txdata[1] = (uint8_t)(FIRMWARE_VERSION >> 8);
				Txdata[2] = (uint8_t)(FIRMWARE_VERSION >> 16);
				Txdata[3] = 0x04; 
				hal.pfdcan->send_message(0x256, Txdata, 4);
		}
	}
	else if (FDCAN_RxHeader.Identifier == 0x260)    //query configration parameter      
	{
		if ((0x0A ==  m_RxData[0]) && (0x00010000 == FDCAN_RxHeader.DataLength))
		{
//			Txdata[0] = BodyPitchResult;
//			Txdata[1] = BodyPitchResult;
//			Txdata[2] = BodyPitchResult;
//			Txdata[3] = BodyPitchResult; 
			hal.pfdcan->send_message(0x261, Txdata, 4);
//			Txdata[0] = BodyPitchResult;
//			Txdata[1] = BodyPitchResult;
//			Txdata[2] = BodyPitchResult;
//			Txdata[3] = BodyPitchResult; 
//			Txdata[4] = BodyPitchResult;
//			Txdata[5] = BodyPitchResult; 
			hal.pfdcan->send_message(0x262, Txdata, 6);
		}
	}
	else if (FDCAN_RxHeader.Identifier == 0x271)   //set configration parameter 1
	{
		if (0x00040000 == FDCAN_RxHeader.DataLength)
		{
			// get parameter 1 here.
			Txdata[0] = 0x01;
			hal.pfdcan->send_message(0x272, Txdata, 1);
		}
	}
	else if (FDCAN_RxHeader.Identifier == 0x273)   //set configration parameter 2
	{
		if (0x00060000 == FDCAN_RxHeader.DataLength)
		{
			// get parameter 2 here.
			Txdata[0] = 0x01;
			hal.pfdcan->send_message(0x274, Txdata, 1);
		}
	}
	else if (FDCAN_RxHeader.Identifier == BOOM_COBID)     //get Boom raw data. 
	{
		if (0x00080000 == FDCAN_RxHeader.DataLength)
		{
			Receive_BoomGyro.x = 0;
			Receive_BoomGyro.y = 0;
			Receive_BoomGyro.z = m_RxData[0] + (m_RxData[1] << 8);
			Receive_BoomAcc.x = m_RxData[2] + (m_RxData[3] << 8); 
			Receive_BoomAcc.y = m_RxData[4] + (m_RxData[5] << 8);
			Receive_BoomAcc.z = m_RxData[6] + (m_RxData[7] << 8);			
			Receive_BoomTime_Pre = Receive_BoomTime; 
			Receive_BoomTime  = scheduler_micros();
			Receive_BoomDelt_T = get_systick_deltaT(Receive_BoomTime_Pre, Receive_BoomTime);  
		}
	}
	else if (FDCAN_RxHeader.Identifier == STICK_COBID)     //get Stick raw data.
	{
		if (0x00080000 == FDCAN_RxHeader.DataLength)
		{
			Receive_StickGyro.x = 0;
			Receive_StickGyro.y = 0;
			Receive_StickGyro.z = m_RxData[0] + (m_RxData[1] << 8);
			Receive_StickAcc.x = m_RxData[2] + (m_RxData[3] << 8);      
			Receive_StickAcc.y = m_RxData[4] + (m_RxData[5] << 8); 
			Receive_StickAcc.z = m_RxData[6] + (m_RxData[7] << 8); 
			Receive_StickTime_Pre = Receive_StickTime; 
			Receive_StickTime  = scheduler_micros();
			Receive_StickDelt_T = get_systick_deltaT(Receive_StickTime_Pre, Receive_StickTime);  
		}
	}
	else if (FDCAN_RxHeader.Identifier == DOGBONE_COBID)   //get Dogbone raw data.
	{
		if (0x00080000 == FDCAN_RxHeader.DataLength)
		{
			Receive_DogboneGyro.x = 0;
			Receive_DogboneGyro.y = 0;
			Receive_DogboneGyro.z = m_RxData[0] + (m_RxData[1] << 8);
			Receive_DogboneAcc.x = m_RxData[2] + (m_RxData[3] << 8);   
			Receive_DogboneAcc.y = m_RxData[4] + (m_RxData[5] << 8); 
			Receive_DogboneAcc.z = m_RxData[6] + (m_RxData[7] << 8); 	
			Receive_DogboneTime_Pre = Receive_DogboneTime; 
			Receive_DogboneTime  = scheduler_micros();
			Receive_DogboneDelt_T = get_systick_deltaT(Receive_DogboneTime_Pre, Receive_DogboneTime);  			
		}
	}
}















