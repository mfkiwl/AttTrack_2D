#include "fastloop.h"
#include "scheduler.h"
#include "global.h"
#include "system.h" 

#include "angle_cal.h"

#include "AttTrack_lib.h"
#include "ComFunc.h"
/********************************************************************************************************/
#if DEBUG_FAST_LOOP_TIME
uint32_t fastLoopStartTime = 0;
uint32_t fastLoopRunTime = 0;
#endif

static uint16_t Sampling_Loop_Count = 0;

void Send_GyroAcc_Infor(void);
void ProcessMachineAngle(void);
/********************************************************************************************************/
// fast loop, This is the foremost loop ----> 1000hz
void fast_loop(uint32_t dt)
{
/* Just for test running time*/
#if DEBUG_FAST_LOOP_TIME
	fastLoopStartTime  = scheduler_micros();
#endif	
	Result_Delt_T += dt;
	
	G_Dt = (float)dt / 1000000.f;   // microsecond transform to second

	if (G_Dt > MAIN_LOOP_THRESHOLD)
	{
		G_Dt = MAIN_LOOP_SECONDS;
	}
	
	SCA3300_Sampling();
	AD7682_Sampling(); 
	
	Sampling_Loop_Count++;
	if (Sampling_Loop_Count >= 5)  //200HZ
	{
		Sampling_Loop_Count = 0;
		
		if (!Error_Status)
			Send_GyroAcc_Infor();   //this function cost 274us
		
		Result_Delt_T = 0; 
	}
  
	ProcessMachineAngle();
	
#if DEBUG_FAST_LOOP_TIME 
	fastLoopRunTime = get_systick_deltaT(fastLoopStartTime, scheduler_micros()); 
#endif
}

#if DEBUG_FAST_LOOP_TIME
uint32_t runSendInfoStartTime = 0;
uint32_t runSendInfoTime = 0;
#endif
void Send_GyroAcc_Infor(void)
{
#if DEBUG_100HZ_LOOP_TIME
			runSendInfoStartTime = hal.psystick->micros();
#endif
//		Vector3f Send_Acc = {0};
//		Vector3f Send_Gyro = {0};
		  float Send_Temp = 0.0;
//--------------------------------------------------------------------------------------------------	
			if (Result_Acc_Count.x)
				Result_Acc.x = Sampling_Acc_Sum.x / Result_Acc_Count.x;
			if (Result_Acc_Count.y)
				Result_Acc.y = Sampling_Acc_Sum.y / Result_Acc_Count.y;
			if (Result_Acc_Count.z)
				Result_Acc.z = Sampling_Acc_Sum.z / Result_Acc_Count.z;
			if (Result_Temp_Count)
				Result_Temp  = Sampling_Temp_Sum  / Result_Temp_Count; 
		
			Sampling_Acc_Sum.x = 0;
			Sampling_Acc_Sum.y = 0;
			Sampling_Acc_Sum.z = 0;
			Sampling_Temp_Sum = 0;
			Result_Acc_Count.x = 0;
			Result_Acc_Count.y = 0;
			Result_Acc_Count.z = 0;
			Result_Temp_Count = 0; 
	
			Send_Acc.x = (double)Result_Acc.x / 1350.0f;
		  Send_Acc.y = (double)Result_Acc.y / 1350.0f;
		  Send_Acc.z = (double)Result_Acc.z / 1350.0f;
		  Send_Temp = (((double)Result_Temp / 18.9f) - 273)*10;
//			sprintf(DebugMessage, "ACC_X: %f, ACC_Y: %f, ACC_Z: %f, TEMP: %f\r\n", Send_Acc.x, Send_Acc.y, Send_Acc.z, Send_Temp);  //cost 4ms
//			hal.pusart->send_message((uint8_t *)DebugMessage); //cost 2.45ms
//--------------------------------------------------------------------------------------------------	
//		start_value = SysTick->VAL;
//		hal.ptimer->wait_us(350);  //cost 271us, between two CAN send message,must add 271us delay
//		end_value = SysTick->VAL;
//		elapse_value = start_value - end_value;
//--------------------------------------------------------------------------------------------------	
		if (Result_Gyro_Count)
		{
			Result_Gyro.x = Sampling_Gyro_Sum.x / Result_Gyro_Count;
			Result_Gyro.y = Sampling_Gyro_Sum.y / Result_Gyro_Count;
			Result_Gyro.z = Sampling_Gyro_Sum.z / Result_Gyro_Count;
			Sampling_Gyro_Sum.x = 0;
			Sampling_Gyro_Sum.y = 0;
			Sampling_Gyro_Sum.z = 0;
			Result_Gyro_Count = 0;

			Send_Gyro.x = (double)(Result_Gyro.x - 32768) / 262.14f;   //13107*0.02=262.14
		  Send_Gyro.y = (double)(Result_Gyro.y - 32768) / 262.14f;
		  Send_Gyro.z = (double)(Result_Gyro.z - 32768) / 262.14f;
		}
			BodyImuDataRaw.accx = Send_Acc.x;
			BodyImuDataRaw.accy = -Send_Acc.y;
			BodyImuDataRaw.accz = Send_Acc.z;
			BodyImuDataRaw.gyox = Send_Gyro.x * D2R;
			BodyImuDataRaw.gyoy = Send_Gyro.y * D2R;
			BodyImuDataRaw.gyoz = Send_Gyro.z * D2R;
		  BodyImuDataRaw.state_acc = 0;
			BodyImuDataRaw.imutimetarget = Result_Delt_T / 1000000.0f;
		  Body_IS203_Pro(&BodyImuDataRaw, BodyRollPitchResult); 
}

void ProcessMachineAngle(void)
{
	/************************大臂角度处理*****************************/			
			BoomImuDataRaw.accx = Receive_BoomAcc.x / 5886.0;//加计零偏补偿
			BoomImuDataRaw.accy = -Receive_BoomAcc.z / 5886.0;
			BoomImuDataRaw.accz = -Receive_BoomAcc.y / 5886.0;
			BoomImuDataRaw.gyox = 0;
			BoomImuDataRaw.gyoy = -(Receive_BoomGyro.z / 50.0f) * D2R;
			BoomImuDataRaw.gyoz = 0; 
			BoomImuDataRaw.imutimetarget = Receive_BoomDelt_T / 1000000.0f;
		  Boom_GA_Pro(&BoomImuDataRaw, &BoomPitchResult);
		//	printf("BoomAccX: %f, BoomAccY: %f, BoomAccZ: %f, BoomGyroZ: %f\r\n",BoomImuDataRaw.accx, BoomImuDataRaw.accy, BoomImuDataRaw.accz, BoomImuDataRaw.gyoy);
			 
/**************************小臂角度处理******************************/
			StickImuDataRaw.accx = (Receive_StickAcc.x / 5886.0);
			StickImuDataRaw.accy = -Receive_StickAcc.z / 5886.0;
			StickImuDataRaw.accz = -Receive_StickAcc.y / 5886.0;
			StickImuDataRaw.gyox = 0;
			StickImuDataRaw.gyoy = -(Receive_StickGyro.z / 50.0f) * D2R;
			StickImuDataRaw.gyoz = 0;
			StickImuDataRaw.imutimetarget = Receive_StickDelt_T / 1000000.0f;
		  Stick_GA_Pro(&StickImuDataRaw, &StickPitchResult);
			//printf("StickAccX: %f, StickAccY: %f, StickAccZ: %f, StickGyroZ: %f\r\n",StickImuDataRaw.accx, StickImuDataRaw.accy, StickImuDataRaw.accz, StickImuDataRaw.gyoy);
			
/**************************挖斗角度处理****************************/
			DogBoneImuDataRaw.accx = Receive_DogboneAcc.x / 5886.0;
			DogBoneImuDataRaw.accy = -Receive_DogboneAcc.z/ 5886.0;
			DogBoneImuDataRaw.accz = -Receive_DogboneAcc.y / 5886.0;
			DogBoneImuDataRaw.gyox = 0;
			DogBoneImuDataRaw.gyoy = -(Receive_DogboneGyro.z / 50.0f) * D2R;
			DogBoneImuDataRaw.gyoz = 0;
			DogBoneImuDataRaw.imutimetarget = Receive_DogboneDelt_T / 1000000.0f;
			//DogBoneImuDataRaw.imutimetarget =0.01;
	    Dogbone_GA_Pro(&DogBoneImuDataRaw, &DogBonePitchResult);
		printf("%f,%f,%f,%f\r\n",DogBoneImuDataRaw.accx, DogBoneImuDataRaw.accy, DogBoneImuDataRaw.accz, DogBoneImuDataRaw.gyoy);
//printf("angle:%f,%f,%f\n",BoomPitchResult*180/3.14159,StickPitchResult*180/3.14159,DogBonePitchResult*180/3.14159);
			GetMachineAngle(BodyRollPitchResult, BoomPitchResult, StickPitchResult, DogBonePitchResult);
}







