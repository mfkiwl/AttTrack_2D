#include "tasks.h"
#include "scheduler.h"
#include "hal.h"
#include "global.h"
#include "angle_cal.h"

#include "ComFunc.h"



extern st_MachineResult s_MachineResult;



#if DEBUG_100HZ_LOOP_TIME
uint32_t run100HzStartTime = 0;
uint32_t run100HzTime = 0;
#endif

void run_100hz_task(void)
{
#if DEBUG_100HZ_LOOP_TIME
  run100HzStartTime = hal.psystick->micros();
#endif
  uint8_t Error_Buff[8] = {0};
	if (Error_Status)
	{
		Error_Buff[0] = Error_Status; 
		Error_Buff[1] = Error_Status>>8;
		hal.pfdcan->send_message(0x305, Error_Buff, 2);
	}
	
	MachineResultOutput();
	

	double BodyRollResultTemp  = BodyRollPitchResult[0] * R2D;
	double BodyPitchResultTemp = BodyRollPitchResult[1] * R2D;
	double BoomPitchResultTemp = BoomPitchResult * R2D;
	double StickPitchResultTemp = StickPitchResult * R2D;
	double DogBonePitchResultTemp = DogBonePitchResult * R2D;
//	printf("GYRO_X: %f, GYRO_Y: %f, GYRO_Z: %f, DeltT: %05d\r\n", Send_Gyro.x, Send_Gyro.y, Send_Gyro.z, Result_Delt_T);  //cost 3.735ms
//	printf("BodyRollResult: %f, BodyPitchResult: %f, BoomPitchResult: %f, StickPitchResult: %f, DogBonePitchResult: %f\r\n",
//	        BodyRollResultTemp, BodyPitchResultTemp, BoomPitchResultTemp, StickPitchResultTemp, DogBonePitchResultTemp);  //cost 3.735ms
 
//	printf("H: %d, L: %d, alpha: %d, beta: %d, gamma: %d\r\n",
//				s_MachineResult.y_distance, s_MachineResult.x_distanse, s_MachineResult.angle_alpha, s_MachineResult.angle_beta, s_MachineResult.angle_gamma);  //cost 3.735ms
	
#if DEBUG_100HZ_LOOP_TIME
  run100HzTime = get_systick_deltaT(run100HzStartTime, hal.psystick->micros()); 
#endif
}




