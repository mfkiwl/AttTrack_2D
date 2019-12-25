#include "angle_cal.h"
#include "hal.h"
#include <math.h>
//#include <arm_math.h>


//#include "config.h"
//#include "sens.h"
//#include "myfunc.h"
//#include "sets.h"
//#include "can.h"
//#include "acc.h"
//#include "flash.h"

/********************************************************************************************************/ 

uint32_t s_OutState = DATA_OUT_IDLE;

const st_MachineMsg s_MachineMsg ={
																		.s1 = 5716,
																		.s2 = 2925,
																		.s3 = 1494,
																		.l1 = 640,
																		.l5 = 2515,
																		.DGlen = 240,
																		.l3 = 458,
																		.l2 = 600,
																		.l4 = 411
																	};

st_MachineResult s_MachineResult = {0};
static st_Calresult s_Calresult = {0};
static st_LocalAngle s_LocalAngle = {0};
static st_AngleCorrect s_AngleCorrect = {0};

void AngleCalculate(void);
void CalculateCoordinate(float arm_angle,float forearm_angle,float bone_angle);
/*----------------------------------------------
cos(m) = (l4^2 + s2^2 - l5^2)/2*l4*s2

∠b = 180 - acos(l_dg/s3)
---------------------------------------------*/
void MachineInit(void)
{
	s_Calresult.rad_b = SENS_PI - acosf(s_MachineMsg.DGlen*1.0f/s_MachineMsg.s3);
	//s_Calresult.rad_m = data_square(s_MachineMsg.l4) + data_square(s_MachineMsg.s2) - data_square(s_MachineMsg.l5);
	s_Calresult.rad_m = data_square(s_MachineMsg.l4) + data_square(s_MachineMsg.s2) - data_square(s_MachineMsg.l5);
  //s_Calresult.rad_m = DEG2RAD(5); //acosf(s_Calresult.rad_m/(2*s_MachineMsg.l4*s_MachineMsg.s2));
	s_Calresult.rad_m = acosf(s_Calresult.rad_m/(2*s_MachineMsg.l4*s_MachineMsg.s2));
	s_AngleCorrect.state = CORRECT_FINISH;
}


void MachineConfigInit(int32_t angle_arm, int32_t angle_forearm, int32_t angle_bone, int32_t body_x, int32_t body_y)
{
	s_AngleCorrect.angle_arm = angle_arm;
	s_AngleCorrect.angle_forearm = angle_forearm;
	s_AngleCorrect.angle_bone = angle_bone;
	s_AngleCorrect.body_x = body_x;
	s_AngleCorrect.body_y = body_y;
	s_AngleCorrect.state = CORRECT_FINISH;
}


void MachineAngleCorrect(uint8_t correct_cmd)
{
	if(CORRECT_FINISH  != s_AngleCorrect.state || 
		(correct_cmd != CORRETC_ARM && correct_cmd != CORRECT_FOREARM
		&& correct_cmd != CORRECT_BODY && correct_cmd != CORRECT_BONEDOG))
	{
		return;
	}
	s_AngleCorrect.angle_name = correct_cmd;
	s_AngleCorrect.state = CORRECT_BEGIN;
}


void GetMachineAngle(double *body_angle, double arm_angle, double forearm_angle, double bone_angle)
{
//	if (body_angle == 0)
//		return;
	
//	s_LocalAngle.body_x_angle = (int32_t)(body_angle[0] * 100);
//	s_LocalAngle.body_y_angle = (int32_t)(body_angle[1] * 100);
//	s_LocalAngle.arm_angle = (int32_t)(arm_angle * 100);
//	s_LocalAngle.forearm_angle = (int32_t)(forearm_angle * 100);
//	s_LocalAngle.bone_angle = (int32_t)(bone_angle * 100);
	
//	s_LocalAngle.body_x_angle = (int32_t)(RAD2DEG(body_angle[0]) * 100);
//	s_LocalAngle.body_y_angle = (int32_t)(RAD2DEG(body_angle[1]) * 100);
//	s_LocalAngle.arm_angle = (int32_t)(RAD2DEG(arm_angle) * 100);
//	s_LocalAngle.forearm_angle = (int32_t)(RAD2DEG(forearm_angle) * 100);
//	s_LocalAngle.bone_angle = (int32_t)(RAD2DEG(bone_angle) * 100);
	
//	AngleCalculate();
	MachineInit();
	float arm_angle_1 = (float)arm_angle + DEG2RAD(20.65); ;//DEG2RAD(21.27);
	float forearm_angle_1 = (float)forearm_angle + DEG2RAD(-0.82); //DEG2RAD(-2.5);
	float bone_angle_1 = (float)bone_angle + DEG2RAD(-0.71); //DEG2RAD(-0.85);  
	CalculateCoordinate(arm_angle_1, forearm_angle_1, bone_angle_1);
	
}  
#include "ComFunc.h"

void AngleCalculate(void)
{
	#define CORRECT_NUM  10
//	static uint8_t angle_cnt;
	int32_t body_x_angle = 0;
	int32_t body_y_angle = 0;
	int32_t arm_angle = 0;
	int32_t forearm_angle = 0;
	int32_t bone_angle = 0;
	
	s_AngleCorrect.body_x = 0;
	s_AngleCorrect.body_y = 0;
	s_AngleCorrect.angle_arm = 2127;
	s_AngleCorrect.angle_forearm = -250;
	s_AngleCorrect.angle_bone = -85;

//	body_x_angle = s_LocalAngle.body_x_angle   + s_AngleCorrect.body_x;
//	body_y_angle = s_LocalAngle.body_y_angle	 + s_AngleCorrect.body_y;	
	body_x_angle = 0;
	body_y_angle = 0;	
//	arm_angle 	  = s_LocalAngle.arm_angle     + s_AngleCorrect.angle_arm;
//	forearm_angle = s_LocalAngle.forearm_angle + s_AngleCorrect.angle_forearm;
//	bone_angle	  = s_LocalAngle.bone_angle    + s_AngleCorrect.angle_bone;
	arm_angle 	  = s_LocalAngle.arm_angle     + s_AngleCorrect.angle_arm;
	forearm_angle = s_LocalAngle.forearm_angle + s_AngleCorrect.angle_forearm;
	bone_angle	  = s_LocalAngle.bone_angle    + s_AngleCorrect.angle_bone;
	
	
//	printf("body_x_angle: %d, body_y_angle: %d, arm_angle: %d, forearm_angle: %d, bone_angle: %d\r\n",body_x_angle, body_y_angle, arm_angle, forearm_angle, bone_angle);
	
	CalculateCoordinate(arm_angle/100.0f,forearm_angle/100.0f,bone_angle/100.0f);
	
//	s_MachineResult.y_distance = (int32_t)(s_Calresult.y_coor);	  //H mm
//	s_MachineResult.x_distanse = (int32_t)(s_Calresult.x_coor);   //L
//	s_MachineResult.angle_alpha = arm_angle - body_x_angle;
//	s_MachineResult.angle_beta  = forearm_angle - arm_angle;
//	s_MachineResult.angle_beta  = 18000 + s_MachineResult.angle_beta;
//	s_MachineResult.angle_gamma = 18000 + (int32_t)(RAD2DEG(s_Calresult.rad_gama)*100);	
//	s_MachineResult.arm_body_angle    = s_LocalAngle.arm_angle  - body_x_angle;
//	s_MachineResult.forarm_body_angle = s_LocalAngle.forearm_angle - body_x_angle;
//	s_MachineResult.body_x = body_x_angle;
//	s_MachineResult.body_y = body_y_angle;		

//	s_OutState = DATA_HIGH;
//	if(CORRECT_FINISH == s_AngleCorrect.state)
//	{
//		return;
//	}
//	else if(s_AngleCorrect.state == CORRECT_BUSY)
//	{
//		angle_cnt ++;
//		switch(s_AngleCorrect.angle_name)
//		{
//			case CORRETC_ARM:
//			{
//				s_AngleCorrect.angle_arm += s_LocalAngle.arm_angle;
//				break;
//			}
//			case CORRECT_FOREARM:
//			{
//				s_AngleCorrect.angle_forearm += s_LocalAngle.forearm_angle;
//				break;
//			}
//			case CORRECT_BONEDOG:
//			{
//				s_AngleCorrect.angle_bone += s_LocalAngle.bone_angle;
//				break;
//			}
//			case CORRECT_BODY:
//			{
//				s_AngleCorrect.body_x += x_angle;
//				s_AngleCorrect.body_y += y_angle;
//				break;
//			}
//			default:
//				break;
//		}
//		if(angle_cnt == CORRECT_NUM)  //correct finish,get result
//		{
//			switch(s_AngleCorrect.angle_name)
//			{
//				case CORRETC_ARM:
//				{
//					s_AngleCorrect.angle_arm /= CORRECT_NUM;
//					break;
//				}
//				case CORRECT_FOREARM:
//				{
//					s_AngleCorrect.angle_forearm /= CORRECT_NUM;
//          s_AngleCorrect.angle_forearm += 9000;
//					break;
//				}
//				case CORRECT_BONEDOG:
//				{
//					s_AngleCorrect.angle_bone /= CORRECT_NUM;
//					
//					break;
//				}
//				case CORRECT_BODY:
//				{
//					s_AngleCorrect.body_x /= CORRECT_NUM;
//					s_AngleCorrect.body_y /= CORRECT_NUM;
//					break;
//				}
//				default:
//					break;
//			}
//			s_AngleCorrect.state = CORRECT_FINISH;
//			//SaveConfig(s_AngleCorrect.angle_arm,s_AngleCorrect.angle_forearm,s_AngleCorrect.angle_bone,s_AngleCorrect.body_x,s_AngleCorrect.body_y);
//		}
//	}
//	else if(s_AngleCorrect.state == CORRECT_BEGIN)
//	{
//		s_AngleCorrect.state = CORRECT_BUSY;
//		angle_cnt = 0;
//		switch(s_AngleCorrect.angle_name)
//		{
//			case CORRETC_ARM:
//			{
//				s_AngleCorrect.angle_arm = 0;
//				break;
//			}
//			case CORRECT_FOREARM:
//			{
//				s_AngleCorrect.angle_forearm = 0;
//				break;
//			}
//			case CORRECT_BONEDOG:
//			{
//				s_AngleCorrect.angle_bone = 0;
//			}
//			case CORRECT_BODY:
//			{
//				s_AngleCorrect.body_x = 0;
//				s_AngleCorrect.body_y = 0;
//			}
//			default:
//				break;
//		}
//	}
}

/*******************************
∠k = 狗骨头传感器 - 小臂传感器     
∠c = ∠m + ∠k

l_bd = l6 = sqrt(l1^2 +l4^2 - 2*l1*l4*cos(c))
∠n = ∠n1+ ∠n2
cos(n1) = (l6^2 +l3^2-l2^2)/2*l6*l3
cos(n2) = (l6^2 + l4^2 -l1^2)/2*l6*l4

∠γ = ∠m +∠n +∠b-∠180°     

x = s1*cos(a)+s2*cos(a+β)+s3*cos(a+β+γ)
y = 0
z = s1*sin(a)+ s2*sin(a+β)+s3*sin(a+β+γ)
*******************************/
void CalculateCoordinate(float arm_angle_2,float forearm_angle_2,float bone_angle_2) 
{
	#ifdef __DEBUG_MODE__
	static struct
	{
		float angle_n1;
		float angle_n2;
		float angle_c;
		float l6;
		float gama;
		float beta;
		float angle_abc;
        float s1_x;
        float s1_y;
        float s2_x;
        float s2_y;
		float s3_x;
		float s3_y;
	}debug_msg;
	#endif
	
	float rad_n1;
	float rad_n2;
	float rad_c;
	float l6; 
	float l6_sqr;
	
//	s_Calresult.rad_alpha = DEG2RAD(arm_angle);  //alpha
//	s_Calresult.rad_beta  = DEG2RAD(forearm_angle - arm_angle);   // beta
//	
//	rad_c = s_Calresult.rad_m + DEG2RAD(bone_angle - forearm_angle);   //c = m + k
	s_Calresult.rad_alpha = arm_angle_2 ;    //alpha
 	s_Calresult.rad_beta  = (forearm_angle_2 - arm_angle_2);    // beta
	
	rad_c = s_Calresult.rad_m + (bone_angle_2 - forearm_angle_2);    //c = m + k
	
	//l6 = sqrt(l1^2 +l4^2 - 2*l1*l4*cos(c));
//	l6_sqr = data_square(s_MachineMsg.l1)+data_square(s_MachineMsg.l4)-2*s_MachineMsg.l1* s_MachineMsg.l4*arm_cos_f32(rad_c);
	l6_sqr = data_square(s_MachineMsg.l1)+data_square(s_MachineMsg.l4)-2*s_MachineMsg.l1* s_MachineMsg.l4*cos(rad_c);
	l6 = sqrtf(l6_sqr);
//	printf("l6 : %f\r\n",l6);
	/*cos(n1) = (l6^2 +l3^2-l2^2)/2*l6*l3*/
	rad_n1   = acosf((l6_sqr + data_square(s_MachineMsg.l3) - data_square(s_MachineMsg.l2))/(2*s_MachineMsg.l3*l6));    
	/*cos(n2) = (l6^2 + l4^2 -l1^2)/2*l6*l4*/
	rad_n2  = acosf((l6_sqr + data_square(s_MachineMsg.l4) - data_square(s_MachineMsg.l1))/(2*s_MachineMsg.l4*l6));	
	
	s_Calresult.rad_gama = SENS_PI -( s_Calresult.rad_m + rad_n1+rad_n2 + s_Calresult.rad_b);    //∠γ = 180 - (∠m +∠n +∠b)
	
//	s_Calresult.x_coor = s_MachineMsg.s1*arm_cos_f32(s_Calresult.rad_alpha) + \
//						   s_MachineMsg.s2*arm_cos_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta)+\
//							 s_MachineMsg.s3*arm_cos_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama);
//	s_Calresult.y_coor = s_MachineMsg.s1*arm_sin_f32(s_Calresult.rad_alpha)+ \
//						   s_MachineMsg.s2*arm_sin_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta)+\
//						   s_MachineMsg.s3*arm_sin_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama);
	s_Calresult.x_coor = s_MachineMsg.s1*cos(s_Calresult.rad_alpha) + \
						   s_MachineMsg.s2*cos(s_Calresult.rad_alpha+s_Calresult.rad_beta)+\
							 s_MachineMsg.s3*cos(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama);
							 
	s_Calresult.y_coor = s_MachineMsg.s1*sin(s_Calresult.rad_alpha)+ \
						   s_MachineMsg.s2*sin(s_Calresult.rad_alpha+s_Calresult.rad_beta)+\
						   s_MachineMsg.s3*sin(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama);        //这个地方应该是s_Calresult.z_coor ？
	
	
 // printf("H, %f, s_Calresult.y, %f, rad_c: %f, l6: %f, rad_n1: %f, rad_n2: %f \r\n", s_Calresult.y_coor,
	//				 s_Calresult.x_coor,RAD2DEG(rad_c), l6, RAD2DEG(rad_n1), RAD2DEG(rad_n2));
  //	printf("SX,%f, SY,%f\r\n",s_MachineMsg.s2*cos(s_Calresult.rad_alpha+s_Calresult.rad_beta),s_MachineMsg.s2*sin(s_Calresult.rad_alpha+s_Calresult.rad_beta));
							 
//  printf("SX,%f, SY,%f\r\n", s_MachineMsg.s3*cos(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama),
//	                           s_MachineMsg.s3*sin(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama));

							 
#if __DEBUG_MODE__
	debug_msg.angle_c  = RAD2DEG(rad_c);
	debug_msg.angle_n1 = RAD2DEG(rad_n1);
	debug_msg.angle_n2 = RAD2DEG(rad_n2);
	debug_msg.beta  = RAD2DEG( s_Calresult.rad_beta);
	debug_msg.gama = RAD2DEG(s_Calresult.rad_gama);
	debug_msg.l6 = l6;
	debug_msg.angle_abc = RAD2DEG(s_Calresult.rad_alpha+s_Calresult.rad_beta+s_Calresult.rad_gama);
  debug_msg.s1_x = s_MachineMsg.s1*arm_sin_f32(s_Calresult.rad_alpha);
  debug_msg.s1_y = s_MachineMsg.s1*arm_cos_f32(s_Calresult.rad_alpha);
	debug_msg.s3_x = s_MachineMsg.s3*arm_sin_f32(s_Calresult.rad_gama);
	debug_msg.s3_y = s_MachineMsg.s3*arm_cos_f32(s_Calresult.rad_gama);
  debug_msg.s2_x = s_MachineMsg.s1*arm_cos_f32(s_Calresult.rad_alpha)+ s_MachineMsg.s2*arm_cos_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta);
  debug_msg.s2_y = s_MachineMsg.s1*arm_sin_f32(s_Calresult.rad_alpha) + s_MachineMsg.s2*arm_sin_f32(s_Calresult.rad_alpha+s_Calresult.rad_beta);
  debug_msg.s3_x = s_MachineMsg.s3*arm_cos_f32(s_Calresult.rad_alpha + s_Calresult.rad_beta+s_Calresult.rad_gama);
  debug_msg.s3_y = s_MachineMsg.s3*arm_sin_f32(s_Calresult.rad_alpha + s_Calresult.rad_beta+s_Calresult.rad_gama);
#endif
}


/*
bool SensRcvData(const  uint8_t *pframe,uint16_t canid)
{
		union
		{
			 int32_t data32;
			 uint8_t data[4];
		}dat;
		if(SINGLE_MODE != pframe[0])
		{
        return false;
    }
    dat.data[0] = pframe[1];
    dat.data[1] = pframe[2];
    dat.data[2] = pframe[3];
    dat.data[3] = pframe[4];
    switch (canid)
    {
        case DOGBINE_CANID:
        {
            s_LocalAngle.bone_angle = dat.data32;
            return true;
        }
        case FOREARM_CANID:
        {
            s_LocalAngle.forearm_angle =  dat.data32 ;
            return true;
        }
        case ARM_CANID:
        {
            s_LocalAngle.arm_angle =  dat.data32 ;
            return true;
        }
        default:
					return false;
    }	
}
*/


void MachineResultOutput(void)
{
	uint8_t TxMessage[8] = {0};
	union
	{
		 int32_t data32;
		 uint8_t data[4];
	}sens_dat;	
	
	if( DATA_HIGH == s_OutState)
	{
		sens_dat.data32 = s_MachineResult.y_distance;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3]; 
		hal.pfdcan->ext_send_message(DATA_HIGH, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_LENGTH;
	}
	if( DATA_LENGTH == s_OutState)
	{
		sens_dat.data32 = s_MachineResult.x_distanse;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3]; 
		hal.pfdcan->ext_send_message(DATA_LENGTH, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_ALPHA;
	}
	if(DATA_ANGLE_ALPHA == s_OutState)
	{		
		sens_dat.data32 = s_MachineResult.angle_alpha;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_ALPHA, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_BETA;
	}
	if(DATA_ANGLE_BETA ==  s_OutState)
	{
		sens_dat.data32 = s_MachineResult.angle_beta;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_BETA, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_GAMA;
	}
	if( DATA_ANGLE_GAMA==s_OutState)
	{
		sens_dat.data32 = s_MachineResult.angle_gamma;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_GAMA, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_ARM;
	}
	if( DATA_ANGLE_ARM == s_OutState)
	{
		sens_dat.data32 = s_MachineResult.arm_body_angle;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_ARM, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_FOREARM;
	}
	if( DATA_ANGLE_FOREARM == s_OutState)
	{
		sens_dat.data32 = s_MachineResult.forarm_body_angle;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_FOREARM, TxMessage, 8); 
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_ANGLE_BODY;
	}
	if( DATA_ANGLE_BODY == s_OutState)
	{
		sens_dat.data32 = s_MachineResult.body_x;
		TxMessage[0] = sens_dat.data[0];
		TxMessage[1] = sens_dat.data[1];
		TxMessage[2] = sens_dat.data[2];
		TxMessage[3] = sens_dat.data[3];
		sens_dat.data32 = s_MachineResult.body_y;
		TxMessage[4] = sens_dat.data[0];
		TxMessage[5] = sens_dat.data[1];
		TxMessage[6] = sens_dat.data[2];
		TxMessage[7] = sens_dat.data[3];
		hal.pfdcan->ext_send_message(DATA_ANGLE_BODY, TxMessage, 8);
//		if(CAN_NO_MB == CAN_Transmit(CAN1,&TxMessage))
//		{
//			return;
//		}
		s_OutState = DATA_OUT_IDLE;
	}
}



