#ifndef _ANGLE_CAL_H__
#define _ANGLE_CAL_H__

/********************************************************************************************************/ 
#ifdef __cplusplus
extern "C"{
#endif
//--------------------------------------------------------------------------------------------- 
//#include <stdio.h>
#include <stdint.h>	
	
//#define __DEBUG_MODE__    1


#define MAX_SENS_NUM       4

#define SENS_BUF_MAX       64
#define PASER_BUF_MAX      50
#define SENS_DATA_HEAD     '$'
#define SENS_DATA_TAIL     '\n'
#define SENS_DATA_TAIL2    '\r'
#define TIMEROUT_CNT       12  //120ms
#define SINGLE_MODE        'R'
#define SENS_PI            3.14159265358979f

#define RAD2DEG(x)         ((x)*180/SENS_PI)  //????? 
#define DEG2RAD(x)         ((x)*SENS_PI/180)  //?????
#define data_square(X)     ((X)*(X))

enum
{
	BONE_ANGLE = 1<<0,
	FOREARM_ANGLE=1<<1,
	ARM_ANGLE  = 1<<2,
};

enum
{
	CORRETC_ARM     = 0Xa0,
	CORRECT_FOREARM = 0XA1,
	CORRECT_BONEDOG = 0XA2,
	CORRECT_BODY    = 0XA3,
};

enum
{
	CORRECT_BUSY,
	CORRECT_BEGIN,
	CORRECT_FINISH,
};

enum
{
	ARM_DATA_READY    = 1<< 0,
	FORARM_DATA_READY = 1<<1,
	DOGBONE_DATA_READY     = 1<<2,
};

typedef struct s_MachineResult_t
{
	int32_t x_distanse;
	int32_t y_distance;
	int32_t round_angle;
	int32_t angle_alpha;
	int32_t angle_beta;
	int32_t angle_gamma;
	int32_t arm_body_angle;
	int32_t forarm_body_angle;
	int32_t body_x;
	int32_t body_y;
}st_MachineResult;


typedef struct s_Calresult_t
{
	float x_coor;
	float y_coor;
	float rad_gama;
	float rad_b;
	float rad_alpha;
	float rad_beta;
	float rad_m;
}st_Calresult;


typedef struct s_LocalAngle_t
{
  int32_t body_x_angle;
	int32_t body_y_angle;
	int32_t arm_angle;
	int32_t forearm_angle;
	int32_t bone_angle;
}st_LocalAngle;  

/*-----------------------------------------------


-----------------------------------------------*/

//单位mm
typedef struct s_MachineMsg_t
{
	float s1;	      // 大臂长度
	float s2;		    // 小臂长度
	float s3;       // 铲斗长度
	float l1; 	    // 狗骨头长度
	float l5;       // 假小臂长度
	float DGlen;		// 挖斗下插销与斗尖投影之间的距离
	float l3;       //挖斗上的两个销子长度
	float l2;       //小臂对边长度
	float l4;       //狗骨头销子到铲斗销子的长距离
}st_MachineMsg;

typedef struct s_AngleCorrect_t
{
	uint8_t angle_name;
	uint8_t state;  //true ,busy ;false ready
	int32_t angle_arm;
	int32_t angle_forearm;
	int32_t angle_bone;
	int32_t body_x;
	int32_t body_y;
}st_AngleCorrect ;

enum
{
    DATA_OUT_IDLE      = 0,
    DATA_HIGH 	       = 0x1C070283,
    DATA_LENGTH 	   = 0X1C070383,
    DATA_ANGLE_ALPHA   = 0X1C070403,
    DATA_ANGLE_BETA    = 0X1C070483,
    DATA_ANGLE_GAMA    = 0X1C070503,
    DATA_ANGLE_ARM     = 0X1C070583,
    DATA_ANGLE_FOREARM = 0X1C070603,
    DATA_ANGLE_BODY    = 0X1C070683,
};


//----------------------------------------------------------------------------------------------
void GetMachineAngle(double *body_angle, double arm_angle, double forearm_angle, double bone_angle);
	
void MachineInit(void);	
	
void MachineResultOutput(void);





#ifdef __cplusplus
} // extern "C"
#endif

#endif

//--------------------------------------------End of File---------------------------------------


