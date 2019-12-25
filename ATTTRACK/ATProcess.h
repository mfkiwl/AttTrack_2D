#ifndef _ATPROCESS_H__
#define _ATPROCESS_H__
#include "AttInit.h"
#include "DataStruct.h"
#include "ComFunc.h"
#include "kalmanfilter.h"
#include <stdbool.h>
#define BODY_DATA_OUT 0
#define BOOM_DATA_OUT 0
#define STICK_DATA_OUT 0
#define DOGBONE_DATA_OUT 0
#define ACC_SMOOTH 1
#define Liner_Noise 0
#define Debug_Save 0
#define DEG_NEGPI_PI(x)  {if ((x) > PI) (x) -= 2*PI;    else if ((x) < -PI) (x) += 2*PI; }
#define DEG_NEG0_2PI(x)  {if ((x) > 2*PI) (x) -= 2*PI;    else if ((x) < -2*PI) (x) += 2*PI; }
struct ATProcessSingleAngle
{
	bool battinit;
	bool bkfinit;
	bool bprocessinit;
//	int battinit;
//	int bkfinit;
//	int bprocessinit;
	double dt;             //IMU采样间隔
	double tpre;
	//NED
	double pitch;
	double roll;
	double integ_pitch;//积分计算俯仰角（无反馈修正）
	double heading;
	double gyobias[3];
	double accpre[3];
	double gyopre[3];
	int num_accnorm;        //加计模值小于阈值计数
	int num_gyonorm;        //陀螺模值小于阈值计数
	double qua[4];
	double Cb2n[9];
};

//typedef ATProcessSingleAngle ATProcessSingleAngle_t;
void ProcessSingleAngle(struct ATProcessSingleAngle* atp);
void ATPinit(struct ATProcessSingleAngle* atp);
//~ATProcessSingleAngle();
//int process_gasensor(AngleTrackData& iatd); //推土机
int IS203_Angle_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //车体姿态处理
int Boom_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //大臂姿态处理
int Stick_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //小臂姿态处理
int Dogbone_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //挖斗姿态处理
#endif
