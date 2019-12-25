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
	double dt;             //IMU�������
	double tpre;
	//NED
	double pitch;
	double roll;
	double integ_pitch;//���ּ��㸩���ǣ��޷���������
	double heading;
	double gyobias[3];
	double accpre[3];
	double gyopre[3];
	int num_accnorm;        //�Ӽ�ģֵС����ֵ����
	int num_gyonorm;        //����ģֵС����ֵ����
	double qua[4];
	double Cb2n[9];
};

//typedef ATProcessSingleAngle ATProcessSingleAngle_t;
void ProcessSingleAngle(struct ATProcessSingleAngle* atp);
void ATPinit(struct ATProcessSingleAngle* atp);
//~ATProcessSingleAngle();
//int process_gasensor(AngleTrackData& iatd); //������
int IS203_Angle_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //������̬����
int Boom_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //�����̬����
int Stick_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //С����̬����
int Dogbone_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp); //�ڶ���̬����
#endif
