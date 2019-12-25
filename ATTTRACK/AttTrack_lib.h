#ifndef _ATTTRACK_LIB_H__
#define _ATTTRACK_LIB_H__
#include "stdio.h"
#define SaveAttPro
//#define SaveElePro
//#define TX62_VERSION 2019-09-18


// ���Թ۲�����
struct IMUdata
{
	double imutimetarget;   //IMU�����ʱ�������/���룬100Hz���������ʱ����
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //����X�������     deg/s
	double gyoy;            //    Y             deg/s
	double gyoz;            //    Z             deg/s
	int state_acc;          //���ٶȼƳ�����״̬λ
};
typedef struct IMUdata IMUdata_t;
//GNSS�۲�����
struct GNSSdata
{
	double gnsstimetarget;   //GNSS�۲��ʱ����������룬20Hz
	double lat;              //γ��, WGS84       deg
	double lon;              //���ȣ�WGS84       deg
	double alt;              //�̣߳�WGS84        m
	double gnssyaw;          //˫���ߺ��򣬱�ƫ��(-180 - 180)      deg
	double speed;            //����              m/s
	double speed_ver;        //�߳��ٶ�          m/s
	int state_pos;           //GNSSλ�ý�״̬
	int state_yaw;           //GNSS�����״̬
};
typedef struct GNSSdata GNSSdata_t;
//��̬���ٹ۲�����
struct ATdata
{
	IMUdata_t imu;           //IMU�۲�����
	GNSSdata_t gnss;         //GNSS�۲�����
	int bgnss_updata;        //GNSS�۲����ݸ��±�־�� 0-δ���� 1-����
};
typedef struct ATdata ATdata_t;

//��λ��������Ϣ
struct config
{
	double gyostd_thr;        //��̬�жϵ�����std��ֵ�� deg/s
	double accstd_thr;        //          �Ӽ�       �� g
	double gyo_noise_RP;      //ˮƽ�Ƕȸ��ٵ�����������deg/s
	double gyo_noise_Y;       //����Ƕȸ��ٵ�����������deg/s
	double gnssyaw_noise;     //˫���ߺ���۲�������    deg/s
	int acc_noise_option;     //���ٶȼƹ۲ⶨȨ����
	double leverx_R;          //�ҿ��Ƶ��X��˱ۣ�     m
	double levery_R;          //          Y             m
	double leverz_R;          //          Z             m
	double leverx_L;          //����Ƶ��X��˱ۣ�     m
	double levery_L;          //          Y             m
	double leverz_L;          //          Z             m
};
typedef struct config config_t;
/**************�ھ��������ٶȸ�����֤**************/
int  Body_IS203_Pro(IMUdata_t* imu,  double result[2]);//����IS203 (3300 CRS02)
int  Boom_GA_Pro(IMUdata_t* imu, double* result);//���(GAsensor 2230)
int  Stick_GA_Pro(IMUdata_t* imu, double* result);//С��(GAsensor 2230)
int  Dogbone_GA_Pro(IMUdata_t* imu, double* result);//�ڶ�(GAsensor 2230)

#endif
