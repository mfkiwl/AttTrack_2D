#ifndef _ATTTRACK_LIB_H__
#define _ATTTRACK_LIB_H__
#include "stdio.h"
#define SaveAttPro
//#define SaveElePro
//#define TX62_VERSION 2019-09-18


// 惯性观测数据
struct IMUdata
{
	double imutimetarget;   //IMU输出的时间戳，秒/毫秒，100Hz，计算采样时间间隔
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //陀螺X轴输出，     deg/s
	double gyoy;            //    Y             deg/s
	double gyoz;            //    Z             deg/s
	int state_acc;          //加速度计超量程状态位
};
typedef struct IMUdata IMUdata_t;
//GNSS观测数据
struct GNSSdata
{
	double gnsstimetarget;   //GNSS观测的时间戳，周内秒，20Hz
	double lat;              //纬度, WGS84       deg
	double lon;              //经度，WGS84       deg
	double alt;              //高程，WGS84        m
	double gnssyaw;          //双天线航向，北偏东(-180 - 180)      deg
	double speed;            //地速              m/s
	double speed_ver;        //高程速度          m/s
	int state_pos;           //GNSS位置解状态
	int state_yaw;           //GNSS航向解状态
};
typedef struct GNSSdata GNSSdata_t;
//姿态跟踪观测数据
struct ATdata
{
	IMUdata_t imu;           //IMU观测数据
	GNSSdata_t gnss;         //GNSS观测数据
	int bgnss_updata;        //GNSS观测数据更新标志： 0-未更新 1-更新
};
typedef struct ATdata ATdata_t;

//上位机配置信息
struct config
{
	double gyostd_thr;        //静态判断的陀螺std阈值， deg/s
	double accstd_thr;        //          加计       ， g
	double gyo_noise_RP;      //水平角度跟踪的陀螺噪声，deg/s
	double gyo_noise_Y;       //航向角度跟踪的陀螺噪声，deg/s
	double gnssyaw_noise;     //双天线航向观测噪声，    deg/s
	int acc_noise_option;     //加速度计观测定权策略
	double leverx_R;          //右控制点的X轴杆臂，     m
	double levery_R;          //          Y             m
	double leverz_R;          //          Z             m
	double leverx_L;          //左控制点的X轴杆臂，     m
	double levery_L;          //          Y             m
	double leverz_L;          //          Z             m
};
typedef struct config config_t;
/**************挖掘机单轴角速度跟踪验证**************/
int  Body_IS203_Pro(IMUdata_t* imu,  double result[2]);//车体IS203 (3300 CRS02)
int  Boom_GA_Pro(IMUdata_t* imu, double* result);//大臂(GAsensor 2230)
int  Stick_GA_Pro(IMUdata_t* imu, double* result);//小臂(GAsensor 2230)
int  Dogbone_GA_Pro(IMUdata_t* imu, double* result);//挖斗(GAsensor 2230)

#endif
