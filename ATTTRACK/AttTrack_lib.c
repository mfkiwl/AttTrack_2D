#include <stdio.h>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
int install_flag = 0;
static int install_num = 0;
static int install_delay = 0;
int zero_num = 0;
static double pitch_pre2=0,pitch_pre3=0,pitch_pre4=0;

int Body_IS203_Pro(IMUdata_t* imu, double result[2])
{
	static struct  AngleTrackData iatd1;
	static struct  ATProcessSingleAngle atp1;
	static struct  AttInit aint1;
	static struct  Cal_Install_Error Cal_t1;
	if (!atp1.bprocessinit)
	{
#if Debug_Save
		char outfilepath[] = "E:\\result\\";
		debug_file_open(outfilepath);
#endif
		ATPinit(& atp1);//中间解码结构体初始化
		attInit(&aint1);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd1);//数据结构体初始化
		atp1.bprocessinit = 1;
	}
	decode_gasensor(imu, &iatd1);
	if (!atp1.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle1(&aint1, iatd1.gyo3, iatd1.acc3);
		if (finshattinit == 1)
		{
			atp1.roll = aint1.att[0];//车体横滚角
			atp1.pitch = aint1.att[1];//车体俯仰角
			atp1.gyobias[0] = aint1.bias_gx;//x轴静态零偏
			atp1.gyobias[1] = aint1.bias_gy;//y轴静态零偏
			a2mat_ned(aint1.att, atp1.Cb2n);
			m2qua_ned(atp1.Cb2n, atp1.qua);
			atp1.battinit = 1;
		};
	}
	if (atp1.battinit==1)
	{
		IS203_Angle_Pro(&iatd1,&atp1);
	}
	//结果结构体赋值
	result[0] = atp1.roll;
	result[1] = atp1.pitch;
	AngleTrackData_Rest(&iatd1);
	return 1;
}


int Boom_GA_Pro(IMUdata_t* imu, double* result)
{
	static struct  AngleTrackData iatd2;
	static struct  ATProcessSingleAngle atp2;
	static struct  AttInit aint2;
	static struct  Cal_Install_Error Cal_t2;
	if (!atp2.bprocessinit)
	{
#if Debug_Save
		char outfilepath[] = "E:\\result\\";
		debug_file_open(outfilepath);
#endif
		ATPinit(&atp2);//中间解码结构体初始化
		attInit(&aint2);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd2);//数据结构体初始化
		atp2.bprocessinit = 1;
	}
	decode_gasensor(imu, &iatd2);
	if (!atp2.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle2(&aint2, iatd2.gyo3, iatd2.acc3);
		if (finshattinit == 1)
		{
			atp2.pitch = aint2.att[1];
			atp2.integ_pitch=aint2.att[1];
			atp2.gyobias[1] = aint2.bias_gy;
			atp2.tpre = iatd2.imutime - atp2.dt;
			Mequalm(iatd2.acc3, 3, 1, atp2.accpre);
			Mequalm(iatd2.gyo3, 3, 1, atp2.gyopre);
			atp2.battinit = 1;
		}
	}
	if (atp2.battinit == 1)
	{
		if(imu->state_acc==1)//加速度超量程
		{
			atp2.pitch=pitch_pre2;
		}
		else
		{
		Boom_Pitch_Pro(&iatd2, &atp2);
		}
	}
	//结果结构体赋值
	*result = atp2.pitch;
	pitch_pre2=atp2.pitch;
	AngleTrackData_Rest(&iatd2);
	return 1;
}


int Stick_GA_Pro(IMUdata_t* imu, double* result)
{
	static struct  AngleTrackData iatd3;
	static struct  ATProcessSingleAngle atp3;
	static struct  AttInit aint3;
	static struct  Cal_Install_Error Cal_t3;
	if (!atp3.bprocessinit)
	{
#if Debug_Save
		char outfilepath[] = "E:\\result\\";
		debug_file_open(outfilepath);
#endif
		ATPinit(&atp3);//中间解码结构体初始化
		attInit(&aint3);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd3);//数据结构体初始化
		atp3.bprocessinit = 1;
	}
	decode_gasensor(imu, &iatd3);
	if (!atp3.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle3(&aint3, iatd3.gyo3, iatd3.acc3);
		if (finshattinit == 1)
		{
			atp3.pitch = aint3.att[1];
			atp3.integ_pitch=aint3.att[1];
			atp3.gyobias[1] = aint3.bias_gy;
			atp3.tpre = iatd3.imutime - atp3.dt;
			Mequalm(iatd3.acc3, 3, 1, atp3.accpre);
			Mequalm(iatd3.gyo3, 3, 1, atp3.gyopre);
			atp3.battinit = 1;
		}
	}
	if (atp3.battinit == 1)
	{
		if(imu->state_acc==1)//加速度超量程
		{
			atp3.pitch=pitch_pre3;
		}
		else
		{
		Stick_Pitch_Pro(&iatd3, &atp3);
		}
	}
	//结果结构体赋值
	*result = atp3.pitch;
	pitch_pre3=atp3.pitch;
	AngleTrackData_Rest(&iatd3);
	return 1;
}

int Dogbone_GA_Pro(IMUdata_t* imu, double* result)
{
	static struct  AngleTrackData iatd4;
	static struct  ATProcessSingleAngle atp4;
	static struct  AttInit aint4;
	static struct  Cal_Install_Error Cal_t4;
	if (!atp4.bprocessinit)
	{
#if Debug_Save
		char outfilepath[] = "E:\\result\\";
		debug_file_open(outfilepath);
#endif
		ATPinit(&atp4);//中间解码结构体初始化
		attInit(&aint4);//静态零偏结构体初始化
		AngleTrackData_Init(&iatd4);//数据结构体初始化
		atp4.bprocessinit = 1;
	}
	decode_gasensor(imu, &iatd4);
	if (!atp4.battinit)
	{
		int finshattinit = 0;
		finshattinit = process_singleangle4(&aint4, iatd4.gyo3, iatd4.acc3);
		if (finshattinit == 1)
		{
			atp4.pitch = aint4.att[1];
			atp4.integ_pitch=aint4.att[1];
			atp4.gyobias[1] = aint4.bias_gy;
			atp4.tpre = iatd4.imutime - atp4.dt;
			Mequalm(iatd4.acc3, 3, 1, atp4.accpre);
			Mequalm(iatd4.gyo3, 3, 1, atp4.gyopre);
			atp4.battinit = 1;
		}
	}
	if (atp4.battinit == 1)
	{
		if(imu->state_acc==1)//加速度超量程
		{
			atp4.pitch=pitch_pre4;//当前俯仰角等于上一时刻俯仰角
		}
		else
		{
		Dogbone_Pitch_Pro(&iatd4, &atp4);
		}
	}
	//结果结构体赋值
	*result = atp4.pitch;
	pitch_pre4=atp4.pitch;
	AngleTrackData_Rest(&iatd4);
	return 1;
}