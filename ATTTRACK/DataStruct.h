
//#pragma once
#ifndef _DATASTRUCT_H__
#define _DATASTRUCT_H__

#include "ComFunc.h"
#include "AttTrack_lib.h"

/****************************************************************/
#ifdef __cplusplus
extern "C" {
#endif           	/*__cplusplus*/
/****************************************************************/	
//定义导航系为NED，载体系为FRD
//单轴角度跟踪
//class AngleTrackData
//{
//public:
//	double imutime;            //时间戳，秒
//	double gyo3[3];            //XYZ陀螺角速度，deg/s，前右下
//	double acc3[3];            //XYZ加计线加速度，m/s2,前右下
//public:
//	void Init();
//	void Rest();
//	AngleTrackData& operator=(const AngleTrackData& atdata);
//};
struct AngleTrackData
{
	double imutime;            //时间戳，秒
	double gyo3[3];            //XYZ陀螺角速度，deg/s，前右下
	double acc3[3];            //XYZ加计线加速度，m/s2,前右下
};
void AngleTrackData_Init(struct AngleTrackData* atd);
void AngleTrackData_Rest(struct AngleTrackData* atd);

double dataFilter(double acc_win[], int count);
void decode_gasensor(IMUdata_t* atd, struct AngleTrackData* atdata);


/****************************************************************/
#ifdef __cplutplus
}
#endif				/*__cplusplus*/
/****************************************************************/


#endif





