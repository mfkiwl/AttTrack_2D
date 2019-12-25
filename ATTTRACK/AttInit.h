#ifndef _ATTINIT_H__
#define _ATTINIT_H__

/********************************************************************************************************/ 
#ifdef __cplusplus
extern "C"{
#endif
//--------------------------------------------------------------------------------------------- 	
	
/***********阈值需要实际确定********************/
//#include <algorithm>
#include "ComFunc.h"
//#include "config.h"
//using namespace std;
#define Staticbias_Wlen 600
struct AttInit
{
	//vector<double> vax;
	//vector<double> vay;
	//vector<double> vaz;
	//vector<double> vaz2;
	//vector<double> vgx;
	//vector<double> vgy;
	//vector<double> vgz;
	//vector<double> vgpsyaw;  //0-360
	//vector<double> vHorizontalAngle; //水平倾斜角
	//vector<double> vVerticalAngle;   //竖直倾斜角
	double bias_gx , bias_gy, bias_gz;
	double std_gx, std_gy, std_gz;
	double std_ax, std_ay, std_az, std_az2;
	double mean_gpsyaw, std_gpsyaw;
	double att[3];
	double tilt[2];  //水平偏转角，竖直偏转
	int bfinshinit;  //初始化成功标志，以及错误状态标志
};
//typedef AttInit AttInit_t;
int process_singleangle1(struct AttInit* aint, double gyo[3], double acc1[3]);//车体姿态零偏处理
int process_singleangle2(struct AttInit* aint, double gyo[3], double acc1[3]);//大臂姿态零偏处理
int process_singleangle3(struct AttInit* aint, double gyo[3], double acc1[3]);//小臂姿态零偏处理
int process_singleangle4(struct AttInit* aint, double gyo[3], double acc1[3]);//挖斗姿态零偏处理
void attInit(struct AttInit* aint);
//~AttInit();
//int process(double gyo[3], double acc[3], double gpsyaw, int bgps, AttTrackCfg_t* confg);
//acc1测横滚角，acc2测俯仰角
//int process(double gyo[3], double acc1[2], double acc2[2], double gpsyaw, int bgps);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//--------------------------------------------End of File---------------------------------------


