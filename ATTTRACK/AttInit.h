#ifndef _ATTINIT_H__
#define _ATTINIT_H__

/********************************************************************************************************/ 
#ifdef __cplusplus
extern "C"{
#endif
//--------------------------------------------------------------------------------------------- 	
	
/***********��ֵ��Ҫʵ��ȷ��********************/
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
	//vector<double> vHorizontalAngle; //ˮƽ��б��
	//vector<double> vVerticalAngle;   //��ֱ��б��
	double bias_gx , bias_gy, bias_gz;
	double std_gx, std_gy, std_gz;
	double std_ax, std_ay, std_az, std_az2;
	double mean_gpsyaw, std_gpsyaw;
	double att[3];
	double tilt[2];  //ˮƽƫת�ǣ���ֱƫת
	int bfinshinit;  //��ʼ���ɹ���־���Լ�����״̬��־
};
//typedef AttInit AttInit_t;
int process_singleangle1(struct AttInit* aint, double gyo[3], double acc1[3]);//������̬��ƫ����
int process_singleangle2(struct AttInit* aint, double gyo[3], double acc1[3]);//�����̬��ƫ����
int process_singleangle3(struct AttInit* aint, double gyo[3], double acc1[3]);//С����̬��ƫ����
int process_singleangle4(struct AttInit* aint, double gyo[3], double acc1[3]);//�ڶ���̬��ƫ����
void attInit(struct AttInit* aint);
//~AttInit();
//int process(double gyo[3], double acc[3], double gpsyaw, int bgps, AttTrackCfg_t* confg);
//acc1�����ǣ�acc2�⸩����
//int process(double gyo[3], double acc1[2], double acc2[2], double gpsyaw, int bgps);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//--------------------------------------------End of File---------------------------------------


