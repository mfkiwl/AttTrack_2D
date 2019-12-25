
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
//���嵼��ϵΪNED������ϵΪFRD
//����Ƕȸ���
//class AngleTrackData
//{
//public:
//	double imutime;            //ʱ�������
//	double gyo3[3];            //XYZ���ݽ��ٶȣ�deg/s��ǰ����
//	double acc3[3];            //XYZ�Ӽ��߼��ٶȣ�m/s2,ǰ����
//public:
//	void Init();
//	void Rest();
//	AngleTrackData& operator=(const AngleTrackData& atdata);
//};
struct AngleTrackData
{
	double imutime;            //ʱ�������
	double gyo3[3];            //XYZ���ݽ��ٶȣ�deg/s��ǰ����
	double acc3[3];            //XYZ�Ӽ��߼��ٶȣ�m/s2,ǰ����
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





