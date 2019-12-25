//#pragma once
//#pragma once
#ifndef _KALMANFILTER_H__
#define _KALMANFILTER_H__

//#include "ComFunc.h"
//#include<stdlib.h>
//#include <armadillo>
#include <math.h>
/****************************************************************/
#ifdef __cplusplus
extern "C" {
#endif           	/*__cplusplus*/
/****************************************************************/	

//using namespace arma;
//using namespace std;
//class kalmanfilter
//{
//public:
//	int nox;
//	int noz;
//	//mat xk;
//	double xk[2];
//	//mat Pxk;
//	double Pxk[2][2];
//	//mat Phi;
//	 double Phi[2][2];
//	//mat Qk;
//	 double Qk[2][2];
//	//mat Hk;
//	 double Hk[1][2];
//	//mat Rk;
//	 double Rk;
//	//mat zk;
//	 double zk;
//public:
//	kalmanfilter();
//	~kalmanfilter();
//	kalmanfilter(int nrow, int ncol);
//	kalmanfilter& operator=(const kalmanfilter& kf);
//	int setxk(double xk0[]);
//	int setPxk(double dpxk0[], int scater = 1);
//	int setPhi(double dt);
//	int setPhi(double dt, int option);
//	int setQk(double Qnoise[]);
//	int setHk(int num = 1, int option = 0);
//	int setRk(double Rnoise[], int num = 1);
//	int setzk(double zki[], int unm = 1);
//	int TUpdate(double dt);
//	int MUpdate(double Rnoise[], double zki[]);             //wheel angle angular velocity
//	int MUpdate_WA_IGG3(double Rnoise[], double zki[]);       //wheel angle + wheel angle angular velocity IGG3
//};

struct kalmanfilter
{
	int nox;
	int noz;
	//mat xk;
	double xk[2];
	//mat Pxk;
	double Pxk[2][2];
	//mat Phi;
	double Phi[2][2];
	//mat Qk;
	double Qk[2][2];
	//mat Hk;
	double Hk[1][2];
	//mat Rk;
	double Rk;
	//mat zk;
	double zk;
};
//}kalmanfilter_t;

void kalmanfilterinit(struct kalmanfilter* kft, int nrow, int ncol);
//kalmanfilter(int nrow, int ncol);
//kalmanfilter& operator=(const kalmanfilter& kf);
//int setxk(struct kalmanfilter* kft);
int setxk(struct kalmanfilter* kft,double xk0[]);
int setPxk(struct kalmanfilter* kft, double dpxk0[]);
int setPhi(struct kalmanfilter* kft, double dt);
//int setPhi(kalmanfilter_t* kft,double dt, int option);
int setQk(struct kalmanfilter* kft, double Qnoise[]);
int setHk(struct kalmanfilter* kft, int num, int option);
//int setRk(struct kalmanfilter* kft, double Rnoise[]);
int setRk(struct kalmanfilter* kft, double Rnoise[], int num);
int setzk(struct kalmanfilter* kft, double zki[]);
int TUpdate(struct kalmanfilter* kft, double dt);
int MUpdate(struct kalmanfilter* kft, double Rnoise[], double zki[]);             //wheel angle angular velocity
//int MUpdate_WA_IGG3(double Rnoise[], double zki[]);       //wheel angle + wheel angle angular velocity IGG3


/****************************************************************/
#ifdef __cplutplus
}
#endif				/*__cplusplus*/
/****************************************************************/


#endif




