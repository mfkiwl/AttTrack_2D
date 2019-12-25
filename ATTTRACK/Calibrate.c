#include "Calibrate.h"
#include "ComFunc.h"
#include <math.h>
#include <stdint.h>

//Cal_Install_Error::Cal_Install_Error(int len)
//{
//	winlen = len;
//	winlen_gnss = len;
//	max1 = may1 = maz1 = max2 = may2 = maz2 = 0;
//	sax1 = say1 = saz1 = sax2 = say2 = saz2 = GN;
//	sumheading_A2B=sumheading_B2A = 0;
//	installangle[0] = 0;
//	installangle[1] = 0;
//	installangle[2] = 0;
//}
//int static cal_num = 0;
//double static ax1[Winlen] = { 0 };
//double static ay1[Winlen] = { 0 };
//double static az1[Winlen] = { 0 };
//double static ax2[Winlen] = { 0 };
//double static ay2[Winlen] = { 0 };
//double static az2[Winlen] = { 0 };
static uint16_t cal_num = 0;
//static double ax1[Winlen] = { 0 };
//static double ay1[Winlen] = { 0 };
//static double az1[Winlen] = { 0 };
//static double ax2[Winlen] = { 0 };
//static double ay2[Winlen] = { 0 };
//static double az2[Winlen] = { 0 };

double ax1[Winlen] = { 0 };
double ay1[Winlen] = { 0 };
double az1[Winlen] = { 0 };
double ax2[Winlen] = { 0 };
double ay2[Winlen] = { 0 };
double az2[Winlen] = { 0 };

int calinstallerr(struct AngleTrackData* atdata, struct Cal_Install_Error* Cal_err, int option)
{
	if (option == 1)
	{
		if (cal_num < Winlen)
		{
			ax1[cal_num] = atdata->acc3[0];
			ay1[cal_num] = atdata->acc3[1];
			az1[cal_num] = atdata->acc3[2];
			cal_num++;
		}
		else
		{
			Cal_err->max1 = GetAveStd(ax1, Winlen, 0);
			Cal_err->sax1 = GetAveStd(ax1, Winlen, 1);
			Cal_err->may1 = GetAveStd(ay1, Winlen, 0);
			Cal_err->say1 = GetAveStd(ay1, Winlen, 1);
			Cal_err->maz1 = GetAveStd(az1, Winlen, 0);
			Cal_err->saz1 = GetAveStd(az1, Winlen, 1);
			if (Cal_err->sax1 < 0.01*GN && Cal_err->say1 < 0.01*GN && Cal_err->saz1 < 0.01*GN)
			{
				cal_num = 0;//计数清零
				return 3;  //第一位置采样完成
			}
//			else
//			{
//				//return 2;  //第一位置std超限
//			}
		}
	}
	if (option == 2)
	{
		if (cal_num < Winlen)
		{
			ax2[cal_num] = atdata->acc3[0];
			ay2[cal_num] = atdata->acc3[1];
			az2[cal_num] = atdata->acc3[2];
			cal_num++;
		}
		else
		{
			Cal_err->max2 = GetAveStd(ax2, Winlen, 0);
			Cal_err->sax2 = GetAveStd(ax2, Winlen, 1);
			Cal_err->may2 = GetAveStd(ay2, Winlen, 0);
			Cal_err->say2 = GetAveStd(ay2, Winlen, 1);
			Cal_err->maz2 = GetAveStd(az2, Winlen, 0);
			Cal_err->saz2 = GetAveStd(az2, Winlen, 1);
			if (Cal_err->sax2 < 0.01*GN && Cal_err->say2 < 0.01*GN && Cal_err->saz2 < 0.01*GN)
			{
				cal_num = 0;//计数清零
				return 6;  //第二位置采样完成
			}
//			else
//			{
//				return 5;  //第二位置std超限
//			}
		}
	}
	if (option == 3)
	{
		double roll = asin((Cal_err->max1 - Cal_err->max2) / (2 * GN));
		double pitch = asin((Cal_err->may1 - Cal_err->may2) / (2 * GN));
		Cal_err->installangle[0] = roll;
		Cal_err->installangle[1] = pitch;
		return 7;  //安装误差计算完成 
	}
	
	return 0;
}




void Comp_InstallErr_Acc_IS203(double acc[3], double installroll, double installpitch)
{
	double cphi = cos(installroll); double sphi = sin(installroll);
	double cthe = cos(installpitch); double sthe = sin(installpitch);
	//calculate levering matrix
	double C3[9] = { 0 }, C2[9] = { 0 };
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi;  C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	C2[0 * 3 + 0] = cthe;                        C2[0 * 3 + 2] = -sthe;
	                       C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                        C2[2 * 3 + 2] = cthe;

	double Clevel[3 * 3] = { 0.0 }, Ctemp[3 * 3] = { 0.0 };
	Mmulnm(C3, C2, 3, 3, 3, Ctemp);
	Mtn(Ctemp, 3, 3, Clevel);

	double acc_comp[3] = { 0.0 };
	Mmulnm(Clevel, acc, 3, 3, 1, acc_comp);

	Mequalm(acc_comp, 3, 1, acc);
}

void Comp_InstallErr_Acc_IS201(double acc[3], double installroll, double installpitch)
{
	double cphi = cos(installroll); double sphi = sin(installroll);
	double cthe = cos(installpitch); double sthe = sin(installpitch);
	//calculate levering matrix
	double C1[9] = { 0 };
	C1[0 * 3 + 0] = cthe;	C1[0 * 3 + 1] = sthe;
	C1[1 * 3 + 0] = -sthe;  C1[1 * 3 + 1] =cthe;
 C1[2 * 3 + 2] = 1;
	

	double Clevel[3 * 3] = { 0.0 };
	Mtn(C1, 3, 3, Clevel);

	double acc_comp[3] = { 0.0 };
	Mmulnm(Clevel, acc, 3, 3, 1, acc_comp);

	Mequalm(acc_comp, 3, 1, acc);
}





void calinstallerr_init(struct Cal_Install_Error* Cal_err)
{
	Cal_err->installangle[0] = 0;
	Cal_err->installangle[1] = 0;
	Cal_err->max1 = 0;
	Cal_err->may1 = 0;
	Cal_err->maz1 = 0;
	Cal_err->sax1 = 0;
	Cal_err->say1 = 0;
	Cal_err->saz1 = 0;
	Cal_err->max2 = 0;
	Cal_err->may2 = 0;
	Cal_err->maz2 = 0;
	Cal_err->sax2 = 0;
	Cal_err->say2 = 0;
	Cal_err->saz2 = 0;

}


 double GetAveStd(double acc_cal[], int acc_num,int opt)
{
	int n = acc_num;
	double avg = 0.0, std = 0.0, rms = 0.0, sum = 0.0;

	if (n == 0) return 99999.9;

	for (int i = 0; i<n; i++) {
		sum += acc_cal[i];
		rms += acc_cal[i] * acc_cal[i];
	}
	avg = sum / n;

	if (opt == 0) return avg;

	sum = 0.0;
	for (int i = 0; i<n; i++) {
		sum += (acc_cal[i] - avg)*(acc_cal[i] - avg);
	}

	std = sqrt(sum / (n - 1));
	rms = sqrt(rms / (n));

	if (opt == 1) return std;
	else if (opt == 2) return rms;

	return 0.0;
}



