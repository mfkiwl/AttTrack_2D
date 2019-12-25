#include "AttInit.h"

 double static_acc_x_win1[Staticbias_Wlen] = { 0 };
 double static_acc_y_win1[Staticbias_Wlen] = { 0 };
 double static_acc_z_win1[Staticbias_Wlen] = { 0 };
 double static_gyro_x_win1[Staticbias_Wlen] = { 0 };
 double static_gyro_y_win1[Staticbias_Wlen] = { 0 };
static int NUM1 = 0;
 double static_acc_x_win2[Staticbias_Wlen] = { 0 };
 double static_acc_y_win2[Staticbias_Wlen] = { 0 };
 double static_acc_z_win2[Staticbias_Wlen] = { 0 };
 double gyro_y_win2[Staticbias_Wlen] = { 0 };
int NUM2 = 0;
 double static_acc_x_win3[Staticbias_Wlen] = { 0 };
 double static_acc_y_win3[Staticbias_Wlen] = { 0 };
 double static_acc_z_win3[Staticbias_Wlen] = { 0 };
 double static_gyro_y_win3[Staticbias_Wlen] = { 0 };
int NUM3 = 0;

 double static_acc_x_win4[Staticbias_Wlen] = { 0 };
 double static_acc_y_win4[Staticbias_Wlen] = { 0 };
 double static_acc_z_win4[Staticbias_Wlen] = { 0 };
 double static_gyro_y_win4[Staticbias_Wlen] = { 0 };
int NUM4 = 0;
void attInit(struct AttInit* aint)
{
	aint->bias_gx = 0;
	aint->bias_gy = 0;
	aint->bias_gz = 0;
	aint->std_gx = 0;
	aint->std_gy = 0;
	aint->std_gz = 0;
	aint->std_ax = 0;
	aint->std_ay = 0;
	aint->std_az = 0;
	aint->std_az2 = 0;
	aint->mean_gpsyaw = 0;
	aint->std_gpsyaw = 0;
	for (int i = 0; i < 3; i++)
	{
		aint->att[i] = 0;
	}
	for (int i = 0; i < 2; i++)
	{
		aint->tilt[i] = 0;
	}
	aint->bfinshinit = 0;
}


int process_singleangle1(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	double meanax = 0, meanay = 0, meanaz = 0;
	double meangx = 0;
	double meangy = 0;
	if (NUM1 < static_num)
	{
		static_acc_x_win1[NUM1] = acc1[0];//窗口初始化
		static_acc_y_win1[NUM1] = acc1[1];
		static_acc_z_win1[NUM1] = acc1[2];
		static_gyro_x_win1[NUM1] = gyo[0];//x轴零偏
		static_gyro_y_win1[NUM1] = gyo[1];//y轴零偏
		NUM1++;
		aint->bfinshinit = 0;
		return 0;
	}
	else
	{
		for (int i = 1; i < static_num - 1; i++)
		{
			meanax += static_acc_x_win1[i];
			meanay += static_acc_y_win1[i];
			meanaz += static_acc_z_win1[i];
			meangx += static_gyro_x_win1[i];
			meangy += static_gyro_y_win1[i];
		}
		aint->bias_gx = meangx / (static_num - 2);
		aint->bias_gy = meangy / (static_num - 2);
	//计算横滚和俯仰角
		aint->att[0] = atan2(-meanay, -meanaz);//横滚 (顺时为正，逆时为负)
		aint->att[1] = atan2(meanax ,-meanaz);//俯仰角(抬头为正，低头为负)
		aint->att[2] = 0;
		aint->bfinshinit = 1;
		return 1;
	}
}

int process_singleangle2(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	double meanax = 0, meanay = 0, meanaz = 0;
	double meangy = 0;
	if (NUM2 < static_num&&gyo[1]<0.005&&fabs(gyo[1])>0)
	{
		static_acc_x_win2[NUM2] = acc1[0];//窗口初始化
		static_acc_y_win2[NUM2] = acc1[1];
		static_acc_z_win2[NUM2] = acc1[2];
		gyro_y_win2[NUM2] = gyo[1];
		NUM2++;
		aint->bfinshinit = 0;
		return 0;
	}
	else if (NUM2 == static_num)
	{
		for (int i = 1; i < static_num - 1; i++)
		{
			meanax += static_acc_x_win2[i];
			meanay += static_acc_y_win2[i];
			meanaz += static_acc_z_win2[i];
			meangy += gyro_y_win2[i];
		}
		meanax /= (static_num - 2);
		meanay /= (static_num - 2);
		meanaz /= (static_num - 2);
		aint->bias_gy = meangy / (static_num - 2);
		//计算横滚和俯仰角
		aint->att[0] = atan2(-meanay , -meanaz);//横滚 (顺时为正，逆时为负)
		aint->att[1] = atan2(meanax, -meanaz);//俯仰角 (抬头为正，低头为负)
		aint->att[2] = 0;
		aint->bfinshinit = 1;
		return 1;
	}
}


int process_singleangle3(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	double meanax = 0, meanay = 0, meanaz = 0;
	double meangy = 0;
	if (NUM3 < static_num&&gyo[1]<0.05&&fabs(gyo[1])>0)
	{
		static_acc_x_win3[NUM3] = acc1[0];//窗口初始化
		static_acc_y_win3[NUM3] = acc1[1];
		static_acc_z_win3[NUM3] = acc1[2];
		static_gyro_y_win3[NUM3] = gyo[1];
		NUM3++;
		aint->bfinshinit = 0;
		return 0;
	}
	else if(NUM3 == static_num)
	{
		for (int i = 1; i < static_num - 1; i++)
		{
			meanax += static_acc_x_win3[i];
			meanay += static_acc_y_win3[i];
			meanaz += static_acc_z_win3[i];
			meangy += static_gyro_y_win3[i];
		}
		meanax /= (static_num - 2);
		meanay /= (static_num - 2);
		meanaz /= (static_num - 2);
		aint->bias_gy = meangy / (static_num - 2);
	//计算横滚和俯仰角
	aint->att[0] = atan2(-meanay, -meanaz);//横滚 (顺时为正，逆时为负)
	aint->att[1] = atan2(meanax, -meanaz);//俯仰角(抬头为正，低头为负)
	aint->att[2] = 0;	
	aint->bfinshinit = 1;
		return 1;
	}
}

int process_singleangle4(struct AttInit* aint, double gyo[3], double acc1[3])
{
	int static_num = Staticbias_Wlen;
	double meanax = 0, meanay = 0, meanaz = 0;
	double meangy = 0;
	if (NUM4 < static_num&&gyo[1]<0.005&&fabs(gyo[1])>0)
	{
		static_acc_x_win4[NUM4] = acc1[0];//窗口初始化
		static_acc_y_win4[NUM4] = acc1[1];
		static_acc_z_win4[NUM4] = acc1[2];
		static_gyro_y_win4[NUM4] = gyo[1];
		NUM4++;
		aint->bfinshinit = 0;
		return 0;
	}
	else if (NUM4 == static_num)
	{
		for (int i = 1; i < static_num - 1; i++)
		{
			meanax += static_acc_x_win4[i];
			meanay += static_acc_y_win4[i];
			meanaz += static_acc_z_win4[i];
			meangy += static_gyro_y_win4[i];
		}
		aint->bias_gy = meangy / (static_num - 2);
		meanax /= (static_num - 2);
		meanay /= (static_num - 2);
		meanaz /= (static_num - 2);
		//计算横滚和俯仰角
		aint->att[0] = atan2(-meanay, -meanaz);//横滚 (顺时为正，逆时为负)
		aint->att[1] = atan2(meanax ,-meanaz);//俯仰角(抬头为正，低头为负)
		aint->att[2] = 0;
		aint->bfinshinit = 1;
		return 1;
	}
}


