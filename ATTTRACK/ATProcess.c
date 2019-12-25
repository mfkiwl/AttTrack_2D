#include "ATProcess.h"


 static double acc_x_win1[5] = { 0 };
 static double acc_y_win1[5] = { 0 };
 static double acc_z_win1[5] = { 0 };
 static double acc_x_win2[5] = { 0 };
 static double acc_y_win2[5] = { 0 };
 static double acc_z_win2[5] = { 0 };
 static double acc_x_win3[5] = { 0 };
 static double acc_y_win3[5] = { 0 };
 static double acc_z_win3[5] = { 0 };
 static double acc_x_win4[5] = { 0 };
 static double acc_y_win4[5] = { 0 };
 static double acc_z_win4[5] = { 0 };
int NUM_acc1 = 0;
int NUM_acc2 = 0;
int NUM_acc3 = 0;
int NUM_acc4 = 0;
int extern zero_num;

/****************����������Ƕȸ���********************/
void ProcessSingleAngle(struct ATProcessSingleAngle* atp)
{
	atp->battinit = 0;
	atp->bkfinit = 0;
	atp->bprocessinit = 0;
	atp->dt = 0.02;
	atp->tpre = 0;
	atp->pitch = 0;

	atp->num_accnorm = 0;
	atp->num_gyonorm = 0;
	for (int i = 0; i < 3; i++)
	{
		atp->gyobias[i] = 0.0;
		atp->gyopre[i] = 0.0;
		atp->accpre[i] = 0.0;
	}
}

void ATPinit(struct ATProcessSingleAngle* atp)
{
	atp->battinit = 0;
	atp->bprocessinit = 0;
	atp->bkfinit = 0;
	atp->num_accnorm = 0;
	atp->pitch=0;
	atp->integ_pitch=0;
	atp->roll=0;
for(int i=0;i<3;i++)
	{
		atp->gyobias[i]=0;
	}
}
int IS203_Angle_Pro(struct AngleTrackData* iatd,struct ATProcessSingleAngle* atp)
{
		double roll_acc = 0;
		double pitch_acc = 0;
		static struct  kalmanfilter kft1p;//�����ǽṹ��
		static struct  kalmanfilter kft1r;//����ǽṹ��
		if (!atp->bkfinit)
	{
		/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
		kalmanfilterinit(&kft1r, 2,1);//����ǽṹ���ʼ
		kalmanfilterinit(&kft1p, 2,1);//�����ǽṹ���ʼ
		double dxk0[2] = { 0.1*D2R,0.1*D2R };
		setPxk(&kft1p, dxk0);
		setPxk(&kft1r, dxk0);
		double noise_imuy[2] = { 0.025*D2R,3.5*D2R / 3600.0 };//0.025,8.0(IS203-----SCC3300)
		setQk(&kft1r, noise_imuy);
		setQk(&kft1p, noise_imuy);
		atp->bkfinit = 1;
	}
		double dtime = iatd->imutime;
		double gyo_bias[3] = { 0 };
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
		Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd->acc3, 3, 1, atp->accpre);
		Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ		
#if 1
		Mmul(gyo_bias, 3, 1, dtime);
		qupdt(atp->qua, gyo_bias);
		double atttemp[3] = { 0.0 };
		q2mat_ned(atp->qua, atp->Cb2n);
		m2att_ned(atp->Cb2n, atttemp);
		atp->roll = atttemp[0];
		atp->pitch = atttemp[1];
		atp->heading = atttemp[2];
		//atp->pitch += iatd->imutime*(iatd->gyo3[1]-atp->gyobias[1]);//���ݻ���������
		//atp->roll += iatd->imutime*(iatd->gyo3[0]-atp->gyobias[0]);//���ݻ���������
#endif
		setPhi(&kft1r, dtime);
		setPhi(&kft1p, dtime);
		TUpdate(&kft1r, dtime);
		TUpdate(&kft1p, dtime);
	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/
#if ACC_SMOOTH	
	double smooth_ax = 0, smooth_ay = 0, smooth_az = 0;
		if (NUM_acc1 <WinLen)
	{
		acc_x_win1[NUM_acc1] = iatd->acc3[0];
		acc_y_win1[NUM_acc1] = iatd->acc3[1];
		acc_z_win1[NUM_acc1] = iatd->acc3[2];
	}
		if (NUM_acc1 >= WinLen)
	{
		for (int i = 0;i < WinLen;i++)
		{
			acc_x_win1[i] = acc_x_win1[i + 1];
			acc_y_win1[i] = acc_y_win1[i + 1];
			acc_z_win1[i] = acc_z_win1[i + 1];
		}
		acc_x_win1[WinLen - 1] = iatd->acc3[0];
		acc_y_win1[WinLen - 1] = iatd->acc3[1];
		acc_z_win1[WinLen - 1] = iatd->acc3[2];
		smooth_ax = dataFilter(acc_x_win1, WinLen);
		smooth_ay = dataFilter(acc_y_win1, WinLen);
		smooth_az = dataFilter(acc_z_win1, WinLen);
		 roll_acc = atan2(-smooth_ay, -smooth_az);
		 pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
	}
	//���ݹ�ʽ(����ż��ٶȴ�С���)
	/******************������Ϊ��ֵ*******************/
		double noise_roll[1] = { 1.5*D2R }, noise_pitch[1] = {1.5*D2R };
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
#endif
		
	/************���ٶȼƾ�ֵ***********/
#if Liner_Noise
		pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//���ٶȼ��㸩����
		roll_acc = atan2(-accmean[1],  -accmean[2]);//���ٶȼƼ�������
	/*******************��������̬����*************/
		double noise_pitch[1] = { 0 };
		double noise_roll[1] = { 0 };
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		if (normacc < 0.05)  //10mg
	{
		atp->num_accnorm++;
	}
		else
	{
		atp->num_accnorm = 0;
	}
	if (atp->num_accnorm >100)  //׼��̬����1��
	{
		noise_pitch[0] = 0.5*D2R;//0.5
		noise_roll[0] = 0.5*D2R;
		/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
	}
	else
	{
		noise_pitch[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
		noise_roll[0] = (0.5 + (normacc - 0.05) * 50)*D2R;
		//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
	}
	if (normacc > 0.1)
	{
		//printf("���ż��ٶȴ�\n");
//		return 1;
	}
#endif
		double zk_pitch[1] = { 0 };
		double zk_roll[1]={0};
		zk_pitch[0] = atp->pitch - pitch_acc;//����������ֵ
		zk_roll[0]=atp->roll-roll_acc;//���������ֵ
		DEG_NEG0_2PI(zk_pitch[0]);
		DEG_NEG0_2PI(zk_roll[0]);
		setzk(&kft1r, zk_roll);
		setzk(&kft1p, zk_pitch);
		setRk(&kft1r, noise_roll, 1);
		setRk(&kft1p, noise_pitch, 1);
		setHk(&kft1r, 1, 0);
		setHk(&kft1p, 1, 0);
		MUpdate(&kft1r, noise_roll, zk_roll);
		MUpdate(&kft1p, noise_pitch, zk_pitch);
		atp->roll -= kft1r.xk[0];//����ǽǶȲ���
		atp->gyobias[0] += kft1r.xk[1];//x����ƫ����
		atp->pitch -= kft1p.xk[0];//�����Ƿ���
		atp->gyobias[1] += kft1p.xk[1];//y����ƫ��������
#if BODY_DATA_OUT
	printf("acc_angle:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
	atp->roll*R2D,roll_acc*R2D,atp->pitch*R2D,pitch_acc*R2D,iatd->gyo3[0],iatd->gyo3[1],iatd->gyo3[2],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,
	zk_roll[0]*R2D,zk_pitch[0]*R2D,kft1r.xk[0],kft1r.xk[1]);
#endif
#if Debug_Save
	debug_out(2,"%f,%f,%f,%f,%f,%f,%f,%f\n", kft1r.xk[0], kft1r.xk[1], kft1p.xk[0], kft1p.xk[1],
		atp->gyobias[0] * R2D, atp->gyobias[1] * R2D, zk_roll[0] * R2D, zk_pitch[0] * R2D);
#endif
		kft1r.xk[0] = 0;//״̬����
		kft1r.xk[1] = 0;
		kft1p.xk[0] = 0;//״̬����
		kft1p.xk[1] = 0;
		NUM_acc1++;
		double atti[3] = { atp->roll,atp->pitch,atp->heading }; 
		a2mat_ned(atti, atp->Cb2n);
		m2qua_ned(atp->Cb2n, atp->qua);
		return 1;
}

/****************************��۸����Ǹ���******************************/
int Boom_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp)
{
		double pitch_acc = 0;
		static struct  kalmanfilter kft2;
		if (!atp->bkfinit)
		{
			/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
			kalmanfilterinit(&kft2, 2, 1);
			double dxk0[2] = {0.05*D2R,0.1*D2R };
			setPxk(&kft2, dxk0);
			double noise_imuy[2] = { 0.0619*D2R,2.0*D2R / 3600.0 };//0.025,8.0(GAsensor ------scc2230)
			setQk(&kft2, noise_imuy);
			atp->bkfinit = 1;
			printf("kalman init\n");
		}
		double dtime = iatd->imutime;
		double gyo_bias[3] = { 0 };
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
		Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd->acc3, 3, 1, atp->accpre);
		Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ										 
		atp->pitch += iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���ݻ���������
		atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���������ǣ��޷���������
		setPhi(&kft2, dtime);
		TUpdate(&kft2, dtime);
/*******************���ٶȼƿ���ƽ�����˳���Ƶ�ź�****************/
/************************����������ֵ****************************/
#if ACC_SMOOTH 
		double smooth_ax = 0, smooth_ay = 0, smooth_az = 0;
		if (NUM_acc2 <WinLen)
		{
			acc_x_win2[NUM_acc2] = iatd->acc3[0];
			acc_y_win2[NUM_acc2] = iatd->acc3[1];
			acc_z_win2[NUM_acc2] = iatd->acc3[2];
		}
			else
		{
			for (int i = 0;i < WinLen;i++)
			{
				acc_x_win2[i] = acc_x_win2[i + 1];
				acc_y_win2[i] = acc_y_win2[i + 1];
				acc_z_win2[i] = acc_z_win2[i + 1];
			}
			acc_x_win2[WinLen - 1] = iatd->acc3[0];
			acc_y_win2[WinLen - 1] = iatd->acc3[1];
			acc_z_win2[WinLen - 1] = iatd->acc3[2];
			smooth_ax = dataFilter(acc_x_win2, WinLen);
			smooth_ay = dataFilter(acc_y_win2, WinLen);
			smooth_az = dataFilter(acc_z_win2, WinLen);
			//pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
			pitch_acc = atan2(smooth_ax, (-smooth_az));//˫����ٶȼ��㸩����
		}
		double noise_pitch[1] = { 2.0*D2R };
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
		if (fabs(zk_pitch[0])>5)
		{
			noise_pitch[0] = 6 * D2R;
		}
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
#endif
		
/*************************���ٶ�ȡ��ֵ******************************/	
/******************������������ż��ٶ��������*********************/
#if Liner_Noise//���ٶ�ȡ��ֵ��������������̬����
	//	pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		pitch_acc = atan2(accmean[0], (-accmean[2]));
		//���ݹ�ʽ(����ż��ٶȴ�С���)
	/*******************��������̬����*************/
		double noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		if (normacc < 0.02)  //10mg
		{
			atp->num_accnorm++;
		}
		else
		{
			atp->num_accnorm = 0;
		}
		if (atp->num_accnorm > 100)  //׼��̬����1��
		{
			noise_pitch[0] = 0.5*D2R;
		}
		else
		{
			noise_pitch[0] = (0.5 + (normacc - 0.02) * 10)*D2R;
			//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
		}
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
#endif
		if (fabs(zk_pitch[0]) >PI / 4)
		{
			zk_pitch[0] = 0;
		}
		setzk(&kft2, zk_pitch);
		setRk(&kft2, noise_pitch, 1);
		setHk(&kft2, 1, 0);
		MUpdate(&kft2, noise_pitch, zk_pitch);
		atp->pitch -= kft2.xk[0];
		atp->gyobias[1] += kft2.xk[1];
#if Debug_Save
		debug_out(2, "%f, %f, %f,%f,%f,%f,%f\n", kft2.xk[0], kft2.xk[1],
			atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, zk_pitch[0] * R2D, atp->integ_pitch*R2D);
#endif
#if BOOM_DATA_OUT
		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",kft2.xk[0], kft2.xk[1],atp->gyobias[1],pitch_acc*R2D,atp->pitch*R2D,
		iatd->gyo3[1],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,atp->integ_pitch*R2D);
#endif		
		kft2.xk[0] = 0;
		kft2.xk[1] = 0;
		NUM_acc2++;
		return 1;
}


/********************************С�۸����Ǹ���**************************************/
int Stick_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp)
{
		double pitch_acc = 0;
		static struct  kalmanfilter kft3;
		if (!atp->bkfinit)
		{
			kalmanfilterinit(&kft3, 2, 1);
			double dxk0[2] = { 0.05*D2R,0.1*D2R };
			setPxk(&kft3, dxk0);
			//0.2mg/sqrt(Hz)*sqrt(41) 0.2Ϊ�������ܶ� 41Ϊ��ֹƵ�� 
			//double noise_imuy[2] = { 0.0619*2*D2R,2.0*D2R / 3600.0 };//0.025,8.0(GAsensor ------scc2230)
			double noise_imuy[2] = { 0.0619*D2R,2.0*D2R / 3600.0 };
			setQk(&kft3, noise_imuy);
			atp->bkfinit = 1;
		}
		double dtime = iatd->imutime;
		double gyo_bias[3] = { 0 };
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
		Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd->acc3, 3, 1, atp->accpre);
		Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ										 
		atp->pitch += (iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]));//���ݻ���������
		atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���������ǣ��޷���������
		setPhi(&kft3, dtime);
		TUpdate(&kft3, dtime);
/*******************���ٶȼƿ���ƽ�����˳���Ƶ�ź�****************/
/************************����������ֵ****************************/
#if ACC_SMOOTH //���ٶ�ƽ���˲�����������ȡ��ֵ
		double smooth_ax = 0, smooth_ay = 0, smooth_az = 0;
		if (NUM_acc3 <WinLen)
		{
			for (int i = 0;i < WinLen;i++)
			{
				acc_x_win3[i] = iatd->acc3[0];
				acc_y_win3[i] = iatd->acc3[1];
				acc_z_win3[i] = iatd->acc3[2];
			}
		}
		else
		{
			for (int i = 0;i < WinLen-1;i++)
			{
				acc_x_win3[i] = acc_x_win3[i + 1];
				acc_y_win3[i] = acc_y_win3[i + 1];
				acc_z_win3[i] = acc_z_win3[i + 1];
			}
			acc_x_win3[WinLen - 1] = iatd->acc3[0];
			acc_y_win3[WinLen - 1] = iatd->acc3[1];
			acc_z_win3[WinLen - 1] = iatd->acc3[2];
			smooth_ax = dataFilter(acc_x_win3, WinLen);
			smooth_ay = dataFilter(acc_y_win3, WinLen);        
			smooth_az = dataFilter(acc_z_win3, WinLen);
			//pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));//������ٶȼ��㸩����
			pitch_acc = atan2(smooth_ax, (-smooth_az));//˫����ٶȼ��㸩����
		}
		double noise_pitch[1] = {1.0*D2R };
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
		if (fabs(zk_pitch[0])>5)
		{
			noise_pitch[0] = 5 * D2R;
		}
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
#endif
/*************************���ٶ�ȡ��ֵ******************************/	
/******************������������ż��ٶ��������*********************/
#if Liner_Noise
		//pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		pitch_acc = atan2(accmean[0], (-accmean[2]));
		//���ݹ�ʽ(����ż��ٶȴ�С���)
	/*******************��������̬����*************/
		double noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		if (normacc < 0.03)  //10mg
		{
			atp->num_accnorm++;
		}
		else
		{
			atp->num_accnorm = 0;
		}
		if (atp->num_accnorm > 100)  //׼��̬����1��
		{
			//noise_pitch[0] = 0.05*D2R;
			noise_pitch[0] = 0.5*D2R;
		}
		else
		{
			noise_pitch[0] = (0.5 + (normacc - 0.03) * 10)*D2R;
			//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
		}
		if (normacc > 0.1)
		{
			//return 0;
		}
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
#endif
		setzk(&kft3, zk_pitch);
		setRk(&kft3, noise_pitch, 1);
		setHk(&kft3, 1, 0);
		MUpdate(&kft3, noise_pitch, zk_pitch);
		atp->pitch -= kft3.xk[0];
		atp->gyobias[1] += kft3.xk[1];
#if Debug_Save
		debug_out(2, "%f, %f, %f,%f,%f,%f,%f\n", kft3.xk[0], kft3.xk[1],
			atp->gyobias[1] * R2D, pitch_acc*R2D, normacc, zk_pitch[0] * R2D, atp->integ_pitch*R2D);
#endif
#if STICK_DATA_OUT
		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",kft3.xk[0], kft3.xk[1],atp->gyobias[1],pitch_acc*R2D,
		atp->pitch*R2D,iatd->gyo3[1],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,atp->integ_pitch*R2D);
#endif		
/*״̬����*/
		kft3.xk[0] = 0;
		kft3.xk[1] = 0;
		NUM_acc3++;
		return 1;
}


/*********************************�ڶ������Ǹ���****************************************/
int Dogbone_Pitch_Pro(struct AngleTrackData* iatd, struct ATProcessSingleAngle* atp)
{
		double pitch_acc = 0;
		static struct  kalmanfilter kft4;
		if (!atp->bkfinit)
		{
			/*********ˮƽ�Ƕȸ����˲�����ʼ��********/
			kalmanfilterinit(&kft4, 2, 1);
			double dxk0[2] = { 0.01*D2R,0.1*D2R };
			setPxk(&kft4, dxk0);
			//double xk0_init[2] = {atp->pitch,atp->gyobias[1]};
			//setxk(&kft4, dxk0);
			//�˲�����״̬Ϊ�Ƕ�������ƫ������������QΪ���ݽ��ٶ���������ƫ���ȶ���
			//0.004deg/s/sqrt(Hz)*sqrt(41) 0.004Ϊ�������ܶ� 41Ϊ��ֹƵ��
			//��ƫ���ȶ���8deg/h
			//0.2mg/sqrt(Hz)*sqrt(41) 0.2Ϊ�������ܶ� 41Ϊ��ֹƵ�� 
			double noise_imuy[2] = { 0.0619*2*D2R,2.0*D2R / 3600.0 };//0.0619,2.0(GAsensor ------scc2230)
			setQk(&kft4, noise_imuy);
			atp->bkfinit = 1;
		}
		double dtime = iatd->imutime;
		double gyo_bias[3] = { 0 };
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(atp->accpre, iatd->acc3, accmean, 3, 1);
		Maddn(atp->gyopre, iatd->gyo3, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd->acc3, 3, 1, atp->accpre);
		Mequalm(iatd->gyo3, 3, 1, atp->gyopre);
		Mminn(gyomean, atp->gyobias, gyo_bias, 3, 1); //��������ȡ��ֵ										 
		atp->pitch += iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���ݻ���������
		atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - atp->gyobias[1]);//���������ǣ��޷���������
		//atp->integ_pitch+= iatd->imutime*(iatd->gyo3[1] - gbias);
		setPhi(&kft4, dtime);
		TUpdate(&kft4, dtime);
/*******************���ٶȼƿ���ƽ�����˳���Ƶ�ź�****************/
/************************����������ֵ****************************/
#if ACC_SMOOTH
		double smooth_ax = 0, smooth_ay = 0, smooth_az = 0;
		if (NUM_acc4 <WinLen)
		{
			acc_x_win4[NUM_acc4] = iatd->acc3[0];
			acc_y_win4[NUM_acc4] = iatd->acc3[1];
			acc_z_win4[NUM_acc4] = iatd->acc3[2];
		}
		else
		{
			for (int i = 0;i < WinLen;i++)
			{
				acc_x_win4[i] = acc_x_win4[i + 1];
				acc_y_win4[i] = acc_y_win4[i + 1];
				acc_z_win4[i] = acc_z_win4[i + 1];
			}
			acc_x_win4[WinLen - 1] = iatd->acc3[0];
			acc_y_win4[WinLen - 1] = iatd->acc3[1];
			acc_z_win4[WinLen - 1] = iatd->acc3[2];
			smooth_ax = dataFilter(acc_x_win4, WinLen);
			smooth_ay = dataFilter(acc_y_win4, WinLen);
			smooth_az = dataFilter(acc_z_win4, WinLen);
			//pitch_acc = atan2(smooth_ax, sqrt(smooth_ay * smooth_ay + smooth_az * smooth_az));
			pitch_acc = atan2(smooth_ax, (-smooth_az));
		}
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
		double noise_pitch[1] = { 2.5*D2R };
		if (fabs(zk_pitch[0])>10)
		{
			noise_pitch[0] = 4 * D2R;
		}
		if (fabs(zk_pitch[0])>20)
		{
			noise_pitch[0] = 20 * D2R;
		}
		if (fabs(iatd->gyo3[1]) > 0.4)
		{
			noise_pitch[0] = 30 * D2R;
		}
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
#endif
	
/*************************���ٶ�ȡ��ֵ******************************/	
/******************������������ż��ٶ��������*********************/
#if Liner_Noise
		//pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//������ٶȼ��㸩����
		pitch_acc = atan2(accmean[0], (-accmean[2]));//˫��ӼƼ��㸩����
		//���ݹ�ʽ(����ż��ٶȴ�С���)
		double noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd->acc3[0] * iatd->acc3[0] + iatd->acc3[1] * iatd->acc3[1] + iatd->acc3[2] * iatd->acc3[2]) / GN - 1.0);
		if (normacc < 0.02)  //10mg
		{
			atp->num_accnorm++;
		}
		else
		{
			atp->num_accnorm = 0;
		}
		if (atp->num_accnorm > 100)  //׼��̬����1��
		{
			noise_pitch[0] = 0.5*D2R;
		}
		else
		{
			noise_pitch[0] = (0.5 + (normacc - 0.02) * 2)*D2R;
			//noise_pitch[0] = (0.5 + normacc1* 1.5)*D2R;
		}
		double zk_pitch[1] = { 0 };
		DEG_NEGPI_PI(atp->pitch);
		zk_pitch[0] = atp->pitch - pitch_acc;
		DEG_NEGPI_PI(zk_pitch[0]);
#endif
/*�������*/
		setzk(&kft4, zk_pitch);
		setRk(&kft4, noise_pitch, 1); 
		setHk(&kft4, 1, 0);
		MUpdate(&kft4, noise_pitch, zk_pitch);
		atp->pitch -= kft4.xk[0];
		atp->gyobias[1] += kft4.xk[1];
#if Debug_Save
		debug_out(2, "%f, %f, %f,%f,%f,%f,%f\n",kft4.xk[0], kft4.xk[1],
			atp->gyobias[1] * R2D,pitch_acc*R2D, normacc, zk_pitch[0] * R2D, atp->integ_pitch*R2D);
#endif
#if DOGBONE_DATA_OUT
		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",kft4.xk[0], kft4.xk[1],atp->gyobias[1],pitch_acc*R2D,atp->pitch*R2D,
		iatd->gyo3[1],iatd->acc3[0],iatd->acc3[1],iatd->acc3[2],iatd->imutime,atp->integ_pitch*R2D,iatd->imutime);
#endif
/*״̬������*/
		kft4.xk[0] = 0;
		kft4.xk[1] = 0;
		NUM_acc4++;
		return 1;
}
