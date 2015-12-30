#pragma once
#ifndef __LI_IMU_L__
#define __LI_IMU_L__

#include<math.h>
#include <windows.h>
#include <cv.h>
#include <highgui.h>


#define G0L 9.8
#define PI_L 3.1415926

#define Kp_L 1.800f    //误差增益    HARS              
#define Ki_L 0.002f  

int CDU_P[5];
int CDU_R[5];
int CDU_Y[5];

int CDU_P_Reg[5][3];

typedef struct _AttitudeL_
{
	float q[4];//传感器坐标系四元数
	float qb[4];//传感器坐标系用于平滑保存的四元数
	float QGL[4];//OpenGL坐标系，可用于OpenGL显示。

	float ax,ay,az;//传感器重力场
	float gx,gy,gz;//传感器陀螺仪
	float mx,my,mz;//传感器磁场
	float offset_mx,offset_my,offset_mz;//传感器拟合磁场偏差

	float exInt,eyInt,ezInt;//传感器坐标系四元数姿态更新累积误差
	float Yaw,Pitch,Rool;   //传感器坐标系 航向角、俯仰角、翻滚角，单位：弧度。
	int Yaw_angle,Pitch_angle,Rool_angle;//传感器坐标系 航向角、俯仰角、翻滚角，单位：度。
	int Yaw_angle_Offset;                //传感器坐标系航向角矫正偏差

	float Yaw_GL,Pitch_GL,Rool_GL;//OpenGL坐标系，航向角、俯仰角、翻滚角，单位：弧度
	int Yaw_angle_GL,Pitch_angle_GL,Rool_angle_GL;//OpenGL坐标系，航向角、俯仰角、翻滚角，单位：度

	float Yaw_Local_GL,Pitch_Local_GL,Rool_Local_GL;//OpenGL 局部坐标系，航向角、俯仰角、翻滚角，单位：弧度
	int Yaw_Local_angle_GL,Pitch_Local_angle_GL,Rool_Local_angle_GL;//OpenGL 局部坐标系，航向角、俯仰角、翻滚角，单位：度

	float Hx[3];
	float Hy[3];
	
}AttitudeL;

AttitudeL ATL0,ATL1,ATL2,ATL3,ATL4,ATL5;

typedef struct _GL_QuaterB_ //四元数简易 结构体
{
	float q0;
	float q1;
	float q2;
	float q3;
}GL_QuaterB;


/*********************单个四元数初始化********************/
void Element_4_intL(AttitudeL &ATLN)
{
	//----------------------------------------
	ATLN.ax=0;
	ATLN.ay=0;
	ATLN.az=G0L;

	ATLN.gx=0;
	ATLN.gy=0;
	ATLN.gz=0;

	ATLN.mx=0;
	ATLN.my=0;
	ATLN.mz=0;

	ATLN.exInt=0;
	ATLN.eyInt=0;
	ATLN.ezInt=0;

	ATLN.Pitch=0;
	ATLN.Rool=0;
	ATLN.Yaw=0;

	ATLN.Pitch_angle=0;
	ATLN.Rool_angle=0;
	ATLN.Yaw_angle=0;

	//----------------------------------------

	float acx=1;
	float acy=0;
	float acz=9.8;
	float a=0;
	

	float rot_xx= atan2(acy,acz);//弧度 单位
	float rot_yy= -asin(acx/G0L);

	float rot_zz= a;
	//****** 四元数初始化 ******//
	ATLN.q[0] = cos(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[1] = sin(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)-cos(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[2] = cos(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[3] = cos(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2)-sin(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2);

	ATLN.qb[0]=ATLN.q[0];
	ATLN.qb[1]=ATLN.q[1];
	ATLN.qb[2]=ATLN.q[2];
	ATLN.qb[3]=ATLN.q[3];

	ATLN.QGL[0]=1;
	ATLN.QGL[1]=0;
	ATLN.QGL[2]=0;
	ATLN.QGL[3]=0;

	for(int i=0;i<3;i++)
	{
		ATLN.Hx[i]=1;
		ATLN.Hy[i]=1;
	}
	
}

/*********************多个四元数初始化********************/
void Element_intL()
{
	Element_4_intL(ATL0);
	Element_4_intL(ATL1);
	Element_4_intL(ATL2);
	Element_4_intL(ATL3);
	Element_4_intL(ATL4);
	Element_4_intL(ATL5);
#if 1

	// 108.6145  299.2770    5.4395
	ATL0.offset_mx= 108.3132;
	ATL0.offset_my=     298.8433 ;
	ATL0.offset_mz=     6.8748;
	ATL0.Yaw_angle_Offset=0;
	

	ATL1.offset_mx= 92.3286;
	ATL1.offset_my=     119.2020;
	ATL1.offset_mz=     18.6367;
	ATL1.Yaw_angle_Offset=0;

	ATL2.offset_mx=-50.6673;
	ATL2.offset_my=  220.0930;
	ATL2.offset_mz=  -266.3655;
	ATL2.Yaw_angle_Offset=0;

	ATL3.offset_mx=  156.4907;
	ATL3.offset_my=    203.4193;
	ATL3.offset_mz=    -201.5305;
	ATL3.Yaw_angle_Offset=0;

	ATL4.offset_mx=  304.2780;
	ATL4.offset_my=     277.2242;
	ATL4.offset_mz=        -64.2688;
	ATL4.Yaw_angle_Offset=0;

	ATL5.offset_mx=  45.1969;
	ATL5.offset_my=     297.2519;
	ATL5.offset_mz=  -146.9489;
	ATL5.Yaw_angle_Offset=0;
#else //------------------------Home
	ATL0.offset_mx=-7.8289;
	ATL0.offset_my=   193.4118;
	ATL0.offset_mz=  -200.1012;
	ATL0.Yaw_angle_Offset=0;

	ATL1.offset_mx=156.3892 ;
	ATL1.offset_my=  161.3432;
	ATL1.offset_mz=  -179.9610;
	ATL1.Yaw_angle_Offset=0;

	ATL2.offset_mx=86.8705   ;
	ATL2.offset_my=  46.5035;
	ATL2.offset_mz=  -313.9411;
	ATL2.Yaw_angle_Offset=0;

	ATL3.offset_mx=-30.5149;
	ATL3.offset_my=    114.4771 ;
	ATL3.offset_mz= -108.4904;
	ATL3.Yaw_angle_Offset=0;

	ATL4.offset_mx=8.4941;
	ATL4.offset_my=  109.5580 ;
	ATL4.offset_mz= -73.3547;
	ATL4.Yaw_angle_Offset=0;

	ATL5.offset_mx=-70.3524;
	ATL5.offset_my=   16.1663;
	ATL5.offset_mz= -108.7213;
	ATL5.Yaw_angle_Offset=0;
#endif
}

/*********************************************************/


/******************************************/
void OlA_T_Quarter_L(AttitudeL &ATLN)
{
	int Rool_angleN=ATLN.Rool_angle;
	int Pitch_angleN=ATLN.Pitch_angle;
	int Yaw_angleN=ATLN.Yaw_angle;

	float p= float(Pitch_angleN)*3.1415926/180/2;//弧度 单位
	float r= float(Rool_angleN)*3.1415926/180/2;
	float y= float(Yaw_angleN)*3.1415926/180/2;

	ATLN.q[0]=cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);
	ATLN.q[1]=sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);
	ATLN.q[2]=cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);
	ATLN.q[3]=cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);
}
/********************传感器坐标系 四元数转欧拉角 形式1**********************/
void Quarter_T_OlA_L(AttitudeL &ATLN)
{
	float qq0=ATLN.q[0];
	float qq1=ATLN.q[1];
	float qq2=ATLN.q[2];
	float qq3=ATLN.q[3];
	////方向余弦矩阵寄存器
	//更新方向余弦矩阵
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;
	t11=qq0*qq0+qq1*qq1-qq2*qq2-qq3*qq3;
	t12=2.0*(qq1*qq2+qq0*qq3);
	t13=2.0*(qq1*qq3-qq0*qq2);
	t21=2.0*(qq1*qq2-qq0*qq3);
	t22=qq0*qq0-qq1*qq1+qq2*qq2-qq3*qq3;
	t23=2.0*(qq2*qq3+qq0*qq1);
	t31=2.0*(qq1*qq3+qq0*qq2);
	t32=2.0*(qq2*qq3-qq0*qq1);
	t33=qq0*qq0-qq1*qq1-qq2*qq2+qq3*qq3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	float Rool2 = atan2(t23,t33);
	float Pitch2 = -asin(t13);
	float Yaw2 = atan2(t12,t11);

	if (Yaw2 < 0) Yaw2 += 360/180*3.1415926;

	//Yaw += 360/180*PI;
	//	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;

	//--- 弧度转为角度
	int Rool_angle2 = Rool2/3.1415926*180;
	int Pitch_angle2 = Pitch2/3.1415926*180;
	int Yaw_angle2 = Yaw2/3.1415926*180;

	Yaw_angle2=Yaw_angle2%360;

	ATLN.Rool=Rool2;
	ATLN.Pitch=Pitch2;
	ATLN.Yaw=Yaw2;

	if (Yaw_angle2 < 0) Yaw_angle2 += 360;

	Yaw_angle2 = Yaw2/3.1415926*180;

	ATLN.Rool_angle=Rool_angle2;
	ATLN.Pitch_angle=Pitch_angle2;
	ATLN.Yaw_angle=Yaw_angle2;
}

/********************传感器坐标系 四元数转欧拉角 形式2**********************/
void Quarter_T_OlA2_L(AttitudeL &ATLN,float qq0,float qq1,float qq2,float qq3)
{
	////方向余弦矩阵寄存器
	//更新方向余弦矩阵
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;
	t11=qq0*qq0+qq1*qq1-qq2*qq2-qq3*qq3;
	t12=2.0*(qq1*qq2+qq0*qq3);
	t13=2.0*(qq1*qq3-qq0*qq2);
	t21=2.0*(qq1*qq2-qq0*qq3);
	t22=qq0*qq0-qq1*qq1+qq2*qq2-qq3*qq3;
	t23=2.0*(qq2*qq3+qq0*qq1);
	t31=2.0*(qq1*qq3+qq0*qq2);
	t32=2.0*(qq2*qq3-qq0*qq1);
	t33=qq0*qq0-qq1*qq1-qq2*qq2+qq3*qq3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	ATLN.Rool = atan2(t23,t33);
	ATLN.Pitch = -asin(t13);
	
		ATLN.Yaw = atan2(t12,t11);

		if (ATLN.Yaw < 0) ATLN.Yaw += 360/180*3.1415926;

	//--- 弧度转为角度
	ATLN.Rool_angle = ATLN.Rool/3.1415926*180;
	ATLN.Pitch_angle = ATLN.Pitch/3.1415926*180;
	ATLN.Yaw_angle = ATLN.Yaw/3.1415926*180;

	if (ATLN.Yaw_angle < 0) ATLN.Yaw_angle += 360;
}

/***********************************更新四元数*********************************/

void IMU_AHRSupdateComPass(AttitudeL &ATLN,float halfT) 
{
	float rot_xx;//弧度 单位
	float rot_yy;

	float rot_zz;
	float p,r,y;
	float Hx,Hy;
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float tempq0,tempq1,tempq2,tempq3;
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;

	// 先把这些用得到的值算好
	float q0q0 = ATLN.q[0]*ATLN.q[0];
	float q0q1 = ATLN.q[0]*ATLN.q[1];
	float q0q2 = ATLN.q[0]*ATLN.q[2];
	float q0q3 = ATLN.q[0]*ATLN.q[3];
	float q1q1 = ATLN.q[1]*ATLN.q[1];
	float q1q2 = ATLN.q[1]*ATLN.q[2];
	float q1q3 = ATLN.q[1]*ATLN.q[3];
	float q2q2 = ATLN.q[2]*ATLN.q[2];   
	float q2q3 = ATLN.q[2]*ATLN.q[3];
	float q3q3 = ATLN.q[3]*ATLN.q[3];    

	//-------------------
	ATLN.mx=ATLN.mx-ATLN.offset_mx;
	ATLN.my=ATLN.my-ATLN.offset_my;
	ATLN.mz=ATLN.mz-ATLN.offset_mz;
	/************************/
	/*key_num=KEY_Scan();
	if(key_num==3)
	{
		ATLN.q[0]=1;ATLN.q[1]=0;ATLN.q[2]=0;ATLN.q[3]=0;
	}*/

	if(abs(ATLN.gz)<2.0f/PI_L)
	{
		ATLN.ezInt=0;
	}

	// 重力坐标系
	//矫正零偏
	//		ATLN.ax=(ATLN.ax-0.5)/9.85;
	//		ATLN.ay=(ATLN.ay-0.5)/9.85;
	//		ATLN.az=(ATLN.az+0.675)/10.025;

	// normalise the measurements
	norm = sqrt(ATLN.ax*ATLN.ax + ATLN.ay*ATLN.ay + ATLN.az*ATLN.az);       
	ATLN.ax = ATLN.ax / norm;
	ATLN.ay = ATLN.ay / norm;
	ATLN.az = ATLN.az / norm;
	//        norm = sqrt(ATLN.mx*ATLN.mx + ATLN.my*ATLN.my + ATLN.mz*ATLN.mz);          
	//        ATLN.mx = ATLN.mx / norm;
	//        ATLN.my = ATLN.my / norm;
	//        ATLN.mz = ATLN.mz / norm;         
	//        
	//        // compute reference direction of flux
	//        hx = 2*ATLN.mx*(0.5 - q2q2 - q3q3) + 2*ATLN.my*(q1q2 - q0q3) + 2*ATLN.mz*(q1q3 + q0q2);
	//        hy = 2*ATLN.mx*(q1q2 + q0q3) + 2*ATLN.my*(0.5 - q1q1 - q3q3) + 2*ATLN.mz*(q2q3 - q0q1);
	//        hz = 2*ATLN.mx*(q1q3 - q0q2) + 2*ATLN.my*(q2q3 + q0q1) + 2*ATLN.mz*(0.5 - q1q1 - q2q2);         
	//        bx = sqrt((hx*hx) + (hy*hy));
	//        bz = hz;        

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//vz=2*q0q0 - 1.0f + 2*q3q3	;
	//        wx = bx*(0.5 - q2q2 - ATLN.q[3]*ATLN.q[3]) + bz*(q1q3 - q0q2);
	//        wy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
	//        wz = bx*(q0q2 + q1q3) + bz*(0.5 - q1q1 - q2q2);  

	//	halfvx = q1q3 - q0q2;
	//	halfvy = q0q1 + q2q3;
	//	halfvz = q0q0 - 0.5f + q3q3;


	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ATLN.ay*vz - ATLN.az*vy);// + (ATLN.my*wz - ATLN.mz*wy);
	ey = (ATLN.az*vx - ATLN.ax*vz);// + (ATLN.mz*wx - ATLN.mx*wz);
	ez = (ATLN.ax*vy - ATLN.ay*vx);// + (ATLN.mx*wy - ATLN.my*wx);

	// integral error scaled integral gain


	ATLN.exInt = ATLN.exInt + ex*Ki_L;
	ATLN.eyInt = ATLN.eyInt + ey*Ki_L;
	ATLN.ezInt = ATLN.ezInt + ez*Ki_L;

	//	printf("&%d *%d ",(int)(ATLN.gx*100000),(int)(ex*100000));

	// adjusted gyroscope measurements
	ATLN.gx = ATLN.gx + Kp_L*ex + ATLN.exInt;
	ATLN.gy = ATLN.gy + Kp_L*ey + ATLN.eyInt;
	ATLN.gz = ATLN.gz + Kp_L*ez + ATLN.ezInt;

	// integrate quaternion rate and normalise
	float q0r = ATLN.q[0] + (-ATLN.q[1]*ATLN.gx - ATLN.q[2]*ATLN.gy - ATLN.q[3]*ATLN.gz)*halfT;
	float q1r = ATLN.q[1] + (ATLN.q[0]*ATLN.gx + ATLN.q[2]*ATLN.gz - ATLN.q[3]*ATLN.gy)*halfT;
	float q2r = ATLN.q[2] + (ATLN.q[0]*ATLN.gy - ATLN.q[1]*ATLN.gz + ATLN.q[3]*ATLN.gx)*halfT;
	float q3r = ATLN.q[3] + (ATLN.q[0]*ATLN.gz + ATLN.q[1]*ATLN.gy - ATLN.q[2]*ATLN.gx)*halfT;  

	ATLN.q[0]=q0r;
	ATLN.q[1]=q1r;
	ATLN.q[2]=q2r;
	ATLN.q[3]=q3r;

	// normalise quaternion
	norm = sqrt(ATLN.q[0]*ATLN.q[0] + ATLN.q[1]*ATLN.q[1] + ATLN.q[2]*ATLN.q[2] + ATLN.q[3]*ATLN.q[3]);
	ATLN.q[0] = ATLN.q[0] / norm;
	ATLN.q[1] = ATLN.q[1] / norm;
	ATLN.q[2] = ATLN.q[2] / norm;
	ATLN.q[3] = ATLN.q[3] / norm;


	//------------------------------------------------------------------------------------------------------
	//将上位机的磁场移到下位机
#if 1
	//更新方向余弦矩阵
	t11=ATLN.q[0]*ATLN.q[0]+ATLN.q[1]*ATLN.q[1]-ATLN.q[2]*ATLN.q[2]-ATLN.q[3]*ATLN.q[3];
	t12=2.0*(ATLN.q[1]*ATLN.q[2]+ATLN.q[0]*ATLN.q[3]);
	t13=2.0*(ATLN.q[1]*ATLN.q[3]-ATLN.q[0]*ATLN.q[2]);
	t21=2.0*(ATLN.q[1]*ATLN.q[2]-ATLN.q[0]*ATLN.q[3]);
	t22=ATLN.q[0]*ATLN.q[0]-ATLN.q[1]*ATLN.q[1]+ATLN.q[2]*ATLN.q[2]-ATLN.q[3]*ATLN.q[3];
	t23=2.0*(ATLN.q[2]*ATLN.q[3]+ATLN.q[0]*ATLN.q[1]);
	t31=2.0*(ATLN.q[1]*ATLN.q[3]+ATLN.q[0]*ATLN.q[2]);
	t32=2.0*(ATLN.q[2]*ATLN.q[3]-ATLN.q[0]*ATLN.q[1]);
	t33=ATLN.q[0]*ATLN.q[0]-ATLN.q[1]*ATLN.q[1]-ATLN.q[2]*ATLN.q[2]+ATLN.q[3]*ATLN.q[3];

	ATLN.Rool = atan2(t23,t33);
	ATLN.Pitch = -asin(t13);

	
	ATLN.Yaw_angle=180.0*ATLN.Yaw/3.1415926;

	ATLN.Yaw_angle=ATLN.Yaw_angle%360;
	if (ATLN.Yaw_angle < 0) ATLN.Yaw_angle += 360;

	//------------------
#if 1
	//Hx=mx*sin(r)*sin(p)+my*cos(p)-mz*cos(r)*sin(p);
	//Hy=mx*cos(r)+mz*sin(r);

	float Hxx=ATLN.mx*sin(ATLN.Rool)*sin(ATLN.Pitch)+ATLN.my*cos(ATLN.Pitch)-ATLN.mz*cos(ATLN.Rool)*sin(ATLN.Pitch);
	float Hyy=ATLN.mx*cos(ATLN.Rool)+ATLN.mz*sin(ATLN.Rool);

	//Hx Hy滤波
	//for(int i=2;i>0;i--)
	//{
	//	ATLN.Hx[i]=ATLN.Hx[i-1];
	//	ATLN.Hy[i]=ATLN.Hy[i-1];
	//}
	//ATLN.Hx[0]=Hxx;
	//ATLN.Hy[0]=Hyy;

	//float Hxx_Sum=0;
	//float Hyy_Sum=0;
	//for(int i=0;i<3;i++)
	//{
	//	Hxx_Sum+=ATLN.Hx[i];
	//	Hyy_Sum+=ATLN.Hy[i];
	//}

	//if(abs(ATLN.Pitch)<80)
	//{
	//	Hxx=Hxx_Sum;
	//	Hyy=Hyy_Sum;
	//}
	
	//--------------------

	ATLN.Yaw=atan2(Hxx,Hyy);
	
	ATLN.Yaw_angle=180.0*ATLN.Yaw/3.1415926;

	ATLN.Yaw_angle=ATLN.Yaw_angle%360;
	if (ATLN.Yaw_angle < 0) ATLN.Yaw_angle += 360;
	
	

	rot_xx=ATLN.Rool;
	rot_yy=ATLN.Pitch;
	rot_zz=ATLN.Yaw;
	ATLN.q[0] = cos(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[1] = sin(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)-cos(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[2] = cos(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2);

	ATLN.q[3] = cos(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2)-sin(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2);
#endif
#endif
	
}

/***************************************串口十六进制数据 转 传感器含义数据*****************************************/
void Got_Data_L(AttitudeL &ATLN,int cnum,
	                          unsigned char *AHx,unsigned char *ALx,
	                          unsigned char *AHy,unsigned char *ALy,
							  unsigned char *AHz,unsigned char *ALz,

							  unsigned char *GHx,unsigned char *GLx,
							  unsigned char *GHy,unsigned char *GLy,
							  unsigned char *GHz,unsigned char *GLz,

							  unsigned char *MHx,unsigned char *MLx,
							  unsigned char *MHy,unsigned char *MLy,
							  unsigned char *MHz,unsigned char *MLz
	)
{
	
	short LL=ALx[cnum];
	short HH=AHx[cnum]<<8;
	int HL=HH+LL;
	ATLN.ax=float(HL)/1.6384*0.00098;

	LL=ALy[cnum];
	HH=AHy[cnum]<<8;
	HL=HH+LL;
	ATLN.ay=float(HL)/1.6384*0.00098;

	LL=ALz[cnum];
	HH=AHz[cnum]<<8;
	HL=HH+LL;
	ATLN.az=float(HL)/1.6384*0.00098;

	//------
	LL=GLx[cnum];
	HH=GHx[cnum]<<8;
	HL=HH+LL;
	ATLN.gx=float(HL)/164*10*3.14159/180;

	LL=GLy[cnum];
	HH=GHy[cnum]<<8;
	HL=HH+LL;
	ATLN.gy=float(HL)/164*10*3.14159/180;

	LL=GLz[cnum];
	HH=GHz[cnum]<<8;
	HL=HH+LL;
	ATLN.gz=float(HL)/164*10*3.14159/180;

	//------
	LL=MLx[cnum];
	HH=MHx[cnum]<<8;
	HL=HH+LL;
	ATLN.mx=float(HL);

	LL=MLy[cnum];
	HH=MHy[cnum]<<8;
	HL=HH+LL;
	ATLN.my=float(HL);

	LL=MLz[cnum];
	HH=MHz[cnum]<<8;
	HL=HH+LL;
	ATLN.mz=float(HL);
}

/********************************* MPU9250 传感器坐标系欧拉角 转 OpenGL坐标系四元数  **********************************/
void LXW_IMU_6B_L(AttitudeL &ATLN, int Rool_angle0,int Pitch_angle0,int Yaw_angle0)
{
	float p= float(Pitch_angle0)*3.1415926/180/2;//弧度 单位
	float r= float(Rool_angle0)*3.1415926/180/2;
	float y= float(Yaw_angle0)*3.1415926/180/2;

	ATLN.QGL[0] = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);

	ATLN.QGL[1] = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);

	ATLN.QGL[2] = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);

	ATLN.QGL[3] = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);

}
/*************************************姿态平滑 +  传感器坐标系欧拉角 转 OpenGL坐标系四元数**********************************/
void Smooth_GL_L(AttitudeL &ATLN)
{
	//平滑
	float starting[4];
	starting[0]=ATLN.qb[1];
	starting[1]=ATLN.qb[2];
	starting[2]=ATLN.qb[3];
	starting[3]=ATLN.qb[0];

	float ending[4];
	ending[0]=ATLN.q[1];
	ending[1]=ATLN.q[2];
	ending[2]=ATLN.q[3];
	ending[3]=ATLN.q[0];
	float result[4];

	//参数越小越平滑
	slerp( result, starting, ending, 0.389999 );

	Quarter_T_OlA2_L(ATLN,result[3],result[0],result[1],result[2]);

	ATLN.qb[0]=result[3];
	ATLN.qb[1]=result[0];
	ATLN.qb[2]=result[1];
	ATLN.qb[3]=result[2];

	LXW_IMU_6B_L(ATLN,ATLN.Rool_angle,ATLN.Pitch_angle,ATLN.Yaw_angle);//传感器坐标系 转 OpenGL坐标系
}
/****************************** OpenGL 四元数 转 欧拉角 ***************************/
void GL_OLA_T_Quarter_L(AttitudeL &ATLN)
{
	float w=ATLN.QGL[0];
	float x=ATLN.QGL[1];
	float y=ATLN.QGL[2];
	float z=ATLN.QGL[3];

	ATLN.Rool_GL=atan2(2*(w*z+x*y),1-2*(z*z+x*x));
	ATLN.Pitch_GL=asin(2*(w*x-y*z));
	ATLN.Yaw_GL=atan2(2*(w*y+z*x),1-2*(x*x+y*y));

	ATLN.Rool_angle_GL=ATLN.Rool_GL*180/PI;
	ATLN.Pitch_angle_GL=ATLN.Pitch_GL*180/PI;
	ATLN.Yaw_angle_GL=ATLN.Yaw_GL*180/PI;

	ATLN.Yaw_angle_GL%=360;
	if(ATLN.Yaw_angle_GL<0)
		ATLN.Yaw_angle_GL+=360;

}


/***********************************************************************************************/
/***********************共轭求逆**************************/
//输入参数 ：四元数
//输出参数 : 四元数的逆
//公式查看： 交接文档 ——————手势建模 -姿态运算说明
GL_QuaterB Conjugate_QB(GL_QuaterB QN)
{
	GL_QuaterB QN_C;//共轭
	float norm = sqrt(QN.q0*QN.q0+QN.q1*QN.q1+QN.q2*QN.q2+QN.q3*QN.q3);//求模

	QN_C.q0= QN.q0;
	QN_C.q1=-QN.q1;
	QN_C.q2=-QN.q2;
	QN_C.q3=-QN.q3;

	GL_QuaterB QN_I;//求逆

	QN_I.q0=QN_C.q0/norm;
	QN_I.q1=QN_C.q1/norm;
	QN_I.q2=QN_C.q2/norm;
	QN_I.q3=QN_C.q3/norm;

	return QN_I;
}

/***********************四元数相乘**************************/
//公式查看： 交接文档 ——————手势建模 -姿态运算说明
//输入参数：两个四元数
//输出参数：两个四元数的乘积
GL_QuaterB MUL_QB(GL_QuaterB Q1,GL_QuaterB Q2)
{
	CvMat* A1 = cvCreateMat(4,4,CV_32FC1);//构造4*4四元数矩阵
	CvMat* A2 = cvCreateMat(4,1,CV_32FC1);//构造4*1四元数矩阵
	CvMat* MM = cvCreateMat(4,1,CV_32FC1);//构造4*1四元数矩阵

	float lm=Q1.q0;
	float p1=Q1.q1;
	float p2=Q1.q2;
	float p3=Q1.q3;
	//------4*4矩阵赋值
	cvmSet(A1,0,0,lm);
	cvmSet(A1,0,1,-p1);
	cvmSet(A1,0,2,-p2);
	cvmSet(A1,0,3,-p3);

	cvmSet(A1,1,0,p1);
	cvmSet(A1,1,1,lm);
	cvmSet(A1,1,2,-p3);
	cvmSet(A1,1,3,p2);

	cvmSet(A1,2,0,p2);
	cvmSet(A1,2,1,p3);
	cvmSet(A1,2,2,lm);
	cvmSet(A1,2,3,-p1);

	cvmSet(A1,3,0,p3);
	cvmSet(A1,3,1,-p2);
	cvmSet(A1,3,2,p1);
	cvmSet(A1,3,3,lm);
	//------4*1矩阵赋值

	cvmSet(A2,0,0,Q2.q0);
	cvmSet(A2,1,0,Q2.q1);
	cvmSet(A2,2,0,Q2.q2);
	cvmSet(A2,3,0,Q2.q3);
	//------

	cvMatMulAdd( A1, A2, 0, MM );//矩阵相乘

	GL_QuaterB QN;
	//输出四元数 赋值
	QN.q0=cvmGet(MM,0,0);
	QN.q1=cvmGet(MM,1,0);
	QN.q2=cvmGet(MM,2,0);
	QN.q3=cvmGet(MM,3,0);
	//释放矩阵缓存
	cvReleaseMat(&A1);
	cvReleaseMat(&A2);
	cvReleaseMat(&MM);

	return QN;//返回求得 四元数
}

/**************************************** 全局 推局部旋转 ***************************************/
void GL_Quarter_GLobal_T_Local(AttitudeL &ATLNO,AttitudeL &ATLN,int &CDU_PN,int &CDU_RN,int &CDU_YN)
{
	GL_QuaterB Q0;//父节点
	Q0.q0=ATLNO.QGL[0];
	Q0.q1=ATLNO.QGL[1];
	Q0.q2=ATLNO.QGL[2];
	Q0.q3=ATLNO.QGL[3];

	GL_QuaterB Q1;//子节点
	Q1.q0=ATLN.QGL[0];
	Q1.q1=ATLN.QGL[1];
	Q1.q2=ATLN.QGL[2];
	Q1.q3=ATLN.QGL[3];

	GL_QuaterB Q0_N=Conjugate_QB(Q0);


	GL_QuaterB Q1_Local=MUL_QB(Q0_N,Q1);//子节点

	//--------------------------------------------------
	float w=Q1_Local.q0;
	float x=Q1_Local.q1;
	float y=Q1_Local.q2;
	float z=Q1_Local.q3;

	ATLN.Rool_Local_GL=atan2(2*(w*z+x*y),1-2*(z*z+x*x));
	ATLN.Pitch_Local_GL=asin(2*(w*x-y*z));
	ATLN.Yaw_Local_GL=atan2(2*(w*y+z*x),1-2*(x*x+y*y));

	ATLN.Rool_Local_angle_GL=ATLN.Rool_Local_GL*180/PI;
	ATLN.Pitch_Local_angle_GL=ATLN.Pitch_Local_GL*180/PI;
	ATLN.Yaw_Local_angle_GL=ATLN.Yaw_Local_GL*180/PI;

	ATLN.Yaw_Local_angle_GL%=360;
	if(ATLN.Yaw_Local_angle_GL<0)
		ATLN.Yaw_Local_angle_GL+=360;

	CDU_RN=ATLN.Rool_Local_angle_GL;
	CDU_PN=ATLN.Pitch_Local_angle_GL;
	CDU_YN=ATLN.Yaw_Local_angle_GL;

}
#endif