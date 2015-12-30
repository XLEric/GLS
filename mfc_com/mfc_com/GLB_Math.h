#pragma once
#ifndef __GLB_MAT_H__
#define __GLB_MAT_H__
#include <glut.h>
#include <windows.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "CvvImage.h"



IplImage* FrameT1=cvCreateImage(cvSize(320,240) , 8 , 3);

int Compass_Yaw_Data[8]={0};

typedef struct _GL_Vector_
{
	float x;
	float y;
	float z;
}GL_Vector;

typedef struct _GL_Point_
{
	float x;
	float y;
	float z;
}GL_Point;

GL_Point Fig_GL[5];
typedef struct _GL_Quater_ //四元数简易 结构体
{
	float q0;
	float q1;
	float q2;
	float q3;
}GL_Quater;

typedef struct _GL_Hand_ //Hand
{
	GL_Point Thumb_Local[2]; //GL 局部坐标 ,0为起始点，1为Fig连接点
	GL_Point Index_Local[2]; //GL 局部坐标 ,0为起始点，1为Fig连接点
	GL_Point Mid_Local[2];   //GL 局部坐标 ,0为起始点，1为Fig连接点
	GL_Point Ring_Local[2];  //GL 局部坐标 ,0为起始点，1为Fig连接点
	GL_Point Little_Local[2];//GL 局部坐标 ,0为起始点，1为Fig连接点


	GL_Point Thumb_Global[5]; //GL 世界坐标 ,0为起始点，1为Fig连接点
	GL_Point Index_Global[5]; //GL 世界坐标 ,0为起始点，1为Fig连接点
	GL_Point Mid_Global[5];   //GL 世界坐标 ,0为起始点，1为Fig连接点
	GL_Point Ring_Global[5];  //GL 世界坐标 ,0为起始点，1为Fig连接点
	GL_Point Little_Global[5];//GL 世界坐标 ,0为起始点，1为Fig连接点


	//尾部平面
	float A,B,C,D;
	//尾部平面点
	GL_Point Back_Pt;
	GL_Point Back_PtS[4];

	//点到尾部平面的距离,考虑用距离考量伸缩量
	float Dist[5];
	//手部横向线
	GL_Point Line[2];
	//顶点 到 手部横向线的 垂足
	GL_Point Pedal_Pt[5];
	// 获取手指某点 到 手部横线垂足的一个向量 与 手的方向向量之间夹角， 用俯仰角限制的非正常弯曲姿态
	int Angle_Cos[5];
}GL_Hand;

GL_Hand GL_HandL,GL_HandR;

//手掌方向球
GL_Point Plam_NormF;

//手掌方向球 旋转轴
GL_Point Plam_NormF_Right;

bool flagPP=0;

//------------------------------------------------------------------------------------------------ 函数申明
void GLB_OlA_Quater(GL_Quater &QtN,int Rool_angle0,int Pitch_angle0,int Yaw_angle0);
void GLB_Fig_BuildOne_OneDOF(GL_Point FigNN,GL_Quater QtN,GL_Point Global_1,float pos_xN,float pos_yN,float pos_zN,short Num,int CDUP,int CDUR,int CDUY);
void GLB_Fig_BuildOne(GL_Point FigNN,GL_Quater QtN,
	GL_Point &Global_1,GL_Point &Global_2,GL_Point &Global_3,GL_Point &Global_4,
	float pos_xN,float pos_yN,float pos_zN,short Num);
//------------------------------------------------------------------------------------------------ 函数申明 Well Done!
/************************************三维向量夹角*********************************/
int GLB_Cos_Angle(GL_Point PtA,GL_Point PtB,GL_Point PtAA,GL_Point PtBB)
{
	float a1=PtB.x-PtA.x;
    float a2=PtB.y-PtA.y;
	float a3=PtB.z-PtA.z;
	
	float b1=PtBB.x-PtAA.x;
	float b2=PtBB.y-PtAA.y;
	float b3=PtBB.z-PtAA.z;

	int angle=0;
	float a=a1*b1+a2*b2+a3*b3;

	float b=a1*a1+a2*a2+a3*a3;
	b=sqrt(b);

	float c=b1*b1+b2*b2+b3*b3;
	c=sqrt(c);

	if(b*c!=0)
	{
		float angle_r=a/(b*c);
		angle=float(acos(angle_r)/3.1415926)*180;

		return angle;
	}
	return angle;
}
/************************************获取单个手指三维垂足点*********************************/
GL_Point GLB_Got_Pedal(GL_Point PtO,GL_Point PtA,GL_Point PtB)
{
	GL_Point PtN;
	PtN.x=0;
	PtN.y=0;
	PtN.z=0;

	float a= (PtA.x-PtO.x)*(PtB.x-PtA.x) + (PtA.y-PtO.y)*(PtB.y-PtA.y) + (PtA.z-PtO.z)*(PtB.z-PtA.z) ;
	float b= (PtB.x-PtA.x)*(PtB.x-PtA.x) + (PtB.y-PtA.y)*(PtB.y-PtA.y) + (PtB.z-PtA.z)*(PtB.z-PtA.z) ;
	
	if(b==0)
	{
		return PtN;
	}
	float k=-a/b;

	PtN.x=k*(PtB.x-PtA.x)+PtA.x;
	PtN.y=k*(PtB.y-PtA.y)+PtA.y;
	PtN.z=k*(PtB.z-PtA.z)+PtA.z;

	return PtN;
}
/*********************************获得四个手指尖端（Index、Middle、Ring、Little）到手部横线垂足************************************/
void GLB_Got_Pedals(GL_Hand &GL_HandX,int *CDU_P,int *CDU_R,int *CDU_Y,float pos_x,float pos_y,float pos_z)
{
	//获得四个手指尖端（Index、Middle、Ring、Little）到手部横线垂足
	GL_Point GL_Pt1=GLB_Got_Pedal(GL_HandR.Index_Global[2],GL_HandR.Line[0],GL_HandR.Line[1]);
	GL_Point GL_Pt2=GLB_Got_Pedal(GL_HandR.Mid_Global[2],GL_HandR.Line[0],GL_HandR.Line[1]);
	GL_Point GL_Pt3=GLB_Got_Pedal(GL_HandR.Ring_Global[2],GL_HandR.Line[0],GL_HandR.Line[1]);
	GL_Point GL_Pt4=GLB_Got_Pedal(GL_HandR.Little_Global[2],GL_HandR.Line[0],GL_HandR.Line[1]);

	//--------
	GL_Point PtAA;
	PtAA.x=pos_x;
	PtAA.y=pos_y;
	PtAA.z=pos_z;

	for(int i=0;i<5;i++)
	{
		GL_HandX.Angle_Cos[i]=0;
	}
	//----------- 获取手指某点 到 手部横线垂足的一个向量 与 手的方向向量之间夹角，并考虑 用俯仰角限制的非正常弯曲姿态,或者忽略
	//if(CDU_P[1]>0)
	{
		GL_HandX.Angle_Cos[1]=GLB_Cos_Angle(GL_Pt1,GL_HandR.Index_Global[2],PtAA,Plam_NormF);
	}
	//if(CDU_P[2]>0)
	{
		GL_HandX.Angle_Cos[2]=GLB_Cos_Angle(GL_Pt2,GL_HandR.Mid_Global[2],PtAA,Plam_NormF);
	}
	//if(CDU_P[3]>0)
	{
		GL_HandX.Angle_Cos[3]=GLB_Cos_Angle(GL_Pt3,GL_HandR.Ring_Global[2],PtAA,Plam_NormF);
	}
	//if(CDU_P[4]>0)
	{
		GL_HandX.Angle_Cos[4]=GLB_Cos_Angle(GL_Pt4,GL_HandR.Little_Global[2],PtAA,Plam_NormF);
	}
	
	//------------------打印 手指某点 到 手部横线垂足的一个向量 与 手的方向向量之间夹角
	CvFont font1;  
	cvInitFont( &font1, CV_FONT_VECTOR0,0.8, 0.8, 0, 2, 2);
	char buf[256];
	cvSet(FrameT1,CV_RGB(255,255,255));

	sprintf_s(buf,256,"Index :  %d",GL_HandX.Angle_Cos[1]);
	cvPutText(FrameT1,buf,cvPoint(10,30),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Mid   :  %d",GL_HandX.Angle_Cos[2]);
	cvPutText(FrameT1,buf,cvPoint(10,60),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Ring  :  %d",GL_HandX.Angle_Cos[3]);
	cvPutText(FrameT1,buf,cvPoint(10,90),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Little:  %d",GL_HandX.Angle_Cos[4]);
	cvPutText(FrameT1,buf,cvPoint(10,120),&font1,CV_RGB(0,0,0));

	cvShowImage("FrameT1",FrameT1);
	//-----------------------------------
	//// GL_Pt1
	//glPushMatrix();//储存当前视图矩阵
	//glLineWidth(1); 
	//glColor3f(1.0f, 0.0f, 1.0f); 
	//glTranslatef(GL_Pt1.x,GL_Pt1.y,GL_Pt1.z);
	//glutSolidSphere(35, 20, 20);
	//glPopMatrix();//弹出上次保存的位置

	//// GL_Pt2
	//glPushMatrix();//储存当前视图矩阵
	//glLineWidth(1); 
	//glColor3f(0.8f, 0.0f, 0.0f); 
	//glTranslatef(GL_Pt2.x,GL_Pt2.y,GL_Pt2.z);
	//glutSolidSphere(29, 20, 20);
	//glPopMatrix();//弹出上次保存的位置

	//// GL_Pt3
	//glPushMatrix();//储存当前视图矩阵
	//glLineWidth(1); 
	//glColor3f(0.0f, 0.7f, 0.0f); 
	//glTranslatef(GL_Pt3.x,GL_Pt3.y,GL_Pt3.z);
	//glutSolidSphere(23, 20, 20);
	//glPopMatrix();//弹出上次保存的位置

	//// GL_Pt4
	//glPushMatrix();//储存当前视图矩阵
	//glLineWidth(1); 
	//glColor3f(0.0f, 0.0f, 0.7f); 
	//glTranslatef(GL_Pt4.x,GL_Pt4.y,GL_Pt4.z);
	//glutSolidSphere(19, 20, 20);
	//glPopMatrix();//弹出上次保存的位置
}
/*********************************************************************/
//计算对应 Normals Vector1 X Vector2  
GL_Vector GLB_CHAJI(float ax,float ay,float az,float bx,float by,float bz)
{
	GL_Vector v;
	v.x=0;
	v.y=0;
	v.z=0;

	float N_X=(ay*bz-az*by);
	float N_Y=(az*bx-ax*bz);
	float N_Z=(ax*by-ay*bx);

	float norm=N_X*N_X + N_Y*N_Y + N_Z*N_Z;
	norm = sqrt(norm);
	
	if(norm!=0)
	{
		v.x=N_X/norm;
		v.y=N_Y/norm;
		v.z=N_Z/norm;
		return v;
	}

	return v;
}

/************************************三维向量夹角*********************************/
float GLB_Angle(float a1,float a2,float a3,float b1,float b2,float b3)
{
	float angle=0;
	float a=a1*b1+a2*b2+a3*b3;

	float b=a1*a1+a2*a2+a3*a3;
	b=sqrt(b);

	float c=b1*b1+b2*b2+b3*b3;
	c=sqrt(c);

	if(b*c!=0)
	{
		float angle_r=a/(b*c);
		angle=acos(angle_r);

		return angle;
	}
	return angle;
}
/************************************三维点绕任意轴旋转*********************************/
GL_Point GLB_Point(float old_x,float old_y,float old_z,float angle,float x,float y,float z)
{
	GL_Point New_Pt;
	float new_x,new_y,new_z;

	//angle=3.1415926*20/180;
	float c=cos(angle);
	float s=sin(angle);

	new_x = (x*x*(1-c)+c) * old_x + (x*y*(1-c)-z*s) * old_y + (x*z*(1-c)+y*s) * old_z;

	new_y = (y*x*(1-c)+z*s) * old_x + (y*y*(1-c)+c) * old_y + (y*z*(1-c)-x*s) * old_z;

	new_z = (x*z*(1-c)-y*s) * old_x + (y*z*(1-c)+x*s) * old_y + (z*z*(1-c)+c) * old_z;

	New_Pt.x=new_x;
	New_Pt.y=new_y;
	New_Pt.z=new_z;

	return New_Pt;
}

/***************************************** 磁场旋转 *****************************************/
int GLB_Angle_Compass_Yaw(float mx,float my,float mz,float Rr,int PitchN,int RollN,int YawN)
{
	float p=-3.1415926*float(PitchN)/180;
	float r=-3.1415926*float(RollN)/180;
	float norm =mx*mx+my*my+mz*mz;
	norm=sqrt(norm);

	if(norm!=0)
	{
		mx/=norm;
		my/=norm;
		mz/=norm;
	}
	

	float Hx=mx*cos(p)+my*sin(r)*sin(p)-mz*cos(r)*sin(p);
	float Hy=my*cos(r)+mz*sin(r);

	//float Hx=HM_x*cos(Pitch)+HM_y*sin(Rool)*sin(Pitch)-HM_z*cos(Rool)*sin(Pitch);
	//float Hy=HM_y*cos(Rool)+HM_z*sin(Rool);


	float anglef=0;
	anglef=atan2(Hx,Hy);

	int Angle=180.0*anglef/3.1415926;

	if(Angle<=0)Angle=360+Angle;

	
	return Angle;
}

/***********************共轭求逆**************************/
//输入参数 ：四元数
//输出参数 : 四元数的逆
//公式查看： 交接文档 ——————手势建模 -姿态运算说明
GL_Quater Conjugate_Q(GL_Quater QN)
{
	GL_Quater QN_C;//共轭
	float norm = sqrt(QN.q0*QN.q0+QN.q1*QN.q1+QN.q2*QN.q2+QN.q3*QN.q3);//求模

	QN_C.q0= QN.q0;
	QN_C.q1=-QN.q1;
	QN_C.q2=-QN.q2;
	QN_C.q3=-QN.q3;

	GL_Quater QN_I;//求逆

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
GL_Quater MUL_Q(GL_Quater Q1,GL_Quater Q2)
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

	GL_Quater QN;
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
/*****************************对于每个手指 手掌到手指第一节建模********************************/
//Local_0 Local_1 手指第一节骨架（两端点）局部坐标系
//Global_0 Global_1 手指第一节骨架（两端点）全局坐标系
void GLB_Hand_BuildOne(GL_Point Local_0,GL_Point Local_1,GL_Point &Global_0,GL_Point &Global_1,
	GL_Quater QtN,float pos_xN,float pos_yN,float pos_zN,short Num)
{
	// 0
	GL_Quater QtN_N=Conjugate_Q(QtN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=Local_0.x;
	Q_Point.q2=Local_0.y;
	Q_Point.q3=Local_0.z;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	float x0=Q022.q1;
	float y0=Q022.q2;
	float z0=Q022.q3;

	Global_0.x=x0+pos_xN;
	Global_0.y=y0+pos_yN;
	Global_0.z=z0+pos_zN;

	// 1
	Q_Point.q0=0;
	Q_Point.q1=Local_1.x;
	Q_Point.q2=Local_1.y;
	Q_Point.q3=Local_1.z;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	float x1=Q022.q1;
	float y1=Q022.q2;
	float z1=Q022.q3;

	Global_1.x=x1+pos_xN;
	Global_1.y=y1+pos_yN;
	Global_1.z=z1+pos_zN;

	//SHOW

	if(Num==0)
	{
		glPushMatrix();//储存当前视图矩阵
		glLineWidth(1); 
		glColor3f(1.0f, 1.55f, 0.2f); 
		glTranslatef(Global_0.x,Global_0.y,Global_0.z);
		glutSolidSphere(32, 20, 20);
		glPopMatrix();//弹出上次保存的位置
	}
	else
	{
		glPushMatrix();//储存当前视图矩阵
		glLineWidth(1); 
		glColor3f(1.0f, 0.85f, 0.0f); 
		glTranslatef(Global_0.x,Global_0.y,Global_0.z);
		glutSolidSphere(30, 20, 20);
		glPopMatrix();//弹出上次保存的位置
	}
	

	glLineWidth(10);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(Global_0.x,Global_0.y,Global_0.z);
	glVertex3f(Global_1.x,Global_1.y,Global_1.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.25f, 1.0f, 1.0f); 
	glTranslatef(Global_1.x,Global_1.y,Global_1.z);
	glutSolidSphere(28, 20, 20);
	glPopMatrix();//弹出上次保存的位置
}

/*****************************Hand父节点建模（Plam基础建模）********************************/
void GLB_Hand_Build(GL_Hand &GL_HandX,GL_Quater QtN,float Wwn,float pos_xN,float pos_yN,float pos_zN)
{
	//--------------------------
	GL_HandX.Thumb_Local[0].x=120; GL_HandX.Thumb_Local[1].x=300;
	GL_HandX.Thumb_Local[0].y=0; GL_HandX.Thumb_Local[1].y=0;
	GL_HandX.Thumb_Local[0].z=-180; GL_HandX.Thumb_Local[1].z=30;

	GL_HandX.Index_Local[0].x=120; GL_HandX.Index_Local[1].x=120;
	GL_HandX.Index_Local[0].y=0; GL_HandX.Index_Local[1].y=0;
	GL_HandX.Index_Local[0].z=20; GL_HandX.Index_Local[1].z=180;

	GL_HandX.Mid_Local[0].x=40; GL_HandX.Mid_Local[1].x=40;
	GL_HandX.Mid_Local[0].y=0; GL_HandX.Mid_Local[1].y=0;
	GL_HandX.Mid_Local[0].z=20; GL_HandX.Mid_Local[1].z=180;

	GL_HandX.Ring_Local[0].x=-40; GL_HandX.Ring_Local[1].x=-40;
	GL_HandX.Ring_Local[0].y=0; GL_HandX.Ring_Local[1].y=0;
	GL_HandX.Ring_Local[0].z=20; GL_HandX.Ring_Local[1].z=180;

	GL_HandX.Little_Local[0].x=-120; GL_HandX.Little_Local[1].x=-120;
	GL_HandX.Little_Local[0].y=0; GL_HandX.Little_Local[1].y=0;
	GL_HandX.Little_Local[0].z=20; GL_HandX.Little_Local[1].z=180;
	//---------------------------------------------------------------
	// 绘制 Plam
	//绘制 旋转长方体
	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_xN,pos_yN,pos_zN);         
	glRotatef(Wwn,QtN.q1,QtN.q2,QtN.q3);

	glColor3f(0.8f, 0.2f, 0.0f );
	glScaled(100,3,95);
	glutSolidCube(3);

	glPopMatrix(); 
	//绘制 旋转长方体
	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_xN,pos_yN,pos_zN);          // Move Left 1.5 Units And Into The Screen 6.0
	glRotatef(Wwn,QtN.q1,QtN.q2,QtN.q3);
	glColor3f(0.0f, 0.8f, 0.1f );
	glScaled(88,3,80);
	glutSolidCube(3);

	glPopMatrix(); 
	

	//并行从上到下
	GL_Point Plam_Pt[6];

	GL_Quater QtN_N=Conjugate_Q(QtN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=180;
	Q_Point.q2=0;
	Q_Point.q3=220;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	float x0=Q022.q1;
	float y0=Q022.q2;
	float z0=Q022.q3;

	Plam_Pt[0].x=x0+pos_xN;
	Plam_Pt[0].y=y0+pos_yN;
	Plam_Pt[0].z=z0+pos_zN;

	//------------------------------------------ 0 1
	Q_Point.q0=0;
	Q_Point.q1=-180;
	Q_Point.q2=0;
	Q_Point.q3=220;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	x0=Q022.q1;
	y0=Q022.q2;
	z0=Q022.q3;

	Plam_Pt[1].x=x0+pos_xN;
	Plam_Pt[1].y=y0+pos_yN;
	Plam_Pt[1].z=z0+pos_zN;


	glLineWidth(3);
	glColor3f(0.0f, 0.15f, 0.55f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[0].x,Plam_Pt[0].y,Plam_Pt[0].z);
	glVertex3f(Plam_Pt[1].x,Plam_Pt[1].y,Plam_Pt[1].z); 
	glEnd();
	
	
	//------------------------------------------ 2 3
	Q_Point.q0=0;
	Q_Point.q1=220;
	Q_Point.q2=0;
	Q_Point.q3=-20;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	x0=Q022.q1;
	y0=Q022.q2;
	z0=Q022.q3;

	Plam_Pt[2].x=x0+pos_xN;
	Plam_Pt[2].y=y0+pos_yN;
	Plam_Pt[2].z=z0+pos_zN;

	//------------------------------------------
	Q_Point.q0=0;
	Q_Point.q1=-220;
	Q_Point.q2=0;
	Q_Point.q3=-20;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	x0=Q022.q1;
	y0=Q022.q2;
	z0=Q022.q3;

	Plam_Pt[3].x=x0+pos_xN;
	Plam_Pt[3].y=y0+pos_yN;
	Plam_Pt[3].z=z0+pos_zN;

	glLineWidth(6);
	glColor3f(0.0f, 0.15f, 1.05f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[0].x,Plam_Pt[0].y,Plam_Pt[0].z);
	glVertex3f(Plam_Pt[2].x,Plam_Pt[2].y,Plam_Pt[2].z); 
	glEnd();

	glLineWidth(6);
	glColor3f(0.0f, 0.15f, 1.05f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[3].x,Plam_Pt[3].y,Plam_Pt[3].z);
	glVertex3f(Plam_Pt[1].x,Plam_Pt[1].y,Plam_Pt[1].z); 
	glEnd();

	//------------------------------------------  4 5
	Q_Point.q0=0;
	Q_Point.q1=90;
	Q_Point.q2=0;
	Q_Point.q3=-230;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	x0=Q022.q1;
	y0=Q022.q2;
	z0=Q022.q3;

	Plam_Pt[4].x=x0+pos_xN;
	Plam_Pt[4].y=y0+pos_yN;
	Plam_Pt[4].z=z0+pos_zN;

	//------------------------------------------
	Q_Point.q0=0;
	Q_Point.q1=-90;
	Q_Point.q2=0;
	Q_Point.q3=-230;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	x0=Q022.q1;
	y0=Q022.q2;
	z0=Q022.q3;

	Plam_Pt[5].x=x0+pos_xN;
	Plam_Pt[5].y=y0+pos_yN;
	Plam_Pt[5].z=z0+pos_zN;

	glLineWidth(10);
	glColor3f(1.0f, 0.15f, 1.05f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[4].x,Plam_Pt[4].y,Plam_Pt[4].z);
	glVertex3f(Plam_Pt[2].x,Plam_Pt[2].y,Plam_Pt[2].z); 
	glEnd();

	glLineWidth(10);
	glColor3f(1.0f, 0.15f, 1.05f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[3].x,Plam_Pt[3].y,Plam_Pt[3].z);
	glVertex3f(Plam_Pt[5].x,Plam_Pt[5].y,Plam_Pt[5].z); 
	glEnd();

	glLineWidth(10);
	glColor3f(1.0f, 0.0f, 0.0f); 
	glBegin(GL_LINES);
	glVertex3f(Plam_Pt[4].x,Plam_Pt[4].y,Plam_Pt[4].z);
	glVertex3f(Plam_Pt[5].x,Plam_Pt[5].y,Plam_Pt[5].z); 
	glEnd();

	// 获取手部横线
	GL_HandX.Line[0]=Plam_Pt[0];
	GL_HandX.Line[1]=Plam_Pt[1];
	//----------------------------------------------------------------------
	//手掌法向球
	GL_Point Plam_NormP;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=120;
	Q_Point.q3=0;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	Plam_NormP.x=Q022.q1+pos_xN;
	Plam_NormP.y=Q022.q2+pos_yN;
	Plam_NormP.z=Q022.q3+pos_zN;

	glLineWidth(10);
	glColor3f(0.0f, 0.0f, 1.0f); 
	glBegin(GL_LINES);
	glVertex3f(pos_xN,pos_yN,pos_zN);
	glVertex3f(Plam_NormP.x,Plam_NormP.y,Plam_NormP.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.8f, 0.7f, 0.7f); 
	glTranslatef(Plam_NormP.x,Plam_NormP.y,Plam_NormP.z);
	glutSolidSphere(33, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//手掌方向球
	//GL_Point Plam_NormF;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=116;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	Plam_NormF.x=Q022.q1+pos_xN;
	Plam_NormF.y=Q022.q2+pos_yN;
	Plam_NormF.z=Q022.q3+pos_zN;

	glLineWidth(10);
	glColor3f(1.0f, 0.0f, 0.98f); 
	glBegin(GL_LINES);
	glVertex3f(pos_xN,pos_yN,pos_zN);
	glVertex3f(Plam_NormF.x,Plam_NormF.y,Plam_NormF.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 0.0f, 0.0f); 
	glTranslatef(Plam_NormF.x,Plam_NormF.y,Plam_NormF.z);
	glutSolidSphere(23, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//手掌尾部方向球
	
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=-276;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	GL_HandX.Back_Pt.x=Q022.q1+pos_xN;
	GL_HandX.Back_Pt.y=Q022.q2+pos_yN;
	GL_HandX.Back_Pt.z=Q022.q3+pos_zN;

	glLineWidth(10);
	glColor3f(1.0f, 0.0f, 0.98f); 
	glBegin(GL_LINES);
	glVertex3f(pos_xN,pos_yN,pos_zN);
	glVertex3f(GL_HandX.Back_Pt.x,GL_HandX.Back_Pt.y,GL_HandX.Back_Pt.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(1.0f, 0.28f, 0.08f); 
	glTranslatef(GL_HandX.Back_Pt.x,GL_HandX.Back_Pt.y,GL_HandX.Back_Pt.z);
	glutWireSphere(56, 20, 20);
	glPopMatrix();//弹出上次保存的位置


	//求尾部平面的方程系数
	GL_HandX.A=GL_HandX.Back_Pt.x-pos_xN;
	GL_HandX.B=GL_HandX.Back_Pt.y-pos_yN;
	GL_HandX.C=GL_HandX.Back_Pt.z-pos_zN;
	// AX+BY+CZ+D=0  => D=-AX-BY-CZ
	GL_HandX.D=-GL_HandX.A*GL_HandX.Back_Pt.x - GL_HandX.B*GL_HandX.Back_Pt.y - GL_HandX.C*GL_HandX.Back_Pt.z;

#if 0
	//------------------ 尾部平面
	//-----1p
	Q_Point.q0=0;
	Q_Point.q1=200;
	Q_Point.q2=200;
	Q_Point.q3=-276;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	GL_HandR.Back_PtS[0].x=Q022.q1+pos_xN;
	GL_HandR.Back_PtS[0].y=Q022.q2+pos_yN;
	GL_HandR.Back_PtS[0].z=Q022.q3+pos_zN;
	//------ 2p
	Q_Point.q0=0;
	Q_Point.q1=-200;
	Q_Point.q2=200;
	Q_Point.q3=-276;

	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	GL_HandR.Back_PtS[1].x=Q022.q1+pos_xN;
	GL_HandR.Back_PtS[1].y=Q022.q2+pos_yN;
	GL_HandR.Back_PtS[1].z=Q022.q3+pos_zN;
	//---- 3p
	Q_Point.q0=0;
	Q_Point.q1=-200;
	Q_Point.q2=-200;
	Q_Point.q3=-276;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	GL_HandR.Back_PtS[2].x=Q022.q1+pos_xN;
	GL_HandR.Back_PtS[2].y=Q022.q2+pos_yN;
	GL_HandR.Back_PtS[2].z=Q022.q3+pos_zN;

	//----- 4p
	Q_Point.q0=0;
	Q_Point.q1=200;
	Q_Point.q2=-200;
	Q_Point.q3=-276;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	GL_HandR.Back_PtS[3].x=Q022.q1+pos_xN;
	GL_HandR.Back_PtS[3].y=Q022.q2+pos_yN;
	GL_HandR.Back_PtS[3].z=Q022.q3+pos_zN;



	/*下面开始绘制四边形*/
	glColor3f(0.6f, 0.6f, 0.0f); 
	glBegin(GL_QUADS);
	glVertex3f(GL_HandR.Back_PtS[0].x,GL_HandR.Back_PtS[0].y,GL_HandR.Back_PtS[0].z); // 左上顶点
	glVertex3f(GL_HandR.Back_PtS[1].x,GL_HandR.Back_PtS[1].y,GL_HandR.Back_PtS[1].z); // 右上顶点
	glVertex3f(GL_HandR.Back_PtS[2].x,GL_HandR.Back_PtS[2].y,GL_HandR.Back_PtS[2].z); // 右下顶点
	glVertex3f(GL_HandR.Back_PtS[3].x,GL_HandR.Back_PtS[3].y,GL_HandR.Back_PtS[3].z); // 左下顶点
	glEnd(); // 四边形绘制结束
#endif
	//-------------------------------
	//Plam_NormF_Right
	Q_Point.q0=0;
	Q_Point.q1=-280;
	Q_Point.q2=0;
	Q_Point.q3=0;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	Plam_NormF_Right.x=Q022.q1+pos_xN;
	Plam_NormF_Right.y=Q022.q2+pos_yN;
	Plam_NormF_Right.z=Q022.q3+pos_zN;

	glLineWidth(3);
	glColor3f(0.0f, 0.0f, 1.0f); 
	glBegin(GL_LINES);
	glVertex3f(pos_xN,pos_yN,pos_zN);
	glVertex3f(Plam_NormF_Right.x,Plam_NormF_Right.y,Plam_NormF_Right.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.8f, 0.0f, 0.0f); 
	glTranslatef(Plam_NormF_Right.x,Plam_NormF_Right.y,Plam_NormF_Right.z);
	glutSolidSphere(23, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//手掌方向球
	//GL_Point Plam_NormF;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=116;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN_N);

	Plam_NormF.x=Q022.q1+pos_xN;
	Plam_NormF.y=Q022.q2+pos_yN;
	Plam_NormF.z=Q022.q3+pos_zN;

	glLineWidth(10);
	glColor3f(1.0f, 0.0f, 0.98f); 
	glBegin(GL_LINES);
	glVertex3f(pos_xN,pos_yN,pos_zN);
	glVertex3f(Plam_NormF.x,Plam_NormF.y,Plam_NormF.z); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 0.0f, 0.0f); 
	glTranslatef(Plam_NormF.x,Plam_NormF.y,Plam_NormF.z);
	glutSolidSphere(23, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//-----------------------------------------------------------------------------------------------------


	//------>> Thumb
	GLB_Hand_BuildOne(GL_HandX.Thumb_Local[0],GL_HandX.Thumb_Local[1],
		              GL_HandX.Thumb_Global[0],GL_HandX.Thumb_Global[1],
		              QtN,pos_xN,pos_yN,pos_zN,0);

	//------>> Index
	GLB_Hand_BuildOne(GL_HandX.Index_Local[0],GL_HandX.Index_Local[1],
		GL_HandX.Index_Global[0],GL_HandX.Index_Global[1],
		QtN,pos_xN,pos_yN,pos_zN,1);

	//------>> Middle
	GLB_Hand_BuildOne(GL_HandX.Mid_Local[0],GL_HandX.Mid_Local[1],
		GL_HandX.Mid_Global[0],GL_HandX.Mid_Global[1],
		QtN,pos_xN,pos_yN,pos_zN,2);

	//------>> Ring
	GLB_Hand_BuildOne(GL_HandX.Ring_Local[0],GL_HandX.Ring_Local[1],
		GL_HandX.Ring_Global[0],GL_HandX.Ring_Global[1],
		QtN,pos_xN,pos_yN,pos_zN,3);

	//------>> Little
	GLB_Hand_BuildOne(GL_HandX.Little_Local[0],GL_HandX.Little_Local[1],
		GL_HandX.Little_Global[0],GL_HandX.Little_Global[1],
		QtN,pos_xN,pos_yN,pos_zN,4);

	//-----------------------------------------------------------------------------------------------
	
}

/****************************************获取 手指尖端点的坐标***************************************/
// Global_4为手指尖端点
void GLB_Fig_BuildOne_Bend(GL_Point FigNN,GL_Quater QtN,
	GL_Point &Global_1,GL_Point &Global_2,GL_Point &Global_3,GL_Point &Global_4,
	float pos_xN,float pos_yN,float pos_zN,short Num)
{

	float x0=Global_1.x;
	float y0=Global_1.y;
	float z0=Global_1.z;

	GL_Quater QtN_N=Conjugate_Q(QtN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.8;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.41;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	float x1=Q022.q1;
	float y1=Q022.q2;
	float z1=Q022.q3;

	x1=x1+x0;
	y1=y1+y0;
	z1=z1+z0;

	Global_2.x=x1;
	Global_2.y=y1;
	Global_2.z=z1;

	//------ 拓展 Fig 1
	GL_Quater QtN1;

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtN1,0,0,-25);
	else
		GLB_OlA_Quater(QtN1,0,30,0);

	QtN1=MUL_Q(QtN,QtN1);
	GL_Quater QtN1_N=Conjugate_Q(QtN1);


	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.55;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.35;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN1,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN1_N);
	//坐标更新
	float x2=Q022.q1;
	float y2=Q022.q2;
	float z2=Q022.q3;

	x2+=x1;
	y2+=y1;
	z2+=z1;

	Global_3.x=x2;
	Global_3.y=y2;
	Global_3.z=z2;


	//------ 拓展 Fig 2
	GL_Quater QtN2;

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtN2,0,0,-15);
	else
		GLB_OlA_Quater(QtN2,0,10,0);

	QtN2=MUL_Q(QtN1,QtN2);
	GL_Quater QtN2_N=Conjugate_Q(QtN2);


	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.53;

	if(Num==0)
		Q_Point.q3=FigNN.z*0.35;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN2,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN2_N);
	//坐标更新
	float x3=Q022.q1;
	float y3=Q022.q2;
	float z3=Q022.q3;

	x3+=x2;
	y3+=y2;
	z3+=z2;

	Global_4.x=x3;
	Global_4.y=y3;
	Global_4.z=z3;

	
}
/*****************************************获取单指指尖对于手尾部的 距离值***************************************/
// Global_Pt手指尖端点
// Num手指编号
void GLB_Fig_Dist_Bend(GL_Hand &GL_HandX,GL_Point Global_Pt,int Num)
{
	float A=GL_HandX.A;
	float B=GL_HandX.B;
	float C=GL_HandX.C;
	float D=GL_HandX.D;
	
	float X=Global_Pt.x;
	float Y=Global_Pt.y;
	float Z=Global_Pt.z;
	
	// distance = abs(AX+BY+CZ+D)/sqrt(A*A+B*B+C*C);
	GL_HandX.Dist[Num] = abs(A*X+B*Y+C*Z+D)/sqrt(A*A+B*B+C*C);
}


/***************************************基于长度的 四指单自由度设计，Thumb指除外。*****************************************/
//除大拇指外，其它的拇指弯曲程度，由各手指的最顶端到手尾部平面的距离决定。
void GLB_FigS_Build_Bend( float q0,float q1,float q2,float q3,
	float q10,float q11,float q12,float q13,
	float q20,float q21,float q22,float q23,
	float q30,float q31,float q32,float q33,
	float q40,float q41,float q42,float q43,
	float q50,float q51,float q52,float q53,
	float pos_x,float pos_y,float pos_z,
	int *CDU_P,int *CDU_R,int *CDU_Y

	)
{
	//------ 获取手指的尖端坐标
	GL_Quater Qt0;

	Qt0.q0=q10;
	Qt0.q1=q11;
	Qt0.q2=q12;
	Qt0.q3=q13;
	Fig_GL[0].z=220;
	GLB_Fig_BuildOne_Bend(Fig_GL[0],Qt0,
		GL_HandR.Thumb_Global[1],GL_HandR.Thumb_Global[2],GL_HandR.Thumb_Global[3],GL_HandR.Thumb_Global[4],
		pos_x,pos_y,pos_z,0);
	Qt0.q0=q20;
	Qt0.q1=q21;
	Qt0.q2=q22;
	Qt0.q3=q23;
	Fig_GL[1].z=220;
	GLB_Fig_BuildOne_Bend(Fig_GL[1],Qt0,
		GL_HandR.Index_Global[1],GL_HandR.Index_Global[2],GL_HandR.Index_Global[3],GL_HandR.Index_Global[4],
		pos_x,pos_y,pos_z,1);


	Qt0.q0=q30;
	Qt0.q1=q31;
	Qt0.q2=q32;
	Qt0.q3=q33;
	Fig_GL[2].z=250;
	GLB_Fig_BuildOne_Bend(Fig_GL[2],Qt0,
		GL_HandR.Mid_Global[1],GL_HandR.Mid_Global[2],GL_HandR.Mid_Global[3],GL_HandR.Mid_Global[4],
		pos_x,pos_y,pos_z,2);

	Qt0.q0=q40;
	Qt0.q1=q41;
	Qt0.q2=q42;
	Qt0.q3=q43;
	Fig_GL[3].z=210;
	GLB_Fig_BuildOne_Bend(Fig_GL[3],Qt0,
		GL_HandR.Ring_Global[1],GL_HandR.Ring_Global[2],GL_HandR.Ring_Global[3],GL_HandR.Ring_Global[4],
		pos_x,pos_y,pos_z,3);

	Qt0.q0=q50;
	Qt0.q1=q51;
	Qt0.q2=q52;
	Qt0.q3=q53;
	Fig_GL[4].z=180;
	GLB_Fig_BuildOne_Bend(Fig_GL[4],Qt0,
		GL_HandR.Little_Global[1],GL_HandR.Little_Global[2],GL_HandR.Little_Global[3],GL_HandR.Little_Global[4],
		pos_x,pos_y,pos_z,4);

	//----------------------------------------------------
	
	//获得四个手指尖端（Index、Middle、Ring、Little）到手部横线垂足
	bool Flag_Enable_Angle=1;
	GLB_Got_Pedals(GL_HandR,CDU_P,CDU_R,CDU_Y,pos_x, pos_y, pos_z);

	if(Flag_Enable_Angle==1)
	{
		if(CDU_P[1]>0 || GL_HandR.Angle_Cos[1]>100){  CDU_P[1]= float(GL_HandR.Angle_Cos[1])*2/4 ;}
		if(CDU_P[2]>0 || GL_HandR.Angle_Cos[2]>100){  CDU_P[2]= float(GL_HandR.Angle_Cos[2])*2/4 ;}
		if(CDU_P[3]>0 || GL_HandR.Angle_Cos[3]>100){  CDU_P[3]= float(GL_HandR.Angle_Cos[3])*2/4 ;}
		//if(CDU_P[4]>0 || GL_HandR.Angle_Cos[4]>100){  CDU_P[4]= float(GL_HandR.Angle_Cos[4])*2/4 ;}//小拇指单独运动
		if(CDU_P[3]>0 || GL_HandR.Angle_Cos[3]>100){  CDU_P[4]= float(GL_HandR.Angle_Cos[3])*2/4 ;}//小拇指与无名指联动
		CDU_P[4]=CDU_P[3];//小拇指与无名指联动
	}

	//if()
	//获得四个手指尖端（Index、Middle、Ring、Little） 到 尾部平面距离
	/*GLB_Fig_Dist_Bend(GL_HandR,GL_HandR.Index_Global[4],1);
	GLB_Fig_Dist_Bend(GL_HandR,GL_HandR.Mid_Global[4],2);
	GLB_Fig_Dist_Bend(GL_HandR,GL_HandR.Ring_Global[4],3);
	GLB_Fig_Dist_Bend(GL_HandR,GL_HandR.Little_Global[4],4);*/

	//四个手指点 的 距离值 打印显示
	/*CvFont font1;  
	cvInitFont( &font1, CV_FONT_VECTOR0,0.8, 0.8, 0, 2, 2);
	char buf[256];
	cvSet(FrameT1,CV_RGB(255,255,255));

	sprintf_s(buf,256,"Index :  %f",GL_HandR.Dist[1]);
	cvPutText(FrameT1,buf,cvPoint(10,30),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Mid   :  %f",GL_HandR.Dist[2]);
	cvPutText(FrameT1,buf,cvPoint(10,60),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Ring  :  %f",GL_HandR.Dist[3]);
	cvPutText(FrameT1,buf,cvPoint(10,90),&font1,CV_RGB(0,0,0));

	sprintf_s(buf,256,"Little:  %f",GL_HandR.Dist[4]);
	cvPutText(FrameT1,buf,cvPoint(10,120),&font1,CV_RGB(0,0,0));

	cvShowImage("FrameT1",FrameT1);*/

	//--------------------------------------------------- 
	//考虑规避 手指上翻的异常情况。如果手指不上翻，将手指局部相对于手掌尾部的距离变化，转化为局部俯仰角，以此来衡量弯曲程度。
	/*if(Flag_Enable_Angle==0)
	{
	if(CDU_P[1]>0){  CDU_P[1]= float(850-GL_HandR.Dist[1])/850*85 ;}
	if(CDU_P[2]>0){  CDU_P[2]= float(900-GL_HandR.Dist[2])/900*85 ;}
	if(CDU_P[3]>0){  CDU_P[3]= float(820-GL_HandR.Dist[3])/820*85 ;}
	if(CDU_P[4]>0){  CDU_P[4]= float(770-GL_HandR.Dist[4])/770*85 ;}
	}*/
	
	//----------------------------------------------------
	//绘制大拇指
	Qt0.q0=q10;
	Qt0.q1=q11;
	Qt0.q2=q12;
	Qt0.q3=q13;
	Fig_GL[0].z=220;
	GLB_Fig_BuildOne(Fig_GL[0],Qt0,
		GL_HandR.Thumb_Global[1],GL_HandR.Thumb_Global[2],GL_HandR.Thumb_Global[3],GL_HandR.Thumb_Global[4],
		pos_x,pos_y,pos_z,0);
	//绘制 Index、Middle、Ring、Little 的弯曲。
	Qt0.q0=q0;
	Qt0.q1=q1;
	Qt0.q2=q2;
	Qt0.q3=q3;
	Fig_GL[1].z=220;
	GLB_Fig_BuildOne_OneDOF(Fig_GL[1],Qt0,GL_HandR.Index_Global[1],pos_x,pos_y,pos_z,1,CDU_P[1],CDU_R[1],CDU_Y[1]);

	Fig_GL[2].z=250;
	GLB_Fig_BuildOne_OneDOF(Fig_GL[2],Qt0,GL_HandR.Mid_Global[1],pos_x,pos_y,pos_z,2,CDU_P[2],CDU_R[2],CDU_Y[2]);

	Fig_GL[3].z=210;
	GLB_Fig_BuildOne_OneDOF(Fig_GL[3],Qt0,GL_HandR.Ring_Global[1],pos_x,pos_y,pos_z,3,CDU_P[3],CDU_R[3],CDU_Y[3]);

	Fig_GL[4].z=180;
	GLB_Fig_BuildOne_OneDOF(Fig_GL[4],Qt0,GL_HandR.Little_Global[1],pos_x,pos_y,pos_z,4,CDU_P[4],CDU_R[4],CDU_Y[4]);
}

/********************************* GL坐标系欧拉角 转 OpenGL坐标系四元数  **********************************/
void GLB_OlA_Quater(GL_Quater &QtN,int Rool_angle0,int Pitch_angle0,int Yaw_angle0)
{
	float p= float(Pitch_angle0)*3.1415926/180/2;//弧度 单位
	float r= float(Rool_angle0)*3.1415926/180/2;
	float y= float(Yaw_angle0)*3.1415926/180/2;

	QtN.q0= cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);

	QtN.q1 = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);

	QtN.q2 = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);

	QtN.q3 = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);

}


/***************************************************单指 3自由度设计*************************************************/
void GLB_Fig_BuildOne(GL_Point FigNN,GL_Quater QtN,
	                  GL_Point &Global_1,GL_Point &Global_2,GL_Point &Global_3,GL_Point &Global_4,
					  float pos_xN,float pos_yN,float pos_zN,short Num)
{

	float x0=Global_1.x;
	float y0=Global_1.y;
	float z0=Global_1.z;

	GL_Quater QtN_N=Conjugate_Q(QtN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.8;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.5;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtN_N);
	//坐标更新
	float x1=Q022.q1;
	float y1=Q022.q2;
	float z1=Q022.q3;

	x1=x1+x0;
	y1=y1+y0;
	z1=z1+z0;

	Global_2.x=x1;
	Global_2.y=y1;
	Global_2.z=z1;

	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x0,y0,z0);
	glVertex3f(x1,y1,z1); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(1.0f, 0.0f, 1.0f); 
	glTranslatef(x1,y1,z1);
	glutSolidSphere(32, 20, 20);
	glPopMatrix();//弹出上次保存的位置


	//------ 拓展 Fig 1
	GL_Quater QtN1;

	if(Num==0)//Thumb特殊处理
	GLB_OlA_Quater(QtN1,0,0,-25);
	else
    GLB_OlA_Quater(QtN1,0,30,0);

	QtN1=MUL_Q(QtN,QtN1);
	GL_Quater QtN1_N=Conjugate_Q(QtN1);

	
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.55;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.3;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	 Q011=MUL_Q(QtN1,Q_Point);
	//------计算2次相乘
	 Q022=MUL_Q(Q011,QtN1_N);
	//坐标更新
	float x2=Q022.q1;
	float y2=Q022.q2;
	float z2=Q022.q3;

	x2+=x1;
	y2+=y1;
	z2+=z1;

	Global_3.x=x2;
	Global_3.y=y2;
	Global_3.z=z2;

	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x2,y2,z2);
	glVertex3f(x1,y1,z1); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 1.0f, 0.0f); 
	glTranslatef(x2,y2,z2);
	glutSolidSphere(33, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//------ 拓展 Fig 2
	GL_Quater QtN2;

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtN2,0,0,-15);
	else
		GLB_OlA_Quater(QtN2,0,10,0);

	QtN2=MUL_Q(QtN1,QtN2);
	GL_Quater QtN2_N=Conjugate_Q(QtN2);


	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.53;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.25;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtN2,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtN2_N);
	//坐标更新
	float x3=Q022.q1;
	float y3=Q022.q2;
	float z3=Q022.q3;

	x3+=x2;
	y3+=y2;
	z3+=z2;

	Global_4.x=x3;
	Global_4.y=y3;
	Global_4.z=z3;

	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x2,y2,z2);
	glVertex3f(x3,y3,z3); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 0.0f, 1.0f); 
	glTranslatef(x3,y3,z3);
	glutSolidSphere(28, 20, 20);
	glPopMatrix();//弹出上次保存的位置
}

void GLB_FigS_Build( float q10,float q11,float q12,float q13,
	                 float q20,float q21,float q22,float q23,
					 float q30,float q31,float q32,float q33,
					 float q40,float q41,float q42,float q43,
					 float q50,float q51,float q52,float q53,
					 float pos_x,float pos_y,float pos_z

	)
{
	GL_Quater Qt0;

	Qt0.q0=q10;
	Qt0.q1=q11;
	Qt0.q2=q12;
	Qt0.q3=q13;
	Fig_GL[0].z=220;
	GLB_Fig_BuildOne(Fig_GL[0],Qt0,
		GL_HandR.Thumb_Global[1],GL_HandR.Thumb_Global[2],GL_HandR.Thumb_Global[3],GL_HandR.Thumb_Global[4],
		pos_x,pos_y,pos_z,0);
	Qt0.q0=q20;
	Qt0.q1=q21;
	Qt0.q2=q22;
	Qt0.q3=q23;
	Fig_GL[1].z=220;
	GLB_Fig_BuildOne(Fig_GL[1],Qt0,
		GL_HandR.Index_Global[1],GL_HandR.Index_Global[2],GL_HandR.Index_Global[3],GL_HandR.Index_Global[4],
		pos_x,pos_y,pos_z,1);


	Qt0.q0=q30;
	Qt0.q1=q31;
	Qt0.q2=q32;
	Qt0.q3=q33;
	Fig_GL[2].z=250;
	GLB_Fig_BuildOne(Fig_GL[2],Qt0,
		GL_HandR.Mid_Global[1],GL_HandR.Mid_Global[2],GL_HandR.Mid_Global[3],GL_HandR.Mid_Global[4],
		pos_x,pos_y,pos_z,2);

	Qt0.q0=q40;
	Qt0.q1=q41;
	Qt0.q2=q42;
	Qt0.q3=q43;
	Fig_GL[3].z=210;
	GLB_Fig_BuildOne(Fig_GL[3],Qt0,
		GL_HandR.Ring_Global[1],GL_HandR.Ring_Global[2],GL_HandR.Ring_Global[3],GL_HandR.Ring_Global[4],
		pos_x,pos_y,pos_z,3);

	Qt0.q0=q50;
	Qt0.q1=q51;
	Qt0.q2=q52;
	Qt0.q3=q53;
	Fig_GL[4].z=180;
	GLB_Fig_BuildOne(Fig_GL[4],Qt0,
		GL_HandR.Little_Global[1],GL_HandR.Little_Global[2],GL_HandR.Little_Global[3],GL_HandR.Little_Global[4],
		pos_x,pos_y,pos_z,4);
}
/*********************************************************点乘************************************************/
int Angle_Cos_Dot(GL_Quater QtN,GL_Point Global_1,float pos_xN,float pos_yN,float pos_zN,int CDUP,int CDUR,int CDUY)
{
	//通过方向余弦角度差设计 Pitch
	Plam_NormF;

	float x0=Global_1.x;
	float y0=Global_1.y;
	float z0=Global_1.z;

	GL_Quater QtNN;
	GLB_OlA_Quater(QtNN,CDUR,CDUP,CDUY);

	GL_Quater QtNN_N=Conjugate_Q(QtNN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=10;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtNN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtNN_N);
	//坐标更新
	float x1=Q022.q1;
	float y1=Q022.q2;
	float z1=Q022.q3;

	/*x1=x1+x0;
	y1=y1+y0;
	z1=z1+z0;*/
	int angle=GLB_Angle(x1,y1,z1,Plam_NormF.x-pos_xN,Plam_NormF.y-pos_yN,Plam_NormF.z-pos_zN)/3.1415926*180;

	/*GL_Vector V1=GLB_CHAJI(x1,y1,z1,Plam_NormF.x-pos_xN,Plam_NormF.y-pos_yN,Plam_NormF.z-pos_zN);
	int angle2=GLB_Angle(V1.x,V1.y,V1.z,Plam_NormF_Right.x-pos_xN,Plam_NormF_Right.y-pos_yN,Plam_NormF_Right.z-pos_zN)/3.1415926*180;
	if(angle2<180)
	flagPP=1;*/
	return angle;
}

/*****************************************************基于相对于手的手指俯仰角的一自由度弯曲****************************************************/
//QtN 为父节点四元数

void GLB_Fig_BuildOne_OneDOF(GL_Point FigNN,GL_Quater QtN,GL_Point Global_1,float pos_xN,float pos_yN,float pos_zN,short Num,int CDUP,int CDUR,int CDUY)
{
	

	float x0=Global_1.x;
	float y0=Global_1.y;
	float z0=Global_1.z;
	GL_Quater QtNN;

	//--------------------------------
	flagPP=0;

	
	//int angle=Angle_Cos_Dot(QtN, Global_1,pos_xN, pos_yN,pos_zN, CDUP, CDUR, CDUY);

	/*if(Num==1)
	{
		CvFont font1;  
		cvInitFont( &font1, CV_FONT_VECTOR0,0.8, 0.8, 0, 2, 2);
		char buf[256];
		cvSet(FrameT1,CV_RGB(255,255,255));

		sprintf_s(buf,256,"angle:  %04d",angle);
		cvPutText(FrameT1,buf,cvPoint(10,30),&font1,CV_RGB(0,0,0));

		cvShowImage("FrameT1",FrameT1);
	}*/
	

	if(CDUP<0)CDUP=0;//手指上翻非正常 异常处理
	
	//保留 基于俯仰弯曲的一个自由度
	CDUR=0;
	CDUY=0;

	//--------------------------------

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtNN,CDUR,CDUP,CDUY);
	else
		GLB_OlA_Quater(QtNN,CDUR,CDUP,CDUY);

	QtNN=MUL_Q(QtN,QtNN);

	GL_Quater QtNN_N=Conjugate_Q(QtNN); //------ 根节点四元数 求逆

	GL_Quater Q_Point;
	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.8;

	if(Num==0)
		Q_Point.q3=FigNN.z*0.5;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	GL_Quater Q011=MUL_Q(QtNN,Q_Point);
	//------计算2次相乘
	GL_Quater Q022=MUL_Q(Q011,QtNN_N);
	//坐标更新
	float x1=Q022.q1;
	float y1=Q022.q2;
	float z1=Q022.q3;

	x1=x1+x0;
	y1=y1+y0;
	z1=z1+z0;


	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x0,y0,z0);
	glVertex3f(x1,y1,z1); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(1.0f, 0.0f, 1.0f); 
	glTranslatef(x1,y1,z1);
	glutSolidSphere(32, 20, 20);
	glPopMatrix();//弹出上次保存的位置


	//------ 拓展 Fig 1
	GL_Quater QtN1;

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtN1,CDUR,CDUP,CDUY);
	else
		GLB_OlA_Quater(QtN1,CDUR,CDUP,CDUY);

	QtNN=MUL_Q(QtNN,QtN1);

	QtNN_N=Conjugate_Q(QtNN); //------ 根节点四元数 求逆

	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.55;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.3;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtNN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtNN_N);
	//坐标更新
	float x2=Q022.q1;
	float y2=Q022.q2;
	float z2=Q022.q3;

	x2+=x1;
	y2+=y1;
	z2+=z1;

	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x2,y2,z2);
	glVertex3f(x1,y1,z1); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 1.0f, 0.0f); 
	glTranslatef(x2,y2,z2);
	glutSolidSphere(33, 20, 20);
	glPopMatrix();//弹出上次保存的位置

	//------ 拓展 Fig 2
	GL_Quater QtN2;

	if(Num==0)//Thumb特殊处理
		GLB_OlA_Quater(QtN2,CDUR,CDUP,CDUY);
	else
		GLB_OlA_Quater(QtN2,CDUR,CDUP,CDUY);

	QtNN=MUL_Q(QtNN,QtN2);

	QtNN_N=Conjugate_Q(QtNN); //------ 根节点四元数 求逆


	Q_Point.q0=0;
	Q_Point.q1=0;
	Q_Point.q2=0;
	Q_Point.q3=FigNN.z*0.53;
	if(Num==0)
		Q_Point.q3=FigNN.z*0.3;
	//四元数点局部坐标系更新计算 Loc_New_Point=Q*Loc_Old_Point*Q_N
	//------计算1次相乘
	Q011=MUL_Q(QtNN,Q_Point);
	//------计算2次相乘
	Q022=MUL_Q(Q011,QtNN_N);
	//坐标更新
	float x3=Q022.q1;
	float y3=Q022.q2;
	float z3=Q022.q3;

	x3+=x2;
	y3+=y2;
	z3+=z2;

	glLineWidth(9);
	glColor3f(1.0f, 0.15f, 0.05f); 
	glBegin(GL_LINES);
	glVertex3f(x2,y2,z2);
	glVertex3f(x3,y3,z3); 
	glEnd();

	glPushMatrix();//储存当前视图矩阵
	glLineWidth(1); 
	glColor3f(0.0f, 0.0f, 1.0f); 
	glTranslatef(x3,y3,z3);
	glutSolidSphere(28, 20, 20);
	glPopMatrix();//弹出上次保存的位置
}

#endif