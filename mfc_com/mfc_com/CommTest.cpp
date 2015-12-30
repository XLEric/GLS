// CommTest.cpp : 实现文件
//

#include "stdafx.h"
#include "mfc_com.h"
#include "CommTest.h"
#include "afxdialogex.h"
#include "OpenGL.h"
#include "LXW_IMU_6.h"
#include "windows.h"
#include "Param_XF.h"

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include <winsock.h>
#include "Li_IMU_L.h"

//#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
/******************************************************
*基于TCP客户端程序设计 
******************************************************/
#include <iostream>
#include <string>
#include <windows.h>

long fpsnum=0;
float Q0=1;//OpenGL四元数
float Q1=0;
float Q2=0;
float Q3=0;

float Q0b=1;//OpenGL四元数
float Q1b=0;
float Q2b=0;
float Q3b=0;

float Q0r=1;//传感器四元数
float Q1r=0;
float Q2r=0;
float Q3r=0;

bool flag_com=0;
bool flag_Ncommon=0;
bool flag_connect=0;

double timex=0;

long int start_time;
long int end_time;

int LXW_control=0;
short Cnt_flag=0;

GLfloat Mccx[16]={0};

bool flag_axyz[3]={0};

int G_ax=0,G_ay=0,G_az=0;
float G0_Pitch=0,G0_Roll=0;
CvCapture* capture1,*capture2;
IplImage* frame1=cvCreateImage(cvSize(320,240) , 8 , 3);
IplImage* frame2=cvCreateImage(cvSize(320,240) , 8 , 3);

IplImage* frameGL=cvCreateImage(cvSize(320,240) , 8 , 3);
bool flag_Init=0;
/*初始化套接字*/
WSADATA wsa;
SOCKET s ;//= socket(AF_INET,SOCK_STREAM,0);
struct sockaddr_in sa;
char sendBuf[1024];

bool Compass_Flag_OffsetG=0;
// CCommTest 对话框

IMPLEMENT_DYNAMIC(CCommTest, CDialog)

CCommTest::CCommTest(CWnd* pParent /*=NULL*/)
	: CDialog(CCommTest::IDD, pParent)
	, m_EditSend(_T(""))
	, m_EditReceive(_T(""))
	, C_Time(_T(""))
{

}

CCommTest::~CCommTest()
{
}

void CCommTest::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_MSCOMM1, m_mscomm);
	DDX_Text(pDX, IDC_EDIT1, m_EditSend);
	DDX_Text(pDX, IDC_EDIT2, m_EditReceive);
	DDX_Text(pDX, IDC_EDIT3, C_Time);
}


BEGIN_MESSAGE_MAP(CCommTest, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE, &CCommTest::OnBnClickedButtonClose)
	ON_BN_CLICKED(IDC_BUTTON_SEND, &CCommTest::OnBnClickedButtonSend)
	ON_BN_CLICKED(IDC_BUTTON_OPEN, &CCommTest::OnBnClickedButtonOpen)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON1, &CCommTest::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CCommTest::OnBnClickedButton2)
END_MESSAGE_MAP()


// CCommTest 消息处理程序
BEGIN_EVENTSINK_MAP(CCommTest, CDialog)
	ON_EVENT(CCommTest, IDC_MSCOMM1, 1, CCommTest::OnCommMscomm1, VTS_NONE)
END_EVENTSINK_MAP()

void CCommTest::OnCommMscomm1()
{
	// TODO: 在此处添加消息处理程序代码
	static unsigned int cnt=0;
	VARIANT variant_inp;
	COleSafeArray safearray_inp;
	long len,k;
	unsigned int data[1024]={0};
	char rxdata[1024]; //设置 BYTE 数组
	
	CString strtemp;
	char command_a=0,command_b=0;

	unsigned char AHx[6],ALx[6];
	unsigned char AHy[6],ALy[6];
	unsigned char AHz[6],ALz[6];

	unsigned char GHx[6],GLx[6];
	unsigned char GHy[6],GLy[6];
	unsigned char GHz[6],GLz[6];

	unsigned char MHx[6],MLx[6];
	unsigned char MHy[6],MLy[6];
	unsigned char MHz[6],MLz[6];

	if(m_mscomm.get_CommEvent()==2)//值为 2 表示接收缓冲区内有字符
	{
		cnt++;
		variant_inp=m_mscomm.get_Input();//读缓冲区消息
		safearray_inp=variant_inp;///变量转换
		len=safearray_inp.GetOneDimSize();//得到有效的数据长度

		for(k=0;k<len;k++)
		{
			safearray_inp.GetElement(&k,rxdata+k);
			//帧头
			if(k==0)command_a=rxdata[k];
			if(k==1)command_b=rxdata[k];
			//---------------------------
			if(k==2)AHx[0]=rxdata[k];
			if(k==3)ALx[0]=rxdata[k];
			if(k==4)AHy[0]=rxdata[k];
			if(k==5)ALy[0]=rxdata[k];
			if(k==6)AHz[0]=rxdata[k];
			if(k==7)ALz[0]=rxdata[k];

			if(k==8)GHx[0]=rxdata[k];
			if(k==9)GLx[0]=rxdata[k];
			if(k==10)GHy[0]=rxdata[k];
			if(k==11)GLy[0]=rxdata[k];
			if(k==12)GHz[0]=rxdata[k];
			if(k==13)GLz[0]=rxdata[k];

			if(k==14)MHx[0]=rxdata[k];
			if(k==15)MLx[0]=rxdata[k];
			if(k==16)MHy[0]=rxdata[k];
			if(k==17)MLy[0]=rxdata[k];
			if(k==18)MHz[0]=rxdata[k];
			if(k==19)MLz[0]=rxdata[k];

			//---------------------------
			if(k==20)AHx[1]=rxdata[k];
			if(k==21)ALx[1]=rxdata[k];
			if(k==22)AHy[1]=rxdata[k];
			if(k==23)ALy[1]=rxdata[k];
			if(k==24)AHz[1]=rxdata[k];
			if(k==25)ALz[1]=rxdata[k];

			if(k==26)GHx[1]=rxdata[k];
			if(k==27)GLx[1]=rxdata[k];
			if(k==28)GHy[1]=rxdata[k];
			if(k==29)GLy[1]=rxdata[k];
			if(k==30)GHz[1]=rxdata[k];
			if(k==31)GLz[1]=rxdata[k];

			if(k==32)MHx[1]=rxdata[k];
			if(k==33)MLx[1]=rxdata[k];
			if(k==34)MHy[1]=rxdata[k];
			if(k==35)MLy[1]=rxdata[k];
			if(k==36)MHz[1]=rxdata[k];
			if(k==37)MLz[1]=rxdata[k];

			//---------------------------
			if(k==38)AHx[2]=rxdata[k];
			if(k==39)ALx[2]=rxdata[k];
			if(k==40)AHy[2]=rxdata[k];
			if(k==41)ALy[2]=rxdata[k];
			if(k==42)AHz[2]=rxdata[k];
			if(k==43)ALz[2]=rxdata[k];

			if(k==44)GHx[2]=rxdata[k];
			if(k==45)GLx[2]=rxdata[k];
			if(k==46)GHy[2]=rxdata[k];
			if(k==47)GLy[2]=rxdata[k];
			if(k==48)GHz[2]=rxdata[k];
			if(k==49)GLz[2]=rxdata[k];

			if(k==50)MHx[2]=rxdata[k];
			if(k==51)MLx[2]=rxdata[k];
			if(k==52)MHy[2]=rxdata[k];
			if(k==53)MLy[2]=rxdata[k];
			if(k==54)MHz[2]=rxdata[k];
			if(k==55)MLz[2]=rxdata[k];
			
			//---------------------------
			if(k==56)AHx[3]=rxdata[k];
			if(k==57)ALx[3]=rxdata[k];
			if(k==58)AHy[3]=rxdata[k];
			if(k==59)ALy[3]=rxdata[k];
			if(k==60)AHz[3]=rxdata[k];
			if(k==61)ALz[3]=rxdata[k];

			if(k==62)GHx[3]=rxdata[k];
			if(k==63)GLx[3]=rxdata[k];
			if(k==64)GHy[3]=rxdata[k];
			if(k==65)GLy[3]=rxdata[k];
			if(k==66)GHz[3]=rxdata[k];
			if(k==67)GLz[3]=rxdata[k];

			if(k==68)MHx[3]=rxdata[k];
			if(k==69)MLx[3]=rxdata[k];
			if(k==70)MHy[3]=rxdata[k];
			if(k==71)MLy[3]=rxdata[k];
			if(k==72)MHz[3]=rxdata[k];
			if(k==73)MLz[3]=rxdata[k];

			//---------------------------
			if(k==74)AHx[4]=rxdata[k];
			if(k==75)ALx[4]=rxdata[k];
			if(k==76)AHy[4]=rxdata[k];
			if(k==77)ALy[4]=rxdata[k];
			if(k==78)AHz[4]=rxdata[k];
			if(k==79)ALz[4]=rxdata[k];

			if(k==80)GHx[4]=rxdata[k];
			if(k==81)GLx[4]=rxdata[k];
			if(k==82)GHy[4]=rxdata[k];
			if(k==83)GLy[4]=rxdata[k];
			if(k==84)GHz[4]=rxdata[k];
			if(k==85)GLz[4]=rxdata[k];

			if(k==86)MHx[4]=rxdata[k];
			if(k==87)MLx[4]=rxdata[k];
			if(k==88)MHy[4]=rxdata[k];
			if(k==89)MLy[4]=rxdata[k];
			if(k==90)MHz[4]=rxdata[k];
			if(k==91)MLz[4]=rxdata[k];

			//---------------------------
			if(k==92)AHx[5]=rxdata[k];
			if(k==93)ALx[5]=rxdata[k];
			if(k==94)AHy[5]=rxdata[k];
			if(k==95)ALy[5]=rxdata[k];
			if(k==96)AHz[5]=rxdata[k];
			if(k==97)ALz[5]=rxdata[k];

			if(k==98)GHx[5]=rxdata[k];
			if(k==99)GLx[5]=rxdata[k];
			if(k==100)GHy[5]=rxdata[k];
			if(k==101)GLy[5]=rxdata[k];
			if(k==102)GHz[5]=rxdata[k];
			if(k==103)GLz[5]=rxdata[k];

			if(k==104)MHx[5]=rxdata[k];
			if(k==105)MLx[5]=rxdata[k];
			if(k==106)MHy[5]=rxdata[k];
			if(k==107)MLy[5]=rxdata[k];
			if(k==108)MHz[5]=rxdata[k];
			if(k==109)MLz[5]=rxdata[k];

		}
		flag_connect=0;
		if(command_a=='W' && command_b=='X')//帧头校验
		{
			
			//-------------------- 解码 ------------------

			Got_Data_L(ATL0,0,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			Got_Data_L(ATL1,1,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			Got_Data_L(ATL2,2,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			Got_Data_L(ATL3,3,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			Got_Data_L(ATL4,4,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			Got_Data_L(ATL5,5,AHx,ALx,AHy,ALy,AHz,ALz,GHx,GLx,GHy,GLy,GHz,GLz,MHx,MLx,MHy,MLy,MHz,MLz);
			
			Compass_X=ATL0.mx;
			Compass_Y=ATL0.my;
			Compass_Z=ATL0.mz;
			////--------------------------------------

			//if(Compass_Start && abs(Compass_X)<1286  && abs(Compass_Y)<1286  && abs(Compass_Z)<1286)
			//{
			//	FILE *file=fopen(".//Glove//Data0.txt","a+");
			//	{
			//		//fprintf(file,"(%d,%d,%d) (%f,%f,%f) (%d,%d,%d)\n",G_ax,G_ay,G_az,Rotx,Roty,Rotz,Compass_X,Compass_Y,Compass_Z);
			//		fprintf(file,"(%d,%d,%d)\n",Compass_X,Compass_Y,Compass_Z);
			//	}
			//	fclose(file);
			//}
			//--------------------------------------

			flag_connect=1;//通信 正确标志
			SetDlgItemText(IDC_EDIT2," ");
			UpdateData(true);

			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("Fig0:%f"),ATL5.ax);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL5.ay);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL5.az);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			//------------------------------------
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("Fig1:%f"),ATL5.ax);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL5.ay);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL5.az);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行


			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("Cmps:%f"),ATL1.mx);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL1.my);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行
			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("----:%f"),ATL1.mz);
			m_EditReceive+=strtemp;
			strtemp=_T("\r\n");//换行

			strtemp=_T("\r\n");//换行
			m_EditReceive+=strtemp;
			strtemp.Format(_T("------>>>\r\n"));
			m_EditReceive+=strtemp;
			

			if(!Compass_Start)
			{
				strtemp=_T("\r\n");//换行
				strtemp=_T("\r\n");//换行
				m_EditReceive+=strtemp;
				strtemp.Format(_T("Non start"));
				m_EditReceive+=strtemp;
			}
			else
			{
				strtemp=_T("\r\n");//换行
				strtemp=_T("\r\n");//换行
				m_EditReceive+=strtemp;
				strtemp.Format(_T("Start!"));
				m_EditReceive+=strtemp;
			}


		}

		CString temp2;
		end_time=clock();
		timex=float(end_time-start_time)/1000;
		temp2.Format("%f",timex);C_Time=temp2;
		UpdateData(false);//显示编辑框的内容
		//LXW_control++;
		start_time=clock();
		UpdateData(FALSE);//更新编辑框内容 

	}

	flag_com=1;
	//m_mscomm.put_InputLen(0);

}

void CCommTest::OnBnClickedButtonClose()
{
	// TODO: 在此添加控件通知处理程序代码
	m_mscomm.put_PortOpen(FALSE);//关闭串口
	AfxMessageBox(_T("串口1已关闭"));
}

void CCommTest::OnBnClickedButtonSend()
{
	// TODO: 在此添加控件通知处理程序代码
#if 0
	CString str;
	str="a";
	
	CByteArray hexdata;//发送的数据	
    hexdata.Add(0x31);

	UpdateData(true);//读取编辑框内容
	m_mscomm.put_Output(COleVariant(hexdata));//发送数据
	m_EditSend.Empty();//发送后清空输入框
	UpdateData(false);//更新编辑框内容
#endif
}


void CCommTest::OnBnClickedButtonOpen()
{
	// TODO: 在此添加控件通知处理程序代码
	//CString cstr[10]={"COM1","COM2","COM3","COM4","COM5","COM6","COM7","COM8","COM9"};
	const short com_num[10]={1,2,3,4,5,6,7,8,9,10};
	bool com_enable[10]={0,0,0,0,0,0,0,0,0,0};
	short Com_Num_En=0;
	HANDLE M_COM;
	CString com[10]={"COM1","COM2","COM3","COM4","COM5","COM6","COM7","COM8","COM9","COM10"};
	CString str="";
	int cnt=0;
	for(int i=0;i<10;i++)
	{
		M_COM=CreateFile(com[i],GENERIC_READ | GENERIC_WRITE,0,NULL,OPEN_EXISTING,
			             FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,NULL);
		if(M_COM==INVALID_HANDLE_VALUE)// 如果没有该设备，或者被其他应用程序在用    *******************
		{ 
		//str+=com[i];// 记录下该串口名称，以备后面提示用
		//  str+=" ";

		  com_enable[i]=0;
		  
		}
		else
		{
			cnt=i+1;// 如果存在，则记录下来。这里只记录了一个，也可以采用一个数组来记录所有存在串口
			com_enable[i]=1;
			Com_Num_En=i+1;
		}
		CloseHandle(M_COM);
	}
	//------------------------------------------------------------
	
	
		if(m_mscomm.get_PortOpen())//如果串口是打开的，则行关闭串口
		{m_mscomm.put_PortOpen(FALSE);}
		
		m_mscomm.put_CommPort(Com_Num_En);//选择COM1
		m_mscomm.put_InBufferSize(1024);//接收缓冲区
		m_mscomm.put_OutBufferSize(1024);//发送缓冲区 
		m_mscomm.put_InputLen(0);//设置当前接收区数据长度为0,表示全部读取
		m_mscomm.put_InputMode(1);//以二进制方式读写数据
		m_mscomm.put_RThreshold(38+36+36);//(26+24);//26+36//接收缓冲区有1个及1个以上字符时，将引发接收数据的OnComm事件
		//m_mscomm.put_SThreshold(0);          //每发送一个字符时，不触发OnComm事件
		m_mscomm.put_Settings(_T("128000,n,8,1"));//波特率9600无检验位，8个数据位，1个停止位
		if(!m_mscomm.get_PortOpen())//如果串口没有打开则打开
		{
			m_mscomm.put_PortOpen(TRUE);//打开串口
			CString str;
			str.Format(_T("串口%d打开成功"),Com_Num_En);
			AfxMessageBox(str);
			
		}
		else
		{
			m_mscomm.put_OutBufferCount(0);
			AfxMessageBox(_T("串口打开失败"));
		}
	
}


void CCommTest::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	
#if 1
	if(flag_Init)
	{
		
			Element_intL();
			flag_Init=0;
	}

	if(flag_com==0 )
	{
		CByteArray hexdata;//发送的数据	
		hexdata.Add(0x43);
		hexdata.Add(0xa1);
		hexdata.Add(0x0d);
		hexdata.Add(0x0a);
		m_mscomm.put_Output(COleVariant(hexdata));//发送数据
		//Sleep(6);
	}
#if 0
	if(flag_com==1)
	{
		
		// 当不引入磁场时 需要 传感器四元数坐标系 转 OpenGL坐标系；
		//不然当有磁场矫正引入时，欧拉转四元数OlA_T_Quarter()，已经包括 传感器四元数坐标系 转 OpenGL坐标系功能。
		
		//Q0=Q0r;//传感器四元数坐标系 转 OpenGL坐标系
		//Q1=Q2r;
		//Q2=Q3r;
		//Q3=Q1r;

		//Q0=Q0r;//传感器四元数坐标系 转 OpenGL坐标系
		//Q1=Q2r;
		//Q2=Q3r;
		//Q3=Q1r;

		q0=Q0r;
		q1=Q1r;
		q2=Q2r;
		q3=Q3r;

		Quarter_T_OlA(Q0r,Q1r,Q2r,Q3r);	


	Yaw_angle_Offset0=Yaw_angle;//获取航向角矫正偏移量
	Yaw_angle-=Yaw_angle_Offset;//航向角矫正

	OlA_T_Quarter(Rool_angle,Pitch_angle,Yaw_angle);//矫正偏移后的 欧拉角转 四元数

#if 1
	//平滑
	float starting[4];
	starting[0]=q1b;
	starting[1]=q2b;
	starting[2]=q3b;
	starting[3]=q0b;

	float ending[4];
	ending[0]=q1;
	ending[1]=q2;
	ending[2]=q3;
	ending[3]=q0;
	float result[4];

	//参数越小越平滑
	slerp( result, starting, ending, 0.30999 );

	Quarter_T_OlA(result[3],result[0],result[1],result[2]);

	q0b=result[3];
	q1b=result[0];
	q2b=result[1];
	q3b=result[2];


	LXW_IMU_6B(Rool_angle,Pitch_angle,Yaw_angle);//传感器四元数坐标系 转 OpenGL坐标系
	if(abs(Rotx)>0.0065 | abs(Roty)>0.0065 | abs(Rotz)>0.0065)
	{
		Q0=q0; Q1=q1;Q2=q2;Q3=q3; 
	}
#else

	if(abs(Rotx)>0.0065 | abs(Roty)>0.0065 | abs(Rotz)>0.0065)
	{
		Q0=q0; Q1=q2;Q2=q3;Q3=q1; //传感器四元数坐标系 转 OpenGL坐标系
	}

#endif
	
			//四元数 角
		float w=acos(Q0)*2*180/PI;//---- 融合 磁偏角 q0 改为 qq0

			//// 旋转角度
			//float s=acos(Q0)*2;
			//// 之前 参数 q1  q3 -q2
			//// 之前 参数 -q2  q3 -q1
			//float x=Q1;
			//float y=Q2;
			//float z=Q3;

			////旋转 向量单位化
			//float length = sqrt( x * x + y * y + z * z );
			//x /= length;
			//y /= length;
			//z /= length;
			//
			////------ 旋转矩阵求得
			//Mccx[0]=(1-cos(s))*x*x+cos(s);
			//Mccx[1]=(1-cos(s))*x*y+sin(s)*z;
			//Mccx[2]=(1-cos(s))*x*z-sin(s)*y;
			//Mccx[3]=0;

			//Mccx[4]=(1-cos(s))*x*y-sin(s)*z;
			//Mccx[5]=(1-cos(s))*y*y+cos(s);
			//Mccx[6]=(1-cos(s))*y*z+sin(s)*x;
			//Mccx[7]=0;

			//Mccx[8]=(1-cos(s))*x*z+sin(s)*y;
			//Mccx[9]=(1-cos(s))*y*z-sin(s)*x;
			//Mccx[10]=(1-cos(s))*z*z+cos(s);
			//Mccx[11]=0;

			//Mccx[12]=0;
			//Mccx[13]=0;
			//Mccx[14]=0;
			//Mccx[15]=1;
		/****************************传输**************************/
		memset(sendBuf,'\0',sizeof(sendBuf));// 传输 TCP 数据
		//39
		sprintf(sendBuf, "LiID:Q4:39:%6d %6d %6d %6d\n",int(Q0*10000),int(Q1*10000),int(Q2*10000),int(Q3*10000));
		//（22+48+8）/2=39
		send(s,sendBuf,strlen(sendBuf),0);

		
		/****************************传输**************************/


		m_openGL.RenderGLScene(w,Q0,Q1,Q2,Q3,Hx,Hy,Rool_angle,Pitch_angle, Yaw_angle,
			                   Mccx,1,pos_x_Glob,pos_y_Glob,pos_z_Glob,Yaw_angle_Offset0,Yaw_angle_Offset,
			                   Compass_Angle,0,Compass_Start,Compass_X,Compass_Y,Compass_Z,flag_connect); //---- 融合 磁偏角 q1 q2 q3  改为 qq1 qq2 qq3
		flag_com=0;	    

	}
#else
if(flag_com==1)
{
	//if(abs(ATL0.gx)>1/PI && abs(ATL0.gy)>1/PI && abs(ATL0.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL0,timex/2);
	Quarter_T_OlA_L(ATL0);	
	Yaw_angle_Offset0=ATL0.Yaw_angle;//获取航向角矫正偏移量
	ATL0.Yaw_angle-=ATL0.Yaw_angle_Offset;//航向角矫正
	OlA_T_Quarter_L(ATL0);
	}

	//if(abs(ATL1.gx)>1/PI && abs(ATL1.gy)>1/PI && abs(ATL1.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL1,timex/2);
	Quarter_T_OlA_L(ATL1);	
	Yaw_angle_Offset00=ATL1.Yaw_angle;//获取航向角矫正偏移量
	ATL1.Yaw_angle-=ATL1.Yaw_angle_Offset;//航向角矫正
	OlA_T_Quarter_L(ATL1);
	}

	//if(abs(ATL2.gx)>1/PI && abs(ATL2.gy)>1/PI && abs(ATL2.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL2,timex/2);
	Quarter_T_OlA_L(ATL2);	
	Yaw_angle_Offset2=ATL2.Yaw_angle;//获取航向角矫正偏移量
	ATL2.Yaw_angle-=ATL2.Yaw_angle_Offset;//航向角矫正
	OlA_T_Quarter_L(ATL2);
	}

	//if(abs(ATL3.gx)>1/PI && abs(ATL3.gy)>1/PI && abs(ATL3.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL3,timex/2);
		Quarter_T_OlA_L(ATL3);	
		Yaw_angle_Offset3=ATL3.Yaw_angle;//获取航向角矫正偏移量
		ATL3.Yaw_angle-=ATL3.Yaw_angle_Offset;//航向角矫正
		OlA_T_Quarter_L(ATL3);
	}
	

	//if(abs(ATL4.gx)>1/PI && abs(ATL4.gy)>1/PI && abs(ATL4.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL4,timex/2);
		Quarter_T_OlA_L(ATL4);	
		Yaw_angle_Offset4=ATL4.Yaw_angle;//获取航向角矫正偏移量
		ATL4.Yaw_angle-=ATL4.Yaw_angle_Offset;//航向角矫正
		OlA_T_Quarter_L(ATL4);
	}
	

	//if(abs(ATL5.gx)>1/PI && abs(ATL5.gy)>1/PI && abs(ATL5.gz)>1/PI)
	{
		IMU_AHRSupdateComPass(ATL5,timex/2);
		Quarter_T_OlA_L(ATL5);	
		Yaw_angle_Offset5=ATL5.Yaw_angle;//获取航向角矫正偏移量
		ATL5.Yaw_angle-=ATL5.Yaw_angle_Offset;//航向角矫正
		OlA_T_Quarter_L(ATL5);
	}
	

	//-------------------------------------------

	//平滑
	
	Smooth_GL_L(ATL0);
	Smooth_GL_L(ATL1);
	Smooth_GL_L(ATL2);
	Smooth_GL_L(ATL3);
	Smooth_GL_L(ATL4);
	Smooth_GL_L(ATL5);

	//-------------------------------------------
	Q0=ATL0.QGL[0];
	Q1=ATL0.QGL[1];
	Q2=ATL0.QGL[2];
	Q3=ATL0.QGL[3];

	float Q10=ATL1.QGL[0];
	float Q11=ATL1.QGL[1];
	float Q12=ATL1.QGL[2];
	float Q13=ATL1.QGL[3];

	float Q20=ATL2.QGL[0];
	float Q21=ATL2.QGL[1];
	float Q22=ATL2.QGL[2];
	float Q23=ATL2.QGL[3];

	float Q30=ATL3.QGL[0];
	float Q31=ATL3.QGL[1];
	float Q32=ATL3.QGL[2];
	float Q33=ATL3.QGL[3];

	float Q40=ATL4.QGL[0];
	float Q41=ATL4.QGL[1];
	float Q42=ATL4.QGL[2];
	float Q43=ATL4.QGL[3];

	float Q50=ATL5.QGL[0];
	float Q51=ATL5.QGL[1];
	float Q52=ATL5.QGL[2];
	float Q53=ATL5.QGL[3];

	//-------------------------------------------------------------------
	//GL_OLA_T_Quarter_L(ATL0);
	for(int i=0;i<5;i++)
	{
		CDU_P[i]=0;
		CDU_R[i]=0;
		CDU_Y[i]=0;
	}
	
	GL_Quarter_GLobal_T_Local(ATL0,ATL1,CDU_P[0],CDU_R[0],CDU_Y[0]);//获取局部坐标
	GL_Quarter_GLobal_T_Local(ATL0,ATL2,CDU_P[1],CDU_R[1],CDU_Y[1]);//获取局部坐标
	GL_Quarter_GLobal_T_Local(ATL0,ATL3,CDU_P[2],CDU_R[2],CDU_Y[2]);//获取局部坐标
	GL_Quarter_GLobal_T_Local(ATL0,ATL4,CDU_P[3],CDU_R[3],CDU_Y[3]);//获取局部坐标
	GL_Quarter_GLobal_T_Local(ATL0,ATL5,CDU_P[4],CDU_R[4],CDU_Y[4]);//获取局部坐标

	/*if(CDU_P[1]<0)CDU_P[1]=0;
	if(CDU_P[2]<0)CDU_P[2]=0;
	if(CDU_P[3]<0)CDU_P[3]=0;
	if(CDU_P[4]<0)CDU_P[4]=0;

	for(int i=0;i<5;i++)
	CDU_R[i]=0;


	for(int k=0;k<5;k++)
	{
		for(int i=2;i>0;i--)
		{
			CDU_P_Reg[k][i]=CDU_P_Reg[k][i-1];
		}

		CDU_P_Reg[k][0]=CDU_P[k];
	}

	int CDU_P_Sum[5]={0,0,0,0,0};
	for(int k=0;k<5;k++)
	{
		for(int i=0;i<3;i++)
		{
			CDU_P_Sum[k]+=CDU_P_Reg[k][i];
		}
		CDU_P[k]=CDU_P_Sum[k]/3;
	}*/
	
	


	CvFont font1;  
	cvInitFont( &font1, CV_FONT_VECTOR0,0.8, 0.8, 0, 2, 2);
	char buf[256];
	//cvSet(frameGL,CV_RGB(255,255,255));

	//sprintf_s(buf,256,"YRP1:%04d %04d %04d",ATL1.Yaw_Local_angle_GL,ATL1.Rool_Local_angle_GL,ATL1.Pitch_Local_angle_GL);
	//cvPutText(frameGL,buf,cvPoint(10,30),&font1,CV_RGB(0,0,0));
/*
	sprintf_s(buf,256,"YRP2:%04d %04d %04d",ATL2.Yaw_Local_angle_GL,ATL2.Rool_Local_angle_GL,ATL2.Pitch_Local_angle_GL);
	cvPutText(frameGL,buf,cvPoint(10,60),&font1,CV_RGB(0,0,0)); 

	sprintf_s(buf,256,"YRP3:%04d %04d %04d",ATL3.Yaw_Local_angle_GL,ATL3.Rool_Local_angle_GL,ATL3.Pitch_Local_angle_GL);
	cvPutText(frameGL,buf,cvPoint(10,90),&font1,CV_RGB(0,0,0)); 
	cvShowImage("CVGL_SHOW",frameGL);

	sprintf_s(buf,256,"YRP4:%04d %04d %04d",ATL4.Yaw_Local_angle_GL,ATL4.Rool_Local_angle_GL,ATL4.Pitch_Local_angle_GL);
	cvPutText(frameGL,buf,cvPoint(10,120),&font1,CV_RGB(0,0,0)); 

	sprintf_s(buf,256,"YRP5:%04d %04d %04d",ATL5.Yaw_Local_angle_GL,ATL5.Rool_Local_angle_GL,ATL5.Pitch_Local_angle_GL);
	cvPutText(frameGL,buf,cvPoint(10,150),&font1,CV_RGB(0,0,0)); 
	*/

//cvShowImage("CVGL_SHOW",frameGL);


	//-------------------------------------------------------------------
	/****************************传输**************************/
	//四元数 角
	float w=acos(Q0)*2*180/PI;//---- 融合 磁偏角 q0 改为 qq0
	float w1=acos(Q10)*2*180/PI;//---- 融合 磁偏角 q0 改为 qq0
	float w2=acos(Q20)*2*180/PI;
	float w3=acos(Q30)*2*180/PI;
	float w4=acos(Q40)*2*180/PI;
	float w5=acos(Q50)*2*180/PI;


	//FILE *file=fopen(".//Glove//Quarter.txt","a+");
	//{
	//	fprintf(file,"%f,%f,%f,%f",Q0,Q1,Q2,Q3);
	//	fprintf(file,"%f,%f,%f,%f",Q10,Q11,Q12,Q13);
	//	fprintf(file,"%f,%f,%f,%f",Q20,Q21,Q22,Q23);
	//	fprintf(file,"%f,%f,%f,%f",Q30,Q31,Q32,Q33);
	//	fprintf(file,"%f,%f,%f,%f",Q40,Q41,Q42,Q43);
	//	fprintf(file,"%f,%f,%f,%f",Q50,Q51,Q52,Q53);
	//	fprintf(file,"\r\n");
	//}
	//fclose(file);

	m_openGL.RenderGLScene(w,Q0,Q1,Q2,Q3, w1,Q10,Q11,Q12,Q13, w2,Q20,Q21,Q22,Q23, w3,Q30,Q31,Q32,Q33,
		                   w4,Q40,Q41,Q42,Q43,w5,Q50,Q51,Q52,Q53,
						   flag_Init,
		CDU_P,CDU_R,CDU_Y,
		Hx,Hy,Rool_angle,Pitch_angle, Yaw_angle,
		Mccx,1,pos_x_Glob,pos_y_Glob,pos_z_Glob,
		Yaw_angle_Offset0,ATL0.Yaw_angle_Offset,
		Yaw_angle_Offset00,ATL1.Yaw_angle_Offset,
		Yaw_angle_Offset2,ATL2.Yaw_angle_Offset,
		Yaw_angle_Offset3,ATL3.Yaw_angle_Offset,
		Yaw_angle_Offset4,ATL4.Yaw_angle_Offset,
		Yaw_angle_Offset5,ATL5.Yaw_angle_Offset,


		Compass_Angle,0,Compass_Start,Compass_X,Compass_Y,Compass_Z,flag_connect,
		Compass_Flag_OffsetG
		); //---- 融合 磁偏角 q1 q2 q3  改为 qq1 qq2 qq3
	Compass_Flag_OffsetG=0;
	flag_com=0;	    

}
#endif
	
#endif

	CDialog::OnTimer(nIDEvent);
}


void CCommTest::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	

	SetTimer(0,18,NULL);
	//CByteArray hexdata;//发送的数据	
	//hexdata.Add(0x43);
	//hexdata.Add(0xa1);
	//hexdata.Add(0x0d);
	//hexdata.Add(0x0a);
	//m_mscomm.put_Output(COleVariant(hexdata));//发送数据
}

void CCommTest::MouseAction(unsigned char m_get)  //控制鼠标动作主程序
{
#if 0
	if (m_get==0xa1)          //左移
	{
		pt.x-=2;
		::SetCursorPos(pt.x,pt.y);
		::GetCursorPos(&pt);
	}
	else if (m_get==0xa2)    //右移
	{
		pt.x+=2;
		::SetCursorPos(pt.x,pt.y);
		::GetCursorPos(&pt);
	}
	else if (m_get==0xa3)    //上移
	{
		pt.y-=2;
		::SetCursorPos(pt.x,pt.y);
		::GetCursorPos(&pt);
	}
	else if (m_get==0xa4)    //下移
	{
		pt.y+=2;
		::SetCursorPos(pt.x,pt.y);
		::GetCursorPos(&pt);
	}
	else if (m_get==0xb1)   //左键单击
	{

		mouse_event (MOUSEEVENTF_LEFTDOWN, pt.x, pt.y, 0, 0 );
		mouse_event (MOUSEEVENTF_LEFTUP, pt.x, pt.y, 0, 0 );

		

	}
	else if (m_get==0xb2)    //右键单击
	{

		mouse_event (MOUSEEVENTF_RIGHTDOWN, pt.x, pt.y, 0, 0 );
		mouse_event (MOUSEEVENTF_RIGHTUP, pt.x, pt.y, 0, 0 );

	}
	else if (m_get==0xb4)    //鼠标双击
	{
		mouse_event (MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, pt.x, pt.y, 0, 0 );
		mouse_event (MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, pt.x, pt.y, 0, 0 );


	}
#endif
}

BOOL CCommTest::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  在此添加额外的初始化
	 CRect rect;
	GetDlgItem(IDC_OPENGL)->GetWindowRect(rect);
	ScreenToClient(rect);
	m_openGL.Create(rect, this); 

	//------------------------------------------
#if 0
	//Socket
	/*初始化套接字*/
	wsa;
	WSAStartup(MAKEWORD(2,0),&wsa);
	s = socket(AF_INET,SOCK_STREAM,0);
	if(s == INVALID_SOCKET)
	{
		printf("创建套接字失败!\n");
		WSACleanup();
		return 1;
	}
	/*填写本地数据*/
	struct sockaddr_in sa;
	sa.sin_family = AF_INET;
	sa.sin_port = htons(8080);
	sa.sin_addr.S_un.S_addr = inet_addr("192.168.1.142");
	int nAddrlen = sizeof(sa);

	/*等待被连接*/
	if (connect(s,(sockaddr *)(&sa),nAddrlen) == -1)
	{
		printf("连接服务器失败!\n");
		WSACleanup();
		return -1;
	}

	
	memset(sendBuf,'\0',sizeof(sendBuf));
	strcpy(sendBuf,"I'm from client\n");
	int n = send(s,sendBuf,strlen(sendBuf),0);

#endif

	//--------------------------------------------
	Element_intL();

	//--------------------------------------------

	//capture1 = cvCreateCameraCapture( 0 );
	//capture2 = cvCreateCameraCapture( 1 );
	//int w = 320, h = 240;
	//cvSetCaptureProperty ( capture1, CV_CAP_PROP_FRAME_WIDTH,  w );  
	//cvSetCaptureProperty ( capture1, CV_CAP_PROP_FRAME_HEIGHT, h );
	//cvSetCaptureProperty ( capture2, CV_CAP_PROP_FRAME_WIDTH,  w );  
	//cvSetCaptureProperty ( capture2, CV_CAP_PROP_FRAME_HEIGHT, h );

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


//HRESULT CCommTest::accDoDefaultAction(VARIANT varChild)
//{
//	// TODO: 在此添加专用代码和/或调用基类
//
//	return CDialog::accDoDefaultAction(varChild);
//}


void CCommTest::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	Compass_Flag_OffsetG=1;
}
