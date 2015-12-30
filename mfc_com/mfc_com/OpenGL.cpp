// OpenGL.cpp : implementation file
//

#include "stdafx.h"
//#include "practice.h"
#include "OpenGL.h"
#include"iostream"
#include <gl/glut.h>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "CvvImage.h"
#include "GLB_Math.h"


using namespace cv;
using namespace std;

#define Move_Thr 5

IplImage* Frame=cvCreateImage(cvSize(900,900) , 8 , 3);

CvFont font;  
char buf_T[256];

unsigned char show_wordx[]=
{	48 ,49, 50, 51, 52, 53, 54, 55, 56, 57,  // 数字
};


bool flag_clear=0;

GLfloat Mc[16];

bool flag_dw=0,flag_up=0;
bool flag_shoot=0;
bool Flag_Start_Q=0;

short Color_Choose=0;
//------ 画线

int gl_x0[30]={0};
int gl_y0[30]={0};
int gl_z0[30]={0};

int gl_x[30]={0};
int gl_y[30]={0};
int gl_z[30]={0};

int gl_mx[30]={0};
int gl_my[30]={0};
int gl_mz[30]={0};

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// GL 视图旋转角度
short Angle_Rot=0;
//旋转角度更新计数器
unsigned char Cnt_Angle_Rot=0;
//GL 清屏 标志
bool Clear_GL_Screen=0;
typedef struct __Pos_3D__
{
	float x;
	float y;
	float z;
	float distance;
	float flag;
}Pos_3D;
// OpenGL 视野
int GL_Look_X=0;
int GL_Look_Y=500;//500;
int GL_Look_Z=-1250;//-450;

//------ 点到直线的距离
// x0 y0 z0 点
// x1 y1 z1 直线上的已知点
// xc yc zc 直线上的所求垂点
// m  n  p  直线向量
Pos_3D Distance_3D(float x0,float y0,float z0,float xc,float yc,float zc,float x1,float y1,float z1,float m, float n, float p)
{
	Pos_3D Pos_3D0;

	float t=0;
	bool flag=0;

	float x=0,y=0,z=zc;



	//直线方程  (xc-x1)/m=(yc-y1)/n=(zc-z1)/p=t;

	if(p!=0)
	{  
		t=(z-z1)/p;
		flag=1;

		x=m*t+x1;
		y=n*t+y1;
	}

	//------ 计算点到直线距离

	float t2=0;
	float distance=0;
	if((m*m+n*n+p*p)!=0)
	{
		t2=(m*(x0-x1)+n*(y0-y1)+p*(z0-z1))/(m*m+n*n+p*p);

		xc= m*t2+x1;
		yc= n*t2+y1;
		zc= p*t2+z1;


		distance=sqrt((x0-xc)*(x0-xc)+(y0-yc)*(y0-yc)+(z0-zc)*(z0-zc));
	}


	Pos_3D0.x=x;
	Pos_3D0.y=y;
	Pos_3D0.z=z;
	Pos_3D0.flag=flag;
	Pos_3D0.distance=distance;


	return Pos_3D0;
}
/////////////////////////////////////////////////////////////////////////////
// COpenGL

COpenGL::COpenGL():m_bInit(FALSE),m_bFullScreen(FALSE),
  m_hDC(NULL),m_hRC(NULL),m_parent(NULL)
{
}

COpenGL::~COpenGL()
{
 KillGLWindow(); // Shutdown
}


BEGIN_MESSAGE_MAP(COpenGL, CWnd)
 //{{AFX_MSG_MAP(COpenGL)
 ON_WM_CREATE()
 ON_WM_PAINT()
 ON_WM_SIZE()
 ON_WM_KEYDOWN()
 //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// COpenGL message handlers

void COpenGL::Create(CRect rect, CWnd *parent)
{
 if (m_bInit) return;
 ASSERT(rect);
 ASSERT(parent);
 m_rect = rect;
 m_parent = parent;

 CString className = AfxRegisterWndClass(
  CS_HREDRAW | CS_VREDRAW | CS_OWNDC,
  NULL,
  (HBRUSH)GetStockObject(BLACK_BRUSH),
  NULL);
 
 CreateEx(
  0,
  className,
  _T("OpenGL"),
  WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
  rect,
  parent,
  0);
}

int COpenGL::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
 if (CWnd::OnCreate(lpCreateStruct) == -1)
  return -1;
 
 // TODO: Add your specialized creation code here
 EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &m_DMsaved);


 GLuint PixelFormat;            // Holds The Results After Searching For A Match
 static PIXELFORMATDESCRIPTOR pfd=         // pfd Tells Windows How We Want Things To Be
 {                  
  sizeof(PIXELFORMATDESCRIPTOR),         // Size Of This Pixel Format Descriptor
   1,                // Version Number
   PFD_DRAW_TO_WINDOW |           // Format Must Support Window
   PFD_SUPPORT_OPENGL |           // Format Must Support OpenGL
   PFD_DOUBLEBUFFER,            // Must Support Double Buffering
   PFD_TYPE_RGBA,             // Request An RGBA Format
   m_DMsaved.dmBitsPerPel,            // Select Our Color Depth
   0, 0, 0, 0, 0, 0,            // Color Bits Ignored
   0,                // No Alpha Buffer
   0,                // Shift Bit Ignored
   0,                // No Accumulation Buffer
   0, 0, 0, 0,              // Accumulation Bits Ignored
   16,                // 16Bit Z-Buffer (Depth Buffer) 
   0,                // No Stencil Buffer
   0,                // No Auxiliary Buffer
   PFD_MAIN_PLANE,             // Main Drawing Layer
   0,                // Reserved
   0, 0, 0               // Layer Masks Ignored
 };                  
 
 
 if ( !( m_hDC = ::GetDC ( m_hWnd ) ) ) {       // Did We Get A Device Context?                 
  KillGLWindow ();             // Reset The Display
  TRACE ( "Can't Create A GL Device Context." );
  return FALSE;            
 }
 
 if ( !( PixelFormat = ChoosePixelFormat ( m_hDC, &pfd ) ) ) {  // Did Windows Find A Matching Pixel Format?                  
  KillGLWindow ();            // Reset The Display
  TRACE ( "Can't Find A Suitable PixelFormat." );
  return FALSE;            
 }
 
 if ( !SetPixelFormat ( m_hDC, PixelFormat, &pfd ) ){    // Are We Able To Set The Pixel Format?                  
  KillGLWindow ();            // Reset The Display
  TRACE ( "Can't Set The PixelFormat." );
  return FALSE;            
 }
 
 if ( !( m_hRC = wglCreateContext ( m_hDC ) ) ) {     // Are We Able To Get A Rendering Context?                 
  KillGLWindow ();            // Reset The Display
  TRACE( " Can't Create A GL Rendering Context." );
  return FALSE;             
 }
 
 if ( !wglMakeCurrent ( m_hDC, m_hRC ) ) {       // Try To Activate The Rendering Context                  
  KillGLWindow ();            // Reset The Display
  TRACE ( "Can't Activate The GL Rendering Context." );
  return FALSE;             
 }
 
 if ( !InitGL () ) {             // Initialize Our Newly Created GL Window                  
  KillGLWindow ();            // Reset The Display
  TRACE ( "Initialization Failed." );
  return FALSE;             
 }
 m_bInit = TRUE;
 
 return 0;
}

void COpenGL::KillGLWindow()
{
 if (m_bFullScreen)               // Are We In Fullscreen Mode?
 {
  if (!ChangeDisplaySettings(NULL,CDS_TEST)) {      // if the shortcut doesn't work
   ChangeDisplaySettings(NULL,CDS_RESET);       // Do it anyway (to get the values out of the registry)
   ChangeDisplaySettings(&m_DMsaved,CDS_RESET);      // change it to the saved settings
  } else {
   ChangeDisplaySettings(NULL,CDS_RESET);
  }
  
  ShowCursor(TRUE);             // Show Mouse Pointer
 }                  
 
 if ( m_hRC ) {               // Do We Have A Rendering Context?                  
  if ( !wglMakeCurrent ( NULL, NULL ) ) {       // Are We Able To Release The DC And RC Contexts?
   TRACE ( "Release Of DC And RC Failed." );
  }
  
  if ( !wglDeleteContext ( m_hRC ) ) {        // Are We Able To Delete The RC?
   TRACE ( "Release Rendering Context Failed." );
  }
  m_hRC = NULL;              // Set RC To NULL
 }
 
 if ( m_hDC && !::ReleaseDC ( m_hWnd, m_hDC ) ) {      // Are We Able To Release The DC
  TRACE ( "Release Device Context Failed." );
  m_hDC = NULL;              // Set DC To NULL
 }
 
 if ( m_hWnd && !::DestroyWindow ( m_hWnd ) ) {       // Are We Able To Destroy The Window?
  TRACE( "Could Not Release m_hWnd." );
  m_hWnd = NULL;              // Set m_hWnd To NULL
 }
}

int COpenGL::InitGL()
{
	/*
 glShadeModel(GL_SMOOTH);            // Enable Smooth Shading
 glClearColor(0.0f, 0.0f, 0.0f, 0.5f);         // Black Background
 glClearDepth(1.0f);              // Depth Buffer Setup
 glEnable(GL_DEPTH_TEST);            // Enables Depth Testing
 glDepthFunc(GL_LEQUAL);             // The Type Of Depth Testing To Do
 glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);      // Really Nice Perspective Calculations
 glEnable(GL_TEXTURE_2D);            // Enable Texture Mapping
 */

   glViewport (0, 0, (GLsizei) 640*2, (GLsizei) 480*2); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(85.0, (GLfloat) 640*2/(GLfloat) (480*2), 1.0, 1250.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt (0, 255, -285, 0.0, 50.0, 0.0, 0.0, 1.0, 0.0);
 
 return TRUE;               // Initialization Went OK 
}

void COpenGL::RenderGLScene(float Ww,float q0,float q1,float q2,float q3,
	                        float Ww1,float q10,float q11,float q12,float q13,
	                        float Ww2,float q20,float q21,float q22,float q23,
							float Ww3,float q30,float q31,float q32,float q33,
							float Ww4,float q40,float q41,float q42,float q43,
							float Ww5,float q50,float q51,float q52,float q53,
							bool &flag_InitN,
							int *CDU_P,int *CDU_R,int *CDU_Y,
							float Hxn,float Hyn,
	                        int RoolN,int PitchN,int YawN,
							GLfloat *Mccx , bool flag_Destroy, float &pos_x,float &pos_y ,float &pos_z,
							int &Yaw_angle_OffsetN0,int &Yaw_angL0,
							int &Yaw_angle_OffsetN1,int &Yaw_angL1,
							int &Yaw_angle_OffsetN2,int &Yaw_angL2,
							int &Yaw_angle_OffsetN3,int &Yaw_angL3,
							int &Yaw_angle_OffsetN4,int &Yaw_angL4,
							int &Yaw_angle_OffsetN5,int &Yaw_angL5,
							int Compass_AngleN,
							bool flag_touch,bool &Compass_StartN,int CompassX,int CompassY,int CompassZ,bool flag_connect,
							bool Compass_Flag_Offset//磁场航向角偏移置标志位
	)
{
 if(!m_bInit) return;

 //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);   // Clear Screen And Depth Buffer
 //glClearColor( 0.8f, 0.9f, 0.8f, 1.0f );

//------------------------------------------

 glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glClearColor( 0.8f, 0.8f, 0.6f, 1.0f );

   glViewport (0, 0, (GLsizei) 400*2, (GLsizei) 320*2); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(88.0, (GLfloat) 400*2/(GLfloat) (320*2), 0.0, 2250.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   
   gluLookAt (GL_Look_X, GL_Look_Y, GL_Look_Z, 0.0, 120.0, 0.0, 0.0, 1.0, 0.0);
   //视野旋转
   Cnt_Angle_Rot++;
   if(Cnt_Angle_Rot>=5)
   {
	   Cnt_Angle_Rot=0;
	   Angle_Rot+=3;
	   if(Angle_Rot>=360)
	   {
		   Angle_Rot=0;
	   }
   }
   //glRotatef(Angle_Rot,0.0f,1.0f,0.0f);

   int i=0;
 // OpenCV按键控制
   //cvNamedWindow("OPenCV",0);
   cvSet(Frame,CV_RGB(0,0,255));
   int key=cvWaitKey(1);
   if(key=='q')pos_x+=25;
   if(key=='a')pos_x-=25;

   if(key=='w')pos_y+=25;
   if(key=='s')pos_y-=25;

   if(key=='e')pos_z+=25;
   if(key=='d')pos_z-=25;

   if(key=='o'||key=='O')Compass_StartN=1;
   if(key=='p'||key=='P')Compass_StartN=0;

   if(key=='x')GL_Look_X+=15;
   if(key=='X')GL_Look_X-=15;

   if(key=='y')GL_Look_Y+=15;
   if(key=='Y')GL_Look_Y-=15;

   if(key=='z')GL_Look_Z+=15;
   if(key=='Z')GL_Look_Z-=15;

   if(key=='c'||key=='C')
   {
		if(Color_Choose>5)
		{
			Color_Choose=0;
		}
		else
		{
			Color_Choose++;
		}
   }
   

   if(key=='i')
   {
	   if(flag_shoot==0)
		   flag_shoot=1;
	   else if(flag_shoot==1)
		   flag_shoot=0;
	   Yaw_angle_OffsetN0=0;
	   Yaw_angle_OffsetN1=0;
	   Flag_Start_Q=1;
   }

   if(key=='p'|| Compass_Flag_Offset)//航向角归零
   {
#if 1
	    Yaw_angL0=Yaw_angle_OffsetN0;
	    Yaw_angL1=Yaw_angle_OffsetN1;
		Yaw_angL2=Yaw_angle_OffsetN2;
		Yaw_angL3=Yaw_angle_OffsetN3;
		Yaw_angL4=Yaw_angle_OffsetN4;
		Yaw_angL5=Yaw_angle_OffsetN5;

#endif
		//flag_InitN=1;
	   Clear_GL_Screen=0;//GL 清屏启动
   }
 
   //------读取磁场

   float C_ModelX=0;
   float C_ModelY=0;
   float C_ModelZ=0;
   //float C_Rr=214.8073;
   //float C_Rr2=  263.0563;

   //
   ////绘制拟合球面
   ////地域A
   //glPushMatrix();//储存当前视图矩阵
   //glLineWidth(1); 
   //glColor3f(1.0f, 1.0f, 0.0f); 
   //glTranslatef(0,0,0);
   //glutWireSphere(C_Rr, 30, 30);
   //glPopMatrix();//弹出上次保存的位置

   /***********************************************************************/
   //地域B
   //glPushMatrix();//储存当前视图矩阵
   //glLineWidth(1); 
   //glColor3f(0.1f, 1.0f, 1.0f); 
   //glTranslatef(0,0,0);
   //glutWireSphere(C_Rr2, 30, 30);

   //glPopMatrix();//弹出上次保存的位置

   //绘制实时磁场球

 //  glPushMatrix();//储存当前视图矩阵
 //  glLineWidth(2); 
 //  glColor3f(0.25f, 0.85f, 1.0f); 
 //  glTranslatef(CompassY-C_ModelY,CompassZ-C_ModelZ,CompassX-C_ModelX);
 //  glutSolidSphere(20,10,10);
 //  glPopMatrix();//弹出上次保存的位置

 //  for(i=29;i>0;i--)
 //  {
	//   gl_mx[i]=gl_mx[i-1];
	//   gl_my[i]=gl_my[i-1];
	//   gl_mz[i]=gl_mz[i-1];
 //  }
 //  gl_mx[0]=CompassY-C_ModelY;
 //  gl_my[0]=CompassZ-C_ModelZ;
 //  gl_mz[0]=CompassX-C_ModelX;

 //  glLineWidth(4); 
	//glBegin(GL_LINES);//绘制居中轨迹
	//for(int i=0;i<29;i++)
	//{
	//	glColor3f(1.0f, 0.0f, 1.0f); 
	//	glVertex3f(gl_mx[i],gl_my[i],gl_mz[i]);
	//	glVertex3f(gl_mx[i+1],gl_my[i+1],gl_mz[i+1]);
	//}
	//glEnd();

   cvInitFont( &font, CV_FONT_VECTOR0,1.5, 1.2, 0, 2, 2);


   if(flag_connect)
   {
	   sprintf_s(buf_T,256,"Connected");
	   cvPutText(Frame,buf_T,cvPoint(10,800),&font,CV_RGB(0,255,255));
   }
   else
   {
	   sprintf_s(buf_T,256,"Non Connect");
	   cvPutText(Frame,buf_T,cvPoint(10,800),&font,CV_RGB(255,0,55));
   }


   sprintf_s(buf_T,256,"Y R P:%04d %04d %04d",YawN,RoolN,PitchN);
   cvPutText(Frame,buf_T,cvPoint(10,60),&font,CV_RGB(0,0,0)); 


   float r=0.0,g=0.0,b=1.0;

   switch(Color_Choose)
   {
	case 0:r=0.0;g=0.0,b=1.0;break;
	case 1:r=0.0;g=1.0,b=0.0;break;
	case 2:r=1.0;g=0.0,b=1.0;break;
	case 3:r=0.0;g=1.0,b=1.0;break;
	case 4:r=1.0;g=1.0,b=0.0;break;
	case 5:r=0.2;g=0.8,b=0.86;break;
	default:r=0.0;g=0.0;b=0.0;break;
   }

   //sprintf_s(buf_T,256,"color:%01d",Color_Choose);
   //cvPutText(Frame,buf_T,cvPoint(10,560),&font,CV_RGB(0,0,0)); 

   //FILE *pFile;
   //pFile = fopen(".//Glove//Data0.txt","r+");

   //while(!feof(pFile))
   //{
	  // int xq,yq,zq;
	  // fscanf(pFile,"(%d,%d,%d)\n",&xq,&yq,&zq);
	  // glPushMatrix();//储存当前视图矩阵
	  // glColor3f(r,g,b); 
	  // glTranslatef(yq-C_ModelY,zq-C_ModelZ,xq-C_ModelX);
	  // glutSolidSphere(4,3,3);
	  // glPopMatrix();//弹出上次保存的位置
   //}
   //fclose(pFile);



#if 1
//绘制世界坐标原点
   glPushMatrix();//储存当前视图矩阵

   glColor3f(0.0f, 1.0f, 1.0f); 
   glTranslatef(0,0,0);
   glutWireSphere(6, 10, 10);

   glPopMatrix();//弹出上次保存的位置
 /*************绘制网格线*************/
   glPushMatrix();//储存当前视图矩阵
   
   glColor3f (0.08, 0.28, 0.88);
   glLineWidth(1); 
   //画纵平面
   glBegin(GL_LINES);
   for(i=0;i<11;i++)
   {
   glVertex3f(-60.0f,i*10,0.0f);
   glVertex3f( 60.0f,i*10,0.0f); 
   }
    glEnd();
	glBegin(GL_LINES);
    for(i=-6;i<7;i++)
   {
	 glColor3f (0.08, 0.28, 0.88);
	 glLineWidth(1); 
	 if(i==0)
	 {
		 glColor3f (1.08, 0.28, 0.28);
		 glLineWidth(5); 
	 }
   glVertex3f(i*10,0.0f,0.0f);
   glVertex3f(i*10,100.0f,0.0f); 
   }
    glEnd();
	//画水平平面
	glLineWidth(1); 
	glColor3f (1.0, 0.0, 1.0);
	glBegin(GL_LINES);
    for(i=0;i<=5;i++)
   {
   glVertex3f(-60.0f,0.0f,i*10);
   glVertex3f(60.0f ,0.0f,i*10); 
   }
    glEnd();

	glBegin(GL_LINES);
    for(i=-6;i<=6;i++)
   {
   glVertex3f(i*10,0.0f,0.0f);
   glVertex3f(i*10,0.0f,50.0f); 
   }
    glEnd();
 
	glPopMatrix();//储存当前视图矩阵
#endif
	

 //////////////////////////////////////////////////////////////////////////////

	//-----------------------------------------
	//------ 空间转换矩阵
	GLfloat xx0,yy0,zz0;

	//GLfloat xx0=Mccx[0]*0 + Mccx[4]*0 - Mccx[8]*80+Mccx[12]*1;

	//GLfloat yy0=Mccx[1]*0 + Mccx[5]*0 - Mccx[9]*80+Mccx[13]*1;

	//GLfloat zz0=Mccx[2]*0 + Mccx[6]*0 - Mccx[10]*80+Mccx[14]*1;

	//---------------------

#if 0
	
	/***************************************/
	//绘制 旋转长方体 ID1
	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+300,pos_y,pos_z);         
	glRotatef(Ww1,q11,q12,q13);

	glColor3f(0.8f, 0.2f, 0.0f );
	glScaled(24,3,160);
	glutSolidCube(3);

	glPopMatrix(); 

	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+300,pos_y,pos_z);         
	glRotatef(Ww1,q11,q12,q13);

	glColor3f(0.0f, 1.0f, 0.0f );
	glScaled(18,3,100);
	glutSolidCube(3);

	glPopMatrix(); 

	//绘制 旋转长方体 ID2
	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+200,pos_y,pos_z);         
	glRotatef(Ww2,q21,q22,q23);

	glColor3f(0.8f, 0.2f, 0.0f );
	glScaled(24,3,160);
	glutSolidCube(3);

	glPopMatrix(); 

	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+200,pos_y,pos_z);         
	glRotatef(Ww2,q21,q22,q23);

	glColor3f(0.0f, 1.0f, 1.0f );
	glScaled(18,3,100);
	glutSolidCube(3);

	glPopMatrix(); 

	//绘制 旋转长方体 ID3
	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+100,pos_y,pos_z);         
	glRotatef(Ww3,q31,q32,q33);

	glColor3f(0.2f, 1.0f, 0.0f );
	glScaled(24,3,160);
	glutSolidCube(3);

	glPopMatrix(); 

	glPushMatrix();//储存当前视图矩阵
	glTranslatef(pos_x+100,pos_y,pos_z);         
	glRotatef(Ww3,q31,q32,q33);

	glColor3f(1.0f, 0.0f, 1.0f );
	glScaled(18,3,100);
	glutSolidCube(3);

	glPopMatrix(); 

	/***************************************/
	
	
 
	
#endif
	/***************************** 四元数坐标运算 ****************************/
	//---------------------------------------------------------------------
	//根节点四元数
	GL_Quater Qt0;
	Qt0.q0=q0;
	Qt0.q1=q1;
	Qt0.q2=q2;
	Qt0.q3=q3;

	GLB_Hand_Build(GL_HandR,Qt0,Ww,pos_x,pos_y,pos_z);

#if 0
	GLB_FigS_Build(  q10, q11, q12, q13,
					 q20, q21, q22, q23,
					 q30, q31, q32, q33,
					 q40, q41, q42, q43,
					 q50, q51, q52, q53,
					 pos_x, pos_y, pos_z);
	
#else

	GLB_FigS_Build_Bend( q0, q1, q2, q3,
		 q10, q11, q12, q13,
		 q20, q21, q22, q23,
		 q30, q31, q32, q33,
		 q40, q41, q42, q43,
		 q50, q51, q52, q53,
		 pos_x, pos_y, pos_z,
		CDU_P,CDU_R,CDU_Y);
#endif
	//--------------------------------------------------------------------
	////绘制GL 激光线
	//float pos_x2=pos_x+xx0*5;
	//float pos_y2=pos_y+yy0*5;
	//float pos_z2=pos_z+zz0*5;
	//glLineWidth(4);
	//glColor3f(1.0f, 0.15f, 0.05f); 
	//glBegin(GL_LINES);
	//glVertex3f(xx0+pos_x,yy0+pos_y,zz0+pos_z);
	//glVertex3f(pos_x2,pos_y2,pos_z2); 
	//glEnd();
	
	//---------------------------------------
	//递推保存 轨迹坐标
	//for(i=29;i>0;i--)
	//{
	//	gl_x[i]=gl_x[i-1];
	//	gl_y[i]=gl_y[i-1];
	//	gl_z[i]=gl_z[i-1];
	//}
	//	gl_x[0]=xx0+pos_x;
	//	gl_y[0]=yy0+pos_y;
	//	gl_z[0]=zz0+pos_z;

	////-------------------------------------------------
	//glLineWidth(4); 
	//glBegin(GL_LINES);//绘制居中轨迹
	//for(int i=0;i<29;i++)
	//{
	//	glColor3f(0.32f, 0.32f, 0.32f); 
	//	glVertex3f(gl_x[i],gl_y[i],gl_z[i]);
	//	glVertex3f(gl_x[i+1],gl_y[i+1],gl_z[i+1]);
	//}
	//glEnd();
 //----------------- OpenCV
	
	
	//for(int i=0;i<29;i++)//绘制OpenCV 二维平面 XOY 轨迹
	//{
	//	cvCircle(Frame,cvPoint(-gl_x[i]*4+450,gl_y[i]*4+450),6,CV_RGB(255,0,0),-1);
	//	CvPoint pt0;
	//	pt0.x=-gl_x[i]*4+450;
	//	pt0.y=gl_y[i]*4+450;
	//	CvPoint pt1;
	//	pt1.x=-gl_x[i+1]*4+450;
	//	pt1.y=gl_y[i+1]*4+450;
	//	cvLine(Frame,pt0,pt1,CV_RGB(0,255,0),2);
	//	pt0=pt1;
	//}
	Frame->origin=1;
	cvNamedWindow("OPenCV2",0);
	cvShowImage("OPenCV2",Frame);
 // Decrease The Rotation Variable For The Quad
 //////////////////////////////////////////////////////////////////////////////
 // EXAMPLE OPENGL CODE END //////////////////////////////////////////////////////////
 
 // Swap our scene to the front
 
 SwapBuffers(m_hDC);
 
 Invalidate(FALSE);

 //glutIdleFunc(&myIdlex);
 //glutMainLoop();
}

void COpenGL::OnPaint()
{
 CPaintDC dc(this); // device context for painting
 
 // TODO: Add your message handler code here
 ::ValidateRect ( m_hWnd, NULL );
 
 // Do not call CWnd::OnPaint() for painting messages
}


void COpenGL::OnSize(UINT nType, int cx, int cy)
{
 CWnd::OnSize(nType, cx, cy);
 
 // TODO: Add your message handler code here
 if ( cy==0) {              // Prevent A Divide By Zero By                 
  cy=1;               // Making Height Equal One
 }                  
 /*
 glViewport(0,0,cx,cy);           // Reset The Current Viewport
 
 glMatrixMode(GL_PROJECTION);           // Select The Projection Matrix
 glLoadIdentity();              // Reset The Projection Matrix
 
 gluPerspective(45.0f,(GLfloat)cx/(GLfloat)cy,0.1f,100.0f);  // Calculate The Aspect Ratio Of The Window
 
 glMatrixMode(GL_MODELVIEW);            // Select The Modelview Matrix
 glLoadIdentity();              // Reset The Modelview Matrix 
 */

   glViewport (0, 0, (GLsizei) 640, (GLsizei) 480); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(85.0, (GLfloat) 640/(GLfloat) 300, 1.0, 1250.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt (20, 155, -185, 0.0, 50.0, 0.0, 0.0, 1.0, 0.0);
 
}

BOOL COpenGL::SetFullScreen(int width, int height, int depth)
{
 if(!m_bInit) return FALSE;
 if (m_bFullScreen) return TRUE;
 DEVMODE dmScreenSettings;            // Device Mode
 memset(&dmScreenSettings,0,sizeof(dmScreenSettings));     // Makes Sure Memory's Cleared
 dmScreenSettings.dmSize=sizeof(dmScreenSettings);      // Size Of The Devmode Structure
 dmScreenSettings.dmPelsWidth = width;        // Selected Screen Width
 dmScreenSettings.dmPelsHeight = height;       // Selected Screen Height
 dmScreenSettings.dmBitsPerPel = depth;        // Selected Bits Per Pixel
 dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;
 
 // Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
 
 if (ChangeDisplaySettings(&dmScreenSettings,CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL)
 {
  return m_bFullScreen = FALSE;
 }

 SetParent(NULL);
 SetWindowPos(&CWnd::wndTop,
            0, 0,
      GetSystemMetrics(SM_CXSCREEN),
      GetSystemMetrics(SM_CYSCREEN),
      SWP_SHOWWINDOW);

 ShowCursor(FALSE);
 SetFocus();
 return m_bFullScreen = TRUE;
}

BOOL COpenGL::SetNormScreen()
{
 if(!m_bInit) return FALSE;

 if (m_bFullScreen)               // Are We In Fullscreen Mode?
 {
  if (!ChangeDisplaySettings(NULL,CDS_TEST)) {      // if the shortcut doesn't work
   ChangeDisplaySettings(NULL,CDS_RESET);       // Do it anyway (to get the values out of the registry)
   ChangeDisplaySettings(&m_DMsaved,CDS_RESET);      // change it to the saved settings
  } else {
   ChangeDisplaySettings(NULL,CDS_RESET);
  }
  SetParent(m_parent);
  SetWindowPos(&CWnd::wndTop,
            m_rect.left, m_rect.top,
      m_rect.Width(),
      m_rect.Height(),
      SWP_SHOWWINDOW);  
  ShowCursor(TRUE); // Show Mouse Pointer
  m_bFullScreen = FALSE;
 }
 return TRUE;
}