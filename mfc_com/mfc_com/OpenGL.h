#if !defined(AFX_OPENGL_H__38B5D1C8_2DFF_4A7D_9A99_3AC401C19D72__INCLUDED_)
#define AFX_OPENGL_H__38B5D1C8_2DFF_4A7D_9A99_3AC401C19D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// OpenGL.h : header file
//
/////////////////////////////////////////////////////////////////////////////
// COpenGL window
#include <gl/glut.h>

class COpenGL : public CWnd
{
// Construction
public:
 COpenGL();


// Attributes
public:

// Operations
public:

// Overrides
 // ClassWizard generated virtual function overrides
 //{{AFX_VIRTUAL(COpenGL)
 //}}AFX_VIRTUAL

// Implementation
public:
 BOOL SetNormScreen();
 BOOL SetFullScreen(int width, int height, int depth);
 virtual void RenderGLScene(float Ww,float q0,float q1,float q2,float q3,
							float Ww1,float q10,float q11,float q12,float q13,
							float Ww2,float q20,float q21,float q22,float q23,
							float Ww3,float q30,float q31,float q32,float q33,
							float Ww4,float q40,float q41,float q42,float q43,
							float Ww5,float q50,float q51,float q52,float q53,
							bool &flag_InitN,
							int *CDU_P,int *CDU_R,int *CDU_Y,
							float Hxn,float Hyn,
	                        int Rool,int Pitch, int Yaw,
							GLfloat *Mccx,bool flag_Destory,
							float &pos_x,float &pos_y ,float &pos_z,

							int &Yaw_angle_OffsetN0,int &Yaw_angL0,
							int &Yaw_angle_OffsetN1,int &Yaw_angL1,
							int &Yaw_angle_OffsetN2,int &Yaw_angL2,
							int &Yaw_angle_OffsetN3,int &Yaw_angL3,
							int &Yaw_angle_OffsetN4,int &Yaw_angL4,
							int &Yaw_angle_OffsetN5,int &Yaw_angL5,

							int Compass_AngleN,
							bool flag_touch,bool &Compass_StartN,int CompassX,int CompassY,int CompassZ,bool flag_connect,
							bool Compass_Flag_Offset
							);
  void Create(CRect rect, CWnd *parent);
 virtual ~COpenGL();

 // Generated message map functions
protected:
 CRect m_rect;
 CWnd* m_parent;
 BOOL m_bFullScreen;
 DEVMODE m_DMsaved;
 BOOL m_bInit;
 int InitGL();
 void KillGLWindow();
 HDC m_hDC;
 HGLRC m_hRC;
 //{{AFX_MSG(COpenGL)
 afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
 afx_msg void OnPaint();
 afx_msg void OnSize(UINT nType, int cx, int cy);
 //}}AFX_MSG
 DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_OPENGL_H__38B5D1C8_2DFF_4A7D_9A99_3AC401C19D72__INCLUDED_)


