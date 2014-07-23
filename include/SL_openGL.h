/*!=============================================================================
  ==============================================================================

  \file    SL_openGL.h

  \author  Stefan Schaal
  \date    Feb. 1999

  ==============================================================================
  \remarks
  
  Header file for SL_OpenGL.c and related programs
  
  ============================================================================*/

#ifndef _SL_openGL_
#define _SL_openGL_

/*! defines */
#define LEFT_MENU   1
#define MIDDLE_MENU 2
#define RIGHT_MENU  3

/*! basic window settings */
#define EYEX0       0.7
#define EYEY0       2.0
#define EYEZ0       0.3
#define CENTERX0    0.0
#define CENTERY0    0.0
#define CENTERZ0    0.0
#define UPX0        0.0
#define UPY0        0.0
#define UPZ0        1.0

/*! structure to create and maintain openGL windows */
typedef struct OpenGLWindow {
  int    openGLId;         /*!< unique integer assigned by OpenGL 
			      to window */
  int    ID;               /*!< ID assigned by SL */
  char  *wptr;             /*!< window pointer, assigned by the 
			      OS-specific operating system */
  int    x;                /*!< x position of window */
  int    y;                /*!< y position of window */
  int    height;           /*!< height of window */
  int    width;            /*!< width of window */
  char   name[100];        /*!< name of window */
  int    hide;             /*!< hide window or not */
  int    hide_me;          /*!< request to hide */
  int    show_me;          /*!< request to show */
  int    draw_axis;        /*!< TRUE/FALSE */
  int    follow_basis;     /*!< TRUE/FALSE */
  double fovea;            /*!< fovea angle vertical (image y) in degrees */
  /*!<  view variables for the window based on gluLookAt */
  GLdouble eye[N_CART+1];
  GLdouble center[N_CART+1];
  GLdouble up[N_CART+1];
  /*!<  default view variables for the window needed to reset the view */
  GLdouble eye0[N_CART+1];
  GLdouble center0[N_CART+1];
  GLdouble up0[N_CART+1];
  /*!<  glut function associated with the window */
  void   (*idle)();
  void   (*display)();
  void   (*keyboard)(unsigned char, int, int);
  void   (*mouse)(int, int, int, int);
  void   (*motion)(int, int);
  void   (*reshape)();
  void   (*special)();
  void   (*menu)();
  /*!<  pointer to next window */
  char   *next_wptr;       
  
} OpenGLWindow, *OpenGLWPtr;

#ifdef __cplusplus
extern "C" {
#endif

  /* global functions */
  int        initGraphics(int* argc, char*** argv);
  int        init_user_openGL(int argc, char** argv);
  int        createWindow(OpenGLWindow *wptr);
  OpenGLWPtr getOpenGLWindow(void);
  void       addMenuItem(char *name, void (*fptr)(void), int mID);
  void       drawObjects(void);
  void       drawContacts(double);
  void       glutPostRedisplayAll(void);
  OpenGLWPtr whichGLWindow(void);
  void       toggleHideWindow(OpenGLWPtr ptr);
  void       changeWindowUpdateRate(double rate);
  void       hideWindowByName(char *name, int hide);
  void       clmcplotUpdateState(void);
  void       playbackUpdateState(void);
  void       userGraphicsUpdateState(void);
  int        initUserGraphics(void);
  int        checkForMessages(void);
  void       followBaseByName(char *name, int follow);
  void       followBaseByPtr(OpenGLWPtr ptr, int follow);
  void       setUserGraphicsUpdateMode(int mode);
  void       switchCheckerBoard(int flag);
  void       displayComet(void);
  void       updateComet(void);
  void       switchEndeffectorCometDisplay(int id, int s);
  void       switchLinkCometDisplay(int id, int s);
  void       switchCometDisplay(int flag, int n_steps);
  void       resetCometDisplay(void);
  void       displayCoord(void);
  void       receiveOscilloscopeData(void);
  int        initOscWindow(void);
  int        displayListFromObjFile(char *fname, double scale);
  int        displayListFromObjFileFlag(char *fname, double scale, int flag);
  void       toggleShowAxesByName(char *name, int status);
  void       toggleShowAxesByPtr(OpenGLWPtr ptr,int status);
  void       drawArrow(double *sp, double *ep, double width);

  
  /* exported variables */
  extern int solid;
  extern int pause_flag;
  extern double window_update_rate;
  extern int real_time;
  extern int clmcplot_mode;
  extern int playback_mode;
  extern int userGraphics_mode;
  extern Matrix playback_D;
  extern int playback_n_cols;
  extern int playback_current_row;
  extern char **playback_vnames;
  extern SL_Jstate* userGraphics_joint_state;
  extern SL_Cstate userGraphics_base_state;
  extern SL_quat userGraphics_base_orient;
  extern int cometDisplay;
  
#ifdef __cplusplus
}
#endif


#endif  /* _SL_openGL_ */
