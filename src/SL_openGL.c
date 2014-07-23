/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  This file contains all function to handle the openGL graphics output of the
  simulation.
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// UNIX specific headers
#ifdef UNIX
#include "sys/ioctl.h"
#include "sys/stat.h"
#ifdef sparc
#include "sys/filio.h"
#include "unistd.h"
#endif
#endif

// openGL headers
#include "GL/freeglut_std.h"
#include "GL/freeglut_ext.h"
#include "GL/glu.h"
#include <X11/Xlib.h>

// mathematica headers
#include "mdefs.h"

// user specific headers
#include "SL.h"
#include "SL_openGL.h"
#include "SL_openGL_servo.h"
#include "SL_terrains.h"
#include "SL_common.h"
#include "SL_objects.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_kinematics.h"
#include "SL_vx_wrappers.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"


//! local defines
#define   RAD2DEG (180./3.1416)
#define   LIGHT_TURN_RATE	10
#define   VIEW_TURN_RATE	30
#define   MAX_ITEMS             100

// global functions 
void  SLGenericDisplay(void);
int   initializeLighting(OpenGLWPtr wptr);

// local functions
static void  glutMenu(OpenGLWPtr wptr);
static void  reshape(int w, int h);
static void  scene_display(void);
static void  special(int key, int x, int y);
static void  idle(void);
static void  keyboard(unsigned char key, int x, int y);
static void  mouse(int button, int state, int x, int y);
static void  motion(int x, int y);
static void  my_exit(void);
static void  toggleClmcplotMode(void);
static void  togglePlaybackMode(void);
static void  toggleUserGraphicsMode(void);
static void  printCameraParameters(void);
static void  extractState(char **vnames, Vector D, int n_cols);
static void  reset_playback(void);
static void  go_playback(void);
static void  step_playback(void);
static void  stepsize_playback(void);
static void  new_playback(void);
static void  createTerrainDisplayList(ObjectPtr ptr,double *rgb);
static int   checkWindowHide(char *);
static void  togglePause(void);
static int   initCheckerBoard(void);
static void  toggleCheckerBoard(void);
static void  displayCheckerBoard(void);
static void  initCometDisplay(int n_steps);
static void  toggleCometDisplay(void);
static void  toggleCoordDisplay(void);
static void  toggleContactDisplay(void);
static void  updateWindowHide(int status);
static void  drawCoordSystem(double length, double **A, char *name);

// global variables 
int        solid = TRUE;
double     window_update_rate = 30.;
int        clmcplot_mode  = FALSE;
int        playback_mode = FALSE;
int        userGraphics_mode = FALSE;
int        pause_flag    = FALSE;
int        modifiers     = 0;
int        mouseX	 = 0;
int        mouseY        = 0;
SL_Jstate* userGraphics_joint_state;
SL_Cstate  userGraphics_base_state;
SL_quat    userGraphics_base_orient;
int        cometDisplay = FALSE;


/* local variables */
static OpenGLWPtr first_window_ptr = NULL;
static char       menu[3+1][MAX_ITEMS+1][20];
static int        n_menu[3+1]={0,0,0,0};
static void       (*menu_ptr[MAX_ITEMS+1])(void);
static char       command[MAX_ITEMS+1][20];
static int        n_command=0;
static void       (*command_ptr[MAX_ITEMS+1])(void);

static double VIEW_TRANSX0   =   0.0;
static double VIEW_TRANSY0   =   0.0;
static double VIEW_TRANSZ0   =   0.0;
static double VIEW_DISTANCE0 =  -2.0;
static double VIEW_ROTX0     = -80.0;
static double VIEW_ROTY0     =   0.0;
static double VIEW_ROTZ0     =-165.0;

// variables for clmcplot_mode and playback_mode
Matrix playback_D = NULL;
int    playback_n_cols;
int    playback_current_row;
char   **playback_vnames = NULL;

static char   **playback_units  = NULL;
static double playback_freq;
static int    playback_n_rows;
static int    playback_step_mode = FALSE;
static int    playback_step_size = 1;
static char   playback_file_name[100];

/*!*****************************************************************************
*******************************************************************************
\note  initGraphics
\date  August 7, 1992
   
\remarks 

initializes openGl routines

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 

******************************************************************************/
int
initGraphics(int *argc, char*** argv)

{

  int i;
  int rc;
  unsigned char string[255];

  // initialize OpenGL
  glutInit(argc, *argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
  
  // get shared memory
  if (!init_shared_memory())
    return FALSE;

  // object handling
  if (!initObjects())
    return FALSE;

  // oscilloscope window
  if (!initOscWindow())
    return FALSE;

  // add the quit command
  addCommand("quit",my_exit);

  // clmcplot mode for debugging data files
  addToMan("clmcplotMode","toggle clmcplot visualization",toggleClmcplotMode);
  
  // userGraphics mode for controlling the robot state from user graphics
  addToMan("userGraphicsMode","toggle userGraphics robot state visualization",
	   toggleUserGraphicsMode);

  // playback mode for debugging data files
  addToMan("playbackMode","toggle playback visualization",togglePlaybackMode);
  addToMan("r","reset playback file to first frame",reset_playback);
  addToMan("s","step playback file to next frame",step_playback);
  addToMan("g","run playback file",go_playback);
  addToMan("n","new playback file",new_playback);
  addToMan("ss","playback step size",stepsize_playback);
  addToMan("p","toggle pause",togglePause);

  // needed to toggle hide status of windows
  window_check_function = checkWindowHide;

  // memory allocation for userGraphics_joint_state:
  userGraphics_joint_state = (SL_Jstate*)malloc((n_dofs+1)*sizeof(SL_Jstate));

  // checkerboard toggle
  addToMan("checkerBoardMode","draws a checker board on the floor",toggleCheckerBoard);

  // comet toggle
  addToMan("cometDisplay","draws a comet-like line for select robot vertices",toggleCometDisplay);

  // initialize comet display for all endeffectors
  for (i=1; i<=n_endeffs; ++i)
    switchEndeffectorCometDisplay(i,TRUE);

  // coordinate display toggle
  addToMan("coordDisplay","draws local coordinate systems",toggleCoordDisplay);

  // contact point display toggle
  addToMan("contactDisplay","draws all active contact points",toggleContactDisplay);

  // print the extrinsic camera parameters
  addToMan("printCamera","print the camera position",printCameraParameters);

  return TRUE;

}

static void
my_exit(void) {
  exit(-1);
}

/*!*****************************************************************************
*******************************************************************************
\note  reshape
\date  August 7, 1992
   
\remarks 

window moving function

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void 
reshape(int w, int h)

{

  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  glViewport(0,0,(GLsizei) w, (GLsizei) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(ptr->fovea, (GLfloat) w/ (GLfloat) h, 0.1, 20.0);
  glMatrixMode(GL_MODELVIEW);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  mouse
\date  Dec 11, 2007
   
\remarks 

Callback for mouse button up/down events
Used for rotating, panning and zooming the view

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void  mouse(int button, int state, int x, int y)
{
  if (button==GLUT_RIGHT_BUTTON && state==GLUT_DOWN)
    {
      modifiers=glutGetModifiers();
      mouseX=x;
      mouseY=y;
    }
  else
    modifiers=0;
}

/*!*****************************************************************************
*******************************************************************************
\note  mouse
\date  Dec 11, 2007
   
\remarks 

Callback for mouse motion events
Used for rotating, panning and zooming the view

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void  
motion(int x, int y)
{
  int flag=0, i;
  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  if (modifiers)
    {
      int dx=x-mouseX;
      int dy=y-mouseY;
      if ((modifiers & GLUT_ACTIVE_SHIFT)
	  &&
	  (modifiers & GLUT_ACTIVE_CTRL))
	{
	  // rotation
	  double distance=0.0;
	  double phi;
	  double theta;
	  double x=ptr->eye[_X_]-ptr->center[_X_];
	  double y=ptr->eye[_Y_]-ptr->center[_Y_];
	  double z=ptr->eye[_Z_]-ptr->center[_Z_];

	  distance=sqrt(x*x+y*y+z*z);
	  phi = atan2(sqrt(x*x+y*y),z);
	  theta = atan2(y,x);
	  theta-=(double)dx*.01;
	  phi-=(double)dy*.01;
			
	  if (phi >= PI)
	    phi = PI - .001;
	  if (phi <= 0)
	    phi = .001;

	  ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
	  ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
	  ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
	  flag++;
	}
      else if (modifiers & GLUT_ACTIVE_SHIFT)
	{
	  // panning
	  double x=ptr->eye[_X_]-ptr->center[_X_];
	  double y=ptr->eye[_Y_]-ptr->center[_Y_];
	  double theta = atan2(y,x);
	  double ddx = .003*(-dy*cos(theta) + dx*sin(theta));
	  double ddy = .003*(dx*cos(theta) + dy*sin(theta));
	  ptr->center[_X_]+=ddx;
	  ptr->center[_Y_]-=ddy;
	  ptr->eye[_X_]+=ddx;
	  ptr->eye[_Y_]-=ddy;
	  flag++;
	}
      else if (modifiers & GLUT_ACTIVE_CTRL)
	{
	  // zooming
	  double distance=0.0;
	  double newDist=0.0;
	  double ddist=0.003*(double)dy;
	  for (i=_X_; i<=_Z_; i++)
	    {
	      distance+=sqr(ptr->center[i]-ptr->eye[i]);
	    }
	  distance=sqrt(distance);
	  newDist=distance+ddist;
	  for (i=_X_; i<=_Z_; i++)
	    {
	      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])*(newDist/distance);
	    }
	  flag++;
	}
    }
  mouseX=x;
  mouseY=y;
  
  if (flag)
    glutPostRedisplayAll();
}

/*!*****************************************************************************
*******************************************************************************
\note  keyboard
\date  August 7, 1992
   
\remarks 

function for keyboard interaction

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     key : which key was pressed
\param[in]     x   : x location where the key was pressed
\param[in]     y   : y location where the key was pressed

******************************************************************************/
static void 
keyboard(unsigned char key, int x, int y)

{
  
  int i,j,n,flag=FALSE;
  OpenGLWPtr ptr;
  double phi;
  double theta;
  double distance;
  char c[100];

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  switch (key) {
  case 't':
    ++flag;
    printf("%c\n",key);
    if (solid == TRUE)
      solid = FALSE;
    else
      solid = TRUE;
    break;
    
  case 'q':
  case 27: /* ESC */
    ++flag;
    printf("%c\n",key);
    exit(-1);
    break;
    
    /* start of view position functions */
  case GLUT_KEY_RIGHT:  /* right turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    theta -= 0.05;
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Right\n");
    break;

  case GLUT_KEY_LEFT:  /* left turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    theta += 0.05;
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Left\n");
    break;

  case GLUT_KEY_UP:  /* upward turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    phi -= 0.05;
    if (phi <= 0) {
      phi = 0.001;
      printf("Camera cannot turn lower\n");
    }
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Downward\n");
    break;
    
  case GLUT_KEY_DOWN:  /* downward turning */
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    phi = atan2(sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+sqr(ptr->eye[_Y_]-ptr->center[_Y_])),(ptr->eye[_Z_]-ptr->center[_Z_]));
    theta = atan2((ptr->eye[_Y_]-ptr->center[_Y_]),(ptr->eye[_X_]-ptr->center[_X_]));
    phi += 0.05;
    if (phi >= PI) {
      phi = PI - 0.001;
      printf("Camera cannot turn higer\n");
    }
    ptr->eye[_Z_] = distance*cos(phi) + ptr->center[_Z_];
    ptr->eye[_Y_] = ptr->center[_Y_] + distance*sin(phi)*sin(theta);
    ptr->eye[_X_] = ptr->center[_X_] + distance*sin(phi)*cos(theta);
    printf("Turn Upward\n");
    break;

  case 'i':
    ++flag;
    ptr->center[_Y_] -= 0.1;  
    ptr->eye[_Y_]    -= 0.1;  
    printf("Camera +y\n");
    break;

  case 'k':
    ++flag;
    ptr->center[_Y_] += 0.1;  
    ptr->eye[_Y_]    += 0.1;  
    printf("Camera -y\n");
    break;

  case 'l':
    ++flag;
    ptr->center[_X_] += 0.1;  
    ptr->eye[_X_]    += 0.1;  
    printf("Camera +x\n");
    break;

  case 'j':
    ++flag;
    ptr->center[_X_] -= 0.1;  
    ptr->eye[_X_]    -= 0.1;  
    printf("Camera -x\n");
    break;

  case 'u':
    ++flag;
    ptr->center[_Z_] += 0.1;  
    ptr->eye[_Z_]    += 0.1;  
    printf("Camera +z\n");
    break;

  case 'h':
    ++flag;
    ptr->center[_Z_] -= 0.1;  
    ptr->eye[_Z_]    -= 0.1;  
    printf("Camera -z\n");
    break;

  case 'a':
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    distance -= 0.1;
    if (distance <= 0.1) {
      distance = 0.1;
      printf("Camera cannot come closer\n");
    }
    for (i=_X_; i<=_Z_; i++) {
      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])
	*(distance/sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
			sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_])));
    }
    printf("Camera closer\n");
    break;

  case 'z':
    ++flag;
    distance = sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
		    sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_]));
    distance += 0.1;
    for (i=_X_; i<=_Z_; i++) {
      ptr->eye[i]=ptr->center[i]+(ptr->eye[i]-ptr->center[i])
	*(distance/sqrt(sqr(ptr->eye[_X_]-ptr->center[_X_])+
			sqr(ptr->eye[_Y_]-ptr->center[_Y_])+sqr(ptr->eye[_Z_]-ptr->center[_Z_])));
    }
    printf("Camera further away\n");
    break;

  case GLUT_KEY_F1:  /* default posture */
  case 'r':  /* default posture */
    ++flag;
    ptr->eye[_X_]    = ptr->eye0[_X_];
    ptr->eye[_Y_]    = ptr->eye0[_Y_];
    ptr->eye[_Z_]    = ptr->eye0[_Z_];
    ptr->center[_X_] = ptr->center0[_X_];
    ptr->center[_Y_] = ptr->center0[_Y_];
    ptr->center[_Z_] = ptr->center0[_Z_];
    ptr->up[_X_]     = ptr->up0[_X_];
    ptr->up[_Y_]     = ptr->up0[_Y_];
    ptr->up[_Z_]     = ptr->up0[_Z_];
    printf("Default View\n");

    break;
 
  case 'b':
    ++flag;
    if (ptr->follow_basis==0)
      followBaseByPtr(ptr,TRUE);
    else
      followBaseByPtr(ptr,FALSE);
    break;
 
  case 'x':
    ++flag;
    if (ptr->draw_axis==0){
      ptr->draw_axis=1;
      printf("Show axes ON\n");
    }
    else  {
      ptr->draw_axis=0;
      printf("Show axes OFF\n");
    }
    break;
  
  case ' ':
    ++flag;
    togglePause();
    break;

    /* end of view postions functions */
  default:
    ptr = first_window_ptr;
    while (ptr != NULL) {
      sprintf(c,"%d",ptr->ID);
      if (c[0] == key) {
	toggleHideWindow(ptr);
	return;
      }
      ptr = (OpenGLWPtr) ptr->next_wptr;
    }
    break;
    
  }

  glutPostRedisplayAll();

  if (flag)
    printPrompt();

}

/*!*****************************************************************************
*******************************************************************************
\note  initialize_lighting
\date  August 7, 1992
   
\remarks 

initializes the light and coordinate settings for OpenGL

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     ptr : pointer to window where lighting is to be applied

******************************************************************************/
int
initializeLighting(OpenGLWPtr ptr)

{

  int i;
  GLfloat mat_specular[]   ={ (float)0.5, (float)0.5, (float)0.5, (float)1.0 };
  GLfloat light_position[] ={ (float)1.0, (float)1.0, (float)1.0, (float)0.0 };

  /* a nice position for lighting */
  glLoadIdentity();
  gluLookAt(ptr->eye[_X_],ptr->eye[_Y_],ptr->eye[_Z_],
	    ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_],
	    ptr->up[_X_],ptr->up[_Y_],ptr->up[_Z_]);
 	
  /* lighting settings */
  glClearColor ((float)0.9, (float)0.9, (float)1.0, (float)1.0);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, (float)50.0);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  createWindow
\date  August 7, 1992
   
\remarks 

creates an openGl window

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     wptr : pointer to window structure 

******************************************************************************/
int
createWindow(OpenGLWPtr wptr)

{

  int i;
  char *ptr;

  /* create the window */
  glutInitWindowPosition(wptr->x,wptr->y);
  glutInitWindowSize(wptr->width, wptr->height);
  wptr->openGLId = glutCreateWindow(wptr->name);  /* note that this 
						     operation make
						     the window current */
  
  /* attach appropriate OpenGL functions to the current window */
  if (wptr->idle != NULL)
    glutIdleFunc(wptr->idle);
  else
    glutIdleFunc(idle);

  /* The same display function is now used for all simulations, just that
     it calls a window specific display function inside */
  glutDisplayFunc(SLGenericDisplay);
  glutWindowStatusFunc(updateWindowHide);
  if (wptr->display == NULL)
    wptr->display = scene_display;
  
  if (wptr->keyboard != NULL)
    glutKeyboardFunc(wptr->keyboard);
  else
    glutKeyboardFunc(keyboard);

  if (wptr->mouse != NULL)
    glutMouseFunc(wptr->mouse);
  else
    glutMouseFunc(mouse);

  if (wptr->motion != NULL)
    glutMotionFunc(wptr->motion);
  else
    glutMotionFunc(motion);
  
  if (wptr->special != NULL)
    glutSpecialFunc(wptr->special);
  else
    glutSpecialFunc(special);
  
  if (wptr->menu != NULL)
    (*wptr->menu)();
  else
    glutMenu(wptr);
 		
  /* add window to internal administration structure */
  if (first_window_ptr == NULL) {
    first_window_ptr = wptr;
  } else {
    ptr = (char *)first_window_ptr;
    while (((OpenGLWPtr)ptr)->next_wptr != NULL)
      ptr = ((OpenGLWPtr)ptr)->next_wptr;
    ((OpenGLWPtr)ptr)->next_wptr = (char *) wptr;
  }

  glutReshapeFunc(reshape);
  initializeLighting(wptr);	
 	

  return TRUE;

}


/*!*****************************************************************************
*******************************************************************************
\note  getOpenGLWindow
\date  Feb. 99
   
\remarks 

allocates memory structure for openGL window and fills in reasonable
defaults

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none 

******************************************************************************/
OpenGLWPtr
getOpenGLWindow(void)

{
  OpenGLWPtr wptr;
  static int n_windows = 0;
	
  wptr = (OpenGLWPtr) my_calloc(1, sizeof(OpenGLWindow), MY_STOP);

  wptr->x = 10;
  wptr->y = 40;
  wptr->width = 200;
  wptr->height = 200;
  wptr->fovea = 65;

  strcpy(wptr->name,robot_name);

  wptr->idle     = NULL;
  wptr->display  = scene_display;
  wptr->keyboard = keyboard;
  wptr->reshape  = NULL;
  wptr->special  = special;
  wptr->menu     = NULL;

  wptr->openGLId = -1;
  wptr->ID       = ++n_windows;
  wptr->draw_axis=  TRUE;
  wptr->follow_basis = FALSE;

  /* default view variables for the window needed to reset the view */
  wptr->eye[_X_]    = wptr->eye0[_X_]      = EYEX0;
  wptr->eye[_Y_]    = wptr->eye0[_Y_]      = EYEY0;
  wptr->eye[_Z_]    = wptr->eye0[_Z_]      = EYEZ0;
  wptr->center[_X_] = wptr->center0[_X_]   = CENTERX0;
  wptr->center[_Y_] = wptr->center0[_Y_]   = CENTERY0;
  wptr->center[_Z_] = wptr->center0[_Z_]   = CENTERZ0;
  wptr->up[_X_]     = wptr->up0[_X_]       = UPX0;
  wptr->up[_Y_]     = wptr->up0[_Y_]       = UPY0;
  wptr->up[_Z_]     = wptr->up0[_Z_]       = UPZ0;

  wptr->hide          = FALSE;

  return wptr;

}

/*!*****************************************************************************
*******************************************************************************
\note  special
\date  August 7, 1992
   
\remarks 

a special MacOS add-on to handle other than ASCII keys

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
special(int key, int x, int y)

{
  int rc,dum=0;
  	
  keyboard((unsigned char) key,dum,dum);
	
}

/*!*****************************************************************************
*******************************************************************************
\note  SLGenericDisplay
\date  Feb. 2001
   
Generic display function used for every window


*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
void
SLGenericDisplay(void)

{
  int i;
  static SL_Cstate  *basec = &base_state;
  GLfloat  objscolor[4]={(float)0.2,(float)0.2,(float)0.2,(float)1.0};
  OpenGLWPtr ptr = first_window_ptr;

#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
  // lock out the keyboard interaction 
  sl_rt_mutex_lock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

  ptr = whichGLWindow();
  if (ptr == NULL) {
#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_unlock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif
    return;
  }

  // check hide and show request status
  if (ptr->show_me) {
    ptr->show_me = FALSE;
    glutShowWindow();
    // reactivate the window -- somehow this is neede
    glViewport(0,0,(GLsizei) ptr->width, (GLsizei) ptr->height);
    glutPositionWindow(ptr->x, ptr->y-22);
#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_unlock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif
    return;
  }

  if (ptr->hide_me) {
    ptr->hide_me = FALSE;
    // remember the window position
    ptr->x = glutGet(GLUT_WINDOW_X);
    ptr->y = glutGet(GLUT_WINDOW_Y);
    glutIconifyWindow();
    ptr->hide = TRUE;
#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_unlock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif
    return;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glPushMatrix();
  gluLookAt(ptr->eye[_X_],ptr->eye[_Y_],ptr->eye[_Z_],
	    ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_],
	    ptr->up[_X_],ptr->up[_Y_],ptr->up[_Z_]);
  
  /* Follow the basis, if necessary (see keys) */
  if (ptr->follow_basis) {
    glTranslatef(-basec[0].x[1],-basec[0].x[2],-basec[0].x[3]);
  }

  /* Draw the scene */
  (*ptr->display)();

  /* Draw the global axes (see keys) */
  if (ptr->draw_axis) {
    glDisable(GL_LIGHTING); /*to have constant colors */
    glColor4f (1.0,0.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(-100,0,0);       
    glVertex3f(100,0,0);  
    glEnd();
    glColor4f (0.0,1.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,-100,0);       
    glVertex3f(0,100,0);  
    glEnd();
    glColor4f (0.0,0.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,0,-100);       
    glVertex3f(0,0,100);  
    glEnd();
    glEnable(GL_LIGHTING);   
  }
                                      
  /* Allow camera movements (see keys) */ 
  glTranslatef(ptr->center[_X_],ptr->center[_Y_],ptr->center[_Z_]); 

  /* Draw the rotation point of the camera */
  if (ptr->draw_axis) {
    glDisable(GL_LIGHTING); /*to have constant colors */                
    glColor4f (1.0,0.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(-0.25,0,0);       
    glVertex3f(0.25,0,0);  
    glEnd();
    glColor4f (0.0,1.0,0.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,-0.25,0);       
    glVertex3f(0,0.25,0);  
    glEnd();
    glColor4f (0.0,0.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3f(0,0,-0.25);       
    glVertex3f(0,0,0.25);  
    glEnd();
    glEnable(GL_LIGHTING);   
  }

  glPopMatrix();

#ifdef MAKEGIF
  make_gif();
#endif

  glutSwapBuffers();

  // continue keyboard interaction
#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_unlock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

}

/*!*****************************************************************************
*******************************************************************************
\note  udpateWindowHide
\date  Oct 2010

adjusts the hide status of the current window, such that openGL stops
rendering when the window is hidden


*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  status: status provided by openGL


******************************************************************************/
static void
updateWindowHide(int status)
{
  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;

  if (status == GLUT_FULLY_COVERED || status == GLUT_HIDDEN)
    ptr->hide = TRUE;
  else
    ptr->hide = FALSE;

}
 
/*!*****************************************************************************
*******************************************************************************
\note  scene_display
\date  June 1999
 
\remarks 
 
a test function to display something in the case of a NULL
scene_display function
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
 
******************************************************************************/
static void
scene_display(void)
 
{
  /* clear the window */
 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutSolidTeapot((GLdouble) 1.);
  glutSwapBuffers();
 
}                                                                               


	
/*!*****************************************************************************
*******************************************************************************
\note  idle
\date  June 1999
   
\remarks 

the local idle function

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
idle(void)

{

  ;
 
}

	
/*!*****************************************************************************
*******************************************************************************
\note  glutMenu & menu_select
\date  June 1999
   
\remarks 

a simple example how to use menus and pass on the results
to the keyboard

*******************************************************************************
Function Parameters: [in]=input,[out]=output


******************************************************************************/
static void
menu_select(int mode)
{
  int dum=0;
	
  keyboard((unsigned char) mode, dum,dum);

}

static void
glutMenu(OpenGLWPtr ptr)
{
  char string[100];
  char c[100];

  if (ptr->hide)
    return;

  glutCreateMenu(menu_select);
  glutAddMenuEntry("Rotation   [SHIFT+CTRL+RMB]", 'n');
  glutAddMenuEntry("Panning   [SHIFT+RMB]", 'n');
  glutAddMenuEntry("Zooming   [CTRL+RMB]", 'n');
  glutAddMenuEntry("-----------------------------------", 'n');
  glutAddMenuEntry("Toggle Pause   [space bar]", ' ');
  glutAddMenuEntry("Toggle Show Axes   [x]", 'x');
  glutAddMenuEntry("Toggle follow base coordinates   [b]", 'b');
  glutAddMenuEntry("Turn Camera Left   [left arrow]", GLUT_KEY_LEFT);
  glutAddMenuEntry("Turn Camera Right   [right arrow]", GLUT_KEY_RIGHT);
  glutAddMenuEntry("Turn Camera Upward   [up arrow]", GLUT_KEY_UP);
  glutAddMenuEntry("Turn Camera Downward   [down arrow]", GLUT_KEY_DOWN);
  glutAddMenuEntry("Zoom in   [a]", 'a');
  glutAddMenuEntry("Zoom out   [z]", 'z');
  glutAddMenuEntry("Move Focus Camera -x   [j]", 'j');
  glutAddMenuEntry("Move Focus Camera +x   [l]", 'l');
  glutAddMenuEntry("Move Focus Camera -y   [k]", 'k');
  glutAddMenuEntry("Move Focus Camera +y   [i]", 'i');
  glutAddMenuEntry("Move Focus Camera -z   [h]", 'h');
  glutAddMenuEntry("Move Focus Camera +z   [u]", 'u');
  glutAddMenuEntry("Default View   [F1,r]", GLUT_KEY_F1);
  glutAddMenuEntry("Toggle Wireframe   [t]", 't');
  glutAddMenuEntry("Quit   [q,ESC]", 'q');
  glutAddMenuEntry("-----------------------------------", 'n');
  sprintf(string,"Toggle Hide %s   [%d]",ptr->name,ptr->ID);
  sprintf(c,"%d",ptr->ID);
  glutAddMenuEntry(string,c[0]);

  glutAttachMenu(GLUT_LEFT_BUTTON);
  
}


/*!*****************************************************************************
*******************************************************************************
\note  addMenuItem
\date  June 1999
   
\remarks 

add a menu item to a particular menu

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : name of the item
\param[in]     fptr : function pointer to be called with this item
\param[in]     mID  : which menu to choose


******************************************************************************/
void
addMenuItem(char *name, void (*fptr)(void), int mID) 

{

  if (mID <1 || mID>3)
    return;

  if (n_menu[mID] < MAX_ITEMS) {
    ++n_menu[mID];
    strcpy(menu[mID][n_menu[mID]],name);
    menu_ptr[n_menu[mID]] = fptr;
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  drawObjects
\date  Nov 2000
   
\remarks 

draws all objects in the environment

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none 


******************************************************************************/
void
drawObjects(void)
{

  ObjectPtr ptr;
  GLfloat  objscolor[4]={(float)0.2,(float)0.2,(float)0.2,(float)1.0};

  if (objs == NULL)
    return;

  ptr = objs;

  do {

    if (!ptr->hide) {

      objscolor[0] = ptr->rgb[1];
      objscolor[1] = ptr->rgb[2];
      objscolor[2] = ptr->rgb[3];
      glColor4fv(objscolor); 
      
      switch (ptr->type) {
	
      case CUBE:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);      
	glScaled(ptr->scale[1],ptr->scale[2],ptr->scale[3]);
	if (solid)
	  glutSolidCube(1.0);
	else
	  glutWireCube(1.0);
	glPopMatrix();
	
	break;
	
      case SPHERE:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);      
	glScaled(ptr->scale[1]/2.,ptr->scale[2]/2.,ptr->scale[3]/2.);
	if (solid)
	  glutSolidSphere(1.0,ptr->object_parms[1],ptr->object_parms[1]);
	else
	  glutWireSphere(1.0,ptr->object_parms[1],ptr->object_parms[1]);
	glPopMatrix();
	
	break;
	
      case CYLINDER:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);      
	glScaled(ptr->scale[1]/2.,ptr->scale[2]/2.,ptr->scale[3]);
	glTranslated((GLdouble)0.0,(GLdouble)0.0,(GLdouble)-0.5);
	if (solid)
	  glutSolidCylinder(1.0,1.0,ptr->object_parms[1],1);
	else
	  glutWireCylinder(1.0,1.0,ptr->object_parms[1],1);
	glPopMatrix();
	
	break;
	
      case TERRAIN:
	glPushMatrix();
	glTranslated((GLdouble)ptr->trans[1],
		     (GLdouble)ptr->trans[2],
		     (GLdouble)ptr->trans[3]);
	if (ptr->rot[1] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[1],(GLdouble)1.,
		    (GLdouble)0.,(GLdouble)0.);      
	if (ptr->rot[2] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[2],(GLdouble)0.,
		    (GLdouble)1,(GLdouble)0.);      
	if (ptr->rot[3] != 0.0)
	  glRotated((GLdouble)(180./PI)*ptr->rot[3],(GLdouble)0.0,
		    (GLdouble)0.,(GLdouble)1.);       
	
	if (!ptr->display_list_active)
	  createTerrainDisplayList(ptr,ptr->rgb);
	glCallList((GLuint)((unsigned long)ptr));
	glPopMatrix();
	
	break;
	
      }
      
    }

    ptr = (ObjectPtr) ptr->nptr;


  } while (ptr != NULL);

  // the checker board for the floor
  displayCheckerBoard();

}

/*!*****************************************************************************
*******************************************************************************
\note  glutPostRedisplayAll
\date  Dec.2000
   
\remarks 

call glutPostRedisplay for all window in the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
glutPostRedisplayAll(void)
{
  OpenGLWPtr ptr = first_window_ptr;

  while (ptr != NULL) {
    if (!ptr->hide)
      glutPostWindowRedisplay(ptr->openGLId);
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 
}

/*!*****************************************************************************
*******************************************************************************
\note  whichGLWindow
\date  Dec.2000
   
\remarks 

returns the pointer to the structure of the current window

*******************************************************************************
Function Parameters: [in]=input,[out]=output

returns the current window pointer

******************************************************************************/
OpenGLWPtr
whichGLWindow(void) 
{
  int ID;
  OpenGLWPtr ptr = first_window_ptr;

  ID = glutGetWindow();

  while (ptr != NULL) {
    if (ptr->openGLId == ID)
      return ptr;
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 

  return NULL;
}


/*!*****************************************************************************
*******************************************************************************
\note  hide and show of windows
\date  March 2003
   
\remarks 

toggle hide status of windows. Note that the actual glut commands must be given
from the glutMainLoop, and not from the command line thread.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
toggleHideWindow(OpenGLWPtr ptr)
{

  // make sure we don't interfer with the display functions
#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_lock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

  if (ptr->hide) {
    printf("Click Icon >%s< To Make Window Visible\n",ptr->name);
    //ptr->hide    = FALSE;
    //ptr->show_me = TRUE;
    //printf("Show Window %s [%d]\n",ptr->name,ptr->ID);
  } else {
    ptr->hide_me = TRUE;
    printf("Hide Window %s [%d]\n",ptr->name,ptr->ID);
  }

#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
    sl_rt_mutex_unlock( &mutex1 );
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

}

/*!*****************************************************************************
*******************************************************************************
\note  hideWindowByName
\date  March 2003
   
\remarks 

toggle hide status of windows by window name

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name  : string containing the name of the window
\param[in]     hide  : TRUE/FALSE = hide or not

******************************************************************************/
void
hideWindowByName(char *name, int hide)
{
  OpenGLWPtr ptr = first_window_ptr;

  while (ptr != NULL) {
    if (strcmp(name,ptr->name)==0) {
      if (ptr->hide != hide)
	toggleHideWindow(ptr);
      return;
    }
    ptr = (OpenGLWPtr) ptr->next_wptr;
  } 
}

/*!*****************************************************************************
*******************************************************************************
\note  followBaseByName
\date  March 2003
   
\remarks 

toggle follow-base status of windows by window name

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name    : string containing the name of the window
\param[in]     follow  : TRUE/FALSE = follow or not

******************************************************************************/
void
followBaseByName(char *name, int follow)
{
  OpenGLWPtr ptr = first_window_ptr;
  int i;

  while (ptr != NULL) {
    if (strcmp(name,ptr->name)==0) {
      followBaseByPtr(ptr,follow);
      return;
    }
    ptr = (OpenGLWPtr) ptr->next_wptr;
  }

}
/*!*****************************************************************************
*******************************************************************************
\note  toggleShowAxesByName
\date  March 2012
   
\remarks 

toggle show-axes status of windows by window name

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name    : string containing the name of the window
\param[in]     status  : TRUE/FALSE = draw or not

******************************************************************************/
void
toggleShowAxesByName(char *name, int status)
{
  OpenGLWPtr ptr = first_window_ptr;
  int i;

  while (ptr != NULL) {
    if (strcmp(name,ptr->name)==0) {
      toggleShowAxesByPtr(ptr,status);
      return;
    }
    ptr = (OpenGLWPtr) ptr->next_wptr;
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  followBaseByPtr
\date  March 2003
   
\remarks 

toggle follow-base status of windows by window ptr

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     ptr     : string containing the name of the window
\param[in]     follow  : TRUE/FALSE = follow or not

******************************************************************************/
void
followBaseByPtr(OpenGLWPtr ptr, int follow)
{
  int i;

  if (follow){
    ptr->follow_basis=1;
    for (i=1; i<=N_CART; ++i)
      ptr->center[i] = 0.0;
    printf("Following the base coordinates\n");
  }
  else  {
    ptr->follow_basis=0;
    for (i=1; i<=N_CART; ++i)
      ptr->center[i] = base_state.x[i];
    printf("Return to global view\n");
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  toggleShowAxesByPtr
\date  March 2003
   
\remarks 

toggle show axes status of windows by window ptr

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     ptr     : string containing the nae of the window
\param[in]     status  : TRUE/FALSE = draw or not

******************************************************************************/
void
toggleShowAxesByPtr(OpenGLWPtr ptr,int status)
{
  int i;

  ptr->draw_axis=status;

  if (ptr->draw_axis){
    printf("Show axes ON\n");
  }
  else  {
    printf("Show axes OFF\n");
  }

}

/*!*****************************************************************************
*******************************************************************************
\note  changeWindowUpdateRate
\date  March 2003
   
\remarks 

changes the rate at which openGL windows are updated. This is
a speed-smoothness tradeoff.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
changeWindowUpdateRate(double rate)
{

  if (rate > 0)
    window_update_rate = rate;
  else
    window_update_rate = 0;

  printf("Current OpenGL window update rate is %f [hz]\n",window_update_rate);

}


/*!*****************************************************************************
*******************************************************************************
\note  toggleContactDisplay
\date  May 2010
   
\remarks 

allows switching on/off the diplay of contact points

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static int contactDisplay = FALSE;
static void 
toggleContactDisplay(void)
{
  int n;
  double aux;
  
  if(contactDisplay==TRUE) {
    contactDisplay=FALSE;
  } else {
    contactDisplay=TRUE;
  }
}

/*!*****************************************************************************
*******************************************************************************
\note  drawContacts
\date  June 1999
   
\remarks 

draws all contact points and force vectors acting on the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

******************************************************************************/
void
drawContacts(double fscale)

{
  int i,j;
  GLfloat   color_point[4]={(float)1.0,(float)0.35,(float)0.35,(float)1.0};
  GLfloat   color_point_passive[4]={(float)0.35,(float)0.35,(float)1.0,(float)1.0};
  double    x[N_CART+1];
  static int    firsttime = TRUE;
  static double radius = 0.01;

  /* if there are no objects, exit */
  if (objs==NULL)
    return;

  if (firsttime) {
    double w;
    firsttime = FALSE;
    if (read_parameter_pool_double(config_files[PARAMETERPOOL],"contact_point_radius", &w))
      radius = w;
  }

  for (i=0; i<=n_contacts; ++i) { /* loop over all contact points */

    // check whether there is an active contact
    if (!contacts[i].active || (!contacts[i].status && !contactDisplay))
      continue;

    // compute the point of contact
    computeContactPoint(&(contacts[i]),link_pos_sim,Alink_sim,x);


    // draw a blob at the point of contact
    glPushMatrix();
    glTranslated((GLdouble)x[_X_],(GLdouble)x[_Y_],(GLdouble)x[_Z_]);
    if (!contacts[i].status)
      glColor4fv(color_point_passive);
    else
      glColor4fv(color_point);

    if (solid)
      glutSolidSphere(radius,10,10);
    else
      glutWireSphere(radius,10,10);

    // draw the force and torque vector
    glDisable(GL_LIGHTING); //to have constant colors 
    glColor4f (0.0,1.0,1.0,1.0);      
    glBegin(GL_LINES);     
    glVertex3d(0.0,0.0,0.0);       
    //printf("%f %f %f\n",contacts[i].f[_X_],contacts[i].f[_Y_],contacts[i].f[_Z_]);
    glVertex3d(contacts[i].f[_X_]*fscale,
	       contacts[i].f[_Y_]*fscale,
	       contacts[i].f[_Z_]*fscale);  
    glEnd();

    /*
    glColor4f (1.0,0.0,0.0,1.0);      
    glBegin(GL_LINES);     
    //printf("%f %f %f\n",contacts[i].n[_X_],contacts[i].n[_Y_],contacts[i].n[_Z_]);
    glVertex3d(0.0,0.0,0.0);       
    glVertex3d(contacts[i].n[_X_],
	       contacts[i].n[_Y_],
	       contacts[i].n[_Z_]);  
    glEnd();
    */

    glEnable(GL_LIGHTING);   

    glPopMatrix();

  }

}

/*!*****************************************************************************
*******************************************************************************
\note  toggleClmcplotMode
\date  Nov. 2005
   
\remarks 

puts the simulation into CLMCPLOT mode, i.e., it wait for data
files .clmcplot_current_point witten by clmcplot, and just visualizes
the position state given in these files.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleClmcplotMode(void) 
{
  int i;

  if (clmcplot_mode == 0) {

    clmcplot_mode = TRUE;
    playback_mode = FALSE;
    userGraphics_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    printf("CLMCPLOT mode switched on\n");

  } else {

    clmcplot_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("CLMCPLOT mode switched off\n");

  }
  
}

static void 
printCameraParameters(void) 
{
  int i;
  OpenGLWPtr ptr;

  ptr = whichGLWindow();
  if (ptr == NULL)
    return;
  
  printf("eye =    [ ");
  for (i=1; i<=N_CART; i++) printf("%1.3f ",ptr->eye[i]); 
  printf("]\n");
  printf("center = [ ");
  for (i=1; i<=N_CART; i++) printf("%1.3f ",ptr->center[i]); 
  printf("]\n");
  printf("up =     [ ");
  for (i=1; i<=N_CART; i++) printf("%1.3f ",ptr->up[i]); 
  printf("]\n");
}

/*!*****************************************************************************
*******************************************************************************
\note  toggleUserGraphicsMode
\date  Oct. 2008
   
\remarks 

puts the opengl display into "userGraphics" mode, which means that
the state of the robot can be set by user graphics functions instead
of the simulation. The relevant variables to set are base_user_state,
base_user_orient, and joint_user_state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleUserGraphicsMode(void) 
{
  int i;

  if (userGraphics_mode == 0) {

    userGraphics_mode = TRUE;
    clmcplot_mode = FALSE;
    playback_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    printf("User graphics mode switched on\n");

  } else {

    userGraphics_mode = FALSE;
    clmcplot_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("User graphics mode switched off\n");

  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  setUserGraphicsUpdateMode
\date  Oct. 2008
   
\remarks 

puts the opengl display into "userGraphics" mode, which means that
the state of the robot can be set by user graphics functions instead
of the simulation. The relevant variables to set are base_user_state,
base_user_orient, and joint_user_state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
setUserGraphicsUpdateMode(int mode) 
{
  int i;
  if (mode)
    mode = TRUE;
  else
    mode = FALSE;
  if (mode!=userGraphics_mode)
    toggleUserGraphicsMode();
}

/*!*****************************************************************************
*******************************************************************************
\note  togglePlaybackMode
\date  Nov. 2005
   
\remarks 

puts the simulation into PLAYBACK mode, i.e., opens a data file
and visualizes the state that should be store in this file frame
by frame.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
togglePlaybackMode(void) 
{
  int i,rc;
  char fname[100];
  FILE *in;
  int   file_number = 0;

  if (playback_mode == 0) {

    if ( ( in = fopen( ".last_data", "r" ) ) != NULL )   {
      rc=fscanf( in, "%d\n", &file_number );
      fclose( in );
    }
    sprintf( fname, "d%05d", file_number-1 );

    while (TRUE) {

      if (!get_string("Filename for playback",fname,fname))
	return;

      // try to read the file
      if (playback_D != NULL) {
	my_free_matrix(playback_D,1,playback_n_rows,1,playback_n_cols);
	playback_D = NULL;
      }
      if (clmcplot_convert(fname,&playback_D,&playback_vnames,&playback_units,
			   &playback_freq,&playback_n_cols,&playback_n_rows))
	break;

    }

    playback_mode = TRUE;
    clmcplot_mode = FALSE;
    userGraphics_mode = FALSE;
    pause_flag = FALSE;
    togglePause();
    playback_current_row   = 1;
    strcpy(playback_file_name,fname);

    printf("PLAYBACK mode switched on\n");

  } else {

    playback_mode = FALSE;
    pause_flag = TRUE;
    togglePause();
    printf("PLAYBACK mode switched off\n");

  }
  
  // reset simulation 
  playback_step_mode = FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  reset_playback
\date  Sept. 2006
   
\remarks 

resets the framecount of play file to 1

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
reset_playback(void) 
{
  if (!playback_mode)
    return;

  playback_current_row = 1;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  go_playback
\date  Sept. 2006
   
\remarks 

runs playback mode without stepping

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
go_playback(void) 
{
  if (!playback_mode)
    return;

  playback_step_mode = FALSE;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  step_playback
\date  Sept. 2006
   
\remarks 

advances one step in playback frame

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
step_playback(void) 
{
  if (!playback_mode)
    return;

  playback_step_mode = TRUE;

  if (playback_current_row <= playback_n_rows-playback_step_size)
    playback_current_row += playback_step_size;
  else 
    playback_current_row = 1;

  if (playback_current_row < 1)
    playback_current_row = playback_n_rows;
  
}

/*!*****************************************************************************
*******************************************************************************
\note  stepsize_playback
\date  Sept. 2006
   
\remarks 

sets playback stepsize

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
stepsize_playback(void) 
{
  if (!playback_mode)
    return;

  get_int("Playback step size",playback_step_size,&playback_step_size);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  new_playback
\date  Sept. 2006
   
\remarks 

reads a new playback file

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
new_playback(void) 
{
  char fname[100];
  FILE *in;
  int   file_number = 0;
  Matrix Dold;
  int rc;

  if (!playback_mode)
    return;

  if ( ( in = fopen( ".last_data", "r" ) ) != NULL )   {
    rc=fscanf( in, "%d\n", &file_number );
    fclose( in );
  }
  sprintf( fname, "d%05d", file_number-1 );
  
  while (TRUE) {
    
    if (!get_string("Filename for playback",fname,fname))
      return;
    
    // try to read the file
    Dold = playback_D;
    playback_D = NULL;
    if (clmcplot_convert(fname,&playback_D,&playback_vnames,&playback_units,&playback_freq,
			 &playback_n_cols,&playback_n_rows)) {
      if (Dold != NULL) 
	my_free_matrix(Dold,1,playback_n_rows,1,playback_n_cols);
      break;
    }
    playback_D = Dold;
    
  }
  
  playback_current_row   = 1;
  strcpy(playback_file_name,fname);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  userGraphicsUpdateState
\date  Oct 2008
   
\remarks 

Copies the user graphics updated robot states into the real ones

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
userGraphicsUpdateState(void) 
{
  int i;
  for (i=1; i<=n_dofs; i++)
    joint_sim_state[i] = userGraphics_joint_state[i];
  base_state = userGraphics_base_state;
  base_orient = userGraphics_base_orient;
}

/*!*****************************************************************************
*******************************************************************************
\note  playbackUpdateState
\date  March 2006
   
\remarks 

visualizes the next data record

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
playbackUpdateState(void) 
{

  // update the state from the current data record
  extractState(playback_vnames,playback_D[playback_current_row],playback_n_cols);

  if (!playback_step_mode) {
    playback_current_row += playback_step_size;
    if (playback_current_row > playback_n_rows)
      playback_current_row = 1;
    if (playback_current_row < 1)
      playback_current_row = playback_n_rows;
  }
  
}

/*!*****************************************************************************
*******************************************************************************
\note  clmcplotUpdateState
\date  March 2006

\remarks 

tries to read the .clmcplot_current_point, and if successful, 
passes the data on to a parser that extracts the current robot
state.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void 
clmcplotUpdateState(void) 
{
  int i,j,rc;
  char fname[]=".clmcplot_current_point";
  char string[100];
  FILE *fp;
  Matrix Dold;

  if ((fp=fopen(fname,"r")) == NULL)
    return;

  rc=fscanf(fp,"%s %d",string,&playback_current_row);

  fclose(fp);

  // erase the file if successful, such that we notice an update
  remove(fname);

  // try to read the file
  if (strcmp(string,playback_file_name) != 0) {
    Dold = playback_D;
    playback_D = NULL;
    if (!clmcplot_convert(string,&playback_D,&playback_vnames,&playback_units,
			  &playback_freq,&playback_n_cols,&playback_n_rows)) {
      playback_D = Dold;
      return;
    }
    if (Dold != NULL)
      my_free_matrix(Dold,1,playback_n_rows,1,playback_n_cols);
    strcpy(playback_file_name,string);
    printf("clmcplot uses file %s\n",playback_file_name);
  }

  // update the state from the current data record
  extractState(playback_vnames,playback_D[playback_current_row],playback_n_cols);
  
}

/*!*****************************************************************************
*******************************************************************************
\note  extractState
\date  March 2006
   
\remarks 

This function searches for position state variables of the
robot. Each found position (joint_state, base_state,
base_orient) is used to overwrite the simulation state such
that the current point of clmcplot is visualized.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     vnames : the variable names in this data set
\param[in]     D      : one data vector
\param[in]     n_cols : number of elements in data vector

******************************************************************************/
static void 
extractState(char **vnames, Vector D, int n_cols) 
{
  int i,j;
  char   string[100];

  // search for state variables
  for (i=1; i<=n_dofs; ++i) {
    sprintf(string,"%s_th",joint_names[i]);
    joint_sim_state[i].th = joint_default_state[i].th;
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	joint_sim_state[i].th = D[j];
	break;
      }
    }
  }


  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s",misc_sensor_names[i]);
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	misc_sim_sensor[i] = D[j];
	break;
      }
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_x",vnames[j])==0) {
      base_state.x[_X_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_y",vnames[j])==0) {
      base_state.x[_Y_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_z",vnames[j])==0) {
      base_state.x[_Z_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q0",vnames[j])==0) {
      base_orient.q[_Q0_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q1",vnames[j])==0) {
      base_orient.q[_Q1_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q2",vnames[j])==0) {
      base_orient.q[_Q2_] = D[j];
      break;
    }
  }

  for (j=1; j<=n_cols; ++j) {
    if (strcmp("base_q3",vnames[j])==0) {
      base_orient.q[_Q3_] = D[j];
      break;
    }
  }

  for (i=0; i<=n_contacts; ++i) {
    contacts[i].status = FALSE;
    sprintf(string,"CP%d_cstat",i);
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].status = D[j];
	break;
      }
    }
    sprintf(string,"CP%d_cfx",i);
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_X_] = D[j];
	break;
      }
    }
    sprintf(string,"CP%d_cfy",i);
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_Y_] = D[j];
	break;
      }
    }
    sprintf(string,"CP%d_cfz",i);
    for (j=1; j<=n_cols; ++j) {
      if (strcmp(string,vnames[j])==0) {
	contacts[i].f[_Z_] = D[j];
	break;
      }
    }
  }

  
}

/*!*****************************************************************************
*******************************************************************************
\note  createTerrainDisplayList
\date  March 2006
   
\remarks 

For terrain objects, an openGL display list is generated

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     vnames : the variable names in this data set
\param[in]     D      : one data vector
\param[in]     n_cols : number of elements in data vector
\param[in]     rgb    : terrain color

******************************************************************************/
static void
createTerrainDisplayList(ObjectPtr ptr, double *rgb) 
{
  int     i,j,k;
  SL_quat q;
  double  x_min,x_max,y_min,y_max;
  int     n_steps_x;
  int     n_steps_y;
  double  display_grid_delta_x; 
  double  display_grid_delta_y; 
  double  display_grid_delta; 
  char    fname[100];
  double  x,y,z,no_go;
  double  n[N_CART+1];
  GLfloat objscolor[4];

  // for terrains we need to create a display list for this terrain
  if (ptr->type != TERRAIN)
    return;

  if (ptr->display_list_active)
    return;

  // get min and max of the terrain board
  strcpy(fname,ptr->name);
  if (!getContactTerrainMinMax(ptr->name,&x_min,&x_max,&y_min,&y_max))
    return;

  // create a display list for this terrain
  display_grid_delta = ptr->object_parms[4];
  n_steps_x = ceil((x_max-x_min)/display_grid_delta);
  n_steps_y = ceil((y_max-y_min)/display_grid_delta);
  display_grid_delta_x = (x_max-x_min)/(double)n_steps_x;
  display_grid_delta_y = (y_max-y_min)/(double)n_steps_y;
  
  glNewList((GLuint)((unsigned long) ptr), GL_COMPILE);
  
  for (i=1; i<=n_steps_x; ++i) {
    x = i*display_grid_delta_x+x_min;
    
    for (j=1; j<=n_steps_y; ++j) {
      y = j*display_grid_delta_y + y_min;
      
      if (!getContactTerrainInfo(x, y, fname, &z, n, &no_go))
	continue;
      
      for (k=0; k<3; ++k)
	objscolor[k]=rgb[k+1];
      objscolor[3] = 1.0;

      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      
      glBegin(GL_POLYGON);	    
      
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      x -= display_grid_delta_x;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      y -= display_grid_delta_y;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      x += display_grid_delta_x;
      getContactTerrainInfo(x, y, fname, &z, n, &no_go);
      objscolor[0] += (1.-rgb[1])*no_go;
      glColor4fv(objscolor); 	    
      glNormal3d( (GLdouble)n[_X_],(GLdouble)n[_Y_],(GLdouble)n[_Z_]);
      glVertex3d((GLdouble)x,(GLdouble)y,(GLdouble)z);
      
      glEnd();
    }
    
  }
  
  glEndList();

  ptr->display_list_active = TRUE;
  
}


/*!*****************************************************************************
*******************************************************************************
\note  checkWindowHide
\date  June 1999
   
\remarks 

a function that allows hide/display of windows from the command line
interface

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     IDname  : name of the window as the SL ID number of the window

******************************************************************************/
static int
checkWindowHide(char *IDname)

{
  int i,rc;
  OpenGLWPtr ptr = first_window_ptr;
  int ID;

  rc = sscanf(IDname,"%d",&ID);
  if (rc != 1)
    return FALSE;

  /* check whether command is for window hide status */
  while (ptr != NULL) {
    if (ID == ptr->ID) {
      toggleHideWindow(ptr);
      return TRUE;
    }
    ptr = (OpenGLWPtr)ptr->next_wptr;
  }

  return FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  togglePause
\date  Nov. 2007
   
\remarks 

toggles pause flag

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
togglePause(void)
{
  if (pause_flag==FALSE){
    pause_flag=TRUE;
    semGive(sm_pause_sem);
    printf("Pausing the simulation\n");
    semFlush(sm_openGL_servo_sem);
  }
  else  {
    pause_flag=FALSE;
    clmcplot_mode = FALSE;
    playback_mode = FALSE;
    semGive(sm_pause_sem);
    printf("Resuming the simulation\n");
    semFlush(sm_openGL_servo_sem);
  }
}


/*!*****************************************************************************
*******************************************************************************
\note  initCheckerBoard
\date  May 2010
   
\remarks 

initializes checkerboard image variables for adding checker board to floor

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static int        checkImageWidth=128;
static int        checkImageHeight=128;
static int        DrawCheckerBoard=TRUE;
static double     flooroffset = 0.001;
static ObjectPtr  floorObjectPtr;
static GLubyte   *checkImage;
static GLubyte ***checkImagePtrs;
static GLuint     texName;

static int 
initCheckerBoard(void)
{
  int i, j, c;
  double w,h,o;

  // the checker board is only applied to the floor object
  floorObjectPtr = getObjPtrByName("floor");
  if(floorObjectPtr == NULL) {
    printf("DisplayCheckerBoard>> ERROR: could not find \"floor\" object\n");
    return FALSE;
  }

  // look for user parameters
  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"checkerboard_width", &w))
    checkImageWidth = round(floorObjectPtr->scale[_X_]/w);
  
  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"checkerboard_height", &h))
    checkImageHeight = round(floorObjectPtr->scale[_Y_]/h);
  
  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"checkerboard_offset", &o))
    flooroffset = o;

  // allocate all memory for checker board image
  checkImage = 
    (GLubyte *) my_calloc(checkImageHeight*checkImageWidth*4, sizeof(GLubyte), MY_STOP);

  // allocate memory for the pointers to this memory
  checkImagePtrs = (GLubyte ***) my_calloc(checkImageHeight, sizeof(GLubyte **), MY_STOP);
  for (i=0; i<checkImageHeight; ++i) {
    checkImagePtrs[i] = (GLubyte **) my_calloc(checkImageWidth, sizeof(GLubyte *), MY_STOP);
    for (j=0; j<checkImageWidth; ++j)
      checkImagePtrs[i][j] = &(checkImage[i*checkImageWidth*4+j*4]);
  }

  // create the checkerboard image
  for(i=0; i<checkImageHeight; i++) {
    for(j=0; j<checkImageWidth; j++) {
      if( (i+j)%2 == 1) {
	c=255;
      } else {
	c=0;
      }
      checkImagePtrs[i][j][0] = (GLubyte) c;
      checkImagePtrs[i][j][1] = (GLubyte) c;
      checkImagePtrs[i][j][2] = (GLubyte) c;
      checkImagePtrs[i][j][3] = (GLubyte) 255;
    }
  }

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  toggleCheckerBoard
\date  May 2010
   
\remarks 

allows switching on/off the checker board drawing

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleCheckerBoard(void)
{
  if(DrawCheckerBoard==TRUE) {
    DrawCheckerBoard=FALSE;
  } else {
    DrawCheckerBoard=TRUE;
  }
}

/*!*****************************************************************************
*******************************************************************************
\note  switchCheckerBoard
\date  May 2010
   
\remarks 

allows switching on/off the checker board drawing

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in] flag: TRUE or FALSE for the checker board on/off

******************************************************************************/
void 
switchCheckerBoard(int flag)
{
  if(flag==TRUE) {
    DrawCheckerBoard=TRUE;
  } else {
    DrawCheckerBoard=FALSE;
  }
}

/*!*****************************************************************************
*******************************************************************************
\note  displayCheckerBoard
\date  May 2010
   
\remarks 

visualizes the checker board on top of the floor

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
displayCheckerBoard(void )
{
  static int firsttime = TRUE;

  // check whether checkerboard is turned on
  if(!DrawCheckerBoard)
    return;

  glPushAttrib(GL_CURRENT_COLOR);

  // initialization
  if(firsttime) {
    OpenGLWPtr ptr = first_window_ptr;
    int        openGLId;
	  
    if(initCheckerBoard() == FALSE) {
      printf("DisplayCheckerBoard>> ERROR: could not initialize checker board.\n");
      return;
    }

    openGLId = glutGetWindow();

    while (ptr != NULL) {   // this needs to be executed for every window
      glutSetWindow(ptr->openGLId);
      ptr = (OpenGLWPtr) ptr->next_wptr;
	  
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      
      glGenTextures(1, &texName);
      glBindTexture(GL_TEXTURE_2D, texName);
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth, checkImageHeight, 
		 0, GL_RGBA, GL_UNSIGNED_BYTE, checkImage);
    } 

    glutSetWindow(openGLId);
	
    firsttime=FALSE;
  }


  glPushMatrix();
  glEnable(GL_BLEND);
	
  glColor4f(1.0f,1.0f,1.0f,0.5f);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	
  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  glBindTexture(GL_TEXTURE_2D, texName);
	
  glBegin(GL_QUADS);
  GLfloat goalColor[] = { floorObjectPtr->rgb[_X_], floorObjectPtr->rgb[_Y_], 
			  floorObjectPtr->rgb[_Z_], 0.4 };
  glColor4fv(goalColor);
  /* glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, goalColor); */
	
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-(GLdouble)floorObjectPtr->scale[_X_] /2.0, 
	     -(GLdouble)floorObjectPtr->scale[_Y_] /2.0, 
	     (GLdouble)floorObjectPtr->trans[_Z_]+
	     (GLdouble)floorObjectPtr->scale[_Z_]/2.0 + flooroffset);
	
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-(GLdouble)floorObjectPtr->scale[_X_] /2.0, 
	     (GLdouble)floorObjectPtr->scale[_Y_] /2.0, 
	     (GLdouble)floorObjectPtr->trans[_Z_]+
	     (GLdouble)floorObjectPtr->scale[_Z_]/2.0 + flooroffset);
	
  glTexCoord2f(1.0, 1.0);
  glVertex3f((GLdouble)floorObjectPtr->scale[_X_] /2.0, 
	     (GLdouble)floorObjectPtr->scale[_Y_] /2.0, 
	     (GLdouble)floorObjectPtr->trans[_Z_]+
	     (GLdouble)floorObjectPtr->scale[_Z_]/2.0 + flooroffset);
	
  glTexCoord2f(1.0, 0.0);
  glVertex3f((GLdouble)floorObjectPtr->scale[_X_] /2.0, 
	     -(GLdouble)floorObjectPtr->scale[_Y_] /2.0, 
	     (GLdouble)floorObjectPtr->trans[_Z_]+
	     (GLdouble)floorObjectPtr->scale[_Z_]/2.0 + flooroffset);
  glEnd();
	
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
	
	
  glPopMatrix();
  glPopAttrib();
	
}

/*!*****************************************************************************
*******************************************************************************
\note  initCometDisplay
\date  May 2010
   
\remarks 

initializes the comet display feature

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  n_steps: length of comet buffer

******************************************************************************/
static int      n_steps_comet = 500;
static Vector **comet_buffer;
static int      n_elements_comet_buffer = 0;
static int      current_index_comet_buffer = 1;
static int     *comet_display_list=NULL;
static int      comet_display_initialized = FALSE;

static void
initCometDisplay(int n_steps)
{
  int i,j;
  int store;

  if (comet_buffer != NULL && n_steps == n_steps_comet)
    return;

  store = cometDisplay;
  cometDisplay = FALSE;

  // free old memory if needed
  if (comet_buffer != NULL) 
    for (i=1; i<=n_endeffs+n_links; ++i)
      for (j=1; j<=N_CART; ++j)
	my_free_vector(comet_buffer[i][j],1,n_steps_comet);

  // now get new memory
  if (comet_buffer == NULL) {
    comet_buffer = (Vector **)my_calloc(n_endeffs+n_links+1,sizeof(Vector *),MY_STOP);
    for (i=1; i<=n_endeffs+n_links; ++i)
      comet_buffer[i] = (Vector *)my_calloc(N_CART+1,sizeof(Vector),MY_STOP);
  }

  if (comet_display_list == NULL)
    comet_display_list = (int *) my_calloc(n_endeffs+n_links+1,sizeof(int), MY_STOP);

  n_steps_comet = n_steps;
  for (i=1; i<=n_endeffs+n_links; ++i)
    for (j=1; j<=N_CART; ++j)
      comet_buffer[i][j] = my_vector(1,n_steps_comet);

  comet_display_initialized = TRUE;

  resetCometDisplay();

  cometDisplay = store;

}

/*!*****************************************************************************
*******************************************************************************
\note  resetCometDisplay
\date  May 2010
   
\remarks 

resets to comet display buffer to being empty

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
resetCometDisplay(void)
{
  int i,j;

  n_elements_comet_buffer = 0;
  current_index_comet_buffer = 1;

}

/*!*****************************************************************************
*******************************************************************************
\note  switchEndeffectorCometDisplay
\date  May 2010
   
\remarks 

turns on/off  an endeffector for the comet dispaly

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  id   : id number of endeffector
\param[in]  s    : status TRUE or FALSE for turing the display on/off

******************************************************************************/
void
switchEndeffectorCometDisplay(int id, int s)
{
  if (id < 1 || id > n_endeffs) {
    printf("Error: ID=%d is an invalid endeffector ID\n",id);
    return;
  }

  // make sure we are initialized
  if (!comet_display_initialized)
    initCometDisplay(n_steps_comet);

  comet_display_list[id] = s;

  resetCometDisplay();

}

/*!*****************************************************************************
*******************************************************************************
\note  switchLinkCometDisplay
\date  May 2010
   
\remarks 

turns on/off  a link  for the comet dispaly

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  id   : id number of link
\param[in]  s    : status TRUE or FALSE for turing the display on/off

******************************************************************************/
void
switchLinkCometDisplay(int id, int s)
{
  if (id < 1 || id > n_links) {
    printf("Error: ID=%d is an invalid link ID\n",id);
    return;
  }

  // make sure we are initialized
  if (!comet_display_initialized)
    initCometDisplay(n_steps_comet);

  comet_display_list[n_endeffs+id] = s;

  resetCometDisplay();

}

/*!*****************************************************************************
*******************************************************************************
\note  updateComet
\date  May 2010
   
\remarks 

updates the comet_display_list

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
updateComet(void)
{

  int i,j;

  if (!comet_display_initialized || !cometDisplay || openGL_servo_calls < 1)
    return;

  // advance the index
  if (++current_index_comet_buffer > n_steps_comet)
    current_index_comet_buffer = 1;

  // first add the lastest information to the comet_display_list
  for (i=1; i<=n_endeffs; ++i) 
    if (comet_display_list[i]) 
      for (j=1; j<=N_CART; ++j)
	comet_buffer[i][j][current_index_comet_buffer] = link_pos_sim[link2endeffmap[i]][j];
	
  for (i=1; i<=n_links; ++i) 
    if (comet_display_list[n_endeffs+i]) 
      for (j=1; j<=N_CART; ++j) 
	comet_buffer[n_endeffs+i][j][current_index_comet_buffer] = link_pos_sim[i][j];
	
  if (++n_elements_comet_buffer > n_steps_comet)
    n_elements_comet_buffer = n_steps_comet;


}


/*!*****************************************************************************
*******************************************************************************
\note  displayComet
\date  May 2010
   
\remarks 

visualizes link vertices and endeffectors as a 3D trajectories when turned on

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
displayComet(void)
{

  int i,j,c,cc;
  double shade;

  if (!comet_display_initialized || !cometDisplay || openGL_servo_calls < 1)
    return;


  // draw the comets
  for (i=1; i<=n_endeffs+n_links && n_elements_comet_buffer > 1; ++i)  {

    if (!comet_display_list[i]) 
      continue;
    
    glPushMatrix();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDisable(GL_LIGHTING); /*to have constant colors */
    glLineWidth(2.0);
    glBegin(GL_LINE_STRIP);     
    c = current_index_comet_buffer;

    for (j=1; j<=n_elements_comet_buffer-1; ++j) {

      shade = (double)(n_steps_comet-j+1)/(double)n_steps_comet;
      glColor4f (shade,shade,shade,1.0);
      glVertex3d(comet_buffer[i][_X_][c],comet_buffer[i][_Y_][c],comet_buffer[i][_Z_][c]);
      if (--c < 1)
	c = n_steps_comet;

    }

    glEnd();
    glEnable(GL_LIGHTING);   
    glDisable(GL_BLEND);
    glLineWidth(1.0);
    glPopMatrix();

  }


}

/*!*****************************************************************************
*******************************************************************************
\note  toggleCometDisplay
\date  May 2010
   
\remarks 

allows switching on/off the comet dispaly

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void 
toggleCometDisplay(void)
{
  int n;

  if(cometDisplay==TRUE) {
    cometDisplay=FALSE;
  } else {
    get_int("How many point in comet?",n_steps_comet,&n);

    if (n > 1 && n != n_steps_comet)
      initCometDisplay(n);

    cometDisplay=TRUE;
  }
  resetCometDisplay();
}

/*!*****************************************************************************
*******************************************************************************
\note  switchCometDisplay
\date  May 2010
   
\remarks 

allows switching on/off the comet display

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in] flag   : TRUE/FALSE for turining the comet display on/off
\param[in[ n_steps: how many steps in comet buffer (only for flag = TRUE)

******************************************************************************/
void 
switchCometDisplay(int flag, int n_steps)
{
  if(flag==TRUE) {
    initCometDisplay(n_steps);
    cometDisplay=TRUE;
  } else {
    cometDisplay=FALSE;
  }
  resetCometDisplay();
}

/*!*****************************************************************************
*******************************************************************************
\note  toggleCoordDisplay
\date  May 2010
   
\remarks 

allows switching on/off the comet dispaly

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static int coordDisplay = FALSE;
static double coordAxisLength = 0.1;
static iVector coordDisplayFlagsLinks=NULL;
static iVector coordDisplayFlagsDOFs=NULL;
static void 
toggleCoordDisplay(void)
{
  int i,n;
  double aux;
  int    iaux;
  static int link_flag = FALSE;
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    coordDisplayFlagsLinks = my_ivector(1,n_links);
    coordDisplayFlagsDOFs  = my_ivector(1,n_dofs);
  }
  
  if(coordDisplay==TRUE) {

    coordDisplay=FALSE;

  } else {

    get_double("Length of coordinate axes?",coordAxisLength,&aux);
    if (aux > 0)
      coordAxisLength = aux;

    get_int("Link Coordinates=1 or Joint Coordinates=0?",link_flag,&link_flag);

    if (link_flag) {
      get_int("Which link coordinate axes? All=0, or specific number",0,&iaux);
      if (iaux == 0) {
	for (i=0; i<=n_links; ++i)
	  coordDisplayFlagsLinks[i] = TRUE;
      } else {
	if (iaux > 0 && iaux <= n_links) {
	  for (i=0; i<=n_links; ++i)
	    coordDisplayFlagsLinks[i] = FALSE;
	  coordDisplayFlagsLinks[iaux] = TRUE;
	} else {
	  printf("Link number %d out of range (%d to %d)\n",iaux,1,n_links);
	  return;
	}
	for (i=0; i<=n_dofs; ++i)
	  coordDisplayFlagsDOFs[i] = FALSE;
      }

    } else {

      get_int("Which DOF coordinate axes? All=0, or specific number",0,&iaux);
      if (iaux == 0) {
	for (i=0; i<=n_dofs; ++i)
	  coordDisplayFlagsDOFs[i] = TRUE;
      } else {
	if (iaux > 0 && iaux <= n_dofs) {
	  for (i=0; i<=n_dofs; ++i)
	    coordDisplayFlagsDOFs[i] = FALSE;
	  coordDisplayFlagsDOFs[iaux] = TRUE;
	} else {
	  printf("DOF number %d out of range (%d to %d)\n",iaux,1,n_dofs);
	  return;
	}
	for (i=0; i<=n_links; ++i)
	  coordDisplayFlagsLinks[i] = FALSE;
      }

    }
    coordDisplay=TRUE;
  }
}

/*!*****************************************************************************
*******************************************************************************
\note  displayCoord
\date  May 2010
   
\remarks 

draws the local coordinate systems for each link

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
displayCoord(void)
{

  int    i,j;
  double v[N_CART+1+1];
  double r[N_CART+1+1];
  
  if (!coordDisplay || coordDisplayFlagsLinks == NULL)
    return;
  
  for (i=0; i<=n_links; ++i)  {
    if (coordDisplayFlagsLinks[i])
      drawCoordSystem(coordAxisLength, Alink_sim[i],link_names[i]);
  }

  for (i=0; i<=n_dofs; ++i)  {
    if (coordDisplayFlagsDOFs[i])
      drawCoordSystem(coordAxisLength, Adof_sim[i],joint_names[i]);
  }


}

/*!*****************************************************************************
*******************************************************************************
\note  drawCoordSystem
\date  March 2013
   
\remarks 

draws a coordinate system from a homogenous transformation matrix

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  length: length of coordinate axes
\param[in]  A     : homogeneous transformation matrix
\param[in]  name  : label of coordinate system

******************************************************************************/
static void
drawCoordSystem(double length, double **A, char *name)
{

  int    i,j;
  double v[N_CART+1+1];
  double r[N_CART+1+1];
  double s[N_CART+1+1];
  double arrow_width = 0.005;
  
  // draw the coordinate systems
  glPushMatrix();
  glLineWidth(2.0);

  v[_X_] = length;
  v[_Y_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (1.0,0.0,0.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_X_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'X');

  v[_Y_] = length;
  v[_X_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,1.0,0.0,0.0);
  drawArrow(s,r,arrow_width);
  
  v[_Y_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Y');
  
  v[_Z_] = length;
  v[_Y_] = v[_X_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,0.0,1.0,0.0);
  drawArrow(s,r,arrow_width);
  
  v[_Z_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Z');
  
  glColor4f (0.0,0.0,0.0,0.0);
  glBegin(GL_LINES);     
  glVertex3d(A[_X_][4],A[_Y_][4],A[_Z_][4]);
  glVertex3d(A[_X_][4],A[_Y_][4],A[_Z_][4]+0.1);
  glEnd();
  glRasterPos3f(A[_X_][4],A[_Y_][4],A[_Z_][4]+0.1);
  glutBitmapString(GLUT_BITMAP_HELVETICA_18,(const unsigned char *)name);
  
  glLineWidth(1.0);
  glPopMatrix();

}

/*!*****************************************************************************
 *******************************************************************************
\note  displayListFromObjFileFlag
\date  April 2011
   
\remarks 

        reads an OBJ (WAVEFRONT/MAJA) file and creates a display list
	from the faces.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname   : file name
 \param[in]     scale   : scale multiplier for converting units
 \param[in]     flag    : invert normal of faces (TRUE or FALSE)

 returns the ID of the display list, or FALSE

 ******************************************************************************/
#define MAX_STRING_LEN 1000
int
displayListFromObjFile(char *fname, double scale) 
{
  return displayListFromObjFileFlag(fname, scale, FALSE);
}

int
displayListFromObjFileFlag(char *fname, double scale,int flag)
{
  int     i,j,k,r,rc;
  FILE   *fp;
  double  v1[3+1];
  double  v2[3+1];
  double  v3[3+1];
  double  x,y,z;
  char    string[MAX_STRING_LEN+1];
  char    fnamebin[300];
  char    c;
  char    key[10];
  int     n_v=0;
  int     n_f=0;
  int     n_vn=0;
  Matrix  v;
  Matrix  vn=NULL;
  iMatrix f;
  double  max_v[N_CART+1] = {0.0,-1.e10,-1.e10,-1.e10};
  double  min_v[N_CART+1] = {0.0,+1.e10,+1.e10,+1.e10};
  int     nx,ny;
  struct  stat sfile;
  struct  stat sfilebin;

  // check whether binary version exists 
  sprintf(fnamebin,"%s.bin",fname);

  // check whether binary versin needs to be re-generated
  if (stat(fnamebin,&sfilebin) == 0) { // the file exists

    // check date in comparison to filename file and regenerate .bin if needed
    stat(fname,&sfile);
    if (sfile.st_mtime > sfilebin.st_mtime) // need update
      remove(fnamebin);
  }

  fp = fopen(fnamebin,"r");

  if (fp == NULL) { // try reading the ascii file

    // try to open the file
    fp = fopen(fname,"r");
    if (fp == NULL) {
      printf("Cannot read OBJ file >%s<\n",fname);
      return FALSE;
    }
    
    printf("Processing %s ...\n",fname);
    
    // count the number of faces, vertices, normals in file
    while (TRUE) {
      
      // get the next line
      i=0;
      while ((c=fgetc(fp)) != '\n' && c != EOF) {
	string[i++] = c;
	if ( i >= MAX_STRING_LEN ) {
	  printf("ERROR: max. string length exceeded\n");
	  return FALSE;
	}
      }
      string[i] = '\0';
      
      // check for vertices and faces
      sscanf(string,"%s",key);
      if (strcmp(key,"vn")==0)
	++n_vn;
      else if (strcmp(key,"f")==0)
	++n_f;
      else if (strcmp(key,"v")==0)
	++n_v;
      
      if (c == EOF)
	break;
      
    }
    
    printf("#faces=%d  #vertices=%d   #normals=%d\n",n_f,n_v,n_vn);
    
    rewind(fp);
    
    // allocate memory
    v  = my_matrix(1,n_v,1,3);
    f  = my_imatrix(1,n_f,1,9); // first 3 for vertex, next for texture, last 3 for normal
    if (n_vn > 0)
      vn = my_matrix(1,n_vn,1,3);
    
    // get the data
    n_v = n_f = n_vn = 0;
    while (TRUE) {
      char s1[100],s2[100],s3[100];
      
      // get the next line
      i=0;
      while ((c=fgetc(fp)) != '\n' && c != EOF) {
	string[i++] = c;
	if ( i >= MAX_STRING_LEN ) {
	  printf("ERROR: max. string length exceeded\n");
	  return FALSE;
	}
      }
      string[i] = '\0';
      
      // check for vertices, normals, and faces
      sscanf(string,"%s",key);
      if (strcmp(key,"vn")==0) {
	++n_vn;
	sscanf(string,"%s %lf %lf %lf",key,&vn[n_vn][1],&vn[n_vn][2],&vn[n_vn][3]);
	
      } else if (strcmp(key,"v")==0) {
	++n_v;
	sscanf(string,"%s %lf %lf %lf",key,&v[n_v][1],&v[n_v][2],&v[n_v][3]);
	
	v[n_v][1]*=scale;
	v[n_v][2]*=scale;
	v[n_v][3]*=scale;
	
	for (j=1; j<=N_CART; ++j) {
	  if (v[n_v][j] > max_v[j])
	    max_v[j] = v[n_v][j];
	  if (v[n_v][j] < min_v[j])
	    min_v[j] = v[n_v][j];
	}
	
      } else if (strcmp(key,"f")==0) {
	++n_f;
	
	sscanf(string,"%s %s %s %s",key,s1,s2,s3);
	
	if (sscanf(s1,"%d/%d/%d",&f[n_f][1],&f[n_f][1+3],&f[n_f][1+6]) == 3) {
	  ; // this is the best scenario with vertex,texture, and normal info
	} else if (sscanf(s1,"%d//%d",&f[n_f][1],&f[n_f][1+6]) == 2) {
	  ; // texture info is missing
	} else if (sscanf(s1,"%d/%d/",&f[n_f][1],&f[n_f][1+3]) == 2) {
	  ; // normal info is missing
	} else {
	  if (sscanf(s1,"%d",&f[n_f][1]) != 1) 
	    printf("Couldn't parse face informamtion\n");
	}
	
	if (sscanf(s2,"%d/%d/%d",&f[n_f][2],&f[n_f][2+3],&f[n_f][2+6]) == 3) {
	  ; // this is the best scenario with vertex,texture, and normal info
	} else if (sscanf(s2,"%d//%d",&f[n_f][2],&f[n_f][2+6]) == 2) {
	  ; // texture info is missing
	} else if (sscanf(s2,"%d/%d/",&f[n_f][2],&f[n_f][2+3]) == 2) {
	  ; // normal info is missing
	} else {
	  if (sscanf(s2,"%d",&f[n_f][2]) != 1) 
	    printf("Couldn't parse face informamtion\n");
	}
	
	if (sscanf(s3,"%d/%d/%d",&f[n_f][3],&f[n_f][3+3],&f[n_f][3+6]) == 3) {
	  ; // this is the best scenario with vertex,texture, and normal info
	} else if (sscanf(s3,"%d//%d",&f[n_f][3],&f[n_f][3+6]) == 2) {
	  ; // texture info is missing
	} else if (sscanf(s3,"%d/%d/",&f[n_f][3],&f[n_f][3+3]) == 2) {
	  ; // normal info is missing
	} else {
	  if (sscanf(s3,"%d",&f[n_f][3]) != 1) 
	    printf("Couldn't parse face informamtion\n");
	}
	
      }
      
      if (c == EOF)
	break;
    }
    fclose(fp);
    
    printf("max: x=%6.2f y=%6.2f z=%6.2f\n",max_v[_X_],max_v[_Y_],max_v[_Z_]);
    printf("min: x=%6.2f y=%6.2f z=%6.2f\n",min_v[_X_],min_v[_Y_],min_v[_Z_]);
    
    // write binary files
    fp = fopen(fnamebin,"w");
    fprintf(fp,"%d %d %d\n",n_v,n_vn,n_f);

    fwrite_mat(fp,v);
    if (n_vn > 0)
      fwrite_mat(fp,vn);
    fwrite_imat(fp,f);

    fclose(fp);

    printf("... done with %s\n",fname);
    
  } else { // read binary files
    
    rc = fscanf(fp,"%d %d %d",&n_v,&n_vn,&n_f);
    fgetc(fp);

    // allocate memory
    v  = my_matrix(1,n_v,1,3);
    f  = my_imatrix(1,n_f,1,9); // first 3 for vertex, next for texture, last 3 for normal
    if (n_vn > 0)
      vn = my_matrix(1,n_vn,1,3);
    
    // binary read
    fread_mat(fp,v);
    if (n_vn > 0) {
      fread_mat(fp,vn);
      if (flag) 
	mat_mult_scalar(vn,-1.0,vn);
    }
    fread_imat(fp,f);

    fclose(fp);

  }

  
  // create one display list item
  GLuint index = glGenLists(1);
  glNewList((GLuint)index, GL_COMPILE);

  for (i=1; i<=n_f; ++i) {
    double normal[N_CART+1];
    double v1[N_CART+1];
    double v2[N_CART+1];

    // compute default normal vector if needed
    for (j=1; j<=N_CART; ++j) {
      v1[j] = (v[f[i][2]][j] - v[f[i][1]][j]);
      v2[j] = (v[f[i][3]][j] - v[f[i][2]][j]);
    }
    vec_mult_outer_size(v1, v2, N_CART, normal);
    if (flag)
      for (j=1; j<=N_CART; ++j)
	normal[j] *= -1.0;

    // draw the face
    glBegin(GL_TRIANGLES);

    if ( f[i][6+1] == 0 )
      glNormal3d( (GLdouble)normal[_X_],(GLdouble)normal[_Y_],(GLdouble)normal[_Z_]);
    else
      glNormal3d((GLdouble)vn[f[i][6+1]][_X_],(GLdouble)vn[f[i][6+1]][_Y_],(GLdouble)vn[f[i][6+1]][_Z_]);

    glVertex3dv(&(v[f[i][1]][_X_]));

    if ( f[i][6+2] == 0 )
      glNormal3d( (GLdouble)normal[_X_],(GLdouble)normal[_Y_],(GLdouble)normal[_Z_]);
    else
      glNormal3d((GLdouble)vn[f[i][6+2]][_X_],(GLdouble)vn[f[i][6+2]][_Y_],(GLdouble)vn[f[i][6+2]][_Z_]);

    glVertex3dv(&(v[f[i][2]][_X_]));

    if ( f[i][6+3] == 0 )
      glNormal3d( (GLdouble)normal[_X_],(GLdouble)normal[_Y_],(GLdouble)normal[_Z_]);
    else
      glNormal3d((GLdouble)vn[f[i][6+3]][_X_],(GLdouble)vn[f[i][6+3]][_Y_],(GLdouble)vn[f[i][6+3]][_Z_]);

    glVertex3dv(&(v[f[i][3]][_X_]));

    glEnd();

  }
  glEndList();

  // clean up
  my_free_matrix(v,1,n_v,1,3);
  if (n_vn > 0)
    my_free_matrix(vn,1,n_vn,1,3);
  my_free_imatrix(f,1,n_f,1,9);

  return index;

}

/*!*****************************************************************************
 *******************************************************************************
\note  drawArrow
\date  March 2013
   
\remarks 

 draws a 3D arrow from a start to an endpoint

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     sp   : start point vector
 \param[in]     ep   : end point vector
 \param[in]     width: width of arrow cylinder


 ******************************************************************************/
void
drawArrow(double *sp, double *ep, double width)
{
  double gamma;
  double beta;
  double l;
  int i;

  glPushMatrix();
  
  // translate to center of cyclinder
  glTranslated((GLdouble) sp[_X_],(GLdouble)sp[_Y_], (GLdouble)sp[_Z_]);


  // rotate the z axis along the cylinder
  gamma = atan2_save((ep[_X_]-sp[_X_]),(ep[_Y_]-sp[_Y_]));

  l = 0.0;
  for (i=1; i<=N_CART; ++i)
    l += sqr(ep[i]-sp[i]);
  l = sqrt(l);

  beta = acos((ep[_Z_]-sp[_Z_])/l);

  glRotated((GLdouble)(180./PI)*gamma,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);      
  glRotated((GLdouble)(180./PI)*beta,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);      

  glutSolidCylinder(width/2.,l,10,1);
  glTranslated((GLdouble) 0.0,(GLdouble)0.0, l);
  glutSolidCone((GLdouble) 2.*width,(GLdouble) 4*width,10,5);


  glPopMatrix();

}
