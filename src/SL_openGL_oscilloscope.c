/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL_oscilloscope.c

  \author  Stefan Schaal
  \date    Aug 2010

  ==============================================================================
  \remarks

  This file contains all function to handle the openGL graphics output for the
  oscilloscope

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// UNIX specific headers
#ifdef UNIX
#include "sys/ioctl.h"
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

// global functions

// local functions
static void  osc_display(void);
static void  osc_reshape(int w, int h);
static void  addOscData(SL_oscEntry entry);

// global variables 

/* local variables */
#define MAX_VARS_PER_PLOT 10            //<! the max number of variables per osc plot
#define MAX_OSC_DATA      10000
static int    openGLId_osc = 0;
int    periods_window_AD = 2;           //<! number of task_servo periods to display
double time_window_vars = 5.0;          //<! time window variables in seconds
static int    n_oscilloscope_plots = 3;
static int    osc_enabled = FALSE;

typedef struct {  //<! this keeps all the data for oscilloscope display
  char   names[MAX_VARS_PER_PLOT+1][40];
  int    n_active;
  int    current_index[MAX_VARS_PER_PLOT+1];
  int    n_data[MAX_VARS_PER_PLOT+1];
  double data[MAX_VARS_PER_PLOT+1][MAX_OSC_DATA+1][2+1];
  double max;
  double min;
} OscData;

static OscData *osc_data = NULL;

/*!*****************************************************************************
 *******************************************************************************
\note  initOscWindow
\date  May 2010

\remarks 

initializes the oscilloscope window

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
initOscWindow(void)
{
  int      i,j;
  int      x = 600;
  int      y = 20;
  int      width = 600;
  int      height = 400;
  char     string[100];
  char     xstring[100];
  Display *disp;
  int      screen_num;
  int      display_width;
  int      display_height;
  double   aux;

  // is the oscilloscope enabled?
  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"osc_enabled", &i))
    osc_enabled = macro_sign(abs(i));

  if (!osc_enabled)
    return TRUE;

  // how many oscilloscope graphs?
  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"n_osc_plots", &i)) {
    if (i > 0)
      n_oscilloscope_plots = i;
  }

  // #periods of oscilloscope A/D plot
  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"osc_periods_ad", &i)) {
    if (i > 0)
      periods_window_AD = i;
  }

  // window size of variable display
  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"osc_time_window_vars", &aux)) {
    if (aux > 0)
      time_window_vars = aux;
  }

  // allocate memory for the plots and initialize
  osc_data = (OscData *) my_calloc(n_oscilloscope_plots+1,sizeof(OscData),MY_STOP);
  for (i=0; i<=n_oscilloscope_plots; ++i) {
    for (j=1; j<=MAX_VARS_PER_PLOT; ++j) {
      osc_data[i].current_index[j] = 1;
    }
    osc_data[i].max = -1.e10;
    osc_data[i].min =  1.e10;
  }

  // connect to X server using the DISPLAY environment variable
  if ( (disp=XOpenDisplay(NULL)) == NULL ) {
    printf("Cannot connect to X servo %s\n",XDisplayName(NULL));
    exit(-1);
  }

  // get screen size from display structure macro 
  screen_num = DefaultScreen(disp);
  display_width = DisplayWidth(disp, screen_num);
  display_height = DisplayHeight(disp, screen_num);

  // overwrite the default window options from ParameterPool.cf
  if (read_parameter_pool_string(config_files[PARAMETERPOOL], 
                                 "osc_window_geometry", string))
    parseWindowSpecs(string, display_width,display_height,xstring, 
                     &x,
                     &y,
                     &width,
                     &height);

  // create the window
  glutInitWindowPosition(x,y);
  glutInitWindowSize(width,height);
  openGLId_osc = glutCreateWindow("Oscilloscope"); // makes window current, too

  // attach appropriate OpenGL functions to the current window
  glutDisplayFunc(osc_display);
  glutReshapeFunc(osc_reshape);
  /*
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutSpecialFunc(special);
  glutMenu(wptr);
   */

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  osc_display
\date  Aug 2010

display function for oscilloscope


 *******************************************************************************
Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
static void
osc_display(void)

{
  int i,j,c,r;
  static int firsttime = TRUE;
  static int listID = 999;
  static double freq_AD = 0;
  static double start_time_AD = 0;
  double offx_left = 0.2;
  double offx_right = 0.1;
  double offy = 0.05;
  double offy_extra = 0.1;
  double tick_length = 0.1;
  int    its=0;
  int    ivs=0;
  double tstart=0,tend=1;
  double tend_ivs=0;
  int    count = 0;
  double last_ts = 0;
  int    last_c = 0;
  char   string[40];
  double aux;
  double dist = 1.e10;
  double last_dist = 1.e10;


  GLfloat  colors[][4]={
                        {(float)0.0,(float)1.0,(float)1.0,(float)1.0},   // cyan
                        {(float)0.5,(float)0.5,(float)1.0,(float)1.0},   // blue
                        {(float)1.0,(float)0.25,(float)0.25,(float)1.0}, // red
                        {(float)0.1,(float)0.5,(float)0.5,(float)1.0},   // green
                        {(float)0.8,(float)0.8,(float)0.8,(float)1.0},   // gray
                        {(float)0.7,(float)1.0,(float)0.7,(float)1.0},   // light green
                        {(float)1.,(float)1.,(float)1.0,(float)1.0},     // white
                        {(float)1.0,(float)1.0,(float)0.0,(float)1.0},   // yellow
                        {(float)1.0,(float)0.7,(float)0.7,(float)1.0},   // pink
                        {(float)1.0,(float)0.0,(float)1.0,(float)1.0},   // magenta
  };


  if (firsttime) {
    firsttime = FALSE;

    // cache all possible information in a display list
    glNewList((GLuint)((unsigned long) listID), GL_COMPILE);
    glBegin(GL_LINE_STRIP);
    glVertex2d(0.0,0.0);
    glVertex2d(0.0,1.0);
    glVertex2d(1.0,1.0);
    glVertex2d(1.0,0.0);
    glVertex2d(0.0,0.0);
    glEnd();
    glBegin(GL_LINES);
    for (i=1; i<=9; ++i) {
      glVertex2d(0.1*i,0.0);
      glVertex2d(0.1*i,tick_length);
    }
    glEnd();
    glEndList();
  }

  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_LIGHTING); 
  glLoadIdentity();
  glPushMatrix();

  // draw the coordinate axes
  glColor4f (1.0,1.0,1.0,1.0);      

  for (i=0; i<=n_oscilloscope_plots; ++i) {
    glPushMatrix();
    if (i==0)
      glTranslated(offx_left,(n_oscilloscope_plots-i)+offy,0.0);
    else 
      glTranslated(offx_left,(n_oscilloscope_plots-i)+offy-offy_extra,0.0);
    glScaled(1.-offx_left-offx_right,1-2*offy,1.0);
    glCallList((GLuint)((unsigned long)listID));
    glPopMatrix();
  }

  // draw data traces ===========================================================
  // the 0-th plot is special for the A/D signals -------------------------------

  // find "D2A_task" for trigger
  for (i=1; i<=osc_data[0].n_active; ++i) {
    if (strcmp("D2A_task [%]",osc_data[0].names[i])==0)
      break;
  }

  if ( i > osc_data[0].n_active ) {
    if (osc_data[0].n_active > 1000)
      printf("Could not find D2A_task in oscilloscope data\n");
    glutSwapBuffers();
    return;
  } else {
    its = i; // store this index
  }

  // find "D2A_vision" to try to sync the vision servo in trigger
  for (i=1; i<=osc_data[0].n_active; ++i) {
    if (strcmp("D2A_vision",osc_data[0].names[i])==0) {
      ivs = i;
      break;
    }
  }

  // search for "periods_window_AD" number of complete periods backwards, and
  // if vision servo exists, try to find a trigger point the closest to the vision
  // servo such that the trigger image is more coherent

  // first find last vision_servo minimum
  if (ivs) {
    tend_ivs = 0;
    for (i=0; i < osc_data[0].n_data[ivs]; ++i) {

      c = osc_data[0].current_index[ivs]-i;
      if (c < 1)
        c += MAX_OSC_DATA;

      if (osc_data[0].data[ivs][c][2] == 0.0) {
        tend_ivs = osc_data[0].data[ivs][c][1];
        break;
      }
    }
    if (tend_ivs == 0) // nothing found
      ivs = FALSE;
  }

  count  = 0;
  for (i=0; i < osc_data[0].n_data[its]; ++i) {

    c = osc_data[0].current_index[its]-i;
    if (c < 1)
      c += MAX_OSC_DATA;

    if (osc_data[0].data[its][c][2] == 0.0 && count == 0) {
      tend = osc_data[0].data[its][c][1];
      count = 1;
      if (ivs)
        last_dist = fabs(tend_ivs - tend);
    } else if (osc_data[0].data[its][c][2] == 0.0 && count == periods_window_AD) {
      if (ivs) {
        dist = fabs(tend_ivs - osc_data[0].data[its][c][1]);
        if (dist < last_dist) { // change tend and count
          tend = osc_data[0].data[its][c][1];
          last_dist = dist;
          count = 1;
          continue;
        }
      }
      tstart = osc_data[0].data[its][c][1];
      ++count;
      break;
    } else if (osc_data[0].data[its][c][2] == 0.0) {
      ++count;
      if (ivs) {
        dist = fabs(tend_ivs - osc_data[0].data[its][c][1]);
        if (dist < last_dist) { // change tend and count
          tend = osc_data[0].data[its][c][1];
          last_dist = dist;
          count = 1;
        }
      }
    }

  }

  if (count != periods_window_AD+1) { // not enough periods yet
    glutSwapBuffers();
    return;
  }

  // plot all data within the tstart->tend time window
  glPushMatrix();

  glTranslated(offx_left,n_oscilloscope_plots+offy,0.0);
  glScaled(1.-offx_left-offx_right,1-2*offy,1.0);

  aux = (-offx_left*0.95)/(1.-offx_left-offx_right);

  for (j=1; j<=osc_data[0].n_active; ++j) {
    glColor4fv(colors[j%10]);    
    glRasterPos2d(aux,(j-1)*0.1);
    glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)osc_data[0].names[j]);
  }

  glColor4f (1.0,1.0,1.0,1.0);      

  start_time_AD = -(tend-tstart) * 0.01 + 0.99*start_time_AD;
  sprintf(string,"%3.1e",start_time_AD);
  glRasterPos2d(-0.0,-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  freq_AD = ((double)periods_window_AD)/(tend-tstart)*0.01 + 0.99*freq_AD;
  sprintf(string,"%5.1f Hz",freq_AD);
  glRasterPos2d(0.5,-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  sprintf(string,"0.0");
  glRasterPos2d(1.0,-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  sprintf(string,"% .2f",osc_data[0].min);
  glRasterPos2d(1.0,0.0);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  sprintf(string,"% .2f",osc_data[0].max);
  glRasterPos2d(1.0,1.0-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);


  for (j=1; j<=osc_data[0].n_active; ++j) {

    glColor4fv(colors[j%10]);    
    last_ts = 0.0;
    last_c  = 0;
    glBegin(GL_LINE_STRIP);
    for (i=0; i < osc_data[0].n_data[j]; ++i) {

      c = osc_data[0].current_index[j]-i;
      if (c < 1)
        c += MAX_OSC_DATA;

      if (osc_data[0].data[j][c][1] > tend)
        continue;

      if (osc_data[0].data[j][c][1] < tstart) {
        glVertex2d(0.0,osc_data[0].data[j][last_c][2]/100.);
        break;
      }

      // A/D data is unequally sampled and needs thus to be drawn as steps
      if (last_ts != 0)
        glVertex2d((last_ts-tstart)/(tend-tstart),osc_data[0].data[j][c][2]/100.);
      else
        glVertex2d(1.0,osc_data[0].data[j][c][2]/100.);

      glVertex2d((osc_data[0].data[j][c][1]-tstart)/(tend-tstart),osc_data[0].data[j][c][2]/100.);

      last_ts = osc_data[0].data[j][c][1];
      last_c  = c;

    }
    glEnd();

  }

  glPopMatrix();

  // the variable plots  ---------------------------------------------------------

  for (r=1; r<=n_oscilloscope_plots; ++r) {


    // determine start and end time from all the trajectories
    tend = -1.e10;
    for (j=1; j<=osc_data[r].n_active; ++j) {
      if (osc_data[r].data[j][osc_data[r].current_index[j]][1] > tend)
        tend = osc_data[r].data[j][osc_data[r].current_index[j]][1];
    }
    tstart = tend - time_window_vars;

    for (j=1; j<=osc_data[r].n_active; ++j) {

      glPushMatrix();

      glTranslated(offx_left,(n_oscilloscope_plots-r)+offy-offy_extra,0.0);
      glScaled(1.-offx_left-offx_right,1-2*offy,1.0);

      aux = (-offx_left*0.95)/(1.-offx_left-offx_right);

      glColor4fv(colors[j%10]);    
      glRasterPos2d(aux,(j-1)*0.1);
      glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)osc_data[r].names[j]);

      glColor4f (1.0,1.0,1.0,1.0);      

      sprintf(string,"% .2f",osc_data[r].min);
      glRasterPos2d(1.0,0.0);
      glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

      sprintf(string,"% .2f",osc_data[r].max);
      glRasterPos2d(1.0,1.0-2*offy);
      glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

      glColor4fv(colors[j%10]);    

      glBegin(GL_LINE_STRIP);
      for (i=0; i < osc_data[r].n_data[j]; ++i) {

        c = osc_data[r].current_index[j]-i;
        if (c < 1)
          c += MAX_OSC_DATA;

        if (osc_data[r].data[j][c][1] > tend)
          continue;

        if (osc_data[r].data[j][c][1] < tstart)
          break;

        // just make the data a vertex
        glVertex2d((osc_data[r].data[j][c][1]-tstart)/(tend-tstart),
                   (osc_data[r].data[j][c][2]-osc_data[r].min)/(osc_data[r].max-osc_data[r].min));

      }
      glEnd();

      glPopMatrix();

    }

  }

  glPushMatrix();

  glTranslated(offx_left,offy-offy_extra,0.0);
  glScaled(1.-offx_left-offx_right,1-2*offy,1.0);
  glColor4f (1.0,1.0,1.0,1.0);      

  sprintf(string,"%6.3f",-(tend-tstart));
  glRasterPos2d(-0.0,-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  sprintf(string,"0.0");
  glRasterPos2d(1.0,-2*offy);
  glutBitmapString(GLUT_BITMAP_HELVETICA_10,(const unsigned char *)string);

  glPopMatrix();

  glutSwapBuffers();




}

/*!*****************************************************************************
 *******************************************************************************
\note  osc_reshape
\date  August 2010

\remarks 

reshape function for oscillosope

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]   w: new width of window
\param[in]   h: new height of window

 ******************************************************************************/
static void 
osc_reshape(int w, int h)
{

  OpenGLWPtr ptr;

  glViewport(0,0,(GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0.0,1.0,-0.3,(double)(n_oscilloscope_plots+1));
  glMatrixMode(GL_MODELVIEW);

}

/*!****************************************************************************
 ******************************************************************************
\note  receiveOscilloscopeData
\date  Aug. 2010

\remarks

copies all local oscilloscope data to shared memory structure

 *****************************************************************************
Function Parameters: [in]=input,[out]=output

none

 *****************************************************************************/
#define TIME_OUT_SHORT_NS   10000  //!< time out in nano seconds
void 
receiveOscilloscopeData(void)
{

  int i,j,r;
  int count;

  if (!osc_enabled)
    return;

#ifdef __XENO__
  // we want to be in real-time mode here
  //printf("..\n");
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif

  // try to take semaphore with very short time-out -- we don't care if we 
  // cannot copy the data to shared memory every time, as this is not a time 
  // critical operation
  if (semTake(sm_oscilloscope_sem,ns2ticks(TIME_OUT_SHORT_NS)) == ERROR)
  {
#ifdef __XENO__
    // we want to be in secondary mode here
    rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

    return;
  }
  // copy first-in-first-out data from shared memory into ring buffer
  count = sm_oscilloscope->n_entries;
  for (i=1; i<=count; ++i)
    addOscData(sm_oscilloscope->entries[i]);

  sm_oscilloscope->n_entries = 0;

  // give back semaphore
  semGive(sm_oscilloscope_sem);

#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

  // update the oscilloscope window
  if (pause_flag || stand_alone_flag)
    return;
  else if (count > 0)
    glutPostWindowRedisplay(openGLId_osc);

}


/*!****************************************************************************
 ******************************************************************************
\note  addOscData
\date  Aug. 2010

\remarks

add an entry to the oscilloscope ring buffers

 *****************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  entry: a data entry

 *****************************************************************************/
static void 
addOscData(SL_oscEntry entry)
{
  int i,j,c,r;
  int pID;
  int tID = 0;

  if (entry.plotID < 0 || entry.plotID > n_oscilloscope_plots)
    return;

  // catch rese messages
  if (strcmp(entry.name,"osc_var_reset")==0) {

    for (r=1; r<=n_oscilloscope_plots; ++r) {
      for (j=1; j<=MAX_VARS_PER_PLOT; ++j) {
        osc_data[r].current_index[j] = 1;
        osc_data[r].n_data[j] = 0;
      }
      osc_data[r].max = -1.e10;
      osc_data[r].min =  1.e10;
      osc_data[r].n_active = 0;
    }    

    return;
  }

  pID = entry.plotID;

  // loock for existing name
  for (i=1; i<=osc_data[pID].n_active; ++i) {
    if (strcmp(osc_data[pID].names[i],entry.name)==0) {
      tID = i;
      break;
    }
  }

  // add new name to plot
  if (tID == 0 && osc_data[pID].n_active < MAX_VARS_PER_PLOT) {

    ++osc_data[pID].n_active;
    tID = osc_data[pID].n_active;
    strcpy(osc_data[pID].names[tID],entry.name);
    osc_data[pID].current_index[tID] = 1;
    osc_data[pID].n_data[tID] = 0;

  } else if (tID == 0)
    return;

  // add the data
  c = osc_data[pID].current_index[tID] + 1;
  if (c > MAX_OSC_DATA)
    c = 1;

  osc_data[pID].current_index[tID] = c;
  if (++osc_data[pID].n_data[tID] > MAX_OSC_DATA)
    osc_data[pID].n_data[tID] = MAX_OSC_DATA;
  osc_data[pID].data[tID][c][1] = entry.ts;
  osc_data[pID].data[tID][c][2] = entry.v;

  if (entry.v > osc_data[pID].max)
    osc_data[pID].max = entry.v;

  if (entry.v < osc_data[pID].min)
    osc_data[pID].min = entry.v;

}
