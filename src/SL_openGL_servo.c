/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL_servo.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  the main functions for the openGL_servo
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// openGL headers
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

// local headers
#include "SL.h"
#include "SL_openGL.h"
#include "SL_common.h"
#include "SL_objects.h"
#include "utility.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "SL_userGraphics.h"

//! time out for semaphores
#define TIME_OUT_NS 1000000000

// global variables
long    openGL_servo_calls=0;
long    last_openGL_servo_calls=0;
int     openGL_servo_rate = R60HZ;
int     openGL_servo_errors=0;
double  openGL_servo_time=0;
double  servo_time=0;
int     servo_enabled;
int     stand_alone_flag = FALSE;
  
// local functions 
static void status(void);
static void togglePause(void);
static void dos(void);
static void disable_openGL_servo(void);

  
/*!*****************************************************************************
*******************************************************************************
\note  init_openGL_servo
\date  July 1998
 
\remarks 
 
initialization of the servo
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 
******************************************************************************/
int 
init_openGL_servo(int argc, char** argv)

{
  int i,j,n;
  int rc;
  int ans; 

  // servo name
  sprintf(servo_name,"openGL");

  // add to man pages 
  addToMan("status","displays status information about servo",status);
  addToMan("p","toggles pausing of the simulation",togglePause);
  addToMan("dos","disables the openGL servo",dos);

  /* inverse dynamics */
  if (!init_dynamics()) 
    return FALSE;
  
  // initialize kinematics
  init_kinematics();
  
  // initialize graphics
  if (!initGraphics(&argc, &argv))
    return FALSE;
  
  // generic initialization program
  if (!init_user_openGL(argc, argv)) 
    return FALSE;

  // initialize user specific graphics
  initUserGraph();               // general initialization
  if (!initUserGraphics())       // user specific intialization
    return FALSE;

  // start the main loop
  servo_enabled = TRUE;
  
  return TRUE;
}


/*!*****************************************************************************
*******************************************************************************
\note  receive_sim_state
\date  Nov. 2007
   
\remarks 

recieves the entire joint_sim_state from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_sim_state(void)
{
  
  int i;
  double aux;
  static int firsttime = TRUE;

  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i)
    sm_joint_sim_state_data[i] = sm_joint_sim_state->joint_sim_state[i];
  
  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,FLOAT2DOUBLE);

  openGL_servo_time = servo_time = sm_joint_sim_state->ts;
    
  semGive(sm_joint_sim_state_sem);

  // base state
  if (semTake(sm_base_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  if (firsttime) { // this is a shared memory initialzation
    cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);
    sm_base_state->state[1] = sm_base_state_data[1];
    sm_base_state->ts = openGL_servo_time;
  }

  sm_base_state_data[1] = sm_base_state->state[1];
  cSL_Cstate(&base_state-1, sm_base_state_data, 1, FLOAT2DOUBLE);
  semGive(sm_base_state_sem);


  // base orient
  if (semTake(sm_base_orient_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  if (firsttime) { // this is a shared memory initialzation
    cSL_quat(&base_orient-1, sm_base_orient_data, 1, DOUBLE2FLOAT);
    sm_base_orient->orient[1] = sm_base_orient_data[1];
    sm_base_orient->ts = openGL_servo_time;
  }

  sm_base_orient_data[1] = sm_base_orient->orient[1];
  cSL_quat(&base_orient-1, sm_base_orient_data, 1, FLOAT2DOUBLE);
  semGive(sm_base_orient_sem);

  firsttime = FALSE;

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  receive_misc_sensors
\date  Nov. 2007
   
\remarks 

receives the entire misc_sim_sensors from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_misc_sensors(void)
{
  
  int i;

  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_misc_sensors; ++i)
    misc_sim_sensor[i] = sm_misc_sim_sensor->value[i];
  
  semGive(sm_misc_sim_sensor_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  receive_contacts
\date  Nov. 2007
   
\remarks 

receive the contact forces from shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_contacts(void)
{
  
  int i,j;

  if (semTake(sm_contacts_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++openGL_servo_errors;
    return FALSE;

  } 

  for (i=0; i<=n_contacts; ++i) {
    contacts[i].status = sm_contacts->contact[i].status;
    if (contacts[i].status) {
      for (j=1; j<=N_CART; ++j) {
	contacts[i].f[j] = sm_contacts->contact[i].f[j];
	contacts[i].n[j] = sm_contacts->contact[i].n[j];
      }
      contacts[i].optr = getObjPtrByName(sm_contacts->contact[i].name);
    } else {
      for (j=1; j<=N_CART; ++j) {
	contacts[i].f[j] = 0.0;
	contacts[i].n[j] = 0.0;
      }
      contacts[i].optr = NULL;
    }
  }
  
  semGive(sm_contacts_sem);

  return TRUE;
}


/*!*****************************************************************************
*******************************************************************************
\note  status
\date  Nov 2007
   
\remarks 

prints out all important variables

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
status(void)
{

  printf("\n");
  printf("            Time                   = %f\n",openGL_servo_time);
  printf("            Servo Calls            = %ld\n",openGL_servo_calls);
  printf("            Servo Rate             = %d\n",openGL_servo_rate);
  printf("            Servo Errors           = %d\n",openGL_servo_errors);
  printf("            CLMCplot Mode          = %d\n",clmcplot_mode);
  printf("            Playback Mode          = %d\n",playback_mode);
  printf("            Pause Flag             = %d\n",pause_flag);
  printf("            Window Update Rate     = %5.2f\n",window_update_rate);
  printf("            Stand Alone Flag       = %d\n",stand_alone_flag);
#ifdef __XENO__
  extern long count_xenomai_mode_switches;
  extern int  delay_ns;
  printf("            Xeonmai Mode Swiches   = %ld\n",count_xenomai_mode_switches);
  printf("            Delay [ns]             = %d\n",delay_ns);

#endif

  printf("\n");

}

/*!*****************************************************************************
*******************************************************************************
\note  togglePause
\date  Nov. 2005
   
\remarks 

pauses the simulation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
togglePause(void) 
{
  if (pause_flag==0){
    pause_flag=1;
    printf("Pausing the simulation\n");
  }
  else  {
    pause_flag=0;
    printf("Resuming the simulation\n");
    semFlush(sm_simulation_servo_sem);
  }
}  

/*!*****************************************************************************
*******************************************************************************
\note  checkForMessages
\date  Nov. 2007
   
\remarks 

Messages can be given to the servo for hard-coded tasks.This allows
some information passing between the different processes on variables
of common interest, e.g., the endeffector specs, object information,
etc.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
checkForMessages(void)
{
  int i,j,k;
  char name[20];

  // check whether a message is available
  if (semTake(sm_openGL_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_openGL_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take openGL message semaphore\n");
    return FALSE;
  }

  for (k=1; k<=sm_openGL_message->n_msgs; ++k) {

    // get the name of this message
    strcpy(name,sm_openGL_message->name[k]);
    
    // act according to the message name
    // ---------------------------------------------------------------------------
    if (strcmp(name,"followBase") == 0) { 
      
      followBaseByName(robot_name, TRUE);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"toggleShowAxes") == 0) { 
      struct {
	int  status;
      } data;

      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));

      toggleShowAxesByName(robot_name,data.status);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"addObject") == 0) {
      struct {
	char    name[STRING100];                    /*!< object name */
	int     type;                               /*!< object type */
	double  trans[N_CART+1];                    /*!< translatory offset of object */
	double  rot[N_CART+1];                      /*!< rotational offset of object */
	double  scale[N_CART+1];                    /*!< scaling in x,y,z */
	double  rgb[N_CART+1];                      /*!< color information */
	double  object_parms[MAX_OBJ_PARMS+1];      /*!< object parameters */
	int     contact_model;                      /*!< which contact model to be used */
	double  contact_parms[MAX_CONTACT_PARMS+1]; /*!< contact parameters */
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      addObject(data.name, data.type, data.contact_model, data.rgb, data.trans, data.rot, 
		data.scale, data.contact_parms,data.object_parms);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"hideObject") == 0) {
      struct {
	int  hide;
	char obj_name[100];
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      changeHideObjByName(data.obj_name, data.hide);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"changeObjectPos") == 0) {
      struct {
	char   obj_name[100];
	double pos[N_CART+1];
	double rot[N_CART+1];
      } data;

      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      changeObjPosByName(data.obj_name, data.pos, data.rot);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"deleteObject") == 0) {
      struct {
	char obj_name[100];
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      deleteObjByName(data.obj_name);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"changeObjPosByName") == 0) { 
      struct {
	char   obj_name[100];
	double pos[N_CART+1];
	double rot[N_CART+1];
      } data;
      unsigned char buf[sizeof(data)];
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      changeObjPosByName(data.obj_name,data.pos,data.rot);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"hideWindowByName") == 0) { 
      char cbuf[101];
      
      memcpy(&cbuf,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(cbuf));
      printf("%s %d\n",cbuf,cbuf[100]);
      hideWindowByName(cbuf, (int)(cbuf[sizeof(cbuf)-1]));
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"switchCometDisplay") == 0) { 
      struct {
	int status;
	int n_steps;
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      if (data.status == TRUE)
	switchCometDisplay(data.status,data.n_steps);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"updateOscTimeWindow") == 0) { 
      extern double time_window_vars;
      struct {
	float w;
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      time_window_vars = data.w;
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"updateOscPeriodsAD") == 0) { 
      extern int periods_window_AD;
      struct {
	int w;
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));
      periods_window_AD = data.w;
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"resetCometDisplay") == 0) { 

      resetCometDisplay();

    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"status") == 0) { 

      status();

    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"resetCometVars") == 0) { 

      for (i=1; i<=n_endeffs; ++i)
	switchEndeffectorCometDisplay(i,FALSE);

      for (i=1; i<=n_links; ++i)
	switchLinkCometDisplay(i,FALSE);

    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"switchCometVars") == 0) { 
      struct {
	int endeffID;
	int endeffStatus;
	int linkID;
	int linkStatus;
      } data;
      
      memcpy(&data,sm_openGL_message->buf+sm_openGL_message->moff[k],sizeof(data));

      if (data.endeffID != 0)
	switchEndeffectorCometDisplay(data.endeffID,data.endeffStatus);

      if (data.linkID != 0)
	switchLinkCometDisplay(data.linkID,data.linkStatus);
      
    }

  }

  // give back semaphore
  sm_openGL_message->n_msgs = 0;
  sm_openGL_message->n_bytes_used = 0;
  semGive(sm_openGL_message_sem);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  dos & disable_openGL_servo
\date  July 2010
   
\remarks 

disables the openGL servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none
     
 ******************************************************************************/
void 
dos(void)
{
  disable_openGL_servo();
}

void 
disable_openGL_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("OpenGL Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "OpenGL Servo is not on!\n" );
  
}
