/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_simulation_servo.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  functions used in the simulation servo

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_integrate.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_simulation_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_oscilloscope.h"
#include "SL_userSimulation.h"

#define TIME_OUT_NS  1000000000

// global variables
int     servo_enabled;
long    simulation_servo_calls=0;
int     simulation_servo_rate;
int     simulation_servo_errors=0;
double  last_simulation_servo_time=0;
double  simulation_servo_time=0;
double  servo_time=0;


int     n_integration = 1;   
int     integrate_method = INTEGRATE_EULER;
int     real_time     = FALSE;

// local variables
double *controller_gain_th;
double *controller_gain_thd;
double *controller_gain_int;


// global functions 

// local functions
static void setIntRate(void);
static void setIntMethod(void);
static void status(void);


static void toggleRealTime(void);
static void initDataCollection(void);
static void where_gains(void);

// external functions

/*!*****************************************************************************
 *******************************************************************************
\note  where_gains
\date  Nov 2010
\remarks 

 printf the PID gains of all the DOFs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static void
where_gains(void)
{
  int i,j;
  int start=1;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: simulation servo is not running!!\n");
  }

  extern double        servo_time;
  printf("Current Gains %f:\n",servo_time);

  for (i=start; i<=start+n_dofs-1; ++i) {

    printf("%2d: %5s: P=% 5.3f D=% 6.3f I=% 6.2f\n",
	   i,joint_names[i],
	   controller_gain_th[i],
	   controller_gain_thd[i],
	   controller_gain_int[i]
	   );

  }
  printf("\n");

}
/*!*****************************************************************************
 *******************************************************************************
\note  init_simulation_servo
\date  Nov. 2007
\remarks 

initializes everything for this servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
int 
init_simulation_servo(void)
{
  int i, j;

  // the servo name
  sprintf(servo_name,"sim");

  // initialization of variables
  simulation_servo_rate = servo_base_rate;

  // read controller gains
  controller_gain_th = my_vector(1,n_dofs);
  controller_gain_thd = my_vector(1,n_dofs);
  controller_gain_int = my_vector(1,n_dofs);
  if (!read_gains(config_files[GAINS],controller_gain_th, 
		  controller_gain_thd, controller_gain_int))
    return FALSE;

  /* inverse dynamics */
  if (!init_dynamics()) 
    return FALSE;
  
  /* initialize kinematicss */
  init_kinematics();
  
  // get shared memory
  if (!init_shared_memory())
    return FALSE;

  // object handling
  if (!initObjects())
    return FALSE;

  // need sensor offsets
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    return FALSE;

  // initializes user specific issues
  if (!init_user_simulation())
    return FALSE;

  // add to man pages 
  addToMan("setIntRate","set number of integration cycles",setIntRate);
  addToMan("setIntMethod","set integration method",setIntMethod);
  addToMan("realTime","toggle real-time processing",toggleRealTime);
  addToMan("status","displays status information about servo",status);
  addToMan("dss","disables the simulation servo",dss);
  addToMan("where_gains","Print gains of the joints",where_gains);

  // data collection
  initDataCollection();

  // oscilloscope
  initOsc();

  // initialize user specific simulations
  initUserSim();                // general initialization
  if (!initUserSimulation())    // user specific intialization
    return FALSE;

  servo_enabled = TRUE;

  return TRUE;

}
 
/*!*****************************************************************************
 *******************************************************************************
\note  run_simulation_servo
\date  Nov 2007
\remarks 

        main functions executed during running the simulation servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
run_simulation_servo(void)

{
  int    i;
  double k,kd;
  double delta;
  double dt;
  int    dtick;
  double max_vel = 10.;
  double aux;

  static double last_time = 0;
  static double current_time = 0;


  // advance the simulation servo
  ++simulation_servo_calls;
  simulation_servo_time += 1./(double)simulation_servo_rate;
  servo_time = simulation_servo_time;

  // check for missed calls to the servo
  dtick = round((simulation_servo_time - last_simulation_servo_time)*(double)simulation_servo_rate);
  if (dtick != 1 && simulation_servo_calls > 2) // need transient ticks to sync servos
    simulation_servo_errors += abs(dtick-1);

  // first, send out all the current state variables
  // to shared memory
  send_sim_state();
  send_misc_sensors();
  send_contacts();

  // zero any external forces
  bzero((void *)uext_sim,sizeof(SL_uext)*(n_dofs+1));

  // read the commands
  receive_des_commands();

  // check for messages
  checkForMessages();

  // only after the write/read steps above, trigger the motor servo to avoid that
  // the motor servo can read data that has the wrong time stamp
  semGive(sm_motor_servo_sem);

  // real-time processing if needed 
#ifdef __XENO__
  RTIME t = rt_timer_read();
  current_time = (double)t/1.e9;

  if (real_time) {
    double delta_time;
    
    delta_time = (1./(double)simulation_servo_rate-(current_time - last_time));
    if (delta_time < 0)
      delta_time = 0;
    else
      taskDelay(ns2ticks((long)(delta_time*1.e9)));

    RTIME t = rt_timer_read();
    current_time = (double)t/1.e9;

  }
#else
  struct timeval t;
  gettimeofday(&t,NULL);
  current_time = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
  if (real_time) {
    while (current_time - last_time < 1./(double)simulation_servo_rate) {
      taskDelay(ns2ticks((long)(1./((double)simulation_servo_rate)*1000000000./5.)));
      gettimeofday(&t,NULL);
      current_time = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
    }
  }
#endif
  last_time = current_time;

  // check limits
  for (i=1; i<=n_dofs; ++i) {

    // use Geyer limited rebound model for (set aux=1 to avoid)
    k  = controller_gain_th[i]*10.0;
    kd = controller_gain_thd[i]*sqrt(10.0);

    delta = joint_sim_state[i].th - joint_range[i][MIN_THETA];

    if ( delta < 0 ){ // only print at 10Hz
      if ((int)(((double)simulation_servo_calls)/
		(((double)simulation_servo_rate)/10.))%10 == 0) {
	printf("-%s",joint_names[i]);
	fflush(stdout);
      }
      
      if (joint_sim_state[i].thd > max_vel) 
	aux = 0.0;
      else 
	aux = 1.-joint_sim_state[i].thd/max_vel;

      joint_sim_state[i].u += - delta * k * aux - joint_sim_state[i].thd * kd * aux;
    }

    delta = joint_range[i][MAX_THETA] - joint_sim_state[i].th;
    if (delta < 0){ // only print at 10Hz
      if ((int)(((double)simulation_servo_calls)/
		(((double)simulation_servo_rate)/10.))%10 == 0) {
	printf("+%s",joint_names[i]);
	fflush(stdout);
      }

      if (joint_sim_state[i].thd < -max_vel) 
	aux = 0.0;
      else 
	aux = 1.-joint_sim_state[i].thd/(-max_vel);

      joint_sim_state[i].u += delta * k * aux - joint_sim_state[i].thd * kd * aux;

    }
  }

  // general numerical integration: integration runs at higher rate
  dt = 1./(double)(simulation_servo_rate)/(double)n_integration;

  for (i=1; i<=n_integration; ++i) {
    
    // integrate the simulation
    switch (integrate_method) {
    case INTEGRATE_RK:
      SL_IntegrateRK(joint_sim_state, &base_state, 
		     &base_orient, ucontact, endeff,dt,n_dofs);
      break;
      
    case INTEGRATE_EULER:
      SL_IntegrateEuler(joint_sim_state, &base_state, 
			&base_orient, ucontact, endeff,dt,n_dofs,TRUE);
      break;
      
    default:
      printf("invalid integration method\n");
      
    }
    
  }

  // compute miscellenous sensors
  run_user_simulation();

  // run user specific simulations
  runUserSimulation();

  // data collection
  writeToBuffer();

  last_simulation_servo_time = simulation_servo_time;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  toggle_real_time
\date  Nov. 2005
   
\remarks 

        resets simulation to initial conditions

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void 
toggleRealTime(void) 
{
  int i;

  if (real_time == 0) {
    real_time = TRUE;
    printf("Real-time processing switched on\n");
  } else {
    real_time = FALSE;
    printf("Real-time processing switched off\n");
  }
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  initDataCollection
\date  Feb. 2004
   
\remarks 

        initializes all variables for data collection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static void
initDataCollection(void)

{
  int i;
  char string[100];

  /* this will be updated later by the servos */
  initCollectData(simulation_servo_rate);

  /* add variables to data collection */
  for (i=1; i<=n_dofs; ++i) {

    printf("%d...",i);
    fflush(stdout);

    sprintf(string,"%s_th",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].th),string,"rad", DOUBLE,FALSE);
    sprintf(string,"%s_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].thd),string,"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].thdd),string,"rad/s^2", DOUBLE,FALSE);

    sprintf(string,"%s_u",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].u),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_ufb",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].ufb),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_sim_state[i].load),string,"Nm", DOUBLE,FALSE);

    sprintf(string,"%s_cfx",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].f[_X_]),string,"N", DOUBLE,FALSE);
    sprintf(string,"%s_cfy",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].f[_Y_]),string,"N", DOUBLE,FALSE);
    sprintf(string,"%s_cfz",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].f[_Z_]),string,"N", DOUBLE,FALSE);

    sprintf(string,"%s_ctx",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].t[_A_]),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_cty",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].t[_B_]),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_ctz",joint_names[i]);
    addVarToCollect((char *)&(ucontact[i].t[_G_]),string,"Nm", DOUBLE,FALSE);
  }

  /* the state of the base */
  addVarToCollect((char *)&(base_state.x[_X_]),"base_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Y_]),"base_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Z_]),"base_z","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xd[_X_]),"base_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Y_]),"base_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Z_]),"base_zd","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xdd[_X_]),"base_xdd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Y_]),"base_ydd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Z_]),"base_zdd","m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_orient.q[_Q0_]),"base_q0","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q1_]),"base_q1","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q2_]),"base_q2","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q3_]),"base_q3","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.qd[_Q0_]),"base_qd0","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q1_]),"base_qd1","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q2_]),"base_qd2","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q3_]),"base_qd3","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.ad[_A_]),"base_ad","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_B_]),"base_bd","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_G_]),"base_gd","-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.add[_A_]),"base_add","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_B_]),"base_bdd","-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_G_]),"base_gdd","-",DOUBLE,FALSE);

  for (i=0; i<=n_contacts; ++i) {

    sprintf(string,"CP%d_cstat",i);
    addVarToCollect((char *)&(contacts[i].status),string,"-",INT,FALSE);

    sprintf(string,"CP%d_cx",i);
    addVarToCollect((char *)&(contacts[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cy",i);
    addVarToCollect((char *)&(contacts[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cz",i);
    addVarToCollect((char *)&(contacts[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_cnx",i);
    addVarToCollect((char *)&(contacts[i].normal[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cny",i);
    addVarToCollect((char *)&(contacts[i].normal[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cnz",i);
    addVarToCollect((char *)&(contacts[i].normal[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_cnxd",i);
    addVarToCollect((char *)&(contacts[i].normvel[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cnyd",i);
    addVarToCollect((char *)&(contacts[i].normvel[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cnzd",i);
    addVarToCollect((char *)&(contacts[i].normvel[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_ctx",i);
    addVarToCollect((char *)&(contacts[i].tangent[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cty",i);
    addVarToCollect((char *)&(contacts[i].tangent[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_ctz",i);
    addVarToCollect((char *)&(contacts[i].tangent[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_ctxd",i);
    addVarToCollect((char *)&(contacts[i].tanvel[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_ctyd",i);
    addVarToCollect((char *)&(contacts[i].tanvel[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_ctzd",i);
    addVarToCollect((char *)&(contacts[i].tanvel[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_cvxd",i);
    addVarToCollect((char *)&(contacts[i].viscvel[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cvyd",i);
    addVarToCollect((char *)&(contacts[i].viscvel[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cvzd",i);
    addVarToCollect((char *)&(contacts[i].viscvel[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"CP%d_cfx",i);
    addVarToCollect((char *)&(contacts[i].f[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cfy",i);
    addVarToCollect((char *)&(contacts[i].f[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"CP%d_cfz",i);
    addVarToCollect((char *)&(contacts[i].f[_Z_]),string,"m",DOUBLE,FALSE);

  }
  
  updateDataCollectScript();

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
  printf("            Time                   = %f\n",simulation_servo_time);
  printf("            Servo Calls            = %ld\n",simulation_servo_calls);
  printf("            Servo Rate             = %d\n",simulation_servo_rate);
  printf("            Servo Errors           = %d\n",simulation_servo_errors);
  printf("            Real-Time Flag         = %d\n",real_time);
  printf("            Gravity                = %f\n",gravity);
  printf("            Integration Rate       = %d\n",n_integration);
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
\note  setIntRate
\date  April 2006
\remarks 

 sets the numbers of integration cycles

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void
setIntRate(void)
{
  int i;

  i = n_integration;
  get_int("Number of numercial integrations per servo cycle",i,&i);
  n_integration = i;

}
/*!*****************************************************************************
 *******************************************************************************
\note  setIntMethod
\date  April 2006
\remarks 

 sets the numbers of integration cycles

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void
setIntMethod(void)
{

  if (integrate_method == INTEGRATE_EULER) {
    integrate_method = INTEGRATE_RK;
    printf("Switched to Runge Kutta Integration (Rate=%d)\n",n_integration);
  } else {
    printf("Switched to Euler Integration (Rate=%d)\n",n_integration);
    integrate_method = INTEGRATE_EULER;
  }

}


/*!*****************************************************************************
 *******************************************************************************
\note  reset
\date  Nov. 2005
   
\remarks 

        resets simulation to initial conditions

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void 
reset(void) 
{
  int i;
  
  bzero((char *)&(joint_sim_state[1]),sizeof(SL_Jstate)*n_dofs);
  bzero((void *)ucontact,sizeof(SL_uext)*(n_dofs+1));
  for (i=1; i<=n_dofs; ++i)
    joint_sim_state[i].th = joint_default_state[i].th;
  bzero((void *)&base_state,sizeof(SL_Cstate));
  bzero((void *)&base_orient,sizeof(SL_quat));

  base_orient.q[_Q0_] = freeze_base_quat[_Q0_];
  base_orient.q[_Q1_] = freeze_base_quat[_Q1_];
  base_orient.q[_Q2_] = freeze_base_quat[_Q2_];
  base_orient.q[_Q3_] = freeze_base_quat[_Q3_];

  base_state.x[_X_] = freeze_base_pos[_X_];
  base_state.x[_Y_] = freeze_base_pos[_Y_];
  base_state.x[_Z_] = freeze_base_pos[_Z_];

  printf("Simulation was reset\n");

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
  if (semTake(sm_simulation_message_ready_sem,NO_WAIT) == ERROR) {
    return FALSE;
  }

  // receive the message
  if (semTake(sm_simulation_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take simulation message semaphore\n");
    return FALSE;
  }


  for (k=1; k<=sm_simulation_message->n_msgs; ++k) {

    // get the name of this message
    strcpy(name,sm_simulation_message->name[k]);

    // act according to the message name

    // -------------------------------------------------------------------------
    if (strcmp(name,"reset") == 0) { // reset simulation -----------------------
      float buf[N_CART+N_QUAT+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float)*(N_CART+N_QUAT));
      
      j = 0;
      for (i=1; i<=N_CART; ++i) {
	freeze_base_pos[i] = buf[++j];
      }
      for (i=1; i<=N_QUAT; ++i) {
	freeze_base_quat[i] = buf[++j];
      }
      reset();
      

    // -------------------------------------------------------------------------
    } else if (strcmp(name,"changePIDGains") == 0) { // change the gains -------
      float buf[n_dofs*3+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float)*(3*n_dofs));

      for (i=1; i<=n_dofs; ++i) {
	controller_gain_th[i]  = (double) buf[i];
	controller_gain_thd[i] = (double) buf[i+n_dofs];
	controller_gain_int[i] = (double) buf[i+2*n_dofs];
      }

    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"where_gains") == 0) { 
      where_gains();
      
    // -------------------------------------------------------------------------
    } else if (strcmp(name,"setUextSim") == 0) { // receive external simulated forces
      int   count = 0;
      float buf[n_dofs*N_CART*2+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float)*(2*n_dofs*N_CART));

      for (i=1; i<=n_dofs; ++i) {
	for (j=1; j<=N_CART; ++j) {
	  uext_sim[i].f[j] = buf[++count];
	  uext_sim[i].t[j] = buf[++count];
	}
      }

    // -------------------------------------------------------------------------
    } else if (strcmp(name,"changeRealTime") == 0) { // real time simualtion on/off
      float buf[1+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float));
      
      if (buf[1] == 0) {
	real_time = FALSE;
	printf("Real-time processing switched off\n");
      } else {
	real_time = TRUE;
	printf("Real-time processing switched on\n");
      }
      
    // -------------------------------------------------------------------------
    } else if (strcmp(name,"freezeBase") == 0) {    // freeze base in simualtion on/off
      float buf[1+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float));
      
      if (buf[1] == 0) {
	freeze_base = FALSE;
	printf("Freeze base switched off\n");
      } else {
	freeze_base = TRUE;
	printf("Freeze base switched on\n");
      }
      
    // -------------------------------------------------------------------------
    } else if (strcmp(name,"setG") == 0) {     // change the gravity constant 
      float buf[1+1];
      
      memcpy(&(buf[1]),sm_simulation_message->buf+sm_simulation_message->moff[k],
	     sizeof(float));
      
      gravity = buf[1];
      printf("Gravity set to %f\n",gravity);
      

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
      
      memcpy(&data,sm_simulation_message->buf+sm_simulation_message->moff[k],sizeof(data));
      addObject(data.name, data.type, data.contact_model, data.rgb, data.trans, data.rot, 
		data.scale, data.contact_parms,data.object_parms);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"hideObject") == 0) {
      struct {
	int  hide;
	char obj_name[100];
      } data;
      
      memcpy(&data,sm_simulation_message->buf+sm_simulation_message->moff[k],sizeof(data));
      changeHideObjByName(data.obj_name, data.hide);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"changeObjectPos") == 0) {
      struct {
	char   obj_name[100];
	double pos[N_CART+1];
	double rot[N_CART+1];
      } data;

      memcpy(&data,sm_simulation_message->buf+sm_simulation_message->moff[k],sizeof(data));
      changeObjPosByName(data.obj_name, data.pos, data.rot);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"deleteObject") == 0) {
      struct {
	char obj_name[100];
      } data;
      
      memcpy(&data,sm_simulation_message->buf+sm_simulation_message->moff[k],sizeof(data));
      deleteObjByName(data.obj_name);
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"status") == 0) { 

      status();

    }
    // ---------------------------------------------------------------------------
    
    
  }

  // give back semaphore
  sm_simulation_message->n_msgs = 0;
  sm_simulation_message->n_bytes_used = 0;
  semGive(sm_simulation_message_sem);


  return TRUE;
}


/*!*****************************************************************************
*******************************************************************************
\note  receive_des_commands
\date  Nov. 2007
   
\remarks 

reads the commands from the joint_sim_state shared memory
structure
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
receive_des_commands(void)
{
  
  int i;
  extern double *controller_gain_th;
  extern double *controller_gain_thd;


  if (semTake(sm_des_commands_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i) {
    joint_sim_state[i].u   = (double) sm_des_commands->des_command[i].u - 
      (double) sm_des_commands->des_command[i].upd;
    joint_des_state[i].th  = (double) sm_des_commands->des_command[i].th;
    joint_des_state[i].thd = (double) sm_des_commands->des_command[i].thd;
  }
  
  // re-generate and add the PD command from the motor servo, as the
  // delay between motor servo and simulation servo can cause instabilities 
  // in case of stiff contacts -- the reason is that the state info on the 
  // motor servo is one tick delayed; the old PD command has been subtracted
  // from the total command above
  
  for (i=1; i<=n_dofs; ++i) {
    if (sm_des_commands->des_command[i].upd != 0.0) {
      joint_sim_state[i].u += (joint_des_state[i].th - joint_sim_state[i].th) * 
	controller_gain_th[i];
      joint_sim_state[i].u += (joint_des_state[i].thd - joint_sim_state[i].thd) * 
	controller_gain_thd[i];
    }

    if (fabs(joint_sim_state[i].u) > u_max[i]) {
      joint_sim_state[i].u = macro_sign(joint_sim_state[i].u) * u_max[i];
    }

  }
  
  semGive(sm_des_commands_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_sim_state
\date  Nov. 2007
   
\remarks 

sends the entire joint_sim_state to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_sim_state(void)
{
  
  int i;

  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,DOUBLE2FLOAT);
    
  for (i=1; i<=n_dofs; ++i)
    sm_joint_sim_state->joint_sim_state[i] = sm_joint_sim_state_data[i];

  sm_joint_sim_state->ts = simulation_servo_time;
  
  semGive(sm_joint_sim_state_sem);


  // base state
  if (semTake(sm_base_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);

  sm_base_state->state[1] = sm_base_state_data[1];

  sm_base_state->ts = simulation_servo_time;
  
  semGive(sm_base_state_sem);


  // base orient
  if (semTake(sm_base_orient_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  cSL_quat(&base_orient-1, sm_base_orient_data, 1, DOUBLE2FLOAT);

  sm_base_orient->orient[1] = sm_base_orient_data[1];

  sm_base_orient->ts = simulation_servo_time;
  
  semGive(sm_base_orient_sem);


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_misc_sensors
\date  Nov. 2007
   
\remarks 

sends the entire misc_sim_sensors to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_misc_sensors(void)
{
  
  int i;

  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_misc_sensors; ++i)
    sm_misc_sim_sensor->value[i] = misc_sim_sensor[i];

  sm_misc_sim_sensor->ts = simulation_servo_time;
  
  semGive(sm_misc_sim_sensor_sem);

  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  send_contacts
\date  Nov. 2007
   
\remarks 

sends the contact forces to shared memory
	

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_contacts(void)
{
  
  int i,j;

  if (semTake(sm_contacts_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++simulation_servo_errors;
    return FALSE;

  } 

  for (i=0; i<=n_contacts; ++i) {
    sm_contacts->contact[i].status = contacts[i].status;
    if (contacts[i].status) {

      for (j=1; j<=N_CART; ++j) {
	sm_contacts->contact[i].f[j] = contacts[i].f[j];
	sm_contacts->contact[i].n[j] = contacts[i].n[j];
      }
      strncpy(sm_contacts->contact[i].name,contacts[i].optr->name,STRING100);
    } else {
      for (j=1; j<=N_CART; ++j) {
	sm_contacts->contact[i].f[j] = 0.0;
	sm_contacts->contact[i].n[j] = 0.0;
      }
      strncpy(sm_contacts->contact[i].name,"\0",STRING100);
    }
  }
  
  semGive(sm_contacts_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  dss & disable_simulation_servo
\date  July 2010
   
\remarks 

disables the simulation servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none
     
 ******************************************************************************/
void 
dss(void)
{
  disable_simulation_servo();
}

void 
disable_simulation_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Simulation Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "Simulation Servo is not on!\n" );
  
}


















