/*!=============================================================================
  ==============================================================================

  \ingroup SLros
  
  \file    SL_ros_servo.cpp

  \author  Stefan Schaal
  \date    July 2010

  ==============================================================================
  \remarks

  manages the communcation to the ROS master node

  ============================================================================*/

#include "SL_ros_communicator.h"

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_ros_servo.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_oscilloscope.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"

#define TIME_OUT_NS  1000000

/* variables for the task servo */
int    ros_servo_errors;
long   ros_servo_calls=0;
int    ros_servo_initialized = FALSE;
double last_ros_servo_time=0;
double ros_servo_time=0;
double servo_time=0;
int    servo_enabled;
int    ros_servo_rate;

static sl2ros::SL_ros_communicator ros_communicator_;
static int default_publisher_enabled_ = 0;

/* local variables */

/* global functions */

/* local functions */
static int  receive_ros_state(void);
static void compute_kinematics(void);
static int  checkForMessages(void);
static void disable_ros_servo(void);
static void drs(void);

/*!*****************************************************************************
 *******************************************************************************
  \note  init_ros_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
init_ros_servo(void)
{
  int j, i;
  char string[100];

  if (ros_servo_initialized) {
    printf("Task Servo is already initialized\n");
    return;
  }

  // the servo name
  sprintf(servo_name,"ros");

  // initialize shared memories and shared semaphores
  if (!init_shared_memory())
    return;

  /* inverse dynamics and other basic kinematic stuff*/
  if (!init_dynamics())
    return;

  // initialize kinematics
  init_kinematics();

  // add variables to data collection
  ros_servo_rate=servo_base_rate/task_servo_ratio;
  initCollectData(ros_servo_rate);

  for (i=1; i<=n_dofs; ++i) {
    printf("%d...",i);
    fflush(stdout);
    sprintf(string,"%s_th",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].th),string,(char *)"rad", DOUBLE,FALSE);
    sprintf(string,"%s_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thd),string,(char *)"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thdd),string,(char *)"rad/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_u",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].u),string,(char *)"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_ufb",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].ufb),string,(char *)"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].load),string,(char *)"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_des_th",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].th),string,(char *)"rad",DOUBLE,FALSE);
    sprintf(string,"%s_des_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thd),string,(char *)"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thdd),string,(char *)"rad/s^2",DOUBLE,FALSE);
    sprintf(string,"%s_uff",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].uff),string,(char *)"Nm",DOUBLE,FALSE);
  }

  // Cartesian variables
  for (i=1; i<=n_endeffs; ++i) {

    sprintf(string,"%s_x",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_y",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_z",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_X_]),string,(char *)"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Y_]),string,(char *)"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Z_]),string,(char *)"rad/s",DOUBLE,FALSE);

    sprintf(string,"%s_add",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_X_]),string,(char *)"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_bdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Y_]),string,(char *)"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_gdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Z_]),string,(char *)"rad/s2",DOUBLE,FALSE);

    sprintf(string,"%s_des_x",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_y",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_z",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_X_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Y_]),string,(char *)"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Z_]),string,(char *)"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_X_]),string,(char *)"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Y_]),string,(char *)"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Z_]),string,(char *)"rad/",DOUBLE,FALSE);

    sprintf(string,"%s_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q0_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q1_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q2_]),string,(char *)"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q3_]),string,(char *)"-",DOUBLE,FALSE);

  }

  // misc sensors
  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_sensor[i]),string,(char *)"-",DOUBLE,FALSE);
  }


  // the state of the base
  addVarToCollect((char *)&(base_state.x[_X_]),(char *)"base_x",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Y_]),(char *)"base_y",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.x[_Z_]),(char *)"base_z",(char *)"m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xd[_X_]),(char *)"base_xd",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Y_]),(char *)"base_yd",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xd[_Z_]),(char *)"base_zd",(char *)"m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_state.xdd[_X_]),(char *)"base_xdd",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Y_]),(char *)"base_ydd",(char *)"m",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_state.xdd[_Z_]),(char *)"base_zdd",(char *)"m",DOUBLE,FALSE);
  
  addVarToCollect((char *)&(base_orient.q[_Q0_]),(char *)"base_q0",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q1_]),(char *)"base_q1",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q2_]),(char *)"base_q2",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.q[_Q3_]),(char *)"base_q3",(char *)"-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.qd[_Q0_]),(char *)"base_qd0",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q1_]),(char *)"base_qd1",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q2_]),(char *)"base_qd2",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.qd[_Q3_]),(char *)"base_qd3",(char *)"-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.ad[_A_]),(char *)"base_ad",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_B_]),(char *)"base_bd",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.ad[_G_]),(char *)"base_gd",(char *)"-",DOUBLE,FALSE);

  addVarToCollect((char *)&(base_orient.add[_A_]),(char *)"base_add",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_B_]),(char *)"base_bdd",(char *)"-",DOUBLE,FALSE);
  addVarToCollect((char *)&(base_orient.add[_G_]),(char *)"base_gdd",(char *)"-",DOUBLE,FALSE);

  printf("done\n");

  updateDataCollectScript();

  // add to man pages
  addToMan((char *)"status",(char *)"displays information about the servo",status);
  addToMan((char *)"drs",(char *)"disables the ros servo",drs);

  int argc = 1;
  char name[] = "SL2ROS_Publisher";
  char* argv[1];
  argv[0] = name;
  ros::init(argc, argv, "SL2ROS_Publisher");

  if (!read_parameter_pool_int(config_files[PARAMETERPOOL], "default_publisher_enabled", &default_publisher_enabled_))
    default_publisher_enabled_ = 0;

  if(default_publisher_enabled_ && !ros_communicator_.initialize())
  {
    printf("ERROR: could not initialize ros communication\n");
    return;
  }

  // initializes user specific issues
  init_user_ros();

  // if all worked out, we mark the servo as ready to go
  ros_servo_initialized = TRUE;

  // set oscilloscope to start value
  initOsc();
  setOsc(d2a_cr,0.0);

  scd();
	return;
}

/*!*****************************************************************************
 *******************************************************************************
\note  run_ros_servo
\date  Feb 1999
\remarks 

This program executes the sequence of ros servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
int
run_ros_servo(void)

{
  int    j,i;
  double dt;
  int    dticks;

  setOsc(d2a_cr,0.0);
  
  /*********************************************************************
   * adjust time
   */
  ++ros_servo_calls;
  ros_servo_time += 1./(double)ros_servo_rate;
  servo_time = ros_servo_time;

  // check for unexpected time drift
  dticks = round((ros_servo_time - last_ros_servo_time)*(double)ros_servo_rate);
  if (dticks != 1 && ros_servo_calls > 2) // need transient ticks to sync servos
    ros_servo_errors += abs(dticks-1);


  /*********************************************************************
   * check for messages
   */
  
  checkForMessages();

  /*********************************************************************
   * receive sensory data
   */

  if (!receive_ros_state()) {
    printf("Problem when receiving ros state\n");
    return FALSE;
  }
  
  setOsc(d2a_cr,10.0);

  /*********************************************************************
   * compute useful kinematic variables
   */
  
  compute_kinematics();
  
  setOsc(d2a_cr,20.0);

#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif

  /**********************************************************************
   * do ROS communication
   */
  if (default_publisher_enabled_)
    ros_communicator_.publish();
  
  /**********************************************************************
   * do user specific ROS functions
   */

  run_user_ros();

  ros::spinOnce();

#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif

  setOsc(d2a_cr,90.0);
  
  /*************************************************************************
   * collect data
   */

  writeToBuffer();
  sendOscilloscopeData();


  setOsc(d2a_cr,100.0);

  
  /*************************************************************************
   * end of program sequence
   */

  last_ros_servo_time = ros_servo_time;

  return TRUE;
  
}
 
/*!*****************************************************************************
 *******************************************************************************
\note  status
\date  July 2010
   
\remarks 

prints out all important variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
status(void)
{

  printf("\n");
  printf("            Time                   = %f\n",ros_servo_time);
  printf("            Servo Calls            = %ld\n",ros_servo_calls);
  printf("            Servo Rate             = %d\n",ros_servo_rate);
  printf("            Servo Errors           = %d\n",ros_servo_errors);
  printf("            Servo Initialize       = %d\n",ros_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
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
\note  receive_ros_state
\date  July 2010
   
\remarks 

receives all relevant state variables to ROS process
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int 
receive_ros_state(void)
{
  
  int i,j;
  SL_fJstate  *fJstate;
  SL_fDJstate *fDJstate;
  SL_fCstate  *fCstate;
  SL_fquat    *fquat;
  float       *misc;
  double       ts;
  int          dticks;

  if (semTake(sm_ros_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++ros_servo_errors;
    return FALSE;
  }

  // create pointers to share memory
  fJstate   = (SL_fJstate *) sm_ros_state->data;
  fDJstate  = (SL_fDJstate *) &(sm_ros_state->data[sizeof(SL_fJstate)*(n_dofs+1)]);
  fCstate   = (SL_fCstate *) &(sm_ros_state->data[sizeof(SL_fJstate)*(n_dofs+1)+
						  sizeof(SL_fDJstate)*(n_dofs+1)]);
  fquat     = (SL_fquat *) &(sm_ros_state->data[sizeof(SL_fJstate)*(n_dofs+1)+
						sizeof(SL_fDJstate)*(n_dofs+1)+
						sizeof(SL_fCstate)*(1+1)]);
  misc      = (float *) &(sm_ros_state->data[sizeof(SL_fJstate)*(n_dofs+1)+
					     sizeof(SL_fDJstate)*(n_dofs+1)+
					     sizeof(SL_fCstate)*(1+1)+
					     sizeof(SL_fquat)*(1+1)]);
  
  memcpy((void *)(&sm_joint_state_data[1]),(const void*)(&(fJstate[1])),sizeof(SL_fJstate)*n_dofs);
  memcpy((void *)(&sm_joint_des_state_data[1]),(const void*)(&(fDJstate[1])),sizeof(SL_fDJstate)*n_dofs);
  memcpy((void *)(&sm_base_state_data[1]),(const void*)(&(fCstate[1])),sizeof(SL_fCstate)*1);
  memcpy((void *)(&sm_base_orient_data[1]),(const void*)(&(fquat[1])),sizeof(SL_fquat)*1);
  memcpy((void *)(&sm_misc_sensor_data[1]),(const void*)(&(misc[1])),sizeof(float)*n_misc_sensors);

  cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,FLOAT2DOUBLE);
  cSL_DJstate(joint_des_state, sm_joint_des_state_data, n_dofs,FLOAT2DOUBLE);
  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, FLOAT2DOUBLE);
  cSL_quat((&base_orient)-1, sm_base_orient_data, 1, FLOAT2DOUBLE);
  for (i=1; i<=n_misc_sensors; ++i)
    misc_sensor[i] = (double) sm_misc_sensor_data[i];

  // get time stamp and adjust time
  ros_servo_time = servo_time = sm_ros_state->ts;

  semGive(sm_ros_state_sem);
   
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  compute_kinematics
\date  June 99
   
\remarks 

       computes kinematic variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
static void
compute_kinematics(void)
{
  int i,j,r,o;
  static int firsttime = TRUE;
  static SL_DJstate *temp;
  static Matrix last_J;
  static Matrix last_Jbase;
  
  if (firsttime) {
    temp=(SL_DJstate *)my_calloc((unsigned long)(n_dofs+1),sizeof(SL_DJstate),MY_STOP);
    last_J      = my_matrix(1,6*n_endeffs,1,n_dofs);
    last_Jbase  = my_matrix(1,6*n_endeffs,1,2*N_CART);
  }

  /* compute the desired link positions */
  linkInformationDes(joint_des_state,&base_state,&base_orient,endeff,
		     joint_cog_mpos_des,joint_axis_pos_des,joint_origin_pos_des,
		     link_pos_des,Alink_des,Adof_des);

  /* the desired endeffector information */
  for (i=1; i<=N_CART; ++i) {
    for (j=1; j<=n_endeffs; ++j) {
      cart_des_state[j].x[i] = link_pos_des[link2endeffmap[j]][i];
    }
  }

  /* the desired quaternian of the endeffector */
  for (j=1; j<=n_endeffs; ++j) {
    linkQuat(Alink_des[link2endeffmap[j]],&(cart_des_orient[j]));
  }


  /* addititional link information */
  linkInformation(joint_state,&base_state,&base_orient,endeff,
		  joint_cog_mpos,joint_axis_pos,joint_origin_pos,
		  link_pos,Alink,Adof);

  /* create the endeffector information */
  for (i=1; i<=N_CART; ++i) {
    for (j=1; j<=n_endeffs; ++j) {
      cart_state[j].x[i] = link_pos[link2endeffmap[j]][i];
    }
  }

  /* the quaternian of the endeffector */
  for (j=1; j<=n_endeffs; ++j) {
    linkQuat(Alink[link2endeffmap[j]],&(cart_orient[j]));
  }

  /* the COG position */
  compute_cog();

  /* the jacobian */
  jacobian(link_pos,joint_origin_pos,joint_axis_pos,J);
  baseJacobian(link_pos,joint_origin_pos,joint_axis_pos,Jbase);

  jacobian(link_pos_des,joint_origin_pos_des,joint_axis_pos_des,Jdes);
  baseJacobian(link_pos_des,joint_origin_pos_des,joint_axis_pos_des,Jbasedes);

  /* numerical time derivative of Jacobian */
  if (!firsttime) {
    mat_sub(J,last_J,dJdt);
    mat_mult_scalar(dJdt,(double)ros_servo_rate,dJdt);
    mat_sub(Jbase,last_Jbase,dJbasedt);
    mat_mult_scalar(dJbasedt,(double)ros_servo_rate,dJbasedt);
  }
  mat_equal(J,last_J);
  mat_equal(Jbase,last_Jbase);

  /* compute the cartesian velocities and accelerations */

  for (j=1; j<=n_endeffs; ++j) {

    for (i=1; i<=N_CART; ++i) {

      cart_state[j].xd[i]     = 0.0;
      cart_state[j].xdd[i]    = 0.0;

      cart_orient[j].ad[i]     = 0.0;
      cart_orient[j].add[i]    = 0.0;

      cart_des_state[j].xd[i] = 0.0;
      cart_des_orient[j].ad[i] = 0.0;

      /* contributations from the joints */
      for (r=1; r<=n_dofs; ++r) {
	cart_state[j].xd[i]     += J[(j-1)*6+i][r] * joint_state[r].thd;
	cart_orient[j].ad[i]    += J[(j-1)*6+i+3][r] * joint_state[r].thd;

	cart_des_state[j].xd[i] += Jdes[(j-1)*6+i][r] *joint_des_state[r].thd;
	cart_des_orient[j].ad[i]+= Jdes[(j-1)*6+i+3][r] * joint_des_state[r].thd;

	cart_state[j].xdd[i]    += J[(j-1)*6+i][r] * joint_state[r].thdd + 
	  dJdt[(j-1)*6+i][r] * joint_state[r].thd;
	cart_orient[j].add[i]   += J[(j-1)*6+i+3][r] * joint_state[r].thdd + 
	  dJdt[(j-1)*6+i+3][r] * joint_state[r].thd;
      }

      /* contributations from the base */
      for (r=1; r<=N_CART; ++r) {
	cart_state[j].xd[i]     += Jbase[(j-1)*6+i][r] * base_state.xd[r];
	cart_orient[j].ad[i]    += Jbase[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_state[j].xd[i]     += Jbase[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_orient[j].ad[i]    += Jbase[(j-1)*6+i+3][3+r] * base_orient.ad[r];

	cart_des_state[j].xd[i]     += Jbasedes[(j-1)*6+i][r] * base_state.xd[r];
	cart_des_orient[j].ad[i]    += Jbasedes[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_des_state[j].xd[i]     += Jbasedes[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_des_orient[j].ad[i]    += Jbasedes[(j-1)*6+i+3][3+r] * base_orient.ad[r];

	cart_state[j].xdd[i]    += Jbase[(j-1)*6+i][r] * base_state.xdd[r] + 
	  dJbasedt[(j-1)*6+i][r] * base_state.xd[r];
	cart_orient[j].add[i]   += Jbase[(j-1)*6+i+3][r] * base_state.xdd[r] + 
	  dJbasedt[(j-1)*6+i+3][r] * base_state.xd[r];

	cart_state[j].xdd[i]    += Jbase[(j-1)*6+i][3+r] * base_orient.add[r] + 
	  dJbasedt[(j-1)*6+i][3+r] * base_orient.ad[r];
	cart_orient[j].add[i]   += Jbase[(j-1)*6+i+3][3+r] * base_orient.add[r] + 
	  dJbasedt[(j-1)*6+i+3][3+r] * base_orient.ad[r];
      }

    }

    /* compute quaternion derivatives */
    quatDerivatives(&(cart_orient[j]));
    quatDerivatives(&(cart_des_orient[j]));
    for (r=1; r<=N_QUAT; ++r)
      cart_des_orient[j].qdd[r] = 0.0; // we don't have dJdes_dt so far

  }

  /* reset first time flag */
  firsttime = FALSE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  drs & disable_ros_servo
\date  July 2010
   
\remarks 

disables the ros servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void 
drs(void)
{
  disable_ros_servo();
}

void 
disable_ros_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("ROS Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "ROS Servo is not on!\n" );
  
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
static int
checkForMessages(void)
{
  int i,j;
  char name[20];

  // check whether a message is available
  if (semTake(sm_ros_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_ros_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++ros_servo_errors;
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_ros_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_ros_message->name[i]);

    // act according to the message name

    // ---------------------------------------------------------------------------
    if (strcmp(name,"status") == 0) { 

      status();

    }


  }

  // give back semaphore
  sm_ros_message->n_msgs = 0;
  sm_ros_message->n_bytes_used = 0;
  semGive(sm_ros_message_sem);


  return TRUE;
}

