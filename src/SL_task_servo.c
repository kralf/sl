/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo.c

  \author  Stefan Schaal
  \date    1997

  ==============================================================================
  \remarks

  manages the control loop to run different tasks 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_shared_memory.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_filters.h"
#include "SL_oscilloscope.h"
#include "SL_objects.h"

#define TIME_OUT_NS  1000000000

/* variables for the task servo */
int    task_servo_errors;
long   task_servo_calls=0;
int    task_servo_initialized = FALSE;
double last_task_servo_time=0;
double task_servo_time=0;
double servo_time=0;
int    servo_enabled;
int    task_servo_rate;
int    frame_counter = 0;
int    exit_on_stop = FALSE;

/* local variables */

/* global functions */
int  step(int jid, int iamp);

/* local functions */
static int  send_commands(void);
static int  send_cartesian(void);
static int  receive_sensors(void);
static int  receive_blobs(void);
static void compute_kinematics(void);
static void sim_stop(void);
static void sim_step(void);
static int  checkForMessages(void);
static void freezeBaseToggle(void);
static void statusAll(void);
static int  send_ros_state(void);
static void read_link_parms(void);


/*!*****************************************************************************
 *******************************************************************************
  \note  init_task_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
void
init_task_servo(void)
{
  int j, i;
  char   string[100];
  FILE  *in;
  extern void add_goto_task(void);
  extern void add_sine_task(void);
  extern void add_traj_task(void);
  extern void add_goto_cart_task(void);
  extern int  init_user_commands(void);

  if (task_servo_initialized) {
    printf("Task Servo is already initialized\n");
    return;
  }

#ifdef VX
  /* vxworks specific initialization */
  if (!init_vxworks()) 
    return;
#endif

  /* the servo name */
  sprintf(servo_name,"task");

  /* initialize the tasks */
  initTasks();
  add_goto_task();
  add_sine_task();
  add_traj_task();
  add_goto_cart_task();

  /* initialize shared memories and shared semaphores */
  if (!init_shared_memory())
    return;

   /* initialize system commands and user commands */
  if (!init_commands())
    return;
    
  if (!init_user_commands())
    return;

  /* inverse dynamics */
  if (!init_dynamics()) 
    return;
  
  /* initialize kinematicss */
  init_kinematics();
  
  /* get the max, min, and offsets of the position sensors */
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    return;

  /* read the control gains and max controls  */
  if (!read_gains(config_files[GAINS],NULL,NULL,NULL))
    return;

  /* filtering */
  if (!init_filters())
    return;

  /* object handling */
  if (!initObjects())
    return;
  
  /* add variables to data collection */
  task_servo_rate=servo_base_rate/task_servo_ratio;
  initCollectData(task_servo_rate);

  for (i=1; i<=n_dofs; ++i) {
    printf("%d...",i);
    fflush(stdout);
    sprintf(string,"%s_th",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].th),string,"rad", DOUBLE,FALSE);
    sprintf(string,"%s_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thd),string,"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].thdd),string,"rad/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_u",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].u),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_ufb",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].ufb),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].load),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_des_th",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].th),string,"rad",DOUBLE,FALSE);
    sprintf(string,"%s_des_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thd),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_thdd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thdd),string,"rad/s^2",DOUBLE,FALSE);
    sprintf(string,"%s_uff",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].uff),string,"Nm",DOUBLE,FALSE);
  }

  for (i=1; i<=n_endeffs; ++i) {

    sprintf(string,"%s_x",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_y",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_z",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_X_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Y_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].ad[_Z_]),string,"rad/s",DOUBLE,FALSE);

    sprintf(string,"%s_add",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_X_]),string,"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_bdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Y_]),string,"rad/s2",DOUBLE,FALSE);
    sprintf(string,"%s_gdd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].add[_Z_]),string,"rad/s2",DOUBLE,FALSE);

    sprintf(string,"%s_des_x",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_y",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_z",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_yd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_xdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_ydd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_des_zdd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_des_ad",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_X_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_bd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Y_]),string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_des_gd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].ad[_Z_]),string,"rad/",DOUBLE,FALSE);

    sprintf(string,"%s_tx",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_ty",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tz",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].x[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_txd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tyd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tzd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_txdd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_X_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tydd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_Y_]),string,"m",DOUBLE,FALSE);
    sprintf(string,"%s_tzdd",cart_names[i]);
    addVarToCollect((char *)&(cart_target_state[i].xdd[_Z_]),string,"m",DOUBLE,FALSE);

    sprintf(string,"%s_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].q[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_orient[i].qdd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].q[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3d",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_des_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q0_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q1_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q2_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_des_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cart_des_orient[i].qdd[_Q3_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_cons_x",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_X_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_y",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_Y_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_z",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[_Z_]),string,"-",INT,FALSE);

    sprintf(string,"%s_cons_a",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_A_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_b",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_B_]),string,"-",INT,FALSE);
    sprintf(string,"%s_cons_g",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].c[N_CART+_G_]),string,"-",INT,FALSE);

    sprintf(string,"%s_cfx",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_X_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_cfy",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_Y_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_cfz",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].cf[_Z_]),string,"-",DOUBLE,FALSE);

    sprintf(string,"%s_cta",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_A_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_ctb",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_B_]),string,"-",DOUBLE,FALSE);
    sprintf(string,"%s_ctg",cart_names[i]);
    addVarToCollect((char *)&(endeff[i].ct[_G_]),string,"-",DOUBLE,FALSE);


  }

  for (i=1; i<=max_blobs; ++i) {

    sprintf(string,"%s_stat",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_x",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_y",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_z",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Z_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_xd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_X_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_yd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Y_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_zd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Z_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_xdd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_X_]),string,"m/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_ydd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_Y_]),string,"m/s^2", DOUBLE,FALSE);
    sprintf(string,"%s_zdd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xdd[_Z_]),string,"m/s^2", DOUBLE,FALSE);

    sprintf(string,"%s_rstat1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_rstat2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_Y_]),string,"m", DOUBLE,FALSE);
  }

  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_sensor[i]),string,"-",DOUBLE,FALSE);
  }


  /* the center of gravity */
  addVarToCollect((char *)&(cog.x[_X_]),"cog_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.x[_Y_]),"cog_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.x[_Z_]),"cog_z","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog.xd[_X_]),"cog_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.xd[_Y_]),"cog_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog.xd[_Z_]),"cog_zd","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog_des.x[_X_]),"cog_des_x","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.x[_Y_]),"cog_des_y","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.x[_Z_]),"cog_des_z","m",DOUBLE,FALSE);

  addVarToCollect((char *)&(cog_des.xd[_X_]),"cog_des_xd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.xd[_Y_]),"cog_des_yd","m",DOUBLE,FALSE);
  addVarToCollect((char *)&(cog_des.xd[_Z_]),"cog_des_zd","m",DOUBLE,FALSE);

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



  addVarToCollect((char *)&(frame_counter),"frame_counter","-",INT,FALSE);

  printf("done\n");

  updateDataCollectScript();

  /* add to man pages */
  addToMan("status","displays information about the servo",status);
  addToMan("freeze","freeze the robot in current posture",freeze);
  addToMan("f","freeze the robot in current posture",f);
  addToMan("step","step commands to a joint",sim_step);
  addToMan("stop","kills the robot control",sim_stop);
  addToMan("dts","disables the task servo",dts);
  addToMan("statusAll","prints status on all processes",statusAll);
  addToMan("read_link_params","re-reads the link parameters",read_link_parms);
  if (!real_robot_flag) {
    addToMan("reset","reset state of simulation",reset);
    addToMan("setG","set gravity constant",setG);
    addToMan("freezeBase","freeze the base at orgin",freezeBaseToggle);
  }


  /* if all worked out, we mark the servo as ready to go */
  task_servo_initialized = TRUE;

  /* set oscilloscope to start value */
  initOsc();
  setOsc(d2a_ct,0.0);

  scd();

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_task_servo
\date  Feb 1999
\remarks 

        This program executes the sequence of task servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
run_task_servo(void)

{
  int    j,i;
  double dt;
  int    dticks;
  static int firsttime = TRUE;

  setOsc(d2a_ct,0.0);
  
  /*********************************************************************
   * adjust servo time
   */

  ++task_servo_calls;
  task_servo_time += 1./(double)task_servo_rate;
  servo_time = task_servo_time;

  // check for missed calls to the servo
  dticks = round((task_servo_time - last_task_servo_time)*(double)task_servo_rate);
  if (dticks != 1 && task_servo_calls > 2) // need transient ticks to sync servos
    task_servo_errors += abs(dticks-1);


  /*********************************************************************
   * start up chores
   */

  /* zero any external simulated forces */
  bzero((void *)uext_sim,sizeof(SL_uext)*(n_dofs+1));

  /* check for messages */
  checkForMessages();

  /*********************************************************************
   * receive sensory data
   */

  if (!receive_sensors()) {
    stop("Problem when receiving sensor data");
    return FALSE;
  }

  setOsc(d2a_ct,10.0);

  if (firsttime) { // initialize desired at first servo tick
    for (i=1; i<=n_dofs; ++i) {
      joint_des_state[i].th  = joint_state[i].th;
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].uff = 0.0;
    }
    firsttime = FALSE;
  }

  /*********************************************************************
   * compute useful kinematic variables
   */
  
  compute_kinematics();
  
  setOsc(d2a_ct,20.0);

  /**********************************************************************
   * send out the kinematic variables
   */
  
  if (!send_cartesian()) {
    stop("Problem when sending cartesian state");
    return FALSE;
  }
  
  setOsc(d2a_ct,30.0);
  
  
  /**********************************************************************
   * receive vision blobs
   */
  
  if (!receive_blobs()) {
    stop("Problem when receiving vision blobs");
    return FALSE;
  }
  
  setOsc(d2a_ct,50.0);
  
  /*********************************************************************
   * call the tasks
   */
  
#ifndef VX
  /* run general simulation specific task servo computations */
  if (!run_user_task()) {
    stop("Problem in run_user_task");
    return FALSE;
  }
#endif

  /* run user tasks */
  runTask();
  
  setOsc(d2a_ct,70.0);
  
  /**********************************************************************
   * send out the new commands
   */

  if (!send_commands()) {
    stop("Problem when sending commands");
    return FALSE;
  }

  setOsc(d2a_ct,80.0);
  
  
  /**********************************************************************
   * send out ros state
   */

  send_ros_state();

  setOsc(d2a_ct,90.0);
  
  /*************************************************************************
   * collect data
   */

  writeToBuffer();
  sendOscilloscopeData();

  setOsc(d2a_ct,100.0);
  
  /*************************************************************************
   * end of program sequence
   */

  last_task_servo_time = task_servo_time;

  return TRUE;
  
}
 
/*!*****************************************************************************
 *******************************************************************************
\note  status
\date  August 7, 1992
   
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
  printf("            Time                   = %f\n",task_servo_time);
  printf("            Servo Calls            = %ld\n",task_servo_calls);
  printf("            Servo Rate             = %d\n",task_servo_rate);
  printf("            Servo Errors           = %d\n",task_servo_errors);
  printf("            Servo Initialize       = %d\n",task_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("            Task                   = %s\n",current_task_name);
  printf("            Vision Frame Counter   = %d\n",frame_counter);
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
\note  receive_sensors
\date  April 1999
   
\remarks 

        receives new sensory data 
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_sensors(void)
{
  
  int i;
  double ts;
  int dticks;

  // the joint state
  if (semTake(sm_joint_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {

    ++task_servo_errors;
    return FALSE;

  } 

  memcpy((void *)(&sm_joint_state_data[1]),(const void*)(&sm_joint_state->joint_state[1]),
	 sizeof(SL_fJstate)*n_dofs);

  cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,FLOAT2DOUBLE);

  // get time stamp and adjust servo time
  task_servo_time = servo_time = sm_joint_state->ts;
  
  semGive(sm_joint_state_sem);


  // the misc sensors

  if (n_misc_sensors > 0) {
    
    if (semTake(sm_misc_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
      
      ++task_servo_errors;
      return FALSE;
      
    } 
    
    memcpy((void *)(&sm_misc_sensor_data[1]),(const void*)(&sm_misc_sensor->value[1]),
	   sizeof(float)*n_misc_sensors);
    
    for (i=1; i<=n_misc_sensors; ++i)
      misc_sensor[i] = (double) sm_misc_sensor_data[i];

    semGive(sm_misc_sensor_sem);

  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_commands
\date  April 1999
   
\remarks 

        just copies the sensor data to the share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_commands(void)
{
  
  int i,j;
  

  if (semTake(sm_sjoint_des_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++task_servo_errors;
    return FALSE;
  }

  /* check for joint limits */

  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      if (joint_des_state[i].th > joint_range[i][MAX_THETA])
	joint_des_state[i].th = joint_range[i][MAX_THETA];
      if (joint_des_state[i].th < joint_range[i][MIN_THETA]) 
	joint_des_state[i].th = joint_range[i][MIN_THETA];
    }
  }

  cSL_SDJstate(joint_des_state,sm_sjoint_des_state_data,n_dofs,DOUBLE2FLOAT);
    
  for (i=1; i<=n_dofs; ++i) {
    if (whichDOFs[i]) {
      sm_sjoint_des_state_data[i].status = TRUE;
      sm_sjoint_des_state->sjoint_des_state[i] = sm_sjoint_des_state_data[i];

      // check if user wants to cancel motor_servo PD controller
      if (joint_des_state[i].th == joint_state[i].th)
	sm_sjoint_des_state_data[i].zero_ufb_P = TRUE;
      else
	sm_sjoint_des_state_data[i].zero_ufb_P = FALSE;

      if (joint_des_state[i].thd == joint_state[i].thd)
	sm_sjoint_des_state_data[i].zero_ufb_D = TRUE;
      else
	sm_sjoint_des_state_data[i].zero_ufb_D = FALSE;
	
    }
  }

  semGive(sm_sjoint_des_state_sem);
  semGive(sm_sjoint_des_state_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  stop
\date  August 7, 1992 
   
\remarks 

       stops ongoing processing on this servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
static void
sim_stop(void)
{
  stop("Simulation Triggered Stop");   /* for simulation environment only */
}
int
stop(char *msg)
{

  int i;

  beep(1);
  printf("%s\n",msg);
  fflush(stdout);
  dts();

#ifdef UNIX
  if (exit_on_stop)
    // this allows real robots to abort completely on stop()
    exit(-1);
  else {
    // the simulation just resets
    // extern void reset(void);
    // reset();
    ;
  }
#endif
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  freeze
\date  August 7, 1992 
   
\remarks 

       freezes an ongoing task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
f(void) {
  freeze();
}
void
freeze(void)
{
  int i;
  setTaskByName(NO_TASK);
  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].thd   = 0.0;
    joint_des_state[i].thdd = 0.0;
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  step
\date  April 1999
   
\remarks 

        simple steps to check out the gains
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     jid  : jointID
 \param[in]     amp  : amplitude

 ******************************************************************************/
static void
sim_step(void)  /* for simulation only */
{
  static int jid=1,iamp=5;

  get_int("Step which joint?",jid,&jid);
  get_int("Step amplitude as integer number (times 0.1 rad)",iamp,&iamp);
  step(jid,iamp);
}

int
step(int jid, int iamp)

{
  static int flag = 1;
  double amp;

  amp = (double)iamp/10.;

  if (jid < 1 || jid > n_dofs || strcmp(current_task_name,NO_TASK)!= 0 ||
      amp > 0.5 || amp < 0)
    return FALSE;

  if (flag) {
    printf("Step %s by amp = %f\n",joint_names[jid],amp);
    joint_des_state[jid].th += amp;
    flag = FALSE;
  } else {
    printf("Step %s by amp = %f\n",joint_names[jid],-amp);
    joint_des_state[jid].th -= amp;
    flag = TRUE;
  }

  scd();
  
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
    mat_mult_scalar(dJdt,(double)task_servo_rate,dJdt);
    mat_sub(Jbase,last_Jbase,dJbasedt);
    mat_mult_scalar(dJbasedt,(double)task_servo_rate,dJbasedt);
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
\note  send_cartesian
\date  April 1999
   
\remarks 

        just copies the cartesian data to share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_cartesian(void)
{
  
  int i;
  
  if (semTake(sm_cart_states_sem,NO_WAIT) == ERROR) {
    
    ;

  } else {

    cSL_Cstate(cart_state,sm_cart_states_data,n_endeffs,DOUBLE2FLOAT);

    memcpy((void *)(&sm_cart_states->state[1]),(const void*)(&sm_cart_states_data[1]),
	   sizeof(SL_fCstate)*n_endeffs);

    sm_cart_states->ts = task_servo_time;

    semGive(sm_cart_states_sem);

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_blobs
\date  April 1999
   
\remarks 

        receives vision blob information
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_blobs(void)
{
  
  static int count = 0;
  int i,j,k,rc;
  char string[40];
  
  if (semTake(sm_vision_blobs_sem,NO_WAIT) == ERROR)
    {
      ;
    }
  else 
    {
      rc = sm_vision_blobs->frame_counter;
      if (frame_counter != rc)
	{
	  frame_counter = rc;
	  for (i=1; i<=max_blobs; ++i)
	    {
	      if (sm_vision_blobs->blobs[i].status)
		{
		  sm_vision_blobs_data[i] = sm_vision_blobs->blobs[i];
		} 
	      else
		sm_vision_blobs_data[i].status = FALSE;
	      sm_vision_blobs_data_aux[i] = sm_vision_blobs_aux->blobs[i];
	    }
	  cSL_VisionBlob(blobs,sm_vision_blobs_data,max_blobs,FLOAT2DOUBLE);
	  cSL_VisionBlobaux(raw_blobs2D,sm_vision_blobs_data_aux,max_blobs,
			    FLOAT2DOUBLE);

	}

      strcpy(sm_vision_blobs->pp_name,current_vision_pp);
      semGive(sm_vision_blobs_sem);
    }
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_raw_blobs
\date  April 1999
   
\remarks 

        just copies the raw blobs to share memory -- this is used
	to overwrite the vision info and allows simulation of visual
	blobs
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int 
send_raw_blobs(void)
{
  
  int i;
  
  if (semTake(sm_raw_blobs_sem,NO_WAIT) == ERROR) {
    return FALSE;
  }

  cBlob3D(raw_blobs,sm_raw_blobs_data,max_blobs,DOUBLE2FLOAT);

  for (i=1; i<=max_blobs; ++i) {
    if (sm_raw_blobs_data[i].status)
      sm_raw_blobs->blobs[i] = sm_raw_blobs_data[i];
  }
  
  semGive(sm_raw_blobs_sem);
  semGive(sm_raw_blobs_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_raw_blobs2D
\date  April 1999
   
\remarks 

        just copies the raw blobs2D to share memory -- this is used
	to overwrite the vision info and allows simulation of visual
	blobs
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int 
send_raw_blobs2D(void)
{
  
  int i;
  
  if (semTake(sm_raw_blobs2D_sem,NO_WAIT) == ERROR) {
    return FALSE;
  }

  cBlob2D(raw_blobs2D,sm_raw_blobs2D_data,max_blobs,DOUBLE2FLOAT);

  for (i=1; i<=max_blobs*2; ++i) {
    if (sm_raw_blobs2D_data[i].status)
      sm_raw_blobs2D->blobs[i] = sm_raw_blobs2D_data[i];
  }
  
  semGive(sm_raw_blobs2D_sem);
  semGive(sm_raw_blobs_ready_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  changePIDGains
\date  Nov. 2005
   
\remarks 

        sends the request to change the PID gains to the relevant servos

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     pGain : the proportional gains
 \param[in]     dGain : the derivative gains
 \param[in]     iGain : the integral gains

 ******************************************************************************/
void 
changePIDGains(double *pGain, double *dGain, double *iGain) 
{
  int i,j;
  float buf[3*n_dofs+1];
  unsigned char cbuf[(3*n_dofs)*sizeof(float)];

  for (i=1; i<=n_dofs; ++i) {
    buf[i] = pGain[i];
    buf[i+n_dofs] = dGain[i];
    buf[i+2*n_dofs] = iGain[i];
  }
    
  memcpy(cbuf,(void *)&(buf[1]),(3*n_dofs)*sizeof(float));
    
  sendMessageSimulationServo("changePIDGains",(void *)cbuf,(3*n_dofs)*sizeof(float));
  sendMessageMotorServo("changePIDGains",(void *)cbuf,(3*n_dofs)*sizeof(float));

}


/*!*****************************************************************************
 *******************************************************************************
\note  switchCometDisplay
\date  May 2010
   
\remarks 

switches on/off the comet display for openGL

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     status  : TRUE/FALSE for turning comet on/off
 \param[in]     n_steps : if turned off, the history of the comet is set to n_steps

 ******************************************************************************/
void 
switchCometDisplay(int status, int n_steps)
{
  int i,j;
  int ibuf[3];
  unsigned char cbuf[2*sizeof(int)];

  ibuf[1] = status;
  ibuf[2] = n_steps;

  memcpy(cbuf,(void *)&(ibuf[1]),2*sizeof(float));
    
  sendMessageOpenGLServo("switchCometDisplay",(void *)cbuf,2*sizeof(int));

}

/*!*****************************************************************************
 *******************************************************************************
\note  scdMotor
\date  March 2011
   
\remarks 

starts collect data on the motor servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
void 
scdMotor(void)
{
  int i,j;

  sendMessageMotorServo("scdMotor",NULL,0);

}

/*!*****************************************************************************
 *******************************************************************************
\note  resetCometDisplay
\date  May 2010
   
\remarks 

resets the history of the comet display

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
void 
resetCometDisplay(void)
{
  int i,j;
  unsigned char cbuf[1];

  sendMessageOpenGLServo("resetCometDisplay",(void *)cbuf,0);

}

/*!*****************************************************************************
 *******************************************************************************
\note  resetCometDisplayVars
\date  May 2010
   
\remarks 

resets the variables for comet display

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
void 
resetCometDisplayVars(void)
{
  int i,j;
  unsigned char cbuf[1];

  sendMessageOpenGLServo("resetCometVars",(void *)cbuf,0);

}

/*!*****************************************************************************
 *******************************************************************************
\note  switchCometDisplayVars
\date  May 2010
   
\remarks 

Switches comet display variables on and off. Two kinds of variables are possible,
endeffector variables and links. This function communicates the ID number of
both one endeffector and one link, and the associated status for display. Pass
a zero ID if either the link or the endeffector are not be used.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]  endeffID    : endeffector ID to switch
 \param[in]  endeffStatus: TRUE/FALSE for displaying this endeffector
 \param[in]  linkID      : endeffector ID to switch
 \param[in]  linkStatus  : TRUE/FALSE for displaying this link


 ******************************************************************************/
void 
switchCometDisplayVars(int endeffID, int endeffStatus, int linkID, int linkStatus)
{
  int i,j;
  int ibuf[5];
  unsigned char cbuf[4*sizeof(int)];

  ibuf[1] = endeffID;
  ibuf[2] = endeffStatus;
  ibuf[3] = linkID;
  ibuf[4] = linkStatus;

  memcpy(cbuf,(void *)&(ibuf[1]),4*sizeof(float));
    
  sendMessageOpenGLServo("switchCometVars",(void *)cbuf,4*sizeof(int));

}


/*!*****************************************************************************
 *******************************************************************************
\note  dts & disable_task_servo
\date  April 1999
   
\remarks 

        disables the task servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void 
dts(void)
{
  disable_task_servo();
}

void 
disable_task_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Task Servo Terminated\n");

    exit(-1);
    
  } else
    fprintf( stderr, "task servo is not on!\n" );
  
}


/*!*****************************************************************************
 *******************************************************************************
\note  sendUserGraphics
\date  Nov. 2007
   
\remarks 

      sends out information for user graphics to shared memory

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name   : name of graphics
 \param[in]     buf    : byte buffer with information
 \param[in]     n_bytes: number of bytest in buffer

 ******************************************************************************/
int
sendUserGraphics(char *name, void *buf, int n_bytes)
{
  int i,j;
  
  // send the user graphics data
  if (semTake(sm_user_graphics_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take user_graphics semaphore\n");
    return FALSE;
  }

  // first, check whether there is an older graphics object with the same
  // name -- it will be overwritten, as only the latest graphics objects
  // matters for visualization
  for (i=1; i<=sm_user_graphics->n_entries; ++i) {
    if (strcmp(name,sm_user_graphics->name[i]) == 0) {

      // just overwrite the data
      memcpy(sm_user_graphics->buf+sm_user_graphics->moff[i],buf,n_bytes);

      // give semaphores
      semGive(sm_user_graphics_sem);
      semGive(sm_user_graphics_ready_sem);

      return TRUE;

    }
  }

  // make sure the pointer offset buffer is correct
  if (sm_user_graphics->n_entries == 0) {
    sm_user_graphics->moff[1] = 0;
    sm_user_graphics->n_bytes_used = 0;
  }

  // check whether there is space for the graphics object
  if (sm_user_graphics->n_entries >= MAX_N_MESSAGES) {
    printf("User graphics buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  if (MAX_BYTES_USER_GRAPHICS-sm_user_graphics->n_bytes_used < n_bytes) {
    printf("User Graphics memory buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  // update the logistics
  ++sm_user_graphics->n_entries;
  sm_user_graphics->n_bytes_used += n_bytes;
  
  // specify the name and info of this entry
  strcpy(sm_user_graphics->name[sm_user_graphics->n_entries],name);
  memcpy(sm_user_graphics->buf+sm_user_graphics->moff[sm_user_graphics->n_entries],buf,n_bytes);
  
  // prepare pointer buffer for next entry
  if (sm_user_graphics->n_entries < MAX_N_MESSAGES)
    sm_user_graphics->moff[sm_user_graphics->n_entries+1]=
      sm_user_graphics->moff[sm_user_graphics->n_entries]+n_bytes;

  // give semaphores
  semGive(sm_user_graphics_sem);
  semGive(sm_user_graphics_ready_sem);
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  reset
\date  Nov. 2005
   
\remarks 

        sends message to simulation_servo to reset

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void 
reset(void) 
{
  int i,j;
  float buf[N_CART+N_QUAT+1];
  unsigned char cbuf[(N_CART+N_QUAT)*sizeof(float)];


  if (real_robot_flag)
    return;

  j = 0;
  for (i=1; i<=N_CART; ++i)
    buf[++j] = freeze_base_pos[i];
    
  for (i=1; i<=N_QUAT; ++i)
    buf[++j] = freeze_base_quat[i];

  memcpy(cbuf,(void *)&(buf[1]),(N_CART+N_QUAT)*sizeof(float));
    
  sendMessageSimulationServo("reset",(void *)cbuf,(N_CART+N_QUAT)*sizeof(float));
  freeze();
  setDefaultPosture();

}


/*!*****************************************************************************
 *******************************************************************************
\note  followBase
\date  Nov. 2005
   
\remarks 

        sends message to openGL_servo to follow the base coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void 
followBase(void) 
{
  int i,j;
  unsigned char buf[1];

  sendMessageOpenGLServo("followBase",(void *)buf,0);
}


/*!*****************************************************************************
 *******************************************************************************
\note  toggleShowAxes
\date  Nov. 2005
   
\remarks 

sends message to openGL_servo to show coordinate axes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void 
toggleShowAxes(int status) 
{
  int buf[1+1];
  unsigned char cbuf[sizeof(int)];

  buf[1] = status;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(int));

  sendMessageOpenGLServo("toggleShowAxes",(void *)cbuf,sizeof(int));

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
  if (semTake(sm_task_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_task_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_task_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_task_message->name[i]);

    // act according to the message name
    if (strcmp(name,"reset") == 0) {
      reset();
      
    } else if (strcmp(name,"changeObjectPos") == 0) {      
      struct {
        char   obj_name[100];
        double pos[N_CART+1];
        double rot[N_CART+1];
      } data;
     
      memcpy(&data,sm_task_message->buf+sm_task_message->moff[i],sizeof(data));
      
      ObjectPtr ptr = getObjPtrByName(data.obj_name);
      if (ptr != NULL) { 
        for (i=1;i<=N_CART; ++i) {
          ptr->trans[i]=data.pos[i];
          ptr->rot[i]=data.rot[i];
        }
      }
    }

  }

  // give back semaphore
  sm_task_message->n_msgs = 0;
  sm_task_message->n_bytes_used = 0;
  semGive(sm_task_message_sem);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  sendUextSim
\date  Nov. 2005
   
\remarks 

        sends simulated external force to simulation servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void 
sendUextSim(void) 
{
  int i,j;
  int count = 0;
  float buf[2*n_dofs*N_CART+1];
  unsigned char cbuf[(2*n_dofs*N_CART)*sizeof(float)];

  for (i=1; i<=n_dofs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      buf[++count] = uext_sim[i].f[j];
      buf[++count] = uext_sim[i].t[j];
    }
  }

  memcpy(cbuf,(void *)&(buf[1]),(2*n_dofs*N_CART)*sizeof(float));
    
  sendMessageSimulationServo("setUextSim",(void *)cbuf,(2*n_dofs*N_CART)*sizeof(float));

}


/*!*****************************************************************************
 *******************************************************************************
\note  changeRealTime
\date  May 2008
   
\remarks 

        sends messages to simulation servo to turn on/off real-time
        simualtion

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     flag : TRUE/FALSE real time flag

 ******************************************************************************/
void 
changeRealTime(int flag) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = flag;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));
    
  sendMessageSimulationServo("changeRealTime",(void *)cbuf,sizeof(float));

}


/*!*****************************************************************************
 *******************************************************************************
\note  setG
\date  Nov. 2005
\remarks 

 set the gravity constant to an arbitrary value

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
void
setG(void)
{
  if (get_double("Enter new gravity constant",gravity,&gravity))
    setGravity(gravity);
}

/*!*****************************************************************************
 *******************************************************************************
\note  setGravity
\date  July 2009
   
\remarks 

        sends messages to simulation servo to adjust gravity constant

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     grav : gravity

 ******************************************************************************/
void 
setGravity(double grav) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = grav;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));

  sendMessageSimulationServo("setG",(void *)cbuf,sizeof(float));

}

/*!*****************************************************************************
 *******************************************************************************
\note  hideWindowByName
\date  March 2003
   
\remarks 

      allows toggling the hide status of a particular window

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name  : string containing the name of the window
 \param[in]     hide  : TRUE/FALSE = hide or not

 ******************************************************************************/
void
hideWindowByName(char *name, int hide)
{
  int i;
  unsigned char cbuf[101];

  if (strlen(name) > sizeof(cbuf)-1) {
    printf("Too long name in hideWindowByName\n");
    return;
  }

  sprintf((char *)cbuf,"%s",name);
  cbuf[sizeof(cbuf)-1] = hide;

  printf("%s %d\n",cbuf,cbuf[100]);

  sendMessageOpenGLServo("hideWindowByName",(void *)cbuf,sizeof(cbuf));

}

/*!*****************************************************************************
 *******************************************************************************
\note  freezeBase
\date  May 2008
   
\remarks 

        sets the freezeBase flag in the simulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     flag : TRUE/FALSE for freeze base

 ******************************************************************************/
static void 
freezeBaseToggle(void) 
{

  if (freeze_base == 0) {
    freeze_base = TRUE;
    printf("Base is fixed at origin\n");
  } else {
    freeze_base = FALSE;
    printf("Base is floating\n");
  }

  freezeBase(freeze_base);
  
}

void 
freezeBase(int flag) 
{
  int i,j;
  float buf[1+1];
  unsigned char cbuf[sizeof(float)];

  buf[1] = freeze_base = flag;

  memcpy(cbuf,(void *)&(buf[1]),sizeof(float));
    
  sendMessageSimulationServo("freezeBase",(void *)cbuf,sizeof(float));

}


/*!*****************************************************************************
 *******************************************************************************
\note  statusAll
\date  July 2010
   
\remarks 

triggers a status diplay on all processes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
statusAll(void)
{
  int i;
  unsigned char cbuf[1];

  sendMessageOpenGLServo("status",(void *)cbuf,0);
  sendMessageMotorServo("status",(void *)cbuf,0);
  sendMessageROSServo("status",(void *)cbuf,0);
  sendMessageSimulationServo("status",(void *)cbuf,0);
  sendMessageVisionServo("status",(void *)cbuf,0);
  status();

}

/*!*****************************************************************************
 *******************************************************************************
\note  send_ros_state
\date  July 2010
   
\remarks 

sends all relevant state variables to ROS process
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int 
send_ros_state(void)
{
  
  int i,j;
  SL_fJstate  *fJstate;
  SL_fDJstate *fDJstate;
  SL_fCstate  *fCstate;
  SL_fquat    *fquat;
  float       *misc;

  if (semTake(sm_ros_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++task_servo_errors;
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
  
  cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,DOUBLE2FLOAT);
  cSL_DJstate(joint_des_state, sm_joint_des_state_data, n_dofs,DOUBLE2FLOAT);
  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);
  cSL_quat((&base_orient)-1, sm_base_orient_data, 1, DOUBLE2FLOAT);
  for (i=1; i<=n_misc_sensors; ++i)
    sm_misc_sensor_data[i] = (float) misc_sensor[i];

  memcpy((void*)(&(fJstate[1])),(const void *)(&sm_joint_state_data[1]),sizeof(SL_fJstate)*n_dofs);
  memcpy((void*)(&(fDJstate[1])),(const void *)(&sm_joint_des_state_data[1]),sizeof(SL_fDJstate)*n_dofs);
  memcpy((void*)(&(fCstate[1])),(const void *)(&sm_base_state_data[1]),sizeof(SL_fCstate)*1);
  memcpy((void*)(&(fquat[1])),(const void *)(&sm_base_orient_data[1]),sizeof(SL_fquat)*1);
  if (n_misc_sensors > 0)
    memcpy((void*)(&(misc[1])),(const void *)(&sm_misc_sensor_data[1]),sizeof(float)*n_misc_sensors);
  
  sm_ros_state->ts = task_servo_time;

  semGive(sm_ros_state_sem);
  semGive(sm_ros_servo_sem);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_link_params
\date  May 2000
\remarks 

interactive function to read link parameters from file again

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
read_link_parms(void) 
{

  char string[100];

  // check whether we are idle
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Link parameters cannot only be read when no task is running!\n");
    return;
  }

  // read link parameters
  sprintf(string,"%s",config_files[LINKPARAMETERS]);
  if (!get_string("Parameter File Name",string,string))
    return;

  read_link_parameters(string);

}
