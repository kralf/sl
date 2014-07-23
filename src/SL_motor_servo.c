/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_motor_servo.c

  \author  Stefan Schaal
  \date    1997

  ==============================================================================
  \remarks

  low level motor control for the robots and simulations

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_motor_servo.h"
#include "SL_collect_data.h"
#include "SL_controller.h"
#include "SL_shared_memory.h"
#include "SL_sensor_proc.h"
#include "SL_common.h"
#include "SL_filters.h"
#include "SL_oscilloscope.h"
#include "SL_man.h"
#include "utility.h"



#define COUNT_NO_RECEIVE_MAX 30
#define TIME_OUT_NS          1000  //!< time out in nano seconds
#define TRANSIENT_TICKS 10         //!< servo ticks until filter convergence

/* variables for the motor servo */
int           motor_servo_errors;
long          motor_servo_calls=0;
int           motor_servo_initialized = FALSE;
int           motor_servo_rate;
double        servo_time=0;
double        motor_servo_time=0;
double        last_motor_servo_time=0;
int           servo_enabled;
int           count_no_receive = 0;
double        count_no_receive_total = 0;
int           count_no_broadcast = 0;
int           no_receive_flag = TRUE;
int          *zero_ufb_P_flag;
int          *zero_ufb_D_flag;
int           real_time_clock_flag  = FALSE;


/* local variables */
static int       *joint_invalid;

/* global functions */
int  run_motor_servo(void);

/* local functions */
static int  receive_commands(void);
static int  broadcast_sensors(void);
static void triggerSynchronization(void);
static int  checkForMessages(void);
static void sim_stop(void);
static void reset(void);
static void where_gains(void);


/*!*****************************************************************************
 *******************************************************************************
  \note  init_motor_servo
  \date  Dec. 1997

  \remarks 

  initializes servo specific things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
void
init_motor_servo(void)
{
  int j, i;
  STATUS error;
  char   string[100];
  extern int    init_user_commands(void);
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    joint_invalid               = my_ivector(1,n_dofs);
    zero_ufb_P_flag             = my_ivector(1,n_dofs);
    zero_ufb_D_flag             = my_ivector(1,n_dofs);
  }
  
  if (motor_servo_initialized) {
    printf("Motor Servo is already initialized\n");
    return;
  }

  /* the default servo rate  and the name of the servo*/
  sprintf(servo_name,"motor");
  motor_servo_rate=servo_base_rate;
  
  /* vxworks specific initialization */
  
#ifdef VX
  printf("Init vxworks ...");
  if (!init_vxworks()) 
    return;
  printf("done\n");
#endif
  
  /* initialize the controller */
  printf("Init controller ...");
  if (!init_controller())
    return;
  printf("done\n");

  /* init data collection */
  initCollectData(motor_servo_rate);

  /* initialize the sensory processing */
  printf("Init sensor processing ...");
  if (!init_sensor_processing())
    return;
  printf("done\n");

  /* initialize shared memories and shared semaphores */
  printf("Init shard memory ...");
  if (!init_shared_memory())
    return;
  printf("done\n");

   /* initialize system commands and user commands */
  printf("Init commands ...");
  if (!init_commands())
    return;
    
  if (!init_user_commands())
    return;

  printf("done\n");

  /* add variables to data collection */

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
    sprintf(string,"%s_load",joint_names[i]);
    addVarToCollect((char *)&(joint_state[i].load),string,"Nm", DOUBLE,FALSE);
    sprintf(string,"%s_des_th",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].th),string,"rad",DOUBLE,FALSE);
    sprintf(string,"%s_des_thd",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].thd),
		    string,"rad/s",DOUBLE,FALSE);
    sprintf(string,"%s_uff",joint_names[i]);
    addVarToCollect((char *)&(joint_des_state[i].uff),string,"Nm",DOUBLE,FALSE);
  }

  addVarToCollect((char *)&(motor_servo_errors),"MSErrors","-",INT,FALSE);
  addVarToCollect((char *)&(count_no_receive),"MSNoReceive","-",INT,FALSE);

  updateDataCollectScript();

  printf("done\n");

  // add to man pages 
  addToMan("dms","disables the motor servo",dms);
  addToMan("status","displays status information about servo",status);
  addToMan("stop","kills the robot control",sim_stop);
  addToMan("where_gains","Print gains of the joints",where_gains);


  /* initialize user specific things */
  if (!init_user_motor())
    return;

  /* if all worked out, we mark the servo as ready to go */
  motor_servo_initialized = TRUE;

  /* set oscilloscope to start value */
  initOsc();
  setOsc(d2a_cm,0.0);

  scd();

}

/*!*****************************************************************************
 *******************************************************************************
\note  run_motor_servo
\date  Feb 1999
\remarks 

        set of subroutines executed in the motor servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
run_motor_servo(void)

{
  int    j,i;
  int    dticks;

  setOsc(d2a_cm,100.0);
  
  /*********************************************************************
   * timing
   */

  ++motor_servo_calls;
  motor_servo_time += 1./(double)motor_servo_rate;
  servo_time = motor_servo_time;

  // check for missed calls to the servo
  dticks = round((motor_servo_time - last_motor_servo_time)*(double)motor_servo_rate);
  if (dticks != 1 && motor_servo_calls > 2) // need transient ticks to sync servos
    motor_servo_errors += abs(dticks-1);
  
  /*********************************************************************
   * check for messages
   */

  checkForMessages();

  /*********************************************************************
   * read data from the robot: this returns all sensors in SI units
   */

  if (!read_sensors()) {
    stop("Problem when reading sensors"); 
    return FALSE;
    ;
  }

  setOsc(d2a_cm,90.0);
  
  /*********************************************************************
   * filtering and differentiation of the data
   */

  if (!process_sensors()) {
    stop("Problem when processing sensors");
    return FALSE;
  }

  setOsc(d2a_cm,80.0);
  
  /*************************************************************************
   * provide sensor readings in shared memory
   */

  if (!broadcast_sensors()) {
    stop("Problem when broadcasting sensor readings");
    return FALSE;
  }
  
  
  setOsc(d2a_cm,70.0);
  
  /*************************************************************************
   *  trigger synchronization processes
   */
  
  if (motor_servo_calls > TRANSIENT_TICKS) { // needed for filters to converge
    triggerSynchronization();
  }   
  
  setOsc(d2a_cm,50.0);
  
  /**********************************************************************
   * get desired values and feedforward commands
   */

  if (motor_servo_calls > TRANSIENT_TICKS) {
    if (!receive_commands()) {
      stop("Problem when receiving commands");
      return FALSE;
    }
  } else {
    for (i=1; i<=n_dofs; ++i) {
      joint_des_state[i].th  = joint_state[i].th;
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].uff = 0.0;
    }
  }

  setOsc(d2a_cm,40.0);
  
  /**********************************************************************
   * the feedforward/feedback controller
   */
  
  if (!generate_total_commands()) {
    stop("Problem when generating total command");
    return FALSE;
  }
  
  setOsc(d2a_cm,25.0);
  
  /**********************************************************************
   *  send commands to the robot
   */

  if (!send_commands()) {
    stop("Problem when sending command");
    return FALSE;
  }
  
  setOsc(d2a_cm,10.0);
  
  /*************************************************************************
   * collect data
   */

  writeToBuffer();
  sendOscilloscopeData();

  setOsc(d2a_cm,0.0);
  
  /*************************************************************************
   * end of functions
   */

  last_motor_servo_time = motor_servo_time;

  return TRUE;

}
/*!*****************************************************************************
 *******************************************************************************
\note  receive_commands
\date  April 1999
   
\remarks 

        checks for new command input, copies those to local a structure,
	and counts the frequency of incoming commands
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_commands(void)
{
  
  int i;
  double rate = 0.999;
  int  wait_flag;

  if (real_time_clock_flag ) {
    wait_flag = NO_WAIT;
  } else {
    if (count_no_receive < task_servo_ratio-1) // force tight syncornization
      wait_flag = NO_WAIT;
    else
      wait_flag = WAIT_FOREVER;
  }


  if (semTake(sm_sjoint_des_state_ready_sem,wait_flag) == ERROR) {

    if (wait_flag == WAIT_FOREVER) // an error in WAIT_FOREVER must be terminated
      exit(-1);

    if (++count_no_receive < task_servo_ratio) {
      // if the motor servo run higher rate than the task servo, the desired
      // joint states gets integrated to be more accurate
      for (i=1; i<=n_dofs; ++i) {
	joint_des_state[i].th  += joint_des_state[i].thd  * 1./(double)motor_servo_rate;
	joint_des_state[i].thd += joint_des_state[i].thdd * 1./(double)motor_servo_rate;
      }
    }
    count_no_receive_total += 1./(double)task_servo_ratio;

  } else {

    if (semTake(sm_sjoint_des_state_sem,NO_WAIT) == ERROR) {
      
      ++count_no_receive;
      count_no_receive_total += 1.0;

    } else {

      count_no_receive = 0;
      count_no_receive_total = (int) count_no_receive_total;

      memcpy((void *)(&sm_sjoint_des_state_data[1]),
	     (const void*)(&sm_sjoint_des_state->sjoint_des_state[1]),
	     sizeof(SL_fSDJstate)*n_dofs);

      for (i=1; i<=n_dofs; ++i) {
	// check the joint status and only copy data if TRUE
	sm_sjoint_des_state->sjoint_des_state[i].status = FALSE;
	if (sm_sjoint_des_state_data[i].status) {
	  cSL_SDJstate(&(joint_des_state[i])-1,
		       &(sm_sjoint_des_state_data[i])-1,1,FLOAT2DOUBLE);
	}
	// check whether the user wants to overwrite local feedback servo
	if (sm_sjoint_des_state_data[i].zero_ufb_P)
	  zero_ufb_P_flag[i] = task_servo_ratio;
	if (sm_sjoint_des_state_data[i].zero_ufb_D)
	  zero_ufb_D_flag[i] = task_servo_ratio;
      }

      semGive(sm_sjoint_des_state_sem);

    }

  }

  /* check status of joints */
  for (i=1; i<=n_dofs; ++i) {

    /* if we do not receive enough input, we go into a default mode */
    if (!sm_sjoint_des_state_data[i].status) {
      if (++joint_invalid[i] > COUNT_NO_RECEIVE_MAX) {
	if (fabs(joint_des_state[i].th-joint_default_state[i].th) > 0.001) 
	  joint_des_state[i].th = joint_des_state[i].th * rate + 
	    (1.-rate) * joint_default_state[i].th;
	joint_des_state[i].thd = 0;
	joint_des_state[i].uff = 0;
      }
    } else {
      joint_invalid[i] = 0;
      sm_sjoint_des_state_data[i].status = FALSE;
    }

  }

  /* if we receive not information at all show it on the oscilloscope */

  if (count_no_receive > COUNT_NO_RECEIVE_MAX) {

    no_receive_flag = TRUE;

    setOsc(d2a_cm,10.0);

    /* after 0.5 hours, we automatically shut down the pump */

    if ((double)count_no_receive/(double)motor_servo_rate > 30.0*60.0) {
#ifdef VX
      user_kill();
#else
      exit(-1);
#endif
    }

  } else {

    no_receive_flag = FALSE;

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  broadcast_sensors
\date  April 1999
   
\remarks 

        just copies the sensor data to the share memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
broadcast_sensors(void)
{
  
  int i,j;

  /* only write at the task servo rate to reduce traffic */
  if (motor_servo_calls%task_servo_ratio!=0)
    return TRUE;

  if (semTake(sm_joint_state_sem,NO_WAIT) == ERROR) {
    
    ++count_no_broadcast;
    
  } else {
    
    cSL_Jstate(joint_state,sm_joint_state_data,n_dofs,DOUBLE2FLOAT);

    memcpy((void *)(&sm_joint_state->joint_state[1]),
	   (const void*)(&sm_joint_state_data[1]),
	   sizeof(SL_fJstate)*n_dofs);

    sm_joint_state->ts = motor_servo_time;

    semGive(sm_joint_state_sem);

  }
  
  // the misc sensors
  if (n_misc_sensors > 0) {

    if (semTake(sm_misc_sensor_sem,NO_WAIT) == ERROR) {
      
      ++count_no_broadcast;
      
    } else {
      
      for (i=1; i<=n_misc_sensors; ++i)
	sm_misc_sensor_data[i] = (float) misc_sensor[i];
      
      memcpy((void *)(&sm_misc_sensor->value[1]),
	     (const void*)(&sm_misc_sensor_data[1]),
	     sizeof(float)*n_misc_sensors);

      sm_misc_sensor->ts = motor_servo_time;
      
      semGive(sm_misc_sensor_sem);
      
    }

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  triggerSynchronization
\date  April 1999
   
\remarks 

        flushes the synchronization semaphores -- not that under SL
        Unix we use exact synchronization between task servo and motor
        servo, such that semGive is used. (Tight sync uses
        WAIT_FOREVER on the synchronization semaphores, such that one
        must not miss a sychronization signal (otherwise all processes
        would hang)
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static void
triggerSynchronization(void)
{
  
  int i;
  int iaux;
  static double last_openGL_time = 0.0;
  double current_time;

  // note: synchronizing on the remainder=1 allows starting all servos
  // immediately in the first run of the motor servo

  if (task_servo_ratio > 1) {
    if (motor_servo_calls%task_servo_ratio==1) {
#ifdef VX
      semFlush(sm_task_servo_sem);
#else
      semGive(sm_task_servo_sem);
#endif
    }
  } else {
#ifdef VX
    semFlush(sm_task_servo_sem);
#else
    semGive(sm_task_servo_sem);
#endif
  }


  iaux = (int)(((double)motor_servo_rate)/60.0+0.5);
  if (motor_servo_calls%iaux==1)
    semGive(sm_vision_servo_sem);

#ifdef VX
  if (motor_servo_calls%iaux==1)
    semFlush(sm_openGL_servo_sem);
#else
  {
#ifdef __XENO__
    RTIME t = rt_timer_read();
    current_time = (double)t / 1.e9;
#else
    struct timeval t;
    
    gettimeofday(&t,NULL);
    current_time = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;
#endif


    if (current_time-last_openGL_time >= 1./R60HZ) {
      semFlush(sm_openGL_servo_sem);
      last_openGL_time = current_time; 
    }
 
  }
#endif

  
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
  int i,j,k;
  char name[20];

  // check whether a message is available
  if (semTake(sm_motor_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;

  // receive the message
  if (semTake(sm_motor_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn't take motor message semaphore\n");
    return FALSE;
  }

  for (k=1; k<=sm_motor_message->n_msgs; ++k) {

    // get the name of this message
    strcpy(name,sm_motor_message->name[k]);

    // act according to the message name
    if (strcmp(name,"changePIDGains") == 0) {
      float buf[n_dofs*3+1];
      extern double *controller_gain_th;
      extern double *controller_gain_thd;
      extern double *controller_gain_int;
      
      memcpy(&(buf[1]),sm_motor_message->buf+sm_motor_message->moff[k],sizeof(float)*(3*n_dofs));

      for (i=1; i<=n_dofs; ++i) {
	controller_gain_th[i]  = (double) buf[i];
	controller_gain_thd[i] = (double) buf[i+n_dofs];
	controller_gain_int[i] = (double) buf[i+2*n_dofs];
      }
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"where_gains") == 0) { 
      where_gains();
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"scdMotor") == 0) { 
      scd();
      
    // ---------------------------------------------------------------------------
    } else if (strcmp(name,"status") == 0) { 
      extern void status(void);
      status();

    }

    // see whether the user programmed a message interception
    userCheckForMessage(name,k);

  }

  // give back semaphore
  sm_motor_message->n_msgs = 0;
  sm_motor_message->n_bytes_used = 0;
  semGive(sm_motor_message_sem);


  return TRUE;
}

/*!*****************************************************************************
*******************************************************************************
\note  dms & disable_motor_servo
\date  December 1997
   
\remarks 

disables the motor servo

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     param   :

******************************************************************************/
void 
dms(void)
{
  disable_motor_servo();
}

void 
disable_motor_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Motor Servo Terminated\n");

    exit(-1);

  } else
    fprintf( stderr, "motor servo is not on!\n" );
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
  printf("            Controller Kind        = %d\n",controller_kind);
  printf("            Time                   = %f\n",servo_time);
  printf("            Servo Calls            = %ld\n",motor_servo_calls);
  printf("            Servo Rate             = %d\n",motor_servo_rate);
  printf("            Servo Errors           = %d\n",motor_servo_errors);
  printf("            Count No Receive       = %d\n",(int)count_no_receive_total);
  printf("            Count No Broadcast     = %d\n",count_no_broadcast);
  printf("            Servo Initialize       = %d\n",motor_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("            Power Status           = %d\n",power_on);
  printf("            Real Time Clock        = %d\n",real_time_clock_flag);
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
\note  stop
\date  August 7, 1992 
   
\remarks 

send pump into low pressure and terminate loops

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
  if (real_robot_flag)
    dms();
  else
    reset();
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  reset
 \date  Nov. 2005
 
 \remarks 
 
 sends message to task_servo to reset
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
******************************************************************************/
static void 
reset(void) 
{
  int i,j;
  unsigned char buf[1];

  if (real_robot_flag)
    return;

  sendMessageTaskServo("reset",(void *)buf,0);
  setDefaultPosture();

}


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
    printf("WARNING: motor servo is not running!!\n");
  }

  extern double        servo_time;
	extern double* controller_gain_th;
	extern double* controller_gain_thd;
	extern double* controller_gain_int;
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
