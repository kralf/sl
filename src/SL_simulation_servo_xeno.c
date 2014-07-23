/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_simulation_servo_xeno.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  main program for running a numerical simulation of a robot, i.e.,
  this is where the equations of motion are integrated. This is the xenomai
  version.

  ============================================================================*/

/* system includes */
#include "stdio.h"
#include "math.h"
#include "string.h"

// xenomai includes
#include "SL_xeno_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_integrate.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_vx_wrappers.h"
#include "SL_simulation_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_man.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_man.h"

#define TIME_OUT_NS  1000000000

// global variables
int     delay_ns = FALSE;

// local variables
static int     pause_flag = FALSE;
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 90;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;

// global functions 

// local functions
static  void simulation_servo(void *dummy);

// external functions

/*!*****************************************************************************
 *******************************************************************************
\note  main
\date  Feb 1999
\remarks 

initializes everything and starts the servo loop

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings

 ******************************************************************************/
int 
main(int argc, char**argv)
{
  int  i, j;
  int  rc;
  char name[100];

  // signal handlers
  installSignalHandlers();

  // parse command line options
  parseOptions(argc, argv);

  // initialize xenomai specific variables and real-time environment
  initXeno("sim");

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // initalize the servo
  if (!init_simulation_servo())
    return FALSE;

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
                       &servo_stack_size,&cpuID,&delay_ns);

  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo_%d",servo_name, parent_process_id);

    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
                          T_FPU | T_JOINABLE | T_CPU(cpuID),simulation_servo,NULL))) {
      printf("rt_task_spawn returned %d\n",rc);
    }

    // spawn command line interface thread
    spawnCommandLineThread(NULL);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // wait for the task to finish
    rt_task_join(&servo_ptr);

  } else {

    // spawn command line interface thread
    spawnCommandLineThread(NULL);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // run this servo
    simulation_servo(NULL);

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  simulation_servo
 \date  Oct 2009
 \remarks 

 This program is clocked by the motor servo and uses a shared
 memory semaphore for synchronization

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]  dummy: dummary argument

 ******************************************************************************/
static void
simulation_servo(void *dummy) 

{
  int rc;

  //forces the mode switch
  rt_printf("entering simulation servo\n");

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL))) 
    printf("rt_task_set_mode returned %d\n",rc);

  // boardcast the current state such that the motor servo can generate a command
  send_sim_state();
  send_misc_sensors();
  send_contacts();
  semGive(sm_motor_servo_sem);  

  // run the servo loop
  while (servo_enabled) {

    // force delay ticks if user wishes
    if (delay_ns > 0)
      taskDelay(ns2ticks(delay_ns));

    // check for PAUSE
    if (semTake(sm_pause_sem,NO_WAIT) != ERROR) {
      if (pause_flag)
        pause_flag = FALSE;
      else
        pause_flag = TRUE;
    }

    if (pause_flag) {
      taskDelay(ns2ticks(100000000));
      continue;
    }

    // wait to take semaphore 
    if (semTake(sm_simulation_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated\n");
      return;
    }

    // lock out the keyboard interaction
    sl_rt_mutex_lock(&mutex1);

    // run the simulation servo routines
    if (!run_simulation_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock(&mutex1);


  }  /* end servo while loop */

}

