/*!=============================================================================
  ==============================================================================

  \ingroup SLtask

  \file    SL_task_servo_xeno.c

  \author  Stefan Schaal
  \date    Oct 2009

  ==============================================================================
  \remarks

  main program for the task servo adjusted for xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "SL_common.h"
#include "SL_task_servo.h"
#include "SL_tasks.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"

#define TIME_OUT_NS  1000000000

// global variabes
char initial_user_command[100]="";
int  delay_ns = FALSE;

// local variables
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 25;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;

// global functions 

// local functions
static void task_servo(void *dummy);


// external functions
extern void initUserTasks(void);

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
  initXeno("task");

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // initializes the servo
  init_task_servo();
  read_whichDOFs(config_files[WHICHDOFS],"task_servo");

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
                       &servo_stack_size,&cpuID,&delay_ns);

  // generic computations
  init_user_task();

  // reset task_servo variables
  servo_enabled           = 1;
  task_servo_calls        = 0;
  task_servo_time         = 0;
  last_task_servo_time    = 0;
  task_servo_errors       = 0;
  task_servo_rate         = servo_base_rate/(double) task_servo_ratio;

  setTaskByName(NO_TASK);
  changeCollectFreq(task_servo_rate);

  // the user tasks as defined in initUserTasks.c 
  initUserTasks();

  // reset the simulation
  if (!real_robot_flag)
    reset();

  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo_%d",servo_name, parent_process_id);

    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
                          T_FPU | T_JOINABLE | T_CPU(cpuID),task_servo,NULL))) {
      printf("rt_task_spawn returned %d\n",rc);
    }

    // spawn command line interface thread
    spawnCommandLineThread(initial_user_command);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // wait for the task to finish
    rt_task_join(&servo_ptr);

  } else {

    // spawn command line interface thread
    spawnCommandLineThread(initial_user_command);

    // signal that this process is initialized
    semGive(sm_init_process_ready_sem);

    // run this servo
    task_servo(NULL);

  }


  printf("Task Servo Error Count = %d\n",task_servo_errors);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  task_servo
\date  Oct 2009
\remarks 

This program is clocked by the motor servo and uses a shared
memory semaphore for synchronization

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  dummy: dummary argument

 ******************************************************************************/
static void
task_servo(void *dummy) 

{
  int rc;

  //forces the mode switch
  rt_printf("entering task servo\n");

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL)))
    printf("rt_task_set_mode returned %d\n",rc);

  // run the servo loop
  while (servo_enabled) {

    // force delay ticks if user wishes
    if (delay_ns > 0)
      taskDelay(ns2ticks(delay_ns));

    // wait to take semaphore 
    if (semTake(sm_task_servo_sem,WAIT_FOREVER) == ERROR)
      stop("semTake Time Out -- Servo Terminated");

    // lock out the keyboard interaction
    sl_rt_mutex_lock(&mutex1);

    // run the task servo routines
    if (!run_task_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock(&mutex1);



  }  /* end servo while loop */

}


