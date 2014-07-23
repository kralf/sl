/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo_unix.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  main program for the task servo

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_task_servo.h"
#include "SL_tasks.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_objects_defines.h"

#define TIME_OUT_NS  1000000

// global variabes
char initial_user_command[100]="";

// global functions 
void task_servo(void);

// local functions

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
  int i, j;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_task_servo();
  read_whichDOFs(config_files[WHICHDOFS],"task_servo");

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

  // spawn command line interface thread
  spawnCommandLineThread(initial_user_command);

  // reset the simulation
  if (!real_robot_flag)
    reset();
  
  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    // wait to take semaphore 
    if (semTake(sm_task_servo_sem,WAIT_FOREVER) == ERROR)
      stop("semTake Time Out -- Servo Terminated");

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_task_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("Task Servo Error Count = %d\n",task_servo_errors);

  return TRUE;

}
 
