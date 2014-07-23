/*!=============================================================================
  ==============================================================================

  \ingroup SLros
  
  \file    SL_ros_servo_unix.c

  \author  Stefan Schaal
  \date    July, 2010

  ==============================================================================
  \remarks

  main program for the ros servo

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_ros_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"

#define TIME_OUT_NS  1000000

// global functions 
void ros_servo(void);

// local functions

// external functions

/*!*****************************************************************************
 *******************************************************************************
\note  main
\date  July 2010
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
  init_ros_servo();

  // reset ros_servo variables
  servo_enabled          = 1;
  ros_servo_calls        = 0;
  last_ros_servo_time   = 0;
  ros_servo_time         = 0;
  ros_servo_errors       = 0;
  ros_servo_rate         = servo_base_rate/(double)task_servo_ratio;

  changeCollectFreq(ros_servo_rate);

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop (this is the same as the task servo)
  while (servo_enabled) {

    // wait to take semaphore 
    if (semTake(sm_ros_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated\n");
      return FALSE;
    }

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_ros_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("ROS Servo Error Count = %d\n",ros_servo_errors);

  return TRUE;

}
 
