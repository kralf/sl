/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo_unix.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks

  Initialization and semaphore communication for the vision servo
  under unix

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_vision_servo.h"

#define TIME_OUT_NS  100000

/* global variables */
extern int stereo_mode;

/* local variables */

/* global functions */

/* local functions */


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
  int rc;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_vision_servo();

  // setup servo
  servo_enabled           = 1;
  vision_servo_calls      = 0;
  last_vision_servo_time  = 0;
  vision_servo_time       = 0;
  servo_time              = 0;
  vision_servo_errors     = 0;
  vision_servo_rate       = VISION_SERVO_RATE;
  changeCollectFreq(vision_servo_rate);

  // initialize all vision variables to safe values
  init_vision_states();

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    // check whether there is some hardware interaction -- the vision hardware would
    // provide the clocking
    if (!no_hardware_flag && !raw_blob_overwrite_flag) {

      // this is just added to notice when SL is aborted
      if (semGet(sm_vision_servo_sem,&rc) == ERROR)
	stop("semTake Time Out -- Servo Terminated");

      if (!acquire_blobs(raw_blobs2D)) {
	no_hardware_flag = TRUE;
      }

    } else { // with no hardware, we rely on the internal clock

      // wait to take semaphore 
      if (semTake(sm_vision_servo_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      
      // reset the blob status
      for (i=1; i<=max_blobs; ++i) {
	raw_blobs2D[i][1].status = FALSE;
	raw_blobs2D[i][2].status = FALSE;
      }
      
    }

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_vision_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

  return TRUE;

}

