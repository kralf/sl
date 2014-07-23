/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_simulation_servo_unix.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  main program for running a numerical simulation of a robot, i.e.,
  this is where the equations of motion are integrated.

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
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_man.h"

#define TIME_OUT_NS  1000000

// global variables

// local variables
static int pause_flag = FALSE;

// global functions 

// local functions

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
  int i, j;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initalize the servo
  if (!init_simulation_servo())
    return FALSE;

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // boardcast the current state such that the motor servo can generate a command
  send_sim_state();
  send_misc_sensors();
  send_contacts();
  semGive(sm_motor_servo_sem);  

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);
  
  // run the servo loop
  while (servo_enabled) {

    // check for PAUSE
    if (semTake(sm_pause_sem,NO_WAIT) != ERROR) {
      if (pause_flag)
	pause_flag = FALSE;
      else
	pause_flag = TRUE;
    }

    if (pause_flag) {
      usleep(10000);
      continue;
    }

    // wait to take semaphore 
    if (semTake(sm_simulation_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated\n");
      return FALSE;
    }

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the simulation servo routines
    if (!run_simulation_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  return TRUE;

}
 
