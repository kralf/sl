/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo_xeno.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks

  Initialization and semaphore communication for the vision servo
  under xenomai

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_xeno_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_vision_servo.h"

/* global variables */
extern int stereo_mode;
int     delay_ns = FALSE;

/* local variables */
static RT_TASK servo_ptr;
static int     use_spawn = TRUE;
static int     servo_priority = 10;
static int     servo_stack_size = 2000000;
static int     cpuID = 0;

/* global functions */
void status(void);
int  stop(char *msg);

/* local functions */
static void vision_servo(void *dummy);
static int  checkForMessages(void);


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


  // parse command line options
  parseOptions(argc, argv);

  // initialize xenomai specific variables and real-time environment
  initXeno("vision");

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

  // get the servo parameters
  sprintf(name,"%s_servo",servo_name);
  read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority,
		       &servo_stack_size,&cpuID,&delay_ns);

  // initialize all vision variables to safe values
  init_vision_states();

  // make this process real-time
  if (use_spawn) {

    sprintf(name,"%s_servo_%d",servo_name, parent_process_id);
    
    if ((rc=rt_task_spawn(&servo_ptr,name,servo_stack_size,servo_priority,
			  T_FPU | T_JOINABLE | T_CPU(cpuID),vision_servo,NULL))) {
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
    vision_servo(NULL);

  }

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vision_servo
 \date  Oct 2009
 \remarks 
 
 This program is clocked by the motor servo and uses a shared
 memory semaphore for synchronization
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  dummy: dummary argument
 
 ******************************************************************************/
static void
vision_servo(void *dummy) 
{

  int i;
  int rc;

  //forces the mode switch
  rt_printf("entering vision servo\n");

  // warn upon mode switch
  if ((rc=rt_task_set_mode(0,T_WARNSW,NULL))) 
      printf("rt_task_set_mode returned %d\n",rc);
  
  // run the servo loop
  while (servo_enabled) {

    // force delay ticks if user wishes
    if (delay_ns > 0)
      taskDelay(ns2ticks(delay_ns));

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
    sl_rt_mutex_lock(&mutex1);

    // run the task servo routines
    if (!run_vision_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock(&mutex1);


  }  /* end servo while loop */

}
