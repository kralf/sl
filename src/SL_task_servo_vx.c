/*!=============================================================================
  ==============================================================================

  \file    SL_task_servo_vx.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  manages the control loop to run different tasks 

  ============================================================================*/

/* vxWorks includes */

#include "stdio.h"
#include "math.h"
#include "string.h"
#include "math.h"
#include "SL_vx_headers.h"

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

#define TIME_OUT_TICKS  500

/* variables for the task servo */
static int    task_servo_tid;
static int    task_servo_errors;
static long   task_servo_calls=0;
static int    local_task_servo_ratio;

/* global functions */
void ets( int param );
void enable_task_servo (int parm);
void disable_task_servo(void);
void task_servo(void);

/* local functions */
static void  usrFppCreateHook(FAST WIND_TCB *pTcb);

/*!*****************************************************************************
 *******************************************************************************
\note  init_vxworks
\date  Feb 1999
\remarks 

initializes all necessary things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_vxworks( void )

{
  int i, j;
  STATUS status;
  char  *path;
  
  /* set the current path of the target */
  path = getenv("path");
  if (path != NULL) {
    cd(path);
  } else {
    printf("WARNING: set the >path< variable to current working directory\n");
  }
  
  /* set the main clock rate */
  status = sysClkRateSet(SYS_CLOCK_RATE);
  if (status == ERROR) {
    printf("ERROR in init_vxworks\n");
    return FALSE;
  }

  /* avoid floating point underflow */
  fppCreateHookRtn = (FUNCPTR)usrFppCreateHook;

  /* add to man pages */
  addToMan("ets","enables the task servo",NULL);
  addToMan("dts","disables the task servo",NULL);

  /* real robot flag needs to be set */
  real_robot_flag = TRUE;
  setRealRobotOptions();

  return TRUE;
  
}    

LOCAL void 
usrFppCreateHook(FAST WIND_TCB *pTcb)
{

  /*
  ** Modify the the floating-point status and control register in the
  ** floating- point context of the new task.
  */
  pTcb->pFpContext->fpcsr &= ~_PPC_FPSCR_UE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  ets & enable_task_servo
  \date  Dec 1997

  \remarks 

  enables the servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     param   : indicator at which ratio the servo is to run
                    1, 2, 3, 4, times down sampled, or 60 for 60Hz

 ******************************************************************************/
void 
ets( int param ) 
{
  enable_task_servo( param );
}

void 
enable_task_servo( int param )
{
  int status;
  int temp;

  if (!task_servo_initialized) {
    printf("ERROR: Task servo is not initialized\n");
    return;
  }
  
  if ( param < 1 ) param = task_servo_ratio;

  if ( servo_enabled == 0 ) {
    servo_enabled      = 1;
    task_servo_calls        = 0;
    task_servo_time         = 0;
    local_task_servo_ratio  = param;
    task_servo_errors       = 0;
    task_servo_rate         = servo_base_rate/param;
    if (param == R60HZ)
      task_servo_rate = R60HZ;

    changeCollectFreq(task_servo_rate);

    setTaskByName(NO_TASK);

    setDefaultPosture();

    task_servo_tid = taskSpawn("task_servo",0,VX_FP_TASK,10000,
			       (FUNCPTR) task_servo,0,0,0,0,0,0,0,0,0,0);
  }  else {
    fprintf( stderr, "task servo is already on!\n" );
  }

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
    
    semFlush(sm_1to1_sem);
    semFlush(sm_1to2_sem);
    semFlush(sm_1to3_sem);
    semFlush(sm_1to4_sem);
    semFlush(sm_1to5_sem);
    semFlush(sm_60Hz_sem);
      
  } else
    fprintf( stderr, "task servo is not on!\n" );
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  task_servo
\date  Feb 1999
\remarks 

        This program is clocked by the motor servo and uses a shared
	memory semaphore

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
void
task_servo(void)

{
  int    j,i;
  double dt;

  if (task_servo_calls == 0) {
    
    /* do some initializations if necessary */
    
  }
    
  /**********************************************************************
   * start the loop 
   */
  
  while (servo_enabled) {

    /* wait to take semaphore */
   
    switch (local_task_servo_ratio) {
    case R1TO1:
      if (semTake(sm_1to1_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO2:
      if (semTake(sm_1to2_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO3:
      if (semTake(sm_1to3_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO4:
      if (semTake(sm_1to4_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R1TO5:
      if (semTake(sm_1to5_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    case R60HZ:
      if (semTake(sm_60Hz_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      break;
      
    default:
      if (semTake(sm_1to2_sem,TIME_OUT_TICKS) == ERROR)
	stop("semTake Time Out -- Servo Terminated");

    }

    if (!servo_enabled)
      break;

    /* run the task servo routines */
    if (!run_task_servo())
      break;

  }  /* end servo while loop */

  printf("Task Servo Error Count = %d\n",task_servo_errors);

  
}
 
