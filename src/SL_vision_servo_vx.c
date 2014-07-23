/*!=============================================================================
  ==============================================================================

  \file    SL_vision_servo_vx.c

  \author  Stefan Schaal
  \date    June 1999

  ==============================================================================
  \remarks

  The program communicates with the QuickMag system and receives
  the (open loop) vision information. Since there is no data
  buffering in the QuickMag, it is necessary to poll the QuickMag
  connection all the time. The 712 transition module of the 
  QuickMag is used for communication.
  
  ============================================================================*/

/* vxWorks includes */
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "math.h"
#include "SL_vx_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_vision_servo.h"
#include "SL_shared_memory.h"
#include "lwpr.h"
#include "SL_man.h"

#define TIME_OUT_TICKS  500

/* global variables */
extern int stereo_mode;

/* local variables */
static int    vision_servo_tid;

/* global functions */
void status(void);
int stop(char *msg);

/* local functions */
static void vision_servo(void);
LOCAL void  usrFppCreateHook(FAST WIND_TCB *pTcb);

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
  STATUS stat;
  char  *path;

  /* set the current path of the target */
  path = getenv("path");
  if (path != NULL) {
    cd(path);
  } else {
    printf("WARNING: set the >path< variable to current working directory\n");
  }
  
  /* set the main clock rate */
  stat = sysClkRateSet(SYS_CLOCK_RATE);
  if (stat == ERROR) {
    printf("ERROR in init_vxworks\n");
    return FALSE;
  }

  /* avoid floating point underflow */
  fppCreateHookRtn = (FUNCPTR)usrFppCreateHook;

  /* add to man pages */
  addToMan("evs","enables the vision servo",NULL);
  addToMan("dvs","disables the vision servo",NULL);
  addToMan("status","displays status information about servo",NULL);

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
  \note  evs & enable_vision_servo
  \date  June 1999

  \remarks 

  enables the servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
void 
evs(void) 
{
  enable_vision_servo();
}

void 
enable_vision_servo( void )
{
  int status;
  int temp;

  if (!vision_servo_initialized) {
    printf("ERROR: Vision servo is not initialized\n");
    return;
  }
  
  if ( servo_enabled == 0 ) {
    servo_enabled           = 1;
    vision_servo_calls      = 0;
    vision_servo_time       = 0;
    servo_time              = 0;
    vision_servo_errors     = 0;
    vision_servo_rate       = VISION_SERVO_RATE;
    no_hardware_flag        = FALSE;
    changeCollectFreq(vision_servo_rate);

    /* initialize all vision variables to safe values */
    init_vision_states();
    init_learning();

    vision_servo_tid = taskSpawn("vision_servo",100,VX_FP_TASK,10000,
			       (FUNCPTR) vision_servo,0,0,0,0,0,0,0,0,0,0);
  }  else {
    fprintf( stderr, "vision servo is already on!\n" );
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  dvs & disable_vision_servo
\date  April 1999
   
\remarks 

        disables the vision servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void 
dvs(void)
{
  disable_vision_servo();
}

void 
disable_vision_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    
  } else
    fprintf( stderr, "vision servo is not on!\n" );
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  vision_servo
\date  June 1999
   
\remarks 

        clocked program to perform vision tasks

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static void 
vision_servo(void)

{

  int i,j;

  if (vision_servo_calls == 0) {
    
    /* do some initializations if necessary */
    
  }
    
  /**********************************************************************
   * start the loop 
   */
  
  while (servo_enabled) {

    /*************************************************************************
     * get the blobs
     */

    dta(d2a_hwv,d2a_bv,d2a_cv,0);
    

    if (!no_hardware_flag && !raw_blob_overwrite_flag) {

      if (!acquire_blobs(raw_blobs2D)) {
	dvs();
	return;
      }

    } else {

      dta(d2a_hwv,d2a_bv,d2a_cv,1000);
      
      if (semTake(sm_60Hz_sem,TIME_OUT_TICKS) == ERROR) {
	stop("semTake Time Out -- Servo Terminated");
	break;
      }

      for (i=1; i<=max_blobs; ++i) {
	raw_blobs2D[i][1].status = FALSE;
	raw_blobs2D[i][2].status = FALSE;
      }
      
    }

    
    /*************************************************************************
     *  run the vision servo routines
     */

    if (!run_vision_servo()) {
      dvs();
      return;
    }

    /*************************************************************************
     * end of while loop
     */

  }

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

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
  printf("            Time                   = %f\n",vision_servo_time);
  printf("            Servo Calls            = %d\n",vision_servo_calls);
  printf("            Servo Rate             = %d\n",vision_servo_rate);
  printf("            Servo Errors           = %d\n",vision_servo_errors);
  printf("            Servo Initialize       = %d\n",vision_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("           #frames read            = %d\n",count_all_frames);
  printf("           #frames lost            = %d\n",count_lost_frames);
  printf("            vision_pp              = %s\n",current_pp_name);
  printf("\n");

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
int
stop(char *msg)
{

  int i;

  dvs();
  beep(1);
  printf("%s\n",msg);
  
  return TRUE;

}

