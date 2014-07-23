/*!=============================================================================
  ==============================================================================

  \file    SL_motor_servo_vx.c

  \author 
  \date   

  ==============================================================================
  \remarks

  motor servo function that are not needed for simulation

  ============================================================================*/

/* vxWorks includes */

#include "stdio.h"
#include "math.h"
#include "string.h"
#include "math.h"
#include "SL_vx_headers.h"

/* private includes */
#include "SL.h"
#include "SL_motor_servo.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_controller.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_common.h"

/* variables for the motor servo */
static SEM_ID motor_servo_sem;
static int    motor_servo_tid;

/* global variables */
extern int           motor_servo_errors;
extern int           motor_servo_initialized;
extern int           count_no_receive;
extern double        count_no_receive_total;
extern int           count_no_broadcast;
extern int           no_receive_flag;

/* global functions */
void ems( int param );
void enable_motor_servo (int parm);
void disable_motor_servo(void);
void motor_servo_isr(void);
void motor_servo(void);
void status(void);

/* local functions */
static void usrFppCreateHook(FAST WIND_TCB *pTcb);

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
    printf("Path set to: %s\n",path);
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
  addToMan("ems","enables the motor servo",NULL);
  addToMan("dms","disables the motor servo",NULL);
  addToMan("status","displays status information about servo",NULL);

  /* real robot flag needs to be set */
  real_robot_flag = TRUE;
  real_time_clock_flag = TRUE;
  setRealRobotOptions();

  return TRUE; }

static void 
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
  \note  ems & enable_motor_servo
  \date  Dec 1997

  \remarks 

  enables the servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     param   : servo rate

 ******************************************************************************/
void 
ems( int param ) 
{
  enable_motor_servo( param );
}

void 
enable_motor_servo( int param )
{
  int status;
  int temp;

  if (!motor_servo_initialized) {
    printf("ERROR: Motor servo is not initialized\n");
    return;
  }
  
  if ( param < 1 ) param = servo_base_rate;

  if ( servo_enabled == 0 ) {
    servo_enabled            = 1;
    motor_servo_calls        = 0;
    servo_time               = 0;
    motor_servo_time         = 0;
    motor_servo_rate         = param;
    motor_servo_errors       = 0;
    count_no_receive         = 0;
    count_no_receive_total   = 0;
    count_no_broadcast       = 0;

    changeCollectFreq(param);

    zero_integrator();

    setDefaultPosture();

    sysAuxClkDisable();
    printf( "sysAuxClkRateGet() returns: %d\n", sysAuxClkRateGet() );
    status = sysAuxClkConnect( (FUNCPTR) motor_servo_isr, 0 );
    printf( "sysAuxClkConnect() returns: %d (%d)\n", status, OK ); 
    if ( status != OK )  {
      fprintf( stderr, "sysAuxClkConnect() NOT OK: %d (%d)\n",status, OK );
      return;
    }
    sysAuxClkRateSet( param );
    printf( "sysAuxClkRateGet() returns: %d\n", sysAuxClkRateGet() );

    motor_servo_sem = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
    motor_servo_tid = taskSpawn("motor_servo",0,VX_FP_TASK,10000,
				  (FUNCPTR) motor_servo,0,0,0,0,0,0,0,0,0,0);
    sysAuxClkEnable();

  }  else {
    fprintf( stderr, "motor servo is already on!\n" );
  }

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
    sysAuxClkDisable();
    servo_enabled = 0;
    semGive(motor_servo_sem);
  } else
    fprintf( stderr, "motor servo is not on!\n" );
}

/*!*****************************************************************************
 *******************************************************************************
\note  motor_servo_isr
\date  December 1997
   
\remarks 

        this program is clocked by the ISR, increments the time counter
	and gives the semaphore for other programs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
/* variable for the floating point context */
static FP_CONTEXT pfc;   
void 
motor_servo_isr(void)
{
  fppSave(&pfc);  
  ++motor_servo_calls;
  servo_time += 1./(double)motor_servo_rate;
  semGive(motor_servo_sem);
  fppRestore(&pfc); 
}

/*!*****************************************************************************
 *******************************************************************************
\note  motor_servo
\date  Feb 1999
\remarks 

        This program is clocked by the ISR indirectly throught the
	motor_servo_sem semaphore.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
void
motor_servo(void)

{
  int    j,i;
  int    last_tick=0;
  double dt;

  if (motor_servo_calls == 1) {
    
    /* do some initializations if necessary */
    
  }
    
  /**********************************************************************
   * start the loop 
   */
  
  while (servo_enabled) {

    /* wait to take semaphore */
    semTake(motor_servo_sem,WAIT_FOREVER); 

    /* execute the motor servo functions */
    if (!run_motor_servo())
      break;

    if (no_receive_flag)
      dta(d2a_hwm,d2a_bm,d2a_cm,2000);
    else
      dta(d2a_hwm,d2a_bm,d2a_cm,0);
    
  }  /* end servo while loop */
  
  semDelete(motor_servo_sem);
  printf("Motor Servo Error Count = %d\n",motor_servo_errors);

}

/*!*****************************************************************************
 *******************************************************************************
\note  local_status
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
  printf("\n");

}

