/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_motor_servo_unix.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  main program for the motor servo

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* system includes */
#ifdef i386
#ifndef i386mac
#ifndef x86_64mac
#include "bits/time.h"
#endif
#endif
#endif
#ifdef x86_64
#ifndef x86_64mac
#include "bits/time.h"
#endif
#endif

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_motor_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_controller.h"

// external variables
extern int           motor_servo_errors;
extern int           motor_servo_initialized;
extern int           count_no_receive;
extern double        count_no_receive_total;
extern int           count_no_broadcast;
extern int           no_receive_flag;

// global functions 
void motor_servo(void);

// local functions

// local variables
static sl_rt_cond cond;
static double real_time;
static double real_time_dt;

// external functions
#ifdef i386
#ifndef i386mac
#ifndef x86_64mac
extern int clock_nanosleep (clockid_t __clock_id, int __flags,
			    __const struct timespec *__req,
			    struct timespec *__rem);
#endif
#endif
#endif

#ifdef x86_64
extern int clock_nanosleep (clockid_t __clock_id, int __flags,
			    __const struct timespec *__req,
			    struct timespec *__rem);
#endif


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
  struct timespec   ts;
  struct timeval    tp,start_tp;
  double last_real_time;

  //init the cond
  sl_rt_cond_init(&cond);

  // parse command line options
  parseOptions(argc, argv);

  // get the clock option, i.e., the motor servo acts as a real-time clock
  real_time_clock_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-rtc")==0) {
      real_time_clock_flag = TRUE;
      break;
    }
  }

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_motor_servo();
  read_whichDOFs(config_files[WHICHDOFS],"motor_servo");

  // reset motor_servo variables
  servo_enabled            = 1;
  motor_servo_calls        = 0;
  servo_time               = 0;
  motor_servo_time         = 0;
  last_motor_servo_time    = 0;
  motor_servo_rate         = servo_base_rate;
  motor_servo_errors       = 0;
  count_no_receive         = 0;
  count_no_receive_total   = 0;
  count_no_broadcast       = 0;
  
  changeCollectFreq(motor_servo_rate);
  if (real_time_clock_flag) {
    addVarToCollect((char *)&(real_time),"real_time","s", DOUBLE,FALSE);
    addVarToCollect((char *)&(real_time_dt),"real_time_dt","s", DOUBLE,FALSE);
  }
  updateDataCollectScript();
  setDefaultPosture();
  zero_integrator();
  
  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // initialize time
  gettimeofday(&start_tp, NULL);
  ts.tv_sec       = start_tp.tv_sec;
  ts.tv_nsec      = start_tp.tv_usec*1000;
  last_real_time  = 0;

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    if (real_time_clock_flag) { // motor_servo acts as real time clock

      // increment the time
      ts.tv_nsec += (int)(1000000000./(double)motor_servo_rate);
      ts.tv_sec  += (int)(ts.tv_nsec/1000000000.);
      ts.tv_nsec  = ts.tv_nsec%1000000000;

      // check for too slow processing (or slow system clock)
      gettimeofday(&tp, NULL);
      real_time      = (tp.tv_sec-start_tp.tv_sec)+(tp.tv_usec-start_tp.tv_usec)/1000000.;
      real_time_dt   = real_time-last_real_time;
      last_real_time = real_time;

      if ( (tp.tv_sec-ts.tv_sec) + (tp.tv_usec*1000-ts.tv_nsec)/1000000000. > 0) {
 
	motor_servo_errors += (int) ((tp.tv_sec-ts.tv_sec) + 
				     (tp.tv_usec*1000-ts.tv_nsec)/1000000000.)* 
	  motor_servo_rate;
	
      }

#ifdef i386
#ifdef i386mac
      // use pthread_cond_timedwait for absolute timing
      sl_rt_time timeout = (sl_rt_time)(ts.tv_nsec) + (sl_rt_time)(ts.tv_sec * 1000000000);
      sl_rt_mutex_lock(&mutex1);
      sl_rt_cond_timedwait(&cond, &mutex1, timeout);
      sl_rt_mutex_unlock(&mutex1);
#else
#ifdef x86_64mac
      // use pthread_cond_timedwait for absolute timing
      sl_rt_time timeout = (sl_rt_time)(ts.tv_nsec) + (sl_rt_time)(ts.tv_sec * 1000000000);
      sl_rt_mutex_lock(&mutex1);
      sl_rt_cond_timedwait(&cond, &mutex1, timeout);
      sl_rt_mutex_unlock(&mutex1);
#else
      clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts,NULL);
#endif
#endif
#else
#ifdef x86_64
      clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts,NULL);
#else
      // use pthread_cond_timedwait for absolute timing
      sl_rt_time timeout = (sl_rt_time)(ts.tv_nsec) + (sl_rt_time)(ts.tv_sec * 1000000000);
      sl_rt_mutex_lock(&mutex1);
      sl_rt_cond_timedwait(&cond, &mutex1, timeout);
      sl_rt_mutex_unlock(&mutex1);
#endif
#endif

      if (semGive(sm_motor_servo_sem) == ERROR)
	exit(-1);

    } 

    // wait to take semaphore
    if (semTake(sm_motor_servo_sem,WAIT_FOREVER) == ERROR) {
      printf("semTake Time Out -- Servo Terminated");
      exit(-1);
    }
    
    // lock out the keyboard interaction
    sl_rt_mutex_lock(&mutex1);

    // run the task servo routines
    if (!run_motor_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock(&mutex1);

    // trigger the simulation servo
    if (semGive(sm_simulation_servo_sem) == ERROR)
      exit(-1);


  }  /* end servo while loop */

  printf("Motor Servo Error Count = %d\n",motor_servo_errors);

  return TRUE;

}
 
