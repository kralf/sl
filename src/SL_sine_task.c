/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_sine_task.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  simple sine task 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_dynamics.h"

/* local variables */

#define MAX_SINE 4
static double       *off;
static double      **amp;
static double      **phase;
static double      **freq;
static int          *n_sine;
static SL_DJstate   *target;
static double        task_time;
static double        speed=1.0;
static int           invdyn = TRUE;
static double        trans_mult;
static double        trans_period;

/* global variables */

/* global functions */

void add_sine_task(void);

/* local functions */
static int init_sine_task(void);
static int run_sine_task(void);
static int change_sine_task(void);
static int read_sine_script(void);
 
/*!*****************************************************************************
 *******************************************************************************
\note  add_sine_task
\date  Feb 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_sine_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;
    off = my_vector(1,n_dofs);
    amp = my_matrix(1,n_dofs,1,MAX_SINE);
    phase = my_matrix(1,n_dofs,1,MAX_SINE);
    freq = my_matrix(1,n_dofs,1,MAX_SINE);
    n_sine = my_ivector(1,n_dofs);
    target = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    
    addTask("Sine Task", init_sine_task,run_sine_task, change_sine_task);
    
  }
  
}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_sine_task
  \date  Dec. 1997

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_sine_task(void)
{
  int j, i;
  char string[100];
  double max_range=0;
  int ans;

  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("New task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* allow or speed adjustment */
  get_double("Frequency Multiplier",speed,&speed);

  /* allow inverse dynamics */
  get_int("Use Inverse Dynamics",invdyn,&invdyn);

  /* read the script for this task */
  if (!read_sine_script())
    return FALSE;

  /* transient multiplier: ramps up the amplitude in one period to 
     avoid discontinuous motor commands */
  trans_mult = 0.0;

  /* what is the longest period? this is what we use to ramp up trans_mult to one */
  trans_period = 0.0;
  for (i=1; i<=n_dofs; ++i) 
    for (j=1; j<=n_sine[i]; ++j)
      if (freq[i][j] > 0)
	if (1./freq[i][j] > trans_period)
	  trans_period = 1./freq[i][j];

  /* go to a save posture */
  bzero((char *)&(target[1]),n_dofs*sizeof(target[1]));
  for (i=1; i<=n_dofs; ++i) {
    target[i].th  = off[i];
    for (j=1; j<=n_sine[i]; ++j) {
      target[i].th  += trans_mult*amp[i][j]*sin(phase[i][j]);
    }
  }

  if (invdyn) {
    if (!go_target_wait_ID(target))
      return FALSE;
  } else {
    if (!go_target_wait(target))
      return FALSE;
  }

  /* do we really want to do this task? */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  if (ans != 1) 
    return FALSE;

  task_time = 0.0;
  scd();

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  run_sine_task
  \date  Dec. 1997

  \remarks 

  run the task from the task servo: REAL TIME requirements!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
run_sine_task(void)
{
  int j, i;

  task_time += 1./(double)task_servo_rate;

  trans_mult = task_time/trans_period;
  if (trans_mult > 1.0)
    trans_mult = 1.0;

  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].th  = off[i];
    joint_des_state[i].thd = 0.0;
    joint_des_state[i].thdd = 0.0;
    joint_des_state[i].uff = 0.0;
    for (j=1; j<=n_sine[i]; ++j) {
      joint_des_state[i].th += 
	trans_mult*amp[i][j] * sin(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
      joint_des_state[i].thd += trans_mult*amp[i][j] * 2.*PI*speed*freq[i][j] * 
	cos(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
      joint_des_state[i].thdd += -trans_mult*amp[i][j] * sqr(2.*PI*speed*freq[i][j]) * 
	sin(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
    }
  }

  if (invdyn)
    SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_sine_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_sine_task(void)
{
  int j, i;
  char string[100];


  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  read_sine_script
  \date  June 1999

  \remarks 

  parse a script which describes the sine task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
read_sine_script(void)
{
  int j,i,rc;
  static char string[100];
  static char fname[100] = "default.sine";
  FILE *fp;
  int found = FALSE;
  double total_amp;
  double ratio;

  /* clear the current parameters */
  for (i=1; i<=n_dofs; ++i) {
    off[i] = joint_default_state[i].th;
    n_sine[i] = 0;
    for (j=1; j<=MAX_SINE; ++j) {
      amp[i][j] = phase[i][j] = freq[i][j] = 0.0;
    }
  }

  /* open the script, and parse the parameters */

  while (TRUE) {
    if (!get_string("Name of the Sine Script File\0",fname,fname)) 
      return FALSE;
    
    /* try to read this file */
    sprintf(string,"%s%s",PREFS,fname);
    fp = fopen_strip(string);
    if (fp != NULL)
      break;
  }

  for (i=1; i<= n_dofs; ++i) {
    if (find_keyword(fp, &(joint_names[i][0]))) {
      found = TRUE;
      total_amp = 0.0;
      rc=fscanf(fp,"%d %lf",&n_sine[i],&off[i]);
      /* check for out of range */
      if (off[i] > joint_range[i][MAX_THETA]) {
	off[i] = joint_range[i][MAX_THETA];
	printf("Reduced offset of joint %s to %f\n",joint_names[i],off[i]);
      }
      if (off[i] < joint_range[i][MIN_THETA]) {
	off[i] = joint_range[i][MIN_THETA];
	printf("Reduced offset of joint %s to %f\n",joint_names[i],off[i]);
      }
      for (j=1; j<=n_sine[i]; ++j) {
	rc=fscanf(fp,"%lf %lf %lf",&(amp[i][j]),&(phase[i][j]),&(freq[i][j]));
	total_amp += amp[i][j];
      }
      /* check for out of range */
      if (total_amp+off[i] > joint_range[i][MAX_THETA]) {
	ratio = total_amp/(joint_range[i][MAX_THETA]-off[i]+1.e-10);
	for (j=1; j<=n_sine[i]; ++j)
	  amp[i][j] /= ratio;
	printf("Reduced amplitude of joint %s to %f\n",joint_names[i],
	       amp[i][j]);
      }
      if (-total_amp+off[i] < joint_range[i][MIN_THETA]) {
	ratio = total_amp/(off[i]-joint_range[i][MIN_THETA]+1.e-10);
	for (j=1; j<=n_sine[i]; ++j)
	  amp[i][j] /= ratio;
	printf("Reduced amplitude of joint %s to %f\n",joint_names[i],
	       amp[i][j]);
      }
    }
  }
  
  fclose(fp);
  remove_temp_file();


  return found;

}
