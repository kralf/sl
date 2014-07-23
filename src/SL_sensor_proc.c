/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_sensor_proc.c

  \author  Stefan Schaal
  \date    2000

  ==============================================================================
  \remarks

  Core functions for sensor processing. Several user provided routines
  are needed

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"
#ifdef UNIX
#include "sys/ioctl.h"
#endif

/* private includes */
#include "utility.h"
#include "SL.h"
#include "SL_man.h"
#include "SL_sensor_proc.h"
#include "SL_motor_servo.h"
#include "SL_common.h"
#include "SL_collect_data.h"
#include "SL_filters.h"

/* local variables */
static SL_Jstate        *joint_raw_state;
static double           *misc_raw_sensor;
static int               filter_flag = TRUE;

/* global variables */
double          **joint_lin_rot;
double           *pos_polar;
double           *load_polar;

/* variables for filtering */
static Filter *fth;
static Filter *fthd;
static Filter *fthdd;
static Filter *fload;
static Filter *fmisc_sensor;

/* global functions */
void where_off(void);
void where_raw(void);
int  read_sensor_filters(char *fname);
void monitor_min_max(void);

/* local functions */
void   toggle_filter(void);

/*!*****************************************************************************
 *******************************************************************************
\note  init_sensor_processing
\date  May 2000
   
\remarks 

          Initializes all sensory processing

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_sensor_processing(void)

{
  
  int i,j;
  FILE      *in;
  char       string[100];
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    joint_lin_rot     = my_matrix(1,n_dofs,1,6);
    pos_polar         = my_vector(1,n_dofs);
    load_polar        = my_vector(1,n_dofs);
    joint_raw_state   = (SL_Jstate *) 
      my_calloc((unsigned long)(n_dofs+1),sizeof(SL_Jstate),MY_STOP);
    misc_raw_sensor   = (double *) 
      my_calloc((unsigned long)(n_misc_sensors+1),sizeof(double),MY_STOP);
    fth = (Filter *)
      my_calloc((unsigned long)(n_dofs+1),sizeof(Filter),MY_STOP);
    fthd = (Filter *)
      my_calloc((unsigned long)(n_dofs+1),sizeof(Filter),MY_STOP);
    fthdd = (Filter *)
      my_calloc((unsigned long)(n_dofs+1),sizeof(Filter),MY_STOP);
    fload = (Filter *)
      my_calloc((unsigned long)(n_dofs+1),sizeof(Filter),MY_STOP);
    fmisc_sensor = (Filter *)
      my_calloc((unsigned long)(n_misc_sensors+1),sizeof(Filter),MY_STOP);
  }

  /* initalizes translation to and from units */
  if (!init_user_sensor_processing())
    return FALSE;

  /* initialize filtering */
  if (!init_filters())
    return FALSE;

  /* read several variables from files */

  /* first, get the calibration values for dealing with the linear to
     rotary conversion */
  if (!read_sensor_calibration(config_files[SENSORCALIBRATION],
      joint_lin_rot,pos_polar,load_polar))
    return FALSE;

  /* second, get the max, min , and offsets of the position sensors */
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    return FALSE;

  /* third, get the filter cutoff values for all variable */
  if (!read_sensor_filters(config_files[SENSORFILTERS]))
    return FALSE;

  /* add function to man pages */
  addToMan("where_off","sensor readings without offsets",where_off);
  addToMan("where_raw","raw sensor readings",where_raw);
  addToMan("monitor_min_max","records min/max values of sensors",monitor_min_max);

  if (!real_robot_flag)
    addToMan("toggle_filter","toggles sensor filtering on and off",toggle_filter);


  /* make raw variables available for output */

  for (i=1; i<=n_dofs; ++i) {
    sprintf(string,"%s_rth",joint_names[i]);
    addVarToCollect((char *)&(joint_raw_state[i].th),string,"rad", DOUBLE,FALSE);
    sprintf(string,"%s_rthd",joint_names[i]);
    addVarToCollect((char *)&(joint_raw_state[i].thd),string,"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_rload",joint_names[i]);
    addVarToCollect((char *)&(joint_raw_state[i].load),string,"Nm", DOUBLE,FALSE);
  }

  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s_r",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_raw_sensor[i]),string,"-", DOUBLE,FALSE);
  }
  

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  toggle_filters
\date  Dec 1997
   
\remarks 

        reads the filter coefficients and initializes the appropriate
	variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
toggle_filter(void)
{

  if (filter_flag) {
    filter_flag = FALSE;
    printf("\n      --- Sensor filtering is off ---\n\n");
  } else {
    filter_flag = TRUE;
    printf("\n      --- Sensor filtering is on ---\n\n");
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_sensors
\date  Dec 1997
   
\remarks 

    gets sensor readings from the robot and converts them into standard
    units

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
read_sensors(void)

{

  int i,j;

  // get raw sensor reading from user routine
  if (!read_user_sensors(joint_raw_state,misc_raw_sensor))
    return FALSE;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  process_sensors
\date  Feb 1999
   
\remarks 

    filtering and differentiation of the sensor readings

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
process_sensors(void)

{

  int i,j;
  double aux;

  /* first the joint variables */
  for (i=1; i<=n_dofs; ++i) {
    joint_state[i].th    = filt(joint_raw_state[i].th,&fth[i]);
    joint_state[i].thd   = filt(joint_raw_state[i].thd,&fthd[i]);
    aux = (fthd[i].raw[1]-fthd[i].raw[2])*(double)motor_servo_rate;
    joint_state[i].thdd  = filt(aux,&fthdd[i]);
    joint_state[i].load  = filt(joint_raw_state[i].load,&fload[i]);

    if (!filter_flag && !real_robot_flag) {
      joint_state[i].th    = joint_sim_state[i].th;
      joint_state[i].thd   = joint_sim_state[i].thd;
      joint_state[i].thdd  = joint_sim_state[i].thdd;
      joint_state[i].load  = joint_sim_state[i].u;
    }

  }


  /* second the misc sensors */
  for (i=1; i<=n_misc_sensors; ++i) {
    misc_sensor[i]    = filt(misc_raw_sensor[i],&fmisc_sensor[i]);
    if (!filter_flag && !real_robot_flag) {
      misc_sensor[i] = misc_sim_sensor[i];
    }
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  send_commands
\date  April 1999
   
\remarks 

    translates the commands into raw and sends them to the robot

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
send_commands(void)

{

  int i,j;

  /* for simulations, the command is just copied to the simulation
     structure */

  send_user_commands(joint_state);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  where_off
\date  Feb 1999
\remarks 

 prints the raw positions without offsets

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
where_off(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: motor servo is not running!!\n");
  }

  printf("Note: Offset is relative to joint_default_state\n");

  for (i=1; i<=n_dofs; ++i) {

    printf("%10s  %f  %f  %f  %f  %f  %f\n",
	   joint_names[i],
	   joint_range[i][MIN_THETA],
	   joint_range[i][MAX_THETA],
	   joint_default_state[i].th,
	   joint_opt_state[i].th,
	   joint_opt_state[i].w,
	   joint_state[i].th + joint_range[i][THETA_OFFSET]
	   -joint_default_state[i].th);
	   
  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  where_raw
\date  Feb 1999
\remarks 

      prints the raw joint state

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
where_raw(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: motor servo is not running!!\n");
  }


  for (i=1; i<=n_dofs; ++i) {

    printf("%5s: th=% 5.3f  thd=% 6.3f  load=% 6.2f\n",
	   joint_names[i],
	   joint_raw_state[i].th,
	   joint_raw_state[i].thd,
	   joint_raw_state[i].load);
	   
  }
  printf("\n");

  for (i=1; i<=n_misc_sensors; ++i) {

    printf("%20s: value=% 5.3f\n",
	   misc_sensor_names[i],
	   misc_raw_sensor[i]);
	   
  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  monitor_min_max
\date  Dec. 2000
\remarks 

  monitors the min & max values of all joints continuously until keyboard
  is hit. Then, an appropriate min_max structure is printed out for usuage
  in SensorOffsets.cf

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
monitor_min_max(void)
{
  int i,j;
  Matrix  minmax;
  int n_bytes;
  double  offset = 0.05;

  if (!servo_enabled) {
    beep(1);
    printf("ERROR: motor servo is not running!!\n");
    return;
  }

  minmax = my_matrix(1,n_dofs,MIN_THETA,MAX_THETA);

  for (i=1; i<=n_dofs; ++i) {
    minmax[i][MIN_THETA] =  1.e10;
    minmax[i][MAX_THETA] = -1.e10;
  }

  printf("Hit any key to stop monitoring .....");
  fflush(stdout);

  /* just check for min & max values */
  while (TRUE) {
    
    for (i=1; i<=n_dofs; ++i) {
      
      /* update min/max values */
      if (joint_state[i].th > minmax[i][MAX_THETA])
	minmax[i][MAX_THETA] = joint_state[i].th;

      if (joint_state[i].th < minmax[i][MIN_THETA])
	minmax[i][MIN_THETA] = joint_state[i].th;
      
    }

    /* check whether the keyboard was hit */
#ifdef VX
    ioctl(0 , FIONREAD, (int) (&n_bytes));
#else
    ioctl( 0, FIONREAD, (void *) (&n_bytes) );
#endif
    if (n_bytes != 0)
      break;

    taskDelay(ns2ticks(10000000)); // wait 10ms

  }

  getchar();

  get_double("Safety margin from joint stop?",offset,&offset);

  /* print the min/max values in SensorOffset.cf format */
  for (i=1; i<=n_dofs; ++i) {

    printf("%10s  %f  %f  %f  %f  %f  %f\n",
	   joint_names[i],
	   minmax[i][MIN_THETA]+offset,
	   minmax[i][MAX_THETA]-offset,
	   joint_default_state[i].th,
	   joint_opt_state[i].th,
	   joint_opt_state[i].w,
	   joint_range[i][THETA_OFFSET]);
	   
  }
  printf("\n");
  fflush(stdout);

  my_free_matrix(minmax,1,n_dofs,MIN_THETA,MAX_THETA);

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_sensor_filters
\date  May 2000
\remarks 

parses the sensor filter configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : name of configuration file

 ******************************************************************************/
int
read_sensor_filters(char *fname) {

  int j, i,rc;
  char   string[100];
  FILE  *in;
  double dum;

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen(string,"r");
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all joint variables and read them into the appropriate array */
  for (i=1; i<= n_dofs; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      printf("ERROR: Cannot find filters for >%s<!\n",joint_names[i]);
      return FALSE;
    }
    rc=fscanf(in,"%d %d %d %d",&(fth[i].cutoff),
	   &(fthd[i].cutoff),&(fthdd[i].cutoff),&(fload[i].cutoff));
  }
  
  for (i=1; i<= n_misc_sensors; ++i) {
    if (!find_keyword(in, &(misc_sensor_names[i][0]))) {
      printf("ERROR: Cannot find filters for >%s<!\n",misc_sensor_names[i]);
      return FALSE;
    }
    rc=fscanf(in,"%d",&(fmisc_sensor[i].cutoff));
  }
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  init_joint_state_filters
\date  Oct. 2010
\remarks 

sets the history of the joint_state filters from a provided joint state.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     js : the joint state with all the values for filter initialization

 ******************************************************************************/
void
init_joint_state_filters(SL_Jstate *js)
{
  int i,j;

  for (i=1; i<=n_dofs; ++i) {
    for (j=0; j<=FILTER_ORDER; ++j) {
      fth[i].raw[j]    = js[i].th;
      fth[i].filt[j]   = js[i].th;
      fthd[i].raw[j]   = js[i].thd;
      fthd[i].filt[j]  = js[i].thd;
      fthdd[i].raw[j]  = js[i].thdd;
      fthdd[i].filt[j] = js[i].thdd;
      fload[i].raw[j]  = js[i].load;
      fload[i].filt[j] = js[i].load;
    }
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  init_misc_sensor_filters
\date  Oct. 2010
\remarks 

sets the history of the misc_sensor filters from a provided misc sensor array

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ms : the misc sensor state

 ******************************************************************************/
void
init_misc_sensor_filters(double *ms)
{
  int i,j;

  for (i=1; i<=n_dofs; ++i) {
    for (j=0; j<=FILTER_ORDER; ++j) {
      fmisc_sensor[i].raw[j]  = ms[i];
      fmisc_sensor[i].filt[j] = ms[i];
    }
  }

}

