/*!=============================================================================
  ==============================================================================

  \ingroup SLmotor

  \file    SL_controller.c

  \author 
  \date   

  ==============================================================================
  \remarks

      feedback & feedforwad controller

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_motor_servo.h"
#include "SL_controller.h"
#include "SL_man.h"
#include "SL_sensor_proc.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "utility.h"

/* global variables */
double *controller_gain_th;
double *controller_gain_thd;
double *controller_gain_int;
double *upd;

/* local variables */
static double *u;
static double *ufb;
static double *integrator_state;

/*! availabe control models */
#define PD    1
#define PID   2
#define PDFF  3
#define PIDFF 4

int controller_kind = PDFF;
int power_on;


/* global functions */

/* local functions */
static void changePIDGains(double *pGain, double *dGain, double *iGain);

/* external variables */

extern int no_receive_flag;

/*!*****************************************************************************
 *******************************************************************************
\note  init_controller
\date  Feb 1999
\remarks 

initializes all necessary things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_controller( void )

{
  int i, j;
  FILE *in;
  char string[100];
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    u = my_vector(1,n_dofs);
    ufb = my_vector(1,n_dofs);
    upd = my_vector(1,n_dofs);
    controller_gain_th = my_vector(1,n_dofs);
    controller_gain_thd = my_vector(1,n_dofs);
    controller_gain_int = my_vector(1,n_dofs);
    integrator_state = my_vector(1,n_dofs);
  }

  /* read the control gains and max controls  */
  if (!read_gains(config_files[GAINS],controller_gain_th, controller_gain_thd, controller_gain_int))
    return FALSE;

  addToMan("ck","change controller",ck);
  addToMan("setGains","Change PID gain settings",setGainsSim);

  return TRUE;
  
}    

/*!*****************************************************************************
 *******************************************************************************
\note  ck & controllerKind
\date  August 7, 1992
   
\remarks 

        determines the current control mode

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void 
ck(void) {
  
  controllerKind();

}

void
controllerKind() 

{
  int aux,i;
  long count=0;

#ifdef VX
  if (!no_receive_flag) {

    printf("Shut down the task loop before changing the controller\n");
    return;

  }
#endif

  AGAIN:
    printf("\n\n\nChoose Controller Kind:\n\n");
    printf("        PD                              ---> %d\n",PD);
    printf("        PID                             ---> %d\n",PID);
    printf("        PD  + Feedforward               ---> %d\n",PDFF);
    printf("        PID + Feedforward               ---> %d\n",PIDFF);
    printf("\n");
    get_int("        ----> Input",controller_kind,&aux);

    if (aux > PIDFF || aux < PD) {

      goto AGAIN;

    } else {

      if (controller_kind != PIDFF && aux == PIDFF)
	zero_integrator();
      if(controller_kind != PID && aux == PID)
        zero_integrator();
      controller_kind = aux;

    }

}

/*!*****************************************************************************
 *******************************************************************************
\note  generate_total_commands
\date  April 1999
   
\remarks 

        this is the feedback/feedforward controller

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
generate_total_commands( void)

{
  int    j,i;

#ifdef VX
  power_on = user_power_status(); 
#else
  power_on = TRUE;
#endif

  if (power_on == -1) 
    return FALSE;



  switch (controller_kind) {

  case PD:

    for (i=1; i<=n_dofs; ++i) {

      ufb[i] = 0.0;

      if (zero_ufb_P_flag[i] > 0) 
	--zero_ufb_P_flag[i];
      else
	ufb[i] += (joint_des_state[i].th - joint_state[i].th) * 
	  controller_gain_th[i];

      if (zero_ufb_D_flag[i] > 0) 
	--zero_ufb_D_flag[i];
      else
	ufb[i] += (joint_des_state[i].thd - joint_state[i].thd) * 
	  controller_gain_thd[i];

      u[i]   = ufb[i];
      upd[i] = ufb[i];

    }
    break;

  case PID:

    for (i=1; i<=n_dofs; ++i) {

      ufb[i] = 0.0;

      if (zero_ufb_P_flag[i] > 0) 
	--zero_ufb_P_flag[i];
      else {
	integrator_state[i] += joint_des_state[i].th - joint_state[i].th;
	ufb[i] += (joint_des_state[i].th - joint_state[i].th) * 
	  controller_gain_th[i];
      }

      if (zero_ufb_D_flag[i] > 0) 
	--zero_ufb_D_flag[i];
      else
	ufb[i] += (joint_des_state[i].thd - joint_state[i].thd) * 
	  controller_gain_thd[i];

      if (!power_on) 
	integrator_state[i] = 0;

      upd[i] = ufb[i];
      ufb[i] += integrator_state[i] * controller_gain_int[i];

      u[i] = ufb[i];
    }
    break;

  case PDFF:

    for (i=1; i<=n_dofs; ++i) {

      ufb[i] = 0.0;

      if (zero_ufb_P_flag[i] > 0) 
	--zero_ufb_P_flag[i];
      else
	ufb[i] += (joint_des_state[i].th - joint_state[i].th) * 
	  controller_gain_th[i];

      if (zero_ufb_D_flag[i] > 0) 
	--zero_ufb_D_flag[i];
      else
	ufb[i] += (joint_des_state[i].thd - joint_state[i].thd) * 
	  controller_gain_thd[i];

      upd[i] = ufb[i];
      u[i]   = ufb[i] + joint_des_state[i].uff;
    }
    break;

  case PIDFF:
 
    for (i=1; i<=n_dofs; ++i) {

      ufb[i] = 0.0;

      if (zero_ufb_P_flag[i] > 0) 
	--zero_ufb_P_flag[i];
      else {
	integrator_state[i] += joint_des_state[i].th - joint_state[i].th;
	ufb[i] += (joint_des_state[i].th - joint_state[i].th) * 
	  controller_gain_th[i];
      }

      if (zero_ufb_D_flag[i] > 0) 
	--zero_ufb_D_flag[i];
      else
	ufb[i] += (joint_des_state[i].thd - joint_state[i].thd) * 
	  controller_gain_thd[i];

      if (!power_on) 
	integrator_state[i] = 0;

      upd[i]  = ufb[i];
      ufb[i] += integrator_state[i] * controller_gain_int[i];

      u[i] = ufb[i] + joint_des_state[i].uff;
    }
    break;

  default:

    stop("Invalid Controller");

  }

  // allow the user to modify the controller
  user_controller(u,ufb);

  // apply the new control
  for (i=1; i<=n_dofs; ++i) {
    if (my_isnan(u[i])) {
      char   string[100];
      bzero((char *)&(ufb[1]),sizeof(double)*n_dofs);
      bzero((char *)&(u[1]),sizeof(double)*n_dofs);
      sprintf(string,"NaN in motor command of joint %s",joint_names[i]);
      stop(string);
    }
    if (fabs(u[i]) > u_max[i]) {
      u[i] = macro_sign(u[i]) * u_max[i];
    }
    joint_state[i].u   = u[i];
    joint_state[i].ufb = ufb[i];
  }

  return TRUE;

}
 

/*!*****************************************************************************
 *******************************************************************************
\note  zero_integrator
\date  April 1999
   
\remarks 

        zeros the integrator states

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
zero_integrator( void) 

{

  int i;

  for (i=1; i<=n_dofs; ++i) {
    integrator_state[i] = 0;
  }

}

#ifdef VX
/*!*****************************************************************************
 *******************************************************************************
\note  stop
\date  August 7, 1992 
   
\remarks 

       send pump into low pressure and terminate loops

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
int
stop(char *msg)
{

  int i;

  user_kill();
  zero_integrator();
  dms();
  beep(1);
  printf("%s\n",msg);
  
  return TRUE;

}
#endif
/*!*****************************************************************************
 *******************************************************************************
\note  setGainsSim
\date  August 7, 1992 
   
\remarks 

       adjust the gains, with interface for simulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
setGainsSim(void)
{
  static int jID=0;

  if (!get_int("Which joint?",jID,&jID))
    return;

  setGains(jID);
}

/*!*****************************************************************************
 *******************************************************************************
\note  setGains
\date  August 7, 1992 
   
\remarks 

       adjust the gains

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
setGains(int id)
{

  int i;
  char string[100];
  double temp;

  if (id < 1 || id > n_dofs) {
    get_int("Give the ID of joint for setGains",id,&id);
    if (id < 1 || id > n_dofs) {
      printf("Invalid joint ID ... Aborted ...\n");
      return;
    }
  }
  
  printf("Be careful: Very little error checking is done\n");
  
  i = id;
  
  sprintf(string,"%5s Position Gain",joint_names[i]);
  get_double(string,controller_gain_th[i],&temp);
  if (temp < 0)
    temp = 0;
  if (temp > 1000)
    temp = controller_gain_th[i];
  controller_gain_th[i] = temp;
  
  sprintf(string,"%5s Velocity Gain",joint_names[i]);
  get_double(string,controller_gain_thd[i],&temp);
  if (temp < 0)
    temp = 0;
  if (temp > 1000)
    temp = controller_gain_thd[i];
  controller_gain_thd[i] = temp;
  
  sprintf(string,"%5s Integral Gain",joint_names[i]);
  get_double(string,controller_gain_int[i],&temp);
  if (temp < 0)
    temp = 0;
  if (temp > 1000)
    temp = controller_gain_int[i];
  controller_gain_int[i] = temp;
  
  for (i=1; i<=n_dofs; ++i) {
    printf("%s %f %f %f %f\n",
	   joint_names[i],controller_gain_th[i],controller_gain_thd[i],controller_gain_int[i],u_max[i]);
  }

  // communicate the gain change to other servos
  changePIDGains(controller_gain_th,controller_gain_thd, controller_gain_int);
  
  return;

}

/*!*****************************************************************************
 *******************************************************************************
\note  changePIDGains
\date  Nov. 2005
   
\remarks 

        sends the request to change the PID gains to the relevant servos

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     pGain : the proportional gains
 \param[in]     dGain : the derivative gains
 \param[in]     iGain : the integral gains

 ******************************************************************************/
static void 
changePIDGains(double *pGain, double *dGain, double *iGain) 
{
  int i,j;
  float buf[3*n_dofs+1];
  unsigned char cbuf[(3*n_dofs)*sizeof(float)];

  for (i=1; i<=n_dofs; ++i) {
    buf[i] = pGain[i];
    buf[i+n_dofs] = dGain[i];
    buf[i+2*n_dofs] = iGain[i];
  }
    
  memcpy(cbuf,(void *)&(buf[1]),(3*n_dofs)*sizeof(float));
    
  sendMessageSimulationServo("changePIDGains",(void *)cbuf,(3*n_dofs)*sizeof(float));

}
