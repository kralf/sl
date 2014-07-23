/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_go_cart_task.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  a simple task for point to point movements in cartesian space

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_filters.h"

/*! defines */

/* local variables */
static double     time_step;
static double     *cart;
static SL_Cstate  *ctarget;
static SL_Cstate  *cnext;
static SL_DJstate *target;
static int        *cstatus;
static SL_DJstate *target;
static int        firsttime = TRUE;
static int        special_flag = FALSE;
static double     movement_time = 1.0;
static double     tau;
static Filter    *fthdd;

/* global functions */
void add_goto_cart_task(void);

/* local functions */
static int  init_goto_cart_task(void);
static int  run_goto_cart_task(void);
static int  change_goto_cart_task(void);
static void init_vars(void);
static int  calculate_min_jerk_next_step (SL_Cstate *curr_state,
					  SL_Cstate *des_state,
					  double tau,
					  double delta_t,
					  SL_Cstate *next_states);
 
/*!*****************************************************************************
 *******************************************************************************
\note  add_goto_cart_task
\date  Feb 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_goto_cart_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;

    cart    = my_vector(1,n_endeffs*6);
    ctarget = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cnext   = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cstatus = my_ivector(1,n_endeffs*6);
    target  = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    fthdd   = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);
    target  = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);

    // initialize the filters
    init_filters();
    for (i=1; i<=n_dofs; ++i) 
      fthdd[i].cutoff = 5;
    
    addTask("Goto Cart Task", init_goto_cart_task, 
	    run_goto_cart_task, change_goto_cart_task);
    addToMan("goVisTarget","move one endeff to blob1",goVisTarget);
    
  }

}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_goto_cart_task
  \date  Dec. 1997

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_goto_cart_task(void)
{
  int    j, i;
  char   string[100];
  double max_range=0;
  int    ans;
  double aux;
  int    flag = FALSE;
  int    iaux;
  
  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Goto task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* zero the filters */
  for (i=1; i<=n_dofs; ++i) 
    for (j=0; j<=FILTER_ORDER; ++j)
      fthdd[i].raw[j] = fthdd[i].filt[j] = 0;
  
  /* initialize some variables */
  init_vars();
  time_step = 1./(double)task_servo_rate;

  if (!special_flag) {

    /* go to the same target as the current one, but with ID */
    for (i=1; i<=n_dofs; ++i)
      target[i] = joint_des_state[i];

    if (!go_target_wait_ID(target))
      return FALSE;

    /* input the movement speed */
    get_double("Movement Time?",movement_time,&movement_time);
    if (movement_time < 0.2)
      movement_time = 0.2;
    tau = movement_time;

    
    /* input the cartesian goal */
    for (i=1; i<=n_endeffs; ++i) {
      
      sprintf(string,"%s_x Status",cart_names[i]);
      iaux = cstatus[(i-1)*6+ _X_];
      get_int(string,cstatus[(i-1)*6+ _X_],&(cstatus[(i-1)*6+ _X_]));
      if (cstatus[(i-1)*6+ _X_ ]) {
	if (!iaux)
	  ctarget[i].x[_X_] = cart_des_state[i].x[_X_];
	flag = TRUE;
	sprintf(string,"%s_x Target",cart_names[i]);
	get_double(string,ctarget[i].x[_X_],&aux);
	ctarget[i].x[_X_] = aux;
      }
      
      sprintf(string,"%s_y Status",cart_names[i]);
      iaux = cstatus[(i-1)*6+ _Y_];
      get_int(string,cstatus[(i-1)*6+ _Y_],&(cstatus[(i-1)*6+_Y_]));
      if (cstatus[(i-1)*6+_Y_]) {
	if (!iaux)
	  ctarget[i].x[_Y_] = cart_des_state[i].x[_Y_];
	flag = TRUE;
	sprintf(string,"%s_y Target",cart_names[i]);
	get_double(string,ctarget[i].x[_Y_],&aux);
	ctarget[i].x[_Y_] = aux;
      }
      
      sprintf(string,"%s_z Status",cart_names[i]);
      iaux = cstatus[(i-1)*6+ _Z_];
      get_int(string,cstatus[(i-1)*6+ _Z_],&(cstatus[(i-1)*6+_Z_]));
      if (cstatus[(i-1)*6+_Z_]) {
	if (!iaux)
	  ctarget[i].x[_Z_] = cart_des_state[i].x[_Z_];
	flag = TRUE;
	sprintf(string,"%s_z Target",cart_names[i]);
	get_double(string,ctarget[i].x[_Z_],&aux);
	ctarget[i].x[_Z_] = aux;
      }
      
    }
    
    if (!flag)
      return FALSE;

    /* check whether target is reachable */
    SL_DJstate js[n_dofs+1];

    for (i=1; i<=n_dofs; ++i)
      js[i] = joint_des_state[i];

    if (!checkIKTarget(js, &base_state, &base_orient, endeff, ctarget, cstatus, 1000)) {
      printf("Target was reduced to reachable limits:\n");
      for (j=1; j<=n_endeffs; ++j) {
	for (i=1; i<=N_CART; ++i) {
	  if (cstatus[(j-1)*6+i]) {
	    printf("%s.%d = %f \n",cart_names[j],i,ctarget[j].x[i]);
	  }
	}
      }
    }

  }

  /* the cnext state is the desired state as seen form this program */
  for (i=1; i<=n_endeffs;++i) {
    cnext[i] = cart_des_state[i];
  }

  if (!special_flag) {
    
    /* ready to go */
    ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
	return FALSE;
    }
    
    if (ans != 1) 
      return FALSE;

    scd();
  
  } else {

    special_flag = FALSE;

  }
  
  return TRUE;

}

static void
init_vars(void) 
{
  if (firsttime) {
    firsttime = FALSE;
    ivec_zero(cstatus);
    vec_zero(cart);
    bzero((char *)&(ctarget[1]),n_endeffs*sizeof(ctarget[1]));
  }
}

/*!*****************************************************************************
 *******************************************************************************
  \note  run_goto_task
  \date  Dec. 1997

  \remarks 

  run the task from the task servo: REAL TIME requirements!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
run_goto_cart_task(void)
{
  int j, i;
  double sum=0;
  double aux;

  /* has the movement time expired? I intentially run 0.5 sec longer */
  if (tau <= -0.5 || (tau <= 0.0 && special_flag)) {
    freeze();
    return TRUE; 
  }

  /* progress by min jerk in cartesian space */
  calculate_min_jerk_next_step(cnext,ctarget,tau,time_step,cnext);
  tau -= time_step;
 

  /* shuffle the target for the inverse kinematics */
  for (i=1; i<=n_endeffs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      aux  = cnext[i].x[j] - cart_des_state[i].x[j];
      cart[(i-1)*6+j] = cnext[i].xd[j] + 20.*aux;
    }
  }

  /* inverse kinematics */
  for (i=1; i<=n_dofs; ++i) {
    target[i].th = joint_des_state[i].th;
  }
  if (!inverseKinematics(target,endeff,joint_opt_state,
			 cart,cstatus,time_step)) {
    freeze();
    return FALSE;
  }

  /* prepare inverse dynamics */
  for (i=1; i<=n_dofs; ++i) {
    aux = (target[i].thd-joint_des_state[i].thd)*(double)task_servo_rate;
    target[i].thdd  = filt(aux,&(fthdd[i]));

    joint_des_state[i].thdd = target[i].thdd;
    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].th   = target[i].th;

    if (joint_des_state[i].th > joint_range[i][MAX_THETA]) {
      joint_des_state[i].th = joint_range[i][MAX_THETA];
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].thdd = 0.0;
    }
    if (joint_des_state[i].th < joint_range[i][MIN_THETA]) {
      joint_des_state[i].th = joint_range[i][MIN_THETA];
      joint_des_state[i].thd = 0.0;
      joint_des_state[i].thdd = 0.0;
    }
  }

  /* compute inverse dynamics */
  SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_goto_cart_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_goto_cart_task(void)
{
  int j, i;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go_cart_target_wait
\date  
   
\remarks 

       go to the given cartesian target

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ctar :  cartesian states
 \param[in]     stat :  1/0 for active target or not
 \param[in]       mt :  movement time

 ******************************************************************************/
int
go_cart_target_wait(SL_Cstate *ctar,int *stat, double mt)
{

  int i,j;
  double last_time;
  double last_draw_time;
  double aux;

  special_flag = TRUE;

  /* initialize some variables */
  init_vars();

  /* assign the target variables */
  for (i=1; i<=n_endeffs; ++i) {
    for (j= _X_; j<= _Z_; ++j) {
      cstatus[(i-1)*6+j] = stat[(i-1)*6+j];
      aux = ctar[i].x[j];
      ctarget[i].x[j] = aux;
    }
  }
  
  /* the movement time */
  tau = mt;

  if (!setTaskByName("Goto Cart Task")) {
    special_flag = FALSE;
    return FALSE;
  }

  last_time = last_draw_time = task_servo_time;
  while (strcmp(current_task_name,NO_TASK) != 0) {
    if (task_servo_time - last_time > 2*mt) {
      printf("time out in go_cart_target_wait\n");
      special_flag = FALSE;
      return FALSE;
    }
    taskDelay(ns2ticks(10000000)); // wait 10ms
  }
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  calculate_min_jerk_next_step
\date  August 1994
   
\remarks 

        given a current cart state and a target cart
	state as well as movement duration, the next increment
	for the cart states is calculated. Note that this 
	is done in only in cartesian dimensions with active status.
	NOTE that this function requires velocity and accelerations
	as input as well!!!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     curr_states: the current state
 \param[in]     des_states : the desired state
 \param[in]     tau        : the desired movement duration until the goal is
	                 reached.
 \param[in]     dt         : at which delta time is the next_states from now
 \param[out]    next_states: the next state after dt

 ******************************************************************************/
static int 
calculate_min_jerk_next_step (SL_Cstate *curr_state,
			      SL_Cstate *des_state,
			      double tau,
			      double delta_t,
			      SL_Cstate *next_state)

{
  double t1,t2,t3,t4,t5;
  double tau1,tau2,tau3,tau4,tau5;
  int    i,j;

  if (delta_t > tau || tau < 1./(double)task_servo_rate || delta_t <= 0) {
    return FALSE;
  }

  t1 = delta_t;
  t2 = t1 * delta_t;
  t3 = t2 * delta_t;
  t4 = t3 * delta_t;
  t5 = t4 * delta_t;

  tau1 = tau;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  for (j=1; j<=n_endeffs; ++j) {
    for (i=1; i<=N_CART; ++i) {

      if (cstatus[(j-1)*6+i]) {
	
	/* calculate the constants */
	
	const double dist   = des_state[j].x[i] - curr_state[j].x[i];
	const double p1     = des_state[j].x[i];
	const double p0     = curr_state[j].x[i];
	const double a1t2   = des_state[j].xdd[i];
	const double a0t2   = curr_state[j].xdd[i];
	const double v1t1   = des_state[j].xd[i];
	const double v0t1   = curr_state[j].xd[i];
	
	const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 
	  3.*(v0t1 + v1t1)/tau4;
	const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
	  (8.*v0t1 + 7.*v1t1)/tau3; 
	const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
	  (6.*v0t1 + 4.*v1t1)/tau2; 
	const double c4 = curr_state[j].xdd[i]/2.;
	const double c5 = curr_state[j].xd[i];
	const double c6 = curr_state[j].x[i];
	
	next_state[j].x[i]   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
	next_state[j].xd[i]  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
	next_state[j].xdd[i] = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
	
      }
    }
  }
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  goVisTarget
\date  
   
\remarks 

       moves the hand to the blob1 target

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
void
goVisTarget(void) 

{
  static int ans=1;
  int i,j;
  static int *sta;
  static SL_Cstate *ct;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;
    ct = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate), MY_STOP);
    sta = my_ivector(1,n_endeffs*6);
  }

  if (blobs[1].status) {
    
    get_int("Which Endeffector?",ans,&ans);
    if (ans < 1 || ans > n_endeffs) 
      return;

    for (i=1; i<=n_endeffs*6; ++i)
      sta[i]=0;

    for (i=1; i<=N_CART; ++i) {
      sta[(ans-1)*6+i] = 1;
      ct[ans].x[i] = blobs[1].blob.x[i];
    }
    
    go_cart_target_wait(ct,sta, 1.0);  
    
  }
  
}
