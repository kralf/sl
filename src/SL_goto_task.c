/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_goto_task.c

  \author 
  \date   

  ==============================================================================
  \remarks

  a simple task for point to point movements in joint space

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_dynamics.h"

/* local variables */
static SL_DJstate *joint_goto_state;
static SL_DJstate *joint_increment;
static SL_DJstate *joint_special_state;
static double goto_speed = 1.0;
static int n_steps;
static int n_goto_steps;
static int special_posture_flag = FALSE;

/* global functions */
void add_goto_task(void);

/* local functions */
static int init_goto_task(void);
static int run_goto_task(void);
static int change_goto_task(void);
static void sim_go(void);
static int calculate_min_jerk_next_step (SL_DJstate *state, SL_DJstate *goal,double tau);

 
/*!*****************************************************************************
 *******************************************************************************
\note  add_goto_task
\date  Feb 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_goto_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  double aux;
  
  if (firsttime) {

    firsttime = FALSE;

    joint_goto_state = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    joint_increment = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    joint_special_state = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);

    addTask("Goto Task", init_goto_task, run_goto_task, change_goto_task);
    addToMan("go0","go to default posture",go0);
    addToMan("go","go to a specific posture",sim_go);

    if (read_parameter_pool_double(config_files[PARAMETERPOOL],"goto_speed",&aux)) {
      if (aux > 0 && aux < 3.0) {
	goto_speed = aux;
      } else 
	printf("Invalid goto_speed (%f)in parameter pool file\n",aux);
    }

  }

}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_goto_task
  \date  Dec. 1997

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_goto_task(void)
{
  int j, i;
  char string[100];
  double max_range=0;
  int ans;

  
  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Goto task can only be run if no other task is running!\n");
    return FALSE;
  }
  
  /* ask user where to go */
  
  if (!special_posture_flag) { 
    
    printf("\n");
    
    for (i=1; i<=n_dofs; ++i) {
      
      sprintf(string,"%s: target theta",joint_names[i]);
      joint_goto_state[i].th = joint_des_state[i].th;
      while (TRUE)  {
	if (!get_double(string,joint_goto_state[i].th,&joint_goto_state[i].th))
	  return FALSE;
	if (joint_goto_state[i].th > joint_range[i][MAX_THETA] ||
	    joint_goto_state[i].th < joint_range[i][MIN_THETA]) {
	  printf("Joint limits are from %5.3f to %5.3f\n",
		 joint_range[i][MIN_THETA],
		 joint_range[i][MAX_THETA]);
	  joint_goto_state[i].th = joint_des_state[i].th;
	  beep(1);
	} else {
	  break;
	}
      }
      
      if (fabs(joint_goto_state[i].th - joint_des_state[i].th) > max_range)
	max_range = fabs(joint_goto_state[i].th - joint_des_state[i].th);
      
      sprintf(string,"%s: target uff",joint_names[i]);
      joint_goto_state[i].uff = joint_des_state[i].uff;
      while (TRUE)  {
	if (!get_double(string,joint_goto_state[i].uff,
			&joint_goto_state[i].uff))
	  return FALSE;
	if (fabs(joint_goto_state[i].uff) > u_max[i]) {
	  printf("Too large torque, max = %f!\n",u_max[i]);
	  joint_goto_state[i].uff = joint_des_state[i].uff;
	  beep(1);
	} else {
	  break;
	}
      }
      
      joint_goto_state[i].thd = 0;
      
    }
    
  } else {  /* the special posture flag was set */
    
    max_range = 0;
    for (i=1; i<=n_dofs; ++i) {
      joint_goto_state[i] = joint_special_state[i];
      if (fabs(joint_goto_state[i].th - joint_des_state[i].th) > max_range)
	max_range = fabs(joint_goto_state[i].th - joint_des_state[i].th);
    }
    check_range(joint_goto_state);

  }

  /* ensure that the goal state has zero vel and zero acc */
  for (i=1; i<=n_dofs; ++i) 
    joint_goto_state[i].thd = joint_goto_state[i].thdd = 0.0;


  n_steps = 0;
  n_goto_steps = max_range/goto_speed*task_servo_rate;
  if (n_goto_steps == 0) {
    for (i=1; i<=n_dofs; ++i) {
      if (fabs(joint_goto_state[i].uff - joint_des_state[i].uff) != 0)
	n_goto_steps = 2*task_servo_rate; // this is two second for just ramping ff commands
    }
    if (n_goto_steps == 0) {
      n_goto_steps = task_servo_rate/10; // if no movement at all, just do 0.1 second movement
    } 
  }
  
  printf("Goto Task: steps = %d, time = %f\n",
	 n_goto_steps,n_goto_steps/(double)task_servo_rate);

  for (i=1; i<=n_dofs; ++i) {
    joint_increment[i].th = (joint_goto_state[i].th - joint_des_state[i].th)/
      (double) n_goto_steps;
    joint_increment[i].uff = (joint_goto_state[i].uff - joint_des_state[i].uff)/
      (double) n_goto_steps;
    joint_increment[i].thd = 0;
  }

  if (!special_posture_flag) {
    ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
	return FALSE;
    }
    
    if (ans != 1) 
      return FALSE;
    
  } else {
    
    special_posture_flag = FALSE;
    
  }

  return TRUE;

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
run_goto_task(void)
{
  int j, i;
  double time_to_go;

  time_to_go = (n_goto_steps - n_steps)/((double) task_servo_rate);

  // kinematics follow min merk
  if (!calculate_min_jerk_next_step (joint_des_state, joint_goto_state,time_to_go)) {
    setTaskByName(NO_TASK);
    printf("This should never happen\n");
    return TRUE;
  }

  // uff is just ramped to the desired value
  for (i=1; i<=n_dofs; ++i)
    joint_des_state[i].uff  += joint_increment[i].uff;

  if (++n_steps >= n_goto_steps) 
    setTaskByName(NO_TASK);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_goto_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_goto_task(void)
{
  int j, i;

  get_double("Enter new goto speed",goto_speed,&goto_speed);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go
\date  
   
\remarks 

       a point to point movement

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     jID :  the joint ID

 ******************************************************************************/
static void
sim_go(void)  /* for simulation only */
{
  static int jID=0;
  
  if (!get_int("Which joint? (0=all)",jID,&jID))
    return;
  go(jID);
  
}

int
go(int jID)
{

  int j;
  char string[100];

  if (jID >=1 && jID <=n_dofs) {

    for (j=1; j<=n_dofs; ++j)
      joint_special_state[j] = joint_des_state[j];
    
    sprintf(string,"%s: target theta",joint_names[jID]);
    while (TRUE)  {
      if (!get_double(string,joint_special_state[jID].th,
		      &joint_special_state[jID].th))
	return FALSE;
      if (joint_special_state[jID].th > joint_range[jID][MAX_THETA] ||
	  joint_special_state[jID].th < joint_range[jID][MIN_THETA]) {
	printf("Joint limits are from %5.3f to %5.3f\n",
	       joint_range[jID][MIN_THETA],
	       joint_range[jID][MAX_THETA]);
	joint_special_state[jID].th = joint_des_state[jID].th;
	beep(1);
      } else {
	break;
      }
    }
    
    sprintf(string,"%s: target uff",joint_names[jID]);
    joint_special_state[jID].uff = joint_des_state[jID].uff;
    while (TRUE)  {
      if (!get_double(string,joint_special_state[jID].uff,
		      &joint_special_state[jID].uff))
	return FALSE;
      if (fabs(joint_special_state[jID].uff) > u_max[jID]) {
	printf("Too large torque, max = %f!\n",u_max[jID]);
	joint_special_state[jID].uff = joint_des_state[jID].uff;
	beep(1);
      } else {
	break;
      }
    }

    special_posture_flag = TRUE;
    
  } 

  return setTaskByName("Goto Task");

}

/*!*****************************************************************************
 *******************************************************************************
\note  go0
\date  
   
\remarks 

       a point to point movement to the default posture

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
go0(void)
{
  int i;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = joint_default_state[i];

  setTaskByName("Goto Task");

  return;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go0_wait
\date  
   
\remarks 

       go to the default posture, and returns only after default
       posture has been reached

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
int
go0_wait(void)
{

  int i;
  double last_time;
  double last_draw_time;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = joint_default_state[i];
  
  if (!setTaskByName("Goto Task")) {
    special_posture_flag = FALSE;
    return FALSE;
  }

  last_time = last_draw_time = task_servo_time;
  while (strcmp(current_task_name,NO_TASK) != 0) {
    if (task_servo_time - last_time > 5) {
      special_posture_flag = FALSE;
      printf("time out in go0_wait\n");
      return FALSE;
    }
    taskDelay(ns2ticks(10000000)); // wait 10ms
  }
  
  return TRUE;

}



/*!*****************************************************************************
 *******************************************************************************
\note  go_target_wait
\date  
   
\remarks 

       go to the special posture, and returns only after 
       posture has been reached

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     target : the array of target states

 ******************************************************************************/
int
go_target_wait(SL_DJstate *target)
{

  int i;
  double last_time;
  double last_draw_time;

  special_posture_flag = TRUE;

  for (i=1; i<=n_dofs; ++i)
    joint_special_state[i] = target[i];

  if (!setTaskByName("Goto Task")) {
    special_posture_flag = FALSE;
    return FALSE;
  }

  last_time = last_draw_time = task_servo_time;
  while (strcmp(current_task_name,NO_TASK) != 0) {
    if (task_servo_time - last_time > 5) {
      special_posture_flag = FALSE;
      printf("time out in go_target_wait\n");
      return FALSE;
    }
    taskDelay(ns2ticks(10000000)); // wait 10ms
  }
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  go_target_wait_ID
\date  
   
\remarks 

       go to the special posture, and returns only after 
       posture has been reached -- uses inverse dynamics to
       compute the uff for the final posture

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     target : the array of target states

 ******************************************************************************/
int
go_target_wait_ID(SL_DJstate *target)
{

  int i;
  double last_time;

  /* compute inverse dynamics feedforward command */
  for (i=1; i<=n_dofs; ++i) {
    target[i].thd  = 0.0;
    target[i].thdd = 0.0;
  }

  SL_InvDyn(NULL,target,endeff,&base_state,&base_orient);


   return go_target_wait(target);
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  calculate_min_jerk_next_step
\date  August 1994
   
\remarks 

Given the time to go, the desired state is updated according to the min jerk
formula.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out]      state : the current state
 \param[in]           goal : the desired state
 \param[in]            tau : the desired movement duration until the goal is

 ******************************************************************************/
static int 
calculate_min_jerk_next_step (SL_DJstate *state, SL_DJstate *goal,double tau)

{
  double t1,t2,t3,t4,t5;
  double tau1,tau2,tau3,tau4,tau5;
  int    i,j;
  double delta_t;

  delta_t = 1./((double)task_servo_rate);

  if (delta_t > tau || delta_t <= 0) {
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

  for (j=1; j<=n_dofs; ++j) {
	
    /* calculate the constants */
    const double dist   = goal[j].th - state[j].th;
    const double p1     = goal[j].th;
    const double p0     = state[j].th;
    const double a1t2   = goal[j].thdd;
    const double a0t2   = state[j].thdd;
    const double v1t1   = goal[j].thd;
    const double v0t1   = state[j].thd;
    
    const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 
      3.*(v0t1 + v1t1)/tau4;
    const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
      (8.*v0t1 + 7.*v1t1)/tau3; 
    const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
      (6.*v0t1 + 4.*v1t1)/tau2; 
    const double c4 = state[j].thdd/2.;
    const double c5 = state[j].thd;
    const double c6 = state[j].th;
    
    state[j].th   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
    state[j].thd  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
    state[j].thdd = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
    
  }
  
  return TRUE;
}

