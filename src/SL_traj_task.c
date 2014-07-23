/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_traj_task.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  the standard trajectory following task

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
static SL_DJstate *target;
static double     task_time;
static fMatrix    traj_pos=NULL; 
static fMatrix    traj_vel=NULL;
static fMatrix    traj_acc=NULL;
static fMatrix    traj_uff=NULL;
static int        *column_map;
static int        n_rows=0,n_cols=0, n_vars=0;
static double     sampling_freq;
static int        traj_index;
static int        repeat_flag;
static int        invdyn_flag=FALSE;

/* global variables */

/* global functions */

void add_traj_task(void);

/* local functions */
static int init_traj_task(void);
static int run_traj_task(void);
static int change_traj_task(void);
static int read_traj_file(int flag);
static int check_traj_range(void);
 
/*!*****************************************************************************
 *******************************************************************************
\note  add_traj_task
\date  June 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_traj_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;
    
    target     = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate), MY_STOP);
    column_map = my_ivector(1,n_dofs);
    
    addTask("Traj Task", init_traj_task, 
	    run_traj_task, change_traj_task);
  }
  
}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_traj_task
  \date  Jun. 1999

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_traj_task(void)
{
  int j, i, c;
  char string[100];
  double max_range=0;
  static int ans = 0;

  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("New task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* use desired or real kinematic data? */
  get_int("Extract from DESIRED data? Yes=1 No=0",ans,&ans);

  /* read the script for this task */
  if (!read_traj_file(ans))
    return FALSE;

  if (!check_traj_range())
    return FALSE;
  
  /* check for repeatable task */ 
  repeat_flag = TRUE;
  for (i=1;i<=n_vars;++i){
    if((traj_pos[1][i]==traj_pos[n_rows][i]) && 
       (traj_vel[1][i]==traj_vel[n_rows][i]) && 
       (traj_uff[1][i]==traj_uff[n_rows][i])){
      ;
    }else 
      repeat_flag = FALSE;
  }
  /* do we want to repeat task */
  if (repeat_flag){
    ans = 1;
    get_int("Do you want this task to repeat itself? 1=y,0=n",ans,&ans);
    if (ans!=1)
      repeat_flag = FALSE;
  }

  /* enable inverse dynamics control */
  get_int("Use inverse dynamics",invdyn_flag,&invdyn_flag);

  /* go to start posture */
  traj_index = 1;
  bzero((char *)&(target[1]),n_dofs*sizeof(target[1]));
  for (i=1; i<=n_dofs; ++i) {
    target[i] = joint_default_state[i];
    if (column_map[i] != 0)
      target[i].th = traj_pos[ traj_index ][ column_map[i] ];
  }

  if (invdyn_flag) {
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
  \note  run_traj_task
  \date  Jun. 1999

  \remarks 

  run the task from the task servo: REAL TIME requirements!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
run_traj_task(void)
{
  int j, i;
  double time_of_traj_index;
  double time_of_next_traj_index;
    
  task_time += 1./(double)task_servo_rate;
  
  time_of_traj_index      = ((double)(traj_index-1))/sampling_freq;
  time_of_next_traj_index = ((double)(traj_index))/sampling_freq;

  /* the statement below takes care of numerical round problems */
  while (task_time - time_of_next_traj_index >= -0.00001) {
    if (++traj_index >= n_rows) {
      if(repeat_flag){
	traj_index=1;
	task_time = 1./(double)task_servo_rate;
      } else{
	setTaskByName(NO_TASK);
	return TRUE;
      }
    }
    time_of_traj_index      = (double)(traj_index-1)/sampling_freq;
    time_of_next_traj_index = (double)(traj_index)/sampling_freq;
  } 
							  
  for (i=1; i<=n_dofs; ++i) {
    if (column_map[i] != 0) {

      joint_des_state[i].th  = traj_pos[ traj_index ][ column_map[i] ] +
	(traj_pos[ traj_index+1 ][ column_map[i] ]-traj_pos[ traj_index][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);

      joint_des_state[i].thd = traj_vel[ traj_index ][ column_map[i] ] +
	(traj_vel[ traj_index+1 ][ column_map[i]]-traj_vel[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);

      joint_des_state[i].thdd = traj_acc[ traj_index ][ column_map[i] ] +
	(traj_acc[ traj_index+1 ][ column_map[i]]-traj_acc[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);

      joint_des_state[i].uff  = traj_uff[ traj_index ][ column_map[i] ] +
	(traj_uff[ traj_index+1 ][ column_map[i]]-traj_uff[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);

    }
  }

  if (invdyn_flag)
    SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
 
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_traj_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_traj_task(void)
{
  int j, i;
  char string[100];


  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  read_traj_file
  \date  June 1999

  \remarks 

  parse a script which describes the traj task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     flag : true= use desired data, false use actual data

 ******************************************************************************/
static int 
read_traj_file(int flag)
{
  int j,i,r,k,rc;
  static char string[100];
  static char fname[100] = "test.traj";
  static char current_fname[100] = "";
  FILE * fp=NULL;
  int    found = FALSE;
  char **vnames;
  char **units;
  int    buffer_size;
  fMatrix buff;
  int    aux;
  int    ans=0;

  /* open the file, and parse the parameters */

  while (TRUE) {
    if (!get_string("Name of the Traj File\0",fname,fname)) 
      return FALSE;

    if (strcmp(fname,current_fname) ==0) {
      get_int("Re-read the trajectory file?",ans,&ans);
    }
    
    if (strcmp(fname,current_fname) !=0 || ans) {
      /* try to read this file */
      sprintf(string,"%s%s",PREFS,fname);
      fp = fopen(string,"r");
      if (fp != NULL) {
        break;
      } else {
        printf("ERROR: Could not open file >%s<\n",string);
      }
    } else {
      found = TRUE;
      break;
    }

  }

  if (strcmp(fname,current_fname) !=0 || ans) {
    
    /* clear the current parameters */
    if (traj_pos != NULL) {
      my_free_fmatrix(traj_pos,1,n_rows,1,n_vars);
      traj_pos = NULL;
    }
    if (traj_vel != NULL) {
      my_free_fmatrix(traj_vel,1,n_rows,1,n_vars);
      traj_vel = NULL;
    } 
    if (traj_acc != NULL) {
      my_free_fmatrix(traj_acc,1,n_rows,1,n_vars);
      traj_acc = NULL;
    }
    if (traj_uff != NULL) {
      my_free_fmatrix(traj_uff,1,n_rows,1,n_vars);
      traj_uff = NULL;
    }
    
    /* get the number of rows, columns, sampling frequency
       and calc the bufer_size */
    rc=fscanf(fp,"%d %d %d %lf",&buffer_size,&n_cols,&n_rows,&sampling_freq);
    
    /* alocate memory for the variable names and units
       use MY_STOP for checking for errors in allocation */
    vnames = (char **)my_calloc(n_cols+1, sizeof(char *), MY_STOP);
    units = (char **)my_calloc(n_cols+1, sizeof(char *), MY_STOP);
    
    for (i=1;i<=n_cols;++i){
      vnames[i] = (char *) my_calloc(40, sizeof(char), MY_STOP);
      units[i] = (char *) my_calloc(40, sizeof(char), MY_STOP);
      rc=fscanf(fp, "%s %s", vnames[i], units[i]);
    }
    
    /* there are two extra blank chrs at the end of the block
       and a line return which we must account for */
    fgetc(fp);
    fgetc(fp);
    fgetc(fp);
    
    /* read file into a buffer and check if the matrix size is correct */  
    buff = my_fmatrix(1,n_rows,1,n_cols);
    
    if (fread(&buff[1][1],sizeof(float),n_rows*n_cols,fp)!= n_rows*n_cols){
      printf("cannot fread matrix. \n");
      return MY_ERROR;
    }
    
    fclose(fp);

#ifdef BYTESWAP
  /* convert little-endian to big-endian */ 
  for (j=1; j<=n_cols; ++j) {
    for (i=1; i<=n_rows; ++i) {
      aux = LONGSWAP(*((int *)&(buff[i][j])));
      buff[i][j] = *((float *)&aux);
    }
  }
#endif
    
    /* initialize vars (number of joint names used in file)  
       and look for  the joint names present in file*/
    n_vars = 0;
    
    for (i=1;i<=n_dofs;++i) {
      if (flag)
	sprintf(string, "%s_des_th",joint_names[i]);
      else
	sprintf(string, "%s_th",joint_names[i]);
      
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  ++n_vars;
	  break;
	}
      }
    }
    
    /* create the pos, vel, acc , uff matrices that define the trajectory */
    traj_pos = my_fmatrix(1,n_rows,1,n_vars);
    traj_vel = my_fmatrix(1,n_rows,1,n_vars);
    traj_acc = my_fmatrix(1,n_rows,1,n_vars);
    traj_uff = my_fmatrix(1,n_rows,1,n_vars);
    
    /* initialize column_map , routine to fill pos matrix */
    n_vars = 0;
    for (i=1;i<=n_dofs;++i) {
      if (flag)
	sprintf(string, "%s_des_th",joint_names[i]);
      else
	sprintf(string, "%s_th",joint_names[i]);
      column_map[i] = 0;
      
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  ++n_vars;
	  
	  found = TRUE;
	  
	  /*map the used column number into the column map with N_DOF+1 columns*/
	  column_map[i] = n_vars;
	  
	  /* fill the pos matrix using the right column from buffer*/
	  for (r=1;r<=n_rows;++r){
	    traj_pos[r][n_vars] = buff[r][j];
	  }
	  
	  /* also check for velocity, acceleration, and uff information
	     use the same value of n-vars to fill the remaining matrices*/
	  
	  if (flag)
	    sprintf(string, "%s_des_thd",joint_names[i]);
	  else
	    sprintf(string, "%s_thd",joint_names[i]);

	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_vel[r][n_vars] = buff[r][k];
	      }
	    }
	  }
	  
	  if (flag)
	    sprintf(string, "%s_des_thdd",joint_names[i]);
	  else
	    sprintf(string, "%s_thdd",joint_names[i]);

	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_acc[r][n_vars] = buff[r][k];
	      }
	    }
	  }
	  
	  sprintf(string, "%s_uff",joint_names[i]);
	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_uff[r][n_vars] = buff[r][k];
	      }
	    }
	  }
	  /* assume only one variable of each kind exists*/
	  break;
	}
      }
    }
    
    /* free up memory by deallocating resources */
    my_free_fmatrix (buff,1,n_rows,1,n_cols);
    for (i=1;i<=n_cols;++i){
      free(vnames[i] ); 
      free(units[i]) ;
    }
    
    free(units);
    free(vnames);

  }

  /* keep track fo the current trajectory file to speed up reading */
  strcpy(current_fname,fname);

  return found;

}
/*!*****************************************************************************
 *******************************************************************************
  \note  check_traj_range
  \date  June 1999

  \remarks 

  the given traj lies within joint limits or not

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
check_traj_range(void)
{
  int i,j;
  int idno;
  
  for(i=1;i<=n_dofs;++i){
    if (column_map[i] != 0) {
      idno = column_map[i];
      for(j=1;j<=n_rows;++j){
	if ((traj_pos[j][idno] > joint_range[i][MAX_THETA]) ||
	    ( traj_pos[j][idno]< joint_range[i][MIN_THETA])){
	  printf("Joint Angle Limits Exceeded in joint %s (is %1.3f, should be in range [%1.3f-%1.3f]) at time %f (tick %d)",joint_names[i],traj_pos[j][idno],joint_range[i][MIN_THETA],joint_range[i][MAX_THETA],((double) j)/sampling_freq,j);
	  return FALSE;
	}
	if ((traj_uff[j][idno] > u_max[i]) ||
	    ( traj_uff[j][idno]< -u_max[i])){
	  printf("Feedforward Commands Exceeded in %s a(is %1.3f, should be in range [%1.3f-%1.3f]) at time %f (tick %d)",joint_names[i],traj_uff[j][idno],-u_max[i],u_max[i],((double) j)/sampling_freq,j);
	  return FALSE;
	}
      }
    }
  }

  return TRUE;
}








