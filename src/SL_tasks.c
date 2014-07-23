/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_tasks.c

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  This function manages different tasks, selectable through the
  tasks menu. In VxWorks, these task can be linked dynamically by just
  calling the addTask function.

  ============================================================================*/

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "utility.h"

#include "SL_tasks.h"
#include "SL.h"
#include "SL_task_servo.h"
#include "SL_man.h"

char current_task_name[100];

static Task_Def tasks;
static int (*current_init_function)  (void) = NULL;
static int (*current_run_function)   (void) = NULL;
static int (*current_change_function)(void) = NULL;
static int  tasks_initialized = FALSE;
static char last_task_name[100] = NO_TASK;
static int (*last_change_function)(void) = NULL;


/* local functions */

static int  initNoTask(void);
static int  runNoTask(void);

/*!*****************************************************************************
 *******************************************************************************
\note  st & set_task
\date  April 1999
   
\remarks 

        determines the current task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
st(void) 
{
  setTask();
}
void
setTask(void) 

{

  int i,j;
  Task_Def *tptr;
  int aux=0;


  if (!servo_enabled) {
    printf("WARNING: Servo is not running! -- Aborted!\n");
    beep(1);
    return;
  }
	
  if (strcmp(current_task_name,NO_TASK)!=0) {
    printf("WARNING: Another Task is still running! -- Aborted!\n");
    beep(1);
    return;
  }
	
  if (!tasks_initialized) 
    initTasks();

  /* generate the task menu */
  
AGAIN:
  printf("\n\n\nChoose Task of Robot:\n\n");
  
  tptr = &tasks;
  i=-1;
  while (tptr != NULL) {
    if (tptr->active) {
      printf("        %32s ---> %d\n",tptr->name,++i);
      if (strcmp(tptr->name,last_task_name)==0)
      	aux = i;
    }
    tptr = (Task_Def *) tptr->next_task;
  }

  printf("\n");
  if (!get_int("        ----> Input",aux,&aux)) {
    setTaskByName(NO_TASK);
    return;
  }

  if (aux < 0 || aux > i) {
    printf("\nERROR: Invalid selection\n");
    goto AGAIN;
  }

  if (aux == 0)  /* the NO_TASK */
    return;

  tptr = &tasks;
  for (j=0; j<aux; ++j) {
    if (tptr->active == FALSE)
      --j;
    tptr = (Task_Def *) tptr->next_task;
  }

  /* and execute the initialization of the task */
	
  if (!(*tptr->init_function)()) {
    goto AGAIN;
  }

  current_init_function    = tptr->init_function;
  current_run_function     = tptr->run_function;
  current_change_function  = tptr->change_function;
  strcpy(current_task_name, tptr->name);
  strcpy(last_task_name, tptr->name);
  last_change_function = tptr->change_function;
	
}

/*!*****************************************************************************
 *******************************************************************************
\note  redo
\date  April 1999
   
\remarks 

        executes the last task again

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
redo(void) 

{

  int i,j;
  Task_Def *tptr;
  int aux=0;


  if (!servo_enabled) {
    printf("WARNING: Servo is not running! -- Aborted!\n");
    beep(1);
    return;
  }
	
  if (strcmp(current_task_name,NO_TASK)!=0) {
    printf("WARNING: Another Task is still running! -- Aborted!\n");
    beep(1);
    return;
  }
	
  /* and execute the initialization of the task */
	
  setTaskByName(last_task_name);
	
}

/*!*****************************************************************************
 *******************************************************************************
\note  addTask
\date  April 1999
   
\remarks 

adds a new task to the task structures, and sets it to active, by default

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     tname             : character string containing the name of task
 \param[in]     init_function     : function pointer to initialization routine for task
 \param[in]     run_function      : function pointer to run routine for task (real-time)
 \param[in]     change_function   : function pointer to routine to change the task parameters

 ******************************************************************************/
void
addTask(const char *tname, int (*init_function)(void), int (*run_function)(void),
	int (*change_function)(void))

{
  Task_Def *tptr;
  int overwrite_flag = FALSE;

  if (!tasks_initialized) 
    initTasks();

  if (init_function == NULL) {
    printf("Initialization function cannot be the null pointer\n");
    return;
  }

  if (run_function == NULL) {
    printf("Run function cannot be the null pointer\n");
    return;
  }

  tptr = &tasks;
  do {
    if (strcmp(tname,tptr->name)==0) {
      printf("Task >%s< was re-initialized\n",tname);
      overwrite_flag = TRUE;
      break;
    }
    if (tptr->next_task == NULL)
      break;
    else 
      tptr = (Task_Def *) tptr->next_task;
  } while (TRUE);
  
  if (!overwrite_flag) {
    tptr->next_task = my_calloc(1,sizeof(Task_Def),MY_STOP);
    tptr = (Task_Def *) tptr->next_task;
    strcpy(tptr->name,tname);
  }
  tptr->init_function    = init_function;
  tptr->run_function     = run_function;
  tptr->change_function  = change_function;
  tptr->active           = TRUE;
  if (!overwrite_flag)
    tptr->next_task      = NULL;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  deleteTask
\date  April 1999
   
\remarks 

deletes a task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     tname             : character string containing the name of task

 ******************************************************************************/
void
deleteTask(char *tname)
     
{
  Task_Def *tptr,*stptr;
  
  if (!tasks_initialized) 
    initTasks();
  
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("You can only delete task in No Task mode\n");
  }
  
  if (strcmp(tname,NO_TASK) == 0) {
    printf("The No Task cannot be deleted\n");
  }
  
  tptr  = &tasks;
  stptr = NULL;
  
  while (strcmp(tptr->name,tname) != 0) {
    if (tptr->next_task == NULL)
      return;
    else {
      stptr = tptr;
      tptr = (Task_Def *) tptr->next_task;
    }
  }
  
  stptr->next_task = tptr->next_task;
  free((void *) (tptr));
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  initTasks
\date  April 1999
   
\remarks 

initializes the task servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     tname             : character string containing the name of task

 ******************************************************************************/
void
initTasks(void)
     
{
  int i;

  if (tasks_initialized) 
    return;

  /* assign the No Task as the first task to the task structures */

  strcpy(tasks.name,NO_TASK);
  tasks.active = TRUE;
  tasks.init_function  = initNoTask;
  tasks.run_function   = runNoTask;
  tasks.change_function = NULL; 
  tasks.next_task = NULL;

  /* add to man pages */
  addToMan("setTask","changes the current task of robot",setTask);
  addToMan("st","short for setTask",st);
  addToMan("redo","repeats the last task",redo);
  addToMan("changeTaskParm","allows changing of parameters of the last(current) task",
	   changeTaskParm);
  addToMan("ctp","short for changeTaskParm",ctp);

  tasks_initialized = TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  noTask functions
\date  April 1999
   
\remarks 

function for the noTask

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

void

 ******************************************************************************/
static int
initNoTask(void)

{

  int i;

  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].thd = 0;
    joint_des_state[i].thdd = 0;
    joint_des_state[i].uex = 0;
  }

  return TRUE;

}

static int
runNoTask(void)

{

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  taskServo
\date  April 1999
   
\remarks 

just executes the current task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
runTask(void) 
     
{
  
  if (!tasks_initialized) 
    initTasks();
  
  if (current_run_function != NULL)
    (*current_run_function)();
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  setTaskByName
\date  April 1999
   
\remarks 

just executes the current task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
setTaskByName(char *name)

{
  
  int i,j;
  Task_Def *tptr;
  int aux=0;
	
  if (!tasks_initialized) 
    initTasks();

  tptr = &tasks;
  while (tptr != NULL) {

    if (tptr->active && strcmp(name,tptr->name)==0) {

      /* execute the initialization of the task */
	
      if (!(*tptr->init_function)()) {
	return FALSE;
      }
      
      current_init_function    = tptr->init_function;
      current_run_function     = tptr->run_function;
      current_change_function  = tptr->change_function;
      strcpy(current_task_name, tptr->name);
      if (strcmp(name,NO_TASK) != 0)
	last_change_function     = tptr->change_function;

      return TRUE;
    }
    
    tptr = (Task_Def *) tptr->next_task;
  }

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  changeTaskParm
\date  April 1999
   
\remarks 

executes the change routine for the current task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
ctp(void)
{
  changeTaskParm();
}
void
changeTaskParm(void) 
     
{
  
  if (!tasks_initialized) 
    initTasks();
  
  if (current_change_function != NULL) {
    (*current_change_function)();
    return;
  }
  
  if (last_change_function != NULL)
    (*last_change_function)();
  
}
