/*!=============================================================================
  ==============================================================================

  \file    SL_tasks.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  the header file for the SL_task.c

  ============================================================================*/

#ifndef _SL_tasks_
#define _SL_tasks_

#define NO_TASK "No Task"

typedef struct Task_Def {

  char  name[100];                 /*!< the name of the task */
  int   active;                    /*!< TRUE/FALSE */
  int   (*init_function)(void);    /*!< the initialization function */
  int   (*run_function)(void);     /*!< the execution function */
  int   (*change_function)(void);  /*!< the function to change the task parameters */
  char *next_task;                 /*!< pointer to the next task structure */

} Task_Def;

#ifdef __cplusplus
extern "C" {
#endif

extern char current_task_name[100];

/* functions */
void setTask(void);
void st(void);
void addTask(const char *tname, int (*init_function)(void), int (*run_function)(void),
	     int (*change_function)(void));
void deleteTask(char *tname);
void initTasks(void);
void changeTaskParm(void);
void ctp(void);
int  setTaskByName(char *name);
void runTask(void);
void redo(void);

#ifdef __cplusplus
}
#endif

#endif // _SL_tasks_
