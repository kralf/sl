/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_vx2xeno_wrappers.c

  \author  Stefan Schaal
  \date    Oct. 2009

  ==============================================================================
  \remarks

  replacement functions and  definition of functions of the real-time 
  operating system vxWorks adjusted for xenomai real-time OS

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

#include "utility.h"
#include "time.h"
#include "sys/types.h"
#include "sys/ipc.h"
#include "sys/shm.h"
#include "sys/sem.h"
#include "sys/time.h"
#include "errno.h"
#include "SL.h"
#include "SL_shared_memory.h"

// needed for book keeping of share memory
typedef struct smlist /*!< structure to maintain shared memory list */
{
  void   *smptr;      /*!< pointer to shared memory value */
  char    name[100];  /*!< shared memory name */
  int     type;       /*!< type of shared memory */
  RT_HEAP heap;       /*!< heap structure */
  RT_SEM  sem;        /*!< semaphore structure */
  char   *nptr;       /*!< pointer to next shared memory */
} SMLIST, *SM_PTR;

//! list of all allocated shared memory objects
#define MAX_SM
static int sm_ids_list[1000];

// semaphore linked list 
SM_PTR   smlist = NULL;

//! user function to be called on exit
static void (*user_signal_handler)(void) = NULL;  //!< function pointer

// local functions
static STATUS       
semFindNameByID (SEM_ID sem_id, char *name);

int	
logMsg (char *fmt, int arg1, int arg2,
	int arg3, int arg4, int arg5, int arg6)
{
  return rt_printf(fmt,arg1,arg2,arg3,arg4,arg5,arg6);
}

void * 
smObjLocalToGlobal(void *  localAdrs)
{
  return localAdrs;
}

void * 
smObjGlobalToLocal(void *  globalAdrs)
{
  return globalAdrs;
}

STATUS
taskDelay(int num)
{
  RT_TIMER_INFO info;

  // make sure the delay is at least one period long
  rt_timer_inquire(&info);

  if (info.period == TM_ONESHOT)
    ;
  else if (info.period == 0) {
    printf("Error: Timer period is 0!\n");
    return ERROR;
  }

  if (num > 0)
    rt_task_sleep((RTIME) num);

  return OK;
}

unsigned long
tickGet(void)
{
  return rt_timer_read();
}


/*!*****************************************************************************
 *******************************************************************************
 \note  smMemCalloc
 \date  Nov 2007
 
 \remarks 

 A posix replacement for the vxWorks function smMemCalloc 
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     shmname         : name of the shared memory
 \param[in]     id              : id to make shared memory identifier unique
 \param[in]     elemNum         : number of elements to allocate
 \param[in]     elemSize        : memory size of an element as determined with sizeof()
 
 ******************************************************************************/
static char mist[100000];
void *     
smMemCalloc (char *shmname, int id, int elemNum, int elemSize) 
{
  int      rc;
  char     name[100];
  RT_HEAP  heap;
  void    *ptr;
  STATUS   error;
  int      found_object = FALSE;
  
  // get a shared memory heap with a unique name, but check whether it already
  // exists
  sprintf(name,"%s.%d",shmname,id);
  rc = rt_heap_bind(&heap,name,TM_NONBLOCK);
  if (rc) {
    found_object = FALSE;
    rc = rt_heap_create(&heap,name,(size_t)(elemNum*elemSize),H_SHARED);	
    if (rc) {
      printf("Error: rt_heap_create returned %d\n",rc);
      return NULL;
    }
  } else 
    found_object = TRUE;

  // get the address of the shared memory segment
  rc = rt_heap_alloc(&heap,(size_t)(elemNum*elemSize),TM_NONBLOCK,&ptr);
  if (rc) {
    printf("Error: rt_heap_alloc returned %d\n",rc);
    return NULL;
  }

  // wipe the memory clean, but only if it is newly allocated
  if (!found_object)
    bzero(ptr,(size_t)(elemNum*elemSize));
  
  // add the shared memory to list of shared memory objects
  error = smNameAdd (shmname, &ptr, T_SM_PART_ID, &heap, NULL);
  if (error == ERROR) {
    printf("Unexpected error in shared memory management\n");
    return NULL;
  }

  return ptr;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  smNameFind
 \date  Nov 2007
 
 \remarks 

 A replacement for the vxWorks function smNameFind, althought this searcheds
 only the local shared memory structure, and not all shared memory in the
 kernel heap. But the smMemCalloc takes care of this issues by checking for
 existing names before allocating shared memory. This function here is not
 really useful anymore.
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     name            : name of the shared memory
 \param[out]    pValue          : pointer to shared memory
 \param[out]    pType           : shared memory type
 \param[in]     waitType        : what wait actions to perform 
 
 ******************************************************************************/
STATUS       
smNameFind (char *name, void **pValue, int *pType, int waitType)
{

  SM_PTR sptr;

  sptr = smlist;

  while (sptr!=NULL) {
    if (strcmp(sptr->name,name)==0) {
      *pValue = (void *)sptr->smptr;
      *pType  = sptr->type;
      return OK;
    }
    sptr = (SM_PTR) sptr->nptr;
  }
  return ERROR;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semFindNameByID
 \date  Nov 2007
 
 \remarks 

 A handy function to extract the name of a semaphore from the local 
 house keeping using the sem_id as identifier
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     sem_id          : identifier of semaphore
 \param[out]    name            : name
 
 ******************************************************************************/
static STATUS       
semFindNameByID (SEM_ID sem_id, char *name)
{

  SM_PTR sptr;

  sptr = smlist;

  while (sptr!=NULL) {
    if (&(sptr->sem)==sem_id) {
      strcpy(name,sptr->name);
      return OK;
    }
    sptr = (SM_PTR) sptr->nptr;
  }

  strcpy(name,"not found");
  return ERROR;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  smNameAdd
 \date  Nov 2007
 
 \remarks 

 A replacement for the vxWorks function smNameAdd
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     name            : name of the shared memory
 \param[in,out] value           : pointer to shared memory [in],
                                  for a semaphore, we return the pointer to 
				  semaphore structure [out]
 \param[in]     pType           : shared memory type
 \param[in]     heap            : pointer to heap structure
 \param[in]     sum             : pointer to semaphore structure
 
 ******************************************************************************/
STATUS       
smNameAdd (char * name, void ** value, int pType, RT_HEAP *heap, RT_SEM *sem)
{
  SM_PTR *shandle;
  void *pValue;

  // does the name already exist? 
  if (smNameFind(name,&pValue, &pType, NO_WAIT) == OK)
    return ERROR;

  shandle = &smlist;

  while(*shandle != NULL) {
    shandle = (SM_PTR *)&((*shandle)->nptr);
  }

  *shandle = my_calloc(1,sizeof(SMLIST),MY_STOP);
  (*shandle)->type = pType;
  (*shandle)->nptr = NULL;
  if (heap != NULL) {
    (*shandle)->heap = *heap;
    (*shandle)->smptr= *value;
  }
  if (sem != NULL) {
    (*shandle)->sem = *sem;
    *value = (void *)&((*shandle)->sem);
    (*shandle)->smptr= *value;
  }
  strcpy((*shandle)->name,name);

  return OK;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semBSmCreate
 \date  Nov 2007
 
 \remarks 

 Create a binary shared memory semaphore as in vxWorks
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semname         : name of the shared memory semaphore
 \param[in]     id              : id to make shared memory identifier unique
 \param[in]     options         : options for semaphore
 \param[in]     inititalState   : initial state of semaphore
 
 ******************************************************************************/
SEM_ID       
semBSmCreate (char *semname, int id, int options, SEM_B_STATE initialState)
{
  RT_SEM     sem;
  char       name[100];
  RT_SEM    *sem_id;
  int        rc;
  STATUS     error;

  // get a shared memory heap with a unique name, but check whether it already
  // exists
  sprintf(name,"%s.%d",semname,id);
  rc = rt_sem_bind(&sem,name,TM_NONBLOCK);
  if (rc) {
    rc = rt_sem_create(&sem,name,initialState,S_FIFO);	
    if (rc) {
      printf("Error: rt_sem_create returned %d\n",rc);
      return (SEM_ID) (-1);
    }
  }

  // add the semaphore to list of shared memory objects
  error = smNameAdd (semname, (void **)&sem_id, T_SM_SEM_B, NULL, &sem);
  if (error == ERROR) {
    printf("Unexpected error in semaphore management\n");
    return (SEM_ID)(-1);
  }

  return sem_id;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semTake
 \date  Nov 2007
 
 \remarks 

 Take a binary semaphore
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semId           : id of semaphore
 \param[in]     timeout         : how many ticks to wait before returning ERROR
 
 ******************************************************************************/
STATUS 	
semTake (SEM_ID semId, int timeout)
{

  int rc;

  switch (timeout) {
    
  case WAIT_FOREVER:
    rc = rt_sem_p(semId,TM_INFINITE);
    break;
    
  case NO_WAIT:
    rc = rt_sem_p(semId,TM_NONBLOCK);
    break;
    
  default:
    rc = rt_sem_p(semId,(RTIME) timeout);
    break;
    
  }

  if (rc) {
    if (rc == -EWOULDBLOCK || rc == -ETIMEDOUT ) 
      return ERROR; // the return codes happen normally
    else { //other more servere return codes print a message
      // EINVAL=22 EIDRM=43 EWOULDBLOCK=11 EINTR=4 ETIMEDOUT=110 EPERM=1
      char name[100];
      semFindNameByID(semId,name);
      printf("Error in rt_sem_p (rc=%d, name=%s)\n",rc,name);
      return ERROR;
    }
  }

  return OK;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semGive
 \date  Nov 2007
 
 \remarks 

 Give back a binary semaphore
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semId           : id of semaphore
 
 ******************************************************************************/
STATUS 	
semGive (SEM_ID semId)
{

  RT_SEM_INFO info;
  int         rc;

  // only give the semaphore if empty -- this emulates a binary semaphore
  rc = rt_sem_p(semId,TM_NONBLOCK);
  if (rc == -EWOULDBLOCK) { // this is perfect and means the semaphore should be given
    rc = rt_sem_v(semId);
    if (rc) {
      printf("Error in rt_sem_v (rc=%d)\n",rc);
      return ERROR;
    }
  } else if (rc == 0) { // this means the semaphore was already available
    // check whether semaphore is empty before giving -- not an atomic operation
    rc = rt_sem_inquire(semId,&info);
    if (rc) {
      printf("Error in rt_sem_inquire (rc=%d)\n",rc);
      return ERROR;
    }
    if (info.count == 0) {
      rc = rt_sem_v(semId);
      if (rc) {
	printf("Error in rt_sem_v (rc=%d)\n",rc);
	return ERROR;
      }
    }
  } else {
    char name[100];
    semFindNameByID(semId,name);
    printf("Error in rt_sem_p (rc=%d, name=%s)\n",rc,name);
    return ERROR;
  }

  return OK;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semFlush
 \date  Nov 2007
 
 \remarks 

 unblock all tasks waiting for this semaphore
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semId           : id of semaphore
 
 ******************************************************************************/
STATUS 	
semFlush (SEM_ID semId)
{
  int         rc;

  rc = rt_sem_broadcast(semId);
  if (rc) {
    printf("Error in rt_sem_broadcast (rc=%d)\n",rc);
    return ERROR;
  }

  return OK;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  removeShareMemory
 \date  Nov 2007
 
 \remarks 

 a function to remove all share memory and semaphores, to be used by a
 signal handler
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     dummy : only needed to comply with signal()
 
 ******************************************************************************/
void
removeSharedMemoryAtExit(void)
{
  int dummy=-1;
  removeSharedMemory(dummy);
}
void
removeSharedMemory(int dummy)
{
  
  SM_PTR sptr;
  char   string[100];
  
  sptr = smlist;

  printf("\nRemove shared memory objects and semaphores (sig=%d, %s) ...",dummy,strsignal(dummy));

  // keep a log why the termination happened
  sprintf(string,".%s_log",servo_name);
  FILE *fd = fopen(string,"w");
  fprintf(fd,"Terminated due to sig=%d, %s\n",dummy,strsignal(dummy));  
  fclose(fd);
  
  while (sptr!=NULL) {
    
    switch (sptr->type) {
    case T_SM_SEM_B:
      rt_sem_delete(&(sptr->sem));
      break;
      
    case T_SM_PART_ID:
      rt_heap_free(&(sptr->heap),sptr->smptr);
      rt_heap_delete(&(sptr->heap));
      break;
      
    default:
      break;
      
    }
    
    sptr = (SM_PTR) sptr->nptr;
  }

  // execute user function
  if (user_signal_handler != NULL)
    (*user_signal_handler)();
  
  printf("done\n");
  
  exit(-1);
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  setUserSignalHandler
 \date  Nov 2007
 
 \remarks 

 adds a user defined function for signal handling
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]   fptr: function pointer to void func(void) function
 
 ******************************************************************************/
void
setUserSignalHandler(void(*fptr)(void))
{

  user_signal_handler = fptr;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  semGet
 \date  Nov 2007
 
 \remarks 

 returns the current value of a semaphore
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semId           : id of semaphore
 \param[out]    val             : value of semaphore
 
 ******************************************************************************/
STATUS 	
semGet (SEM_ID semId, int *val)
{
  RT_SEM_INFO info;
  int         rc;

  // only give the semaphore if empty -- this emulates a binary semaphore
  rc = rt_sem_inquire(semId,&info);
  if (rc) {
    printf("Error in rt_sem_inquire (rc=%d)\n",rc);
    return ERROR;
  }

  *val = info.count;

  return OK;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  printAllSem
 \date  Nov 2007
 
 \remarks 

 prints the value of all currently used semaphores
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void
printAllSem(void) {

  SM_PTR sptr;
  RT_SEM_INFO info;  
  int rc;
  
  sptr = smlist;

  while (sptr!=NULL) {

    switch (sptr->type) {
    case T_SM_SEM_B:
      rc = rt_sem_inquire(&(sptr->sem),&info);
      if (rc) {
	printf("Error in rt_sem_inquire (rc=%d)\n",rc);
      } else {
	printf("%s = %ld\n",sptr->name,info.count);
      }
      break;

    default:
      break;

    }

    sptr = (SM_PTR) sptr->nptr;
  }

}


/*!*****************************************************************************
 *******************************************************************************
 \note  ns2ticks
 \date  Nov 2009
 
 \remarks 

 converts nanoseconds to ticks
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  ns: nano second value to be converted to ticks
 
 ******************************************************************************/
long
ns2ticks(long long ns)
{

  return (long) rt_timer_ns2ticks((SRTIME)ns);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  ticks2ns
 \date  Nov 2009
 
 \remarks 

 converts ticks to nanoseconds
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]  ticks: tick value to be converted to nano seconds
 
 ******************************************************************************/
long long
ticks2ns(long ticks)
{

  return (long long) rt_timer_ticks2ns((SRTIME)ticks);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  getClockResolution
 \date  Nov 2009
 
 \remarks 

 returns the resolution of the system clock in nano seconds
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
long long
getClockResolution(void)
{
  RT_TIMER_INFO info;

  rt_timer_inquire(&info);

  if (info.period == TM_ONESHOT)
    return 1;
  else
    return (long long)info.period;

}
