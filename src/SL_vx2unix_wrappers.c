/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_vx2unix_wrappers.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  replacement functions and  definition of functions of the real-time 
  operating system vxWorks adjusted for a generic unix platform

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
#include "SL_unix_common.h"

// needed for book keeping of share memory
typedef struct smlist /*!< share memory list */
{
  void *smptr;      /*!< pointer to shared memory value */
  char  name[100];  /*!< shared memory name */
  int   type;       /*!< type of shared memory */
  long  smid;       /*!< ID of shared memory */
  int   key;        /*!< key of shared memmory */
  char *nptr;       /*!< pointer to next shared memory */
} SMLIST, *SM_PTR;

typedef union {
  int     val;            /*!< value for SETVAL */
  struct  semid_ds *buf;  /*!< buffer for IPC_STAT & IPC_SET */
  unsigned short *array;  /*!< array for GETALL & SETALL */
} semunion;


//! list of all allocated shared memory objects
#define MAX_SM
static int sm_ids_list[1000];

//! user function to be called on exit
static void (*user_signal_handler)(void) = NULL;  //!< function pointer

static int 
tickCount(int tick_freq);
STATUS       
smKeyFind (int key);


/* semaphore linked list */
SM_PTR   smlist = NULL;

int	
logMsg (char *fmt, int arg1, int arg2,
	int arg3, int arg4, int arg5, int arg6)
{
  printf(fmt,arg1,arg2,arg3,arg4,arg5,arg6);
  return OK;
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
  struct timespec ns;
  long period = getClockResolution();;
  long aux;

  // make sure the delay is at least one period long
  if (num <= 0)
    return FALSE;
  
  aux    = num*period;

  ns.tv_sec  = aux/1000000000;
  ns.tv_nsec = aux-ns.tv_sec*1000000000;

  nanosleep(&ns,NULL);

  return TRUE;
}

unsigned long
tickGet(void)
{
  struct timeval t;
  long period = getClockResolution();;
    
  gettimeofday(&t,NULL);

  return ((double)t.tv_sec/(double)period)*1000000000 + ((double)t.tv_usec/(double)period)*1000;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mytok
 \date  Nov 2007
 
 \remarks 
 
 Uses a heuristic to create a unique integer number from a given string 
 and an additional integer. This function mimics ftok(), just without the
 need to access a file. It is not clear that his heuristic creates a
 truely unique identifier -- but I hope it is good enough.
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     string          : a character string
 \param[in]     id              : an additional ID number
 
 ******************************************************************************/
static int
mytok(char *string, int id) {
  unsigned int i;
  int rc = id;

  for (i=0; i<strlen(string); ++i)
    rc += i*100+string[i];

  return rc;
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
void *     
smMemCalloc (char *shmname, int id, int elemNum, int elemSize) 
{
  key_t  shmkey = 0;
  void  *ptr;
  long   shmid;
  struct shmid_ds ds;
  STATUS error;
  
  // create a key for the shared memory
  shmkey = mytok(shmname,id);
  while (smKeyFind(shmkey) == OK)
    ++shmkey;

  // create the share memory
  shmid = shmget(shmkey,elemNum*elemSize,0); // test for existance
  if (shmid == -1) { // new shared memory object
    shmid = shmget(shmkey,elemNum*elemSize, IPC_CREAT | 0666);
    if (shmid == -1) {
      printf("problems with shmget: errno=%d\n",errno);
      return NULL;
    }
  }

  // map the shared memory
  ptr = shmat(shmid,NULL,0);
  if ((long) ptr == -1) {
    printf("problems with shmat: errno=%d\n",errno);
    return NULL;
  }

  // add the shared memory to list of shared memory objects
  error = smNameAdd (shmname, ptr, T_SM_PART_ID, shmid, shmkey);
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

 A replacement for the vxWorks function smNameFind
  
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
 \note  smKeyFind
 \date  Nov 2007
 
 \remarks 

 checks whether a key already exist
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     key             : key of the shared memory

 returns OK if key exists, and ERROR if it does not exist
 
 ******************************************************************************/
STATUS       
smKeyFind (int key)
{

  SM_PTR sptr;

  sptr = smlist;

  while (sptr!=NULL) {
    if (sptr->key == key) {
      return OK;
    }
    sptr = (SM_PTR) sptr->nptr;
  }
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
 \param[in]     value           : pointer to shared memory
 \param[in]     pType           : shared memory type
 \param[in]     smid            : shared memory id
 \param[in]     key             : key of share memory
 
 ******************************************************************************/
STATUS       
smNameAdd (char * name, void * value, int pType, long smid, int key)
{
  SM_PTR *shandle;
  void *pValue;

  // does the name already exist? 
  if (smKeyFind(key) == OK || smNameFind(name,&pValue, &pType, NO_WAIT) == OK)
    return ERROR;

  shandle = &smlist;

  while(*shandle != NULL) {
    shandle = (SM_PTR *)&((*shandle)->nptr);
  }

  *shandle = my_calloc(1,sizeof(SMLIST),MY_STOP);
  (*shandle)->type = pType;
  (*shandle)->nptr = NULL;
  (*shandle)->smptr= value;
  (*shandle)->smid = smid;
  (*shandle)->key  = key;
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
  int         semkey;
  long        semid;
  semunion    arg;
  int         rc;
  STATUS      error;

  // create a key for the semaphore
  semkey = mytok(semname,id);
  while (smKeyFind(semkey) == OK)
    ++semkey;

  // check for existing semaphore
  semid = semget(semkey,1,0);
  if (semid == -1) {

    // create the share semaphore
    semid = semget(semkey,1,IPC_CREAT | 0666);
    if (semid == -1) {
      printf("problems with semget: errno=%d\n",errno);
      return (SEM_ID)(-1);
    }

    // set value of semaphore to defined state
    arg.val = initialState;
    rc = semctl(semid,0,SETVAL,arg);
    if (rc == -1) {
      printf("rc=%d errno=%d\n",rc,errno);
      return (SEM_ID)(-1);
    }

  } 

  // add the semaphore to list of shared memory objects
  error = smNameAdd (semname, (void *)semid, T_SM_SEM_B, semid, semkey);
  if (error == ERROR) {
    printf("Unexpected error in semaphore management\n");
    return (SEM_ID)(-1);
  }

  return semid;
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

  struct sembuf   sembuf;
  int    start_tick;

  sembuf.sem_num =  0;
  sembuf.sem_op  = -1;

  start_tick = tickCount(servo_base_rate);

  while (TRUE) {
    
    switch (timeout) {
      
    case WAIT_FOREVER:
      sembuf.sem_flg =  0; // this implies IPC_WAIT
      do {
        if (semop(semId,&sembuf,1) != -1)
	  return OK;
        else {
	  if (errno!=EINTR) {
	    printf("SemTake(WAIT_FOREVER) exited with errno=%d\n",errno);
	    return ERROR;
	  }
        }
      } while(1);
      
    case NO_WAIT:
      sembuf.sem_flg =  IPC_NOWAIT; 
      if (semop(semId,&sembuf,1) != -1)
	return OK;
      else {
	if (errno!=EAGAIN) {
	  printf("SemTake(IPC_NOWAIT) exited with errno=%d\n",errno);
	  exit(-1);
	}
	return ERROR;
      }
      
    default:
      sembuf.sem_flg =  IPC_NOWAIT; 
      if (semop(semId,&sembuf,1) != -1)
	return OK;
      else {
	if (errno!=EAGAIN) {
	  printf("SemTake(TIME_OUT) exited with errno=%d\n",errno);
	  exit(-1);
	}
	if (tickCount(servo_base_rate)-start_tick > timeout) 
	  return ERROR;
      }
      
    }
    
  }

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
  struct sembuf   sembuf[2];
  int val=0;

  // first ensure that the semaphore is 0
  sembuf[0].sem_num = 0;
  sembuf[0].sem_op  = 0;
  sembuf[0].sem_flg = IPC_NOWAIT;

  // then give the semaphore
  sembuf[1].sem_num =  0;
  sembuf[1].sem_op  =  1;
  sembuf[1].sem_flg =  0;

  if (semop(semId,sembuf,2) != -1)
    return OK;
  else {
    // errno==EAGAIN when the semaphore is not 0
    if (errno==EAGAIN)
      return OK;
    else {
      printf("SemGive terminated with errno=%d\n",errno);
      return ERROR;
    }
  }
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
  semunion    arg;
  struct sembuf   sembuf;
  int         rc;
  int         i;

  // get the number of processes waiting for this semaphore
  rc = semctl(semId,0,GETNCNT,arg);
  if (rc == -1) {
    printf("Error when flushing semaphore %ld\n",semId);
    return ERROR;
  }

  sembuf.sem_num =  0;
  sembuf.sem_op  =  rc;
  sembuf.sem_flg =  0;
  if (semop(semId,&sembuf,1) != -1)
    return OK;
  else {
    printf("SemFlush terminated with errno=%d\n",errno);
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
  semunion arg;
  struct shmid_ds shm_ds;
  char string[100];
  
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
      semctl(sptr->smid,0,IPC_RMID,arg);
      break;

    case T_SM_PART_ID:
      semctl(*((int *)sptr->smptr),0,IPC_RMID,arg);
      shmctl(sptr->smid,IPC_RMID,&shm_ds);
      break;

    default:
      break;

    }

    sptr = (SM_PTR) sptr->nptr;
  }

  // make sure the thread is unlocked
  sl_rt_mutex_unlock( &mutex1 );
  run_command_line_thread_flag = FALSE;

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
 \note  tickCount
 \date  August, 2002
 
 \remarks 
 
 returns the count from the system clock
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     tick_freq : the desired frequency of ticks
  
 ******************************************************************************/
static int 
tickCount(int tick_freq)

{
  struct timeval t;
  static long    tick_offset;
  static int     firsttime = TRUE;

  if (firsttime) {
    gettimeofday(&t,NULL);
    tick_offset = t.tv_sec;
    firsttime = FALSE;
  }

  gettimeofday(&t,NULL);
  return (int) ((t.tv_sec-tick_offset)*tick_freq+(t.tv_usec*tick_freq)/1000000);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  semSet
 \date  Nov 2007
 
 \remarks 

 sets the semaphore to a given value
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     semId           : id of semaphore
 \param[in]     val             : value of semaphore
 
 ******************************************************************************/
STATUS 	
semSet (SEM_ID semId, int val)
{
  semunion    arg;
  int         rc;

  arg.val = val;
  rc = semctl(semId,0,SETVAL,arg);
  if (rc == -1) {
    printf("rc=%d errno=%d\n",rc,errno);
    return ERROR;
  }

  return OK;
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
  struct sembuf   sembuf[1];
  int         rc;

  // first ensure that the semaphore is 0
  sembuf[0].sem_num = 0;
  sembuf[0].sem_op  = 0;
  sembuf[0].sem_flg = IPC_NOWAIT;

  if ((rc = semop(semId,sembuf,1)) != -1)
    *val = 0;
  else if (errno == EAGAIN)
    *val = 1;
  else {
    printf("rc=%d errno=%d\n",rc,errno);
    return ERROR;
  }

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
  semunion arg;
  
  sptr = smlist;

  while (sptr!=NULL) {

    switch (sptr->type) {
    case T_SM_SEM_B:
      printf("%s = %d\n",sptr->name,semctl(sptr->smid,0,GETVAL,arg));
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
  long period = getClockResolution();

  return (long) ns/period;

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
  long period = getClockResolution();

  return (long long) ticks*period;

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
  static int firsttime = TRUE;
  static long  long period = 1e10;
  long long new_period, total_time;
  int i,j;
  int r=25;
  struct timespec ns;
  struct timeval ts,te,fn_start_time;

    
  if (firsttime) {
    firsttime = FALSE;

#ifdef  powerpc
#define __mac__
#endif
#ifdef  i386mac
#define __mac__
#endif
#ifdef x86_64mac
#define __mac__
#endif

#ifdef __mac__

    // Macs don't have clock_getres so far -- get the clock empirically

    gettimeofday(&fn_start_time,NULL);

    // this is an emprical way to determine the system clock approximately
    ns.tv_sec = 0;
    ns.tv_nsec = 1;

    for (j=1; j<=100; ++j) { // try 100 times to find a smaller average value
      gettimeofday(&ts,NULL);
      
      // terminate if we've already spent more than a second in this function
      total_time = (long)((ts.tv_sec-fn_start_time.tv_sec)*1e9+
			  (ts.tv_usec-fn_start_time.tv_usec)*1000);
      if (total_time > 1e9 && j>=2)
        break;

      for (i=1; i<=r; ++i)
	nanosleep(&ns,NULL);

      gettimeofday(&te,NULL);
      new_period = (long)((te.tv_sec-ts.tv_sec)*1e9+(te.tv_usec-ts.tv_usec)*1000)/r;
      if (new_period < period) // we assume the shortest period is what the computer can do
	period = new_period;
    }

#else

    // non-Mac OS X systems have clock_getres() functions
    clock_getres(CLOCK_REALTIME,&ns);
    period = (long)(ns.tv_sec*1e9+ns.tv_nsec);

#endif

  }

  return period;

}
 
