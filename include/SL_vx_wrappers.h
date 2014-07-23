/*!=============================================================================
  ==============================================================================

  \file    SL_vx_wrappers.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  wrappers for vxworks function to allow simulator to run the 
  same files

  ============================================================================*/


#ifndef _SL_vx_wrapper_
#define _SL_vx_wrapper_

/*! from vxWorks.h --------------------------------------------------------- */
#define NO_WAIT         0
#define WAIT_FOREVER  (-1)
#define ERROR         (-1)
#define OK            0

typedef int STATUS;

/* from smLibP.h ---------------------------------------------------------- */

#define SEM_TYPE_BINARY		0x0	/*!< binary semaphore */
#define SEM_TYPE_MUTEX		0x1	/*!< mutual exclusion semaphore */
#define SEM_TYPE_COUNTING	0x2	/*!< counting semaphore */
#define SEM_TYPE_OLD		0x3	/*!< VxWorks 4.0 semaphore */

/* from smLib.h ----------------------------------------------------------- */

/* semaphore options */

#define SEM_Q_MASK		0x3	/*!< q-type mask */
#define SEM_Q_FIFO		0x0	/*!< first in first out queue */
#define SEM_Q_PRIORITY		0x1	/*!< priority sorted queue */
#define SEM_DELETE_SAFE		0x4	/*!< owner delete safe (mutex opt.) */
#define SEM_INVERSION_SAFE	0x8	/*!< no priority inversion (mutex opt.) */

/* binary semaphore initial state */

typedef enum		/*!< SEM_B_STATE */
  {
    SEM_EMPTY,			/*!< 0: semaphore not available */
    SEM_FULL			/*!< 1: semaphore available */
  } SEM_B_STATE;

#ifdef __XENO__
typedef RT_SEM* SEM_ID;
#else
typedef long SEM_ID;
#endif

/* function declarations */

#ifdef __cplusplus
extern "C" {
#endif
  
  extern STATUS 	semGive (SEM_ID semId);
  extern STATUS 	semSet (SEM_ID semId, int val);
  extern STATUS 	semGet (SEM_ID semId, int *val);
  extern STATUS 	semTake (SEM_ID semId, int timeout);
  extern STATUS 	semFlush (SEM_ID semId);
  extern STATUS 	semDelete (SEM_ID semId);
  extern int 	        semInfo (SEM_ID semId, int idList[], int maxTasks);
  extern STATUS 	semBLibInit (void);
  extern SEM_ID 	semBCreate (int options, SEM_B_STATE initialState);
  extern STATUS 	semCLibInit (void);
  extern SEM_ID 	semCCreate (int options, int initialCount);
  extern STATUS 	semMLibInit (void);
  extern SEM_ID 	semMCreate (int options);
  extern STATUS         semMGiveForce (SEM_ID semId);
  extern STATUS 	semOLibInit (void);
  extern SEM_ID 	semCreate (void);
  extern void 	        semShowInit (void);
  extern STATUS 	semShow (SEM_ID semId, int level);
  extern STATUS         taskDelay(int ticks);
  extern unsigned long  tickGet(void);
  extern void           printAllSem(void);
  extern long           ns2ticks(long long ns);
  extern long long      ticks2ns(long ticks);
  extern long long      getClockResolution(void);

  
#ifdef __cplusplus
}
#endif


/* from smNameLib.h ------------------------------------------------------- */

/* default object types */

#define MAX_DEF_TYPE   5	/*!< number of predefined types */

#define T_SM_SEM_B   0		/*!< shared binary semaphore type */
#define T_SM_SEM_C   1		/*!< shared counting semaphore type */
#define T_SM_MSG_Q   2		/*!< shared message queue type */
#define T_SM_PART_ID 3		/*!< shared memory partition type */
#define T_SM_BLOCK   4		/*!< shared memory allocated block type */

/* function declarations */

#ifdef __cplusplus
extern "C" {
#endif
  
#ifdef __XENO__
  extern    STATUS       smNameAdd (char * name, void ** value, int type, 
				    RT_HEAP *heap, RT_SEM *sem);
#else
  extern    STATUS       smNameAdd (char * name, void * value, int type, 
				    long smid, int key);
#endif
  extern    STATUS       smNameFind (char * name, void ** pValue,
				     int * pType, int waitType);
  extern    STATUS       smNameFindByValue (void * value, char * name, 
					    int * pType, int waitType);
  extern    STATUS       smNameRemove (char * name);
  extern    void         smNameShowInit (void);
  extern    STATUS       smNameShow (int level);
  
  /* from semSmLib.h -------------------------------------------------------- */
  extern    SEM_ID       semBSmCreate (char *semname, int id, int options, 
				       SEM_B_STATE initialState);
  extern    SEM_ID       semCSmCreate (int options, int initialCount);
  
  /* from smMemLib.h -------------------------------------------------------- */
  extern STATUS     smMemFree (void * ptr);
  extern int        smMemFindMax (void);
  extern void *     smMemMalloc (unsigned nBytes);
  extern void *     smMemCalloc (char *name, int id, int elemNum, int elemSize);
  extern void *     smMemRealloc (void * pBlock, unsigned newSize);
  extern STATUS     smMemAddToPool (char * pPool, unsigned poolSize);
  extern STATUS     smMemOptionsSet (unsigned options);
  extern void       smMemShowInit (void);
  extern void       smMemShow (int type);
  
  
  /* from logLib.h -  ------------------------------------------------------- */
  extern int	logMsg (char *fmt, int arg1, int arg2,
			int arg3, int arg4, int arg5, int arg6);
  
  /* from smObjLib.h -  ----------------------------------------------------- */
  extern void * smObjLocalToGlobal (void * localAdrs);
  extern void * smObjGlobalToLocal (void * globalAdrs);
  
#ifdef MACOS
  void bzero(char *c,int num);
#endif
  
  /* needed to remove shared memory objects with signal() */
  void removeSharedMemory(int dummy);
  void removeSharedMemoryAtExit(void);
  void setUserSignalHandler(void(*fptr)(void));


  /* some other fake functions */
  int sendUserGraphics(char *name, void *buf, int n_bytes);
  void changeRealTime(int flag);

  
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_vx_wrapper_ */
