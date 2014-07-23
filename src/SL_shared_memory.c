/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_shared_memory.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  manages share memory variables

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

#include "SL.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_common.h"

#define TIME_OUT_NS 1000000000

#define DEBUG FALSE

/* global variables */
SEM_ID             sm_joint_des_state_ready_sem;
SEM_ID             sm_sjoint_des_state_ready_sem;
SEM_ID             sm_raw_blobs_ready_sem;
SEM_ID             sm_learn_invdyn_sem;
SEM_ID             sm_learn_blob2body_sem;
SEM_ID             sm_task_servo_sem;  /* synchronization for task servo */
SEM_ID             sm_vision_servo_sem;  /* synchronization for vision servo */
SEM_ID             sm_openGL_servo_sem;  /* synchronization for openGL servo */
SEM_ID             sm_motor_servo_sem;  /* synchronization for motor servo */
SEM_ID             sm_simulation_servo_sem;  /* synchronization for simulation servo */
SEM_ID             sm_ros_servo_sem;  /* synchronization for ros servo */
SEM_ID             sm_pause_sem; /* signals pause to simulation servo */
SEM_ID             sm_user_graphics_ready_sem; /* signals user graphics available */
SEM_ID             sm_openGL_message_ready_sem; /* signals message to openGl servo */
SEM_ID             sm_simulation_message_ready_sem; /* signals message to sim servo */
SEM_ID             sm_task_message_ready_sem; /* signals message to task servo */
SEM_ID             sm_ros_message_ready_sem; /* signals message to ros servo */
SEM_ID             sm_motor_message_ready_sem; /* signals message to motor servo */
SEM_ID             sm_vision_message_ready_sem; /* signals message to vision servo */
SEM_ID             sm_objects_ready_sem; /* signals that objects are ready for read */
SEM_ID             sm_init_process_ready_sem; /* for starting up the SL processes */
SEM_ID             sm_oscilloscope_sem; /* for writing to the oscilloscope */

smVisionBlobs     *sm_vision_blobs;
SEM_ID             sm_vision_blobs_sem;
SL_fVisionBlob    *sm_vision_blobs_data;

smVisionBlobsaux  *sm_vision_blobs_aux;
SEM_ID             sm_vision_blobs_aux_sem;
SL_fVisionBlobaux *sm_vision_blobs_data_aux;

smCartStates      *sm_cart_states;
SEM_ID             sm_cart_states_sem;
SL_fCstate        *sm_cart_states_data;

smRawBlobs        *sm_raw_blobs;
SEM_ID             sm_raw_blobs_sem;
fBlob3D           *sm_raw_blobs_data;

smRawBlobs2D      *sm_raw_blobs2D;
SEM_ID             sm_raw_blobs2D_sem;
fBlob2D           *sm_raw_blobs2D_data;

smSJointDesStates *sm_sjoint_des_state;
SEM_ID             sm_sjoint_des_state_sem;
SL_fSDJstate      *sm_sjoint_des_state_data;

smJointStates     *sm_joint_state;
SEM_ID             sm_joint_state_sem;
SL_fJstate        *sm_joint_state_data;

smJointDesStates  *sm_joint_des_state;
SEM_ID             sm_joint_des_state_sem;
SL_fDJstate       *sm_joint_des_state_data;

smMiscSensors     *sm_misc_sensor;
SEM_ID             sm_misc_sensor_sem;
float             *sm_misc_sensor_data;

smJointSimStates  *sm_joint_sim_state;
SEM_ID             sm_joint_sim_state_sem;
SL_fJstate        *sm_joint_sim_state_data;

smMiscSimSensors  *sm_misc_sim_sensor;
SEM_ID             sm_misc_sim_sensor_sem;
float             *sm_misc_sim_sensor_data;

smContacts        *sm_contacts;
SEM_ID             sm_contacts_sem;
contactShort      *sm_contacts_data;

smBaseState       *sm_base_state;
SEM_ID             sm_base_state_sem;
SL_fCstate        *sm_base_state_data;

smBaseOrient      *sm_base_orient;
SEM_ID             sm_base_orient_sem;
SL_fquat          *sm_base_orient_data;

smUserGraphics    *sm_user_graphics;
SEM_ID             sm_user_graphics_sem;

smMessage         *sm_simulation_message;
SEM_ID             sm_simulation_message_sem;
smMessage         *sm_openGL_message;
SEM_ID             sm_openGL_message_sem;
smMessage         *sm_task_message;
SEM_ID             sm_task_message_sem;
smMessage         *sm_ros_message;
SEM_ID             sm_ros_message_sem;
smMessage         *sm_motor_message;
SEM_ID             sm_motor_message_sem;
smMessage         *sm_vision_message;
SEM_ID             sm_vision_message_sem;

smDCommands       *sm_des_commands;
SEM_ID             sm_des_commands_sem;

smROSState        *sm_ros_state;
SEM_ID             sm_ros_state_sem;

smOscilloscope    *sm_oscilloscope;
SEM_ID             sm_oscilloscope_sem;

/* local variables */
static int         n_bytes_sm_allocated = 0;

/* local functions */
static int init_sm_object(char *smname, size_t structsize, size_t datasize, 
			  SEM_ID *semptr, void **structptr);
static int init_sm_sem(char *smname, int filltype, void **semptr);

/*!*****************************************************************************
*******************************************************************************
\note  init_shared_memory
\date  April 1999

\remarks 

initializes the share memory objects

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
init_shared_memory(void)
{
  int    j,i,n,rc;
  STATUS error;
  char   string[100];
  char   name[100];
  int    mtype;
  char  *ptr;
  int    flag;

  /* make sure that n_contacts is correct */
  n=count_extra_contact_points(config_files[CONTACTS]);
  n_contacts = n_links + n;


  /********************************************************************/
  /********************************************************************/
  /* shared memory objects */
  /********************************************************************/
  if (init_sm_object("smJointState", 
		     sizeof(smJointStates),
		     sizeof(SL_fJstate)*(n_dofs+1),
		     &sm_joint_state_sem,
		     (void **)&sm_joint_state)) {
    ;
  } else {
    return FALSE;
  }
  sm_joint_state_data = 
    (SL_fJstate *)my_calloc(n_dofs+1,sizeof(SL_fJstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smJointDesState", 
		     sizeof(smJointDesStates),
		     sizeof(SL_fDJstate)*(n_dofs+1),
		     &sm_joint_des_state_sem,
		     (void **)&sm_joint_des_state)) {
    ;
  } else {
    return FALSE;
  }
  sm_joint_des_state_data = 
    (SL_fDJstate *)my_calloc(n_dofs+1,sizeof(SL_fDJstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smSJointDesState", 
		     sizeof(smSJointDesStates),
		     sizeof(SL_fSDJstate)*(n_dofs+1),
		     &sm_sjoint_des_state_sem,
		     (void **)&sm_sjoint_des_state)) {
    ;
  } else {
    return FALSE;
  }
  sm_sjoint_des_state_data = 
    (SL_fSDJstate *)my_calloc(n_dofs+1,sizeof(SL_fSDJstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smJointSimState", 
		     sizeof(smJointSimStates),
		     sizeof(SL_fJstate)*(n_dofs+1),
		     &sm_joint_sim_state_sem,
		     (void **)&sm_joint_sim_state)) {
    ;
  } else {
    return FALSE;
  }
  sm_joint_sim_state_data = 
    (SL_fJstate *)my_calloc(n_dofs+1,sizeof(SL_fJstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smVisionBlobs", 
		     sizeof(smVisionBlobs),
		     sizeof(SL_fVisionBlob)*(max_blobs+1),
		     &sm_vision_blobs_sem,
		     (void **)&sm_vision_blobs)) {
    ;
  } else {
    return FALSE;
  }
  sm_vision_blobs_data = 
    (SL_fVisionBlob *)my_calloc(max_blobs+1,sizeof(SL_fVisionBlob),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smVisionBlobsaux", 
		     sizeof(smVisionBlobsaux),
		     sizeof(SL_fVisionBlobaux)*(max_blobs+1),
		     &sm_vision_blobs_aux_sem,
		     (void **)&sm_vision_blobs_aux)) {
    ;
  } else {
    return FALSE;
  }
  sm_vision_blobs_data_aux = 
    (SL_fVisionBlobaux *)my_calloc(max_blobs+1,sizeof(SL_fVisionBlobaux),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smRawBlobs", 
		     sizeof(smRawBlobs),
		     sizeof(fBlob3D)*(max_blobs+1),
		     &sm_raw_blobs_sem,
		     (void **)&sm_raw_blobs)) {
    ;
  } else {
    return FALSE;
  }
  sm_raw_blobs_data = 
    (fBlob3D *)my_calloc(max_blobs+1,sizeof(fBlob3D),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smRawBlobs2D", 
		     sizeof(smRawBlobs2D),
		     sizeof(fBlob2D)*(2*max_blobs+1),
		     &sm_raw_blobs2D_sem,
		     (void **)&sm_raw_blobs2D)) {
    ;
  } else {
    return FALSE;
  }
  sm_raw_blobs2D_data = 
    (fBlob2D *)my_calloc(2*max_blobs+1,sizeof(fBlob2D),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smCartStates", 
		     sizeof(smCartStates),
		     sizeof(SL_fCstate)*(n_endeffs+1),
		     &sm_cart_states_sem,
		     (void **)&sm_cart_states)) {
    ;
  } else {
    return FALSE;
  }
  sm_cart_states_data = 
    (SL_fCstate *)my_calloc(n_endeffs+1,sizeof(SL_fCstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smMiscSensors", 
		     sizeof(smMiscSensors),
		     sizeof(float)*(n_misc_sensors+1),
		     &sm_misc_sensor_sem,
		     (void **)&sm_misc_sensor)) {
    ;
  } else {
    return FALSE;
  }
  sm_misc_sensor_data = 
    (float *)my_calloc(n_misc_sensors+1,sizeof(float),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smMiscSimSensors", 
		     sizeof(smMiscSimSensors),
		     sizeof(float)*(n_misc_sensors+1),
		     &sm_misc_sim_sensor_sem,
		     (void **)&sm_misc_sim_sensor)) {
    ;
  } else {
    return FALSE;
  }
  sm_misc_sim_sensor_data = 
    (float *)my_calloc(n_misc_sensors+1,sizeof(float),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smContacts", 
		     sizeof(smContacts),
		     sizeof(contactShort)*(n_contacts+1),
		     &sm_contacts_sem,
		     (void **)&sm_contacts)) {
    ;
  } else {
    return FALSE;
  }
  sm_contacts_data = 
    (contactShort *)my_calloc(n_links+1,sizeof(contactShort),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smBaseState", 
		     sizeof(smBaseState),
		     sizeof(SL_fCstate)*(1+1),
		     &sm_base_state_sem,
		     (void **)&sm_base_state)) {
    ;
  } else {
    return FALSE;
  }
  sm_base_state_data = 
    (SL_fCstate *)my_calloc(1+1,sizeof(SL_fCstate),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smBaseOrient", 
		     sizeof(smBaseOrient),
		     sizeof(SL_fquat)*(1+1),
		     &sm_base_orient_sem,
		     (void **)&sm_base_orient)) {
    ;
  } else {
    return FALSE;
  }
  sm_base_orient_data = 
    (SL_fquat *)my_calloc(1+1,sizeof(SL_fquat),MY_STOP);
  /********************************************************************/
  if (init_sm_object("smUserGraphics", 
		     sizeof(smUserGraphics),
		     0,
		     &sm_user_graphics_sem,
		     (void **)&sm_user_graphics)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smOpenGLMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_openGL_message_sem,
		     (void **)&sm_openGL_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smVisionMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_vision_message_sem,
		     (void **)&sm_vision_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smSimMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_simulation_message_sem,
		     (void **)&sm_simulation_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smTaskMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_task_message_sem,
		     (void **)&sm_task_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smROSMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_ros_message_sem,
		     (void **)&sm_ros_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smMotorMessage", 
		     sizeof(smMessage),
		     0,
		     &sm_motor_message_sem,
		     (void **)&sm_motor_message)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smDCommands", 
		     sizeof(smDCommands),
		     sizeof(SL_fDCommands)*(n_dofs+1),
		     &sm_des_commands_sem,
		     (void **)&sm_des_commands)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  if (init_sm_object("smROSState", 
		     sizeof(smROSState),
		     sizeof(SL_fJstate)*(n_dofs+1) +
		     sizeof(SL_fDJstate)*(n_dofs+1) +
		     sizeof(SL_fCstate)*(1+1) +		     
		     sizeof(SL_fquat)*(1+1) +
		     sizeof(float)*(n_misc_sensors+1),
		     &sm_ros_state_sem,
		     (void **)&sm_ros_state)) {
    ;
  } else {
    return FALSE;
  }
  /********************************************************************/
  rc = TRUE;
  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"osc_enabled", &rc))
    rc = macro_sign(abs(rc));

  if (rc) {
    if (init_sm_object("smOscilloscope", 
		       sizeof(smOscilloscope),
		       sizeof(SL_oscEntry)*(OSC_SM_BUFFER_SIZE+1),
		       &sm_oscilloscope_sem,
		       (void **)&sm_oscilloscope)) {
      ;
    } else {
      return FALSE;
    }
  }

  /********************************************************************/
  /********************************************************************/
  /* shared semaphores */

  if (!init_sm_sem("smJointDSReadySem", SEM_EMPTY,(void**)&sm_joint_des_state_ready_sem))
    return FALSE;

  if (!init_sm_sem("smSJointDSReadySem", SEM_EMPTY,(void**)&sm_sjoint_des_state_ready_sem))
    return FALSE;
  
  if (!init_sm_sem("smRawBlobsReadySem", SEM_EMPTY,(void**)&sm_raw_blobs_ready_sem))
    return FALSE;
  
  if (!init_sm_sem("smLearnInvdynSem", SEM_EMPTY,(void**)&sm_learn_invdyn_sem))
    return FALSE;
  
  if (!init_sm_sem("smLearnBlob2BodySem", SEM_EMPTY,(void**)&sm_learn_blob2body_sem))
    return FALSE;
  
  if (!init_sm_sem("smOscilloscopeSem", SEM_FULL,(void**)&sm_oscilloscope_sem))
    return FALSE;
  
  /********************************************************************/
  /********************************************************************/
  /* process synchronization semaphores */

  if (!init_sm_sem("smTaskServoSem", SEM_EMPTY,(void**)&sm_task_servo_sem))
    return FALSE;

  if (!init_sm_sem("smVisionServoSem", SEM_EMPTY,(void**)&sm_vision_servo_sem))
    return FALSE;

  if (!init_sm_sem("smOpenGLServoSem", SEM_EMPTY,(void**)&sm_openGL_servo_sem))
    return FALSE;

  if (!init_sm_sem("smMotorServoSem", SEM_EMPTY,(void**)&sm_motor_servo_sem))
    return FALSE;

  if (!init_sm_sem("smSimServoSem", SEM_EMPTY,(void**)&sm_simulation_servo_sem))
    return FALSE;

  if (!init_sm_sem("smROSServoSem", SEM_EMPTY,(void**)&sm_ros_servo_sem))
    return FALSE;

  if (!init_sm_sem("smPauseSem", SEM_EMPTY,(void**)&sm_pause_sem))
    return FALSE;

  if (!init_sm_sem("smUGraphReadySem", SEM_EMPTY,(void**)&sm_user_graphics_ready_sem))
    return FALSE;

  if (!init_sm_sem("smOpenGLMsgReadySem", SEM_EMPTY,(void**)&sm_openGL_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smVisionMsgReadySem", SEM_EMPTY,(void**)&sm_vision_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smSimMsgReadySem", SEM_EMPTY,(void**)&sm_simulation_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smTaskMsgReadySem", SEM_EMPTY,(void**)&sm_task_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smROSMsgReadySem", SEM_EMPTY,(void**)&sm_ros_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smMotorMsgReadySem", SEM_EMPTY,(void**)&sm_motor_message_ready_sem))
    return FALSE;

  if (!init_sm_sem("smObjectsReadySem", SEM_EMPTY,(void**)&sm_objects_ready_sem))
    return FALSE;

  if (!init_sm_sem("smInitProcReadySem", SEM_EMPTY,(void**)&sm_init_process_ready_sem))
    return FALSE;


  /********************************************************************/
  /********************************************************************/

#ifdef UNIX
  addToMan("showSem","displays all semaphores in the system",printAllSem);
#endif

  printf("Total Shared Memory Allocated = %d Bytes\n",n_bytes_sm_allocated);

  return TRUE;

}


/*!***************************************************************************
******************************************************************************
\note   init_sm_object
\date   May 2000

\remarks

initializes a shared memory object with semaphore as first element and
data array

******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  smname     :  name of the shared memory object
\param[in]  structsize :  size of base structure
\param[in]  datasize   :  size of data structure
\param[out] semptr     :  ptr to object's semaphore
\param[out] structptr  :  handle to object's base structure

*****************************************************************************/
static int
init_sm_object(char *smname, size_t structsize, size_t datasize, 
	       SEM_ID *semptr, void **structptr)

{
  int      mtype;
  SEM_ID  *semptr_sm;
  STATUS   error;
  char     smn[100];
  char     semn[100];

  /* build the full name from both robot name and desired name */
#ifdef VX  /* vxworks has a 20 character limit */
  sprintf(smn,"%s",smname);
#else
  sprintf(smn,"%s.%s",robot_name,smname);
#endif

  if (smNameFind(smn, structptr,&mtype, NO_WAIT) == ERROR) {

    /* get the object's base structure and add datasize to memory chunk */
#ifdef VX
    *structptr = (void *)smMemCalloc(1,structsize+datasize);
#else
    *structptr = (void *)smMemCalloc(smn,parent_process_id,1,structsize+datasize);
#endif
    if (*structptr == NULL) {
      printf("Couldn't create shared memory object %s\n",smn);
      return FALSE;
    }

    /* get the object's semaphore */
    semptr_sm = *structptr;
#ifdef VX
    *semptr = semBSmCreate(SEM_Q_FIFO, SEM_FULL);
    if (*semptr == NULL) {
#else
    sprintf(semn,"%s.%s_sem",robot_name,smname);
    *semptr = semBSmCreate (semn,parent_process_id,SEM_Q_FIFO, SEM_FULL);
    if (*semptr == (SEM_ID) (-1)) {
#endif
      printf("Couldn't create shared semaphore for object %s -- sem_id=%ld\n",
	     semn,(long)*semptr);
      return FALSE;
    }
    
#ifdef __XENO__
    // xenomai does not have a global identifier for semaphores
    *semptr_sm = NULL;
#else
    // vxWorks and Unix use global identifiers for semaphores which
    // we can keep in shared memory
    *semptr_sm = *semptr;
#endif
    
#ifdef VX
    /* add the object to the name database */
    error = smNameAdd(smn,(void*)smObjLocalToGlobal(*structptr),T_SM_PART_ID);
    if (error == ERROR)
      return FALSE;
#endif
    
    } else {
      *structptr = smObjGlobalToLocal(*structptr);
  }
    
  if (DEBUG)
    printf("Shared memory for %s is set at struct: g=0x%lx l=0x%lx\n",
	   smn,
	   (unsigned long) smObjLocalToGlobal((void*)*structptr),
	   (unsigned long) *structptr);

  // keep statistics of allocated shared memory
  n_bytes_sm_allocated += datasize + structsize;
  
  return TRUE;
  
}
  
/*!***************************************************************************
******************************************************************************
\note    init_sm_sem
\date    May 2000

\remarks

initializes a shared semaphore

******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  smname     :  name of the shared memory object
\param[in]  filltype   :  SEM_FULL or SEM_EMPTY
\param[out] semptr     :  handle to semaphore

*****************************************************************************/
static int
init_sm_sem(char *smname, int filltype, void **semptr)
    
{
  int      mtype;
  STATUS   error;
  char     smn[100];
  
  /* build the full name from both robot name and desired name */
#ifdef VX  /* vxworks has a 20 character limit */
  sprintf(smn,"%s",smname);
  smn[19]='\0';  
#else
  sprintf(smn,"%s.%s",robot_name,smname);
#endif
  
  if (smNameFind(smn, semptr, &mtype,NO_WAIT)== ERROR) {
#ifdef VX
    *semptr = (void *)semBSmCreate (SEM_Q_FIFO, filltype);
    if (*semptr == NULL) {
#else
      *semptr = (void *) semBSmCreate (smn,parent_process_id,SEM_Q_FIFO, filltype);
      if (*semptr == (void *)-1 ) {
#endif
	printf("Couldn't create shared semaphore %s\n",smn);
	return FALSE;
      }
      
#ifdef VX    
      error = smNameAdd(smn, (void*)*semptr, T_SM_SEM_B);
      if (error == ERROR)
	return FALSE;
#endif
      
    } else {
      ;
    }
    
    if (DEBUG)
      printf("Shared semaphore for %s is set at 0x%lx\n",smn,(unsigned long) *semptr);
    
    return TRUE;
    
}
  
/*!****************************************************************************
******************************************************************************
\note  sendMessageToServo
\date  Nov. 2007

\remarks

sends message to another servo via shared memory

*****************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  sm_message    : shared memory for message
\param[in]  sm_message_sem: shared memory semaphore for message
\param[in]  ready_sem     : semaphore for signaling message ready status
\param[in]  message       : message name
\param[in]  buf           : byte buffer for message
\param[in]  n_bytes       : number of bytes in buffer

    *****************************************************************************/
void 
sendMessageToServo(smMessage *sm_message, SEM_ID sm_message_sem, 
		   SEM_ID ready_sem, char *message, 
		   void *buf, int n_bytes) 
{
  
  int i;
  
  // try to take semaphore
  if (semTake(sm_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    printf("Couldn'st take message semaphore\n");
    return;
  }
  
  // make sure the pointer offset buffer is correct
  if (sm_message->n_msgs == 0)
    sm_message->moff[1] = 0;
  
  // check whether there is space for the message
  if (sm_message->n_msgs >= MAX_N_MESSAGES) {
    printf("Message buffer exhausted in sendMessageToServo\n");
    semGive(sm_message_sem);
    return;
  }
  
  if (MAX_BYTES_MESSAGES-sm_message->n_bytes_used < n_bytes) {
    printf("Message memory buffer exhausted in sendMessageToServo\n");
    semGive(sm_message_sem);
    return;
  }
  
  // update the message logistics
  ++sm_message->n_msgs;
  sm_message->n_bytes_used += n_bytes;
  
  // specify the name and info of this message
  strcpy(sm_message->name[sm_message->n_msgs],message);
  if (n_bytes > 0)
    memcpy(sm_message->buf+sm_message->moff[sm_message->n_msgs],buf,n_bytes);
  
  // prepare pointer buffer for next message
  if (sm_message->n_msgs < MAX_N_MESSAGES)
    sm_message->moff[sm_message->n_msgs+1]=sm_message->moff[sm_message->n_msgs]+n_bytes;
  
  // give back semaphore
  semGive(sm_message_sem);
  
  // signal message ready
  semGive(ready_sem);
  
}
 
 void 
   sendMessageSimulationServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_simulation_message, sm_simulation_message_sem, 
		      sm_simulation_message_ready_sem, 
		      message, buf, n_bytes);
 }
 
 void 
   sendMessageTaskServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_task_message, sm_task_message_sem, 
		      sm_task_message_ready_sem, 
		      message, buf, n_bytes);
 }
 
 void 
   sendMessageMotorServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_motor_message, sm_motor_message_sem, 
		      sm_motor_message_ready_sem, 
		      message, buf, n_bytes);
 }
 
 void 
   sendMessageOpenGLServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_openGL_message, sm_openGL_message_sem, 
		      sm_openGL_message_ready_sem, 
		      message, buf, n_bytes);
 }

 void 
   sendMessageROSServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_ros_message, sm_ros_message_sem, 
		      sm_ros_message_ready_sem, 
		      message, buf, n_bytes);
 }

 void 
   sendMessageVisionServo(char *message, void *buf, int n_bytes)
 {
   sendMessageToServo(sm_vision_message, sm_vision_message_sem, 
		      sm_vision_message_ready_sem, 
		      message, buf, n_bytes);
 }

