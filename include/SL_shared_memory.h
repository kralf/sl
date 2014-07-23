/*!=============================================================================
  ==============================================================================

  \file    SL_shared_memory.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_shared_memory.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_shared_memory_
#define _SL_shared_memory_

/* note that the flexible length data structure array needs to be the
   last element in the shared memory structure, allocated as an array
   with only one element. We actually allocate more memory at run time
   depending and the desired array length */

typedef struct smVisionBlobs {
  SEM_ID          sm_sem;
  int             frame_counter;
  char            pp_name[30];
  SL_fVisionBlob  blobs[1];
} smVisionBlobs;

typedef struct smVisionBlobsaux {
  SL_fVisionBlobaux blobs[1];
} smVisionBlobsaux;

typedef struct smMiscSensors {
  SEM_ID          sm_sem;
  float           ts;
  float           value[1];
} smMiscSensors;

typedef struct smMiscSimSensors {
  SEM_ID          sm_sem;
  float           ts;
  float           value[1];
} smMiscSimSensors;

typedef struct smCartStates {
  SEM_ID          sm_sem;
  float           ts;
  SL_fCstate      state[1];
} smCartStates;

typedef struct smCartOrients {
  SEM_ID          sm_sem;
  float           ts;
  SL_fCorient     orient[1];
} smCartOrients;

typedef struct smRawBlobs {
  SEM_ID         sm_sem;
  fBlob3D        blobs[1];
} smRawBlobs;

typedef struct smRawBlobs2D {
  SEM_ID         sm_sem;
  fBlob2D        blobs[1];
} smRawBlobs2D;

typedef struct smSJointDesStates {
  SEM_ID         sm_sem;
  float           ts;
  SL_fSDJstate   sjoint_des_state[1];
} smSJointDesStates;

typedef struct smJointStates {
  SEM_ID         sm_sem;
  float           ts;
  SL_fJstate     joint_state[1];
} smJointStates;

typedef struct smROSState {
  SEM_ID         sm_sem;
  float          ts;
  char           data[1];
} smROSState;

typedef struct smJointDesStates {
  SEM_ID          sm_sem;
  float           ts;
  SL_fDJstate     joint_des_state[1];
} smJointDesStates;

typedef struct SL_fDCommands{
  float           th;   //!< desired th
  float           thd;  //!< desired thd
  float           uff;  //!< feedfoward commands
  float           upd;  //!< pd feedback command
  float           u;    //!< final command from controller.c
} SL_fDCommands;

typedef struct smDCommands {
  SEM_ID          sm_sem;
  float           ts;
  SL_fDCommands   des_command[1];
} smDCommands;

typedef struct smJointSimStates {
  SEM_ID          sm_sem;
  float           ts;
  SL_fJstate      joint_sim_state[1];
} smJointSimStates;

typedef struct contactShort {
  int             status;            /*!< contact status */
  float           f[N_CART+1];       /*!< contact force */
  float           n[N_CART+1];       /*!< contact normal */
  char            name[STRING100];   /*!< object name in contact */
} contactShort;

typedef struct smContacts {
  SEM_ID          sm_sem;
  contactShort    contact[1];
} smContacts;

typedef struct smBaseState {
  SEM_ID          sm_sem;
  float           ts;
  SL_fCstate      state[1];
} smBaseState;

typedef struct smBaseOrient {
  SEM_ID          sm_sem;
  float           ts;
  SL_fquat        orient[1];
} smBaseOrient;

typedef struct smUserGraphics {
  SEM_ID          sm_sem;
  int             n_entries;
  int             n_bytes_used;
  char            name[MAX_N_MESSAGES+1][20];
  int             moff[MAX_N_MESSAGES+1];
  unsigned char   buf[MAX_BYTES_USER_GRAPHICS];
} smUserGraphics;

typedef struct smMessage {
  SEM_ID          sm_sem;
  int             n_msgs;
  int             n_bytes_used;
  char            name[MAX_N_MESSAGES+1][20];
  int             moff[MAX_N_MESSAGES+1];
  unsigned char   buf[MAX_BYTES_MESSAGES];
} smMessage;

typedef struct smOscilloscope {
  SEM_ID          sm_sem;
  int             n_entries;
  SL_oscEntry     entries[1];
} smOscilloscope;


#ifdef __cplusplus
extern "C" {
#endif

  /* external variables */
  extern SEM_ID             sm_joint_des_state_ready_sem;
  extern SEM_ID             sm_sjoint_des_state_ready_sem;
  extern SEM_ID             sm_raw_blobs_ready_sem;
  extern SEM_ID             sm_learn_invdyn_sem;
  extern SEM_ID             sm_learn_blob2body_sem;
  extern SEM_ID             sm_task_servo_sem;  /* task servo synchronization */
  extern SEM_ID             sm_vision_servo_sem;  /* vision servo synchronization */
  extern SEM_ID             sm_openGL_servo_sem;  /* openGL servo synchronization */
  extern SEM_ID             sm_motor_servo_sem;  /* motor servo synchronization */
  extern SEM_ID             sm_simulation_servo_sem;  /* simulation servo synchronization */
  extern SEM_ID             sm_ros_servo_sem;  /* ros servo synchronization */
  extern SEM_ID             sm_pause_sem;  /* needed to signal pause to simulation servo */
  extern SEM_ID             sm_user_graphics_ready_sem; /* signals user graphics available */
  extern SEM_ID             sm_openGL_message_ready_sem; /* signals message to openGl servo */
  extern SEM_ID             sm_vision_message_ready_sem; /* signals message to openGl servo */
  extern SEM_ID             sm_simulation_message_ready_sem; /* signals message to sim servo */
  extern SEM_ID             sm_task_message_ready_sem; /* signals message to task servo */
  extern SEM_ID             sm_ros_message_ready_sem; /* signals message to ros servo */
  extern SEM_ID             sm_motor_message_ready_sem; /* signals message to motor servo */
  extern SEM_ID             sm_objects_ready_sem; /* signals that objects are ready for read */
  extern SEM_ID             sm_init_process_ready_sem; /* for starting up the SL processes */
  extern SEM_ID             sm_oscilloscope_sem; /* for writing to the oscilloscope */

  extern smVisionBlobs     *sm_vision_blobs;
  extern SEM_ID             sm_vision_blobs_sem;
  extern SL_fVisionBlob    *sm_vision_blobs_data;
  
  extern smVisionBlobsaux  *sm_vision_blobs_aux;  
  extern SEM_ID             sm_vision_blobs_aux_sem;
  extern SL_fVisionBlobaux *sm_vision_blobs_data_aux;
  
  extern smCartStates      *sm_cart_states;
  extern SEM_ID             sm_cart_states_sem;
  extern SL_fCstate        *sm_cart_states_data;
  
  extern smRawBlobs        *sm_raw_blobs;
  extern SEM_ID             sm_raw_blobs_sem;
  extern fBlob3D           *sm_raw_blobs_data;
  
  extern smRawBlobs2D      *sm_raw_blobs2D;
  extern SEM_ID             sm_raw_blobs2D_sem;
  extern fBlob2D           *sm_raw_blobs2D_data;
  
  extern smSJointDesStates *sm_sjoint_des_state;
  extern SEM_ID             sm_sjoint_des_state_sem;
  extern SL_fSDJstate      *sm_sjoint_des_state_data;
  
  extern smJointStates     *sm_joint_state;
  extern SEM_ID             sm_joint_state_sem;
  extern SL_fJstate        *sm_joint_state_data;
  
  extern smJointDesStates  *sm_joint_des_state;
  extern SEM_ID             sm_joint_des_state_sem;
  extern SL_fDJstate       *sm_joint_des_state_data;
  
  extern smJointSimStates  *sm_joint_sim_state;
  extern SEM_ID             sm_joint_sim_state_sem;
  extern SL_fJstate        *sm_joint_sim_state_data;
  
  extern smMiscSensors     *sm_misc_sensor;
  extern SEM_ID             sm_misc_sensor_sem;
  extern float             *sm_misc_sensor_data;
  
  extern smMiscSimSensors  *sm_misc_sim_sensor;
  extern SEM_ID             sm_misc_sim_sensor_sem;
  extern float             *sm_misc_sim_sensor_data;
  
  extern smContacts        *sm_contacts;
  extern SEM_ID             sm_contacts_sem;
  extern contactShort      *sm_contacts_data;
  
  extern smBaseState       *sm_base_state;
  extern SEM_ID             sm_base_state_sem;
  extern SL_fCstate        *sm_base_state_data;
  
  extern smBaseOrient      *sm_base_orient;
  extern SEM_ID             sm_base_orient_sem;
  extern SL_fquat          *sm_base_orient_data;
  
  extern smUserGraphics    *sm_user_graphics;
  extern SEM_ID             sm_user_graphics_sem;

  extern smMessage         *sm_simulation_message;
  extern SEM_ID             sm_simulation_message_sem;
  extern smMessage         *sm_openGL_message;
  extern SEM_ID             sm_openGL_message_sem;
  extern smMessage         *sm_task_message;
  extern SEM_ID             sm_task_message_sem;
  extern smMessage         *sm_ros_message;
  extern SEM_ID             sm_ros_message_sem;
  extern smMessage         *sm_motor_message;
  extern SEM_ID             sm_motor_message_sem;
  extern smMessage         *sm_vision_message;
  extern SEM_ID             sm_vision_message_sem;

  extern smDCommands       *sm_des_commands;
  extern SEM_ID             sm_des_commands_sem;

  extern smROSState        *sm_ros_state;
  extern SEM_ID             sm_ros_state_sem;

  extern smOscilloscope    *sm_oscilloscope;
  extern SEM_ID             sm_oscilloscope_sem;

  int   init_shared_memory(void);
  void  sendMessageToServo(smMessage *sm_message, SEM_ID sm_message_sem,
			   SEM_ID ready_sem, 
			   char *message, void *buf, int n_bytes);
  void  sendMessageSimulationServo(char *message, void *buf, int n_bytes);
  void  sendMessageTaskServo(char *message, void *buf, int n_bytes);
  void  sendMessageMotorServo(char *message, void *buf, int n_bytes);
  void  sendMessageOpenGLServo(char *message, void *buf, int n_bytes);
  void  sendMessageROSServo(char *message, void *buf, int n_bytes);
  void  sendMessageVisionServo(char *message, void *buf, int n_bytes);

  
#ifdef __cplusplus
}
#endif

#endif  // _SL_shared_memory_
