/*!=============================================================================
  ==============================================================================

  \file    SL_user_common.h

  \author 
  \date   

  ==============================================================================
  \remarks

  common variables and functions shared by many SL modules. This file
  needs to be included in SL_user_common.c 

  ============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

  /* important variable that define the robot */
  const int  n_dofs = N_DOFS;
  const int  n_dofs_est = N_DOFS_EST;
  const int  n_dofs_est_skip = N_DOFS_EST_SKIP;
  const int  n_links = N_LINKS;
  const int  n_endeffs = N_ENDEFFS;
  const int  n_cameras = N_CAMERAS;
  const int  max_blobs = MAX_BLOBS;
  const int  n_misc_sensors = N_MISC_SENSORS;
  const char vision_default_pp[]=VISION_DEFAULT_PP;

  const int  d2a_cm = D2A_CM;
  const int  d2a_cv = D2A_CV;
  const int  d2a_ct = D2A_CT;
  const int  d2a_cr = D2A_CR;

#include "Floating_Base.h"

  /* global variables */
  char         *robot_name = ROBOT_NAME;
  char          servo_name[100]="";
  int           parent_process_id = 0;
  int           real_robot_flag = FALSE;        /* signals that program is used for real robot */
  int           no_graphics_flag = FALSE;       /* signals that no graphics is used */
  int           task_servo_ratio = TASK_SERVO_RATIO;
  int           servo_base_rate = SERVO_BASE_RATE;

  SL_Jstate     joint_state[N_DOFS+1];          /* current states */
  SL_DJstate    joint_des_state[N_DOFS+1];      /* desired states */
  SL_endeff     endeff[N_ENDEFFS+1];            /* endeffector structure */
  SL_Jstate     joint_sim_state[N_DOFS+1];      /* the state of the sim. robot */
  SL_DJstate    joint_default_state[N_DOFS+1];  /* posture for startup */
  double        joint_range[N_DOFS+1][3+1];     /* various info on joint limits */
  double        u_max[N_DOFS+1];                /* actuator output limits */
  SL_VisionBlob blobs[MAX_BLOBS+1];             /* blob info from vision */
  Matrix        J;                              /* kinematic Jacobian */
  Matrix        dJdt;                           /* time derivative of Jacobian */
  Matrix        Jbase;                          /* kinematic Jacobian of base*/
  Matrix        dJbasedt;                       /* time derivative of base Jacobian */
  Matrix        Jbasedes;                       /* kinematic Jacobian of base*/
  Matrix        Jdes;		                /* Jacobian based on des. state */
  Matrix        Jcog;		                /* COG Jacobian */
  Matrix        Jcogdes;		        /* COG base Jacobian based on desired states */
  Matrix        Jbasecog;	                /* COG base Jacobian */
  Matrix        Jbasecogdes;		        /* COG Jacobian based on desired states */
  Matrix        link_pos;                       /* Cart. pos of links */
  Matrix        link_pos_des;                   /* desired cart. pos of links */
  Matrix        link_pos_sim;                   /* simulated cart. pos of links */
  Matrix        joint_cog_mpos;                 /* vector of mass*COG of each joint */
  Matrix        joint_origin_pos;               /* vector of pos. of local joint coord.sys */
  Matrix        joint_axis_pos;                 /* unit vector of joint rotation axis */
  Matrix        joint_cog_mpos_des;             /* vector of mass*COG of each joint based on desireds*/
  Matrix        joint_origin_pos_des;           /* vector of pos. of local joint coord.sys based on des.*/
  Matrix        joint_axis_pos_des;             /* unit vector of joint rotation axis based on des.*/
  Matrix        joint_cog_mpos_sim;             /* vector of mass*COG of each joint based on simsate*/
  Matrix        joint_origin_pos_sim;           /* vector of pos. of local joint coord.sys based on sim.*/
  Matrix        joint_axis_pos_sim;             /* unit vector of joint rotation axis based on sim.*/
  Matrix        Alink[N_LINKS+1];               /* homogeneous transformation matrices for all links */
  Matrix        Alink_des[N_LINKS+1];           /* homogeneous transformation matrices for all links */
  Matrix        Alink_sim[N_LINKS+1];           /* homogeneous transformation matrices for all links */
  Matrix        Adof[N_LINKS+1];                /* homogeneous transformation matrices for all dofs */
  Matrix        Adof_des[N_LINKS+1];            /* homogeneous transformation matrices for all dofs */
  Matrix        Adof_sim[N_LINKS+1];            /* homogeneous transformation matrices for all dofs */
  SL_Cstate     cart_state[N_ENDEFFS+1];        /* endeffector state */
  SL_quat       cart_orient[N_ENDEFFS+1];       /* endeffector orientation */
  SL_Cstate     cart_des_state[N_ENDEFFS+1];    /* endeff.state based on des.state */
  SL_quat       cart_des_orient[N_ENDEFFS+1];   /* endeff.orient based on des.state */
  SL_Cstate     cart_target_state[N_ENDEFFS+1]; /* endeff.target state for inv.kin */
  SL_quat       cart_target_orient[N_ENDEFFS+1];/* endeff.target orient for inv. kin */
  char          current_vision_pp[30];          /* vision post processing specification */
  SL_OJstate    joint_opt_state[N_DOFS+1];      /* rest state for optimization */
  SL_link       links[N_DOFS+1];                /* specs of links: mass, inertia, cm */
  Blob3D        raw_blobs[MAX_BLOBS+1];         /* raw blobs 3D of vision system */
  Blob2D        raw_blobs2D[MAX_BLOBS+1][2+1];  /* raw blobs 2D of vision system */
  int           whichDOFs[N_DOFS+1];            /* which DOFS does a servo compute motor commands for */
  SL_Cstate     base_state;                     /* cartesian state of base coordinate system */
  SL_quat       base_orient;                    /* cartesian orientation of base coordinate system */
  SL_uext       uext[N_DOFS+1];                 /* measured external forces on 
						   every DOF in local coordinates */
  SL_uext       uext_sim[N_DOFS+1];             /* simulated external forces on 
						   every DOF, i.e., due to contact
						   or simulated virtual forces, in
						   global coordinates*/
  double        misc_sensor[N_MISC_SENSORS+1];  /* additional sensors */
  double        misc_sim_sensor[N_MISC_SENSORS+1];  /* additional sensors, simulated */
  SL_Cstate     cog;                            /* center of gravity */
  SL_Cstate     cog_des;                        /* center of gravity based on desired states*/

  double        gravity = G;

  int           n_contacts = N_LINKS;           /* number of contacts and default initialization */
  int           prismatic_joint_flag[N_DOFS+1]; /* TRUE/FALSE indicator for prismatic joints */
  int           jointPredecessor[N_DOFS+1];     /* lists for each joint its predessor joint ID */

#ifdef __cplusplus
}
#endif
 
