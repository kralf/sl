/*!=============================================================================
  ==============================================================================

  \file    SL.h

  \author  Stefan Schaal
  \date    Jan 1999

  ==============================================================================
  \remarks
  
Generic definitions to create SL simulations
  
  ============================================================================*/
  
#ifndef _SL_
#define _SL_

#include "utility.h"

/*! General header file for rigid body dynamics simulation environment */
enum ConfigFile {
  CONFIGFILES,
  GAINS,
  SENSORCALIBRATION,
  SENSORFILTERS,
  LINKPARAMETERS,
  SENSOROFFSETS,
  WHICHDOFS,
  OBJECTS,
  SERVOPARAMETERS,
  STEREOPARAMETERS,
  PARAMETERPOOL,
  CONTACTS,

  N_CONFIG_ENUMS
};

#define N_CONFIG_FILES (N_CONFIG_ENUMS-1)

#define START_INDEX 1
#define N_CART 3
#define _X_ (0+START_INDEX)
#define _Y_ (1+START_INDEX)
#define _Z_ (2+START_INDEX)
#define _A_ (0+START_INDEX)
#define _B_ (1+START_INDEX)
#define _G_ (2+START_INDEX)

#define N_QUAT 4
#define _Q0_ 1
#define _Q1_ 2
#define _Q2_ 3
#define _Q3_ 4

#define _QW_ 1
#define _QX_ 2
#define _QY_ 3
#define _QZ_ 4

#define RIGHT 1
#define LEFT  2

/*! defines for the servo frequencies */
#define R1TO1 1
#define R1TO2 2
#define R1TO3 3
#define R1TO4 4
#define R1TO5 5
#define R60HZ 60

/*! defines for the preference and config files */
#define CONFIG   "config/"
#define TERRAINS "terrain/"
#define PREFS    "prefs/"

/*! defines that are used to parse the config and prefs files */
#define MIN_THETA    1
#define MAX_THETA    2
#define THETA_OFFSET 3

#define LINEAR       1

#define SENSOR       1
#define ACTUATOR     2
#define MOMENTARM    3
#define MOUNTPOINT   4
#define THETA0       5
#define LOADCELL     6

#define DOUBLE2FLOAT 1
#define FLOAT2DOUBLE 2

#define STRING20    20   //!< used to allocate 20 character string
#define STRING40    40   //!< used to allocate 40 character string
#define STRING60    60   //!< used to allocate 60 character string
#define STRING90    80   //!< used to allocate 80 character string
#define STRING100  100   //!< used to allocate 100 character string

#ifdef VX
#define MAX_BYTES_USER_GRAPHICS 5000
#else
#define MAX_BYTES_USER_GRAPHICS 100000
#endif
#define MAX_BYTES_MESSAGES      10000
#define MAX_N_MESSAGES          100

#define OSC_BUFFER_SIZE 10000
#define OSC_SM_BUFFER_SIZE (4*OSC_BUFFER_SIZE)


/* The data structures */
typedef struct { /*!< joint space state for each DOF */
  double   th;   /*!< theta */
  double   thd;  /*!< theta-dot */
  double   thdd; /*!< theta-dot-dot */
  double   ufb;  /*!< feedback portion of command */
  double   u;    /*!< torque command */
  double   load; /*!< sensed torque */
} SL_Jstate;

typedef struct { /*!< joint space state for each DOF */
  float   th;   /*!< theta */
  float   thd;  /*!< theta-dot */
  float   thdd; /*!< theta-dot-dot */
  float   ufb;  /*!< feedback portion of command */
  float   u;    /*!< torque command */
  float   load; /*!< sensed torque */
} SL_fJstate;

typedef struct { /*!< desired values for controller */
  double   th;   /*!< theta */
  double   thd;  /*!< theta-dot */
  double   thdd; /*!< theta-dot-dot */
  double   uff;  /*!< feedforward torque command */
  double   uex;  /*!< externally imposed torque */
} SL_DJstate;

typedef struct { /*!< desired values for controller */
  float   th;    /*!< theta */
  float   thd;   /*!< theta-dot */
  float   thdd;  /*!< theta-dot-dot */
  float   uff;   /*!< feedforward torque command */
  float   uex;   /*!< externally imposed torque */
} SL_fDJstate;

typedef struct {      /*!< desired values for controller: short version for faster communication */
  char    status;     /*!< valid data or not: needed for multi processing */
  float   th;         /*!< desired theta */
  float   thd;        /*!< desired velocity */
  float   uff;        /*!< feedforward command */
  int     zero_ufb_P; /*!< zero ufb proportional part */
  int     zero_ufb_D; /*!< zero ufb derivative part */
} SL_fSDJstate;

typedef struct { /*!< desired state for optimization */
  double   th;   /*!< desired theta */
  double   w;    /*!< feedforward command */
} SL_OJstate;

typedef struct { /*!< desired state for optimization */
  float   th;   /*!< desired theta */
  float   w;    /*!< feedforward command */
} SL_fOJstate;

typedef struct { /*!< Cartesian state */
  double   x[N_CART+1];    /*!< Position [x,y,z] */
  double   xd[N_CART+1];   /*!< Velocity */
  double   xdd[N_CART+1];  /*!< Acceleration */
} SL_Cstate;

typedef struct { /*!< Cartesian state */
  float   x[N_CART+1];    /*!< Position [x,y,z] */
  float   xd[N_CART+1];   /*!< Velocity */
  float   xdd[N_CART+1];  /*!< Acceleration */
} SL_fCstate;

typedef struct { /*!< Cartesian orientation */
  double   a[N_CART+1];    /*!< Position [alpha,beta,gamma] */
  double   ad[N_CART+1];   /*!< Velocity */
  double   add[N_CART+1];  /*!< Acceleration */
} SL_Corient;

typedef struct { /*!< Quaternion orientation */
  double   q[N_QUAT+1];    /*!< Position [q0,q1,q2,q3] */
  double   qd[N_QUAT+1];   /*!< Velocity */
  double   qdd[N_QUAT+1];  /*!< Acceleration */
  double   ad[N_CART+1];   /*!< Angular Velocity [alpha,beta,gamma] */
  double   add[N_CART+1];  /*!< Angular Acceleration */
} SL_quat;

typedef struct { /*!< Cartesian orientation */
  float   a[N_CART+1];    /*!< Position [alpha,beta,gamma] */
  float   ad[N_CART+1];   /*!< Velocity */
  float   add[N_CART+1];  /*!< Acceleration */
} SL_fCorient;

typedef struct { /*!< Quaternion orientation */
  float   q[N_QUAT+1];    /*!< Position [q0,q1,q2,q3] */
  float   qd[N_QUAT+1];   /*!< Velocity */
  float   qdd[N_QUAT+1];  /*!< Acceleration */
  float   ad[N_CART+1];   /*!< Angular Velocity [alpha,beta,gamma] */
  float   add[N_CART+1];  /*!< Angular Acceleration */
} SL_fquat;

typedef struct { /*!< Vision Blob */
  char       status;
  SL_Cstate  blob;
} SL_VisionBlob;

typedef struct { /*!< Vision Blob */
  char        status;
  SL_fCstate  blob;
} SL_fVisionBlob;

typedef struct { /*!< 2D Vision Blob */
  char        status[2+1];
  double blob[2+1][2+1];
} SL_VisionBlobaux;

typedef struct { /*!< 2D Vision Blob */
  char        status[2+1];
  float blob[2+1][2+1];
} SL_fVisionBlobaux;

/*! a structure for raw 3D vision blobs */
typedef struct {
  char       status;
  double     x[N_CART+1];
} Blob3D;

/*! a structure for raw 3D vision blobs */
typedef struct {
  char     status;
  float    x[N_CART+1];
} fBlob3D;

/*! a structure for raw 2D vision blobs */
typedef struct {
  char       status;
  double     x[2+1];
} Blob2D;

/*! a structure for raw 2D vision blobs */
typedef struct {
  char     status;
  float    x[2+1];
} fBlob2D;

typedef struct { /*!< Link parameters */
  double   m;             /*!< Mass */
  double   mcm[N_CART+1]; /*!< Center of mass multiplied with the mass */
  double   inertia[N_CART+1][N_CART+1];  /*!< Moment of inertia */
  double   vis;           /*!< viscous friction term */
  double   coul;          /*!< coulomb friction */
  double   stiff;         /*!< spring stiffness */
  double   cons;          /*!< constant term */
} SL_link;

typedef struct { /*!< Link parameters */
  float   m;          /*!< Mass */
  float   mcm[N_CART+1]; /*!< Center of mass multiplied with the mass */
  float   inertia[N_CART+1][N_CART+1];  /*!< Moment of inertia */
  float   vis;        /*!< viscous friction term */
} SL_flink;

typedef struct { /*!< end-effector parameters */
  double   m;             /*!< Mass */
  double   mcm[N_CART+1]; /*!< mass times Center of mass */
  double   x[N_CART+1];   /*!< end position of endeffector in local coordinates*/
  double   a[N_CART+1];   /*!< orientation of the tool in Euler Angles (x-y-z) */
  int      c[2*N_CART+1]; /*!< constraint status of the endeffector */
  double   cf[N_CART+1];  /*!< contact force in world coordinates at the endeffector */
  double   ct[N_CART+1];  /*!< contact torques in world coordinates at the endeffector */
} SL_endeff;

typedef struct { /*!< end-effector parameters */
  float   m;             /*!< Mass */
  float   mcm[N_CART+1]; /*!< mass times Center of mass */
  float   x[N_CART+1];   /*!< end position of endeffector in local coordinates */
  float   a[N_CART+1];   /*!< orientation of the tool in Euler Angles (x-y-z) */
  int     c[2*N_CART+1]; /*!< constraint status of the endeffector */
  float   cf[N_CART+1];  /*!< contact force in world coordinates at the endeffector */
  float   ct[N_CART+1];  /*!< contact torques in world coordinates at the endeffector */
} SL_fendeff;

typedef struct { /*!< external forces */
  double   f[N_CART+1];   /*!< external forces */
  double   t[N_CART+1];   /*!< external torques */
} SL_uext;

typedef struct {    /*!< oscilloscope data entry structure */
  char    name[40]; /*!< name of variable to be plotted */
  float   v;        /*!< value */
  double  ts;       /*!< time stamp in seconds */
  int     plotID;   /*!< ID of plot to be used for plotting */
} SL_oscEntry;


#ifdef __cplusplus
extern "C" {
#endif

  /* variables shared by all SL programs */
  /* external variables */

  /* constants that define the simulation */
  extern const int    n_dofs;              /* number of degrees of freedom */
  extern const int    n_dofs_est;          /* number of DOFs to be estimated */
  extern const int    n_dofs_est_skip;     /* degrees of freedom to be skipped by estimation */
  extern const int    n_endeffs;           /* number of endeffectors */
  extern const int    n_links;             /* number of links in the robot */
  extern const int    n_misc_sensors;      /* number of miscelleneous sensors in the robot */
  extern const int    invdyn_servo_ratio;  /* divides base freq. to obtain invdyn servo freq.*/
  extern const char   vision_default_pp[]; /* default script for vision processing */
  extern const int    max_blobs;           /* maximal number of blobs in the vision system */
  extern const int    d2a_cm;              /* which D/A channel is used by motor servo */
  extern const int    d2a_ct;              /* which D/A channel is used by the task servo */
  extern const int    d2a_cv;              /* which D/A channel is used by the vision servo */
  extern const int    d2a_cr;              /* which D/A channel is used by the ros servo */
  extern const int    floating_base_flag;  /* indicates floating base robot */


  /* generic external variables */
  extern char         *robot_name;
  extern char          servo_name[];
  extern int           parent_process_id;   /* process id of main program */
  extern int           real_robot_flag;     /* signals that program is used for real robot */
  extern int           no_graphics_flag;    /* signals that no graphics are used */
  extern int           servo_base_rate;     /* base freq. of servos */
  extern int           task_servo_ratio;    /* divides base freq. to obtain task servo freq.*/
  extern char          joint_names[][20];
  extern char          cart_names[][20];
  extern char          misc_sensor_names[][20];
  extern SL_DJstate    joint_default_state[];
  extern SL_OJstate    joint_opt_state[];
  extern SL_Jstate     joint_state[];
  extern SL_DJstate    joint_des_state[];
  extern double        joint_range[][3+1];
  extern double        u_max[];
  extern SL_endeff     endeff[];
  extern SL_Jstate     joint_sim_state[];
  extern SL_VisionBlob blobs[];
  extern Matrix        J;
  extern Matrix        dJdt;
  extern Matrix        Jbase;
  extern Matrix        dJbasedt;
  extern Matrix        Jbasedes;
  extern Matrix        Jdes;
  extern Matrix        Jcog;
  extern Matrix        Jcogdes;
  extern Matrix        Jbasecog;
  extern Matrix        Jbasecogdes;
  extern Matrix        link_pos;
  extern Matrix        link_pos_des;
  extern Matrix        link_pos_sim;
  extern Matrix        joint_cog_mpos;
  extern Matrix        joint_cog_mpos_des;
  extern Matrix        joint_cog_mpos_sim;
  extern Matrix        joint_origin_pos;
  extern Matrix        joint_origin_pos_des;
  extern Matrix        joint_origin_pos_sim;
  extern Matrix        joint_axis_pos;
  extern Matrix        joint_axis_pos_des;
  extern Matrix        joint_axis_pos_sim;
  extern Matrix        Alink[];
  extern Matrix        Alink_des[];
  extern Matrix        Alink_sim[];
  extern Matrix        Adof[];
  extern Matrix        Adof_des[];
  extern Matrix        Adof_sim[];
  extern int           link2endeffmap[];
  extern char          current_vision_pp[];
  extern char          blob_names[][20];
  extern Blob3D        raw_blobs[];
  extern Blob2D        raw_blobs2D[][2+1];
  extern char          link_names[][20];
  extern SL_Cstate     cart_state[];
  extern SL_quat       cart_orient[];
  extern SL_Cstate     cart_des_state[];
  extern SL_quat       cart_des_orient[];
  extern SL_Cstate     cart_target_state[];
  extern SL_quat       cart_target_orient[];
  extern SL_link       links[];
  extern int           whichDOFs[]; 
  extern SL_Cstate     base_state; 
  extern SL_quat       base_orient;
  extern SL_uext       uext[];
  extern SL_uext       uext_sim[];
  extern double        misc_sensor[];
  extern double        misc_sim_sensor[];
  extern SL_Cstate     cog;
  extern SL_Cstate     cog_des;
  extern double        gravity;
  extern int           n_contacts;
  extern int           prismatic_joint_flag[]; 
  extern int           jointPredecessor[];

  /* Function prototypes */
  void  where(void);
  void  where_des(void);
  void  cwhere(void);
  void  bwhere(int flag);
  void  lwhere(void);
  int   check_range(SL_DJstate *des);
  int   init_commands(void);
  void  initContacts(void);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_ */
