/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_common.c

  \author 
  \date   

  ==============================================================================
  \remarks

      File includes functions that are shared by many processors 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "utility_macros.h"
#include "SL_man.h"
#include "mdefs.h"

// global variables and their default assignments
char config_files[][100] = {
  {"ConfigFilesSim.cf"},
  {"Gains.cf"},
  {"SensorCalibration.cf"},
  {"SensorFilter.cf"},
  {"LinkParameters.cf"},
  {"SensorOffset.cf"},
  {"WhichDOFs.cf"},
  {"Objects.cf"},
  {"ServoParameters.cf"},
  {"StereoParameters.cf"},
  {"ParameterPool.cf"},
  {"Contacts.cf"}
};

// local variables
static char config_file_tags[][40]= {
  {"ConfigFiles"},
  {"Gains"}, 
  {"SensorCalibration"},
  {"SensorFilter"},
  {"LinkParameters"},
  {"SensorOffset"},
  {"WhichDOFs"},
  {"Objects"},
  {"ServoParameters"},
  {"StereoParameters"},
  {"ParameterPool"},
  {"Contacts"}
};

static char parameter_pool_vars[] = ".parameter_pool_vars";

/* external variables */
extern int servo_enabled;

/* global functions */

/* local functions */
static void bwhere_sim(void);

/*!*****************************************************************************
 *******************************************************************************
\note  where
\date  Feb 1999
\remarks 

 various functions to print info about all DOFs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
where_utility(int start, int n_dofs)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: motor servo is not running!!\n");
  }

  printf("Current States:\n");

  for (i=start; i<=start+n_dofs-1; ++i) {

    printf("%2d: %5s: th=% 5.3f  thd=% 6.3f  load=% 6.2f  u=% 6.2f  ff=% 6.2f\n",
	   i,joint_names[i],
	   joint_state[i].th,
	   joint_state[i].thd,
	   joint_state[i].load,
	   joint_state[i].u,
	   joint_des_state[i].uff);

  }
  printf("\n");

  return TRUE;

}

void
where(void)

{
  where_utility(1,n_dofs);
}

int
where_des_utility(int start, int n_dofs)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: motor servo is not running!!\n");
  }

  printf("Desired States:\n");

  for (i=start; i<=start+n_dofs-1; ++i) {

    printf("%5s: th=% 5.3f  thd=% 6.3f  thdd=% 9.3f  ff=% 6.2f\n",
	   joint_names[i],
	   joint_des_state[i].th,
	   joint_des_state[i].thd,
	   joint_des_state[i].thdd,
	   joint_des_state[i].uff);

  }
  printf("\n");

  return TRUE;

}

void
where_des(void)

{
  where_des_utility(1,n_dofs);
}

/*!*****************************************************************************
 *******************************************************************************
\note  cwhere
\date  Feb 1999
\remarks 

 print the cartesian information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
cwhere(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current Cartesian States:\n");

  for (i=1; i<=n_endeffs; ++i) {

    printf("%10s: x=% 5.3f (% 5.3f)    y=% 5.3f (% 5.3f)    z=% 5.3f (% 5.3f)\n",
	   cart_names[i],
	   cart_state[i].x[_X_],cart_des_state[i].x[_X_],
	   cart_state[i].x[_Y_],cart_des_state[i].x[_Y_],
	   cart_state[i].x[_Z_],cart_des_state[i].x[_Z_]);
    printf("            xd=% 5.3f (% 5.3f)   yd=% 5.3f (% 5.3f)   zd=% 5.3f (% 5.3f)\n",
	   cart_state[i].xd[_X_],cart_des_state[i].xd[_X_],
	   cart_state[i].xd[_Y_],cart_des_state[i].xd[_Y_],
	   cart_state[i].xd[_Z_],cart_des_state[i].xd[_Z_]);
    printf("            xdd=% 5.3f (% 5.3f)  ydd=% 5.3f (% 5.3f)  zdd=% 5.3f (% 5.3f)\n",
	   cart_state[i].xdd[_X_],cart_des_state[i].xdd[_X_],
	   cart_state[i].xdd[_Y_],cart_des_state[i].xdd[_Y_],
	   cart_state[i].xdd[_Z_],cart_des_state[i].xdd[_Z_]);
    printf("\n");


    printf("            q0=% 5.3f (% 5.3f)    q1=% 5.3f (% 5.3f)    q2=% 5.3f (% 5.3f)    q3=% 5.3f (% 5.3f)\n",
	   cart_orient[i].q[1],cart_des_orient[i].q[1],
	   cart_orient[i].q[2],cart_des_orient[i].q[2],
	   cart_orient[i].q[3],cart_des_orient[i].q[3],
	   cart_orient[i].q[4],cart_des_orient[i].q[4]);
    printf("            qd0=% 5.3f (% 5.3f)   qd1=% 5.3f (% 5.3f)   qd2=% 5.3f (% 5.3f)   qd3=% 5.3f (% 5.3f)\n",
	   cart_orient[i].qd[1],cart_des_orient[i].qd[1],
	   cart_orient[i].qd[2],cart_des_orient[i].qd[2],
	   cart_orient[i].qd[3],cart_des_orient[i].qd[3],
	   cart_orient[i].qd[4],cart_des_orient[i].qd[4]);
    printf("            qdd0=% 5.3f (% 5.3f)  qdd1=% 5.3f (% 5.3f)  qdd2=% 5.3f (% 5.3f)  qdd3=% 5.3f (% 5.3f)\n",
	   cart_orient[i].qdd[1],cart_des_orient[i].qdd[1],
	   cart_orient[i].qdd[2],cart_des_orient[i].qdd[2],
	   cart_orient[i].qdd[3],cart_des_orient[i].qdd[3],
	   cart_orient[i].qdd[4],cart_des_orient[i].qdd[4]);
    printf("\n");
    printf("\n");


  }
  printf("\n");

}


/*!*****************************************************************************
 *******************************************************************************
\note  check_range
\date  Dec 1997
   
\remarks 

          checks the desired states for out of range values

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     des   : array of SL_DJstates

 ******************************************************************************/
int
check_range(SL_DJstate *des)

{

  int i,j;
  int flag = TRUE;

  for (i=1; i<=n_dofs; ++i) {

    if (des[i].th > joint_range[i][MAX_THETA]) {
      flag = FALSE;
      des[i].th = joint_range[i][MAX_THETA];
    }
    if (des[i].th < joint_range[i][MIN_THETA]) {
      flag = FALSE;
      des[i].th = joint_range[i][MIN_THETA];
    }

    if (des[i].uff > u_max[i]) {
      flag = FALSE;
      des[i].uff = u_max[i];
    }
    if (des[i].uff < -u_max[i]) {
      flag = FALSE;
      des[i].uff = -u_max[i];
    }

  }

  return flag;

}

/*!*****************************************************************************
 *******************************************************************************
\note  bwhere
\date  Feb 1999
\remarks 

 print the blob information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     flag : show all blobs (otherwise only those with status 1

 ******************************************************************************/
static void
bwhere_sim(void)
{
  bwhere(FALSE);
}
void
bwhere(int flag)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  if (flag != 0 && flag != 1)
    flag = 0;

  printf("Current Blob States:\n");

  for (i=1; i<=max_blobs; ++i) {

    if (flag || blobs[i].status) {
      
      printf("%10s: Status = %d\n",blob_names[i],blobs[i].status);
      printf("             x=% 5.3f    y=% 5.3f    z=% 5.3f\n",
	     blobs[i].blob.x[_X_],
	     blobs[i].blob.x[_Y_],
	     blobs[i].blob.x[_Z_]);
      printf("            xd=% 5.3f   yd=% 5.3f   zd=% 5.3f\n",
	     blobs[i].blob.xd[_X_],
	     blobs[i].blob.xd[_Y_],
	     blobs[i].blob.xd[_Z_]);
      printf("            xdd=% 5.3f  ydd=% 5.3f  zdd=% 5.3f\n",
	     blobs[i].blob.xdd[_X_],
	     blobs[i].blob.xdd[_Y_],
	     blobs[i].blob.xdd[_Z_]);
      printf("\n");
      
      printf("\n");
    
    }
  }
}


/*!*****************************************************************************
 *******************************************************************************
\note  lwhere
\date  Feb 1999
\remarks 

 print the cartesian information about the links

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
lwhere(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current (Desired) Link Cartesian States:\n");

  for (i=0; i<=n_links; ++i) {

    printf("%2d: %20s: x=% 5.3f (% 5.3f)   y=% 5.3f  (% 5.3f)   z=% 5.3f (% 5.3f)\n",
	   i,link_names[i],
	   link_pos[i][_X_],link_pos_des[i][_X_],
	   link_pos[i][_Y_],link_pos_des[i][_Y_],
	   link_pos[i][_Z_],link_pos_des[i][_Z_]);
  }
  printf("\n");

}


/*!*****************************************************************************
 *******************************************************************************
\note  linfo
\date  Feb 1999
\remarks 

 print the information about all links

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
linfo(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current m*cog,axis,origin information of each joint:\n");

  for (i=1; i<=n_dofs; ++i) {

    printf("%10s: mcogx=% 5.3f    mcogy=% 5.3f    mcogz=% 5.3f\n",
	   joint_names[i],
	   joint_cog_mpos[i][_X_],
	   joint_cog_mpos[i][_Y_],
	   joint_cog_mpos[i][_Z_]);
    printf("%10s  axisx=% 5.3f    axisy=% 5.3f    axisz=% 5.3f\n",
	   "",
	   joint_axis_pos[i][_X_],
	   joint_axis_pos[i][_Y_],
	   joint_axis_pos[i][_Z_]);
    printf("%10s  origx=% 5.3f    origy=% 5.3f    origz=% 5.3f\n",
	   "",
	   joint_origin_pos[i][_X_],
	   joint_origin_pos[i][_Y_],
	   joint_origin_pos[i][_Z_]);
  }
  printf("\n");

}


/*!*****************************************************************************
 *******************************************************************************
\note  where_misc
\date  March 2003
\remarks 

 print the state of miscellaneous sensors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
where_misc(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current Miscellaneous Sensor States:\n");

  for (i=1; i<=n_misc_sensors; ++i) {

    printf("%3d: %20s = % 5.3f\n",
	   i,
	   misc_sensor_names[i],
	   misc_sensor[i]);
  }
  printf("\n");

}


/*!*****************************************************************************
 *******************************************************************************
\note  init_commands
\date  Feb 1999
\remarks 

 initializes man pages for commands

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_commands(void)
{
  addToMan("where","print all current state information",where);
  addToMan("where_des","print all desired joint information",where_des);
  addToMan("bwhere","cartesian state of vision blobs",bwhere_sim);
  addToMan("rbwhere","current state of vision blobs",rbwhere);
  addToMan("rbwhere2D","current state of 2D vision blobs",rbwhere2D);
  addToMan("where_base","current state of base coordiante system",where_base);
  addToMan("where_misc","current state of miscellanious sensors",where_misc);
  addToMan("where_cog","current state of the COG",where_cog);
  addToMan("print_J","current state of Jacobian",print_J);
  if (strcmp(servo_name,"motor") != 0) {
    addToMan("cwhere","cartesian state of endeffectors",cwhere);
    addToMan("lwhere","cartesian state of links",lwhere);
    addToMan("linfo","axis,cog,orgin info of each link",linfo);
  }


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_sensor_offsets
\date  May 2000
\remarks 

parses the sensor offset configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : file name of file where config files are stored

 ******************************************************************************/
int
read_sensor_offsets(char *fname) {

  int j,i,rc;
  char   string[100];
  FILE  *in;

  /* get the max, min, and offsets of the position sensors */

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all joint variables and read them into the appropriate array */

  for (i=1; i<= n_dofs; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      printf("ERROR: Cannot find offset for >%s<!\n",joint_names[i]);
      fclose(in);
      return FALSE;
    }
    rc=fscanf(in,"%lf %lf %lf %lf %lf %lf",
	&joint_range[i][MIN_THETA], &joint_range[i][MAX_THETA],
	   &(joint_default_state[i].th),
	   &(joint_opt_state[i].th),
	   &(joint_opt_state[i].w),
	   &joint_range[i][THETA_OFFSET]);
    joint_default_state[i].thd = 0;
    joint_default_state[i].uff = 0;
  }
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_gains
\date  May 2000
\remarks 

parses the gain configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : the name of the link parameter file
 \param[out]    gth   : p-gains
 \param[out]    gthd  : d-gains
 \param[out]    gint  : i-gains

 ******************************************************************************/
int
read_gains(char *fname, double *gth, double *gthd, double *gint) {

  int j,i,rc;
  char   string[100];
  FILE  *in;
  double dum;

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all joint variables and read them into the appropriate array */

  for (i=1; i<= n_dofs; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      printf("ERROR: Cannot find gains for >%s<!\n",joint_names[i]);
      fclose(in);
      return FALSE;
    } else {
      if (gth != NULL && gthd !=NULL && gint != NULL)
	rc=fscanf(in,"%lf %lf %lf %lf",
	       &gth[i],&gthd[i],&gint[i],&u_max[i]);
      else
	rc=fscanf(in,"%lf %lf %lf %lf",
	       &dum,&dum,&dum,&u_max[i]);
    }
  }
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_link_parameters
\date  May 2000
\remarks 

parses the link parameters configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : the name of the link parameter file

 ******************************************************************************/
int
read_link_parameters(char *fname) {

  int j,i,n,rc;
  char   string[100];
  FILE  *in;
  double dum;
  extern const int floating_base_flag;

  /* read the link parameters */
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all joint variables and read them into the appropriate array;
     note that joint zero is the base, if present */
  
  for (i=0; i<= n_dofs; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      if (strcmp(joint_names[i],"dummy") == 0 ) {
	;  
      } else if (strcmp(joint_names[i],"BASE") == 0 || floating_base_flag ) {
	;
      } else {
	printf("ERROR: Cannot find link parameters for >%s<!\n",
	       joint_names[i]);
	fclose(in);
	return FALSE;
      }
    } else {
      rc=fscanf(in,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		&(links[i].m),
		&(links[i].mcm[_X_]),
		&(links[i].mcm[_Y_]),
		&(links[i].mcm[_Z_]),
		&(links[i].inertia[_X_][_X_]),
		&(links[i].inertia[_X_][_Y_]),
		&(links[i].inertia[_X_][_Z_]),
		&(links[i].inertia[_Y_][_Y_]),
		&(links[i].inertia[_Y_][_Z_]),
		&(links[i].inertia[_Z_][_Z_]),
		&(links[i].vis),
		&(links[i].coul),
		&(links[i].stiff),
		&(links[i].cons));
      if (rc != N_RBD_PARMS) {
	printf("Link Parameter for link >%s< have %d parameters, but should have %d -- maybe coul, stiff, and cons are missing? Missing parameters are zeroed!\n",joint_names[i],rc,N_RBD_PARMS);
      }
      for (j= _X_; j<= _Z_; ++j)
	for (n=j; n<= _Z_; ++n)
	  links[i].inertia[n][j]=links[i].inertia[j][n];
    }
  }
 
  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_whichDOFs
\date  May 2000
\remarks 

parses the whichDOFs  configuration file into global variables given a
keyworkd

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname   : configration file name
 \param[in]     keyword : which parameter set to use

 ******************************************************************************/
int
read_whichDOFs(char *fname, char *keyword) {

  int j,i,n,rc;
  char   string[100];
  FILE  *in;
  double dum;
  int    count = 0;

  /* read the link parameters */
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find the appropriate data set from the keyword */
  if (!find_keyword(in, keyword)) {
      printf("ERROR: Cannot find keyword >%s< in >%s<!\n",
	     keyword,fname);
      return FALSE;
  }
    
  /* read all following strings and determine which DOF they are concerned with */
  for (i=1; i<=n_dofs; ++i)
    whichDOFs[i] = FALSE;

  for (i=1; i<= n_dofs; ++i) {
    rc=fscanf(in,"%s",string);
    for (j=1; j<=n_dofs; ++j) {
      if (strcmp(string,joint_names[j]) == 0) {
	whichDOFs[j] = TRUE;
	++count;
	break;
      }
    }
    /* if we did not find a joint name -> break */
    if (j == n_dofs+1) 
      break;
  }

  printf("Found %d DOFs for whichDOFs array\n",count);

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_servoParameters
\date  May 2000
\remarks 

parses the servoParameters.cf file for the given keyword


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname     : configration file name
 \param[in]     keyword   : name of the servo for which parsing is done
 \param[out]    priority  : priority of the servo [0-100]
 \param[out]    stacksize : stacksize for this servo [byte]
 \param[out]    cpuID     : ID of CPU where to put the job
 \param[out]    dns       : number of delay nano seconds to insert into process
                            to avoid lock-up of linux

 ******************************************************************************/
int
read_servoParameters(char *fname, char *keyword, int *priority, int *stacksize,
		     int *cpuID, int *dns) 

{
  int j,i,n,rc;
  char   string[100];
  FILE  *in;
  double dum;
  int    count = 0;

  /* read the parameters */
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find the appropriate data set from the keyword */
  if (!find_keyword(in, keyword)) {
      printf("ERROR: Cannot find keyword >%s< in >%s<!\n",
	     keyword,fname);
      return FALSE;
  }
    
  // read the two pieces of information 
  rc=fscanf(in,"%d %d %d %d",priority,stacksize,cpuID,dns);
  if (rc != 4) {
    printf("Error when reading from >%s< in >%s<!\n",keyword,fname);
    return FALSE;
  }

  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  setDefaultPosture
\date  
   
\remarks 

       sets the desired states of the robot to the default

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
void
setDefaultPosture(void)
{

  int i;

  for (i=1; i<=n_dofs; ++i) 
    joint_des_state[i] = joint_default_state[i];

}

/*!*****************************************************************************
 *******************************************************************************
\note  conversion functions between float and double structures
\date  
   
\remarks 

       

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    the double structure pointer
    the float  structure pointer
 \param[in]     n    : the number of elements
 \param[in]     flag : DOUBLE2FLOAT or FLOAT2DOUBLE

 ******************************************************************************/
void
cSL_Jstate(SL_Jstate *sd, SL_fJstate *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      sf[i].th   = sd[i].th;
      sf[i].thd  = sd[i].thd;
      sf[i].thdd = sd[i].thdd;
      sf[i].u    = sd[i].u;
      sf[i].ufb  = sd[i].ufb;
      sf[i].load = sd[i].load;
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      sd[i].th   = sf[i].th;
      sd[i].thd  = sf[i].thd;
      sd[i].thdd = sf[i].thdd;
      sd[i].u    = sf[i].u;
      sd[i].ufb  = sf[i].ufb;
      sd[i].load = sf[i].load;
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_SDJstate(SL_DJstate *sd, SL_fSDJstate *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      sf[i].th   = sd[i].th;
      sf[i].thd  = sd[i].thd;
      sf[i].uff  = sd[i].uff;
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      sd[i].th   = sf[i].th;
      sd[i].thd  = sf[i].thd;
      sd[i].uff  = sf[i].uff;
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_DJstate(SL_DJstate *sd, SL_fDJstate *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      sf[i].th   = sd[i].th;
      sf[i].thd  = sd[i].thd;
      sf[i].thdd = sd[i].thdd;
      sf[i].uff  = sd[i].uff;
      sf[i].uex  = sd[i].uex;
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      sd[i].th   = sf[i].th;
      sd[i].thd  = sf[i].thd;
      sd[i].thdd = sf[i].thdd;
      sd[i].uff  = sf[i].uff;
      sd[i].uex  = sf[i].uex;
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_Cstate(SL_Cstate *sd, SL_fCstate *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_CART; ++j) {
	sf[i].x[j]   = sd[i].x[j];
	sf[i].xd[j]  = sd[i].xd[j];
	sf[i].xdd[j] = sd[i].xdd[j];
      }
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_CART; ++j) {
	sd[i].x[j]   = sf[i].x[j];
	sd[i].xd[j]  = sf[i].xd[j];
	sd[i].xdd[j] = sf[i].xdd[j];
      }
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_Corient(SL_Corient *sd, SL_fCorient *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_CART; ++j) {
	sf[i].a[j]   = sd[i].a[j];
	sf[i].ad[j]  = sd[i].ad[j];
	sf[i].add[j] = sd[i].add[j];
      }
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_CART; ++j) {
	sd[i].a[j]   = sf[i].a[j];
	sd[i].ad[j]  = sf[i].ad[j];
	sd[i].add[j] = sf[i].add[j];
      }
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_quat(SL_quat *sd, SL_fquat *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_QUAT; ++j) {
	sf[i].q[j]   = sd[i].q[j];
	sf[i].qd[j]  = sd[i].qd[j];
	sf[i].qdd[j] = sd[i].qdd[j];
      }
      for (j=1; j<=N_CART; ++j) {
	sf[i].ad[j]  = sd[i].ad[j];
	sf[i].add[j] = sd[i].add[j];
      }
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      for (j=1; j<=N_QUAT; ++j) {
	sd[i].q[j]   = sf[i].q[j];
	sd[i].qd[j]  = sf[i].qd[j];
	sd[i].qdd[j] = sf[i].qdd[j];
      }
      for (j=1; j<=N_CART; ++j) {
	sd[i].ad[j]  = sf[i].ad[j];
	sd[i].add[j] = sf[i].add[j];
      }
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_VisionBlob(SL_VisionBlob *sd, SL_fVisionBlob *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      sf[i].status = sd[i].status;
      cSL_Cstate(&(sd[i].blob)-1,&(sf[i].blob)-1,1,flag);
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      sd[i].status = sf[i].status;
      cSL_Cstate(&(sd[i].blob)-1,&(sf[i].blob)-1,1,flag);
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cSL_VisionBlobaux(Blob2D sd[][2+1], SL_fVisionBlobaux *sf, int n, int flag)
{ 
  int i,j,k;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i)
      {
	for( j=1; j<=2; j++ )
	  {
	    sf[i].status[j] = sd[i][j].status;
	    for( k=1; k<=2; k++ )
	      sf[i].blob[j][k] = sd[i][j].x[k];
	  }
      }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i)
      {
	for( j=1; j<=2; j++ )
	  {
	    sd[i][j].status = sf[i].status[j];
	    for( k=1; k<=2; k++ )
	      sd[i][j].x[k] = sf[i].blob[j][k];
	  }
      }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cBlob3D(Blob3D *sd, fBlob3D *sf, int n, int flag)
{ 
  int i,j;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      sf[i].status = sd[i].status;
      for (j=1; j<=N_CART; ++j) {
	sf[i].x[j]   = sd[i].x[j];
      }
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      sd[i].status = sf[i].status;
      for (j=1; j<=N_CART; ++j) {
	sd[i].x[j]   = sf[i].x[j];
      }
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

void
cBlob2D(Blob2D sd[][2+1], fBlob2D *sf, int n, int flag)
{ 
  int i,j,r;
  int count = 0;

  switch (flag) {
  case DOUBLE2FLOAT:
    for (i=1; i<=n; ++i) {
      for (r=1; r<=2; ++r) {
	++count;
	sf[count].status = sd[i][r].status;
	for (j=1; j<=2; ++j) {
	  sf[count].x[j]   = sd[i][r].x[j];
	}
      }
    }
    break;
  case FLOAT2DOUBLE:
    for (i=1; i<=n; ++i) {
      for (r=1; r<=2; ++r) {
	++count;
	sd[i][r].status = sf[count].status;
	for (j=1; j<=2; ++j) {
	  sd[i][r].x[j]   = sf[count].x[j];
	}
      }
    }
    break;
  default:
    printf("Error in float/double conversion\n");
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  rbwhere
\date  Feb 1999
\remarks 

 print the raw blob information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
rbwhere(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current Raw Blob States:\n");

  for (i=1; i<=max_blobs; i++) {

    if (raw_blobs2D[i][1].status || raw_blobs2D[i][2].status) {
      
      printf("%10s: Status = %d %d\n",blob_names[i],
	     raw_blobs2D[i][1].status, raw_blobs2D[i][2].status);
      printf("             x=% 5.3f    y=% 5.3f     x=% 5.3f    y=% 5.3f\n",
	     raw_blobs2D[i][1].x[_X_],
	     raw_blobs2D[i][1].x[_Y_],
	     raw_blobs2D[i][2].x[_X_],
	     raw_blobs2D[i][2].x[_Y_]);
      printf("\n");

    }

  }
  printf("\n");

}


/*!*****************************************************************************
 *******************************************************************************
\note  rbwhere2D
\date  Feb 1999
\remarks 

 print the raw blob information for 2D blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
rbwhere2D(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current Raw Blob 2D States:\n");

  for (i=1; i<=max_blobs; ++i) {

    for (j=1; j<=2; ++j) {

      if (raw_blobs2D[i][j].status) {
	
	printf("%10s.%d: Status = %d\n",
	       blob_names[i],j,raw_blobs2D[i][j].status);
	printf("             x=% 5.3f    y=% 5.3f\n",
	       raw_blobs2D[i][j].x[_X_],
	       raw_blobs2D[i][j].x[_Y_]);
	printf("\n");
      }
    }

  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  where_base
\date  Feb 1999
\remarks 

 print the cartesian information if the base coordinate system

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
where_base(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current Base State:\n");


  printf("            x=% 5.3f    y=% 5.3f    z=% 5.3f\n",
	 base_state.x[_X_],base_state.x[_Y_],base_state.x[_Z_]);
  printf("            xd=% 5.3f   yd=% 5.3f   zd=% 5.3f\n",
	 base_state.xd[_X_],base_state.xd[_Y_],base_state.xd[_Z_]);
  printf("            xdd=% 5.3f  ydd=% 5.3f  zdd=% 5.3f\n",
	 base_state.xdd[_X_],base_state.xdd[_Y_],base_state.xdd[_Z_]);
  printf("\n");

  printf("            ad=% 5.3f   bd=% 5.3f   gd=% 5.3f\n",
	 base_orient.ad[_X_],base_orient.ad[_Y_],base_orient.ad[_Z_]);
  printf("            add=% 5.3f  bdd=% 5.3f  gdd=% 5.3f\n",
	 base_orient.add[_X_],base_orient.add[_Y_],base_orient.add[_Z_]);
  printf("\n");

  printf("            q0=% 5.3f    q1=% 5.3f    q2=% 5.3f    q3=% 5.3f\n",
	 base_orient.q[1],base_orient.q[2],base_orient.q[3],base_orient.q[4]);
  printf("            q0d=% 5.3f   q1d=% 5.3f   q2d=% 5.3f   q3d=% 5.3f\n",
	 base_orient.qd[1],base_orient.qd[2],base_orient.qd[3],base_orient.qd[4]);
  printf("            q0dd=% 5.3f  q1dd=% 5.3f  q2dd=% 5.3f  q3dd=% 5.3f\n",
	 base_orient.qdd[1],base_orient.qdd[2],base_orient.qdd[3],base_orient.qdd[4]);
  printf("\n");


}

/*!*****************************************************************************
 *******************************************************************************
\note  where_cog
\date  Feb 1999
\remarks 

 print the cartesian information if the cog

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
where_cog(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  printf("Current COG State:\n");


  printf("              x=% 5.3f (% 5.3f)   y=% 5.3f (% 5.3f)   z=% 5.3f (% 5.3f)\n",
	 cog.x[_X_],cog_des.x[_X_],cog.x[_Y_],cog_des.x[_Y_],cog.x[_Z_],cog_des.x[_Z_]);
  printf("             xd=% 5.3f (% 5.3f)  yd=% 5.3f (% 5.3f)  zd=% 5.3f (% 5.3f)\n",
	 cog.xd[_X_],cog_des.xd[_X_],cog.xd[_Y_],cog_des.xd[_Y_],cog.xd[_Z_],cog_des.xd[_Z_]);
  printf("            xdd=% 5.3f (% 5.3f) ydd=% 5.3f (% 5.3f) zdd=% 5.3f (% 5.3f)\n",
	 cog.xdd[_X_],cog_des.xdd[_X_],cog.xdd[_Y_],cog_des.xdd[_Y_],cog.xdd[_Z_],cog_des.xdd[_Z_]);
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  linkQuat
\date  July 2005
\remarks 

 computes the quaternian from its rotation matrix
 WARNING: This algorithm is known to have numerical instabilities!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     R   : rotation matrix
 \param[in,out] q   : quaternian structure for output -- the output is chosen
                      such that the new quaternion does not flip sign relative
                      to the previous quaternion

 ******************************************************************************/
/**void
linkQuat(Matrix R, SL_quat *q)
{
  int i,j;
  double T,S,qx,qy,qz,qw;
  double quat_sign;
  double aux;

  T = 1.0 + R[1][1] + R[2][2] + R[3][3];

  if ( T > 0.00000001 ) {

    S  = 0.5 / sqrt(T);
    qw = 0.25 / S;
    qx = ( R[3][2] - R[2][3] ) * S;
    qy = ( R[1][3] - R[3][1] ) * S;
    qz = ( R[2][1] - R[1][2] ) * S;

  } else {

    if ((R[1][1] > R[2][2]) && (R[1][1] > R[3][3])) {
      S = sqrt( 1.0 + R[1][1] - R[2][2] - R[3][3] ) * 2;
      qx = 0.25 * S;
      qy = (R[1][2] + R[2][1] ) / S;
      qz = (R[1][3] + R[3][1] ) / S;
      qw = (R[2][3] - R[3][2] ) / S;
    } else if (R[2][2] > R[3][3]) {
      S = sqrt( 1.0 + R[2][2] - R[1][1] - R[3][3] ) * 2;
      qx = (R[1][2] + R[2][1] ) / S;
      qy = 0.25 * S;
      qz = (R[2][3] + R[3][2] ) / S;
      qw = (R[1][3] - R[3][1] ) / S;
    } else {
      S = sqrt( 1.0 + R[3][3] - R[1][1] - R[2][2] ) * 2;
      qx = (R[1][3] + R[3][1] ) / S;
      qy = (R[2][3] + R[3][2] ) / S;
      qz = 0.25 * S;
      qw = (R[1][2] - R[2][1] ) / S;
    }

  }

  // check whether we have a valid reference quaternion
  aux = 0.0;
  for (i=1;i<=N_QUAT;i++)
    aux += sqr(q->q[i]);
  aux = sqrt(aux);

  // fix the sign of quaternion
  if ( fabs(1.-aux) < 0.01) {
    quat_sign =
      q->q[_Q0_] * qw +
      q->q[_Q1_] * qx +
      q->q[_Q2_] * qy +
      q->q[_Q3_] * qz;

    if (quat_sign < 0.0) {
      qw *= -1.0;
      qx *= -1.0;
      qy *= -1.0;
      qz *= -1.0;
    }
  }

  q->q[_Q0_] = qw;
  q->q[_Q1_] = qx;
  q->q[_Q2_] = qy;
  q->q[_Q3_] = qz;

}
**/

/*!*****************************************************************************
 *******************************************************************************
\note  linkQuat
\date  March 2013
\remarks

 computes the quaternian from its rotation matrix
 from: "Quaternion Calculus and Fast Animation",
       Ken Shoemake, 1987 SIGGRAPH course notes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     R   : rotation matrix
 \param[in,out] q   : quaternian structure for output -- the output is chosen
                      such that the new quaternion does not flip sign relative
                      to the previous quaternion

 ******************************************************************************/
void
linkQuat(Matrix R, SL_quat *q)
{
  int r;
  double T,q_aux[5];
  double quat_sign;
  double aux;

  T = R[1][1] + R[2][2] + R[3][3];

  if ( T > 0.0 ) {
    T  = sqrt(T+1.0);
    q_aux[1] = 0.5*T;
    T = 0.5/T;
    q_aux[2] = ( R[3][2] - R[2][3] ) * T;
    q_aux[3] = ( R[1][3] - R[3][1] ) * T;
    q_aux[4] = ( R[2][1] - R[1][2] ) * T;

  } else {
    int i = 0;
    if (R[2][2] > R[1][1])
      i = 1;
    if (R[3][3] > R[i+1][i+1])
      i = 2;
    int j = (i+1)%3;
    int k = (j+1)%3;

    T = sqrt(R[i+1][i+1]-R[j+1][j+1]-R[k+1][k+1] + 1.0);
    q_aux[i+2] = 0.5 * T;
    T = 0.5/T;
    q_aux[1] = (R[k+1][j+1]-R[j+1][k+1])*T;
    q_aux[j+2] = (R[j+1][i+1]+R[i+1][j+1])*T;
    q_aux[k+2] = (R[k+1][i+1]+R[i+1][k+1])*T;
  }

  // check whether we have a valid reference quaternion
  aux = 0.0;
  for (r=1;r<=N_QUAT;r++)
    aux += sqr(q->q[r]);
  aux = sqrt(aux);

  // fix the sign of quaternion
  if ( fabs(1.-aux) < 0.01) {
    quat_sign =
      q->q[_Q0_] * q_aux[1] +
      q->q[_Q1_] * q_aux[2] +
      q->q[_Q2_] * q_aux[3] +
      q->q[_Q3_] * q_aux[4];

    if (quat_sign < 0.0) {
      q_aux[1] *= -1.0;
      q_aux[2] *= -1.0;
      q_aux[3] *= -1.0;
      q_aux[4] *= -1.0;
    }
  }

  q->q[_Q0_] = q_aux[1];
  q->q[_Q1_] = q_aux[2];
  q->q[_Q2_] = q_aux[3];
  q->q[_Q3_] = q_aux[4];

}

/*!*****************************************************************************
 *******************************************************************************
\note  quatDerivatives
\date  July 2005
\remarks 

 computes the quaternian derivatives from the angular velocity and the
 quaternion

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion and angular vel. and acc.
 \param[out]    q   : the quaternion velocity and acceleration will be filled in

 ******************************************************************************/
void
quatDerivatives(SL_quat *q)
{
  int i,j;
  double Q[4+1][3+1];
  double Qd[4+1][3+1];

  Q[1][1] = -q->q[_Q1_];
  Q[1][2] = -q->q[_Q2_];
  Q[1][3] = -q->q[_Q3_];

  Q[2][1] =  q->q[_Q0_];
  Q[2][2] =  q->q[_Q3_];
  Q[2][3] = -q->q[_Q2_];

  Q[3][1] = -q->q[_Q3_];
  Q[3][2] =  q->q[_Q0_];
  Q[3][3] =  q->q[_Q1_];

  Q[4][1] =  q->q[_Q2_];
  Q[4][2] = -q->q[_Q1_];
  Q[4][3] =  q->q[_Q0_];


  for (i=1; i<=N_QUAT; ++i) {
    q->qd[i] = 0.0;
    for (j=1; j<=N_CART; ++j)
      q->qd[i] += 0.5*Q[i][j]*q->ad[j];
  }
  
  
  Qd[1][1] = -q->qd[_Q1_];
  Qd[1][2] = -q->qd[_Q2_];
  Qd[1][3] = -q->qd[_Q3_];

  Qd[2][1] =  q->qd[_Q0_];
  Qd[2][2] =  q->qd[_Q3_];
  Qd[2][3] = -q->qd[_Q2_];

  Qd[3][1] = -q->qd[_Q3_];
  Qd[3][2] =  q->qd[_Q0_];
  Qd[3][3] =  q->qd[_Q1_];

  Qd[4][1] =  q->qd[_Q2_];
  Qd[4][2] = -q->qd[_Q1_];
  Qd[4][3] =  q->qd[_Q0_];

  for (i=1; i<=N_QUAT; ++i) {
    q->qdd[i] = 0.0;
    for (j=1; j<=N_CART; ++j)
      q->qdd[i] += 0.5*Qd[i][j]*q->ad[j] + 0.5*Q[i][j]*q->add[j];;
  }

  
}

/*!*****************************************************************************
 *******************************************************************************
\note  quatToAngularVelocity
\date  July 2005
\remarks 

 computes the angular velocity from quaternian position and derivatives 

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion and its velocity
 \param[out]    q   : fills in the angular velocity

 ******************************************************************************/
void
quatToAngularVelocity(SL_quat *q)
{
  int i,j;
  double Q[4+1][3+1];
  double Qd[4+1][3+1];

  q->ad[_A_] = 2.*(  -q->q[_Q1_]*q->qd[_Q0_]
		     +q->q[_Q0_]*q->qd[_Q1_]
		     -q->q[_Q3_]*q->qd[_Q2_]
		     +q->q[_Q2_]*q->qd[_Q3_] );


  q->ad[_B_] = 2.*(  -q->q[_Q2_]*q->qd[_Q0_]
		     +q->q[_Q3_]*q->qd[_Q1_]
		     +q->q[_Q0_]*q->qd[_Q2_]
		     -q->q[_Q1_]*q->qd[_Q3_] );


  q->ad[_G_] = 2.*(  -q->q[_Q3_]*q->qd[_Q0_]
		     -q->q[_Q2_]*q->qd[_Q1_]
		     +q->q[_Q1_]*q->qd[_Q2_]
		     +q->q[_Q0_]*q->qd[_Q3_] );

  
}

/*!*****************************************************************************
 *******************************************************************************
\note  quatToRotMat
\date  July 2005
\remarks 

 converts a quaternion into a rotation matrix, where the rotation matrix
 is the transformation from global to local coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion and its velocity
 \param[out]    R   : 3 by 3 rotation matrix

 ******************************************************************************/
void
quatToRotMat(SL_quat *q, Matrix R)
{

  R[1][1] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q1_]);
  R[2][2] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q2_]);
  R[3][3] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q3_]);

  R[1][2] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] + q->q[_Q0_]*q->q[_Q3_]);
  R[1][3] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q2_]);
  R[2][1] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] - q->q[_Q0_]*q->q[_Q3_]);
  R[2][3] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q1_]);
  R[3][1] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q2_]);
  R[3][2] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q1_]);
  
}
/*!*****************************************************************************
 *******************************************************************************
\note  quatToRotMatInv
\date  July 2005
\remarks 

 converts a quaternion into a rotation matrix, where the rotation matrix
 is the transformation from local to global coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion and its velocity
 \param[out]    R   : 3 by 3 rotation matrix

 ******************************************************************************/
void
quatToRotMatInv(SL_quat *q, Matrix R)
{

  R[1][1] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q1_]);
  R[2][2] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q2_]);
  R[3][3] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q3_]);

  R[2][1] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] + q->q[_Q0_]*q->q[_Q3_]);
  R[3][1] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q2_]);
  R[1][2] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] - q->q[_Q0_]*q->q[_Q3_]);
  R[3][2] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q1_]);
  R[1][3] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q2_]);
  R[2][3] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q1_]);
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  quatMatrix
\date  July 2005
\remarks 

 converts a quaternion into a matrix Q(q) which can be used in Quaternion
 multiplication, e.g., q12 = q2 * q1 = Q(q2)*q1. A good source for the math
 is Flashner's book.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion
 \param[out]    Q   : a 4x4 matrix


 ******************************************************************************/
void
quatMatrix(SL_quat *q, Matrix Q)
{
  int i,j;
  
  Q[1][1] =  q->q[_Q0_];
  Q[1][2] = -q->q[_Q1_];
  Q[1][3] = -q->q[_Q2_];
  Q[1][4] = -q->q[_Q3_];

  Q[2][1] =  q->q[_Q1_];
  Q[2][2] =  q->q[_Q0_];
  Q[2][3] =  q->q[_Q3_];
  Q[2][4] = -q->q[_Q2_];

  Q[3][1] =  q->q[_Q2_];
  Q[3][2] = -q->q[_Q3_];
  Q[3][3] =  q->q[_Q0_];
  Q[3][4] =  q->q[_Q1_];

  Q[4][1] =  q->q[_Q3_];
  Q[4][2] =  q->q[_Q2_];
  Q[4][3] = -q->q[_Q1_];
  Q[4][4] =  q->q[_Q0_];
}

#ifndef VX  
#endif

/*!*****************************************************************************
 *******************************************************************************
\note  revoluteGJacColumn
\date  January 2006
\remarks 

 computes one column for the geometric jacobian of a revolute joint
 from the given input vectors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     p    : position of endeffector
 \param[in]     pi   : position of joint origin
 \param[in]     zi   : unit vector of joint axis
 \param[out]    c    : column vector of Jacobian

 ******************************************************************************/
void
revoluteGJacColumn(Vector p, Vector pi, Vector zi, Vector c)
{
  int i,j;

  c[1] = zi[2] * (p[3]-pi[3]) - zi[3] * (p[2]-pi[2]);
  c[2] = zi[3] * (p[1]-pi[1]) - zi[1] * (p[3]-pi[3]);
  c[3] = zi[1] * (p[2]-pi[2]) - zi[2] * (p[1]-pi[1]);
  c[4] = zi[1];
  c[5] = zi[2];
  c[6] = zi[3];

}

/*!*****************************************************************************
 *******************************************************************************
\note  prismaticGJacColumn
\date  January 2011
\remarks 

 computes one column for the geometric jacobian of a prismatic joint
 from the given input vectors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     p    : position of endeffector
 \param[in]     pi   : position of joint origin
 \param[in]     zi   : unit vector of joint axis
 \param[out]    c    : column vector of Jacobian

 ******************************************************************************/
void
prismaticGJacColumn(Vector p, Vector pi, Vector zi, Vector c)
{
  int i,j;

  c[1] = zi[1];
  c[2] = zi[2];
  c[3] = zi[3];
  c[4] = 0.0;
  c[5] = 0.0;
  c[6] = 0.0;

}

/*!*****************************************************************************
 *******************************************************************************
\note  compute_cog
\date  March 2005
   
\remarks 

      computes the center of gravity, assuming, of course, well estimated
      parameters of the robot

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     All results can be computed based on global variables. Results are
     assigned to the cog structure.


 ******************************************************************************/
#define BASE 0
void
compute_cog(void)
{
  int    i,j;
  double mass;

  for (j=1; j<=N_CART; ++j) {
    cog.x[j]     = joint_cog_mpos[BASE][j];
    cog_des.x[j] = joint_cog_mpos_des[BASE][j];
  }

  mass = links[BASE].m;

  for (i=1; i<=n_dofs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      cog.x[j]     += joint_cog_mpos[i][j];
      cog_des.x[j] += joint_cog_mpos_des[i][j];
    }
    mass += links[i].m;
  }

  if (mass != 0 ){
    for (j=1; j<=N_CART; ++j) {
      cog.x[j] /= mass;
      cog_des.x[j] /= mass;
    }
  } else {
    for (j=1; j<=N_CART; ++j) {
      cog.x[j] = 0;
      cog_des.x[j] = 0;
    }
  }


}

/*!*****************************************************************************
 *******************************************************************************
\note  eulerToQuatInv
\date  January 2006
\remarks 

 converts an a-b-g Euler angle notation to a quaternion

 Note: this computation is based on a rotation matrix which is transforming
       from local to global coordinates (which I usually call an inverse
       transformation).

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    q    : quaternion

 ******************************************************************************/
void
eulerToQuatInv(Vector a, SL_quat *q)
{
  int i,j;
  static Matrix R;
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    R = my_matrix(1,N_CART,1,N_CART);
  }

  eulerToRotMat(a,R); // SL quaternions denote the inverse transformation
                      // i.e., local->global
  linkQuat(R,q);

}

/*!*****************************************************************************
 *******************************************************************************
\note  eulerToQuat
\date  January 2006
\remarks 

 converts an a-b-g Euler angle notation to a quaternion

 Note: this computation is based on a rotation matrix which is transforming
       from global to local coordinates (which I usually call a forward
       transformation).

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    q    : quaternion

 ******************************************************************************/
void
eulerToQuat(Vector a, SL_quat *q)
{
  int i,j;
  static Matrix R;
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    R = my_matrix(1,N_CART,1,N_CART);
  }

  eulerToRotMatInv(a,R); // SL quaternions denote the inverse transformation
                         // i.e., local->global
  linkQuat(R,q);

}

/*!*****************************************************************************
 *******************************************************************************
\note  eulerToRotMat
\date  January 2006
\remarks 

 converts an a-b-g Euler angle notation to a rotation matrix. This is
 the forward transformation (from global to local coordinates), computed
 as 

  Rz*Ry*Rx  where

  Rx = {{1, 0, 0}, {0, Cos[a], Sin[a]}, {0, -Sin[a], Cos[a]}}
  Ry = {{Cos[b], 0, -Sin[b]}, {0, 1, 0}, {Sin[b], 0, Cos[b]}}
  Rz = {{Cos[g], Sin[g], 0}, {-Sin[g], Cos[g], 0}, {0, 0, 1}}


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    R    : rotation matrix

 ******************************************************************************/
void
eulerToRotMat(Vector a, Matrix R)
{

  R[1][1] =  cos(a[2])*cos(a[3]);
  R[1][2] =  cos(a[3])*sin(a[1])*sin(a[2]) + cos(a[1])*sin(a[3]);
  R[1][3] = -(cos(a[1])*cos(a[3])*sin(a[2])) +sin(a[1])*sin(a[3]);
  R[2][1] = -(cos(a[2])*sin(a[3]));
  R[2][2] =  cos(a[1])*cos(a[3]) - sin(a[1])*sin(a[2])*sin(a[3]);
  R[2][3] =  cos(a[3])*sin(a[1]) + cos(a[1])*sin(a[2])*sin(a[3]);
  R[3][1] =  sin(a[2]);
  R[3][2] = -(cos(a[2])*sin(a[1]));
  R[3][3] =  cos(a[1])*cos(a[2]);

}

/*!*****************************************************************************
 *******************************************************************************
\note  eulerToRotMatInv
\date  January 2006
\remarks 

 converts an a-b-g Euler angle notation to a rotation matrix. This is
 the inverse transformation (from local to globa coordinates), computed
 as 

  Rx'*Ry'*Rz'  where

  Rx = {{1, 0, 0}, {0, Cos[a], Sin[a]}, {0, -Sin[a], Cos[a]}}
  Ry = {{Cos[b], 0, -Sin[b]}, {0, 1, 0}, {Sin[b], 0, Cos[b]}}
  Rz = {{Cos[g], Sin[g], 0}, {-Sin[g], Cos[g], 0}, {0, 0, 1}}


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    R    : rotation matrix

 ******************************************************************************/
void
eulerToRotMatInv(Vector a, Matrix R)
{

  R[1][1] =  cos(a[2])*cos(a[3]);
  R[2][1] =  cos(a[3])*sin(a[1])*sin(a[2]) + cos(a[1])*sin(a[3]);
  R[3][1] = -(cos(a[1])*cos(a[3])*sin(a[2])) +sin(a[1])*sin(a[3]);
  R[1][2] = -(cos(a[2])*sin(a[3]));
  R[2][2] =  cos(a[1])*cos(a[3]) - sin(a[1])*sin(a[2])*sin(a[3]);
  R[3][2] =  cos(a[3])*sin(a[1]) + cos(a[1])*sin(a[2])*sin(a[3]);
  R[1][3] =  sin(a[2]);
  R[2][3] = -(cos(a[2])*sin(a[1]));
  R[3][3] =  cos(a[1])*cos(a[2]);

}

/*!*****************************************************************************
 *******************************************************************************
\note  quatToEuler
\date  Nov. 2007
\remarks 

 converts a quaternion to a-b-g Euler angle notation assuming that this
 quaternion denotes a forward (global->local) transformation.

 Note: we choose a solution for the b-rotation within [-pi/2,pi/2]

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    q    : quaternion

 ******************************************************************************/
void
quatToEuler(SL_quat *q, Vector a)

{
  static int firsttime = TRUE;
  static Matrix R;

  if (firsttime) {
    firsttime = FALSE;
    R = my_matrix(1,N_CART,1,N_CART);
  }

  quatToRotMat(q, R);
  rotMatToEuler(R,a);
}

/*!*****************************************************************************
 *******************************************************************************
\note  rotMatToEuler
\date  Nov. 2007
\remarks 

 converts a rotation matrix to a-b-g Euler angle notation assuming that this
 matrix denotes a forward (global->local) transformation.

 Note: we choose a solution for the b-rotation within [-pi/2,pi/2]

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     R    : rotation matrix
 \param[out]    q    : quaternion

 ******************************************************************************/
void
rotMatToEuler(Matrix R, Vector a)

{
  a[1] =   atan2_save(R[3][3],-R[3][2]);
  a[2] =   atan2_save(sqrt(sqr(R[3][2])+sqr(R[3][3])),R[3][1]);
  a[3] =   atan2_save(R[1][1],-R[2][1]);
}

/*!*****************************************************************************
 *******************************************************************************
\note  quatToEulerInv
\date  Nov. 2007
\remarks 

 converts a quaternion to a-b-g Euler angle notation assuming that this
 quaternion denotes an inverse (local->globa) transformation.

 Note: we choose a solution for the b-rotation within [-pi/2,pi/2]

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : vector of rotation angles (a,b,g)
 \param[out]    q    : quaternion

 ******************************************************************************/
void
quatToEulerInv(SL_quat *q, Vector a)

{
  static int firsttime = TRUE;
  static Matrix R;

  if (firsttime) {
    firsttime = FALSE;
    R = my_matrix(1,N_CART,1,N_CART);
  }

  quatToRotMatInv(q, R);
  rotMatToEulerInv(R,a);

}

/*!*****************************************************************************
 *******************************************************************************
\note  rotMatToEulerInv
\date  Nov. 2007
\remarks 

 converts a rotation Matrix to a-b-g Euler angle notation assuming that this
 quaternion denotes an inverse (local->globa) transformation.

 Note: we choose a solution for the b-rotation within [-pi/2,pi/2]

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     R    : rotation matrix
 \param[out]    q    : quaternion

 ******************************************************************************/
void
rotMatToEulerInv(Matrix R, Vector a)

{
  a[1] =   atan2_save(R[3][3],-R[2][3]);
  a[2] =   atan2_save(sqrt(sqr(R[2][3])+sqr(R[3][3])),R[1][3]);
  a[3] =   atan2_save(R[1][1],-R[1][2]);
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_config_files
\date  May 2000
\remarks 

  parses the ConfigFiles.cf file an assignes all config file names

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : file name of file where config files are stored

 ******************************************************************************/
int
read_config_files(char *fname) {


  int j,i,rc;
  char   string[100];
  FILE  *in;

  // open the file for read 
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // udpate all config file names as needed
  for (i=1; i<= N_CONFIG_FILES; ++i) {
    if (find_keyword(in, &(config_file_tags[i][0]))) {
      rc=fscanf(in,"%s",config_files[i]);
    } else {
      sprintf(config_files[i],"%s not found",&(config_file_tags[i][0]));
    }
  }
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_sensor_calibration
\date  May 2000
\remarks 

parses the sensor calibration configuration file into global variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of calibration file
 \param[in]     joint_lin_rot   : matrix about sensor configuration
 \param[in]     pos_polar       : sign for positions
 \param[in]     load_polar      : sign for loads

 ******************************************************************************/
int
read_sensor_calibration(char *fname, Matrix joint_lin_rot, 
			Vector pos_polar, Vector load_polar) 

{

  int j,i,rc;
  char   string[100];
  FILE  *in;
  double dum;

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all joint variables and read them into the appropriate array */

  for (i=1; i<= n_dofs; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      printf("ERROR: Cannot find calibration for >%s<!\n",joint_names[i]);
      fclose(in);
      return FALSE;
    } else {
      rc=fscanf(in,"%lf %lf %lf %lf %lf %lf %lf",
	     &joint_lin_rot[i][SENSOR],
	     &joint_lin_rot[i][ACTUATOR],
	     &joint_lin_rot[i][LOADCELL],
	     &joint_lin_rot[i][MOMENTARM],
	     &joint_lin_rot[i][MOUNTPOINT],
	     &pos_polar[i],&load_polar[i]);
      if (joint_lin_rot[i][MOUNTPOINT] != 0.0) 
	joint_lin_rot[i][THETA0] = acos(joint_lin_rot[i][MOMENTARM]/
					joint_lin_rot[i][MOUNTPOINT]);
      else
	joint_lin_rot[i][THETA0] = 0.0;
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  quatError
\date  Nov. 2007
\remarks 

   Computes the norm of the difference between two quaternions, derived
   from a formulation that is used in quaternion feedback control (and
   has a unique zero norm only if two quaternions are the same). Note 
   that is norm is bounded in [0,1]

   Assume each quaternion can be written as [eta eps], where eps is the
   vector component of the quaternion. Then the error is:

   error eta_1 * eps2 - eta_2*eps2 - eps1 x eps2 (x = cross product)


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q1     : quaternion 1
 \param[in]     q2     : quaternion 2


 ******************************************************************************/
double 
quatError(double* q1, double* q2) 

{
  double norm = 0;
  double aux;


  aux = q1[_Q0_] * q2[_Q1_]  - q2[_Q0_] * q1[_Q1_] + ( q1[_Q2_] *  q2[_Q3_] - q2[_Q2_] * q1[_Q3_]);
  norm += sqr(aux);

  aux = q1[_Q0_] * q2[_Q2_]  - q2[_Q0_] * q1[_Q2_] + ( q1[_Q3_] *  q2[_Q1_] - q2[_Q3_] * q1[_Q1_]);
  norm += sqr(aux);

  aux = q1[_Q0_] * q2[_Q3_]  - q2[_Q0_] * q1[_Q3_] + ( q1[_Q1_] *  q2[_Q2_] - q2[_Q1_] * q1[_Q2_]);
  norm += sqr(aux);

  return sqrt(norm);

}

/*!*****************************************************************************
 *******************************************************************************
\note  quatErrorVector
\date  April. 2008
\remarks 

   Computes the difference between two quaternions as an angular velocity.
   This is used in quaternion feedback control.

   Assume each quaternion can be written as [eta eps], where eps is the
   vector component of the quaternion. Then the error is:

   error = eta_1 * eps2 - eta_2*eps2 - eps1 x eps2 (x = cross product)


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q1     : quaternion 1 (the desired state in feedback control)
 \param[in]     q2     : quaternion 2 (the state in feedback control)


 ******************************************************************************/
void
quatErrorVector(double* q1, double* q2, double *ad) 

{

  ad[_A_] = q1[_Q0_] * q2[_Q1_]  - q2[_Q0_] * q1[_Q1_] + ( q1[_Q2_] *  q2[_Q3_] - q2[_Q2_] * q1[_Q3_]);
  ad[_B_] = q1[_Q0_] * q2[_Q2_]  - q2[_Q0_] * q1[_Q2_] + ( q1[_Q3_] *  q2[_Q1_] - q2[_Q3_] * q1[_Q1_]);
  ad[_G_] = q1[_Q0_] * q2[_Q3_]  - q2[_Q0_] * q1[_Q3_] + ( q1[_Q1_] *  q2[_Q2_] - q2[_Q1_] * q1[_Q2_]);


}


/*!*****************************************************************************
 *******************************************************************************
\note  init_parameter_pool
\date  July 2010
\remarks 

some initialization of the parameter pool

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
init_parameter_pool(void)
{

  // remove the file which stores all parameter pool variables
  remove(parameter_pool_vars);

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parameter_pool_double
\date  July 2010
\remarks 

reads a double value from the paramter pool configuration file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of parameter pool file
 \param[in]     keyword         : keyword that starts the line
 \param[out]    value           : the value to be read

 returns TRUE on success

 ******************************************************************************/
int
read_parameter_pool_double(char *fname, char *keyword, double *value)
{

  int    j,i,rc;
  char   string[100];
  FILE  *in;

  in = fopen(parameter_pool_vars,"a");
  if (in != NULL) {
    fseek(in,0,SEEK_END);
    fprintf(in,"%s (double, %s)\n",keyword,servo_name);
    fclose(in);
  }

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return FALSE;
  } else {
    rc=fscanf(in,"%lf",value);
    if (rc != 1)
    {
      fclose(in);
      return FALSE;
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parameter_pool_double_array
\date  July 2010
\remarks 

reads a double array from the paramter pool configuration file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of parameter pool file
 \param[in]     keyword         : keyword that starts the line
 \param[in]     n_values        : number of values to be read
 \param[out]    values          : the values to be read

 returns TRUE on success

 ******************************************************************************/
int
read_parameter_pool_double_array(char *fname, char *keyword, int n_values, double *values)
{

  int    j,i,rc;
  char   string[100];
  FILE  *in;

  in = fopen(parameter_pool_vars,"a");
  if (in != NULL) {
    fseek(in,0,SEEK_END);
    fprintf(in,"%s (%d-coeff double array, %s)\n",keyword,n_values,servo_name);
    fclose(in);
  }

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return FALSE;
  } else {
    for (i=1; i<=n_values; ++i) {
      rc=fscanf(in,"%lf",&(values[i]));
      if (rc != 1)
      {
        fclose(in);
	return FALSE;
      }
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parameter_pool_int
\date  July 2010
\remarks 

reads a int value from the paramter pool configuration file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of parameter pool file
 \param[in]     keyword         : keyword that starts the line
 \param[out]    value           : the value to be read

 returns TRUE on success

 ******************************************************************************/
int
read_parameter_pool_int(char *fname, char *keyword, int *ivalue)
{

  int    j,i,rc;
  char   string[100];
  FILE  *in;

  in = fopen(parameter_pool_vars,"a");
  if (in != NULL) {
    fseek(in,0,SEEK_END);
    fprintf(in,"%s (int, %s)\n",keyword,servo_name);
    fclose(in);
  }

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return FALSE;
  } else {
    rc=fscanf(in,"%d",ivalue);
    if (rc != 1)
    {
      fclose(in);
      return FALSE;
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parameter_pool_int_array
\date  July 2010
\remarks 

reads a int array from the paramter pool configuration file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of parameter pool file
 \param[in]     keyword         : keyword that starts the line
 \param[in]     n_values        : number of values to be read
 \param[out]    values          : the values to be read

 returns TRUE on success

 ******************************************************************************/
int
read_parameter_pool_int_array(char *fname, char *keyword, int n_values, int *ivalues)
{

  int    j,i,rc;
  char   string[100];
  FILE  *in;

  in = fopen(parameter_pool_vars,"a");
  if (in != NULL) {
    fseek(in,0,SEEK_END);
    fprintf(in,"%s (%d-coeff int array, %s)\n",keyword,n_values,servo_name);
    fclose(in);
  }

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return FALSE;
  } else {
    for (i=1; i<=n_values; ++i) {
      rc=fscanf(in,"%d",&(ivalues[i]));
      if (rc != 1)
      {
        fclose(in);
	return FALSE;
      }
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parameter_pool_string
\date  July 2010
\remarks 

reads a string from the paramter pool configuration file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname           : name of parameter pool file
 \param[in]     keyword         : keyword that starts the line
 \param[out]    svalue          : the string to be read (provide enough length)

 returns TRUE on success

 ******************************************************************************/
int
read_parameter_pool_string(char *fname, char *keyword, char *svalue)
{

  int    j,i,rc;
  char   string[100];
  FILE  *in;
  char   c;

  in = fopen(parameter_pool_vars,"a");
  if (in != NULL) {
    fseek(in,0,SEEK_END);
    fprintf(in,"%s (string, %s)\n",keyword,servo_name);
    fclose(in);
  }

  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return FALSE;
  } else {
    while ((c=fgetc(in)) == ' ' || c == '\t') // skip initial blank and tab characters
      ;
    ungetc(c,in);                // reset the file pointer by one

    i = 0;
    while ( TRUE ) {
      c = fgetc(in);
      if ( c == EOF || c == '\n') {
	svalue[i] = '\0';
	break;
      } else {
	svalue[i++] = c;
      }
    }
  }

  fclose(in);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  parseWindowSpecs
\date  July 2010
\remarks 

 From the given string, a window position, and size is determined. The syntax is
 copied from the xterm syntax, e.g.,

 80x12+100+100

 creates a 80 character wide by 12 lines high window at location x=100 y=100
 (using the screen coordinates, i.e., 0,0 is top left)

 For xterm windows, the first two argumets always have the "charcters" and
 "lines" units.

 For graphics windows, this will be interpreted as pixels.

 We allow an additional feature to specify pixel based values in terms of
 percentage of the display. E.g.,

 80x12+10%+20%

 will be translated into 10% of the width and 20% of the height of the display

 It is also possible to use negative values, to indicate starting from the
 max value of each dimension. E.g.,

  
 80x12-100-100

 will create the window 100 pixels of the lower right corner of the display
 
 Percentage values can be used with negative values, too.

 The percentage values can be real numbers for higher precision

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     string          : string to be parsed
 \param[in]     dw              : display width in pixels
 \param[in]     dh              : display height in pixels
 \param[out]    xstring         : string in xterm notation
 \param[out]    x               : x position of window
 \param[out]    y               : y position of windwo
 \param[out]    w               : width of window
 \param[out]    h               : height of window

 returns TRUE on success

 ******************************************************************************/
int
parseWindowSpecs(char *string, int dw, int dh, char *xstring, int *x, int *y, int *w, int *h)
{
  int    i,j[4],t,rc;
  int    sl = strlen(string);
  char   tstring[4][100];
  double rx,ry,rw,rh;

  // remove all blanks and insert one blank before 'x', '+', '%', and '-' signs
  j[0] = j[1] = j[2] = j[3]  = 0;
  t = 0;
  for (i=0; i<sl; ++i)
    if (string[i] != ' ') {
      if (string[i] == 'x' || string[i] == '+' || string[i] == '-') { // start new string
	tstring[t][j[t]] = '\0';
	++t;
      }
      if (string[i] != 'x') { // don't need the 'x' anymore
	if (string[i] == '%') // insert a blank
	  tstring[t][j[t]++] = ' ';
	tstring[t][j[t]++] = string[i];
      }
    }

  // now we can read the four values
  if ((rc = sscanf(tstring[0],"%lf",&rw)) != 1) {
    printf("Error when parsing window spec string >%s<\n",string);
    return FALSE;
  }
  if ((rc = sscanf(tstring[1],"%lf",&rh)) != 1) {
    printf("Error when parsing window spec string >%s<\n",string);
    return FALSE;
  }
  if ((rc = sscanf(tstring[2],"%lf",&rx)) != 1) {
    printf("Error when parsing window spec string >%s<\n",string);
    return FALSE;
  }
  if ((rc = sscanf(tstring[3],"%lf",&ry)) != 1) {
    printf("Error when parsing window spec string >%s<\n",string);
    return FALSE;
  }

  // check for percentage values and replace them
  if (tstring[0][j[0]-1] == '%')
    rw = rw/100.*dw;

  if (tstring[1][j[1]-1] == '%')
    rh = rh/100.*dh;

  if (tstring[2][j[2]-1] == '%')
    rx = rx/100.*dw;

  if (tstring[3][j[3]-1] == '%')
    ry = ry/100.*dh;

  // check for negative signs
  if (rx < 0)
    rx = dw + rx;

  if (ry < 0)
    rx = dh + ry;

  // finally assign the return values
  *x = rx;
  *y = ry;
  *w = rw;
  *h = rh;

  sprintf(xstring,"%dx%d%+d%+d",*w,*h,*x,*y);

  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  count_extra_contact_points
\date  July 2010
\remarks 

check in the appropriate contact configuration file how many extra contacts
the user specified.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : file name of file where config files are stored

 returns the number of extra contact points

 ******************************************************************************/
int
count_extra_contact_points(char *fname) {

  int    j,i,rc;
  char   string[100];
  FILE  *in;
  int    count = 0;
  char   name1[100],name2[100];
  char   fcond1[100],fcond2[100];
  int    n_checks;
  int    active;
  double dum[2*N_CART+1];

  // open file and strip comments 
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // read the file until EOF
  while (TRUE) {
    n_checks = 0;
    rc = fscanf(in,"%s %s %d %d %s %s",name1,name2,&active,&n_checks,fcond1,fcond2);
    if (rc == 6)
      count += n_checks;
    else {
      if (rc != EOF) 
	printf("Parsing error in count_extra_contact_points in SL_common.c (rc=%d)\n",rc);
      break;
    }
    if (strcmp("POINT_CONTACT",name2)==0)
      rc = fscanf(in,"%lf %lf %lf %lf %lf %lf",&(dum[1]),&(dum[2]),&(dum[3]),&(dum[4]),&(dum[5]),&(dum[6]));
  }

  fclose(in);

  //printf("Found %d extract contact points",count);

  return count;
}

/*!*****************************************************************************
 *******************************************************************************
\note  compute_local_interface_forces
\date  January 2011
\remarks 

This function computes the force/torque vector acting at a point
(normally chosen to be the point of the local coordinate system of a
DOF) due to the inertial properties of the link of this joint. In
Featherstone, this is called fnet force. This is the force/torque
vector described in Ann,Atkeson, and Hollerbach.  This force torque
sensor is what would be sensed by a 6-axis force torque sensor at the
joint -- i.e., this can be used to created a force/torque sensor simulation.
If the force/torque from multiple links were to be determined, the force/torques
need to be propagated and added over those links.

Note that this computation needs to be  done in local coordinates which are 
determined by the inertia matrix, which includes the parallel axis theorem
for the local coordinates. Thus, the force/torque vector will be in local
coordinates, too.


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     xdd : acceleration of refence point for force/torque
 \param[in]      ad : angular velocity of reference point
 \param[in]     add : angular acceleration of reference point
 \param[in]       g : gravity vector
 \param[in]      li : RBD parameters of link
 \param[out]      f : force vector
 \param[out]      t : torque vector

 returns the number of extra contact points

 ******************************************************************************/
 void
 compute_local_interface_forces(double *xdd, double *ad, double *add, double *g,
				SL_link li, double *f, double *t)
{
  int i,j;
  MY_MATRIX(K,1,2*N_CART,1,N_RBD_PARMS);
  MY_VECTOR(v,1,N_RBD_PARMS);
    
  
  K[1][1] = -g[1] + xdd[1];
  K[1][2] = -Power(ad[2],2) - Power(ad[3],2);
  K[1][3] = ad[1]*ad[2] - add[3];
  K[1][4] = ad[1]*ad[3] + add[2];
  
  K[2][1] = -g[2] + xdd[2];
  K[2][2] = ad[1]*ad[2] + add[3];
  K[2][3] = -Power(ad[1],2) - Power(ad[3],2);
  K[2][4] = ad[2]*ad[3] - add[1];
  
  K[3][1] = -g[3] + xdd[3];
  K[3][2] = ad[1]*ad[3] - add[2];
  K[3][3] = ad[2]*ad[3] + add[1];
  K[3][4] = -Power(ad[1],2) - Power(ad[2],2);
  
  K[4][1] = 0;
  K[4][2] = 0;
  K[4][3] = -g[3] + xdd[3];
  K[4][4] = g[2] - xdd[2];
  K[4][5] = add[1];
  K[4][6] = -(ad[1]*ad[3]) + add[2];
  K[4][7] = ad[1]*ad[2] + add[3];
  K[4][8] = -(ad[2]*ad[3]),
  K[4][9] = Power(ad[2],2) - Power(ad[3],2);
  K[4][10] = ad[2]*ad[3];
  
  K[5][1] = 0;
  K[5][2] = g[3] - xdd[3];
  K[5][3] = 0;
  K[5][4] = -g[1] + xdd[1];
  K[5][5] = ad[1]*ad[3];
  K[5][6] = ad[2]*ad[3] + add[1];
  K[5][7] = -Power(ad[1],2) + Power(ad[3],2);
  K[5][8] = add[2];
  K[5][9] = -(ad[1]*ad[2]) + add[3];
  K[5][10] = -(ad[1]*ad[3]);
  
  K[6][1] = 0;
  K[6][2] = -g[2] + xdd[2];
  K[6][3] = g[1] - xdd[1];
  K[6][4] = 0;
  K[6][5] = -(ad[1]*ad[2]);
  K[6][6] = Power(ad[1],2) - Power(ad[2],2);
  K[6][7] = -(ad[2]*ad[3]) + add[1];
  K[6][8] = ad[1]*ad[2];
  K[6][9] = ad[1]*ad[3] + add[2];
  K[6][10] = add[3];

  // Note that vis,coul,stiff,off are not relevant as these are parameters that act
  // only locally on a motor, i.e., can be thought to reduce the motor torque, but
  // don't propagate through the dynamics.

  // sort link parameters into vector
  v[1] = li.m;
  v[2] = li.mcm[1];
  v[3] = li.mcm[2];
  v[4] = li.mcm[3];
  v[5] = li.inertia[1][1];
  v[6] = li.inertia[1][2];
  v[7] = li.inertia[1][3];
  v[8] = li.inertia[2][1];
  v[9] = li.inertia[2][2];
  v[10] = li.inertia[3][3];

  
  // compute the result
  for (i=1; i<=N_CART; ++i) {
    f[i] = 0.0;
    for (j=1; j<=N_RBD_PARMS; ++j)
      f[i] += K[i][j]*v[j];
  }

  for (i=1; i<=N_CART; ++i) {
    t[i] = 0.0;
    for (j=1; j<=N_RBD_PARMS; ++j)
      t[i] += K[i+N_CART][j]*v[j];
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  print_J
\date  Feb 1999
\remarks 

 prints the current Jacobian of the endeffectors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
print_J(void)
{
  int i,j;

  if (!servo_enabled) {
    beep(1);
    printf("WARNING: servo is not running!!\n");
  }

  print_mat("Jacobian (actual)",J);
  print_mat("Jacobian (desired)",Jdes);

}


