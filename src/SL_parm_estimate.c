/*!=============================================================================
  ==============================================================================

  \ingroup SLparameterEstimation

  \file    SL_parm_estimate.c

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  estimates the inverse dynamics parameters
      
  July 2009: add base link system for estimation

  ============================================================================*/


/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "utility.h"
#include "utility_macros.h"
#include "statistics.h"
#include "mdefs.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "PE_declare.h"

#define DOWN_SAMPLE_DEFAULT  5
#define DEBUG        FALSE


enum ConstraintEstType {
  PROJECTION = 1,
  CFORCES,
  BASE_ONLY
};

/* local variables */
static double     sampling_freq;
static Matrix     ATA;
static Vector     ATb;
static Vector     beta;
static Vector     vbeta;
static Vector     beta_old;
static Vector     beta_proj;
static Matrix     data_pos=NULL;
static Matrix     data_vel=NULL;
static Matrix     data_acc=NULL;
static Matrix     data_u=NULL;
static Matrix     data_bc_pos=NULL;
static Matrix     data_bc_vel=NULL;
static Matrix     data_bc_acc=NULL;
static Matrix     data_bo_q=NULL;
static Matrix     data_bo_ad=NULL;
static Matrix     data_bo_add=NULL;
static iMatrix    data_endeff_c=NULL;
static Matrix     data_endeff_f=NULL;
static int        n_rows;
static int        n_cols;
static double     sse[N_DOFS+2*N_CART+1];
static int        n_sse;
static SL_Jstate  state[N_DOFS+1];
static SL_DJstate invdyn_state[N_DOFS+1];
static SL_Cstate  basec[1]; 
static SL_quat    baseo[1]; 
static int        get_mse=2;
static int        use_commands = TRUE;
static FILE      *datafp=NULL;
static int        write_data = FALSE;
static int        down_sample = DOWN_SAMPLE_DEFAULT;
static int        got_all_joint_data = FALSE;
static int        got_all_base_data = FALSE;
static int        got_all_constraint_data = FALSE;
static int        got_all_contact_force_data = FALSE;
static int        use_floating_base = FALSE;
static int        constraint_estimation_type = PROJECTION;
static int        use_parm_file = FALSE;
static int        filt_data = 10;
static int        vis_flag = TRUE;
static int        coul_flag = FALSE;
static int        spring_flag = FALSE;
static int        filt_data_dofs[N_DOFS+1];
static int        vis_flag_dofs[N_DOFS+1];
static int        coul_flag_dofs[N_DOFS+1];
static int        spring_flag_dofs[N_DOFS+1];
static char       parm_file_name[100]="xpest_parameters.cf";


static Vector	  least_square_weight=NULL; //used to perform a weighted least square for param est

#define LLSB(x)	((x) & 0xff)		/*!< 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)
#define LONGSWAP(x) ((LLSB(x) << 24) | \
		     (LNLSB(x) << 16)| \
		     (LNMSB(x) << 8) | \
		     (LMSB(x)))

/* global variables */
char   **argv_global;
int      argc_global;
char    *argv_prog_name;
int      servo_enabled = FALSE;
double   servo_time = 0;

/* global functions */

/* local functions */
static int add_to_regression(void);
static int read_file(char *fname);
static int filter_data(void);
static int regress_parameters(void);
static void do_math(SL_endeff *eff);
static int project_parameters(int metric_flag);
static double project_parameters_opt_func(double *vb);
static void project_parameters_gradient_func(double *vb, double *grad);
static void project_parameters_predict(double *vb, double *bp, double *b_m_bp);
static int read_parm_file(void);


extern SL_link    links[N_DOFS+1];
 
/*!*****************************************************************************
 *******************************************************************************
\note  main
\date  02/25/92 
   
\remarks 

	entyr program
	
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     argc : standard arguments to give some initial input to the program
 \param[in]     arv  : s.o.

 ******************************************************************************/
int
main(int argc, char **argv)
{
  int   i,j,n,rc;
  int  ans = 0;
  char fname[100]="ATA_ATb.mat";
  FILE *fid;
  int  metric_flag = FALSE;
  FILE *fp;
  
  /* copy the input arguments */
  argc_global    = argc;
  argv_global    = argv;
  argv_prog_name = argv[0];
  
  /* subtract 1 from argv to get rid of the 0-th argument which is the
     function name itself */
  argc_global -= 1;

  /* read the most recent settings */
  if ((fp=fopen(".xpest_prefs","r")) == NULL) 
    ;
  else {
    rc = fscanf(fp,"%d %d %d %d %d %d %d %s %d %d %d %d %d",
	   &real_robot_flag,
	   &get_mse,
	   &use_floating_base,
	   &constraint_estimation_type,
	   &use_commands,
	   &down_sample,
	   &use_parm_file,
	   parm_file_name,
	   &metric_flag,
	   &filt_data,
	   &vis_flag,
	   &coul_flag,
	   &spring_flag);
    fclose(fp);
  }

  /* read the configration file names */
  real_robot_flag = TRUE;
  get_int("Which parameter files: RealRobot=1 Simulation=0?",real_robot_flag,&real_robot_flag);
  setRealRobotOptions();

  /* some handy matrices: note that these matrices need to be big enough
     to include the dummy DOFs */
  ATA      = my_matrix(1,(N_DOFS+1)*N_RBD_PARMS,1,(N_DOFS+1)*N_RBD_PARMS);
  ATb      = my_vector(1,(N_DOFS+1)*N_RBD_PARMS);
  beta     = my_vector(1,(N_DOFS+1)*N_RBD_PARMS);
  vbeta    = my_vector(1,(N_DOFS+1)*N_RBD_PARMS);
  beta_old = my_vector(1,(N_DOFS+1)*N_RBD_PARMS);
  beta_proj= my_vector(1,(N_DOFS+1)*N_RBD_PARMS);

  //create a least square weight and make it to one by default
  least_square_weight = my_vector(1, N_DOFS+2*N_CART);
  vec_equal_scalar(1, least_square_weight);

  /* initialize the dynamics calculations */
  if (!init_dynamics())
    exit(-1);

  /* initialize the kinematics calculations */
  init_kinematics();

  /* read the sensor offset to know about joint range values */
  if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
    exit(-1);

  /* clear some matrices and vectors*/
  n_sse = 0;
  for (i=0; i<=N_DOFS_EST; ++i) {
    for (j=1; j<=6; ++j) {
      for (n=1; n<=6; ++n)
	Xinv[i][j][n]=0.0;
      st[i][j]=0.0;
    }
  }

  for (i=1; i<=N_DOFS+2*N_CART; ++i)
    sse[i]=0;

  for (i=0; i<=N_DOFS_EST; ++i)
    for (j=1; j<=N_RBD_PARMS; ++j)
      for (n=1; n<=6; ++n)
	A[i][n][j]=0.0;

  for (i=1; i<=N_DOFS_EST+2*N_CART; ++i)
    for (j=1; j<=N_RBD_PARMS*(N_DOFS_EST+1); ++j)
      K[i][j]=0.0;

  /* compute the parameter estimation matrices to make sure that the program can
     run without data files (the map array needs to be assinged) */
  do_math(endeff);

  /* copy the current RBD parameters into the beta_old vector */
  for (i=0; i<=N_DOFS; ++i) {
    beta_old[i*N_RBD_PARMS+1] = links[i].m;
    beta_old[i*N_RBD_PARMS+2] = links[i].mcm[_X_];
    beta_old[i*N_RBD_PARMS+3] = links[i].mcm[_Y_];
    beta_old[i*N_RBD_PARMS+4] = links[i].mcm[_Z_];
    beta_old[i*N_RBD_PARMS+5] = links[i].inertia[1][1];
    beta_old[i*N_RBD_PARMS+6] = links[i].inertia[1][2];
    beta_old[i*N_RBD_PARMS+7] = links[i].inertia[1][3];
    beta_old[i*N_RBD_PARMS+8] = links[i].inertia[2][2];
    beta_old[i*N_RBD_PARMS+9] = links[i].inertia[2][3];
    beta_old[i*N_RBD_PARMS+10]= links[i].inertia[3][3];
    beta_old[i*N_RBD_PARMS+11]= links[i].vis;
    beta_old[i*N_RBD_PARMS+12]= links[i].coul;
    beta_old[i*N_RBD_PARMS+13]= links[i].stiff;
    beta_old[i*N_RBD_PARMS+13]= links[i].cons;
  }

  /* calculate MSE */
  get_int("Calculate MSE & Parameters? Both=2, MSE=1, Parm=0",
	  get_mse,&get_mse);

  /* read the old ATA and ATb matrices? */
  if (!get_int("Read previous ATA and ATb matrices? Yes=1, No=0",ans,&ans))
    exit(-1);

  if (ans==1) {
    if (getFile(fname)) {
      fid = fopen(fname,"r");
      fread_mat(fid,ATA);
      fread_vec(fid,ATb);
      fclose(fid);
    }
  }

  if (!get_int("Use floating base estimation? Yes=1, No=0",
	       use_floating_base,&use_floating_base))
    exit(-1);

  if (use_floating_base) {
    if (!get_int("Which constraint estimation? Q-Project=1 C-Force=2 Base-Only=3",
		 constraint_estimation_type,&constraint_estimation_type))
      exit(-1);
  }

  if (!get_int("Use commands instead of load cells? Yes=1, No=0",
	       use_commands,&use_commands))
    exit(-1);

  if (!get_int("Write regression data? Yes=1, No=0",
	       write_data,&write_data))
    exit(-1);

  if (!get_int("Downsample using which data point?",down_sample,&down_sample))
    exit(-1);

  if (!get_int("Use parameter file for Filters/Vis/Coul/Stiff?",use_parm_file,&use_parm_file))
    exit(-1);

  if (use_parm_file) {
    if (!read_parm_file())
      use_parm_file = FALSE;
  }

  if (!get_int("Use full ATA metric for parameter projection?",metric_flag,&metric_flag))
    exit(-1);



  if (!use_parm_file) {

    if (!get_int("Filter data cutoff [%]? (0 or 100 for no filter)",filt_data,&filt_data))
      exit(-1);

    if (!get_int("Estimation viscous friction?",vis_flag,&vis_flag))
      exit(-1);

    if (!get_int("Estimation coulomb friction?",coul_flag,&coul_flag))
      exit(-1);
    
    if (!get_int("Estimation spring term?",spring_flag,&spring_flag))
      exit(-1);

    for (i=1; i<=N_DOFS; ++i) {
      filt_data_dofs[i] = filt_data;
      vis_flag_dofs[i] = vis_flag;
      coul_flag_dofs[i] = coul_flag;
      spring_flag_dofs[i] = spring_flag;
    }

  }

  /* remember the most recent settings */
  if ((fp=fopen(".xpest_prefs","w")) == NULL) 
    ;
  else {
    fprintf(fp,"%d %d %d %d %d %d %d %s %d %d %d %d %d",
	   real_robot_flag,
	   get_mse,
	   use_floating_base,
	   constraint_estimation_type,
	   use_commands,
	   down_sample,
	   use_parm_file,
	   parm_file_name,
	   metric_flag,
	   filt_data,
	   vis_flag,
	   coul_flag,
	   spring_flag);
    fclose(fp);
  }


  if (write_data)
    datafp = fopen("AbData.mat","w");


  /* loop through all the files */
  for (i=1; i<=argc_global; ++i) {

    printf("Processing %s\n",argv_global[i]);
    
    // read file
    if (!read_file(argv_global[i]))
      continue;

    // check whether this file allows for the desired estimation method
    if (use_floating_base) {
      if (constraint_estimation_type == BASE_ONLY) {
	if (! (got_all_base_data && got_all_contact_force_data)) {
	  printf("Missing variables for BASE-ONLY floating-base parameter estimation from this file\n");
	  continue;
	}
      } else {
	if (! (got_all_joint_data && got_all_base_data && got_all_constraint_data)) {
	  printf("Missing variables for floating-base parameter estimation from this file\n");
	  continue;
	}
	if (constraint_estimation_type == CFORCES) {
	  if (! (got_all_contact_force_data)) {
	    printf("Missing variables for floating-base parameter estimation with contact forces from this file\n");
	    continue;
	  }
	}
      }
    } else {
      if (!got_all_joint_data) {
	printf("Missing variables for parameter estimation from this file\n");
	continue;
      }
    }
    
    // filter
    filter_data();

    // add data
    add_to_regression();
    
  }

  if (write_data)
    fclose(datafp);

  /* write out the ATA and ATb */
  fid = fopen(fname,"w");
  if (fid == NULL) {
    printf("Couldn't open file >%s< for write\n",fname);
    exit(-1);
  }
  fwrite_mat(fid,ATA);
  fwrite_vec(fid,ATb);
  fclose(fid);

  if (get_mse != 1)
    regress_parameters();

  // create physically consistent parameters
  project_parameters(metric_flag);

  return TRUE;
	
}

/*!*****************************************************************************
 *******************************************************************************
  \note  read_file
  \date  June 1999

  \remarks 

  read an MRDPLOT file and sort the variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
read_file(char *fname)
{
  int    j,i,r,k;
  static char string[100];
  FILE * fp;
  int    found = FALSE;
  char **vnames;
  char **units;
  Matrix buff;
  int    aux;

  /* clear the current parameters */
  if (data_pos != NULL) {
    my_free_matrix(data_pos,1,N_DOFS,1,n_rows);
    data_pos = NULL;
  }
  if (data_vel != NULL) {
    my_free_matrix(data_vel,1,N_DOFS,1,n_rows);
    data_vel = NULL;
  } 
  if (data_acc != NULL) {
    my_free_matrix(data_acc,1,N_DOFS,1,n_rows);
    data_acc = NULL;
  }
  if (data_u != NULL) {
    my_free_matrix(data_u,1,N_DOFS,1,n_rows);
    data_u = NULL;
  }
  if (data_bc_pos != NULL) {
    my_free_matrix(data_bc_pos,1,N_CART,1,n_rows);
    data_bc_pos = NULL;
  }
  if (data_bc_vel != NULL) {
    my_free_matrix(data_bc_vel,1,N_CART,1,n_rows);
    data_bc_vel = NULL;
  }
  if (data_bc_acc != NULL) {
    my_free_matrix(data_bc_acc,1,N_CART,1,n_rows);
    data_bc_acc = NULL;
  }
  if (data_bo_q != NULL) {
    my_free_matrix(data_bo_q,1,N_QUAT,1,n_rows);
    data_bo_q = NULL;
  }
  if (data_bo_ad != NULL) {
    my_free_matrix(data_bo_ad,1,N_CART,1,n_rows);
    data_bo_ad = NULL;
  }
  if (data_bo_add != NULL) {
    my_free_matrix(data_bo_add,1,N_CART,1,n_rows);
    data_bo_add = NULL;
  }
  if (data_endeff_c != NULL) {
    my_free_imatrix(data_endeff_c,1,N_ENDEFFS*2*N_CART,1,n_rows);
    data_endeff_c = NULL;
  }
  if (data_endeff_f != NULL) {
    my_free_matrix(data_endeff_f,1,N_ENDEFFS*2*N_CART,1,n_rows);
    data_endeff_f = NULL;
  }

  /* open the file, and parse the parameters */
  buff   = NULL;
  vnames = NULL;
  units  = NULL;
  if (!mrdplot_convert(fname, &buff, &vnames, &units, &sampling_freq, &n_cols, &n_rows))
    return FALSE;

  /* create the pos, vel, acc , uff matrices that define the trajectory */
  data_pos      = my_matrix(1,N_DOFS,1,n_rows);
  data_vel      = my_matrix(1,N_DOFS,1,n_rows);
  data_acc      = my_matrix(1,N_DOFS,1,n_rows);
  data_u        = my_matrix(1,N_DOFS,1,n_rows);
  data_bc_pos   = my_matrix(1,N_CART,1,n_rows);
  data_bc_vel   = my_matrix(1,N_CART,1,n_rows);
  data_bc_acc   = my_matrix(1,N_CART,1,n_rows);
  data_bo_q     = my_matrix(1,N_QUAT,1,n_rows);
  data_bo_ad    = my_matrix(1,N_CART,1,n_rows);
  data_bo_add   = my_matrix(1,N_CART,1,n_rows);
  data_endeff_c = my_imatrix(1,N_ENDEFFS*2*N_CART,1,n_rows);
  data_endeff_f = my_matrix(1,N_ENDEFFS*2*N_CART,1,n_rows);

  printf("Looking for variables ... ");

  got_all_joint_data         = TRUE;
  got_all_base_data          = TRUE;
  got_all_constraint_data    = TRUE;
  got_all_contact_force_data = TRUE;
  
  /* shuffle the matrices */
  for (i=1;i<=N_DOFS-N_DOFS_EST_SKIP;++i) {
    
    sprintf(string, "%s_th",joint_names[i]);
    
    for (j=1;j<=n_cols;++j) {
      if (strcmp(string,vnames[j])==0) {
	
	found = TRUE;
	printf("\n%s ",string);
	
	/* fill the pos matrix using the right column from buffer*/
	for (r=1;r<=n_rows;++r)
	  data_pos[i][r] = buff[r][j];
	
	/* also check for velocity, acceleration, and uff information
	   use the same value of n-vars to fill the remaining matrices*/
	
	sprintf(string, "%s_thd",joint_names[i]);
	for (k=1;k<=n_cols;++k) {
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_vel[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	sprintf(string, "%s_thdd",joint_names[i]);
	for (k=1;k<=n_cols;++k) {
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_acc[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	
	if (use_commands) 
	  sprintf(string, "%s_u",joint_names[i]);
	else
	  sprintf(string, "%s_load",joint_names[i]);
	
	for (k=1;k<=n_cols;++k){
	  if (strcmp(string,vnames[k])==0){
	    printf("%s ",string);
	    for (r=1;r<=n_rows;++r)
	      data_u[i][r] = buff[r][k];
	    break;
	  }
	  if (k > n_cols)
	    got_all_joint_data = FALSE;
	}
	
	/* assume only one variable of each kind exists*/
	break;
      }
    }
    
    if (j > n_cols)
      got_all_joint_data = FALSE;
    
  }
  
  printf("\n\n");
  printf("-------> Status of joint data = %d\n\n",got_all_joint_data);
  
  /* look for base variables */
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"x"},{"y"},{"z"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_pos[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"xd"},{"yd"},{"zd"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_vel[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"xdd"},{"ydd"},{"zdd"}};
    
    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bc_acc[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  
  for (i=1; i<=N_QUAT; ++i) {
    char names[][20] = {{""},{"q0"},{"q1"},{"q2"},{"q3"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_q[i][r] = buff[r][j];
	if (i==N_QUAT)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"ad"},{"bd"},{"gd"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_ad[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }
  
  for (i=1; i<=N_CART; ++i) {
    char names[][20] = {{""},{"add"},{"bdd"},{"gdd"}};

    sprintf(string, "base_%s",names[i]);
    for (j=1;j<=n_cols;++j){
      if (strcmp(string,vnames[j])==0){
	printf("%s ",string);
	for (r=1;r<=n_rows;++r)
	  data_bo_add[i][r] = buff[r][j];
	if (i==N_CART)
	  printf("\n");
	break;
      }
    }
    if (j > n_cols)
      got_all_base_data = FALSE;
  }

  printf("\n");
  printf("-------> Status of base data = %d\n\n",got_all_base_data);

  for (i=1; i<=N_ENDEFFS; ++i) {
    char names[][20] = {{""},{"x"},{"y"},{"z"},{"a"},{"b"},{"g"}};

    for (k=1; k<=2*N_CART; ++k) {
      sprintf(string, "%s_cons_%s",cart_names[i],names[k]);
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  printf("%s ",string);
	  for (r=1;r<=n_rows;++r)
	    data_endeff_c[(i-1)*2*N_CART+k][r] = buff[r][j];
	  if (k==2*N_CART)
	    printf("\n");
	  break;
	}
      }
      if (j > n_cols)
	got_all_constraint_data = FALSE;
    }
  }

  printf("\n");
  printf("-------> Status of constraint data = %d\n\n",got_all_constraint_data);

  for (i=1; i<=N_ENDEFFS; ++i) {
    char names[][20] = {{""},{"cfx"},{"cfy"},{"cfz"},{"cta"},{"ctb"},{"ctg"}};

    for (k=1; k<=2*N_CART; ++k) {
      sprintf(string, "%s_%s",cart_names[i],names[k]);
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  printf("%s ",string);
	  for (r=1;r<=n_rows;++r)
	    data_endeff_f[(i-1)*2*N_CART+k][r] = buff[r][j];
	  if (k==2*N_CART)
	    printf("\n");
	  break;
	}
      }
      if (j > n_cols)
	got_all_contact_force_data = FALSE;
    }
  }

  printf("\n");
  printf("-------> Status of contact force data = %d\n\n",got_all_contact_force_data);

  printf("\n -------- Read %d rows with %d columns ---------\n",n_rows,n_cols);
  
  
  /* free up memory by deallocating resources */
  my_free_matrix (buff,1,n_rows,1,n_cols);
  for (i=1;i<=n_cols;++i){
    free(vnames[i] ); 
    free(units[i]) ;
  }
  
  free(units);
  free(vnames);
  
  return found;
  
}

/*!*****************************************************************************
 *******************************************************************************
  \note  filter_data
  \date  June 1999

  \remarks 

  differentiate and filter

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
filter_data(void)
{
  int j,i,r;

  for (i=1; i<=N_DOFS; ++i) {
    if (filt_data_dofs[i] > 0 && filt_data_dofs[i] < 100) {
      filtfilt(data_pos[i],n_rows,filt_data,data_pos[i]);
      diff(data_pos[i], n_rows, 1./sampling_freq, data_vel[i]);
      diff2(data_pos[i], n_rows, 1./sampling_freq, data_acc[i]);
    }
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  add_to_regression
  \date  June 1999

  \remarks 

  recursively add data to the regression analysis

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
add_to_regression(void)
{
  int    j,i,m,n,r,s;
  int    count=0;
  int    flag;
  float  aux;
  int    n_cons;
  int    use_constraint;
  int    nr_K = 0;

  MY_MATRIX(Jc,1,2*N_CART*N_ENDEFFS,1,N_DOFS+2*N_CART);
  MY_MATRIX(Q,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_MATRIX(Qu,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_MATRIX(R,1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
  MY_VECTOR(sv,1,N_DOFS+2*N_CART);
  MY_MATRIX(Kp,1,N_DOFS+2*N_CART,1,(N_DOFS+1)*N_RBD_PARMS);
  MY_VECTOR(Yp,1,N_DOFS+2*N_CART);
  MY_VECTOR(cf,1,N_CART*2*N_ENDEFFS);

  printf("\nAdd data to regression ");

  for (i=1+100; i<=n_rows-100; i+=down_sample) {

    /* if data is too close to the joint range, ignore the data */
    flag = FALSE;
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      if (data_pos[j][i] > joint_range[j][MAX_THETA]-.025 ||
	  data_pos[j][i] < joint_range[j][MIN_THETA]+.025) {
	flag = TRUE;
      }
    }
    if (flag)
      continue;

    if (++count%1000==0) {
      printf("%d",count);
      fflush(stdout);
    } else if (count%100==0) {
      printf(".");
      fflush(stdout);
    }

    for (j=1; j<=N_DOFS; ++j) {
      invdyn_state[j].th   = state[j].th   = data_pos[j][i];
      invdyn_state[j].thd  = state[j].thd  = data_vel[j][i];
      invdyn_state[j].thdd = state[j].thdd = data_acc[j][i];
      state[j].u           = data_u[j][i];
      invdyn_state[j].uff  = 0;
   }

    for (j=1; j<=N_CART; ++j) {
      basec[0].x[j]   = data_bc_pos[j][i];
      basec[0].xd[j]  = data_bc_vel[j][i];
      basec[0].xdd[j] = data_bc_acc[j][i];
    }

    aux = 0;
    for (j=1; j<=N_QUAT; ++j) {
      baseo[0].q[j]   = data_bo_q[j][i];
      aux += baseo[0].q[j];
    }
    if (aux == 0) // not orientation in data
      baseo[0].q[_Q0_] = 1.0;

    for (j=1; j<=N_CART; ++j) {
      baseo[0].ad[j]  = data_bo_ad[j][i];
      baseo[0].add[j] = data_bo_add[j][i];
    }

    n_cons = 0;
    for (j=1; j<=N_ENDEFFS; ++j) {
      for (m=1; m<=2*N_CART; ++m) {
	endeff[j].c[m] = data_endeff_c[(j-1)*2*N_CART+m][i];
	n_cons += endeff[j].c[m];
      }
    }
    
    if (n_cons > 0) {
      use_constraint = TRUE;
    } else {
      use_constraint = FALSE;
    }

    for (j=1; j<=N_ENDEFFS*2*N_CART; ++j)
	cf[j] = data_endeff_f[j][i];
    
    /* compute the parameter estimation matrices */
    do_math(endeff);

#if 0    
    /* add parameters that only act only independenty: this has been moved to after the projection */
    for (j=1; j<=N_DOFS; ++j) {
      
      /* viscous friction */
      Kp[map[j]][map[j]*N_RBD_PARMS+VIS] = state[j].thd;
      
      /* coulomb friction */
      Kp[map[j]][map[j]*N_RBD_PARMS+COUL] = COULOMB_FUNCTION(state[j].thd);

      /* stiffness due to spring terms */
      Kp[map[j]][map[j]*N_RBD_PARMS+STIFF] = state[j].th;

      /* constant offset of spring  */
      Kp[map[j]][map[j]*N_RBD_PARMS+CONS] = 1.0;
    }
#endif

    /* ======================================================================================== */
    /* zero main regression matrices */
    vec_zero(Yp);
    mat_zero(Kp);
    
    /* project the K matrix if needed */
    if (use_constraint) {
      int nr,nc;

      computeConstraintJacobian(state, basec, baseo, endeff,Jc,&nr,&nc); 

      if (constraint_estimation_type == PROJECTION) {
	
	computeQR(Jc, nr, nc, Q, Qu, R, sv);
	
	// create a range space projection
	mat_mult_outer_size(Qu,N_DOFS+2*N_CART,N_DOFS+2*N_CART-n_cons,Qu);
	nr_K = N_DOFS+2*N_CART;
	
	// run the K matrix through this projection and remap
	for (j=1; j<=nr_K; ++j) {
	  for (m=0; m<=N_DOFS; ++m) {
	    for (r=1; r<=N_RBD_PARMS; ++r ) {
	      
	      Kp[j][m*N_RBD_PARMS+r] = 0.0;
	      
	      for (n=1; n<=N_DOFS+2*N_CART; ++n) {
		int map_n;
		
		if (n > N_DOFS)
		  map_n = N_DOFS_EST+(n-N_DOFS);
		else
		  map_n = map[n];
		
		Kp[j][m*N_RBD_PARMS+r] += Qu[n][j]*K[map_n][map[m]*N_RBD_PARMS+r];
	      }
	    }
	  }
	  
	  Yp[j] = 0.0;
	  for (n=1; n<=N_DOFS+2*N_CART; ++n) {
	    int map_n;
	    
	    if (n > N_DOFS)
	      map_n = N_DOFS_EST+(n-N_DOFS);
	    else
	      map_n = map[n];
	    
	    Yp[j] += Qu[n][j]*Y[map_n];
	  }
	  
	}

      } else if (constraint_estimation_type == CFORCES) {
	double Yc[2*N_CART+N_DOFS+1];

	for (j=1; j<=N_DOFS+6; ++j) {
	  Yc[j] = 0.0;
	  for (m=1; m<=2*N_CART*N_ENDEFFS; ++m)
	    Yc[j] += cf[m]*Jc[m][j];
	}

	nr_K = N_DOFS+2*N_CART;
	
	for (j=1; j<=nr_K; ++j) {
	  int map_j;
	  
	  if (j > N_DOFS)
	    map_j = N_DOFS_EST+(j-N_DOFS);
	  else
	    map_j = map[j];
	  
	  for (m=0; m<=N_DOFS; ++m)
	    for (r=1; r<=N_RBD_PARMS; ++r)
	      Kp[j][m*N_RBD_PARMS+r] = K[map_j][map[m]*N_RBD_PARMS+r];
	  
	  Yp[j] = Y[map_j] + Yc[j];

	}

      } else if (constraint_estimation_type == BASE_ONLY) {
	double Yc[2*N_CART+1];

	for (j=1; j<=2*N_CART; ++j) {
	  Yc[j] = 0.0;
	  for (m=1; m<=2*N_CART*N_ENDEFFS; ++m)
	    Yc[j] += cf[m]*Jc[m][N_DOFS+j];
	}

	nr_K = 2*N_CART;
	
	for (j=1; j<=nr_K; ++j) {
	  int map_j;
	  
	  map_j = N_DOFS_EST+j;
	  
	  for (m=0; m<=N_DOFS; ++m)
	    for (r=1; r<=N_RBD_PARMS; ++r)
	      Kp[j][m*N_RBD_PARMS+r] = K[map_j][map[m]*N_RBD_PARMS+r];
	  
	  Yp[j] = Y[map_j] + Yc[j];

	}


      }

    } else { // no constraints

      nr_K = N_DOFS+2*N_CART;

      for (j=1; j<=nr_K; ++j) {
	int map_j;
	
	if (j > N_DOFS)
	  map_j = N_DOFS_EST+(j-N_DOFS);
	else
	  map_j = map[j];

	for (m=0; m<=N_DOFS; ++m)
	  for (r=1; r<=N_RBD_PARMS; ++r)
	    Kp[j][m*N_RBD_PARMS+r] = K[map_j][map[m]*N_RBD_PARMS+r];

	Yp[j] = Y[map_j];

      }
    }


    /* add parameters that only act only independenty on one joint: this is done AFTER projection as 
       these per-joint forces should not be projected */
    
    for (j=1; j<=N_DOFS; ++j) {
      
      /* viscous friction */
      if (vis_flag_dofs[j])
	Kp[j][j*N_RBD_PARMS+VIS] = state[j].thd;
      
      /* coulomb friction */
      if (coul_flag_dofs[j])
	Kp[j][j*N_RBD_PARMS+COUL] = COULOMB_FUNCTION(state[j].thd,coulomb_slope);

      /* stiffness due to spring terms */
      if (spring_flag_dofs[j])
	Kp[j][j*N_RBD_PARMS+STIFF] = state[j].th;

      /* constant offset of spring  */
      if (spring_flag_dofs[j])
	Kp[j][j*N_RBD_PARMS+CONS] = 1.0;

    }

    if (count == 1 && 0) {
      printf("nr_K=%d\n",nr_K);
      print_mat_size("Jc",Jc,n_cons,N_DOFS+6);
      print_mat("Q",Q);
      print_mat_size("Qu",Qu,N_DOFS+6,nr_K);
      print_mat_size("Kp",Kp,nr_K,(N_DOFS+1)*N_RBD_PARMS);
      print_vec("beta_old",beta_old);
      getchar();

      FILE *fp = fopen("mist","w");
      for (j=1; j<=N_DOFS_EST+6; ++j) {
	for (m=1; m<=(N_DOFS_EST+1)*N_RBD_PARMS; ++m) {
	  fprintf(fp,"% 5.2f ",K[j][m]);
      }
	fprintf(fp,"\n");
      }
      fprintf(fp,"\n");
      fclose(fp);
      getchar();
    }


    /* predict the outcome with old parameters and update sse */
    if (get_mse != 0) {
      double pred[N_DOFS+2*N_CART+1];

      ++n_sse;

      for (j=1; j<=nr_K; ++j) {
	pred[j] = 0.0;
	for (m=1; m<=(N_DOFS+1)*N_RBD_PARMS; ++m)
	  pred[j] += Kp[j][m]*beta_old[m];
      }

      for (j=1; j<=N_DOFS+2*N_CART; ++j) {
	sse[j] += sqr(Yp[j]-pred[j]);

      }

    }

    if (get_mse == 1)
      continue;
    
    /* add data to regression -------------------------------- */
    for (j=1; j<=nr_K; ++j) { // these are the # rows in the K matrix
      
      for (m=1; m<=(N_DOFS+1)*N_RBD_PARMS; ++m)
	for (n=m; n<=(N_DOFS+1)*N_RBD_PARMS; ++n)
	  ATA[m][n] += Kp[j][m]*Kp[j][n] * least_square_weight[j];
      
      for (m=1; m<=(N_DOFS+1)*N_RBD_PARMS; ++m) {
	ATb[m] += Kp[j][m]*Yp[j]* least_square_weight[j];
	aux = Kp[j][m];
	if (write_data) {
#ifdef BYTESWAP
	  aux = byteswap_float(aux);
#endif
	  if (fwrite(&aux,sizeof(float),1,datafp)!= 1) {
	    printf( "cannot fwrite data.\n" );
	    exit(-1);
	  }
	}
      }

      if (write_data) {
	aux = Yp[j];
#ifdef BYTESWAP
	aux = byteswap_float(aux);
#endif
	if (fwrite(&aux,sizeof(float),1,datafp)!= 1) {
	  printf( "cannot fwrite data.\n" );
	  exit(-1);
	}
      }
    }
    
  }
      
  /* ATA is symmetric */
  for (m=1; m<=(N_DOFS+1)*N_RBD_PARMS; ++m) {
    for (n=m; n<=(N_DOFS+1)*N_RBD_PARMS; ++n) {
      ATA[n][m] = ATA[m][n];
    }
  }

  printf("done\n");

  /* print the current SSE */
  
  printf("\nMSE (STD) Statistics (based on %d data points):\n",count);

  if (constraint_estimation_type != BASE_ONLY) {

    for (j=1; j<=N_DOFS; j=j+3) {
      printf("%6s: % 7.2f (% 7.2f)   ",
	     joint_names[j],sse[j]/(double)n_sse,sqrt(sse[j]/(double)n_sse));
      if (j+1 > N_DOFS)
	break;
      printf("%6s: % 7.2f (% 7.2f)   ",
	     joint_names[j+1],sse[j+1]/(double)n_sse,sqrt(sse[j+1]/(double)n_sse));
      if (j+2 > N_DOFS)
	break;
      printf("%6s: % 7.2f (% 7.2f)\n",
	     joint_names[j+2],sse[j+2]/(double)n_sse,sqrt(sse[j+2]/(double)n_sse));
    }
    printf("\n");
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fx",sse[N_DOFS+1]/(double)n_sse,sqrt(sse[N_DOFS+1]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fy",sse[N_DOFS+2]/(double)n_sse,sqrt(sse[N_DOFS+2]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_fz",sse[N_DOFS+3]/(double)n_sse,sqrt(sse[N_DOFS+3]/(double)n_sse));
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_tx",sse[N_DOFS+4]/(double)n_sse,sqrt(sse[N_DOFS+4]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_ty",sse[N_DOFS+5]/(double)n_sse,sqrt(sse[N_DOFS+5]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_tz",sse[N_DOFS+6]/(double)n_sse,sqrt(sse[N_DOFS+6]/(double)n_sse));
    printf("\n");

  } else {

    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fx",sse[1]/(double)n_sse,sqrt(sse[1]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_fy",sse[2]/(double)n_sse,sqrt(sse[2]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_fz",sse[3]/(double)n_sse,sqrt(sse[3]/(double)n_sse));
    
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_tx",sse[4]/(double)n_sse,sqrt(sse[4]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)   ",
	   "base_ty",sse[5]/(double)n_sse,sqrt(sse[5]/(double)n_sse));
    printf("%6s: % 7.2f (% 7.2f)\n",
	   "base_tz",sse[6]/(double)n_sse,sqrt(sse[6]/(double)n_sse));
    printf("\n");


  }
    
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  regress_parameters
  \date  June 1999

  \remarks 

  performs a SVD regression

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
regress_parameters(void)
{
  int j,i,m,n;
  FILE *fp;
  double svdr = 0.01;
  double svdt = 0.01;

  printf("\nStarting SVD ...");
  
  correct_command_svd(ATA, ATb, svdr, svdt, beta, (N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS);
  
  printf("done\n");

  /* write the results to a file */
  fp = fopen("parameters.est","w");
  if (fp==NULL) {
    printf("Couldn't open parameter.est for write\n");
    exit(-1);
  }

  for (i=0; i<=N_DOFS; ++i) {
    fprintf(fp,"%s ",joint_names[i]);

    /* make the file look nice (Jorg, Sun Nov 21 20:31:20 PST 1999) */
    for (j=strlen(joint_names[i]); j<5; j++)
     fprintf(fp," ");

    for (m=1; m<=N_RBD_PARMS; ++m) {
      fprintf(fp,"%9f ",beta[i*N_RBD_PARMS+m]);
    }
    fprintf(fp,"\n");
  }

  fclose(fp);

  return TRUE;

}



/*!*****************************************************************************
 *******************************************************************************
  \note  do_math
  \date  June 1999

  \remarks 

  includes the math for parameter estimation -- it is useful to have this
  in a function for faster compilation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void
do_math(SL_endeff *eff) {

#include "PE_math.h"

}

/*!*****************************************************************************
 *******************************************************************************
  \note  project_parameters
  \date  Oct 2010

  \remarks 

  uses gradient descent to create physically consistant parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in] metric_flag : use ATA as metric int the cost function. While this
                          is theoretcially correct, it turns out to create a
                          very sensitive optimization problem. Using the 
			  identity matrix in steady essentially projects the
                          parameters per DOF in the minimal reconstruction error
                          way, which seem to be more sane.

 ******************************************************************************/
#define RIDGE 1.e-6
static int 
project_parameters(int metric_flag)
{
  int j,i,m,n;
  FILE *fp;
  int iter;
  double fret;
  double aux;

  printf("\nStarting Parameter Projection ...\n");

  // iterate until convergence with comjugate gradient method
  
  // initialize the virtual parameters to random numbers
  for (i=0; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    for (j=1; j<=N_RBD_PARMS; ++j)
      vbeta[i*N_RBD_PARMS+j]=gaussian(0.0,0.01);

  // for all parameters that are essentially zero, kill the elements in ATA
  for (i=1; i<=N_RBD_PARMS*(N_DOFS-N_DOFS_EST_SKIP+1); ++i)
    for (j=1; j<=N_RBD_PARMS*(N_DOFS-N_DOFS_EST_SKIP+1); ++j) {

      if (!metric_flag) {  // make the ATA metric the identity matrix
	if (i!=j) {
	  ATA[i][j] = 0.0;
	} else {
	  ATA[i][j] = 1.0;
	}
      }

      if ( (j%N_RBD_PARMS) >=5 && (j%N_RBD_PARMS) <=10) {// inertial parameters have smaller cutoff
	if (fabs(beta[i]) < 1.e-6 || fabs(beta[j]) < 1.e-6)
	  ATA[i][j] = 0.0;
      } else {
	  if (fabs(beta[i]) < 1.e-3 || fabs(beta[j]) < 1.e-3)
	    ATA[i][j] = 0.0;
      }
    }

  // create the frobenious norm of ATA and devide ATA by it, for numerical stabilty
  aux = 0.0;
  for (i=1; i<=N_RBD_PARMS*(N_DOFS-N_DOFS_EST_SKIP+1); ++i)
    for (j=1; j<=N_RBD_PARMS*(N_DOFS-N_DOFS_EST_SKIP+1); ++j)
      aux += sqr(ATA[i][j]);

  aux = sqrt(aux);

  // this is done over the entire matrix -- just to not leave some part of the matrix
  // inconistent (not really needed).
  for (i=1; i<=N_RBD_PARMS*(N_DOFS+1); ++i)
    for (j=1; j<=N_RBD_PARMS*(N_DOFS+1); ++j)
      ATA[i][j] /= aux;

  // perform the optimization
  iter = 10000;
  /*
  my_frprmn(vbeta,(N_DOFS-N_DOFS_EST_SKIP+1)*N_RBD_PARMS,1.e-10,&iter,&fret,
	    project_parameters_opt_func,
	    project_parameters_gradient_func);
  */
  my_dfpmin(vbeta,(N_DOFS-N_DOFS_EST_SKIP+1)*N_RBD_PARMS,1.e-10,&iter,&fret,
	    project_parameters_opt_func,
	    project_parameters_gradient_func);

  // the final prediction
  project_parameters_predict(vbeta, beta_proj, NULL);
  
  printf("done\n");

  /* write the results to a file */
  fp = fopen("parameters_proj.est","w");
  if (fp==NULL) {
    printf("Couldn't open parameter.est for write\n");
    exit(-1);
  }

  for (i=0; i<=N_DOFS; ++i) {
    fprintf(fp,"%s ",joint_names[i]);

    /* make the file look nice (Jorg, Sun Nov 21 20:31:20 PST 1999) */
    for (j=strlen(joint_names[i]); j<5; j++)
     fprintf(fp," ");

    for (m=1; m<=N_RBD_PARMS; ++m) {
      fprintf(fp,"%9f ",beta_proj[i*N_RBD_PARMS+m]);
    }
    fprintf(fp,"\n");
  }

  fclose(fp);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  project_parameters_opt_fun
  \date  Oct 2010

  \remarks 

  computes the current cost for the optimization

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  \param[in]  vb: virtual parameter vector (to be optimized)

 ******************************************************************************/
static double
project_parameters_opt_func(double *vb) 
{
  int i;
  MY_VECTOR(bp,1,(N_DOFS+1)*N_RBD_PARMS)
  MY_VECTOR(b_m_bp,1,(N_DOFS+1)*N_RBD_PARMS)

  // compute the predicted parameters and the difference between true and predicted
  project_parameters_predict(vb, bp, b_m_bp);

  // compute the optimization function
  return 0.5*mat_mahal_size(ATA,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,b_m_bp) +
    0.5*vec_mult_inner(vb,vb)*RIDGE;
}

/*!*****************************************************************************
 *******************************************************************************
  \note  project_parameters_gradient_func
  \date  Oct 2010

  \remarks 

  computes the gradient for optimization

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  \param[in]    vb: virtual parameter vector (to be optimized)
  \param[in]  grad: gradient of cost function

 ******************************************************************************/
static void
project_parameters_gradient_func(double *vb, double *grad) 
{
  int i,j,r;
  double vm, cmx, cmy, cmz, vI11, vI12, vI13, vI22, vI23, vI33, vvis, vcol, vstiff, vcons;
  MY_MATRIX(Jo,1,(N_DOFS+1)*N_RBD_PARMS,1,(N_DOFS+1)*N_RBD_PARMS);
  MY_MATRIX(Jt,1,N_RBD_PARMS,1,N_RBD_PARMS);
  MY_VECTOR(bp,1,(N_DOFS+1)*N_RBD_PARMS)
  MY_VECTOR(b_m_bp,1,(N_DOFS+1)*N_RBD_PARMS)
  MY_VECTOR(temp,1,(N_DOFS+1)*N_RBD_PARMS)

  // compute the predicted parameters and the difference between true and predicted
  project_parameters_predict(vb, bp, b_m_bp);


  // need the Jacobian from virtual to actual parameters
  for (i=0; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {

    j=i*N_RBD_PARMS;

    vm     = vb[j+1];
    cmx    = vb[j+2];
    cmy    = vb[j+3];
    cmz    = vb[j+4];
    vI11   = vb[j+5];
    vI12   = vb[j+6];
    vI13   = vb[j+7];
    vI22   = vb[j+8];
    vI23   = vb[j+9];
    vI33   = vb[j+10];
    vvis   = vb[j+11];
    vcol   = vb[j+12];
    vstiff = vb[j+13];
    vcons  = vb[j+14];


    Jt[1][1]  = 2*vm;
    Jt[2][1]  = 2*cmx*vm;
    Jt[3][1]  = 2*cmy*vm;
    Jt[4][1]  = 2*cmz*vm;
    Jt[5][1]  = 2*(sqr(cmy)+sqr(cmz))*vm;
    Jt[6][1]  = (-2)*cmx*cmy*vm;
    Jt[7][1]  = (-2)*cmx*cmz*vm;
    Jt[8][1]  = 2*(sqr(cmx)+sqr(cmz))*vm;
    Jt[9][1]  = (-2)*cmy*cmz*vm;
    Jt[10][1] = 2*(sqr(cmx)+sqr(cmy))*vm;

    Jt[2][2]  = sqr(vm);
    Jt[6][2]  = (-1)*cmy*sqr(vm);
    Jt[7][2]  = (-1)*cmz*sqr(vm);
    Jt[8][2]  = 2*cmx*sqr(vm);
    Jt[10][2] = 2*cmx*sqr(vm);

    Jt[3][3]  =	sqr(vm);
    Jt[5][3]  =	2*cmy*sqr(vm);
    Jt[6][3]  =	(-1)*cmx*sqr(vm);
    Jt[9][3]  = (-1)*cmz*sqr(vm);
    Jt[10][3] = 2*cmy*sqr(vm);

    Jt[4][4] = sqr(vm);
    Jt[5][4] = 2*cmz*sqr(vm);
    Jt[7][4] = (-1)*cmx*sqr(vm);
    Jt[8][4] = 2*cmz*sqr(vm);
    Jt[9][4] = (-1)*cmy*sqr(vm);

    Jt[5][5] = 2*vI11;
    Jt[6][5] = vI12;
    Jt[7][5] = vI13;

    Jt[6][6] = vI11;
    Jt[8][6] = 2*vI12;
    Jt[9][6] = vI13;

    Jt[7][7] = vI11;
    Jt[9][7] = vI12;
    Jt[10][7] = 2*vI13;

    Jt[8][8] = 2*vI22;
    Jt[9][8] = vI23;

    Jt[9][9]  = vI22;
    Jt[10][9] = 2*vI23;

    Jt[10][10] = 2*vI33;

    Jt[11][11] = 2*vvis;

    Jt[12][12] = 2*vcol;

    Jt[13][13] = 2*vstiff;

    Jt[13][14] = 2*vstiff*vcons;
    Jt[14][14] = sqr(vstiff);
    

    for (j=1; j<=N_RBD_PARMS; ++j)
      for (r=1; r<=N_RBD_PARMS; ++r)
	Jo[i*N_RBD_PARMS+j][i*N_RBD_PARMS+r]=Jt[j][r];

  }

  // compute the gradient g = -((beta_uc-beta_pred)'*XTX*J)';
  vec_mat_mult_size(b_m_bp,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,
		    ATA,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,temp);
  vec_mat_mult_size(temp,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,
		    Jo,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS,grad);

  for (i=1; i<=(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS; ++i)
    grad[i] *= -1.0;

  // shrinkage term
  for (i=1; i<=(N_DOFS+1-N_DOFS_EST_SKIP)*N_RBD_PARMS; ++i)
    grad[i] -= vb[i]*RIDGE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  project_parameters_predict
  \date  Oct 2010

  \remarks 

  computes the constraint parameters and difference between constraint and
  unconstraint parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  \param[in]        vb: virtual parameter vector (to be optimized)
  \param[out]       bp: predicted constraint parameters
  \param[out]   b_m_bp: difference between beta and vbeta

 ******************************************************************************/
static void
project_parameters_predict(double *vb, double *bp, double *b_m_bp)
{
  int i,j;
  double vm, cmx, cmy, cmz, vI11, vI12, vI13, vI22, vI23, vI33, vvis, vcol, vstiff, vcons;

  for (i=0; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {

    j=i*N_RBD_PARMS;

    vm     = vb[j+1];
    cmx    = vb[j+2];
    cmy    = vb[j+3];
    cmz    = vb[j+4];
    vI11   = vb[j+5];
    vI12   = vb[j+6];
    vI13   = vb[j+7];
    vI22   = vb[j+8];
    vI23   = vb[j+9];
    vI33   = vb[j+10];
    vvis   = vb[j+11];
    vcol   = vb[j+12];
    vstiff = vb[j+13];
    vcons  = vb[j+14];

    bp[j+1]  = sqr(vm);
    bp[j+2]  = cmx*sqr(vm);
    bp[j+3]  = cmy*sqr(vm);
    bp[j+4]  = cmz*sqr(vm);
    bp[j+5]  = sqr(vI11)+(sqr(cmy)+sqr(cmz))*sqr(vm);
    bp[j+6]  = vI11*vI12+(-1)*cmx*cmy*sqr(vm);
    bp[j+7]  = vI11*vI13+(-1)*cmx*cmz*sqr(vm);
    bp[j+8]  = sqr(vI12)+sqr(vI22)+(sqr(cmx)+sqr(cmz))*sqr(vm);
    bp[j+9]  = vI12*vI13+vI22*vI23+(-1)*cmy*cmz*sqr(vm);
    bp[j+10] = sqr(vI13)+sqr(vI23)+sqr(vI33)+(sqr(cmx)+sqr(cmy))*sqr(vm);
    bp[j+11] = sqr(vvis);
    bp[j+12] = sqr(vcol);
    bp[j+13] = sqr(vstiff);
    bp[j+14] = sqr(vstiff)*vcons;

    // zero out prediction for those parameters where beta is zero -- those should not 
    // affect the cost
    for (j=i*N_RBD_PARMS+1; j<=(i+1)*N_RBD_PARMS; ++j) {
      if (fabs(beta[j]) < 1.e-6)
	bp[j] = 0;
    }

    if (b_m_bp != NULL)
      for (j=i*N_RBD_PARMS+1; j<=(i+1)*N_RBD_PARMS; ++j)
	b_m_bp[j] = beta[j]-bp[j];


  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_parm_file
\date  May 2000
\remarks 

reads the filter offset, viscous friction flag, coulomb friction flag, and
spring flag per joint from file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int
read_parm_file(void) {

  int j,i,n,rc;
  FILE  *in;
  double dum;
  char   string[100];

  // get the file name
  if (!get_string("Name of xpest parameters",parm_file_name,parm_file_name))
    return FALSE;

  // read the link parameters
  // first try to read from config directory
  sprintf(string,"%s%s",CONFIG,parm_file_name);
  in = fopen(string,"r");
  if (in == NULL) { 
    in = fopen(parm_file_name,"r"); // try local directory
    if (in == NULL) {
      printf("ERROR: Cannot open file >%s<!\n",parm_file_name);
      return FALSE;
    }
  }

  // find all joint variables and read the relevant parameters
  for (i=1; i<= N_DOFS-N_DOFS_EST_SKIP; ++i) {
    if (!find_keyword(in, &(joint_names[i][0]))) {
      printf("ERROR: Cannot find xpest parameters for >%s<!\n",
	       joint_names[i]);
      fclose(in);
      return FALSE;
    } else {
      rc=fscanf(in,"%d %d %d %d",
		&(filt_data_dofs[i]),
		&(vis_flag_dofs[i]),
		&(coul_flag_dofs[i]),
		&(spring_flag_dofs[i]));
      if (rc != 4) {
	printf("Error: xpest parameter for joint >%s< have %d parameters, but should have %d\n",
	       joint_names[i],rc,4);
      }
    }
  }

  //see if there is a weighting matrix out there
  //it is used to make a weighted least square for param est
  if(!find_keyword(in, "least_square_weight"))  {

      printf("Cannot read any least square weight, using 1s\n");

  }  else  {
    
    for(i = 1; i<=N_DOFS+2*N_CART; i++)	{
      float tmp;
      rc = fscanf(in, "%f", &tmp);
      if(rc != 1) {
	printf("error there should be %d weights but found only %d\n", N_DOFS+2*N_CART, i);
	return FALSE;
      } else {
	least_square_weight[i] = 1.0/(double)tmp;
      }
    }
    print_vec("least_square_weight", least_square_weight);
  }
  
  fclose(in);
  
  return TRUE;

}

