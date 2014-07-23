/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_proc.c

  \author  Stefan Schaal
  \date    June 1999

  ==============================================================================
  \remarks

  filtering and post processing of vision blobs

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_vision_servo.h"
#include "lwpr.h"
#include "nrutil.h"
#include "SL_filters.h"

/* global variables */
int stereo_mode=VISION_3D_MODE;

/* local variables */


/* variables for stereo */
#define B_M  4             /*!< size of the design matrix */
#define B_N  3
#define P_M  3             /*!< size of the projection matrix */
#define P_N  4
#define I_M  7             /*!< number of the internal camera parameters */
static Matrix B;           /* design matrix for stereo calculation */
static Vector y;           /* value vector for stereo calculation */
static Matrix V;           /* V matrix out of svdcmp */
static Vector w;           /* w vector out of svdcmp */
static Vector x;           /* coefficients; result of Least Squares */ 
static Matrix Pl;          /* Projection Matrix for the left eye */
static Matrix Pr;          /* Projection Matrix for the right eye */
static Vector intl;        /* internal parameters for the left eye */
static Vector intr;        /* internal parameters for the right eye */


/*! variables for filtering */
#define KALMAN       1
#define BUTTER       2
#define MAX_PENDING  6
#define NOT_USED     -999

typedef struct BlobPP {
  int       pending;                   /*!< pending status */
  int       filter_type;               /*!< kalman = 1, butter = 2 */
  double    kal[N_CART+1][N_CART+1];   /*!< kalman filter matrix */
  double    acc[N_CART+1];             /*!< accelerations of blob (if known) */
  Filter    fblob[N_CART+1][N_CART+1]; /*!< filter information */
  SL_Cstate predicted_state;           /*!< the predicted state for the next time */
} BlobPP;

static BlobPP *blobpp;
static double predict = 0.033;

/* global functions */
int init_vision_processing(void);
void set_stereo_mode(void);
void q2N (double *u);

/* local functions */
static void   predict_state(double *pos, double *vel, double *acc, double dt);
static int    init_matrices(void);
static int    init_projection_matrices (void);
static void   stereo (double *ul, double *ur, double *);
static int    init_stereo_matrices (char *fname);
static void   dist2undist (double *pos, double *icp);

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_processing
\date  June 1999
   
\remarks 

          initialization for vision processing

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_vision_processing(void)

{
  
  int i,j;
  static int firsttime = TRUE;

  stereo_mode = VISION_3D_MODE;
  
  if (firsttime) {
    firsttime = FALSE;
    blobpp = (BlobPP *) my_calloc(max_blobs+1,sizeof(BlobPP),MY_STOP);
  }

#if 1
  /* initialize matrices */
  if (!init_matrices())
      return FALSE;
#endif

  /* initialize filtering */
  if (!init_filters())
    return FALSE;
  
  /* initialize post processing */
  if (!init_pp((char *)vision_default_pp))
    return FALSE;
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_matrices
\date  Jun 2000
   
\remarks 

        initializes things for matrix computation
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int
init_matrices(void)
{
    
    B = my_matrix(1,B_M,1,B_N);
    V = my_matrix(1,B_N,1,B_N);
    y = my_vector(1,B_M);
    w = my_vector(1,B_N);
    x = my_vector(1,B_N);

    Pl = my_matrix(1,P_M,1,P_N);
    Pr = my_matrix(1,P_M,1,P_N);

    intl = my_vector(1,I_M);
    intr = my_vector(1,I_M);

    if (!init_stereo_matrices (config_files[STEREOPARAMETERS]))
	return FALSE;

    return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  init_stereo_matrices
\date  Jun 2000
   
\remarks 

        initialize stereo matrices (Pr, Pl, intl, intr) by reading a data file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : configuration file name

 ******************************************************************************/
static int
init_stereo_matrices(char *fname)
{
  int  i, j,rc;
    FILE *fp;
    char string[100];

    /* read in the projection matrices file */
    sprintf (string, "%s%s", CONFIG,fname);
    fp = fopen (string, "r");

    if (fp == NULL) {
	printf("Cannot find the projection matrices file >%s<\n",string);
	beep(1); 
	return FALSE;
    }

    printf("Reading binocular camera calibration ...");

    for (i=1; i<=P_M; i++) {
	for (j=1; j<=P_N; j++) {
	    rc=fscanf (fp, "%lf", & Pl[i][j]);
	}
    }
    for (i=1; i<=P_M; i++) {
	for (j=1; j<=P_N; j++) {
	    rc=fscanf (fp, "%lf", & Pr[i][j]);
	}
    }
    for (i=1; i<=I_M; i++) {
	rc=fscanf (fp, "%lf", & intl[i]);
    }
    for (i=1; i<=I_M; i++) { 
	rc=fscanf (fp, "%lf", & intr[i]);
    }
    fclose (fp);

    printf("done\n");

    return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  process_blobs
\date  Feb 1999
   
\remarks 

    filtering and differentiation of the sensor readings

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     raw   :  raw blobs as input
 \param[out]    blobs :  the processed blob information

 ******************************************************************************/
void
process_blobs(Blob2D raw[][2+1])

{

  int i,j,n;
  double pos,vel,acc;
  int rfID;
  static Blob3D *traw;
  static int firsttime = TRUE;
  double x[2+2+1], x1[2+1], x2[2+1];
  
  if (firsttime) {
    firsttime = FALSE;
    traw = (Blob3D *) my_calloc(max_blobs+1,sizeof(Blob3D),MY_STOP);
  }
  
  for (i=1; i<=max_blobs; ++i) {

    if (!raw_blob_overwrite_flag) {

      /* the status of a blob is the AND of the two components */
      traw[i].status = raw[i][1].status && raw[i][2].status;

      switch (stereo_mode) {
	case VISION_3D_MODE:
	  /* coordinate transformation */
	  if (traw[i].status) {
	      x[1] = raw[i][1].x[1];
	      x[2] = raw[i][1].x[2];
	      x[3] = raw[i][2].x[1];
	      x[4] = raw[i][2].x[2];
	      rfID = 0;
	      if (predictLWPROutput(BLOB2ROBOT, 
				    x, x, 0.0, TRUE, traw[i].x, &rfID) == 0) {
		  traw[i].status = FALSE;
	      }
	  }
	break;
	  
	case VISION_2D_MODE:
	  if (traw[i].status) {
	      x1[1] =  raw[i][1].x[1];
	      x1[2] =  raw[i][1].x[2];
	      x2[1] =  raw[i][2].x[1];
	      x2[2] =  raw[i][2].x[2];
#if 1
	      /* stereo calculation based on the calibration */
	      stereo (x1, x2, traw[i].x);
#endif
	  }
	break;
      }
	  
    } else {
      /* use the learning model for the coordinate transformation 
	 (but not in simulation) */
      traw[i] = raw_blobs[i];
      /* CGA: should be updated to use 2D blobs, not 3D blobs. */
    }

    /* post processing */

    /* if the status changed from FALSE to TRUE, it is important
       to re-initialize the filtering */
    if (traw[i].status && !blobs[i].status) {
      for (j= _X_; j<= _Z_; ++j) {
	for (n=0; n<=FILTER_ORDER; ++n) {
	  blobpp[i].fblob[_X_][j].filt[n] = traw[i].x[j];
	  blobpp[i].fblob[_Y_][j].filt[n] = 0;
	  blobpp[i].fblob[_Z_][j].filt[n] = 0;
	  blobpp[i].fblob[_X_][j].raw[n]  = traw[i].x[j];
	  blobpp[i].fblob[_Y_][j].raw[n]  = 0;
	  blobpp[i].fblob[_Z_][j].raw[n]  = 0;
	}
      }
    }

    if (blobpp[i].filter_type == KALMAN) {
      ;
    } else { /* default is butterworth filtering */

      for (j= _X_; j<= _Z_; ++j) {

	if (traw[i].status) {

	  blobpp[i].pending = FALSE;
	  blobs[i].status   = TRUE;

	  /* filter and differentiate */
	  pos = filt(traw[i].x[j],&(blobpp[i].fblob[_X_][j]));
	  vel = (pos - blobpp[i].fblob[_X_][j].filt[2])*(double) vision_servo_rate;
	  vel = filt(vel,&(blobpp[i].fblob[_Y_][j]));
	  acc = (vel - blobpp[i].fblob[_Y_][j].filt[2])*(double) vision_servo_rate;
	  acc = filt(acc,&(blobpp[i].fblob[_Z_][j]));
	  if (blobpp[i].acc[j] != NOT_USED) {
	    acc = blobpp[i].acc[j];
	  }

	} else {

	  if (++blobpp[i].pending <= MAX_PENDING) {
	    pos = blobpp[i].predicted_state.x[j];
	    vel = blobpp[i].predicted_state.xd[j];
	    acc = blobpp[i].predicted_state.xdd[j];
	    // Note: leave blobs[i].status as is, as we don't want 
	    // to set something true which was not true before
	  } else {
	    pos = blobs[i].blob.x[j];
	    vel = 0.0;
	    acc = 0.0;
	    blobs[i].status   = FALSE;
	  }

	  /* feed the filters with the "fake" data to avoid transients
	     when the target status becomes normal */
	  pos = filt(pos,&(blobpp[i].fblob[_X_][j]));
	  vel = filt(vel,&(blobpp[i].fblob[_Y_][j]));
	  acc = filt(acc,&(blobpp[i].fblob[_Z_][j]));

	}

	/* predict ahead */
	blobs[i].blob.x[j]   = pos;
	blobs[i].blob.xd[j]  = vel;
	blobs[i].blob.xdd[j] = acc;

	if (blobs[i].status) {
	  
	  predict_state(&(blobs[i].blob.x[j]),&(blobs[i].blob.xd[j]),
			&(blobs[i].blob.xdd[j]),predict);
	  
	  
	  /* predict the next state */
	  blobpp[i].predicted_state.x[j]   = pos;
	  blobpp[i].predicted_state.xd[j]  = vel;
	  blobpp[i].predicted_state.xdd[j] = acc;
	  
	  predict_state(&(blobpp[i].predicted_state.x[j]),
			&(blobpp[i].predicted_state.xd[j]),
			&(blobpp[i].predicted_state.xdd[j]),
			1./(double)vision_servo_rate);
	}


      }
    }
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  predict_state
\date  June 1999
   
\remarks 

    predicts ahead according to the specified time, assuming a linear
    model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] pos    : position
 \param[in,out] vel    : velocity
 \param[in]     acc  : acceleration
 \param[in]     dt   : predict ahead time

 ******************************************************************************/
static void
predict_state(double *pos, double *vel, double *acc, double dt)

{

  int i,j;

  *pos +=  *vel * dt + 0.5 * sqr(dt) * *acc;
  *vel +=  *acc * dt;

}

/*!*****************************************************************************
 *******************************************************************************
\note  init_pp
\date  June 1999
   
\remarks 

    initialize the post processing

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name : name of the post processing file

 ******************************************************************************/
int
init_pp(char *name)

{

  int i,j,n,rc;
  char   string[100];
  FILE  *in;


  /* get the post processing information */

  sprintf(string,"%s%s",PREFS,name);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  /* find all blobs and read their information */

  sprintf(string,"PREDICT");
  if (find_keyword(in, string)) {
    rc=fscanf(in,"%lf",&predict);
  } else {
    predict = 0.0;
  }

  for (i=1; i<= max_blobs; ++i) {

    sprintf(string,"%s_KALMAN",blob_names[i]);
    if (find_keyword(in, string)) {
      blobpp[i].filter_type = KALMAN;
      for (j= _X_; j<= _Z_; ++j)
	for (n= _X_; n<= _Z_; ++n)
	  rc=fscanf(in,"%lf",&(blobpp[i].kal[j][n]));
    } else {
      for (j= _X_; j<= _Z_; ++j)
	for (n= _X_; n<= _Z_; ++n)
	  blobpp[i].kal[j][n]=1.0;
    }

    sprintf(string,"%s_BUTTER",blob_names[i]);
    if (find_keyword(in, string)) {
      blobpp[i].filter_type = BUTTER;
      for (j= _X_; j<= _Z_; ++j)
	for (n= _X_; n<= _Z_; ++n)
	  rc=fscanf(in,"%d",&(blobpp[i].fblob[j][n].cutoff));
    } else {
      for (j= _X_; j<= _Z_; ++j)
	for (n= _X_; n<= _Z_; ++n)
	  blobpp[i].fblob[j][n].cutoff=100;
    }

    sprintf(string,"%s_ACCELERATION",blob_names[i]);
    if (find_keyword(in, string)) {
      for (j= _X_; j<= _Z_; ++j)
	rc=fscanf(in,"%lf",&(blobpp[i].acc[j]));
    } else {
      for (j= _X_; j<= _Z_; ++j)
	blobpp[i].acc[j]=NOT_USED;
    }

  }

  /* do any blobs coincide with the endeffector? */
  sprintf(string,"ENDEFFECTOR");
  if (find_keyword(in, string)) {
    for (i=1; i<=max_blobs; ++i) {
      rc=fscanf(in,"%d",&blob_is_endeffector[i]);
      if (blob_is_endeffector[i] > n_endeffs || blob_is_endeffector[i] < 0)
	blob_is_endeffector[i] = FALSE;
    }
  } else {
    for (i=1; i<=max_blobs; ++i)
      blob_is_endeffector[i]=FALSE;
  }

  learn_transformation_flag = FALSE;
  for (i=1; i<=max_blobs; ++i) {
    if (blob_is_endeffector[i])
      learn_transformation_flag = TRUE;
  }

  
  fclose(in);
  remove_temp_file();

  /* keep track of which post processing script we are using */
  strcpy(current_pp_name,name);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_states
\date  August 7, 1995
   
\remarks 

initializes all states of the filters to safe initial values

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
init_vision_states(void)

{
  int i,j;
  extern int count_all_frames;
  extern int count_lost_frames;

  count_all_frames = 0;
  count_lost_frames = 0;
  
  /* initialize the vision variables */
  bzero((char *) blobpp,(max_blobs+1)*sizeof(BlobPP));
  bzero((char *) blobs, (max_blobs+1)*sizeof(SL_VisionBlob));
  bzero((char *) raw_blobs, (max_blobs+1)*sizeof(Blob3D));
  bzero((char *) raw_blobs2D, (2+1)*(max_blobs+1)*sizeof(Blob2D));
  init_pp((char *)vision_default_pp);

}

/*!*****************************************************************************
 *******************************************************************************
\note  set_stereo_mode
\date  Jun 2000
\remarks 

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void 
set_stereo_mode (void)
{
  if (servo_enabled) {
    printf("You cannot change stereo-mode while vision_servo is running\n");
    return;
  }
  
  get_int ("Stereo Mode\n\t 0: VISION_3D_MODE, 1:VISION_2D_MODE", stereo_mode,
	   &stereo_mode);
}


/*!*****************************************************************************
 *******************************************************************************
\note  stereo
\date  June 2000
   
\remarks 

    calculate 3D position of a blob from 2-2D information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ul   : [x y] ; positions of a blob in the left eye
 \param[in]     ur   : [x y] ; positions of a blob in the right eye
 \param[out]    y   : 3D position of the blob

 ******************************************************************************/
static void
stereo (double *ul, double *ur, double *r)
{
    double wmax, wmin;
    int    i, j;
    double y[4+1];

    q2N (ul);
    q2N (ur);

    /* distorted coordinate to the undistorted coordinate */
    dist2undist (ul, intl);
    dist2undist (ur, intr);

    /* SVD of the design matrix */
    B[1][1] = ul[1]*Pl[3][1]-Pl[1][1];
    B[1][2] = ul[1]*Pl[3][2]-Pl[1][2];
    B[1][3] = ul[1]*Pl[3][3]-Pl[1][3];
    B[2][1] = ul[2]*Pl[3][1]-Pl[2][1];
    B[2][2] = ul[2]*Pl[3][2]-Pl[2][2];
    B[2][3] = ul[2]*Pl[3][3]-Pl[2][3];
    B[3][1] = ur[1]*Pr[3][1]-Pr[1][1];
    B[3][2] = ur[1]*Pr[3][2]-Pr[1][2];
    B[3][3] = ur[1]*Pr[3][3]-Pr[1][3]; 
    B[4][1] = ur[2]*Pr[3][1]-Pr[2][1];
    B[4][2] = ur[2]*Pr[3][2]-Pr[2][2];
    B[4][3] = ur[2]*Pr[3][3]-Pr[2][3];

    my_svdcmp (B, B_M,B_N, w, V);

    /* preprocessing for svbksb */
    wmax = 0.0;
    for (j=1; j<=B_N; j++) if (w[j] > wmax) wmax = w[j];
    wmin = wmax*1.0e-6;
    for (j=1; j<=B_N; j++) if (w[j] < wmin) w[j] = 0.0;

    /* backsubstitution to get x; 3D position */
    y[1] = Pl[1][4]-ul[1]*Pl[3][4];
    y[2] = Pl[2][4]-ul[2]*Pl[3][4];
    y[3] = Pr[1][4]-ur[1]*Pr[3][4];
    y[4] = Pr[2][4]-ur[2]*Pr[3][4];

    my_svbksb (B,w,V, B_M,B_N, y,x);

    /* set the answer; [X Y Z]' */
    for (i=1; i<=3; i++) r[i] = x[i];
}

/*!*****************************************************************************
 *******************************************************************************
\note  q2N
\date  June 2000
   
\remarks 

     transfer QuickMag coordinate to NTSC coordinate

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] u     : [x y] ; positions of a blob in the QuickMAG coordinate

 ******************************************************************************/
void
q2N (double *u)
{
    u[1] = (u[1]+320)*0.8571+43;
    u[2] = (-u[2]+207)+30;
}

/*!*****************************************************************************
 *******************************************************************************
\note  dist2undist
\date  June 2000
   
\remarks 

       transfer coordinate of a pixel from distorted to undistorted

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] pos     : [x y] ; positions of a blob
 \param[in]     icp   : internal camera parameters of the camera for the blob;
                  [a, c, b, u0, v0, k1, k2]

 ******************************************************************************/
static void 
dist2undist (double *pos, double *icp)
{
    int      i1, i2, n;
    double   value[2], old[2];
    double  new_pixels[12+1];
    double  *param;
    double  *pixels;
    double   x, y, R, dist_u, dist_v, dist;
    double   Jac[2][2], Jac1[2][2];
    const double epsilon = 1.0e-8;
    
    n = 1;                /* not to change Ales's code */
    pixels = pos;       /* not to change Ales's code */
    param = icp;        /* not to change Ales's code */
/*    new_pixels = pos;  / to suppress a warning during compiling */

    n *= 2;
    for (i1 = 1; i1 <= n; i1 += 2) {
      i2 = i1+1;
      new_pixels[i1] = pixels[i1]; new_pixels[i2] = pixels[i2];
      do {
	y = (new_pixels[i2] - param[1+4]) / param[1+2];
	x = (new_pixels[i1] - param[1+3] - param[1+1]*y) / param[1+0];
	R = x*x + y*y;
	dist = param[1+5]*R + param[1+6]*R*R;
	value[0] = new_pixels[i1] + (new_pixels[i1] - param[1+3]) * dist -
	  pixels[i1];
	value[1] = new_pixels[i2] + (new_pixels[i2] - param[1+4]) * dist -
	  pixels[i2];
	
	dist_u = (2*x/param[1+0]) * (param[1+5] + 2*param[1+6]*R);
	dist_v = (-2*x*param[1+1]/param[1+0]/param[1+2] + 2*y/param[1+2]) *
	  (param[1+5] + 2*param[1+6]*R);
	Jac[0][0] = 1 + dist + (new_pixels[i1] - param[1+3]) * dist_u;
	Jac[0][1] = (new_pixels[i1] - param[1+3]) * dist_v;
	Jac[1][0] = (new_pixels[i2] - param[1+4]) * dist_u;
	Jac[1][1] = 1 + dist + (new_pixels[i2] - param[1+4]) * dist_v;
	
	dist = Jac[0][0]*Jac[1][1] - Jac[0][1]*Jac[1][0];
	Jac1[0][0] = Jac[1][1]/dist; Jac1[0][1] = -Jac[0][1]/dist;
	Jac1[1][0] = -Jac[1][0]/dist; Jac1[1][1] = Jac[0][0]/dist;
	old[0] = new_pixels[i1]; old[1] = new_pixels[i2];
	new_pixels[i1] = new_pixels[i1] - Jac1[0][0]*value[0] -
	  Jac1[0][1]*value[1];
	new_pixels[i2] = new_pixels[i2] - Jac1[1][0]*value[0] -
	  Jac1[1][1]*value[1];
      }
      while ((new_pixels[i1] - old[0])*(new_pixels[i1] - old[0]) +
	     (new_pixels[i2] - old[1])*(new_pixels[i2] - old[1]) > epsilon);
    }
    
    pos = new_pixels;
}
 
/*!*****************************************************************************
 *******************************************************************************
\note  w2i
\date  Jul 2000, Tom
   
\remarks 

       transfer coordinate from the world to the images

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     pos   :  3D position of an object
 \param[out]    xy_l :  projected 2D position of the object for the left eye
 \param[out]    xy_r :  projected 2D position of the object for the right eye

 ******************************************************************************/
int
w2i (Vector pos, Vector xy_l, Vector xy_r)
{
  Vector tmp;

  tmp = my_vector (1, 3);

  mat_vec_mult (Pl, pos, tmp);
  if (tmp[3] > 1.0e-4) {
    xy_l[1] = tmp[1]/tmp[3];
    xy_l[2] = tmp[2]/tmp[3];
  } else {
    printf ("numerical problem during calculation of a optical projection\n");
    return FALSE;
  }

  mat_vec_mult (Pr, pos, tmp);
  if (tmp[3] > 1.0e-4) {
    xy_r[1] = tmp[1]/tmp[3];
    xy_r[2] = tmp[2]/tmp[3];
  } else {
    printf ("numerical problem during calculation of a optical projection\n");
    return FALSE;
  }

  my_free_vector (tmp, 1,3);

  return TRUE;
}





