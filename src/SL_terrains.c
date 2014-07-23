/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_terrains.c

  \author 
  \date   

  ==============================================================================
  \remarks

      A set of functions to read, store, and process information about
      terrains for walking, originally developed in the context of the
      LittleDog project, and then generalized to a generic feature in SL.
      The idea is to store complex terrains as a grid file in x,y,z, which
      can be queried very fast. Visualization can be done in a rudementary
      way without too much CPU costs

  ============================================================================*/

// system includes
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "strings.h"
#include "pthread.h"
#ifdef UNIX
#include "sys/stat.h"
#endif
#ifdef VX
#include "sys/stat.h"
#endif

// local includes
#include "SL.h"
#include "SL_common.h"
#include "SL_terrains.h"
#include "utility.h"


#define EMPTY_TERRAIN -9999.0
#define RADIUS_FZ     7

// variable declarations
// local variables
Terrain terrains[MAX_TERRAINS+1];
static double  ground_level_z = 0.0;

static pthread_t       cthread_terrain;                          // thread for caching terrain info
static pthread_mutex_t mutex_terrain= PTHREAD_MUTEX_INITIALIZER; // for a safe thread


// global variables

// bounding box on terrain
double terrain_bounding_box_max[N_CART+1];
double terrain_bounding_box_min[N_CART+1];

// local functions
static int
computeTerrainNormal(int ID, int ix, int iy, int nx, int ny, int down,
		     int cflag, int fflag, TerrainInfo *tinfo);
static int
getTerrainInfoSwitched(double x, double y, int z_only, int no_pad, 
		       TerrainInfo *tinfo);
static void *
cacheTerrainInfoThread(void *dptr);

static double
computeMedian(double *v, int n_v) ;

static int
convertObj2Asc(char *fname);

// extern variables


/*!*****************************************************************************
 *******************************************************************************
\note  readTerrainBoard
\date  April 2006
   
\remarks 

        Reads information from a terrain board

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname   : terrain file name
 \param[in]     ID      : terrain array index ( between 1 and MAX_TERRAIN)

 ******************************************************************************/
int
readTerrainBoard(char *fname, int ID)
{

  int     i,j,m,n;
  int     count,countl;
  double  x,y,z,no_go;
  double  max_x;
  double  min_x;
  double  max_y;
  double  min_y;
  Matrix  temp;
  Matrix  temp_z;
  Matrix  temp_no_go;
  Terrain    *t;
  TerrainInfo tinfo;
  int     nx,ny;

  // check for validity of terrain index
  if (ID < 1 || ID > MAX_TERRAINS) {
    printf("Terrain %d out of range of terrains from 1-%d\n",ID,MAX_TERRAINS);
    return FALSE;
  }

  // use a simpler variable for convenience
  t = &(terrains[ID]);

  // initialize position and orientation of board
  bzero((void *)t,sizeof(Terrain));
  t->ID = 0;
  t->orient.q[_Q0_] = 1.0;

  // read the terrain file
  read_terrain_file(fname,&temp,&nx,&ny,&min_x,&max_x,&min_y,&max_y,FALSE,&count);
  t->nx_local = nx;
  t->ny_local = ny;

  // determine the grid size
  t->dx = (max_x-min_x)/(double)(t->nx_local-1)/1000.;
  t->dy = (max_y-min_y)/(double)(t->ny_local-1)/1000.;

  printf("\nFound Terrain Board >%s< with %d x-values and %d y-values\n",
	 fname,t->nx_local,t->ny_local);
  printf("and %f[m] dx and %f[m] dy grid size\n",t->dx,t->dy);

  // the the position of the Z(1,1) state in local coordinates
  t->dxorg_local = min_x/1000.0;
  t->dyorg_local = min_y/1000.0;

  // allocate memory
  t->z_local     = my_fmatrix(1,t->nx_local,1,t->ny_local);
  t->no_go_local = my_fmatrix(1,t->nx_local,1,t->ny_local);
  temp_z     = my_matrix(1,t->nx_local,1,t->ny_local);
  temp_no_go = my_matrix(1,t->nx_local,1,t->ny_local);
  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j)
      t->z_local[i][j] = EMPTY_TERRAIN;

  // read the data set into the matrix
  countl = 0;
  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j) {
      if (++countl > count)
	continue;
      x     = temp[countl][1];
      y     = temp[countl][2];
      z     = temp[countl][3];
      no_go = temp[countl][4];
      m = rint((x/1000. - t->dxorg_local)/t->dx) + 1;
      n = rint((y/1000. - t->dyorg_local)/t->dy) + 1;
      t->z_local[m][n] = z/1000.0;
      t->no_go_local[m][n] = no_go;
    }

  // fix holes, but not hole in the fringe
  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j) {
      if (t->z_local[i][j] == EMPTY_TERRAIN) {
	int    sx,ex,sy,ey;
	double aux = 0;
	double no_go_aux = 0;
	int    count_aux = 0;
	int    nn = 3;

	// a hole is defined as an empty cell will less than 3 nearest neighbors,
	// and we only search over a distance of maximally 3 neighbors
	for (n=i; n<=i+nn && n<=t->nx_local; ++n) {
	  if (t->z_local[n][j] != EMPTY_TERRAIN) {
	    aux += t->z_local[n][j];
	    no_go_aux += t->no_go_local[n][j];
	    ++count_aux;
	    break;
	  }
	}

	for (n=i; n>=i-nn && n>=1; --n) {
	  if (t->z_local[n][j] != EMPTY_TERRAIN) {
	    aux += t->z_local[n][j];
	    no_go_aux += t->no_go_local[n][j];
	    ++count_aux;
	    break;
	  }
	}

	for (n=j; n<=j+nn && n<=t->ny_local; ++n) {
	  if (t->z_local[i][n] != EMPTY_TERRAIN) {
	    aux += t->z_local[i][n];
	    no_go_aux += t->no_go_local[i][n];
	    ++count_aux;
	    break;
	  }
	}

	for (n=j; n>=j-nn && n>=1; --n) {
	  if (t->z_local[i][n] != EMPTY_TERRAIN) {
	    aux += t->z_local[i][n];
	    no_go_aux += t->no_go_local[i][n];
	    ++count_aux;
	    break;
	  }
	}

	if (count_aux >= 3) {
	  temp_z[i][j] = aux/(double)count_aux;
	  temp_no_go[i][j] = no_go_aux/(double)count_aux;
	} else {
	  temp_z[i][j] = EMPTY_TERRAIN;
	  temp_no_go[i][j] = FALSE;
	}

      }
    }

  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j) 
      if (t->z_local[i][j] == EMPTY_TERRAIN) {
	t->z_local[i][j] = temp_z[i][j];
	t->no_go_local[i][j] = temp_no_go[i][j];
      }
	
  my_free_matrix(temp,1,count,1,4);
  my_free_matrix(temp_z,1,t->nx_local,1,t->ny_local);
  my_free_matrix(temp_no_go,1,t->nx_local,1,t->ny_local);

  // the status is only true after the position and orientation of the
  // board are filled in with setTerrainInfo()
  t->status = FALSE;


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  setTerrainInfo
\date  April 2006
   
\remarks 

        This functions allows to set the position, orientation, and 
        terrain ID of a terrain structure, and it reads the associated
        terrain boardfile.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID      : terrain array index ( between 1 and MAX_TERRAIN)
 \param[in]     tID     : identifier number of the terrain board
 \param[in]     tfname  : file name of the terrain *.asc file
 \param[in]     pos     : 3D position vector of terrain origin
 \param[in]     orient  : 4D unit quaternion vector of terrain origin.
 \param[in]     reg_rad : radius of points used in regression of normal
 \param[in]     reg_down: down sampling for normal regression
 \param[in]     reg_crad: radius of points used to comopute contact normal 
                  (specify 0 to avoid computation)

 ******************************************************************************/
int
setTerrainInfo(int ID, int tID, char *tfname, double *pos, double *orient,
	       int reg_rad, int reg_down, int reg_crad)
{

  int i,j,n,m,r;
  char string[100];
  Matrix X,Y,Z;
  Matrix R;
  Vector v,vv;
  Terrain *t;  
  Matrix  temp_z;
  Matrix  temp_no_go;
  double  max_pos[N_CART+1];
  double  min_pos[N_CART+1];
  double  aux;
  double  no_go_aux;
  int     count;
  double  normal[N_CART+1];
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    for (i=1; i<=N_CART; ++i) {
      terrain_bounding_box_max[i] = -1.e10;
      terrain_bounding_box_min[i] =  1.e10;
    }
  }

  // check for validity of terrain index
  if (ID < 1 || ID > MAX_TERRAINS) {
    printf("Terrain %d out of range of terrains from 1-%d\n",ID,MAX_TERRAINS);
    return FALSE;
  }

  if (!tID)
    return FALSE;

  t = &(terrains[ID]);

  if (t->status)
    return TRUE;

  // read the terrain board information
  sprintf(string,"%s%s",TERRAINS,tfname);
  if (!readTerrainBoard(string, ID)) {
    printf("Could not initialize terrain board #%d from %s\n",ID,string);
    return FALSE;
  }
  t->ID = tID;
  strcpy(t->tfname,tfname);

  // assign position values
  for (i=1; i<=N_CART; ++i)
    t->pos.x[i] = pos[i];

  for (i=1; i<=N_QUAT; ++i)
    t->orient.q[i] = orient[i];

  // store info for normal computation
  t->reg_rad  = reg_rad;
  t->reg_down = reg_down;
  t->reg_crad = reg_crad;
  t->reg_frad = RADIUS_FZ;

  // convert all terrain board locations into world coordinates and create
  // a new version of the terrain board in world coordinates.
  X  = my_matrix(1,t->nx_local,1,t->ny_local);
  Y  = my_matrix(1,t->nx_local,1,t->ny_local);
  Z  = my_matrix(1,t->nx_local,1,t->ny_local);
  R  = my_matrix(1,N_CART,1,N_CART);
  v  = my_vector(1,N_CART);
  vv = my_vector(1,N_CART);

  quatToRotMatInv(&(t->orient),R);      

  // get the terrain board normal in world coordinates
  v[_X_] = 0.0;
  v[_Y_] = 0.0;
  v[_Z_] = 1.0;
  mat_vec_mult(R,v,vv);
  
  for (n=1; n<=N_CART; ++n)
    normal[n] = vv[n];

  for (i=1; i<=N_CART; ++i) {
    max_pos[i] = -1.e10;
    min_pos[i] =  1.e10;
  }

  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j) {

      if (t->z_local[i][j] == EMPTY_TERRAIN) {
	double alpha;

	// This is usually a fringe area of the terrain with missing data. We
	// fill in the z from the ground at these points, which requires a bit
	// of work in order to find this z in local board coordinates

	// get the position of the current grid cell in world coordinates under 
	// the assumption of zero z value
	v[_X_] = t->dxorg_local + (i-1)*t->dx;
	v[_Y_] = t->dyorg_local + (j-1)*t->dy;
	v[_Z_] = 0.0;
	mat_vec_mult(R,v,vv);
	for (n=1; n<=N_CART; ++n)
	  vv[n] += t->pos.x[n];

	// the normal together with the vv point form the equation of a line, 
	// line = vv + alpha * normal (alpha is a scalar)
	// and I need the the point where this line intersects the 
	// z=ground_level_z plane
	alpha = (ground_level_z-vv[_Z_])/normal[_Z_];
	for (n=1; n<=N_CART; ++n)
	  v[n] = vv[n] + alpha * normal[n];

	// Thus, the local height in terrain board coordinates is the inner
	// product: (v-vv)'*normal
	t->z_local[i][j] = 0.0;
	for (n=1; n<=N_CART; ++n)
	  t->z_local[i][j] += (v[n]-vv[n])*normal[n];

      }

      // the current local coordinates of a point
      v[_X_] = t->dxorg_local + (i-1)*t->dx;
      v[_Y_] = t->dyorg_local + (j-1)*t->dy;
      v[_Z_] = t->z_local[i][j];

      // convert to global coordinates and keep track of max and mins
      mat_vec_mult(R,v,vv);
      for (n=1; n<=N_CART; ++n) {

	vv[n] += t->pos.x[n];

	if (vv[n] > max_pos[n])
	  max_pos[n] = vv[n];

	if (vv[n] < min_pos[n])
	  min_pos[n] = vv[n];

      }

      X[i][j] = vv[_X_];
      Y[i][j] = vv[_Y_];
      Z[i][j] = vv[_Z_];

    }

  // deterimine the size of global terrain board
  t->xorg  = min_pos[_X_];
  t->yorg  = min_pos[_Y_];
  t->nx    = ceil((max_pos[_X_]-min_pos[_X_])/t->dx);
  t->ny    = ceil((max_pos[_Y_]-min_pos[_Y_])/t->dy);
  t->max_z = max_pos[_Z_];
  t->min_z = min_pos[_Z_];

  // add the padding offsets
  t->nx += 2*reg_rad;
  t->ny += 2*reg_rad;

  t->padx = reg_rad;
  t->pady = reg_rad;
  
  t->xorg -= t->dx*reg_rad;
  t->yorg -= t->dy*reg_rad;

  // allocate memory for the new terrain board in global coordinates and its
  // cached statistics structures
  t->z      = my_fmatrix(1,t->nx,1,t->ny);
  temp_z    = my_matrix(1,t->nx,1,t->ny);
  t->n_nMSE = my_fmatrix(1,t->nx,1,t->ny);
  t->n_x    = my_fmatrix(1,t->nx,1,t->ny);
  t->n_y    = my_fmatrix(1,t->nx,1,t->ny);
  t->n_z    = my_fmatrix(1,t->nx,1,t->ny);
  t->cached = my_imatrix(1,t->nx,1,t->ny);
  t->pz     = my_fmatrix(1,t->nx,1,t->ny);
  t->fz     = my_fmatrix(1,t->nx,1,t->ny);
  t->no_go  = my_fmatrix(1,t->nx,1,t->ny);
  temp_no_go= my_matrix(1,t->nx,1,t->ny);


  // contact information is kept relative to the local board representation and we also
  // pad the marigins
  t->c_nx     = t->nx_local;
  t->c_ny     = t->ny_local;
  t->c_dxorg  = t->dxorg_local;
  t->c_dyorg  = t->dyorg_local;

  t->c_nx += 2*reg_crad;
  t->c_ny += 2*reg_crad;

  t->c_padx = reg_crad;
  t->c_pady = reg_crad;
  
  t->c_dxorg -= t->dx*reg_crad;
  t->c_dyorg -= t->dy*reg_crad;

  t->c_z     = my_fmatrix(1,t->c_nx,1,t->c_ny);
  t->c_no_go = my_fmatrix(1,t->c_nx,1,t->c_ny);
  t->cn_x    = my_fmatrix(1,t->c_nx,1,t->c_ny);
  t->cn_y    = my_fmatrix(1,t->c_nx,1,t->c_ny);
  t->cn_z    = my_fmatrix(1,t->c_nx,1,t->c_ny);
  t->ccached = my_imatrix(1,t->c_nx,1,t->c_ny);

  for (i=1; i<=t->c_nx; ++i)
    for (j=1; j<=t->c_ny; ++j) {

      if (i > t->c_padx && i <= t->c_nx-t->c_padx && // no-padding area
	  j > t->c_pady && j <= t->c_ny-t->c_pady) {

	t->c_z[i][j]     = t->z_local[i-t->c_padx][j-t->c_pady];
	t->c_no_go[i][j] = t->no_go_local[i-t->c_padx][j-t->c_pady];

      } else { // inside the padding area we get the ground_level_z a z
	double alpha;

	// get the position of the current grid cell in world coordinates under 
	// the assumption of zero z value
	v[_X_] = t->c_dxorg + (i-1)*t->dx;
	v[_Y_] = t->c_dyorg + (j-1)*t->dy;
	v[_Z_] = 0.0;
	mat_vec_mult(R,v,vv);
	for (n=1; n<=N_CART; ++n)
	  vv[n] += t->pos.x[n];

	// the normal together with the vv point from the equation of a line, 
	// line = vv + alpha * normal (alpha is a scalar)
	// and I need the the point where this line intersects the z=0 plane
	alpha = (ground_level_z-vv[_Z_])/normal[_Z_];
	for (n=1; n<=N_CART; ++n)
	  v[n] = vv[n] + alpha * normal[n];

	// Thus, the local height in terrain board coordinates is the inner
	// product: (v-vv)'*normal
	t->c_z[i][j] = 0.0;
	for (n=1; n<=N_CART; ++n)
	  t->c_z[i][j] += (v[n]-vv[n])*normal[n];

      }

    }

  // initialize t->z such that we can detect uninitialized z values
  for (i=1; i<=t->nx; ++i)
    for (j=1; j<=t->ny; ++j)
      t->z[i][j] = EMPTY_TERRAIN;
  
  // sort the X,Y,Z values into this global terrain board
  for (i=1; i<=t->nx_local; ++i)
    for (j=1; j<=t->ny_local; ++j) {
      m = rint((X[i][j] - t->xorg)/t->dx) + 1;
      n = rint((Y[i][j] - t->yorg)/t->dy) + 1;
      if (t->z[m][n] < Z[i][j])  // multiple points may fall in one cell -- take largest
	t->z[m][n] = Z[i][j];
      if (t->no_go[m][n] < t->no_go_local[i][j])
	t->no_go[m][n] = t->no_go_local[i][j];
    }

  // interpolate missing data
  for (i=1+t->padx; i<=t->nx-t->padx; ++i)
    for (j=1+t->pady; j<=t->ny-t->pady; ++j)
      if (t->z[i][j] == EMPTY_TERRAIN) {
	// interpolate from neigbors by gradually increasing the scope of the neighborhood,
	// but not more than 3 neighbors
	count       = 0;
	aux         = 0;
	no_go_aux   = 0;

	for (r=1; r<=t->nx-t->padx && r<=t->ny-t->pady && r <= 3; ++r) {

	  m = i+r;
	  if (m <= t->nx-t->padx)
	    if (t->z[m][j] != EMPTY_TERRAIN) {
	      aux += t->z[m][j];
	      no_go_aux += t->no_go[m][j];
	      ++count;
	    }

	  m = i-r;
	  if (m >= 1+t->padx)
	    if (t->z[m][j] != EMPTY_TERRAIN) {
	      aux += t->z[m][j];
	      no_go_aux += t->no_go[m][j];
	      ++count;
	    }
	  n = j+r;
	  if (n <= t->ny-t->pady)
	    if (t->z[i][n] != EMPTY_TERRAIN) {
	      aux += t->z[i][n];
	      no_go_aux += t->no_go[i][n];
	      ++count;
	    }
	  n = j-r;
	  if (n >= 1+t->pady)
	    if (t->z[i][n] != EMPTY_TERRAIN) {
	      aux += t->z[i][n];
	      no_go_aux += t->no_go[i][n];
	      ++count;
	    }
	  
	  if (count > 0)
	    break;

	}

	if (count == 0) {
	  aux = ground_level_z;
	  no_go_aux = 0;
	  count = 1;
	}

	temp_z[i][j] = aux/(double)count; // use a temp array to no propagate interpolation
	temp_no_go[i][j] = no_go_aux/(double)count;

      }

  // now update the real t->z array
  for (i=1+t->padx; i<=t->nx-t->padx; ++i)
    for (j=1+t->pady; j<=t->ny-t->pady; ++j)
      if (t->z[i][j] == EMPTY_TERRAIN) {
	t->z[i][j] = temp_z[i][j];
	t->no_go[i][j] = temp_no_go[i][j];
      }

  // set status of terrain to TRUE
  t->status = TRUE;

  // update the bounding box
  if (t->xorg < terrain_bounding_box_min[_X_])
    terrain_bounding_box_min[_X_] = t->xorg;

  if (t->xorg+(t->nx-1)*t->dx > terrain_bounding_box_max[_X_])
    terrain_bounding_box_max[_X_] = t->xorg+(t->nx-1)*t->dx;

  if (t->yorg < terrain_bounding_box_min[_Y_])
    terrain_bounding_box_min[_Y_] = t->yorg;

  if (t->yorg+(t->ny-1)*t->dy > terrain_bounding_box_max[_Y_])
    terrain_bounding_box_max[_Y_] = t->yorg+(t->ny-1)*t->dy;

  if (t->min_z < terrain_bounding_box_min[_Z_])
    terrain_bounding_box_min[_Z_] = t->min_z;

  if (t->max_z > terrain_bounding_box_max[_Z_])
    terrain_bounding_box_max[_Z_] = t->max_z;


  // free memory
  my_free_matrix(X,1,t->nx_local,1,t->ny_local);
  my_free_matrix(Y,1,t->nx_local,1,t->ny_local);
  my_free_matrix(Z,1,t->nx_local,1,t->ny_local);
  my_free_matrix(R,1,N_CART,1,N_CART);
  my_free_vector(v,1,N_CART);
  my_free_vector(vv,1,N_CART);
  my_free_matrix(temp_z,1,t->nx,1,t->ny);
  my_free_matrix(temp_no_go,1,t->nx,1,t->ny);


  printf("Terrain Board %d with ID=%d is at: x=% 5.3f  y=% 5.3f  z=% 5.3f\n",
	 ID,tID,pos[_X_],pos[_Y_],pos[_Z_]);
  printf("     with reg_rad=%d reg_down=%d reg_crad=%d\n\n",t->reg_rad,
	 t->reg_down,t->reg_crad);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainInfo
\date  April 2006
   
\remarks 

        Returns z position, slope, and go/no-go info at a given x,y
        position

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point
 \param[in]     y        : y position of query point
 \param[out]    tinfo    : terrain info structure
   
     Returns TRUE if data found in terrains, or FALSE if not. Irrespectively,
     z, s, and go are returned with either computed or default values.

 ******************************************************************************/
int
getTerrainInfo(double x, double y, TerrainInfo *tinfo)
{
  return getTerrainInfoSwitched(x,y,FALSE,FALSE,tinfo);
}

/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainInfoZOnly
\date  Feb 2009
   
\remarks 

        Returns z position at a given x,y position

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point
 \param[in]     y        : y position of query point
 \param[out]    tinfo    : terrain info structure
   
     Returns TRUE if data found in terrains, or FALSE if not. Irrespectively,
     z, s, and go are returned with either computed or default values.

 ******************************************************************************/
int
getTerrainInfoZOnly(double x, double y, TerrainInfo *tinfo)
{
  return 
    getTerrainInfoSwitched(x,y,TRUE,FALSE,tinfo);
}

/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainInfoSwitched
\date  April 2006
   
\remarks 

        Returns z position, slope, and go/no-go info at a given x,y
        position

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point
 \param[in]     y        : y position of query point
 \param[in]     z_only   : don't compute terrain info, just return z 
 \param[in]     no_pad   : don't return info if the state belongs to terrain padding
 \param[out]    tinfo    : terrain info structure
   
     Returns TRUE if data found in terrains, or FALSE if not. Irrespectively,
     z, s, and go are returned with either computed or default values.

 ******************************************************************************/
static int
getTerrainInfoSwitched(double x, double y, int z_only, int no_pad, TerrainInfo *tinfo)
{
  int           i,j,m,n,r;
  Terrain      *t;
  double        norm[N_CART+1];
  double        nnorm[N_CART+1];
  double        alpha;
  double        aux;
  int           found_flag = FALSE;
  int           in_padding_flag = TRUE; // indicates whether the max z point was in padding

  // default assignments
  tinfo->z          = EMPTY_TERRAIN;
  tinfo->pz         = EMPTY_TERRAIN;
  tinfo->fz         = EMPTY_TERRAIN;
  tinfo->n[_X_]     = 0;
  tinfo->n[_Y_]     = 0;
  tinfo->n[_Z_]     = 1.0;
  tinfo->n_nMSE     = 0.0;
  tinfo->slope      = 0.0;
  tinfo->no_go      = FALSE;
  tinfo->ID         = 0;
  tinfo->in_padding = FALSE;

  // loop over all terrain boards and try to find whether query point is
  // covered by the given board

  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    // check whether index falls into terrain board
    m = rint((x - t->xorg)/t->dx) + 1;
    n = rint((y - t->yorg)/t->dy) + 1;

    
    if (m >= 1 && m <= t->nx && n >=1 && n <=t->ny) { // in valid terrain region
      
      if (no_pad && (m <= t->padx || m > t->nx-t->padx || n <= t->pady || n > t->ny-t->pady))
	continue;
      
      // note that we found a terrain board for this data point
      found_flag = TRUE;
      
      // data points found on proper terrain boards superseeds data points found in padding, as
      // padding only serves to smoothly connect terrain boards. Note that the ground_level is
      // intialized as in_padding_flag=TRUE to simplify the logic below

      if (t->z[m][n] > tinfo->z) { 
	if (m <= t->padx || m > t->nx-t->padx || n <= t->pady || n > t->ny-t->pady) { // in padding
	  if (in_padding_flag) {
	    tinfo->z = t->z[m][n];
	    tinfo->no_go = t->no_go[m][n];
	    tinfo->in_padding = TRUE;
	  } else
	    continue; // data points found outside of padding have priority as padding is just a fudge
	} else { // not in padding
	  tinfo->z = t->z[m][n];
	  tinfo->no_go = t->no_go[m][n];
	  in_padding_flag = FALSE;
	  tinfo->in_padding = FALSE;
	}	    
      } else {
	if (in_padding_flag) {
	  if (m <= t->padx || m > t->nx-t->padx || n <= t->pady || n > t->ny-t->pady) { // in padding
	    continue;
	  } else {
	    tinfo->z = t->z[m][n];
	    tinfo->no_go = t->no_go[m][n];
	    in_padding_flag = FALSE;
	    tinfo->in_padding = FALSE;
	  }
	} else
	  continue;
      }

      // terrain info data
      if (!z_only) {

	if (t->cached[m][n]) {

	  tinfo->n_nMSE = t->n_nMSE[m][n];
	  tinfo->n[_X_] = t->n_x[m][n];
	  tinfo->n[_Y_] = t->n_y[m][n];
	  tinfo->n[_Z_] = t->n_z[m][n];
	  tinfo->pz     = t->pz[m][n];
	  tinfo->fz     = t->fz[m][n];

	} else {

	  pthread_mutex_lock( &mutex_terrain );

	  computeTerrainNormal(i, m, n, t->reg_rad, t->reg_rad, t->reg_down, 
			       FALSE, TRUE,tinfo);
	  t->fz[m][n]  = tinfo->fz;

	  computeTerrainNormal(i, m, n, t->reg_rad, t->reg_rad, t->reg_down, 
			       FALSE, FALSE,tinfo);
	  t->n_x[m][n] = tinfo->n[_X_];
	  t->n_y[m][n] = tinfo->n[_Y_];
	  t->n_z[m][n] = tinfo->n[_Z_];
	  t->pz[m][n]  = tinfo->pz;
	  
	  t->cached[m][n] = TRUE;
	  pthread_mutex_unlock( &mutex_terrain );

	}

	tinfo->slope  = fabs(acos(tinfo->n[_Z_]));

      }

      tinfo->ID = t->ID;

    } // end in valid terrain area

  }

  if (!found_flag) {
    tinfo->z      = ground_level_z;
    tinfo->pz     = ground_level_z;
    tinfo->fz     = ground_level_z;
  }


  return found_flag;
}

/*!*****************************************************************************
 *******************************************************************************
\note  getContactTerrainInfo
\date  April 2006
   
\remarks 

        Returns z position and normal of terrain for the purpose of 
        contact checking.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point (local coordinates)
 \param[in]     y        : y position of query point (local coordinates)
 \param[in]     tfname   : terrain file name needed to identify the terrain
 \param[out]    z        : z position of query point (local coordinates)
 \param[out]    norm     : terrain normal at this point
 \param[out]    no_go    : no_go value at this point

   
     Returns TRUE if data found in terrains, or FALSE if not. 

 ******************************************************************************/
int
getContactTerrainInfo(double x, double y, char *tfname, double *z, double *norm,
	double *no_go)
{
  int           i,j,m,n;
  Terrain      *t;
  TerrainInfo  tinfo;

  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    if (strcmp(t->tfname,tfname) != 0)
      continue;

    // determine index into terrain matrix
    m = rint((x - t->c_dxorg)/t->dx) + 1;
    n = rint((y - t->c_dyorg)/t->dy) + 1;

    if (m >= 1 && m <= t->c_nx && n >=1 && n <=t->c_ny) {

      *z = t->c_z[m][n];
      *no_go = t->c_no_go[m][n];

      // compute the normal
      if (t->ccached[m][n]) {
	norm[_X_] = t->cn_x[m][n];
	norm[_Y_] = t->cn_y[m][n];
	norm[_Z_] = t->cn_z[m][n];
      } else {
	pthread_mutex_lock( &mutex_terrain );
	computeTerrainNormal(i, m, n, t->reg_crad, t->reg_crad, 1, TRUE, FALSE,&tinfo);
	norm[_X_] = t->cn_x[m][n] = tinfo.n[_X_];
	norm[_Y_] = t->cn_y[m][n] = tinfo.n[_Y_];
	norm[_Z_] = t->cn_z[m][n] = tinfo.n[_Z_];
	t->ccached[m][n] = TRUE;
	pthread_mutex_unlock( &mutex_terrain );
      }
      return TRUE;
    }

  }

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  computeTerrainNormal
\date  April 2006
   
\remarks 

        Computes the terrain normal at a certain location in a terrain
        using a certain neigborhod of points

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID      : terrain array index ( between 1 and MAX_TERRAIN)
 \param[in]     ix      : the integer index of the x coordinate of terrain location
 \param[in]     iy      : the integer index of the y coordinate of terrain location
 \param[in]     nx      : the number of x neighbors to consider for regression
 \param[in]     ny      : the number of y neighbors to consider for regression
 \param[in]     down    : 1=no down sampling, 2=every other point in
                  x and y direction is chosen, etc.
 \param[in]     cflag   : compute for contact representation
 \param[in]     fflag   : compute for foothold representation
 \param[out]    tinfo   : the terrain info structure
     
 ******************************************************************************/
#define MAX_NEIGHBORS  50
#define USE_CACHE      TRUE
static int
computeTerrainNormal(int ID, int ix, int iy, int nx, int ny, int down,
		     int cflag, int fflag, TerrainInfo *tinfo)
{
  static   int firsttime = TRUE;
  static   int last_n_data = 0;
  static   Matrix **MatrixCache[MAX_NEIGHBORS+1][MAX_NEIGHBORS+1];
  static   Vector   y;
  Matrix   X;
  Matrix   XTX;
  Matrix   XTXinv;
  Matrix   XTXinvXT=NULL;
  int      count;
  int      i,j,m,n;
  Terrain *t;
  TerrainInfo taux;
  int      sx,sy,ex,ey;
  int      n_data;
  double   beta[3+1];
  double   sum_e2;
  double   aux;
  double   max_z;
  int      need_memory = TRUE;
  int      need_matrix = TRUE;
  int      ind1,ind2,ind3,ind4;
  int      tnx,tny;
  double   std;

  // initialize the cache
  if (firsttime) {
    firsttime = FALSE;
    for (i=0; i<=MAX_NEIGHBORS; ++i)
      for (j=0; j<=MAX_NEIGHBORS; ++j)
	MatrixCache[i][j] = NULL;
  }

  t = &(terrains[ID]);

  // default initialization
  tinfo->slope  = 0.0;
  tinfo->n_nMSE = 0.0;
  tinfo->n[_X_] = 0.0;
  tinfo->n[_Y_] = 0.0;
  tinfo->n[_Z_] = 1.0;

  // distinguish between contact and non-contact info of the terrain
  if (cflag) {
    tnx = t->c_nx;
    tny = t->c_ny;
  } else {
    tnx = t->nx;
    tny = t->ny;
  }

  // change variables for foothold z computatioin
  if (fflag) {
    nx   = t->reg_frad;
    ny   = t->reg_frad;
    down = t->reg_frad;
  }

  // check whether ix and iy dimension are OK
  if (ix < 1 || ix > tnx || iy < 1 || iy > tny) {
    printf("Error in terrain indices\n");
    return FALSE;
  }

  // make sure that nx and ny are multiples of the downsampling as caching
  // will brake otherwise
  nx = rint(((double)nx)/((double)down))*down;
  ny = rint(((double)ny)/((double)down))*down;

  // check whether nx and ny dimension are OK
  if (nx > MAX_NEIGHBORS || ny > MAX_NEIGHBORS) {
    printf("Too large neighborhood\n");
    return FALSE;
  }

  // compute start and end indices for the regression data
  // Note: negative indices are allowed -- they will trigger
  // a query outside of the terrain board
  sx = ix-nx;
  ex = ix+nx;
  sy = iy-ny;
  ey = iy+ny;

  // the number of data to be used in the regression
  n_data   = ((ex-sx)/down+1) * ((ey-sy)/down+1);

  // keep enough memory for y arround
  if (last_n_data < n_data) {
    my_free_vector(y,1,last_n_data);
    y = my_vector(1,n_data);
  }

  // the indices into the cache (written for a more general case,
  // one could just write ind1=nx ind2=ny, etc.)
  ind1 = ix-sx;
  ind2 = iy-sy;
  ind3 = ex-ix;
  ind4 = ey-iy;

  // this should not be possible -- just to be safe ....
  if (ind1 < 0 || ind2 < 0 || ind3 < 0 || ind4 < 0) {
    printf("Error in index computation!\n");
  }

#if USE_CACHE
  // check whether some compuations are already cached
  if (MatrixCache[ind1][ind2] != NULL) {
    need_memory = FALSE;
    if (MatrixCache[ind1][ind2][ind3][ind4]!=NULL) {
      XTXinvXT = MatrixCache[ind1][ind2][ind3][ind4];
      need_matrix = FALSE;
    } else
      need_matrix = TRUE;
  } else {
    need_memory = TRUE;
    need_matrix = TRUE;
  }

  // allocate a memory array if needed
  if (need_memory) {
    MatrixCache[ind1][ind2] = (Matrix **)my_calloc(nx+1,sizeof(Matrix *),MY_STOP);
    for (i=0; i<=nx; ++i)
      MatrixCache[ind1][ind2][i] = (Matrix *)my_calloc(ny+1,sizeof(Matrix),MY_STOP);
  }
#endif
  
  // compute the regression model if needed
  if (need_matrix) {
    // allocate memory
    X        = my_matrix(1,n_data,1,3);
    XTX      = my_matrix(1,3,1,3);
    XTXinv   = my_matrix(1,3,1,3);
    XTXinvXT = my_matrix(1,3,1,n_data);
  
    // create XTX and XTy matrices
    count = 0;
    for (i=sx; i<=ex; i+=down) {
      for (j=sy; j<=ey; j+=down) {
	++count;
	X[count][1] = (i-ix)*t->dx;
	X[count][2] = (j-iy)*t->dy;
	X[count][3] = 1.0;
      }
    }
    
    if (count != n_data) {
      printf("Counting Error\n");
      return FALSE;
    }
    
    mat_mult_inner(X,XTX);
    my_inv_ludcmp(XTX, 3, XTXinv);
    mat_mult_normal_transpose(XTXinv,X,XTXinvXT);

    my_free_matrix(X,1,n_data,1,3);
    my_free_matrix(XTX,1,3,1,3);
    my_free_matrix(XTXinv,1,3,1,3);

#if USE_CACHE
    MatrixCache[ind1][ind2][ind3][ind4] = XTXinvXT;
#else
    my_free_matrix(XTXinvXT,1,3,1,n_data);
#endif

  }    

  // generate the y vector
  count = 0;
  max_z = -1.e10;
  for (i=sx; i<=ex; i+=down) {
    for (j=sy; j<=ey; j+=down) {
      ++count;

      if (i >= 1 && i <= tnx && j >= 1 && j<=tny) {

	if (cflag)
	  y[count]    = t->c_z[i][j];
	else
	  if (fflag) {
	    y[count]    = t->z[i][j]; // - sqrt(sqr(i-ix)+sqr(j-iy))*0.001; // ldog specific hack
	  } else
	    y[count]    = t->z[i][j];

      } else {

	double vl[N_CART+1];

	if (cflag) { // contacts are computed from a much more local regression, 
	             // thus, we just assume that outside the board the z coordinate
                     // is the same as the closest board coordinate
          m = i;
          n = j;
          if (m < 1)
	    m = 1;
	  else if (m > tnx)
	    m = tnx;

	  if (n < 1)
	    n = 1;
	  else if (n > tny)
	    n = tny;
	  
	  y[count] = t->c_z[m][n];

	} else {

	  // compute query in world coordinates
	  vl[_X_] = (i-1)*t->dx+t->xorg;
	  vl[_Y_] = (j-1)*t->dy+t->yorg;
	  vl[_Z_] = 0.0;
	  
	  getTerrainInfoSwitched(vl[_X_], vl[_Y_], TRUE, TRUE, &taux);
	  if (fflag) {
	    y[count] = taux.z; //  - sqrt(sqr(i-ix)+sqr(j-iy))*0.001;; // ldog specific hack
	  } else
	    y[count] = taux.z;
	  
	}
	
      }
      
      if (y[count] > max_z)
	max_z = y[count];

    }
  }

  // compute the regression results
  mat_vec_mult_size(XTXinvXT,3,n_data,y,n_data,beta);

  // compute nMSE value
  sum_e2 = 0.0;
  count  = 0.0;
  for (i=sx; i<=ex; i+=down) {
    for (j=sy; j<=ey; j+=down) {
      ++count;
      sum_e2 += sqr(y[count] - ((i-ix)*t->dx*beta[1] + (j-iy)*t->dy*beta[2] + beta[3]));
    }
  }
  // the normalization is done w.r.t. the leg length -- nMSE creates
  // nonsense if var(y) is very small, as it is in flat terrain
  tinfo->n_nMSE = (sum_e2/(double)n_data)/sqr(0.01);
  std = sqrt(sum_e2/(double)n_data);

  // outlier removal
  
  // finally the surface normal from the cross product of a unit evaluation
  // at (0,1) and (1,0) minus the intercept vector (0,0,beta[3]) -- this
  // give the very simple result for the normal
  aux = sqrt(sqr(beta[1])+sqr(beta[2])+1);
  if (aux <= 1.e-10)
    aux = 1.e-10;
  tinfo->n[_X_] = -beta[1]/aux;
  tinfo->n[_Y_] = -beta[2]/aux;
  tinfo->n[_Z_] = 1./aux;

  // the absolute slope angle
  tinfo->slope = fabs(acos(tinfo->n[_Z_]));

  // the predicted z coordinate from regression (smoothed z)
  if (fflag) {
    //tinfo->fz  = beta[3];
    tinfo->fz  = computeMedian(y,count);
    //tinfo->fz  = max_z; // ldog specific hack
    tinfo->maxz = max_z;
  } else
    tinfo->pz  = beta[3];
  
  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  getNextTerrainID
\date  April 2006
   
\remarks 

        Returns the ID of the next free terrain ID

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   returns the next free terrain ID or FALSE, if nothing available.

 ******************************************************************************/
int
getNextTerrainID(void)
{
  int i;

  for (i=1; i<=MAX_TERRAINS; ++i)
    if (terrains[i].status != TRUE)
      return i;

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  setTerrainGroundZ
\date  April 2006
   
\remarks 

        sets the default z position of the floor

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     z : z position of the floor

 ******************************************************************************/
void
setTerrainGroundZ(double z)
{
  ground_level_z = z;
}


/*!*****************************************************************************
 *******************************************************************************
\note  cacheTerrainInfo
\date  April 2006
   
\remarks 

        This functions caches the terrain info for all existing boards
        if necessary.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

         none

 ******************************************************************************/
void
cacheTerrainInfo(void)
{

  /* initialize a thread for the caching processing to run in the background */
  if (pthread_create( &cthread_terrain, NULL, cacheTerrainInfoThread, (void*) NULL) != 0)
    return;

}

static void *
cacheTerrainInfoThread(void *dptr)
{

  int          i,j,m,n;
  Terrain      *t;
  TerrainInfo  tinfo;
  int          ID;
  
  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (ID=1; ID<=MAX_TERRAINS; ++ID) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[ID]);

    if (!t->status)
      continue;

    printf("Caching terrain %s started ...\n",t->tfname);
    fflush(stdout);

    // pre-compute normal information
    for (i=1; i<=t->nx; ++i) 
      for (j=1; j<=t->ny; ++j) 
	if (!t->cached[i][j]) {
	  pthread_mutex_lock( &mutex_terrain );
	  computeTerrainNormal(ID, i, j, t->reg_rad, t->reg_rad, t->reg_down, 
			       FALSE, TRUE, &tinfo);
	  t->fz[i][j]     = tinfo.fz;
	  computeTerrainNormal(ID, i, j, t->reg_rad, t->reg_rad, t->reg_down, 
			       FALSE, FALSE, &tinfo);
	  t->n_nMSE[i][j] = tinfo.n_nMSE;
	  t->pz[i][j]     = tinfo.pz;
	  t->n_x[i][j]    = tinfo.n[_X_];
	  t->n_y[i][j]    = tinfo.n[_Y_];
	  t->n_z[i][j]    = tinfo.n[_Z_];
	  t->cached[i][j]  = TRUE;
	  pthread_mutex_unlock( &mutex_terrain );
	}

    if (t->reg_crad != 0) 
      for (i=1; i<=t->nx_local; ++i) 
	for (j=1; j<=t->ny_local; ++j) 
	  if (!t->ccached[i][j]) {
	    pthread_mutex_lock( &mutex_terrain );
	    computeTerrainNormal(ID, i, j, t->reg_crad, t->reg_crad, 1, TRUE, FALSE, &tinfo);
	    t->cn_x[i][j]    = tinfo.n[_X_];
	    t->cn_y[i][j]    = tinfo.n[_Y_];
	    t->cn_z[i][j]    = tinfo.n[_Z_];
	    t->ccached[i][j]  = TRUE;
	    pthread_mutex_unlock( &mutex_terrain );
	  }

    printf("Caching terrain %s finished\n",t->tfname);
    
  }

  return NULL;

}

/*!*****************************************************************************
 *******************************************************************************
\note  fillTerrainPadding
\date  April 2006
   
\remarks 

        Computes the z values for the padded terrain locations

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

         none

 ******************************************************************************/
void
fillTerrainPadding(void)
{

  int          i,j,m,n;
  Terrain      *t;
  TerrainInfo  tinfo;
  int          ID;
  double       vl[N_CART+1];
  
  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (ID=1; ID<=MAX_TERRAINS; ++ID) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[ID]);

    if (!t->status)
      continue;

    printf("Padding board %s ...",t->tfname);
    fflush(stdout);

    // loop over all padded terrain states
    for (i=1; i<=t->padx; ++i) {
      for (j=1; j<=t->ny; ++j) {
	// get terrain z 
	vl[_X_] = (i-1)*t->dx+t->xorg;
	vl[_Y_] = (j-1)*t->dy+t->yorg;
	vl[_Z_] = 0.0;
	getTerrainInfoSwitched(vl[_X_], vl[_Y_], TRUE, TRUE, &tinfo);
	t->z[i][j] = tinfo.z;
	//t->z[i][j] = ground_level_z;
      }
    }
    
    for (i=t->nx-t->padx+1; i<=t->nx; ++i) {
      for (j=1; j<=t->ny; ++j) {
	// compute query in world coordinates
	vl[_X_] = (i-1)*t->dx+t->xorg;
	vl[_Y_] = (j-1)*t->dy+t->yorg;
	vl[_Z_] = 0.0;
	getTerrainInfoSwitched(vl[_X_], vl[_Y_], TRUE, TRUE, &tinfo);
	t->z[i][j] = tinfo.z;
	//t->z[i][j] = ground_level_z;
      }
    }

    for (i=t->padx+1; i<=t->nx-t->padx; ++i) {
      for (j=1; j<=t->pady; ++j) {
	// compute query in world coordinates
	vl[_X_] = (i-1)*t->dx+t->xorg;
	vl[_Y_] = (j-1)*t->dy+t->yorg;
	vl[_Z_] = 0.0;
	getTerrainInfoSwitched(vl[_X_], vl[_Y_], TRUE, TRUE, &tinfo);
	t->z[i][j] = tinfo.z;
	//t->z[i][j] = ground_level_z;
      }
    }
    
    for (i=t->padx+1; i<=t->nx-t->padx; ++i) {
      for (j=t->ny-t->pady+1; j<=t->ny; ++j) {
	// compute query in world coordinates
	vl[_X_] = (i-1)*t->dx+t->xorg;
	vl[_Y_] = (j-1)*t->dy+t->yorg;
	vl[_Z_] = 0.0;
	getTerrainInfoSwitched(vl[_X_], vl[_Y_], TRUE, TRUE, &tinfo);
	t->z[i][j] = tinfo.z;
	//t->z[i][j] = ground_level_z;
      }
    }

    printf(" done\n");
    
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  read_terrain_file
\date  April 2006
   
\remarks 

        reads a terrain file into a temporary array, and returns admin
        variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname   : terrain file name
 \param[in,out] array   : pointer to array -- memory will be allocated 
 \param[out]    nx      : number of x values in array
 \param[out]    ny      : number of y values in array
 \param[out]    min_x,max_x,min_y,max_y : min max values of array
 \param[in]     flag    : if TRUE binary terrain file is created if needed
                     but nothing else is done or returned (i.e., array
                     will not be allocated)
 \param[out]    count   : the number elements in array

 ******************************************************************************/
int
read_terrain_file(char *fname, Matrix *array,
		  int *nx, int *ny, double *min_x, double *max_x,
		  double *min_y, double *max_y, int flag, int *count)

{
  int     i,j,rc;
  FILE   *fp;
  double  x,y,z,no_go;
  char    string[100];
  char    command[1000];
  Matrix  temp;
  int     temp_size = 500000;
  char    c;
  int     ic;
  int     n_line_items=3;
  struct  stat sobj;
  struct  stat s;
  struct  stat sbin;


  // try to open the ascii version of file and check how many columns exist
  fp = fopen(fname,"r");
  if (fp != NULL) {
    // test whether a no-go column exists
    ic = 0;
    while (ic < 100 && (string[ic++]=fgetc(fp)) != '\n')
      ;
    string[ic-1] = '\0';
    if (sscanf(string,"%lf %lf %lf %lf",&x,&y,&z,&no_go) == 3)
      n_line_items = 3;
    else
      n_line_items = 4;
    fclose(fp);
  }

  // try to open the noisy .xyz file
  strcpy(string,fname);
  i=strlen(string);
  string[i-3]='x';
  string[i-2]='y';
  string[i-1]='z';
  
  if (stat(string,&sobj) == 0) { // the file exists

    // check date in comparison to .asc file and regenerate .asc if needed
    stat(fname,&s);
    if (sobj.st_mtime > s.st_mtime || n_line_items != 4) // need update
    {
      snprintf(command, 1000, "xundegrade %s %s", string, fname);
      printf("Trying to process noisy file with command: %s\n", command);
      rc=system(command);
    }
  }

  // try to open *.obj version of file
  strcpy(string,fname);
  i=strlen(string);
  string[i-3]='o';
  string[i-2]='b';
  string[i-1]='j';

  if (stat(string,&sobj) == 0) { // the file exists

    // check date in comparison to .asc file and regenerate .asc if needed
    stat(fname,&s);
    if (sobj.st_mtime > s.st_mtime || n_line_items != 4) // need update
      convertObj2Asc(string);
  }

  // try to open binary version of file
  sprintf(string,"%s.bin",fname);
  fp = fopen(string,"r");

  if (fp != NULL) {

    // check whether the .bin file is still up to date 
    stat(fname,&s);
    stat(string,&sbin);

    if (s.st_mtime > sbin.st_mtime) { // process asc file

      fclose(fp);

    } else { // continue processing bin file

      if (flag) {
	fclose(fp);
	return TRUE;
      }

      rc=fscanf(fp,"%d %d %d %lf %lf %lf %lf",
	     count,nx,ny,min_x,max_x,min_y,max_y);
      fgetc(fp);
      temp = my_matrix(1,*count,1,4);
      fread_mat(fp,temp);
      fclose(fp);
      
      *array = temp;

      return TRUE;

    }

  }

  // try to open the ascii version of file
  fp = fopen(fname,"r");
  if (fp == NULL) {
    if (!flag)
      printf("Cannot read terrain file >%s<\n",fname);
    return FALSE;
  }

  *count = 0;
  *max_x = -1.e10;
  *min_x =  1.e10;
  *max_y = -1.e10;
  *min_y =  1.e10;

  temp    = my_matrix(1,temp_size,1,4);

  // test whether a no-go column exists
  ic = 0;
  while (ic < 100 && (string[ic++]=fgetc(fp)) != '\n')
    ;
  string[ic-1] = '\0';
  if (sscanf(string,"%lf %lf %lf %lf",&x,&y,&z,&no_go) == 3)
    n_line_items = 3;
  else
    n_line_items = 4;

  //printf("string = >%s< %d\n",string,n_line_items);

  rewind(fp);


  while (TRUE) {

    if (n_line_items == 3) {
      if ((rc=fscanf(fp,"%lf %lf %lf",&x,&y,&z)) == EOF)
	break;
      no_go = FALSE;
    } else {
      if ((rc=fscanf(fp,"%lf %lf %lf %lf",&x,&y,&z,&no_go)) == EOF)
	break;
    }

    if (++(*count) > temp_size) {
      mat_add_shape(&temp,100000,0);
      temp_size += 100000;
    }
    temp[*count][1] = x;
    temp[*count][2] = y;
    temp[*count][3] = z;
    temp[*count][4] = no_go;

    if (x > *max_x)
      *max_x = x;
    if (x < *min_x)
      *min_x = x;
    if (y > *max_y)
      *max_y = y;
    if (y < *min_y)
      *min_y = y;

  }

  fclose(fp);

  // determine the correct dimensions of the terrain file assuming a mm raster
  *nx = rint(*max_x - *min_x) + 1;
  *ny = rint(*max_y - *min_y) + 1;

  // fudge the matrix size -- this won't hurt anybody
  temp[0][0] = *count;

  // write terrain file as binary file for quicker future processing
  sprintf(string,"%s.bin",fname);
  fp = fopen(string,"w");
  if (fp == NULL) {
    printf("Cannot write terrain file >%s<\n",string);
    return FALSE;
  }

  fprintf(fp,"%d %d %d %lf %lf %lf %lf\n",
	  *count,*nx,*ny,*min_x,*max_x,*min_y,*max_y);
  fwrite_mat(fp,temp);
  fclose(fp);

  if (flag) {
    my_free_matrix(temp,1,*count,1,4);
    return TRUE;
  }


  *array = temp;


  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainName
\date  April 2006
   
\remarks 

        returns the name of the associated terrain file of a terrain

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID      : terrain array index ( between 1 and MAX_TERRAIN)

 ******************************************************************************/
int
getTerrainFileName(int ID, char *tfname)
{
  Terrain *t;  

  // check for validity of terrain index
  if (ID < 1 || ID > MAX_TERRAINS) {
    printf("Terrain %d out of range of terrains from 1-%d\n",ID,MAX_TERRAINS);
    return FALSE;
  }

  t = &(terrains[ID]);

  if (!t->status)
    return FALSE;

  strcpy(tfname,t->tfname);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  setTerrainID
\date  April 2006
   
\remarks 

        allow overwriting a terrain ID

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID      : terrain array index ( between 1 and MAX_TERRAIN)
 \param[in]     tID     : desired new terrain ID

 ******************************************************************************/
int
setTerrainID(int ID, int tID)
{
  Terrain *t;  

  // check for validity of terrain index
  if (ID < 1 || ID > MAX_TERRAINS) {
    printf("Terrain %d out of range of terrains from 1-%d\n",ID,MAX_TERRAINS);
    return FALSE;
  }

  t = &(terrains[ID]);

  if (!t->status)
    return FALSE;

  t->ID = tID;
  printf("**** Terrain %d was set to terrain-ID %d ****\n",ID,tID);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainLocalCoordinatesID
\date  April 2006
   
\remarks 

        for a given x,y,z position in world coordinates, and the terrainID,
        return the position in local terrain coordiantes, irrespective whether
        we really have data at this point of the terrain

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point (world coordiantes)
 \param[in]     y        : y position of query point (world coordiantes)
 \param[in]     z        : z position of query point (world coordiantes)
 \param[in]     tID      : the terrain ID
 \param[out]    xl       : the local x coordiante
 \param[out]    yl       : the local y coordiante
 \param[out]    zl       : the local z coordiante
   
     Returns TRUE if data found in terrains, or FALSE if not.

 ******************************************************************************/
int
getTerrainLocalCoordinatesID(double x, double y, double z, int tID, 
			    double *xl, double *yl, double *zl)
{
  int           i,j,m,n,r;
  Terrain      *t;
  static int    firsttime = TRUE;
  static Matrix R;
  static Vector v,vv;

  if (firsttime) {
    firsttime = FALSE;
    R  = my_matrix(1,N_CART,1,N_CART);
    v  = my_vector(1,N_CART);
    vv = my_vector(1,N_CART);
  }

  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    if (t->ID == tID) {
      
      quatToRotMat(&(t->orient),R);      
      
      v[_X_] = x - t->pos.x[_X_];
      v[_Y_] = y - t->pos.x[_Y_];
      v[_Z_] = z - t->pos.x[_Z_];
      
      mat_vec_mult(R,v,vv);
      
      *xl = vv[_X_];
      *yl = vv[_Y_];
      *zl = vv[_Z_];
      
      return TRUE;
      
    }

  }

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainLocalCoordinates
\date  April 2006
   
\remarks 

        for a given x,y position in world coordinates, return the 
        position in local terrain coordiantes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     x        : x position of query point (world coordiantes)
 \param[in]     y        : y position of query point (world coordiantes)
 \param[out]    tID      : the terrain ID
 \param[out]    xl       : the local x coordiante
 \param[out]    yl       : the local y coordiante
   
     Returns TRUE if data found in terrains, or FALSE if not.

 ******************************************************************************/
int
getTerrainLocalCoordinates(double x, double y, int *tID, double *xl, double *yl)
{
  int           i,j,m,n,r;
  Terrain      *t;
  static int    firsttime = TRUE;
  static Matrix R;
  double        z;
  static Vector v,vv;

  if (firsttime) {
    firsttime = FALSE;
    R  = my_matrix(1,N_CART,1,N_CART);
    v  = my_vector(1,N_CART);
    vv = my_vector(1,N_CART);
  }

  *tID = 0;

  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    // check whether index falls into terrain board
    m = rint((x - t->xorg)/t->dx) + 1;
    n = rint((y - t->yorg)/t->dy) + 1;
    
    if (m >= 1 && m <= t->nx && n >=1 && n <=t->ny) {
      
      *tID = t->ID;
      z   = t->z[m][n];
      quatToRotMat(&(t->orient),R);      
      
      v[_X_] = x - t->pos.x[_X_];
      v[_Y_] = y - t->pos.x[_Y_];
      v[_Z_] = z;
      
      mat_vec_mult(R,v,vv);
      
      *xl = vv[_X_];
      *yl = vv[_Y_];
      
      return TRUE;
      
    }

  }

  return FALSE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  getTerrainWorldCoordinatesID
\date  April 2006
   
\remarks 

        for a given x,y,z position in local terrain coordinates, and the
        terrainID, return the position in world terrain coordiantes,
        irrespective whether we really have data at this point of the
        terrain

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     xl       : x position of query point (local coordinates)
 \param[in]     yl       : y position of query point (local coordiantes)
 \param[in]     zl       : z position of query point (local coordiantes)
 \param[in]     tID      : the terrain ID
 \param[out]    x        : the world x coordiante
 \param[out]    y        : the world y coordiante
 \param[out]    z        : the world y coordiante
   
     Returns TRUE if data found in terrains, or FALSE if not.

 ******************************************************************************/
int
getTerrainWorldCoordinatesID(double xl, double yl, double zl, int tID, 
			     double *x, double *y, double *z)
{
  int           i,j,m,n,r;
  Terrain      *t;
  static int    firsttime = TRUE;
  static Matrix R;
  static Vector v,vv;

  if (firsttime) {
    firsttime = FALSE;
    R  = my_matrix(1,N_CART,1,N_CART);
    v  = my_vector(1,N_CART);
    vv = my_vector(1,N_CART);
  }

  // loop over all terrain boards and try to find whether query point is
  // covered by the given board
  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    if (t->ID == tID) {
      
      quatToRotMatInv(&(t->orient),R);      

      v[_X_] = xl;
      v[_Y_] = yl;
      v[_Z_] = zl;

      mat_vec_mult(R,v,vv);
      
      *x = vv[_X_] + t->pos.x[_X_];
      *y = vv[_Y_] + t->pos.x[_Y_];
      *z = vv[_Z_] + t->pos.x[_Z_];

      return TRUE;
      
    }

  }

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  getContactTerrainMinMax
\date  Dec. 2006
   
\remarks 

        Returns the board min-max dimensions in local coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     tfname   : terrain file name needed to identify the terrain
 \param[out]    x_min    : x local minimum
 \param[out]    x_max    : x local maximum
 \param[out]    y_min    : y local minimum
 \param[out]    y_max    : y local maximum

     Returns TRUE if data found in terrains, or FALSE if not. 

 ******************************************************************************/
int
getContactTerrainMinMax(char *tfname, 
			double *x_min, double *x_max, double *y_min, double *y_max)
{
  int           i,j,m,n;
  Terrain      *t;
  TerrainInfo  tinfo;

  // loop over all terrain boards 
  for (i=1; i<=MAX_TERRAINS; ++i) {

    // use a simpler variable for convenience and check for active terrain board
    t = &(terrains[i]);

    if (!t->status)
      continue;

    if (strcmp(t->tfname,tfname) != 0)
      continue;

    *x_min = t->c_dxorg;
    *y_min = t->c_dyorg;
    
    *x_max = (t->c_nx-1) * t->dx + t->c_dxorg; 
    *y_max = (t->c_ny-1) * t->dy + t->c_dyorg; 

    return TRUE;

  }

  return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  computeMedian
\date  March 2007
   
\remarks 

        returns the median of the given vector

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     v        : vector of values
 \param[in]     n_v      : number of values in v

     Returns the value of the median in v

 ******************************************************************************/
static double
computeMedian(double *v, int n_v) 
{
  int i,j,n;
  static Vector lv = NULL;
  static int    n_lv = 0;
  double aux;

  if (lv == NULL) {
    lv   = my_vector(1,n_v);
    n_lv = n_v;
  }

  if (n_lv < n_v) {
    vec_add_shape(&lv, n_v-n_lv);
    n_lv = n_v;
  }
  
  if (n_v%2 == 0)
    n = n_v/2+1;
  else
    n = ((int)(n_v/2))+1;

  for (i=1; i<=n_v; ++i)
    lv[i] = v[i];

  for (i=1; i<=n; ++i) {
    for (j=1; j<=n_v-i; ++j) {
      if (lv[j+1] > lv[j]) {
	aux     = lv[j];
	lv[j]   = lv[j+1];
	lv[j+1] = aux;
      }
    }
  }

  if (n%2 == 0) 
    aux = (lv[n]+lv[n-1])/2.;
  else
    aux = lv[n];

  return aux;

}

/*!*****************************************************************************
 *******************************************************************************
\note  convertObj2Asc
\date  April 2006
   
\remarks 

        converts an OBJ (WAVEFRONT/MAJA) file to an ASC file, and keeps track
        of undercuts. As a result, the .asc file is written to disk in the
        same directory as the OBJ file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname   : terrain file name

        returns TRUE for success, and FALSE for failure

 ******************************************************************************/
#define MAX_STRING_LEN 100
#define N_CLUSTERS     20
#define CLIFF_HEIGHT   20
#define CLIFF_SEARCH_D  4
#define DIFFUSION_R    20
static int
convertObj2Asc(char *fname)
{
  int     i,j,k,r;
  FILE   *fp;
  double  v1[3+1];
  double  v2[3+1];
  double  v3[3+1];
  double  x,y,z;
  char    string[MAX_STRING_LEN+1];
  char    c;
  char    key[10];
  int     n_v=0;
  int     n_f=0;
  Matrix  v;
  iMatrix f;
  double  max_v[N_CART+1] = {0.0,-1.e10,-1.e10,-1.e10};
  double  min_v[N_CART+1] = {0.0,+1.e10,+1.e10,+1.e10};
  int     nx,ny;
  Matrix  Z[N_CLUSTERS+1];
  iMatrix N[N_CLUSTERS+1];
  Matrix  U;
  double  alpha_1, alpha_2;
  char    fname_out[100];


  // try to open the file
  fp = fopen(fname,"r");
  if (fp == NULL) {
    printf("Cannot read terrain file >%s<\n",fname);
    return FALSE;
  }

  printf("Procdessing %s ...\n",fname);

  // count the number of faces and vertices in file
  while (TRUE) {

    // get the next line
    i=0;
    while ((c=fgetc(fp)) != '\n' && c != EOF) {
      string[i++] = c;
      if ( i >= MAX_STRING_LEN ) {
	printf("ERROR: max. string length exceeded\n");
	return FALSE;
      }
    }
    string[i] = '\0';
    
    // check for vertices and faces
    sscanf(string,"%s",key);
    if (strcmp(key,"v")==0)
      ++n_v;
    else if (strcmp(key,"f")==0)
      ++n_f;

    if (c == EOF)
      break;

  }

  printf("#faces=%d  #vertices=%d\n",n_f,n_v);

  rewind(fp);

  // allocate memory
  v = my_matrix(1,n_v,1,3);
  f = my_imatrix(1,n_f,1,3);

  // get the data
  n_v = n_f = 0;
  while (TRUE) {
    char s1[100],s2[100],s3[100];

    // get the next line
    i=0;
    while ((c=fgetc(fp)) != '\n' && c != EOF) {
      string[i++] = c;
      if ( i >= MAX_STRING_LEN ) {
	printf("ERROR: max. string length exceeded\n");
	return FALSE;
      }
    }
    string[i] = '\0';
    
    // check for vertices and faces
    sscanf(string,"%s",key);
    if (strcmp(key,"v")==0) {
      ++n_v;
      sscanf(string,"%s %lf %lf %lf",key,&v[n_v][1],&v[n_v][2],&v[n_v][3]);
      for (j=1; j<=N_CART; ++j) {
	if (v[n_v][j] > max_v[j])
	  max_v[j] = v[n_v][j];
	if (v[n_v][j] < min_v[j])
	  min_v[j] = v[n_v][j];
      }
    } else if (strcmp(key,"f")==0) {
      ++n_f;
      sscanf(string,"%s %s %s %s",key,s1,s2,s3);
      sscanf(s1,"%d",&f[n_f][1]);
      sscanf(s2,"%d",&f[n_f][2]);
      sscanf(s3,"%d",&f[n_f][3]);
    }

    if (c == EOF)
      break;
  }
  fclose(fp);

  printf("max: x=%6.2f y=%6.2f z=%6.2f\n",max_v[_X_],max_v[_Y_],max_v[_Z_]);
  printf("min: x=%6.2f y=%6.2f z=%6.2f\n",min_v[_X_],min_v[_Y_],min_v[_Z_]);

  
  // what are the dimensions of the terrain board matrix? And get memory ...
  nx     = rint(max_v[_X_] - min_v[_X_]) + 1;
  ny     = rint(max_v[_Y_] - min_v[_Y_]) + 1;
  for (i=1; i<=N_CLUSTERS; ++i) {
    Z[i] = my_matrix(1,nx,1,ny);
    mat_equal_scalar(EMPTY_TERRAIN, Z[i]);
    N[i] = my_imatrix(1,nx,1,ny);
  }
  U = my_matrix(1,nx,1,ny);


  // map all triangles into the terrain board
  for (i=1; i<=n_f; ++i) {
    double l,l1;
    int    n_steps;

    // determine the longest edge of the triangle
    l  = sqrt(vec_euc2_size(v[f[i][1]],v[f[i][2]],N_CART));
    l1 = sqrt(vec_euc2_size(v[f[i][1]],v[f[i][3]],N_CART));
    if (l1 > l)
      l = l1;
    l1 = sqrt(vec_euc2_size(v[f[i][2]],v[f[i][3]],N_CART));
    if (l1 > l)
      l = l1;

    n_steps = ceil(l)+1;

    // sample data from the triangle and project into terrain board
    for (j=1; j<=n_steps; ++j) {
      for (k=1; k<=n_steps; ++k) {
	double point[N_CART+1];
	int    r;
	int    ix,iy;
	int    cid;

	alpha_1 = (double)(j-1)/(double)(n_steps-1);
	alpha_2 = (double)(k-1)/(double)(n_steps-1);
	for (r=1; r<=N_CART; ++r)
	  point[r] =  (v[f[i][1]][r] + alpha_1*(v[f[i][2]][r]-v[f[i][1]][r]) -
	      v[f[i][3]][r]) * alpha_2 + v[f[i][3]][r];
	
	ix = rint(point[_X_] - min_v[_X_]) + 1;
	iy = rint(point[_Y_] - min_v[_Y_]) + 1;

	// cluster the z data to detect undercuts

	cid = ceil((point[_Z_] - min_v[_Z_])/10.0);
	if (cid <= 0)
	  cid = 1;
	if (cid > N_CLUSTERS)
	  cid = N_CLUSTERS;
	
	if (Z[cid][ix][iy] == EMPTY_TERRAIN) {
	  Z[cid][ix][iy] = point[_Z_];
	  ++N[cid][ix][iy];
	} else {
	  if (point[_Z_] > Z[cid][ix][iy])
	    Z[cid][ix][iy] = point[_Z_];
	  ++N[cid][ix][iy];
	}
	
      }
    }

  }

  // analyze potential undercuts from the number of z-clusters
  for (i=1; i<=nx; ++i) {
    for (j=1; j<=ny; ++j) {
      double count_clusters;
      int    ind_max_z; 
      double max_z;
      double aux;
      int    iaux;
      
      count_clusters = 0;
      ind_max_z = 1;
      max_z = -999;
      for (r=1; r<=N_CLUSTERS; ++r) {
	if (N[r][i][j] > 0) {
	  ++count_clusters;
	  if (Z[r][i][j] > max_z) {
	    max_z = Z[r][i][j];
	    ind_max_z = r;
	  }
	}
      }
      
      if (count_clusters > 1) { // make sure the max value is in the first cluster
	aux = Z[1][i][j];
	iaux = N[1][i][j];
	Z[1][i][j] = Z[ind_max_z][i][j];
	N[1][i][j] = N[ind_max_z][i][j];
	Z[ind_max_z][i][j] = aux;
	N[ind_max_z][i][j] = iaux;
      }	
      if (count_clusters > 2)  // sschaal hack "2" -> "3"
	      			// mrinal reverted it back to 2
	U[i][j] = TRUE;

    }
  }

  // search for which of the potential undercuts is close to a "cliff"
  for (i=1; i<=nx; ++i) {
    for (j=1; j<=ny; ++j) {
      int confirm;

      if (U[i][j] == TRUE) { // look for a cliff

	confirm = FALSE;

	for (k=i-CLIFF_SEARCH_D; k<=i+CLIFF_SEARCH_D; k+=2)
	  for (r=j-CLIFF_SEARCH_D; r<=j+CLIFF_SEARCH_D; r+=2) {
	    if (k > nx || k < 1 || r > ny || r < 1)
	      continue;
	    if (fabs(Z[1][i][j]-Z[1][k][r]) > CLIFF_HEIGHT)
	      confirm = TRUE;
	  }
	U[i][j] = confirm;
      }
    }
  }

  // diffuse the underscore cut into the neighbor hood of the undercuts
  for (i=1; i<=nx; ++i) {
    for (j=1; j<=ny; ++j) {
      double aux;

      if (U[i][j] == TRUE) {

	for (k=i-DIFFUSION_R; k<=i+DIFFUSION_R; ++k) {
	  for (r=j-DIFFUSION_R; r<=j+DIFFUSION_R; ++r) {
	    if (k > nx || k < 1 || r > ny || r < 1)
	      continue;
	    aux = 1.0-sqrt((double)(sqr(k-i)+sqr(r-j)))/(double)DIFFUSION_R;
	    if (aux > U[k][r])
	      U[k][r] = aux;
	  }
	}

      }

    }
  }

  strcpy(fname_out,fname);
  j = strlen(fname_out);
  fname_out[j-3]='a';
  fname_out[j-2]='s';
  fname_out[j-1]='c';
  fp = fopen(fname_out,"w");
  for (i=1; i<=nx; ++i)
    for (j=1; j<=ny; ++j) 
      if (Z[1][i][j] != EMPTY_TERRAIN)
	fprintf(fp,"%f %f %f %f\n",i-1+min_v[_X_],j-1+min_v[_Y_],Z[1][i][j],U[i][j]);
  fclose(fp);

  // clean up
  my_free_matrix(v,1,n_v,1,3);
  my_free_imatrix(f,1,n_f,1,3);
  for (r=1; r<=N_CLUSTERS; ++r) {
    my_free_matrix(Z[r],1,nx,1,ny);
    my_free_imatrix(N[r],1,nx,1,ny);
  }
  my_free_matrix(U,1,nx,1,ny);

  printf("... done with %s\n",fname);

  return TRUE;

}

