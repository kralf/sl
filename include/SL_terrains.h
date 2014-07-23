/*!=============================================================================
  ==============================================================================

  \file    SL_terrains.h

  \author 
  \date   

  ==============================================================================
  \remarks

  declarations needed by SL_terrains.c

  ============================================================================*/

#ifndef _SL_terrains_
#define _SL_terrains_

#define MAX_TERRAINS 5

typedef struct {         //!< terrain board structure
  int       status;      //!< TRUE or FALSE for active or inactive
  int       ID;          //!< terrain identifier
  char      tfname[100]; //!< terrain file associated with this terrain
  char      tfnasc[100]; //!< terrain file name with .asc appended
  SL_Cstate pos;         //!< origin of terrain in world coordinates (from Vicon)
  SL_quat   orient;      //!< terrain orientation as unit quaternion (from Vicon)
  double    dxorg_local; //!< x-offset of Z(1,1) relative to local origin on board
  double    dyorg_local; //!< y-offset of Z(1,1) relative to local origin on board
  double    c_dxorg;     //!< x-offset of Z(1,1) relative to local origin on board for contacts
  double    c_dyorg;     //!< y-offset of Z(1,1) relative to local origin on board for contacts
  double    xorg;        //!< x origin of global board in global coordinates
  double    yorg;        //!< y origin of global board in gloabl coordinates
  double    dx;          //!< delta x of terrain grid
  double    dy;          //!< delta y of terrain grid
  double    min_z;       //!< min z on terrain
  double    max_z;       //!< max z on terrain
  int       nx_local;    //!< number of states in x direction local board
  int       ny_local;    //!< number of states in y direction local board
  int       c_nx;        //!< number of states in x direction for contact checking
  int       c_ny;        //!< number of states in y direction for contact checking
  int       nx;          //!< number of states in x direction
  int       ny;          //!< number of states in y direction
  int       reg_rad;     //!< radius used for regression on terrain
  int       reg_down;    //!< down sampling for computing the regression
  int       reg_crad;    //!< regression radius used for contact checking
  int       reg_frad;    //!< regression radius used for foot z height
  int       padx;        //!< amount of padding in x
  int       pady;        //!< amount of padding in y
  int       c_padx;      //!< amount of padding in x for contacts
  int       c_pady;      //!< amount of padding in y for contacts
  fMatrix   z_local;     //!< matrix of heights of terrain at each x,y position
  fMatrix   no_go_local; //!< matrix of heights of terrain at each x,y position
  fMatrix   z;           //!< matrix of heights of terrain at each x,y position
  fMatrix   pz;          //!< matrix of predicted heights of terrain at each x,y position
  fMatrix   fz;          //!< matrix of foot placement heights at each x,y position
  fMatrix   n_nMSE;      //!< matrix of nMSE of normals
  fMatrix   n_x;         //!< matrix of x component of normal
  fMatrix   n_y;         //!< matrix of y component of normal
  fMatrix   n_z;         //!< matrix of z component of normal
  fMatrix   c_z;         //!< matrix of z component for contact checking
  fMatrix   c_no_go;     //!< matrix of no_go information in contact coordinates
  fMatrix   cn_x;        //!< matrix of x component of normal for contact
  fMatrix   cn_y;        //!< matrix of y component of normal for contact
  fMatrix   cn_z;        //!< matrix of z component of normal for contact
  fMatrix   no_go;       //!< matrix of go/no-go information
  iMatrix   cached;      //!< indicator matrix with TRUE/FALSE whether terrain info is cached
  iMatrix   ccached;     //!< indicator matrix with TRUE/FALSE whether terrain contact info is cached
} Terrain;

// a useful structure for terrain information
typedef struct {        //!< terrain board structure
  double    z;          //!< height above ground at query point
  double    pz;         //!< predicted height above ground at query point from regression
  double    fz;         //!< height used for foot placement
  double    maxz;       //!< max height within a 1cm radius
  double    n[N_CART+1];//!< normal vector at query point averaged over neighbors
  double    n_nMSE;     //!< nMSE of normal fitting
  double    slope;      //!< the slope as absolute angle relative to horizontal
  double    no_go;      //!< in [1,0], where 1 is bad, 0 is fine.
  int       ID;         //!< terrain ID
  int       in_padding; //!< TRUE/FALSE for in padding
} TerrainInfo;



#ifdef __cplusplus
extern "C" {
#endif

// global functions
int
readTerrainBoard(char *fname, int ID);
int    
setTerrainInfo(int ID, int tID, char *tfname, double *pos, double *orient,
	       int reg_rad, int reg_down, int reg_crad);
int
getTerrainInfo(double x, double y, TerrainInfo *tinfo);
int
getTerrainInfoZOnly(double x, double y, TerrainInfo *tinfo);
int
getNextTerrainID(void);
int
getContactTerrainInfo(double x, double y, char *tfname, double *z, double *n, double *no_go);
void
setTerrainGroundZ(double z);
void
cacheTerrainInfo(void);
void
fillTerrainPadding(void);
int
read_terrain_file(char *fname, Matrix *array,
		  int *nx, int *ny, double *min_x, double *max_x,
		  double *min_y, double *max_y, int flag, int *count);
int
getTerrainFileName(int ID, char *tfname);
int
setTerrainID(int ID, int tID);
int
getTerrainLocalCoordinates(double x, double y, int *tID, double *xl, double *yl);
int
getTerrainLocalCoordinatesID(double x, double y, double z, int tID, 
			    double *xl, double *yl, double *zl);
int
getTerrainWorldCoordinatesID(double xl, double yl, double zl, int tID, 
			     double *x, double *y, double *z);
int
getContactTerrainMinMax(char *tfname,
			double *x_min, double *x_max, double *y_min, double *y_max);

// external variables
extern double terrain_bounding_box_max[];
extern double terrain_bounding_box_min[];
extern Terrain terrains[MAX_TERRAINS+1];

#ifdef __cplusplus
}
#endif

#endif // _SL_terrains_
