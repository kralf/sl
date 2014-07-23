/*!=============================================================================
  ==============================================================================

  \file    SL_objects.h

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  SL_objects.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_objects_
#define _SL_objects_


#include "SL_objects_defines.h"


/*! structure to create objects in the environment */
typedef struct Object {
  char    name[STRING100];               /*!< object name */
  int     type;                          /*!< object type */
  int     contact_model;                 /*!< which contact model to be used */
  double  trans[N_CART+1];               /*!< translatory offset of object */
  double  rot[N_CART+1];                 /*!< rotational offset of object */
  double  scale[N_CART+1];               /*!< scaling in x,y,z */
  double  rgb[N_CART+1];                 /*!< color information */
  double  *contact_parms;                /*!< contact parameters */
  double  *object_parms;                 /*!< object parameters */
  char   *nptr;                          /*!< pointer to next object */
  double  f[N_CART+1];                   /*!< forces acting on object in world coordinates */
  double  t[N_CART+1];                   /*!< torques acting on object in world coordinates */
  int     display_list_active;           /*!< display list for open GL (only for terrains) */
  int     hide;                          /*!< allows hiding the object in the display */
} Object, *ObjectPtr;


#define MAX_CONNECTED 25

/*! structure to deal with contact forces */
typedef struct Contact {
  int        active;                           /*!< TRUE/FALSE: indicates whether this point should be checked for contacts */
  int        status;                           /*!< contact is true or false */
  int        friction_flag;                    /*!< flag for switching between different friction models */
  ObjectPtr  optr;                             /*!< ptr ofx object that is contacted */
  int        base_dof_start;                   /*!< to which DOF does this point connect? */
  int        off_link_start;                   /*!< which link should be used for moment arm */
  int        base_dof_end;                     /*!< to which DOF does this point connect? */
  int        off_link_end;                     /*!< which link should be used for moment arm */
  int        id_start;                         /*!< link ID where line starts */
  int        id_end;                           /*!< link ID where line ends */
  double     fraction_start;                   /*!< fraction relative to start point */
  double     fraction_end;                     /*!< fraction relative to end point */
  double     x[N_CART+1];                      /*!< point of contact in object coordintates */
  double     x_start[N_CART+1];                /*!< point of first contact */
  double     normal[N_CART+1];                 /*!< normal displacement vector */
  double     normvel[N_CART+1];                /*!< normal velocity vector */
  double     tangent[N_CART+1];                /*!< tangential displacement vector */
  double     tanvel[N_CART+1];                 /*!< tangential velocity vector */
  double     viscvel[N_CART+1];                /*!< velocity vector for viscous friction */
  double     f[N_CART+1];                      /*!< contact forces in world coordinates */
  double     n[N_CART+1];                      /*!< contact normal in world coordinates */
  double     face_index;                       /*!< _X_, _Y_, or _Z_ to indicate with which face we are in contact */
  int        n_connected_links;                /*!< number of connected links */
  int        connected_links[MAX_CONNECTED+1]; /*!< list of connected links (only used for link end points, not intermediate points */
  int        force_condition[MAX_CONNECTED+1]; /*!< what force conditions are permitted */
  // these options are only used for special point contacts 
  int        point_contact_flag;               /*!< indicates that this is a special point contact in local coordinates with norm vector */
  double     local_point_pos[N_CART+1];        /*!< position of point contact in local coordinates */
  double     local_point_norm[N_CART+1];       /*!< normal vector of point contact in local coordinates */
						  
} Contact, *ContactPtr;


#ifdef __cplusplus
extern "C" {
#endif

  // external variables

  // shared functions
  int initObjects(void);

  ObjectPtr  addSphere(char *name, double *rgb, double *pos, double *rot, 
		       double *scale, int contact,
		       double *parms, int n_faces);
  ObjectPtr  addCube(char *name, double *rgb, double *pos, double *rot, 
		     double *scale, int contact,
		     double *parms);
  ObjectPtr  addCylinder(char *name, double *rgb, double *pos, double *rot, 
			 double *scale, int contact,
			 Vector parms, int n_faces);
  ObjectPtr  addObject(char *name, int type, int contact, 
		       double *rgb, double *trans, double *rot, 
		       double *scale, Vector cspecs, Vector ospecs);

  void       checkContacts(void);
  void       readObjects(char *fname);

  int        changeObjPosByName(char *name, double *pos, double *rot);
  int        deleteObjByName(char *name);
  int        changeHideObjByName(char *name, int hide);
  int        getObjForcesByName(char *name, double *f, double *t);

  void       changeObjPosByPtr(ObjectPtr ptr, double *pos, double *rot);
  int        getObjForcesByPtr(ObjectPtr ptr, double *f, double *t);
  ObjectPtr  getObjPtrByName(char *name);

  int        read_extra_contact_points(char *fname);
  void       computeContactPoint(ContactPtr cptr, double **lp, double ***al, double *x);




  // external variables
  extern ObjectPtr  objs;
  extern ContactPtr contacts;
  extern SL_uext   *ucontact;
  extern int        n_contacts;

  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_objects_ */
