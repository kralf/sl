/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_objects.c

  \author 
  \date   

  ==============================================================================
  \remarks

  handling of objects in SL

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_common.h"
#include "SL_kinematics.h"
#include "SL_simulation_servo.h"
#include "SL_shared_memory.h"

// contact related defines
enum ForceConditions {
  F_FULL=1,  //!< admit forces in all directions
  F_HALF,    //!< admit forces only in half space
  F_ORTH,    //!< admit forces only perpindicular to a line
  F_NULL,    //!< place holder for not used

  NFC
};
#define N_FORCE_CONDITIONS (NFC-1)

static char f_cond_names[][20]= {
  {"dummy"},
  {"full"},
  {"half"},
  {"orth"},
  {"no_used"}
};

// global variables
ObjectPtr  objs = NULL;
ContactPtr contacts=NULL;
SL_uext   *ucontact;
int        n_contacts;

// local variables

// global functions 

// local functions
static void  computeContactForces(ObjectPtr optr, ContactPtr cptr);
static void  contactVelocity(int cID, ObjectPtr optr, double *v);
static void  addObjectSync(char *name, int type, int contact, double *rgb, double *pos, 
			   double *rot, double *scale, double *cparms, double *oparms);
static void  deleteObjByNameSync(char *name);
static void  changeHideObjByNameSync(char *name, int hide);
static void  changeObjPosByNameSync(char *name, double *pos, double *rot);
static void  convertGlobal2Object(ObjectPtr optr, double *xg, double *xl);
static void  computeStart2EndNorm(double *xs, double *xe, ObjectPtr optr, double *n);
static void  projectForce(ContactPtr cptr, ObjectPtr optr);


// external functions

/*!*****************************************************************************
 *******************************************************************************
\note  initObjects
\date  Nov 2007
\remarks 

        initializes object related variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
int
initObjects(void) 
{
  int n;

  // check how may contact points we need
  n=count_extra_contact_points(config_files[CONTACTS]);

  // contacts
  contacts = my_calloc(n_links+1+n,sizeof(Contact),MY_STOP);
  ucontact = my_calloc(n_dofs+1,sizeof(SL_uext),MY_STOP);

  // initalize objects in the environment
  readObjects(config_files[OBJECTS]);

  n_contacts = n+n_links;

  return TRUE;

}
 
/*!*****************************************************************************
 *******************************************************************************
\note  addObject
\date  Nov 2000
   
\remarks 

add an object to the environment

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     type       : object type
 \param[in]     contact    : contact type
 \param[in]     rgb        : rgb values for color
 \param[in]     trans      : translatory offset
 \param[in]     rot        : rotary offset
 \param[in]     scale      : scaling of object
 \param[in]     cspecs     : contact specifications (0-th element has #parm)
 \param[in]     ospecs     : object specifications  (0-th element has #parm)

 ******************************************************************************/
ObjectPtr
addObject(char *name, int type, int contact, double *rgb, double *trans, double *rot, 
	  double *scale, Vector cspecs, Vector ospecs)

{
  int        i;
  ObjectPtr  ptr=NULL;
  ObjectPtr  last_ptr=NULL;
  char       string[1000];
  int        n_cps=0, n_ops=0;
  int        new_obj_flag = TRUE;

  /* is object already present and only needs update? */
  if (objs != NULL) {

    ptr = objs;
    do {
      if (strcmp(ptr->name,name)==0) {
	new_obj_flag = FALSE;
	break;
      }
      last_ptr = ptr;
      ptr = (ObjectPtr) ptr->nptr;
    } while ( ptr != NULL);
    
  }

  if (new_obj_flag) {
    /* allocate new object */
    ptr = my_calloc(1,sizeof(Object),MY_STOP);
  }

  if (cspecs == NULL) {
    n_cps = 0;
  } else {
    n_cps = cspecs[0];
    if (n_cps > 0 && new_obj_flag)
      ptr->contact_parms = my_vector(1,n_cps);
  }
  if (ospecs == NULL) {
    n_ops = 0;
  } else {
    n_ops = ospecs[0];
    if (n_ops > 0 && new_obj_flag)
      ptr->object_parms  = my_vector(1,n_ops);
  }

  /* assign values */
  strcpy(ptr->name, name);
  ptr->type = type;
  ptr->display_list_active = FALSE;
  ptr->hide = FALSE;
  ptr->contact_model = contact;
  for (i=1; i<=N_CART; ++i) {
    ptr->rgb[i]   = rgb[i];
    ptr->trans[i] = trans[i];
    ptr->rot[i]   = rot[i];
    ptr->scale[i] = scale[i];
  }
  for (i=1; i<=n_ops; ++i) {
    ptr->object_parms[i]=ospecs[i];
  }
  for (i=1; i<=n_cps; ++i) {
    ptr->contact_parms[i]=cspecs[i];
  }

  if (new_obj_flag) {
    ptr->nptr=NULL;
    if (objs == NULL) {
      objs = ptr;
    } else {
      last_ptr->nptr = (char *) ptr;
    }
  }

  if (strcmp(servo_name,"task")==0)  // communicate info to other servos
    addObjectSync(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

  return ptr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  addCube
\date  Nov 2000
   
\remarks 

       add a cube to the object list

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to rotation vector
 \param[in]     contact    : ID of contact model  
 \param[in]     parms      : array of contact parameters

 ******************************************************************************/
ObjectPtr
addCube(char *name, double *rgb, double *pos, double *rot, 
	double *scale, int contact,
	double *parms)

{
  return (addObject(name,CUBE,contact,rgb,pos,rot,scale,parms,NULL));
}

/*!*****************************************************************************
 *******************************************************************************
\note  addSphere
\date  Nov 2000
   
\remarks 

       add a sphere to the object list

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to rotation vector
 \param[in]     contact    : ID of contact model  
 \param[in]     parms      : array of contact parameters
 \param[in]     n_faces    : number of faces


 ******************************************************************************/
ObjectPtr
addSphere(char *name, double *rgb, double *pos, double *rot, 
	  double *scale, int contact,
	  Vector parms, int n_faces)
{
  double ospecs[1+1];

  ospecs[0]=1;
  ospecs[1]=n_faces;

  return (addObject(name,SPHERE,contact,rgb,pos,rot,scale,parms,ospecs));

}

/*!*****************************************************************************
 *******************************************************************************
\note  addCylinder
\date  Nov 2000
   
\remarks 

       Add a cylinder to the object list. The axis of the cylinder is aligned
       with the local z axis

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to scale vector
 \param[in]     contact    : ID of contact model  
 \param[in]     parms      : array of contact parameters
 \param[in]     n_faces    : number of faces


 ******************************************************************************/
ObjectPtr
addCylinder(char *name, double *rgb, double *pos, double *rot, 
	  double *scale, int contact,
	  Vector parms, int n_faces)
{
  double ospecs[1+1];

  ospecs[0]=1;
  ospecs[1]=n_faces;

  return (addObject(name,CYLINDER,contact,rgb,pos,rot,scale,parms,ospecs));

}

/*!*****************************************************************************
 *******************************************************************************
\note  changeObjPosByName
\date  Nov 2000
   
\remarks 

       changes the object position by name (slower, since search is required)

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector

 ******************************************************************************/
int
changeObjPosByName(char *name, double *pos, double *rot)
{
  ObjectPtr ptr = getObjPtrByName(name);
  if (ptr == NULL) 
    return FALSE;

  changeObjPosByPtr(ptr, pos, rot);

  if (strcmp(servo_name,"task")==0)  // communicate info to other servos
    changeObjPosByNameSync(name, pos, rot);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  changeHideObjByName
\date  Nov 2000
   
\remarks 

       changes the hide status of object  by name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     hide       : TRUE/FALSE

 ******************************************************************************/
int
changeHideObjByName(char *name, int hide)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      ptr->hide = hide;

      if (strcmp(servo_name,"task")==0)  // communicate info to other servos
	changeHideObjByNameSync(name, hide);

      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjForcesByName
\date  June 2007
   
\remarks 

   returns the force and torque vector acting on an object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     f          : pointer to force vector 
 \param[in]     t          : pointer to torque vector

 ******************************************************************************/
int
getObjForcesByName(char *name, double *f, double *t)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      for (i=1;i<=N_CART; ++i) {
	f[i] = ptr->f[i];
	t[i] = ptr->t[i];
      }
      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjForcesByPtr
\date  June 2007
   
\remarks 

   returns the force and torque vector acting on an object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ptr        : pointer to object
 \param[in]     f          : pointer to force vector 
 \param[in]     t          : pointer to torque vector

 ******************************************************************************/
int
getObjForcesByPtr(ObjectPtr ptr, double *f, double *t)
{
  int i;

  for (i=1;i<=N_CART; ++i) {
    f[i] = ptr->f[i];
    t[i] = ptr->t[i];
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  getObjPtrByName
\date  Nov 2000
   
\remarks 

   returns the pointer to a specific object

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 

 ******************************************************************************/
ObjectPtr
getObjPtrByName(char *name)
{
  ObjectPtr ptr;
  int i;

  if (objs == NULL) 
    return NULL;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      return ptr;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  return NULL;
  
}

	

/*!*****************************************************************************
 *******************************************************************************
\note  changeObjPosByPtr
\date  Nov 2000
   
\remarks 

       changes the object position by pointer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ptr        : ptr of the object 
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector

 ******************************************************************************/
void
changeObjPosByPtr(ObjectPtr ptr, double *pos, double *rot)
{

  int i;
  
  for (i=1;i<=N_CART; ++i) {
    ptr->trans[i]=pos[i];
    ptr->rot[i]=rot[i];
  }
  
  if (strcmp(servo_name,"task")==0)  // communicate info to other servos
    changeObjPosByNameSync(ptr->name, pos, rot);

}

/*!*****************************************************************************
 *******************************************************************************
\note  deleteObjByName
\date  Nov 2000
   
\remarks 

       deletes an object by name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 

 ******************************************************************************/
int
deleteObjByName(char *name)
{
  ObjectPtr ptr,ptr2;
  char     *nptr;
  int i;

  if (objs == NULL) 
    return FALSE;

  ptr = objs;

  do {
    if (strcmp(name,ptr->name)==0) {
      ptr2=objs;
      if (ptr2 == ptr) {
	objs = (ObjectPtr) ptr->nptr;
      } else {
	do {
	  if (ptr2->nptr == (char *)ptr) {
	    ptr2->nptr = ptr->nptr;
	    break;
	  }
	  ptr2 = (ObjectPtr) ptr2->nptr;
	} while (ptr2 != NULL);
      }
      free(ptr);
      return TRUE;
    }
    ptr = (ObjectPtr) ptr->nptr;
  } while (ptr != NULL);

  if (strcmp(servo_name,"task")==0)  // communicate info to other servos
    deleteObjByName(name);

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  checkContacts
\date  June 1999
   
\remarks 

      checks for contacts and computes the contact forces

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
checkContacts(void)

{
  int       i,j;
  ObjectPtr optr;
  double    x[N_CART+1];
  double    aux,aux1,aux2;
  int       ind=0;
  int       first_contact_flag = FALSE;
  int       contact_flag = FALSE;
  double    z;
  double    n[N_CART+1];
  double    v[N_CART+1];
  char      tfname[100];
  double    no_go;
  double    dist_z;

  /* zero contact forces */
  bzero((void *)ucontact,sizeof(SL_uext)*(n_dofs+1));
  
  /* if there are no objects, exit */
  if (objs==NULL)
    return;
  
  // zero all object forces and torques
  optr = objs;
  do { 
    for (i=1; i<=N_CART; ++i)
      optr->f[i] = optr->t[i] = 0.0;
    optr = (ObjectPtr) optr->nptr;
  } while (optr != NULL);
  
  
  for (i=0; i<=n_contacts; ++i) { /* loop over all contact points */
    
    if (!contacts[i].active)
      continue;
    
    first_contact_flag = FALSE;
    contact_flag = FALSE;
    
    optr = objs;
    do {   /* check all objects */
      
      /* check whether this is a contact */
      if (optr->contact_model == NO_CONTACT || optr->hide) {
	optr = (ObjectPtr) optr->nptr;
	continue;
      }
      
      /* step one: transform potential contact point into object
	 coordinates */

      // compute the current contact point
      computeContactPoint(&(contacts[i]),link_pos_sim,Alink_sim,x);

      // convert to local coordinates
      for (j=1; j<=N_CART; ++j)
	x[j] -= optr->trans[j];
      
      // rotate the contact point into object centered coordinates
      convertGlobal2Object(optr, x, x);

      // is this point inside the object? 
      switch (optr->type) {
      case CUBE: //---------------------------------------------------------------
	if (fabs(x[1]) < optr->scale[1]/2. &&
	    fabs(x[2]) < optr->scale[2]/2. &&
	    fabs(x[3]) < optr->scale[3]/2.) {
	  
	  // remember which object we are contacting, and also the 
	  // contact point in object centered coordinates
	  
	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }
	  
	  // compute relevant geometric information
	  
	  if (first_contact_flag) {
	    // what is the closest face of the cube
	    if ( (optr->scale[1]/2.-fabs(x[1]) ) < (optr->scale[2]/2.-fabs(x[2]) ) &&
		 (optr->scale[1]/2.-fabs(x[1]) ) < (optr->scale[3]/2.-fabs(x[3]) )) {
	      ind = 1;
	    } else if ( (optr->scale[2]/2.-fabs(x[2]) ) < (optr->scale[1]/2.-fabs(x[1]) ) &&
			(optr->scale[2]/2.-fabs(x[2]) ) < (optr->scale[3]/2.-fabs(x[3]) )) {
	      ind = 2;
	    } else {
	      ind = 3;
	    }
	    contacts[i].face_index = ind;
	  } else {
	    ind = contacts[i].face_index;
	  }
	  
	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  
	  // the normal vector
	  for (j=1; j<=N_CART; ++j) {
	    if (j==ind) {
	      // the contact normal never change direction relativ to the start point
	      contacts[i].normal[j] = 
		optr->scale[j]/2.*macro_sign(contacts[i].x_start[j]) - x[j];
	      contacts[i].normvel[j] = -v[j];
	    } else {
	      contacts[i].normal[j] = 0.0;
	      contacts[i].normvel[j] = 0.0;
	    }
	  }
	  
	  // the tangential vector
	  for (j=1; j<=N_CART; ++j) {
	    if (j!=ind) {
	      contacts[i].tangent[j] = x[j]-contacts[i].x_start[j];
	      contacts[i].tanvel[j]  = v[j];
	    } else {
	      contacts[i].tangent[j] = 0.0;
	      contacts[i].tanvel[j]  = 0.0;
	    }
	  }
	  
	  // the tangential velocity for viscous friction
	  for (j=1; j<=N_CART; ++j) {
	    if (j!=ind) {
	      contacts[i].viscvel[j] = v[j];
	    } else {
	      contacts[i].viscvel[j]=0.0;
	    }
	  }

	  // finally apply contact models
	  computeContactForces(optr,&contacts[i]);
	  optr = NULL;
	  continue; /* only one object can be in contact with a contact point */
	  
	}
	
	break;
	
      case SPHERE: //---------------------------------------------------------------
	if ((sqr(x[1]/optr->scale[1]*2.) + sqr(x[2]/optr->scale[2]*2.) + 
	     sqr(x[3]/optr->scale[3]*2.)) < 1.0) {
	  
	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }
	  
	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  
	  /* the normal displacement vector: map the current x into the unit sphere,
	     compute the point where this vector pierces through the unit sphere,
	     and map it back to the deformed sphere. The difference between this
	     vector and x is the normal displacement */
	  aux2 = 1.e-10;
	  for (j=1; j<=N_CART; ++j) {
	    aux2 += sqr(x[j]/(optr->scale[j]/2.));
	  }
	  aux2 = sqrt(aux2); /* length in unit sphere */
	  
	  aux = 1.e-10;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].normal[j] = x[j]*(1./aux2-1.);
	    aux += sqr(contacts[i].normal[j]);
	  }
	  aux = sqrt(aux);
	  
	  // unit vector of contact normal
	  aux2 = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    n[j] = contacts[i].normal[j]/aux;
	    aux2 += n[j]*v[j];
	  }
	  
	  // the normal velocity
	  for (j=1; j<=N_CART; ++j)
	    contacts[i].normvel[j] = -aux2*n[j];
	  
	  /* the tangential vector */
	  aux1 = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j]= x[j]-contacts[i].x_start[j];
	    aux1 += n[j] * contacts[i].tangent[j];
	  }
	  
	  // subtract the all components in the direction of the normal
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j] -= n[j] * aux1;
	    contacts[i].tanvel[j]   = v[j] - n[j]*aux2;
	  }
	  
	  /* the vicous velocity */
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].viscvel[j] = v[j] - n[j]*aux2;
	  }
	  
	  computeContactForces(optr,&contacts[i]);
	  optr=NULL;
	  continue; /* only one object can be in contact with a contact point */
	  
	} 
	
	break;
	
	
      case CYLINDER: //---------------------------------------------------------------
	// the cylinder axis is aliged with te Z axis
	if ((sqr(x[1]/optr->scale[1]*2.) + sqr(x[2]/optr->scale[2]*2.)) < 1.0 &&
	    fabs(x[3]) < optr->scale[3]/2.) {
	  
	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }
	  
	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  
	  if (first_contact_flag || contacts[i].face_index != _Z_) {
	    /* the normal displacement vector: map the current x into the unit cylinder,
	       compute the point where this vector pierces through the unit cylinder,
	       and map it back to the deformed cylinder. The difference between this
	       vector and x is the normal displacement */
	    aux2 = 1.e-10;
	    for (j=1; j<=_Y_; ++j) {
	      aux2 += sqr(x[j]/(optr->scale[j]/2.));
	    }
	    aux2 = sqrt(aux2); // length in unit cylinder
	    
	    aux = 1.e-10;
	    for (j=1; j<=_Y_; ++j) {
	      contacts[i].normal[j] = x[j]*(1./aux2-1.);
	      aux += sqr(contacts[i].normal[j]);
	    }
	    contacts[i].normal[_Z_] = 0.0;
	    aux = sqrt(aux);
	    
	    // unit vector of contact normal
	    aux2 = 0.0;
	    for (j=1; j<=N_CART; ++j) {
	      n[j] = contacts[i].normal[j]/aux;
	      aux2 += n[j]*v[j];
	    }
	    
	    // the normal velocity
	    for (j=1; j<=N_CART; ++j)
	      contacts[i].normvel[j] = -aux2*n[j];
	    
	    /* the tangential vector */
	    aux1 = 0.0;
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].tangent[j]= x[j]-contacts[i].x_start[j];
	      aux1 += n[j] * contacts[i].tangent[j];
	    }
	    
	    // subtract the all components in the direction of the normal
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].tangent[j] -= n[j] * aux1;
	      contacts[i].tanvel[j]   = v[j] - n[j]*aux2;
	    }
	    
	    /* the vicous velocity */
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].viscvel[j] = v[j] - n[j]*aux2;
	    }

	  }


	  // now we check which face is the nearest to th surface
	  if (first_contact_flag) {
	    // distance from nearest cylinder flat surface
	    dist_z = optr->scale[3]/2.-fabs(x[3]);
	    
	    if (dist_z < aux) 
	      contacts[i].face_index = _Z_;
	    else
	      contacts[i].face_index = _X_; // could also choose _Y_ -- !_Z_ matters

	  }

	  // compute with the cylinder ends as contact face

	  if (contacts[i].face_index == _Z_) {
	    ind = _Z_;
	    // the normal vector
	    for (j=1; j<=N_CART; ++j) {
	      if (j==ind) {
		// the contact normal never change direction relativ to the start point
		contacts[i].normal[j] = 
		  optr->scale[j]/2.*macro_sign(contacts[i].x_start[j]) - x[j];
		contacts[i].normvel[j] = -v[j];
	      } else {
		contacts[i].normal[j] = 0.0;
		contacts[i].normvel[j] = 0.0;
	      }
	    }
	  
	    // the tangential vector
	    for (j=1; j<=N_CART; ++j) {
	      if (j!=ind) {
		contacts[i].tangent[j] = x[j]-contacts[i].x_start[j];
		contacts[i].tanvel[j]  = v[j];
	      } else {
		contacts[i].tangent[j] = 0.0;
		contacts[i].tanvel[j]  = 0.0;
	      }
	    }
	  
	    // the tangential velocity for viscous friction
	    for (j=1; j<=N_CART; ++j) {
	      if (j!=ind) {
		contacts[i].viscvel[j] = v[j];
	      } else {
		contacts[i].viscvel[j]=0.0;
	      }
	    }
	  }

	  computeContactForces(optr,&contacts[i]);
	  optr=NULL;
	  continue; /* only one object can be in contact with a contact point */

	} 

	break;

	
      case TERRAIN: //---------------------------------------------------------------
	if (!getContactTerrainInfo(x[1], x[2], optr->name, &z, n, &no_go))
	  break;

	if (x[3] < z) {
	
	  // remember which object we are contacting, and also the 
	  // contact point in object centered coordinates
	  if (!contacts[i].status || contacts[i].optr != optr ) {
	    for (j=1; j<=N_CART; ++j) {
	      contacts[i].x_start[j] = x[j];
	      contacts[i].x[j] = x[j];
	    }
	    contacts[i].friction_flag = FALSE;
	    first_contact_flag = TRUE;
	  }
	  contacts[i].status = contact_flag = TRUE;
	  contacts[i].optr   = optr;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].x[j] = x[j];
	  }

	  // the local veclocity of the contact point
	  contactVelocity(i,optr,v);
	  aux1 = n[_X_]*v[_X_]+n[_Y_]*v[_Y_]+n[_Z_]*v[_Z_];
	  
	  // compute relevant geometric information

	  // the normal vector
	  for (j=1; j<=N_CART; ++j) {
	    // note: n'*[ 0 0 (z-x[3])] = (z-x[3])*n[3] is the effective
	    // projection of the vertical distance to the surface onto
	    // the normal
	    contacts[i].normal[j]   = n[j]*((z-x[3])*n[3]);
	    contacts[i].normvel[j]  = -aux1*n[j];
	  }

	  // the tangential vector: project x-x_start into the null-space of normal
	  aux  = 0.0;
	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j]=x[j]-contacts[i].x_start[j];
	    aux  += contacts[i].tangent[j]*n[j];
	  }

	  for (j=1; j<=N_CART; ++j) {
	    contacts[i].tangent[j] -= n[j]*aux;
	    contacts[i].tanvel[j]   = v[j]-n[j]*aux1;
	  }

	  // the tangential velocity for viscous friction
	  for (j=1; j<=N_CART; ++j)
	    contacts[i].viscvel[j] = v[j]-n[j]*aux1;

	  computeContactForces(optr,&contacts[i]);
	  optr = NULL;
	  continue; /* only one object can be in contact with a contact point */

	}

	break;

      }

      optr = (ObjectPtr) optr->nptr;

    } while (optr != NULL);

    contacts[i].status = contact_flag;

    if (!contacts[i].status) { // this is just for easy data interpretation
      for (j=1; j<=N_CART; ++j) {
	contacts[i].normal[j] = 0.0;
	contacts[i].normvel[j] = 0.0;
	contacts[i].tangent[j] = 0.0;
	contacts[i].tanvel[j] = 0.0;
	contacts[i].viscvel[j] = 0.0;
	contacts[i].f[j] = 0.0;
	contacts[i].n[j] = 0.0;
      }
    }

  }

  /* add simulated external forces to contact forces */
  for (i=0; i<=n_dofs; ++i)
    for (j=_X_; j<=_Z_; ++j) {
      ucontact[i].f[j] += uext_sim[i].f[j];
      ucontact[i].t[j] += uext_sim[i].t[j];
    }

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeStart2EndNorm
\date  Aug 2010
   
\remarks 

 computes the norm vector from start to end point of line in object centered
 coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]   xs  : start point of line (global coordinates)
 \param[in]   xe  : end point of line (global coordinates)
 \param[in]   optr: point to object
 \param[out]  n   : norm vector from start to end

 note: xl and xg can be the same pointers

 ******************************************************************************/
static void
computeStart2EndNorm(double *xs, double *xe, ObjectPtr optr, double *n)
{
  int i,j;
  double x[N_CART+1];
  double aux = 0;

  // difference vector pointing from start to end
  for (i=1; i<=N_CART; ++i) {
    x[i] = xe[i] - xs[i];
    aux += sqr(x[i]);
  }
  aux = sqrt(aux);

  // normalize
  for (i=1; i<=N_CART; ++i)
    x[i] /= (aux + 1.e-10);

  // convert to local coordinates
  convertGlobal2Object(optr, x, n);

}

/*!*****************************************************************************
 *******************************************************************************
\note  convertGlobal2Object
\date  Aug 2010
   
\remarks 

rotates global coordinates to object coordinates (no translation). This uses
Euler angle notation x-y-z rotations

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]   optr: point to object
 \param[in]   xg  : vector in global coordinates
 \param[out]  xl  : vector in local coordinates

 note: xl and xg can be the same pointers

 ******************************************************************************/
static void
convertGlobal2Object(ObjectPtr optr, double *xg, double *xl)

{
  int    i;
  double aux;

  for (i=1; i<=N_CART; ++i)
    xl[i] = xg[i];

  // the link point in object centered coordinates
  if (optr->rot[1] != 0.0) {
    aux  =  xl[2]*cos(optr->rot[1])+xl[3]*sin(optr->rot[1]);
    xl[3] = -xl[2]*sin(optr->rot[1])+xl[3]*cos(optr->rot[1]);
    xl[2] = aux;
  }
  
  if (optr->rot[2] != 0.0) {
    aux  =  xl[1]*cos(optr->rot[2])-xl[3]*sin(optr->rot[2]);
    xl[3] =  xl[1]*sin(optr->rot[2])+xl[3]*cos(optr->rot[2]);
    xl[1] = aux;
  }
  
  if (optr->rot[3] != 0.0) {
    aux  =  xl[1]*cos(optr->rot[3])+xl[2]*sin(optr->rot[3]);
    xl[2] = -xl[1]*sin(optr->rot[3])+xl[2]*cos(optr->rot[3]);
    xl[1] = aux;
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  projectForce
\date  Aug 2010
   
\remarks 

 projects a force vector onto the space that is defined by the information in
 the contact structure. Note that contact points coinciding with links are
 processed differently than contact points that interpolate between links.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out]   cptr  : pointer to contact structure
 \param[in]       optr  : pointer to object structure

 returns cptr->f adjusted according to the projection information

 ******************************************************************************/
static void
projectForce(ContactPtr cptr, ObjectPtr optr)
{
  int    i,j;
  double aux;
  double nv[N_CART+1];
  double sf[N_CART+1];
  double nnv[N_CART+1];

  /***********************************************************************************************/
  if (cptr->n_connected_links == 0) { // this is a interpolated contact point or a special contact point

    if (cptr->point_contact_flag) { // for special contact points
      double xs[N_CART+1];
      double xe[N_CART+1];

      for (i=1; i<=N_CART; ++i) {
	xs[i] = Alink_sim[cptr->id_start][i][4];
	xe[i] = Alink_sim[cptr->id_start][i][4];
	for (j=1; j<=N_CART; ++j) {
	  xs[i] += Alink_sim[cptr->id_start][i][j]*cptr->local_point_pos[j];
	  xe[i] += Alink_sim[cptr->id_start][i][j]*(cptr->local_point_pos[j]+cptr->local_point_norm[j]);
	}
      }

      // compute norm vector from start to end of line
      computeStart2EndNorm(xs,xe,optr,nv);

      // act according to contact condition, stored in force_condition[1] for point_contacts
      switch (cptr->force_condition[1]) {
	
      case F_FULL: // no projection needed 
	break;
	
      case F_HALF: // project away forces pointing in the same direction of norm
	
	// inner product norm with contact force
	aux = 0.0;
	for (j=1; j<=N_CART; ++j)
	  aux  += cptr->f[j]*nv[j];

	if (aux < 0)
	  aux = 0;

	// subtract this component out
	for (j=1; j<=N_CART; ++j)
	  cptr->f[j] -= aux * nv[j];

	break;

      case F_ORTH: // only allow force components orthogonal to norm

	// inner product norm with contact force
	aux = 0.0;
	for (j=1; j<=N_CART; ++j)
	  aux  += cptr->f[j]*nv[j];

	// subtract this component out
	for (j=1; j<=N_CART; ++j)
	  cptr->f[j] -= aux * nv[j];

	break;

      default:
	printf("Unknown force condition\n");

      } // switch(cptr->force_condition)

    } else {  // for interpolated line contact points

      // compute norm vector from start to end of line
      computeStart2EndNorm(link_pos_sim[cptr->id_start],
			   link_pos_sim[cptr->id_end],			 
			   optr,nv);
      
      // inner product norm with contact force
      aux = 0.0;
      for (j=1; j<=N_CART; ++j)
	aux  += cptr->f[j]*nv[j];
      
      // subtract this component out
      for (j=1; j<=N_CART; ++j)
	cptr->f[j] -= aux * nv[j];
      
    }


  /***********************************************************************************************/
  } else { // this is a contact point coinciding with a link point

    // loop over all connected links, and compute average contact force
    // from all projections
    
    for (j=1; j<=N_CART; ++j)
      sf[j] = 0.0;

    for (i=1; i<=cptr->n_connected_links; ++i) {

      switch (cptr->force_condition[i]) {

      case F_FULL: // no projection needed 

	for (j=1; j<=N_CART; ++j)
	  sf[j] += cptr->f[j];

	break;

      case F_HALF: // project away force aligned with line, pointing out of line

	// compute norm vector from start to end of line
	computeStart2EndNorm(link_pos_sim[cptr->connected_links[i]],
			     link_pos_sim[cptr->id_start],			 
			     optr,nv);

	// inner product norm with contact force
	aux = 0.0;
	for (j=1; j<=N_CART; ++j)
	  aux  += cptr->f[j]*nv[j];

	if (aux < 0)
	  aux = 0;

	// subtract this component out
	for (j=1; j<=N_CART; ++j)
	  sf[j] += cptr->f[j] - aux * nv[j];

	break;

      case F_ORTH: // only allow force components orthogonal to line

	// compute norm vector from start to end of line
	computeStart2EndNorm(link_pos_sim[cptr->connected_links[i]],
			     link_pos_sim[cptr->id_start],			 
			     optr,nv);

	// inner product norm with contact force
	aux = 0.0;
	for (j=1; j<=N_CART; ++j)
	  aux  += cptr->f[j]*nv[j];

	// subtract this component out
	for (j=1; j<=N_CART; ++j)
	  sf[j] += cptr->f[j] - aux * nv[j];

	break;

      default:
	printf("Unknown force condition\n");

      } // switch(cptr->force_condition)

    } // for (i=1; i<=cptr->n_connected_links; ++i)


    // compute average of accumulated force
    for (j=1; j<=N_CART; ++j)
      cptr->f[j] = sf[j]/(double)(cptr->n_connected_links);

  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  readObjects
\date  June 2006
\remarks 

Parses an arbitrary objects file. Objects are defined by multiple lines, given 
as in the following example:

floor 1 		( name and ID of object type )
1.0 0.0 0.0          	( rgb color specification)
0.0 0.0 -1.65  		( 3D position of center of object )
0.0 0.0 0.0  		( x-y-z Euler angle orientation )
40.0 40.0 1.0  		( scale applied to x, y, z of object)
2  			( contact model: see computeContactForces() below )
10         		( object parameters: see SL_openGL.c: drawObjects() )
1000 30 1000 30 0.5 0.4	( contact parameters: see computeContactForces() below )

The number of object parameters and contact parameters can vary for
different objects and different contact models. The parsing of these
parameters is pretty rudimentary, and simply all lines are expected in
the correct sequence, and the "new-line" character is interpreted as the
end of line indicator.


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     cfname : file name of objects file

 ******************************************************************************/
void
readObjects(char *cfname) 

{
  int j, i, ID;
  FILE  *in;
  double dum;
  double rgb[N_CART+1];
  double pos[N_CART+1];
  double rot[N_CART+1];
  double scale[N_CART+1];
  int    contact;
  int    objtype;
  char   name[100];
  char   fname[100];
  char   name2[100];
  double oparms[MAX_OBJ_PARMS+1];
  double cparms[MAX_CONTACT_PARMS+1];
  double display_grid_delta;
  ObjectPtr optr;
  double x,y,z;
  double n[N_CART+1];
  char   string[STRING100];
  static int objects_read = FALSE;

  if (objects_read)
    printf("Re-initializing objects from file >%s%s<\n",CONFIG,cfname);

  
  sprintf(string,"%s%s",CONFIG,cfname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return;
  }

  /* find objects in file */
  while (fscanf(in,"%s %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d",
		name,&objtype,&rgb[1],&rgb[2],&rgb[3],
		&pos[1],&pos[2],&pos[3],&rot[1],&rot[2],&rot[3],
		&scale[1],&scale[2],&scale[3],&contact) == 15) {

    while (fgetc(in) != '\n')
      ;
    i=0;
    while ((string[i++]=fgetc(in))!= '\n' && i<499 )
      ;
    string[i]='\0';
    i=sscanf(string,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	     &oparms[1],&oparms[2],&oparms[3],&oparms[4],&oparms[5],
      	     &oparms[6],&oparms[7],&oparms[8],&oparms[9],&oparms[10],
	     &oparms[11],&oparms[12],&oparms[13],&oparms[14],&oparms[15],
	     &oparms[16],&oparms[17],&oparms[18],&oparms[19],&oparms[20],
	     &oparms[21],&oparms[22],&oparms[23],&oparms[24],&oparms[25],
	     &oparms[26],&oparms[27],&oparms[28],&oparms[29],&oparms[30]);
    oparms[0] = i;

    i=0;
    while ((string[i++]=fgetc(in)) != '\n' && i<499)
      ;
    string[i]='\0';
    i=sscanf(string,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	     &cparms[1],&cparms[2],&cparms[3],&cparms[4],&cparms[5],
      	     &cparms[6],&cparms[7],&cparms[8],&cparms[9],&cparms[10],
	     &cparms[11],&cparms[12],&cparms[13],&cparms[14],&cparms[15],
	     &cparms[16],&cparms[17],&cparms[18],&cparms[19],&cparms[20],
	     &cparms[21],&cparms[22],&cparms[23],&cparms[24],&cparms[25],
	     &cparms[26],&cparms[27],&cparms[28],&cparms[29],&cparms[30]);
    cparms[0] = i;

    if (objtype == TERRAIN) { // terrains use .asc names
      sprintf(name2,"%s.asc",name);
      strcpy(name,name2);
    }
    optr = addObject(name,objtype,contact,rgb,pos,rot,scale,cparms,oparms);
    
    /* if there is a floor, initialize the gound level for the terrains */
    if (strcmp("floor",name) == 0) {
      setTerrainGroundZ(optr->trans[_Z_] + 0.5*optr->scale[_Z_]);
      printf("\nFound object >floor< and intialized ground level to %f\n",
	     optr->trans[_Z_] + 0.5*optr->scale[_Z_]);
    }
    
    // for terrains we need to add the terrain and create a display list for this terrain
    if (objtype == TERRAIN) {
      if ((ID=getNextTerrainID())) { 
	SL_quat q;
	
	eulerToQuat(rot,&q);
	// Note: oparms[1] = reg_rad oparms[2]=reg_down oparms[3]=reg_crad oparms[4]=disp_grid_delta
	setTerrainInfo(ID,ID,name,pos,q.q,
		       (int)rint(oparms[1]),(int)rint(oparms[2]),(int)rint(oparms[3]));
	
      } else {
	printf("No more terrains possible -- increase MAX_TERRAINS\n");
      }
    }
    
  }
  
  fclose(in);
  remove_temp_file();

  objects_read = TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeContactForces
\date  Dec.2000
   
\remarks 

      for a given contact point and object, compute the contact forces
      and add them to the appropriate structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     optr  : ptr to object
 \param[in]     ctpr  : ptr to contact point

 ******************************************************************************/
static void
computeContactForces(ObjectPtr optr, ContactPtr cptr)

{
  int i,j;
  double aux;
  double temp[N_CART+1];
  double temp1[N_CART+1];
  double temp2[N_CART+1];
  double moment_arm[N_CART+1];
  double moment_arm_object[N_CART+1];
  double normal_force;
  double normal_velocity;
  double tangent_force;
  double tangent_velocity;
  double viscvel;
  int    option=0;

  /* compute the contact forces in object centered coordinates */
  switch (optr->contact_model) {

  case DAMPED_SPRING_STATIC_FRICTION:
    
    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = static friction spring coefficient
       contact_parms[4] = static friction damping spring coefficient
       contact_parms[5] = static friction coefficient (friction cone)
       contact_parms[6] = dynamic friction coefficient (proportional to normal force)
    */
    
    // the normal contact force in object centered coordinates
    normal_force = 0;
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = optr->contact_parms[1] * cptr->normal[i] +
	optr->contact_parms[2] * cptr->normvel[i];
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0)
	cptr->f[i] = 0.0;
      
      normal_force += sqr(cptr->f[i]);
    }
    normal_force = sqrt(normal_force);

    // project the spring force according to the information in the contact and object structures
    projectForce(cptr,optr);
    
    // the force due to static friction, modeled as horizontal damper, again
    // in object centered coordinates
    tangent_force = 0;
    viscvel = 1.e-10;
    for (i=1; i<=N_CART; ++i) {
      temp[i] = -optr->contact_parms[3] * cptr->tangent[i] - 
	optr->contact_parms[4]*cptr->tanvel[i];
      tangent_force += sqr(temp[i]);
      viscvel += sqr(cptr->viscvel[i]);
    }
    tangent_force = sqrt(tangent_force);
    viscvel = sqrt(viscvel);

    /* If static friction too large -> spring breaks -> dynamic friction in
       the direction of the viscvel vector; we also reset the x_start
       vector such that static friction would be triggered appropriately,
       i.e., when the viscvel becomes zero */
    if (tangent_force > optr->contact_parms[5] * normal_force || cptr->friction_flag) {
      cptr->friction_flag = TRUE;
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += -optr->contact_parms[6] * normal_force * cptr->viscvel[i]/viscvel;
	if (viscvel < 0.01) {
	  cptr->friction_flag = FALSE;
	  cptr->x_start[i] = cptr->x[i];
	}
      } 
    } else {
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += temp[i];
      }
    }
    break;

  case DAMPED_SPRING_VISCOUS_FRICTION:

    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = viscous friction coefficient
    */

    /* normal contact force */
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = optr->contact_parms[1] * cptr->normal[i] +
	optr->contact_parms[2] * cptr->normvel[i];
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0)
	cptr->f[i] = 0.0;
    }
    
    // project the spring force according to the information in the contact and object structures
    projectForce(cptr,optr);
    
    /* the force due to viscous friction */
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] += -optr->contact_parms[3] * cptr->viscvel[i];
    }
    break;

  case DAMPED_SPRING_LIMITED_REBOUND:
    
    /* note: the contact_parms arrays has the following elements:
       contact_parms[1] = normal spring coefficient
       contact_parms[2] = normal damping coefficient
       contact_parms[3] = static friction spring coefficient
       contact_parms[4] = static friction damping spring coefficient
       contact_parms[5] = static friction coefficient (friction cone)
       contact_parms[6] = dynamic friction coefficient (proportional to normal force)
       contact_parms[7] = normal max rebound velocity
       contact_parms[8] = static friction max rebound velocity
    */
    
    // the normal contact force in object centered coordinates
    normal_velocity = 0.0;
    aux = 0;
    for (i=1; i<=N_CART; ++i) {
      temp1[i] = optr->contact_parms[1] * cptr->normal[i];
      temp2[i] = optr->contact_parms[2] * cptr->normvel[i];
      normal_velocity  += sqr(cptr->normvel[i]);
      aux              += cptr->normvel[i]*cptr->normal[i];
    }
    normal_velocity = -sqrt(normal_velocity)*macro_sign(aux);
    aux = 1.0-normal_velocity/optr->contact_parms[7];
    if (aux < 0)
      aux = 0.0;

    normal_force = 0;
    for (i=1; i<=N_CART; ++i) {
      cptr->f[i] = temp1[i]*aux + temp2[i]*sqrt(aux);
      // make sure the damping part does not attract a contact force with wrong sign
      if (macro_sign(cptr->f[i])*macro_sign(cptr->normal[i])<0) 
	cptr->f[i] = 0.0;
      normal_force     += sqr(cptr->f[i]);
    }
    normal_force = sqrt(normal_force);

    // project the spring force according to the information in the contact and object structures
    projectForce(cptr,optr);

    // the force due to static friction, modeled as horizontal damper, again
    // in object centered coordinates
    tangent_force = 0;
    tangent_velocity = 0;
    viscvel = 1.e-10;
    aux = 0;
    for (i=1; i<=N_CART; ++i) {
      temp1[i] = -optr->contact_parms[3] * cptr->tangent[i];
      temp2[i] = -optr->contact_parms[4]*cptr->tanvel[i];
      tangent_velocity += sqr(cptr->tanvel[i]);
      viscvel          += sqr(cptr->viscvel[i]);
      aux              += (cptr->x[i]-cptr->x_start[i])*cptr->tanvel[i];
    }
    tangent_velocity = -sqrt(tangent_velocity)*macro_sign(aux);
    viscvel          = sqrt(viscvel);

    aux = 1.0-tangent_velocity/optr->contact_parms[8];
    if (aux < 0)
      aux = 0.0;

    tangent_force = 0.0;
    for (i=1; i<=N_CART; ++i) {
      temp[i] = temp1[i]*aux + temp2[i]*sqrt(aux);
      tangent_force    += sqr(temp[i]);
    }
    tangent_force    = sqrt(tangent_force);

    /* If static friction too large -> spring breaks -> dynamic friction in
       the direction of the viscvel vector; we also reset the x_start
       vector such that static friction would be triggered appropriately,
       i.e., when the viscvel becomes zero */
    if (tangent_force > optr->contact_parms[5] * normal_force || cptr->friction_flag) {
      cptr->friction_flag = TRUE;
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += -optr->contact_parms[6] * normal_force * cptr->viscvel[i]/viscvel;
	if (viscvel < 0.01) {
	  cptr->friction_flag = FALSE;
	  cptr->x_start[i] = cptr->x[i];
	}
      } 
    } else {
      for (i=1; i<=N_CART; ++i) {
	cptr->f[i] += temp[i];
      }
    }
    break;

  default:
    break;

  }

  /* convert the object centered forces and normals into global coordinates */

  // assign the normal
  aux = 0.0;
  for (i=1; i<=N_CART; ++i)  {
    cptr->n[i] = cptr->normal[i];
    aux += sqr(cptr->n[i]);
  }
  aux = sqrt(aux);
  for (i=1; i<=N_CART; ++i)
    cptr->n[i] /= aux;


  if (optr->rot[_G_] != 0.0) {
    aux    =  cptr->f[_X_]*cos(optr->rot[_G_])-cptr->f[_Y_]*sin(optr->rot[_G_]);
    cptr->f[_Y_] =  cptr->f[_X_]*sin(optr->rot[_G_])+cptr->f[_Y_]*cos(optr->rot[_G_]);
    cptr->f[_X_] = aux;

    aux          =  cptr->n[_X_]*cos(optr->rot[_G_])-cptr->n[_Y_]*sin(optr->rot[_G_]);
    cptr->n[_Y_] =  cptr->n[_X_]*sin(optr->rot[_G_])+cptr->n[_Y_]*cos(optr->rot[_G_]);
    cptr->n[_X_] = aux;
  }

  if (optr->rot[_B_] != 0.0) {
    aux    =  cptr->f[_X_]*cos(optr->rot[_B_])+cptr->f[_Z_]*sin(optr->rot[_B_]);
    cptr->f[_Z_] = -cptr->f[_X_]*sin(optr->rot[_B_])+cptr->f[_Z_]*cos(optr->rot[_B_]);
    cptr->f[_X_] = aux;

    aux    =  cptr->n[_X_]*cos(optr->rot[_B_])+cptr->n[_Z_]*sin(optr->rot[_B_]);
    cptr->n[_Z_] = -cptr->n[_X_]*sin(optr->rot[_B_])+cptr->n[_Z_]*cos(optr->rot[_B_]);
    cptr->n[_X_] = aux;
  }
  
  if (optr->rot[_A_] != 0.0) {
    aux    =  cptr->f[_Y_]*cos(optr->rot[_A_])-cptr->f[_Z_]*sin(optr->rot[_A_]);
    cptr->f[_Z_] =  cptr->f[_Y_]*sin(optr->rot[_A_])+cptr->f[_Z_]*cos(optr->rot[_A_]);
    cptr->f[_Y_] = aux;

    aux    =  cptr->n[_Y_]*cos(optr->rot[_A_])-cptr->n[_Z_]*sin(optr->rot[_A_]);
    cptr->n[_Z_] =  cptr->n[_Y_]*sin(optr->rot[_A_])+cptr->n[_Z_]*cos(optr->rot[_A_]);
    cptr->n[_Y_] = aux;
  }


  /* add forces to appropriate DOFs and object */

  /* first the start link */
  for (i=1; i<=N_CART; ++i) {
    ucontact[cptr->base_dof_start].f[i] += cptr->f[i]*cptr->fraction_start;
    optr->f[i] += cptr->f[i]*cptr->fraction_start;
    moment_arm[i] = link_pos_sim[cptr->id_start][i]-link_pos_sim[cptr->off_link_start][i];
    moment_arm_object[i] = link_pos_sim[cptr->id_start][i]-optr->trans[i];
  }

  /* get the torque at the DOF from the cross product */
  ucontact[cptr->base_dof_start].t[_A_] += moment_arm[_Y_]*cptr->f[_Z_]*cptr->fraction_start - 
    moment_arm[_Z_]*cptr->f[_Y_]*cptr->fraction_start;
  ucontact[cptr->base_dof_start].t[_B_] += moment_arm[_Z_]*cptr->f[_X_]*cptr->fraction_start - 
    moment_arm[_X_]*cptr->f[_Z_]*cptr->fraction_start;
  ucontact[cptr->base_dof_start].t[_G_] += moment_arm[_X_]*cptr->f[_Y_]*cptr->fraction_start - 
    moment_arm[_Y_]*cptr->f[_X_]*cptr->fraction_start;

  /* get the torque at the object center from the cross product */
  optr->t[_A_] += moment_arm_object[_Y_]*cptr->f[_Z_]*cptr->fraction_start - 
    moment_arm_object[_Z_]*cptr->f[_Y_]*cptr->fraction_start;
  optr->t[_B_] += moment_arm_object[_Z_]*cptr->f[_X_]*cptr->fraction_start - 
    moment_arm_object[_X_]*cptr->f[_Z_]*cptr->fraction_start;
  optr->t[_G_] += moment_arm_object[_X_]*cptr->f[_Y_]*cptr->fraction_start - 
    moment_arm_object[_Y_]*cptr->f[_X_]*cptr->fraction_start;

  /* second the end link */
  for (i=1; i<=N_CART; ++i) {
    ucontact[cptr->base_dof_end].f[i] += cptr->f[i]*cptr->fraction_end;
    optr->f[i] += cptr->f[i]*cptr->fraction_end;
    moment_arm[i] = link_pos_sim[cptr->id_end][i]-link_pos_sim[cptr->off_link_end][i];
    moment_arm_object[i] = link_pos_sim[cptr->id_end][i]-optr->trans[i];
  }
  
  /* get the torque at the DOF from the cross product */
  ucontact[cptr->base_dof_end].t[_A_] += moment_arm[_Y_]*cptr->f[_Z_]*cptr->fraction_end - 
    moment_arm[_Z_]*cptr->f[_Y_]*cptr->fraction_end;
  ucontact[cptr->base_dof_end].t[_B_] += moment_arm[_Z_]*cptr->f[_X_]*cptr->fraction_end - 
    moment_arm[_X_]*cptr->f[_Z_]*cptr->fraction_end;
  ucontact[cptr->base_dof_end].t[_G_] += moment_arm[_X_]*cptr->f[_Y_]*cptr->fraction_end - 
    moment_arm[_Y_]*cptr->f[_X_]*cptr->fraction_end;
  
  /* get the torque at the object center from the cross product */
  optr->t[_A_] += moment_arm_object[_Y_]*cptr->f[_Z_]*cptr->fraction_end - 
    moment_arm_object[_Z_]*cptr->f[_Y_]*cptr->fraction_end;
  optr->t[_B_] += moment_arm_object[_Z_]*cptr->f[_X_]*cptr->fraction_end - 
    moment_arm_object[_X_]*cptr->f[_Z_]*cptr->fraction_end;
  optr->t[_G_] += moment_arm_object[_X_]*cptr->f[_Y_]*cptr->fraction_end - 
    moment_arm_object[_Y_]*cptr->f[_X_]*cptr->fraction_end;

}

/*!*****************************************************************************
 *******************************************************************************
\note  contactVelocity
\date  March 2006
   
\remarks 

computes the velocity of a contact point in object coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     cID    : ID of contact point
 \param[in]     optr   : object pointer
 \param[out]    v      : velocity vector

 ******************************************************************************/
static void 
contactVelocity(int cID, ObjectPtr optr, double *v)
{
  int    i;
  double aux;
  double v_start[N_CART+1];
  double v_end[N_CART+1];

  // get the velocity in world coordinates
  computeLinkVelocity(contacts[cID].id_start, link_pos_sim, joint_origin_pos_sim, 
		      joint_axis_pos_sim, joint_sim_state, v_start);
  computeLinkVelocity(contacts[cID].id_end, link_pos_sim, joint_origin_pos_sim, 
		      joint_axis_pos_sim, joint_sim_state, v_end);

  for (i=1; i<=N_CART; ++i)
    v[i] = v_start[i]*contacts[cID].fraction_start + v_end[i]*contacts[cID].fraction_end;

  // convert the velocity to object coordinates
  if (optr->rot[1] != 0.0) {
    aux  =  v[2]*cos(optr->rot[1])+v[3]*sin(optr->rot[1]);
    v[3] = -v[2]*sin(optr->rot[1])+v[3]*cos(optr->rot[1]);
    v[2] = aux;
  }
  
  if (optr->rot[2] != 0.0) {
    aux  =  v[1]*cos(optr->rot[2])-v[3]*sin(optr->rot[2]);
    v[3] =  v[1]*sin(optr->rot[2])+v[3]*cos(optr->rot[2]);
    v[1] = aux;
  }
  
  if (optr->rot[3] != 0.0) {
    aux  =  v[1]*cos(optr->rot[3])+v[2]*sin(optr->rot[3]);
    v[2] = -v[1]*sin(optr->rot[3])+v[2]*cos(optr->rot[3]);
    v[1] = aux;
  }

}



/*!*****************************************************************************
 *******************************************************************************
\note  addObjectSync
\date  May 2010
   
\remarks 

synchronizes adding of an object through shared memory communication

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     type       : type of object
 \param[in]     contact    : ID of contact model  
 \param[in]     rgb        : rgb values for color
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector
 \param[in]     scale      : pointer to rotation vector
 \param[in]     cparms     : array of contact parameters
 \param[in]     oparms     : array of object parameters

 ******************************************************************************/
static void
addObjectSync(char *name, int type, int contact, double *rgb, double *pos, double *rot, 
	      double *scale, double *cparms, double *oparms)
{
  int i,j;
  int n_objs_parm;
  int n_c_parm;
  struct {
    char    name[STRING100];                   /*!< object name */
    int     type;                              /*!< object type */
    double  trans[N_CART+1];                   /*!< translatory offset of object */
    double  rot[N_CART+1];                     /*!< rotational offset of object */
    double  scale[N_CART+1];                   /*!< scaling in x,y,z */
    double  rgb[N_CART+1];                     /*!< color information */
    double  object_parms[MAX_OBJ_PARMS+1];     /*!< object parameters */
    int     contact_model;                     /*!< which contact model to be used */
    double  contact_parms[MAX_CONTACT_PARMS+1];/*!< contact parameters */
  } data;
  unsigned char cbuf[sizeof(data)];

  strcpy(data.name,name);
  data.type = type;
  for (i=1; i<=N_CART; ++i) {
    data.trans[i] = pos[i];
    data.rot[i] = rot[i];
    data.scale[i] = scale[i];
    data.rgb[i] = rgb[i];
  }

  switch (type) {
  case CUBE:
    n_objs_parm = N_CUBE_PARMS;
    break;

  case SPHERE:
    n_objs_parm = N_SPHERE_PARMS;
    break;

  case TERRAIN:
    n_objs_parm = N_TERRAIN_PARMS;
    break;

  case CYLINDER:
    n_objs_parm = N_CYLINDER_PARMS;
    break;

  default:
    n_objs_parm = 0;
  }

  for (i=1; i<=n_objs_parm; ++i)
    data.object_parms[i] = oparms[i];
  data.object_parms[0] = n_objs_parm;
  

  switch (contact) {
  case NO_CONTACT:
    n_c_parm = N_NO_CONTACT_PARMS;
    break;

  case DAMPED_SPRING_STATIC_FRICTION:
    n_c_parm = N_DAMPED_SPRING_STATIC_FRICTION_PARMS;
    break;

  case DAMPED_SPRING_VISCOUS_FRICTION:
    n_c_parm = N_DAMPED_SPRING_VISCOUS_FRICTION_PARMS;
    break;

  case DAMPED_SPRING_LIMITED_REBOUND:
    n_c_parm = N_DAMPED_SPRING_LIMITED_REBOUND_PARMS;
    break;

  default:
    n_c_parm = 0;
  }

  for (i=1; i<=n_c_parm; ++i)
    data.contact_parms[i] = cparms[i];
  data.contact_parms[0] = n_c_parm;

  data.contact_model = contact;

  memcpy(cbuf,(void *)&data,sizeof(data));
    
  sendMessageSimulationServo("addObject",(void *)cbuf,sizeof(data));
  sendMessageOpenGLServo("addObject",(void *)cbuf,sizeof(data));

}

/*!*****************************************************************************
 *******************************************************************************
\note  changeObjPosByNameSync
\date  May 2010
   
\remarks 

synchronizes change of object position by name on all relevant processes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name       : name of the object 
 \param[in]     pos        : pointer to position vector 
 \param[in]     rot        : pointer to rotation vector

 ******************************************************************************/
static void
changeObjPosByNameSync(char *name, double *pos, double *rot)
{
  int i,j;
  struct {
    char   obj_name[100];
    double pos[N_CART+1];
    double rot[N_CART+1];
  } data;
  unsigned char buf[sizeof(data)];
  static double last_update_pos_time = 0;


  strcpy(data.obj_name,name);
  for (i=1; i<=N_CART; ++i) {
    data.pos[i] = pos[i];
    data.rot[i] = rot[i];
  }

  memcpy(buf,&data,sizeof(data));
  
  if (servo_time - last_update_pos_time > 1./60.) { // 60Hz update is sufficient
    sendMessageOpenGLServo("changeObjectPos",(void *)buf,sizeof(data));
    last_update_pos_time = servo_time;
  }
  sendMessageSimulationServo("changeObjectPos",(void *)buf,sizeof(data));

}

/*!*****************************************************************************
 *******************************************************************************
\note  changeHideObjByNameSync
\date  Nov. 2005
   
\remarks 

synchronizes hiding of objects across processes

 *******************************************************************************
[ Function Parameters: [in]=input,[out]=output

 \param[in]     name : name object
 \param[in]     hide : TRUE/FALSE

 ******************************************************************************/
static void 
changeHideObjByNameSync(char *name, int hide) 
{
  int i,j;
  struct {
    int  hide;
    char obj_name[100];
  } data;
  unsigned char buf[sizeof(data)];

  data.hide = hide;
  strcpy(data.obj_name,name);

  memcpy(buf,&data,sizeof(data));
  sendMessageOpenGLServo("hideObject",(void *)buf,sizeof(data));
  sendMessageSimulationServo("hideObject",(void *)buf,sizeof(data));
}


/*!*****************************************************************************
 *******************************************************************************
\note  deleteObjByNameSync
\date  May 2010
   
\remarks 

synchronizes delete of an object on all servos

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name : name object

 ******************************************************************************/
static void 
deleteObjByNameSync(char *name) 
{
  int i,j;
  struct {
    char obj_name[100];
  } data;
  unsigned char buf[sizeof(data)];

  strcpy(data.obj_name,name);

  memcpy(buf,&data,sizeof(data));
  sendMessageOpenGLServo("deleteObject",(void *)buf,sizeof(data));
  sendMessageSimulationServo("deleteObject",(void *)buf,sizeof(data));
}


/*!*****************************************************************************
 *******************************************************************************
\note  read_extra_contact_points
\date  July 2010
\remarks 

parses from the appropriate contact configuration file the extra contact
point specifications. The contacts data structure is assumed to exist and
to have all memory allocated. Moreover, the first n_links components need to
be assigned correctly, which is achieved from including the LEKin_contacts.h,
which also calls read_extra_contact_points.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : file name of file where config files are stored
 
 ******************************************************************************/
int
read_extra_contact_points(char *fname) 

{
  int    j,i,rc;
  char   string[100];
  FILE  *in;
  int    count;
  char   name1[100],name2[100];
  int    id1=-999,id2=-999;
  char   fcond1[100],fcond2[100];
  int    fcond1ID,fcond2ID;
  int    n_checks;
  int    active;
  int    f_full_flag;
  double lpos[N_CART+1]={0.0,0.0,0.0,0.0};
  double lnorm[N_CART+1]={0.0,0.0,0.0,0.0};
  int    point_contact_flag;
  double aux;

  // open file and strip comments 
  sprintf(string,"%s%s",CONFIG,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // read the file until EOF
  count = n_links;
  while (TRUE) {
    point_contact_flag = FALSE;
    rc = fscanf(in,"%s %s %d %d %s %s",name1,name2,&active,&n_checks,fcond1,fcond2);
    if (rc == 6) { // add the appropriate number of contact points

      if (!active)
	continue;

      // check whether this is a point contact 
      if (strcmp("POINT_CONTACT",name2) == 0) {

	// read the local position vector and norm vector
	rc = fscanf(in,"%lf %lf %lf %lf %lf %lf",&(lpos[_X_]),&(lpos[_Y_]),&(lpos[_Z_]),&(lnorm[_X_]),&(lnorm[_Y_]),&(lnorm[_Z_]));
	if (rc != 6) {
	  printf("Problems reading position and norm vector for contact point >%s< >%s<\n",name1,name2);
	  continue;
	}

	point_contact_flag = TRUE;

	// make sure that the norm vector is unit length
	aux = sqrt(sqr(lnorm[_X_])+sqr(lnorm[_Y_])+sqr(lnorm[_Z_]));
	for (i=1; i<=N_CART; ++i)
	  lnorm[i] /= (aux+1.e-10);

	// overwrite some of the contact variables to ensure they are right
	strcpy(name2,name1); // in line-contact notation, start and end point are the same for contact points
	
	n_checks = 1; // only one check permitted as this is just a point

	strcpy(fcond2,"null"); // to indicate that only the start point will create a force

      }

      // convert link names into IDs
      for (i=0; i<=n_links; ++i)
	if (strcmp(link_names[i],name1) == 0) {
	  id1 = i;
	  break;
	}

      for (i=0; i<=n_links; ++i)
	if (strcmp(link_names[i],name2) == 0) {
	  id2 = i;
	  break;
	}

      if (id1 == -999) {
	printf(">%s< does not seem to be a valid link name\n",name1);
	continue;
      }

      if (id2 == -999) {
	printf(">%s< does not seem to be a valid link name\n",name2);
	continue;
      }

      // set these two link points as active
      contacts[id1].active = active;
      contacts[id2].active = active;

      // parse the force condition
      fcond1ID = F_FULL;
      fcond2ID = F_FULL;
      for (i=1; i<=N_FORCE_CONDITIONS; ++i) {
	if (strcmp(f_cond_names[i],fcond1)==0)
	  fcond1ID = i;
	if (strcmp(f_cond_names[i],fcond2)==0)
	  fcond2ID = i;
      }

      // update the connected link structures for these two links

      if (!point_contact_flag) { // point contacts are isolated and do not fill the connect_links array

	f_full_flag = FALSE;
	for (i=1; i<=contacts[id1].n_connected_links; ++i)
	  if (contacts[id1].force_condition[i] == F_FULL)
	    f_full_flag = TRUE;
	
	if (!f_full_flag) { // a full force condition overwrites any partial force conditions
	  if (fcond1ID == F_FULL) {
	    contacts[id1].force_condition[1] = F_FULL;
	    contacts[id1].connected_links[1] = id2;
	    contacts[id1].n_connected_links  = 1;
	  } else {
	    if (contacts[id1].n_connected_links < MAX_CONNECTED) {
	      j = ++contacts[id1].n_connected_links;
	      contacts[id1].connected_links[j] = id2;
	      contacts[id1].force_condition[j] = fcond1ID;
	    } else {
	      printf("ERROR: ran out of memory for connected links -- increase MAX_CONNECTED in SL_objects.h\n");
	    }
	  }
	}
	
	f_full_flag = FALSE;
	for (i=1; i<=contacts[id2].n_connected_links; ++i)
	  if (contacts[id2].force_condition[i] == F_FULL)
	    f_full_flag = TRUE;
	
	if (!f_full_flag) { // a full force condition overwrites any partial force conditions
	  if (fcond2ID == F_FULL) {
	    contacts[id2].force_condition[1] = F_FULL;
	    contacts[id2].connected_links[1] = id1;
	    contacts[id2].n_connected_links  = 1;
	  } else {
	    if (contacts[id2].n_connected_links < MAX_CONNECTED) {
	      j = ++contacts[id2].n_connected_links;
	      contacts[id2].connected_links[j] = id1;
	      contacts[id2].force_condition[j] = fcond2ID;
	    } else {
	      printf("ERROR: ran out of memory for connected links -- increase MAX_CONNECTED in SL_objects.h\n");
	    }
	  }
	}

      } // if (!point_contact)


      // initialize the current contact structure
      for (i=1; i<=n_checks; ++i) {
	if (++count > n_contacts) {
	  printf("BUG: not enough contact elements allocated\n");
	  return FALSE;
	}
	contacts[count].id_start = id1;
	contacts[count].id_end   = id2;
	contacts[count].off_link_start = contacts[id1].off_link_start;
	contacts[count].off_link_end   = contacts[id2].off_link_end;
	contacts[count].base_dof_start = contacts[id1].base_dof_start;
	contacts[count].base_dof_end   = contacts[id2].base_dof_end;
	contacts[count].active         = active;
	contacts[count].fraction_start = ((double)i)/((double)(n_checks+1));
	if (point_contact_flag) {
	  contacts[count].fraction_start = 1.0;
	  contacts[count].force_condition[1] = fcond1ID;
	}
	contacts[count].fraction_end   = 1.-contacts[count].fraction_start;
	contacts[count].point_contact_flag = point_contact_flag;
	for (j=1; j<=N_CART; ++j) {
	  contacts[count].local_point_pos[j]  = lpos[j];
	  contacts[count].local_point_norm[j] = lnorm[j];
	}
      }

    } else { // should be EOF

      if (rc != EOF)
	printf("Parsing error in read_extra_contact_points in SL_objects.c (rc=%d)\n",rc);
      break;

    }


  } // while (TRUE)

  fclose(in);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  computeContactPoint
\date  Oct 2010
   
\remarks 

computes the global coordinates of the contact point from the contact 
structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]   cptr: pointer to contact structure
 \param[in]   lp  : pointer to link_pos array
 \param[in]   al  : pointer to Alink array
 \param[out]  x   : contact point in global coordinates

 ******************************************************************************/
void
computeContactPoint(ContactPtr cptr, double **lp, double ***al, double *x)

{
  int    i,j;
  double aux;

  if (cptr->point_contact_flag) {  // a point contact

    // convert the local contact point to global coordinates
    for (i=1; i<=N_CART; ++i) {
      x[i] = al[cptr->id_start][i][4];
      for (j=1; j<=N_CART; ++j)
	x[i] += al[cptr->id_start][i][j]*cptr->local_point_pos[j];
    }

  } else { // a line contact
    for (j=1; j<=N_CART; ++j)
      x[j] = (lp[cptr->id_start][j]*cptr->fraction_start + 
	      lp[cptr->id_end][j]*cptr->fraction_end);

  }

}


