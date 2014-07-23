/*!=============================================================================
  ==============================================================================

  \file    SL_objects_defines.h

  \author  Stefan Schaal
  \date    May 2010

  ==============================================================================
  \remarks
  
  SL_objects.c specific header file -- just the defines
  
  ============================================================================*/
  
#ifndef _SL_objects_defines_
#define _SL_objects_defines_

/*! possible object types */
#define CUBE         1
#define SPHERE       2
#define TERRAIN      3
#define CYLINDER     4

#define N_CUBE_PARMS         0
#define N_SPHERE_PARMS       1
#define N_TERRAIN_PARMS      0
#define N_CYLINDER_PARMS     1

/*! possible contact models */
#define NO_CONTACT                       0
#define DAMPED_SPRING_STATIC_FRICTION    1
#define DAMPED_SPRING_VISCOUS_FRICTION   2
#define DAMPED_SPRING_LIMITED_REBOUND    3

#define N_NO_CONTACT_PARMS                       0
#define N_DAMPED_SPRING_STATIC_FRICTION_PARMS    6
#define N_DAMPED_SPRING_VISCOUS_FRICTION_PARMS   3
#define N_DAMPED_SPRING_LIMITED_REBOUND_PARMS    8

#define MAX_OBJ_PARMS 30
#define MAX_CONTACT_PARMS 30

#ifdef __cplusplus
extern "C" {
#endif

  // external variables

  // shared functions

  // external variables
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_objects_defines_ */
