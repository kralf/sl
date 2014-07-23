/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_invdynNE.c

  \author  Stefan Schaal
  \date    Sept 2010

  ==============================================================================
  \remarks

  Newton Euler inverse dynamics for fixed base robotic systems. Two versions
  are implemented, the standard NE algorithm with base state velocity and
  accelerations equal to zero, and a special version that takes the base 
  velocity and acceleration into account. The latter treats the fixed base
  robot as if the base were full actuated as well, and thus requires a desired
  acceleration for the base as input. This special version returns the commands
  needed at the DOFs and base to realize the given acceleration.


  ============================================================================*/

// system headers
#include "SL_system_headers.h"

// private includes
#include "utility.h"
#include "utility_macros.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "mdefs.h"
#include "SL_dynamics.h"

// local variables

// global functions

// local functions
static void 
SL_InvDynNEGeneral(SL_Jstate *cstate,SL_DJstate *lstate,SL_endeff *leff,
		   SL_Cstate *cbase, SL_quat *obase, SL_uext *ux,
		   double *fbase);

// external variables

/*!*****************************************************************************
*******************************************************************************
\note  SL_InvDynNE
\date  Sept 2010

\remarks 

Standard Newton Euler inverse dynamics for fixed base robot.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters
\param[in]     cbase   : the position state of the base
\param[in]     obase   : the orientational state of the base

Returns:
The appropriate feedforward torques are added in the uff component of the lstate
structure.

******************************************************************************/
void 
SL_InvDynNE(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
	    SL_Cstate *cbase, SL_quat *obase)
{
  int       i,j;
  double    fbase[2*N_CART+1];
  SL_Cstate cb;
  SL_quat   ob;
  SL_uext   ux[N_DOFS+1];

  // this simple version of NE does only take the base position/orientation
  // into account, but otherwise the base is fixed.
  cb = *cbase;
  ob = *obase;

  // eliminate all velocity and acceleration information
  for (i=1; i<=N_CART; ++i)
    cb.xd[i] = cb.xdd[i] = 0.0;

  for (i=1; i<=N_CART; ++i)
    ob.ad[i] = ob.add[i] = 0.0;

  for (i=1; i<=N_QUAT; ++i)
    ob.qd[i] = ob.qdd[i] = 0.0;

  for (i=1; i<=N_DOFS; ++i)
    for (j=1; j<=N_CART; ++j)
      ux[i].f[j] = ux[i].t[j] = 0.0;


  SL_InvDynNEGeneral(cstate, lstate, leff, &cb, &ob, ux, fbase);

}

/*!*****************************************************************************
*******************************************************************************
\note  SL_InvDynNEBase
\date  Sept 2010

\remarks 

Standard Newton Euler inverse dynamics for fixed base robot, but this function
allows a full base state (including velocities and accelerations). This is to
be interpretated as if the base if full actuated, and the force/torque vector
in world coordinates is returned to reflect this required base command.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters
\param[in]     cbase   : the position state of the base
\param[in]     obase   : the orientational state of the base
\param[out]    fbase   : the force/torque vector of the base in world coordinates

Returns:
The appropriate feedforward torques are added in the uff component of the lstate
structure. The force/torque vector for the base is returned

******************************************************************************/
void 
SL_InvDynNEBase(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		SL_Cstate *cbase, SL_quat *obase, double *fbase)
{
  int       i,j;
  SL_Cstate cb;
  SL_quat   ob;
  SL_uext   ux[N_DOFS+1];
  
  // this simple version of NE does only take the base position/orientation
  // into account, but otherwise the base is fixed.
  cb = *cbase;
  ob = *obase;

  // eliminate all external forces
  for (i=1; i<=N_DOFS; ++i)
    for (j=1; j<=N_CART; ++j)
      ux[i].f[j] = ux[i].t[j] = 0.0;

  SL_InvDynNEGeneral(cstate, lstate, leff, &cb, &ob, ux, fbase);

}


/*!*****************************************************************************
*******************************************************************************
\note  SL_InvDynNEGeneral
\date  Sept 2010

\remarks 

This is the generalized inverse dynamics function which computes the NE 
inverse dynamics for various special cases, as intialized by specific 
subroutines.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters
\param[in]     cbase   : the position state of the base
\param[in]     obase   : the orientational state of the base
\param[in]     ux      : the external forces acting on each joint, in world 
                         coordinates, e.g., as computed from contact forces
\param[out]    fbase   : the force/torque vector of the base in world coordinates


******************************************************************************/
static void 
SL_InvDynNEGeneral(SL_Jstate *cstate,SL_DJstate *lstate,SL_endeff *leff,
		   SL_Cstate *cbase, SL_quat *obase, SL_uext *ux,
		   double *fbase)

{
#include "InvDynNE_declare.h"
  int i;

  SL_DJstate state[N_DOFS+1];
  SL_endeff  *eff; 
  SL_Cstate  *basec;
  SL_quat    *baseo;
  SL_uext    *uex;
  
  // this makes the arguments global variables
  eff    = leff;
  basec  = cbase;
  baseo  = obase;
  uex    = ux;

#include "InvDynNE_functions.h"

  // create a mixed desired/current state for proper inverse dynamics
  for (i=1; i<=N_DOFS; ++i) {
    state[i]     = lstate[i];
    if (cstate != NULL) {
      state[i].th  = cstate[i].th;
      state[i].thd = cstate[i].thd;
    }
  }

#include "InvDynNE_math.h"

  // add the friction term
  for (i=1; i<=N_DOFS; ++i) {
    SL_Jstate  jt;
    jt.th = state[i].th;
    jt.thd = state[i].thd;
    state[i].uff += compute_independent_joint_forces(jt,links[i]);
    if (i > N_DOFS-N_DOFS_EST_SKIP)
      state[i].uff = 0.0;
    lstate[i].uff = state[i].uff;
  }

}


