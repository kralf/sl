/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_invDynArt.c

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks

  articulated body inverse dynamics, suitable for floating base inverse
  dynamics, but much more expensive to compute that normal inverse
  dynamics algorithms.


  ============================================================================*/

/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "utility_macros.h"
#include "SL.h"
#include "SL_user.h"
#include "mdefs.h"
#include "SL_dynamics.h"
#include "SL_integrate.h"

/* global variables */ 

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  SL_InvDynArt
\date  June 1999
   
\remarks 

computes the torques for all joints, and the acceleration of the base

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters
\param[in,out] cbase   : the position state of the base
\param[in,out] obase   : the orientational state of the base

 ******************************************************************************/
void 
SL_InvDynArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff, 
	     SL_Cstate *cbase, SL_quat *obase)

{
#include "InvDynArt_declare.h"
  int i,j;
  SL_uext ux[N_DOFS+1];

  SL_DJstate  state[N_DOFS+1];
  SL_endeff  *eff;
  SL_Cstate  *basec;
  SL_quat    *baseo;
  SL_uext    *uex;

#include "InvDynArt_functions.h"

  /* the following assignments make the arguments global variables */ 

  /* create a mixed desired/current state for proper inverse dynamics */
  for (i=1; i<=N_DOFS; ++i) {
    state[i]     = lstate[i];
    if (cstate != NULL) {
      state[i].th  = cstate[i].th;
      state[i].thd = cstate[i].thd;
    }
    for (j=1; j<=N_CART; ++j) 
      ux[i].f[j] = ux[i].t[j] = 0.0;

  }

  // zero out the base forces
  for (j=1; j<=N_CART; ++j) 
      ux[0].f[j] = ux[0].t[j] = 0.0;

  eff    = leff;
  basec  = cbase;
  baseo  = obase;
  uex    = ux;

  
#include "InvDynArt_math.h"

  /* add the friction term */
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

/*!*****************************************************************************
 *******************************************************************************
\note  SL_InverseDynamicsArt
\date  June 1999
   
\remarks 

computes the torques for all joints, and the acceleration of the
base

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

\param[in]     cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in,out] cbase   : the position state of the base
\param[in,out] obase   : the orientational state of the base
\param[in]     ux      : the external forces acting on each joint, in local 
                        (link) coordinates, e.g., as measured by a local force
                         or torque cell
 \param[in]    endeff  : the endeffector parameters

 ******************************************************************************/
void 
SL_InverseDynamicsArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_Cstate *cbase,
		      SL_quat *obase, SL_uext *ux, SL_endeff *leff)

{
  int i,j;
  static int firsttime = TRUE;
  static int counter = 0;

  if (firsttime) {
    printf("SL_InverseDynamicsArt is depricated -- use SL_InvDynArt instead\n");
    if (++counter > 10)
      firsttime = FALSE;
  }

  SL_InvDynArt(cstate, lstate, leff, cbase, obase);
} 


