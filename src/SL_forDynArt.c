/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_forDynArt.c

  \author  Stefan Schaal 
  \date    Sept. 2010

  ==============================================================================
  \remarks

  the articulated body inertia forward dynamics method

  ============================================================================*/

/* system headers */
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "utility_macros.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_dynamics.h"
#include "SL_integrate.h"
#include "mdefs.h"


/* global variables */ 

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  SL_ForDynArt
\date  Sept 2010
   
\remarks 

computes the forward dynamics accelerations from the articulate body inertia
method

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state   : the state containing th, thd, thdd, and receiving the
                          appropriate u
 \param[in,out] cbase   : the position state of the base
 \param[in,out] obase   : the orientational state of the base
 \param[in]     ux      : the external forces acting on each joint, 
                          in world coordinates, e.g., as computed from contact 
                          forces
 \param[in]     endeff  : the endeffector parameters

 ******************************************************************************/
void 
SL_ForDynArt(SL_Jstate *lstate,SL_Cstate *cbase,
	     SL_quat *obase, SL_uext *ux, SL_endeff *leff)
{
#include "ForDynArt_declare.h"
  int i,j;
  double fbase[2*N_CART+1];  
  SL_Jstate  *state;
  SL_endeff  *eff;
  SL_Cstate  *basec;
  SL_quat    *baseo;
  SL_uext    *uex;
  
  // this makes the arguments global variables 
  state  = lstate;
  eff    = leff;
  basec  = cbase;
  baseo  = obase;
  uex    = ux;

#include "ForDynArt_functions.h"

  // subtract the friction term temporarily 
  for (i=1; i<=N_DOFS; ++i) {
    state[i].u -= compute_independent_joint_forces(state[i],links[i]);
  }

#include "ForDynArt_math.h"

  // add back the friction term
  for (i=1; i<=N_DOFS; ++i) {
    state[i].u += compute_independent_joint_forces(state[i],links[i]);
  }

} 


