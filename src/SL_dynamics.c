/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_dynamics.c

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks

  shared function used for forward and inverse dynamics

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "utility_macros.h"

// global variables
int    freeze_base               = FALSE;
double freeze_base_pos[N_CART+1] = {0.0,0.0,0.0,0.0};
double freeze_base_quat[N_QUAT+1] = {0.0,1.0,0.0,0.0,0.0};
double coulomb_slope = 10.0;

// local variables
static int forward_dynamics_comp_flag = FALSE;


/*!*****************************************************************************
*******************************************************************************
\note  init_dynamics
\date  Sept 2010
\remarks 

initializes all inverse dynamics meethods

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
init_dynamics( void )

{
  int    i, j,n;
  FILE  *in;
  char   string[100];
  double quat[N_QUAT+1];
  double pos[N_CART+1];
  double euler[N_CART+1];
  double aux;

  // read link parameters
  if (!read_link_parameters(config_files[LINKPARAMETERS]))
    return FALSE;

  // the the default endeffector parameters
  setDefaultEndeffector();

  // initialize the base variables
  bzero((void *)&base_state,sizeof(base_state));
  bzero((void *)&base_orient,sizeof(base_orient));
  base_orient.q[_Q0_] = 1.0;

  if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_pos",N_CART,pos)) {
    for (i=1; i<=N_CART; ++i)
      freeze_base_pos[i] = base_state.x[i] = pos[i];
  }
  
  if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_quat",N_QUAT,quat)) {
    aux = 0.0;
    for (i=1; i<=N_QUAT; ++i)
      aux += sqr(quat[i]);
    aux = sqrt(aux);

    for (i=1; i<=N_QUAT; ++i) 
      freeze_base_quat[i] = base_orient.q[i] = quat[i]/(aux + 1.e-10);
  } else if (read_parameter_pool_double_array(config_files[PARAMETERPOOL],"init_base_euler",N_CART,euler)) {
    SL_quat qtmp;

    bzero((void *)&qtmp,sizeof(qtmp));
    eulerToQuat(euler, &qtmp);
    for (i=1; i<=N_QUAT; ++i) 
      freeze_base_quat[i] = base_orient.q[i] = qtmp.q[i];
  }

  if (read_parameter_pool_int(config_files[PARAMETERPOOL],"use_comp_inertia_fordyn",&n)) {
    if (n == 1)
      forward_dynamics_comp_flag = TRUE;
    else
      forward_dynamics_comp_flag = FALSE;
  }
  
  if (read_parameter_pool_double(config_files[PARAMETERPOOL],"coulomb_slope",&aux)) {
    if (aux >= 0)
      coulomb_slope = aux;
  }
  
  return TRUE;
  
}    

/*!*****************************************************************************
*******************************************************************************
\note  SL_InvDyn
\date  Sept 2010

\remarks 

Standard Newton Euler inverse dynamics, which switches automatically between
floating base and fixed base robots

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
SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
	  SL_Cstate *cbase, SL_quat *obase)
{

  if (floating_base_flag)
    SL_InvDynArt(cstate, lstate, leff, cbase, obase);
  else
    SL_InvDynNE(cstate, lstate, leff, cbase, obase);
}

/*!*****************************************************************************
*******************************************************************************
\note  SL_InverseDynamics
\date  Sept 2010

\remarks 

depricated function for inverse dynamics computation

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in,out] cstate  : the current state (pass NULL to use only desired state)
\param[in,out] lstate  : the desired state
\param[in]     endeff  : the endeffector parameters

******************************************************************************/
void 
SL_InverseDynamics(SL_Jstate *cstate,SL_DJstate *lstate,SL_endeff *leff)
{
  static int firsttime = TRUE;
  static int counter = 0;

  if (firsttime) {
    printf("SL_InverseDynamics is depricated -- use SL_InvDyn instead\n");
    if (++counter > 10)
      firsttime = FALSE;
  }

  SL_InvDyn(cstate, lstate, leff, &base_state, &base_orient);

}

/*!*****************************************************************************
 *******************************************************************************
\note  SL_ForwardDynamics
\date  June 1999
   
\remarks 

        computes the forward dynamics accelerations

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
SL_ForwardDynamics(SL_Jstate *lstate,SL_Cstate *cbase,
		   SL_quat *obase, SL_uext *ux, SL_endeff *leff)
{
  static int firsttime = TRUE;
  static int counter = 0;

  if (firsttime) {
    printf("SL_ForwardDynamics is depricated -- use SL_ForDyn, SL_ForDynArt, or SL_ForDynComp instead\n");
    if (++counter > 10)
      firsttime = FALSE;
  }

  SL_ForDyn(lstate, cbase, obase, ux, leff);

}

/*!*****************************************************************************
 *******************************************************************************
\note  SL_ForDyn
\date  Sept 2010
   
\remarks 

computes the forward dynamics accelerations according to the default method,
which can be composite inertia or articulated body inertia.

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
SL_ForDyn(SL_Jstate *lstate,SL_Cstate *cbase,
	  SL_quat *obase, SL_uext *ux, SL_endeff *leff)

{
  MY_MATRIX(rbdM,1,n_dofs+6,1,n_dofs+6);
  MY_VECTOR(rbdCG,1,n_dofs+6);

  if (forward_dynamics_comp_flag)
    SL_ForDynComp(lstate, cbase, obase, ux, leff, rbdM, rbdCG);
  else 
    SL_ForDynArt(lstate, cbase, obase, ux, leff);

}

/*!*****************************************************************************
 *******************************************************************************
\note  compute_independent_joint_forces
\date  Sept 2010
   
\remarks 

computes the generalized joint forces due to friction and spring terms, i.e., 
the sum of all forces that act per joint independently of all others. The sign
of the terms is as if they were on the LEFT side of the RBD equations:

M qdd + C qd + G + f = u

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in] state       : the joint state of the robot
 \param[in] li          : the link parameters for this joint

 returns the generalized joint force for this joint due friction and spring terms

 ******************************************************************************/
double
compute_independent_joint_forces(SL_Jstate state, SL_link li)
{
  double f=0;

  f = state.thd*li.vis + 
    COULOMB_FUNCTION(state.thd,coulomb_slope)*li.coul +
    state.th*li.stiff +
    li.cons;
  
  return f;
}

/*!*****************************************************************************
*******************************************************************************
\note  test_NEvsForComp
\date  Sept 2010
\remarks 

a test function that checks the compatiblity of Newton Euler Inv.Dyn. with
Composite Inetia For.Dy. The result should have zero difference. This is done
for the floating  base case.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
test_NEvsForComp( void )
{
  int i,j;
  SL_Jstate jts[n_dofs+1];
  SL_DJstate djts[n_dofs+1];
  SL_Cstate bs,bs2;
  SL_quat   bo,bo2;
  SL_uext   ux[n_dofs+1];
  double    fbase[N_CART*2+1];
  double    aux;

  MY_MATRIX(rbdM,1,n_dofs+6,1,n_dofs+6);
  MY_VECTOR(rbdCG,1,n_dofs+6);

  bzero((void *)jts,sizeof(SL_Jstate)*(n_dofs+1));
  bzero((void *)djts,sizeof(SL_DJstate)*(n_dofs+1));
  bzero((void *)ux,sizeof(SL_uext)*(n_dofs+1));
  bzero((void *)&bs,sizeof(bs));
  bzero((void *)&bo,sizeof(bo));
  bo.q[_Q0_] = 1.0;

  // create a random state
  for (i=1; i<=n_dofs; ++i) {
    djts[i].th = jts[i].th = gaussian(0,1.0);
    djts[i].thd = jts[i].thd = gaussian(0,1.0);
    djts[i].thdd = jts[i].thdd = gaussian(0,1.0);
  }

  for (i=1; i<=N_CART; ++i) {
    bs.x[i] = gaussian(0,1.0);
    bs.xd[i] = gaussian(0,1.0);
    bs.xdd[i] = gaussian(0,1.0);
  }

  for (i=1; i<=N_CART; ++i) {
    //bo.ad[i] = gaussian(0,1.0);
    bo.add[i] = gaussian(0,1.0);
  }

  aux = 0.0;
  for (i=1; i<=N_QUAT; ++i) {
    bo.q[i] =  gaussian(0,1.0);
    aux += sqr(bo.q[i]);
  }
  aux = sqrt(aux);

  for (i=1; i<=N_QUAT; ++i)
    bo.q[i] /= aux;


  bs2 = bs;
  bo2 = bo;

  // NE inverse dynamics
  SL_InvDynNEBase(NULL, djts, endeff, &bs, &bo, fbase);

  for (i=1; i<=n_dofs; ++i) // need to subtract viscous force which NE adds automatically
    djts[i].uff -= djts[i].thd*links[i].vis;

  // Comp.Inertia forward dynamics
  SL_ForDynComp(jts, &bs2, &bo2, ux, endeff, rbdM, rbdCG);

  // from the RBD matrix and the C+G vector, we can compute the total
  // (unconstraint) floating base inverse dynamics command
  for (i=1; i<=n_dofs+2*N_CART; ++i) {
    for (j=1; j<=n_dofs; ++j)
      rbdCG[i] += rbdM[i][j] * djts[j].thdd;
    for (j=1; j<=N_CART; ++j)
      rbdCG[i] += rbdM[i][j+n_dofs] * bs.xdd[j];
    for (j=1; j<=N_CART; ++j)
      rbdCG[i] += rbdM[i][j+N_CART+n_dofs] * bo.add[j];
  }
  
  // print out
  for (i=1; i<=n_dofs; ++i) {
    printf("%d: NE=% 6.3f Comp=% 6.3f (% 6.3f)\n",i,djts[i].uff,rbdCG[i],djts[i].uff-rbdCG[i]);
  }

  for (i=1; i<=2*N_CART; ++i) {
    printf("%d: NE=% 6.3f Comp=% 6.3f (% 6.3f)\n",i+n_dofs,fbase[i],rbdCG[i+n_dofs],fbase[i]-rbdCG[i+n_dofs]);
  }
  

}

/*!*****************************************************************************
*******************************************************************************
\note  test_ForArtvsForComp
\date  Sept 2010
\remarks 

a test function that checks the compatiblity of Articulated Body Inertia 
forward ynamics  with Composite Inertia For.Dyn. The result should have zero 
difference. This is done for the floating base case.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
test_ForArtvsForComp( void )
{
  int i,j;
  SL_Jstate jts[n_dofs+1],jts2[n_dofs+1];
  SL_Cstate bs,bs2;
  SL_quat   bo,bo2;
  SL_uext   ux[n_dofs+1];
  double    aux;

  bzero((void *)jts,sizeof(SL_Jstate)*(n_dofs+1));
  bzero((void *)jts2,sizeof(SL_Jstate)*(n_dofs+1));
  bzero((void *)ux,sizeof(SL_uext)*(n_dofs+1));
  bzero((void *)&bs,sizeof(bs));
  bzero((void *)&bo,sizeof(bo));
  bo.q[_Q0_] = 1.0;

  // create a random state
  for (i=1; i<=n_dofs; ++i) {
    jts[i].th = jts2[i].th = gaussian(0,1.0);
    jts[i].thd = jts2[i].thd = gaussian(0,1.0);
    jts[i].u = jts2[i].u = gaussian(0,1.0);
  }

  for (i=1; i<=N_CART; ++i) {
    bs.x[i] = gaussian(0,1.0);
    bs.xd[i] = gaussian(0,1.0);
  }

  for (i=1; i<=N_CART; ++i)
    bo.ad[i] = gaussian(0,1.0);

  aux = 0.0;
  for (i=1; i<=N_QUAT; ++i) {
    bo.q[i] =  gaussian(0,1.0);
    aux += sqr(bo.q[i]);
  }
  aux = sqrt(aux);

  for (i=1; i<=N_QUAT; ++i)
    bo.q[i] /= aux;

  bs2 = bs;
  bo2 = bo;

  // Comp.Inertia forward dynamics
  SL_ForDynArt(jts, &bs, &bo, ux, endeff);

  // Comp.Inertia forward dynamics
  SL_ForDynComp(jts2, &bs2, &bo2, ux, endeff, NULL, NULL);

  
  // print out
  for (i=1; i<=n_dofs; ++i) 
    printf("%d: Art=% 6.3f Comp=% 6.3f (% 6.3f)\n",i,jts[i].thdd,jts2[i].thdd,jts[i].thdd-jts2[i].thdd);


  for (i=1; i<=N_CART; ++i) 
    printf("%d: Art=% 6.3f Comp=% 6.3f (% 6.3f)\n",i+n_dofs,bs.xdd[i],bs2.xdd[i],bs.xdd[i]-bs2.xdd[i]);

  for (i=1; i<=N_CART; ++i) 
    printf("%d: Art=% 6.3f Comp=% 6.3f (% 6.3f)\n",i+N_CART+n_dofs,bo.add[i],bo2.add[i],bo.add[i]-bo2.add[i]);

}
