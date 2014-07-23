/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

   \file    SL_integrate.c

  \author 
  \date   

  ==============================================================================
  \remarks

      functions for numerical intergration

  ============================================================================*/
#include "stdio.h"
#include "math.h"
#include "SL.h"
#include "SL_integrate.h"
#include "utility.h"
#include "strings.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_objects.h"

/* openGL headers */ 
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif
#include "SL_openGL.h"

/*!*****************************************************************************
 *******************************************************************************
\note  SL_IntegrateEuler
\date  June 1999
   
\remarks 

        Euler integration

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state    : the state and commands used for integration
 \param[in,out] cbase    : the position state of the base
 \param[in,out] obase    : the orientational state of the base
 \param[in]     ux       : the external forces acting on each joint
 \param[in]     leff   : the leffector parameters
 \param[in]     dt     : the time step
 \param[in]     ndofs  : the number of DOFS
 \param[in]     flag   : compute forward dynamics yes/no

 ******************************************************************************/
void 
SL_IntegrateEuler(SL_Jstate *state, SL_Cstate *cbase,
		  SL_quat *obase,  SL_uext *ux, 
		  SL_endeff *leff, double dt, int ndofs,
		  int flag)
{
  register int i;
  double aux=0;
  static int firsttime = TRUE;

  if (firsttime) {
    addToMan("freezeBase","freeze the base at orgin",freezeBaseToggle);
    firsttime = FALSE;
  }

  if (flag) {

    // compute the accelerations
    SL_ForDyn(state, cbase, obase, ux, leff);

  }

  // Euler integrate forward 

  // the DOFs
  for(i=1; i<=ndofs; i++) {
    state[i].thd += dt*state[i].thdd;
    state[i].th  += dt*state[i].thd;
  }

  if (!freeze_base) {  // optional freezing of base coordinates
    
    // translations of the base
    for(i=1; i<=N_CART; i++) {
      cbase->xd[i]    += dt*cbase->xdd[i];
      cbase->x[i]     += dt*cbase->xd[i];
    }
    
    // orientation of the base in quaternions
    for(i=1; i<=N_CART; i++) {
      obase->ad[i]    += dt*obase->add[i];
    }

    // compute quaternion velocity and acceleration
    quatDerivatives(obase);

    // integrate to obtain new angular orientation
    for(i=1; i<=N_QUAT; i++) {
      obase->q[i]     += dt*obase->qd[i];
      aux += sqr(obase->q[i]);
    }
    aux = sqrt(aux);
    if (aux == 0) 
      aux = 1.e-10;
    
    // important: renormalize quaternions
    for(i=1; i<=N_QUAT; i++) {
      obase->q[i]/=aux;
    }
    
  } else { // if base coordinates are frozen

       bzero((void *)cbase,sizeof(SL_Cstate));
       bzero((void *)obase,sizeof(SL_quat));
       obase->q[_Q0_] = freeze_base_quat[_Q0_];
       obase->q[_Q1_] = freeze_base_quat[_Q1_];
       obase->q[_Q2_] = freeze_base_quat[_Q2_];
       obase->q[_Q3_] = freeze_base_quat[_Q3_];
       cbase->x[_X_] = freeze_base_pos[_X_];
       cbase->x[_Y_] = freeze_base_pos[_Y_];
       cbase->x[_Z_] = freeze_base_pos[_Z_];
  }


  // update the simulated link positions such that contact forces are correct
  linkInformation(state,cbase,obase,leff,
		  joint_cog_mpos_sim,joint_axis_pos_sim,joint_origin_pos_sim,
		  link_pos_sim,Alink_sim,Adof_sim);
  
  // check for contacts with objects
  checkContacts();


}

/*!*****************************************************************************
 *******************************************************************************
\note  SL_IntegrateRK
\date  June 1999
   
\remarks 

        4th order Runge-Kutta integration

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state    : the state and commands used for integration
 \param[in,out] cbase    : the position state of the base
 \param[in,out] obase    : the orientational state of the base
 \param[in]     ux       : the external forces acting on each joint
 \param[in]     leff   : the leffector parameters
 \param[in]     leff   : the leffector parameters
 \param[in]     dt     : the time step
 \param[in]     ndofs  : the number of DOFS

 ******************************************************************************/
void 
SL_IntegrateRK(SL_Jstate *state, SL_Cstate *cbase,
	       SL_quat  *obase, SL_uext *ux, 
	       SL_endeff *leff, double dt, int ndofs)

{
  register int          i;
  static   int          firsttime=TRUE;
  static SL_Jstate     *fr1,*fr2,*fr3,*fr4;
  static SL_Cstate      cs1,cs2,cs3,cs4;
  static SL_quat        os1,os2,os3,os4;
  static ContactPtr     store_contacts;
 
  
  if (firsttime) {
    firsttime = FALSE;
    fr1 = my_calloc((size_t) ndofs+1,sizeof(SL_Jstate),MY_STOP);
    fr2 = my_calloc((size_t) ndofs+1,sizeof(SL_Jstate),MY_STOP);
    fr3 = my_calloc((size_t) ndofs+1,sizeof(SL_Jstate),MY_STOP);
    fr4 = my_calloc((size_t) ndofs+1,sizeof(SL_Jstate),MY_STOP);
    store_contacts = my_calloc(n_links+1,sizeof(Contact),MY_STOP);
  }

  // notation largely copied from Numerical Recipes (NR)

  // store the current contact state as the numerical differentations
  // in there cannot deal with the RK evaluations
  for (i=0; i<=n_links; ++i)
    store_contacts[i] = contacts[i];

  // evalute accelerations at current state, called k1/h in NR
  for (i=1; i<=ndofs; ++i)
    fr1[i] = state[i];
  cs1 = *cbase;
  os1 = *obase;
  SL_ForDyn(fr1,  &cs1, &os1, ux, leff);

  // integrate fr1 forward by dt/2 and call this fr2
  for (i=1; i<=ndofs; ++i)
    fr2[i] = fr1[i];
  cs2 = cs1;
  os2 = os1;
  SL_IntegrateEuler(fr2,  &cs2, &os2, ux, leff, dt/2.,ndofs,FALSE);

  // evaluate accelerations at fr2, called k2/h in NR
  SL_ForDyn(fr2,  &cs2, &os2, ux, leff);

  // now integrate forward fr1 based on k2/h information
  for (i=1; i<=ndofs; ++i)
    fr3[i] = fr1[i];
  cs3 = cs1;
  os3 = os1;
  for (i=1; i<=n_dofs; ++i)
    fr3[i].thdd = fr2[i].thdd;
  for(i=1; i<=N_CART;i++)
    cs3.xdd[i]=cs2.xdd[i];
  for(i=1; i<=N_CART;i++)
    os3.add[i]=os2.add[i];
  for (i=0; i<=n_links; ++i)
    contacts[i] = store_contacts[i];
  SL_IntegrateEuler(fr3,  &cs3, &os3, ux, leff, dt/2.,ndofs,FALSE);

  // evaluate accelerations at fr3, called k3 in NR
  SL_ForDyn(fr3,  &cs3, &os3, ux, leff);

  // now integrate forward fr1 based on this information
  for (i=1; i<=ndofs; ++i)
    fr4[i] = fr1[i];
  cs4 = cs1;
  os4 = os1;
  for (i=1; i<=n_dofs; ++i)
    fr4[i].thdd = fr3[i].thdd;
  for(i=1; i<=N_CART;i++)
    cs4.xdd[i]=cs3.xdd[i];
  for(i=1; i<=N_CART;i++)
    os4.add[i]=os3.add[i];
  for (i=0; i<=n_links; ++i)
    contacts[i] = store_contacts[i];
  SL_IntegrateEuler(fr4,  &cs4, &os4, ux, leff, dt,ndofs,FALSE);

  // evaluate accelerations at fr4, called k4 in NR
  SL_ForDyn(fr4, &cs4, &os4, ux, leff);

  /*
  for (i=1; i<=n_dofs; ++i) {
    printf("%d: % 7.4f % 7.4f % 7.4f % 7.4f\n",
	   i,fr1[i].thdd,fr2[i].thdd,fr3[i].thdd,fr4[i].thdd);
  }
  for(i=1; i<=N_CART;i++) {
    printf("%d: % 7.4f % 7.4f % 7.4f % 7.4f\n",
	   i,cs1.xdd[i],cs2.xdd[i],cs3.xdd[i],cs4.xdd[i]);
  }
  for(i=1; i<=N_CART;i++) {
    printf("%d: % 7.4f % 7.4f % 7.4f % 7.4f\n",
	   i,os1.add[i],os2.add[i],os3.add[i],os4.add[i]);
  }

  getchar();
  */

  // Use RK updates to obtain the final state updates
  for(i=1; i<=ndofs;i++){
    state[i].thdd = (fr1[i].thdd + 2.*fr2[i].thdd + 2.*fr3[i].thdd + fr4[i].thdd)/6.;
  }

  for(i=1; i<=N_CART;i++){
    cbase->xdd[i] = (cs1.xdd[i] + 2.*cs2.xdd[i] + 2.*cs3.xdd[i] + cs4.xdd[i])/6.;
  }

  for(i=1; i<=N_CART;i++){
    obase->add[i] = (os1.add[i] + 2.*os2.add[i] + 2.*os3.add[i] + os4.add[i])/6.;
  }

  for (i=0; i<=n_links; ++i)
    contacts[i] = store_contacts[i];

  SL_IntegrateEuler(state,cbase,obase, ux, leff, dt,ndofs,FALSE);

}

/*!*****************************************************************************
 *******************************************************************************
\note  freezeBase
\date  Nov. 2005
   
\remarks 

        toggle to freeze or not freeze the base of a floating base robot

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     flag  : TRUE/FALSE for frozen or not

 ******************************************************************************/
void 
freezeBaseToggle(void) 
{
  
  if (freeze_base == 0) {
    freeze_base = TRUE;
    printf("Base is fixed at origin\n");
  } else {
    freeze_base = FALSE;
    printf("Base is floating\n");
  }
  
}
void 
freezeBase(int flag)
{

  freeze_base = flag;
  if (freeze_base != 0)
    freeze_base = TRUE;

}
  
