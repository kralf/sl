/*!=============================================================================
  ==============================================================================

  \file    SL_dynamics.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_dynamics.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_dynamics_
#define _SL_dynamics_

#define COULOMB_FUNCTION(thd,slope) (tanh(thd*slope))

enum RBDParms {
  MASS = 1,
  MCMX,
  MCMY,
  MCMZ,
  I11,
  I12,
  I13,
  I22,
  I23,
  I33,
  VIS,
  COUL,
  STIFF,
  CONS,

  N_RBDParms
};

#define N_RBD_PARMS (N_RBDParms-1)

/* shared functions */

#ifdef __cplusplus
extern "C" {
#endif

  int  init_dynamics(void);
  void setDefaultEndeffector(void);
  
  void SL_ForDyn(SL_Jstate *lstate,SL_Cstate *cbase,
		 SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_ForDynArt(SL_Jstate *lstate,SL_Cstate *cbase,
		    SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_ForDynComp(SL_Jstate *lstate,SL_Cstate *cbase,
		     SL_quat *obase, SL_uext *ux, SL_endeff *leff,
		     Matrix rbdM, Vector rbdCG);
  void SL_ForwardDynamics(SL_Jstate *lstate,SL_Cstate *cbase,
			  SL_quat *obase, SL_uext *ux, SL_endeff *leff);
  void SL_InverseDynamics(SL_Jstate *cstate,SL_DJstate *state,SL_endeff *endeff);
  void SL_InverseDynamicsArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_Cstate *cbase,
			     SL_quat *obase, SL_uext *ux, SL_endeff *leff);

  void SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		 SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynNE(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		   SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynArt(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		    SL_Cstate *cbase, SL_quat *obase);
  void SL_InvDynNEBase(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
		       SL_Cstate *cbase, SL_quat *obase, double *fbase);
  void test_NEvsForComp( void );
  void test_ForArtvsForComp( void );
  double compute_independent_joint_forces(SL_Jstate state, SL_link li);



  // external variables 
  extern int    freeze_base;
  extern double freeze_base_pos[];
  extern double freeze_base_quat[];
  extern const int floating_base_flag;
  extern double coulomb_slope;
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_dynamics_ */
