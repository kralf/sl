/*!=============================================================================
  ==============================================================================

  \file    SL_kinematics.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_kinematics.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_kinematics_
#define _SL_kinematics_

/* external variables */


/* shared functions */

#ifdef __cplusplus
extern "C" {
#endif

  void   init_kinematics(void);
  void   genJacobian(Vector point, int link, Matrix jop, Matrix jap, Matrix J);
  void   jacobian(Matrix lp, Matrix jop, Matrix jap, Matrix J);
  void   baseJacobian(Matrix lp, Matrix jop, Matrix jap, Matrix Jb);
  double inverseKinematics(SL_DJstate *state, SL_endeff *endeff, SL_OJstate *rest,
			   Vector cart, iVector status, double dt);
  double inverseKinematicsClip(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			       Vector cart, iVector status, double dt, double max_rev,
			       double max_pris);

  void   linkInformation(SL_Jstate *state,SL_Cstate *basec,
			 SL_quat *baseo, SL_endeff *eff, 
			 double **Xmcog, double **Xaxis, double **Xorigin, 
			 double **Xlink, double ***Ahmat, double ***Ahmatdof);
  void linkInformationDes(SL_DJstate *state,SL_Cstate *basec,
			  SL_quat *baseo, SL_endeff *eff, 
			  double **Xmcog, double **Xaxis, double **Xorigin, 
			  double **Xlink, double ***Ahmat, double ***Ahmatdof);
  void 
  computeLinkVelocity(int lID, Matrix lp, Matrix jop, Matrix jap, 
		      SL_Jstate *js, double *v);

  void 
  computeLinkVelocityPoint(int lID, double *point, Matrix lp, Matrix jop, Matrix jap, 
			   SL_Jstate *js, double *v);

  void 
  computeConstraintJacobian(SL_Jstate *state,SL_Cstate *basec,
			    SL_quat *baseo, SL_endeff *eff, 
			    Matrix Jc, int *nr, int *nc);
  
  void 
  computeQR(Matrix Jc, int nr, int nc, Matrix Q, Matrix Qu, Matrix R, Vector sv);

  int
  checkIKTarget(SL_DJstate *js, SL_Cstate *bs, SL_quat *bo, SL_endeff *eff, 
		SL_Cstate *ct, int *status, int max_iter);
  

#ifdef __cplusplus
}
#endif

#endif  /* _SL_kinematics_ */
