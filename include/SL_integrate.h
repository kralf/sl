#ifndef SL_INTEGRATE_H
#define SL_INTEGRATE_H

#define INTEGRATE_EULER 1
#define INTEGRATE_RK    2

#ifdef __cplusplus
extern "C" {
#endif
  
  void 
  SL_IntegrateEuler(SL_Jstate *state, SL_Cstate *cbase,
		    SL_quat *obase, SL_uext *ux, 
		    SL_endeff *leff, double dt, int ndofs,
		    int flag);
  void 
  SL_IntegrateRK(SL_Jstate *state, SL_Cstate *cbase,
		 SL_quat *obase, SL_uext *ux, 
		 SL_endeff *leff, double dt, int ndofs);

  void freezeBase(int);
  void freezeBaseToggle(void);
  
#ifdef __cplusplus
}
#endif

#endif
