/*!=============================================================================
  ==============================================================================

  \file    SL_common.h

  \author  Stefan Schaal
  \date    2000

  ==============================================================================
  \remarks

  declarations needed by SL_common.c

  ============================================================================*/

#ifndef _SL_common_
#define _SL_common_

#ifdef __cplusplus
extern "C" {
#endif

  /* global functions */
  int  where_utility(int start, int n_dofs);
  int  where_des_utility(int start, int n_dofs);
  int  read_sensor_offsets(char *fname);
  int  read_gains(char *fname, double *gth, double *gthd, double *gint);
  int  read_link_parameters(char *fname);
  int  read_whichDOFs(char *fname, char *keyword);
  int  read_sensor_calibration(char *fname, Matrix joint_lin_rot, 
			       Vector pos_polar, Vector load_polar);
  void setDefaultPosture(void);
  void cSL_Jstate(SL_Jstate *sd, SL_fJstate *sf, int n, int flag);
  void cSL_SDJstate(SL_DJstate *sd, SL_fSDJstate *sf, int n, int flag);
  void cSL_DJstate(SL_DJstate *sd, SL_fDJstate *sf, int n, int flag);
  void cSL_Cstate(SL_Cstate *sd, SL_fCstate *sf, int n, int flag);
  void cSL_Corient(SL_Corient *sd, SL_fCorient *sf, int n, int flag);
  void cSL_VisionBlob(SL_VisionBlob *sd, SL_fVisionBlob *sf, int n, int flag);
  void cSL_VisionBlobaux(Blob2D sd[][2+1], SL_fVisionBlobaux *sf, int n, int flag);
  void cSL_quat(SL_quat *sd, SL_fquat *sf, int n, int flag);
  void cBlob3D(Blob3D *sd, fBlob3D *sf, int n, int flag);
  void cBlob2D(Blob2D sd[][2+1], fBlob2D *sf, int n, int flag);
  void linkQuat(Matrix R, SL_quat *q);
  void quatDerivatives(SL_quat *q);
  void quatToAngularVelocity(SL_quat *q);
  void quatToRotMat(SL_quat *q, Matrix R);
  void quatToRotMatInv(SL_quat *q, Matrix R);
  void eulerToQuat(Vector a, SL_quat *q);
  void eulerToQuatInv(Vector a, SL_quat *q);
  void eulerToRotMat(Vector a, Matrix R);
  void eulerToRotMatInv(Vector a, Matrix R);
  void quatMatrix(SL_quat *q, Matrix Q);
  void quatToEuler(SL_quat *q, Vector a);
  void quatToEulerInv(SL_quat *q, Vector a);
  void rotMatToEuler(Matrix R, Vector a);
  void rotMatToEulerInv(Matrix R, Vector a);
  void setRealRobotOptions(void);
  int  read_config_files(char *fname);
  double quatError(double* q1, double* q2);
  void quatErrorVector(double* q1, double* q2, double *ad);
  int  read_servoParameters(char *fname, char *keyword, int *priority, 
			    int *stacksize, int *cpuID, int *dns);

  int
  read_parameter_pool_double(char *fname, char *keyword, double *value);
  int
  read_parameter_pool_double_array(char *fname, char *keyword, int n_values, double *values);
  int
  read_parameter_pool_int(char *fname, char *keyword, int *ivalue);
  int
  read_parameter_pool_int_array(char *fname, char *keyword, int n_values, int *ivalues);
  int
  read_parameter_pool_string(char *fname, char *keyword, char *svalue);
  void
  init_parameter_pool(void);
  int
  parseWindowSpecs(char *string, int dw, int dh, char *xstring, int *x, int *y, int *w, int *h);


  int count_extra_contact_points(char *fname);

  void rbwhere(void);
  void rbwhere2D(void);
  void where_base(void);
  void where_cog(void);
  void where_misc(void);
  void revoluteGJacColumn(Vector p, Vector pi, Vector zi, Vector c);
  void prismaticGJacColumn(Vector p, Vector pi, Vector zi, Vector c);
  void compute_cog(void);

  void
  compute_local_interface_forces(double *xdd, double *ad, double *add, double *g,
				 SL_link li, double *f, double *t);

  void print_J(void);


  // external variables
  extern char config_files[][100];

#ifdef __cplusplus
}
#endif

#endif
