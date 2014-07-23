/*!=============================================================================
  ==============================================================================

  \file    SL_sensor_proc.h

  \author  Stefan Schaal
  \date    May 2000

  ==============================================================================
  \remarks
  
  SL_sensor_proc.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_sensor_proc_
#define _SL_sensor_proc_

#ifdef __cplusplus
extern "C" {
#endif
  int init_sensor_processing(void);
  int read_sensors(void);
  int send_commands(void);
  int process_sensors(void);
  
  int  init_user_sensor_processing(void);
  int  read_user_sensors(SL_Jstate *raw, double *misc_raw);
  int  send_user_commands(SL_Jstate *com);
  int  user_kill(void);
  int  user_power_status(void);
  void userCheckForMessage(char *name, int k);
  void user_controller(double *u, double *uf);
  void init_joint_state_filters(SL_Jstate *js);
  void init_misc_sensor_filters(double *ms);
   
  
  extern double          **joint_lin_rot;
  extern double           *pos_polar;
  extern double           *load_polar;
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_sensor_proc_ */
