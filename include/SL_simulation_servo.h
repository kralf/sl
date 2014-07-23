/*!=============================================================================
  ==============================================================================

  \file    SL_simulation_servo.h

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  simulation_servo specific header file
  
  ============================================================================*/
  
#ifndef _SL_simulation_servo_
#define _SL_simulation_servo_

#ifdef __cplusplus
extern "C" {
#endif

  // external variables
  extern int     servo_enabled;
  extern long    simulation_servo_calls;
  extern int     simulation_servo_rate;
  extern int     simulation_servo_errors;
  extern double  last_simulation_servo_time;
  extern double  simulation_servo_time;
  extern double  servo_time;


  extern int     n_integration;
  extern int     integrate_method;
  extern int     real_time;

  // shared functions
  int  init_simulation_servo(void);
  int  init_user_simulation(void);
  int  run_user_simulation(void);
  int  receive_des_commands(void);
  int  send_sim_state(void);
  int  send_misc_sensors(void);
  int  send_contacts(void);
  int  run_simulation_servo(void);
  int  checkForMessages(void);
  void reset(void);
  void dss(void);
  void disable_simulation_servo(void);
  int  initUserSimulation(void);


#ifdef __cplusplus
}
#endif

#endif  // _SL_simulation_servo_
