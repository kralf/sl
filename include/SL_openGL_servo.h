/*!=============================================================================
  ==============================================================================

  \file    SL_openGL_servo.h

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  SL_openGL_servo specific header file
  
  ============================================================================*/
  
#ifndef _SL_openGL_servo_
#define _SL_openGL_servo_

#ifdef __cplusplus
extern "C" {
#endif
  
  // external variables
  extern double  servo_time;
  extern double  openGL_servo_time;
  extern long    openGL_servo_calls;
  extern long    last_openGL_servo_calls;
  extern int     openGL_servo_rate;
  extern int     openGL_servo_errors;
  extern int     stand_alone_flag;
  extern int     servo_enabled;
  
  // shared functions
  int receive_sim_state(void);
  int receive_misc_sensors(void);
  int receive_contacts(void);
  int init_openGL_servo(int argc, char** argv);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_openGL_servo_ */
