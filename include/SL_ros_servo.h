/*!=============================================================================
  ==============================================================================

  \file    SL_ros_servo.h

  \author  Stefan Schaal
  \date    July 2010

  ==============================================================================

  the header file for the ros_servo.c

  ============================================================================*/


#ifndef _SL_ros_servo_
#define _SL_ros_servo_

#ifdef __cplusplus
extern "C" {
#endif

  extern int    ros_servo_errors;
  extern long   ros_servo_calls;
  extern int    ros_servo_initialized;
  extern double last_ros_servo_time;
  extern double ros_servo_time;
  extern double servo_time;
  extern int    servo_enabled;
  extern int    ros_servo_rate;
  
  void init_ros_servo(void);
  int  run_ros_servo(void);
  int  init_user_ros(void);
  int  run_user_ros(void);
  void status(void);

#ifdef __cplusplus
}
#endif

#endif  // _SL_ros_servo_
