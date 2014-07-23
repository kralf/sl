/*!=============================================================================
  ==============================================================================

  \file    SL_motor_servo.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  motor_servo specific header file
  
  ============================================================================*/
  
#ifndef _SL_motor_servo_
#define _SL_motor_servo_

#ifndef SYS_CLOCK_RATE
#define SYS_CLOCK_RATE  60
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern long          motor_servo_calls;
extern int           motor_servo_rate;
extern double        last_motor_servo_time;
extern double        motor_servo_time;
extern double        servo_time;
extern int           servo_enabled;
extern int          *zero_ufb_P_flag;
extern int          *zero_ufb_D_flag;
extern int           real_time_clock_flag;

/* share functions */

int  read_register_file(char *fname_in);
void dms(void);
void disable_motor_servo(void);
void status(void);
void init_motor_servo(void);
int  run_motor_servo(void);
int  init_vxworks( void );
int  init_user_motor(void);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_motor_servo_ */
