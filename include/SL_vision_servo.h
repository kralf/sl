/*!=============================================================================
  ==============================================================================

  \file    SL_vision_servo.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  the header file for the SL_vision_servo.c

  ============================================================================*/


#ifndef _SL_vision_servo_
#define _SL_vision_servo_

#ifndef SYS_CLOCK_RATE  
#define SYS_CLOCK_RATE  60
#endif
#define VISION_SERVO_RATE  60.0

#define BLOB2ROBOT         1
#define BLOB2ROBOT_SCRIPT  "learn/blob2robot.script"

#define VISION_3D_MODE     0
#define VISION_2D_MODE     1

#ifdef __cplusplus
extern "C" {
#endif

extern int           servo_enabled;
extern double        servo_time;
extern double        vision_servo_time;
extern double        last_vision_servo_time;
extern int           vision_servo_rate;
extern int           vision_servo_calls;
extern char          current_pp_name[100];
extern int           vision_servo_initialized;
extern int           vision_servo_errors;
extern int           count_all_frames;
extern int           count_lost_frames;
extern int           *blob_is_endeffector;
extern int           raw_blob_overwrite_flag;
extern int           no_hardware_flag;
extern int           learn_transformation_flag;



/* global functions */
int  run_vision_servo(void);
void init_vision_servo();
void evs( void );
void enable_vision_servo( void );
void dvs(void);
void disable_vision_servo(void);
int  init_vision_hardware(void);
int  acquire_blobs(Blob2D raw_blobs2D[][2+1]);
void process_blobs(Blob2D raw_blobs2D[][2+1]);
int  init_vision_processing(void);
int  init_pp(char *name);
void init_vision_states(void);
int  init_vxworks( void );
int  init_learning( void );
int  init_user_vision(void);
int  stop(char *msg);
int  sendUserGraphics(char *name, void *buf, int n_bytes);


#ifdef __cplusplus
}
#endif

#endif // _SL_vision_servo_


