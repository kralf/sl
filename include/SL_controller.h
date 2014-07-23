/*!=============================================================================
  ==============================================================================

  \file    SL_controller.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_controller.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_controller_
#define _SL_controller_

#ifdef __cplusplus
extern "C" {
#endif

/* external variables */

extern int controller_kind;
extern int power_on;

/* share functions */

void ck(void);
void controllerKind(void);
int  generate_total_commands( void);
int  init_controller(void);
void zero_integrator(void);
void setGains(int);
void setGainsSim(void);
int  stop(char *);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_controller_ */
