/*!=============================================================================
  ==============================================================================

  \file    SL_userSimulation.h

  \author  Stefan Schaal
  \date    Sept. 2010

  ==============================================================================
  \remarks
  
  SL_userSimulation.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_userSimulation_
#define _SL_userSimulation_

/* external variables */

/* shared functions */

#ifdef __cplusplus
extern "C" {
#endif

  void addToUserSimulation(char *abr, char *string, void (*fptr)(void));
  void runUserSimulation(void);
  void clearUserSimulation(void);
  int  activateUserSimulation(char *name);
  void initUserSim(void);

  extern int  user_simulation_update;

#ifdef __cplusplus
}
#endif

#endif  /* _SL_userSimulation_ */
