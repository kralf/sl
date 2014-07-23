/*!=============================================================================
  ==============================================================================

  \file    SL_unix_common.h

  \author 
  \date   

  ==============================================================================
  \remarks

  declarations needed by SL_unix_common.c

  ============================================================================*/

#ifndef _SL_unix_common_
#define _SL_unix_common_

#include "sys/time.h"
#include "unistd.h"
#include "pthread.h"

#include "SL_rt_mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

  // external variables 
  //extern pthread_mutex_t mutex1;
  extern sl_rt_mutex mutex1;
  extern int run_command_line_thread_flag;
  extern int (*window_check_function)(char *);
  extern int    global_argc;
  extern char **global_argv;


  // global functions 
  void  spawnCommandLineThread(char *initial_command);
  void  addCommand(char *name, void (*fptr)(void));
  void  installSignalHandlers(void);
  void  printSLBanner(void);
  void  printPrompt(void);
  void  parseOptions(int argc, char**argv);
  void  sendCommandLineCmd(char *name);
  
#ifdef __cplusplus
}
#endif

#endif
