/*!=============================================================================
  ==============================================================================

  \ingroup SLopenGL

  \file    SL_openGL_servo_unix.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  the main program for graphics visualization
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// openGL headers
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

// local headers
#include "SL.h"
#include "SL_common.h"
#include "SL_openGL_servo.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"

// global variables
  
// local functions 

  
/*!*****************************************************************************
*******************************************************************************
\note  main
\date  July 1998
 
\remarks 
 
entry program: the structure of the entire program is such that we assume 
some initialization functions, and then a final function that continuously
runs the simulation in a "clocked" fashion.
 
*******************************************************************************
Function Parameters: [in]=input,[out]=output
 
\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings
 
******************************************************************************/
int 
main(int argc, char**argv)

{
  int i,j,n;
  int rc;
  int ans; 


  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // get the stand alone flag
  stand_alone_flag = FALSE;
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-sta")==0) {
      stand_alone_flag = TRUE;
      break;
    }
  }

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  if (!init_openGL_servo(argc,argv))
    return FALSE;

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);
  
  // start the main loop
  servo_enabled = TRUE;
  glutMainLoop();
  
  return TRUE;
}


