/*!=============================================================================
  ==============================================================================

  \file    SL_xeno_headers.h

  \author  Stefan Schaal
  \date    Oct. 2009

  ==============================================================================

  xenomai specific header files in one file for convenience

  ============================================================================*/


#ifndef _SL_xeno_headers_
#define _SL_xeno_headers_

#ifdef __XENO__

#include <sys/mman.h>
#include <native/timer.h>
#include <native/task.h>
#include <native/mutex.h>
#include <native/heap.h>
#include <native/sem.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <rtdk.h>

#endif

#endif  /* _SL_xeno_headers_ */
