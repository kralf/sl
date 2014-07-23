/*!=============================================================================
  ==============================================================================

  \file    SL_system_headers.h

  \author  Stefan Schaal
  \date    Oct 2009

  ==============================================================================

  Common system header files for unix, vxworks, and xenomai. This
  should be the first include file everywhere, to make sure the
  xenomai headers come at the right place.

  ============================================================================*/


#ifndef _SL_system_headers_
#define _SL_system_headers_

#ifdef VX

#include "vxWorks.h"
#include "vxLib.h" 
#include "sysLib.h"
#include "intLib.h"
#include "semLib.h"
#include "fppLib.h"
#include "taskLib.h"
#include "smNameLib.h"
#include "semSmLib.h"
#include "smMemLib.h"
#include "smObjLib.h"
#include "usrLib.h"
#include "usrConfig.h"
#include "logLib.h"
#include "usrLib.h"
#include "tickLib.h"

/* device drivers etc. */
#include "dta.h"

#else

#ifdef __XENO__
#include "SL_xeno_headers.h"
#else
#include "pthread.h"
#endif

#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "strings.h"
#include "limits.h"
#include "sys/time.h"
#include "signal.h"

#include "SL_vx_wrappers.h"

#endif

#endif  /* _SL_system_headers_ */
