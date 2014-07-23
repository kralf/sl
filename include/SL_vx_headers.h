/*!=============================================================================
  ==============================================================================

  \file    SL_vx_headers.h

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================

  vxworks specific header files in one file for convenience

  ============================================================================*/


#ifndef _SL_vx_headers_
#define _SL_vx_headers_

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

#include "SL_vx_wrappers.h"
#ifdef  _POSIX_C_SOURCE  /* hack needed for DEC alpha */
#undef  _POSIX_C_SOURCE
#endif

#endif

#endif  /* _SL_vx_headers_ */
