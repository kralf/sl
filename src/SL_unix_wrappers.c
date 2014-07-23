/*!=============================================================================
  ==============================================================================

  \file    SL_unix_wrappers.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  replacement functions and  definition of functions of unix when
  vxWorks is used

  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "string.h"
#include "SL_vx_headers.h"
#include "utility.h"


int
sendUserGraphics(char *name, void *buf, int n_bytes)
{
  return TRUE;
}

void 
changeRealTime(int flag) 
{
  return;
}

void
hideWindowByName(char *name, int hide)
{
  return;
}

