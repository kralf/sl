/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_inv_dynamics_art.c

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  articulated body inverse dynamics, suitable for floating base inverse
  dynamics, but much more expensive to compute that normal inverse
  dynamics algorithms.

  Note: the body was moved to a header file as this file needs robot specific
        header files.

  ============================================================================*/

#include "SL_inv_dynamics_art_body.h"
