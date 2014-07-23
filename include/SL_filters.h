/*!=============================================================================
  ==============================================================================

  \file    SL_filters.h

  \author  Stefan Schaal
  \date    April 2000

  ==============================================================================

      the header file for the filters.c

  ============================================================================*/


#ifndef _SL_filters_
#define _SL_filters_

#define FILTER_ORDER 2
#define N_FILTERS    50

typedef struct Filter {
  int cutoff;
  double raw[3];
  double filt[3];
} Filter;

#ifdef __cplusplus
extern "C" {
#endif

int    init_filters(void);
double filt(double raw, Filter *fptr);

#ifdef __cplusplus
}
#endif

#endif // _SL_filters_
