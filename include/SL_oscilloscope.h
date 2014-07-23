/*!=============================================================================
  ==============================================================================

  \file    SL_oscilloscope.h

  \author  Peter Pastor, Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  declarations needed by SL_oscilloscope.c

  ============================================================================*/

#ifndef _SL_oscilloscope_
#define _SL_oscilloscope_

#ifdef __cplusplus
extern "C" {
#endif
  
  // global functions 
  void       initOsc(void);
  int        setOsc(int channel, double pval);
  void       setD2AFunction(int (*fptr)(int,double));
  void       addEntryOscBuffer(char *name, double v, double ts, int cID);
  void       sendOscilloscopeData(void);
  void       updateOscVars(void);

  // external variables


#ifdef __cplusplus
}
#endif

#endif
