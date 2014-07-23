/*!=============================================================================
  ==============================================================================

  \file    SL_unix_wrappers.h

  \author  Stefan Schaal
  \date    August 2008

  ==============================================================================

  fake declarations for unix to allow vxWorks to run the 
  same files

  ============================================================================*/


#ifndef _SL_unix_wrappers_
#define _SL_unix_wrappers_

/* function declarations */

#ifdef __cplusplus
extern "C" {
#endif
  
  int sendUserGraphics(char *name, void *buf, int n_bytes);
  void changeRealTime(int flag);
  void hideWindowByName(char *name, int hide);
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_unix_wrappers_ */
