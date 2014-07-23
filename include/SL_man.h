/*!=============================================================================
  ==============================================================================

  \file    SL_man.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  SL_man.c specific header file
  
  ============================================================================*/
  
#ifndef _SL_man_
#define _SL_man_

/* external variables */

/* share functions */

#ifdef __cplusplus
extern "C" {
#endif

void initMan(void);
void man(void);
void addToMan(char *abr, char *string, void (*fptr)(void));

#ifdef __cplusplus
}
#endif

#endif  /* _SL_man_ */
