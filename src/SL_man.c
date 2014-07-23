/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_man.c

  \author 
  \date   

  ==============================================================================
  \remarks

         a simple program to allow diplay of "manual" pages      

  ============================================================================*/

#include "string.h"

#include "SL.h"
#include "SL_man.h"

typedef struct ManEntry {
  char abr[20];
  char exp[1000];
  void (*func)(void);    
  char *nptr;
} ManEntry;

/* global variables */

/* local variables */
static ManEntry mans;
static int man_initialized = FALSE;


/*!*****************************************************************************
 *******************************************************************************
\note  addToMan
\date  June 1999
   
\remarks 

      allows to add a function to the man pages

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     abr    :   functional name as typed on the keyboard
 \param[in]     string :   explanation
 \param[in]     fptr   :   pointer to the function

 ******************************************************************************/
void
addToMan(char *abr, char *string, void (*fptr)(void))

{

  int i;
  ManEntry *ptr;
  extern void addCommand();

  if (!man_initialized) {
    initMan();
  }

  ptr = &mans;

  /* check for duplicate entries */
  do {
    if (strcmp(ptr->abr,abr)==0) {
      /* update the function pointer and explantory string */
      ptr->func = fptr;
      strcpy(ptr->exp,string);
#ifndef VX
      addCommand(abr,fptr);
#endif
      return;
    }
    if (ptr->nptr == NULL)
      break;
    else
      ptr = (ManEntry *)ptr->nptr;
  } while (TRUE);
  
  ptr->nptr = (char *) my_calloc(1,sizeof(ManEntry),MY_STOP);
  ptr = (ManEntry *)ptr->nptr;
  strcpy(ptr->abr,abr);
  strcpy(ptr->exp,string);
  ptr->func = fptr;
  ptr->nptr = NULL;

#ifndef VX
  addCommand(abr,fptr);
#endif

}
/*!*****************************************************************************
 *******************************************************************************
\note  initMan
\date  June 1999
   
\remarks 

      initializes the man pages

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
void
initMan(void)
{
  extern void addCommand();

  if (man_initialized)
    return;

  strcpy(mans.abr,"man");
  sprintf(mans.exp,"prints all help messages");
  mans.nptr = NULL;
  mans.func = man;

#ifndef VX
  addCommand(mans.abr,man);
#endif

  man_initialized=TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  man
\date  June 1999
   
\remarks 

      displays man pages

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

   none

 ******************************************************************************/
void
man(void)

{
  int i;
  ManEntry *ptr;

  if (!man_initialized) {
    initMan();
  }

  ptr = &mans;

  while (ptr != NULL) {
    printf("%20s -- %s\n",ptr->abr,ptr->exp);
    ptr = (ManEntry *)ptr->nptr;
  }
  
}
