/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_filters.c

  \author 
  \date   

  ==============================================================================
  \remarks

             manages filtering with Butterworth filters

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_filters.h"
#include "utility.h"


/* variables for filtering */
static float filters_a[N_FILTERS+1][FILTER_ORDER+1];
static float filters_b[N_FILTERS+1][FILTER_ORDER+1];
static int filters_initialized = FALSE;

/* global functions */

/* local functions */

/*!*****************************************************************************
 *******************************************************************************
\note  init_filters
\date  Dec 1997
   
\remarks 

        reads the filter coefficients and initializes the appropriate
	variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_filters(void)

{
  
  int i,j,rc;
  FILE *filterfile;
  int count_v=0, count_r=0;
  char string[100];

  if (filters_initialized)
    return TRUE;
  
  /* read in the filter file */
  
  sprintf(string,"%sbutterworth_table_order_2",CONFIG);
  filterfile = fopen(string,"r");
  
  if (filterfile == NULL) {
    
    printf("Cannot find filter file >%s<\n",string);
    beep(1); 
    
    return FALSE;
    
  }
  
  for (i=1; i<=N_FILTERS; ++i) {
    ++count_r;
    for (j=0; j<= FILTER_ORDER; ++j) {
      ++count_v;
      rc=fscanf(filterfile,"%f",&(filters_a[i][j]));
    }
    
    for (j=0; j<= FILTER_ORDER; ++j) {
      ++count_v;
      rc=fscanf(filterfile,"%f",&(filters_b[i][j]));
    }
  }
  
  fclose (filterfile);
  
  printf("\nRead file of butterworth filter coefficients: #rows=%d  #val=%d\n",
	 count_r,count_v);

  filters_initialized = TRUE;

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  filt
\date  Feb 1999
   
\remarks 

    applies a 2nd order butteworth filter

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     raw : the raw data
 \param[in]     fptr: pointer to the filter data structure 

     returns the filtered value

 ******************************************************************************/
double
filt(double raw, Filter *fptr)

{

  int i,j;

  if (!filters_initialized)
    init_filters();

  fptr->raw[0]  = raw;

  if (fptr->cutoff == 100) {

    fptr->filt[0] = raw;

  } else {

    fptr->filt[0] = 
      filters_b[fptr->cutoff][0] * fptr->raw[0] +
      filters_b[fptr->cutoff][1] * fptr->raw[1] + 
      filters_b[fptr->cutoff][2] * fptr->raw[2] -
      filters_a[fptr->cutoff][1] * fptr->filt[1] -
      filters_a[fptr->cutoff][2] * fptr->filt[2];
    
  }

  fptr->raw[2]  = fptr->raw[1];
  fptr->raw[1]  = fptr->raw[0];
  fptr->filt[2] = fptr->filt[1];
  fptr->filt[1] = fptr->filt[0];

  return fptr->filt[0];

}
