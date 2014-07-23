/*!=============================================================================
  ==============================================================================

  \ingroup SLcommon

  \file    SL_collect_data.c

  \author  Stefan Schaal
  \date    April 1999

  ==============================================================================
  \remarks

      this file has all the programs necessary to create output
      data files of variable length. Basically, each possible
      piece of data gets an ID number and its pointer and title
      are registered.
      When it comes to collecting data, a script file is read in which
      specifies which data should be collected in the buffer. 

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* local includes */
#include "SL.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "utility.h"

#define  MAX_CHARS 20

#ifndef VX
#define LLSB(x)	((x) & 0xff)		/*!< 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)
#define LONGSWAP(x) ((LLSB(x) << 24) | \
		     (LNLSB(x) << 16)| \
		     (LNMSB(x) << 8) | \
		     (LMSB(x)))
#endif

/* buffer to collect data */
static int    store_in_buffer_rate=1;
static fMatrix buffer=NULL;
static int    n_data_in_buffer=0;
static int    n_cols, n_rows;

/*! a structure to store information about variables */
typedef struct Cinfo {
  char *ptr;
  char  name[MAX_CHARS];
  char  units[MAX_CHARS];
  int   type;
  char *nptr;
} Cinfo;

#define MAX_VARS 2000
static Cinfo  vars;              /* contains a chain of all 
				    variables that can be
				    collected */
static Cinfo *cvars[MAX_VARS+1]; /* array to point to all variables to be 
				    collected */
static int    n_cvars=0;
static int    file_number=0;
static int    freq = 1;
static double data_collect_time=0;
static char   script_name[100];
static int    n_save_data_points=1;
static double sampling_freq;
static int    n_calls = 0;
static int    collect_data_initialized = FALSE;

/* global variables */
int    save_data_flag = FALSE;

/* functions */
static int readDataCollectScript( char *fn, int flag );
static void sim_mscds(void);

/*!*****************************************************************************
 *******************************************************************************
\note  initCollectData
\date  April 1999
   
\remarks 

        initialized the data collection package

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     freq : what frequency has the data collection loop

 ******************************************************************************/
void 
initCollectData( int freq )
{

  FILE   *outfile;
  int    i,rc;
  FILE  *in;
  double temp;
  char   string[100];

  if (collect_data_initialized)
    return;

  /* catch user errors */
  if (freq <= 0) 
    freq = 1;

  sampling_freq = freq;

  /* the first variable is always the time */
  vars.ptr   = (char *) &data_collect_time;
  strcpy(vars.name,"time");
  strcpy(vars.units,"s");
  vars.type  = DOUBLE;
  vars.nptr   = NULL;

  /* write all the names of the possible data columns out to a file
     which can serve as a sample to compose script files */

  sprintf(string,"%s%s_sample.script",PREFS,servo_name);
  outfile = fopen(string,"w");
  if (outfile == NULL) {
    printf("Couldn't open file >%s< for write\n",string);
    return;
  }

  fprintf(outfile,"%s\n",vars.name);
  fclose(outfile);

  /* read the default script which has the default settings */

  sprintf(string,"%sdefault_script",PREFS);

  // now open for read
  in = fopen(string,"r");

  if (in == NULL) {

    sprintf(script_name,"%s_default.script",servo_name);

  } else {

    sprintf(string,"%s_default_script",servo_name);
    if (find_keyword(in,string))
      rc=fscanf(in,"%s",script_name);

    sprintf(string,"%s_default_sampling_frequency_Hz",servo_name);
    if (find_keyword(in,string)) {
      rc=fscanf(in,"%lf",&temp);
      if (temp <= 0 || temp > sampling_freq) temp = sampling_freq;
      store_in_buffer_rate = (double) sampling_freq/ (double) temp;
    }

    sprintf(string,"%s_default_sampling_time_sec",servo_name);
    if (find_keyword(in,string)) {
      rc=fscanf(in,"%lf",&temp);
      if (temp <= 0 ) temp = 2.0;
      n_save_data_points 
	= temp /((double) store_in_buffer_rate) * sampling_freq;
    }

    fclose(in);

  }

  /* add to man pages */
  addToMan("scd","start collect data",scd);
  addToMan("stopcd","manuall stop collect data",stopcd);
  addToMan("scds","start collect data, automatic saving",scds);
  addToMan("mscds","start collect data, automatic saving of multiple files",sim_mscds);
  addToMan("saveData","save data from data collection to file",saveData);
  addToMan("outMenu","interactively change data collection settings",outMenu);

  collect_data_initialized = TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  readDataCollectScript
\date  April 1999
   
\remarks 

        reads a data collection script and prepares the current_outputs 
	array. The output script consists of the variable names according
	the sample.script file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fn : the name of the file to be opened
 \param[in]     flag : verbose on or off

 ******************************************************************************/
static int
readDataCollectScript( char *fn, int flag )
     
{

  FILE  *infile;
  int    i,rc;
  char   string[100];
  char   fname[100];
  Cinfo *ptr;

  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return FALSE;
  }

  sprintf(fname,"%s%s",PREFS,fn);
  infile = fopen(fname,"r");

  if (infile == NULL) {
    printf("Error when trying to open file >%s< !\n",fname);
    return FALSE;
  }

  /* make sure no data collection is running */
  save_data_flag = FALSE;
  
  /* now read the file */
  n_cvars = 0;
  while ((rc=fscanf(infile,"%s",string)) != EOF) {

    /* figure out whether the string exists */
    ptr = &vars;
    do {
      if (strcmp(ptr->name,string) == 0) {
	break;
      }
      ptr = (Cinfo *)ptr->nptr;
    } while (ptr != NULL);

    if (ptr == NULL)
      continue;

    /* otherwise, the appropriate variable was found and needs to be
       added to the data collection array */

    if (n_cvars >= MAX_VARS) {
      printf("Memory for number of variables in collect_data exhausted\n");
      break;
    }

    cvars[++n_cvars] = ptr;

  }

  fclose(infile);
  
  if (flag) {
    printf("\nRead %d valid names from file >%s< .\n\n",n_cvars,fname);
  }

  /* ensure that we have an adequate data collection buffer */
  if (buffer != NULL) {
    my_free_fmatrix(buffer,1,n_rows,1,n_cols);
    buffer = NULL;
  } 

  if (n_cvars > 0 && n_save_data_points > 0) {
    n_rows = n_save_data_points;
    n_cols = n_cvars;
    buffer = my_fmatrix(1,n_rows,1,n_cols);
  }

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  writeToBuffer
\date  August 7, 1992
   
\remarks 

        write the current output data to the buffer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

        none

 ******************************************************************************/
void
writeToBuffer(void)

{
  int i,j,n;
  float temp;

  if (!collect_data_initialized) {
    return;
  }

  if (save_data_flag == TRUE && n_calls%store_in_buffer_rate == 0 &&
      n_save_data_points > n_data_in_buffer)  {

    ++n_data_in_buffer;
    
    for (i=1; i<=n_cvars; ++i) {

      switch (cvars[i]->type) {

      case DOUBLE:
	buffer[n_data_in_buffer][i] = 
	  (float) *((double *) cvars[i]->ptr);
	break;

      case FLOAT:
	buffer[n_data_in_buffer][i] = 
	  (float) *((float *) cvars[i]->ptr);
	break;

      case INT:
	buffer[n_data_in_buffer][i] = 
	  (float) *((int *) cvars[i]->ptr);
	break;

      case SHORT:
	buffer[n_data_in_buffer][i] = 
	  (float) *((short *) cvars[i]->ptr);
	break;

      case LONG:
	buffer[n_data_in_buffer][i] = 
	  (float) *((long *) cvars[i]->ptr);
	break;

      case CHAR:
	buffer[n_data_in_buffer][i] = 
	  (float) *((char *) cvars[i]->ptr);
	break;

      }

    }

    if (n_data_in_buffer >= n_save_data_points) {
      save_data_flag = FALSE;
      logMsg("time=%d.%03d : buffer is full! %c\n", (int) data_collect_time,
	     ((int)(data_collect_time*1000))-((int) data_collect_time)*1000,
	     0x7,0,0,0);
    }    

  }     
  
  /* increment the counter of calls to this function */
  ++n_calls;
  data_collect_time = ((double)n_calls) / sampling_freq;

}
  


/*!*****************************************************************************
 *******************************************************************************
\note  saveData
\date  April 1999
   
\remarks 

        writes data out to file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
saveData(void)

{

  int  i,j,rc;
  FILE *fp, *fd, *in;
  char filename[100];
  int  buffer_size;
  int  aux;


  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return;
  }

  if (save_data_flag) {
    printf("Data collection is running -- try saving later\n");
    return;
  }

  if (n_cvars < 1) {
    printf("No variables specified for Data collection -- try outMenu\n");
    return;
  }

  printf( "Saving data:\n" );
  
  // create a file name
  if ( ( in = fopen( "last_data", "r" ) ) != NULL ) { // migration to .last_data
    int rc;
    rc = system("mv last_data .last_data");
    fclose(in);
  }
  if ( ( in = fopen( ".last_data", "r" ) ) == NULL )   {
      printf( "cannot fopen file .last_data for read.\n" );
  } else {
    rc=fscanf( in, "%d\n", &file_number );
    fclose( in );
  }

  sprintf( filename, "d%05d", file_number );
  printf( "Saving data in %s\n", filename );
  file_number++;
  
  if ( ( fp = fopen( filename, "w" ) ) == NULL )  {
    printf( "cannot fopen file %s for write.\n", filename );
    return;
  }

  /* the buffer size, the number of columns, the sampling time, 
     the column names and units */

  buffer_size = n_save_data_points * n_cvars;
  fprintf(fp,"%d  %d %d  %f\n",buffer_size, n_cvars,
	  n_save_data_points,
	  sampling_freq/(double)store_in_buffer_rate);

  for (i=1; i<=n_cvars; ++i) {
    fprintf(fp,"%s  ",cvars[i]->name);
    fprintf(fp,"%s  ",cvars[i]->units);
  }
  fprintf(fp,"\n");

#ifdef BYTESWAP
  /* convert little-endian to big-endian */ 
  for (j=1; j<=n_cols; ++j) {
    for (i=1; i<=n_rows; ++i) {
      aux = LONGSWAP(*((int *)&(buffer[i][j])));
      buffer[i][j] = *((float *)&aux);
    }
  }
#endif

  if (fwrite(&(buffer[1][1]),sizeof(float),n_rows*n_cols,fp)!= n_rows*n_cols) {
    printf( "cannot fwrite matrix.\n" );
    return;
  }

  fclose( fp );
  
  if ( ( fd = fopen( ".last_data", "w" ) ) == NULL )   {
      printf( "cannot fopen file .last_data for write.\n" );
      return;
  }
  fprintf( fd, "%d\n", file_number );
  fclose( fd );

  /* clear buffer for next accumulation */
  
  mat_fzero(buffer);

  printf( "All done, captain!\n" );

}

/*!*****************************************************************************
 *******************************************************************************
\note  scd & startCollectData
\date  August 7, 1992
   
\remarks 

        starts to collect data

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     param   :

 ******************************************************************************/
void
scd(void)

{
  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return;
  }

  startCollectData();
}

int
dscd(int parm)

{
  double start;
  long   count=0;

  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return FALSE;
  }

  start = data_collect_time;
  while (data_collect_time - start < parm) {
    if (data_collect_time - start == parm - 5) {
      beep(300);
    }
    if (++count > 100000 && data_collect_time == start) {
      printf("servo does not seem to be running! Aborted ....\n");
      return FALSE;
    }
    taskDelay(ns2ticks(10000000)); // wait 10ms
  }
  printf("Start Collect Data NOW!\n");
  beep(1);
  startCollectData();
  return TRUE;
}

int
startCollectData()
{

  save_data_flag      = TRUE;
  n_calls             = 0;
  data_collect_time   = 0;
  n_data_in_buffer    = 0;
  return TRUE;

}

void
scds(void)

{
  double last_draw_time;

  scd();
  last_draw_time = data_collect_time;
  while (save_data_flag) {
    taskDelay(ns2ticks(10000000)); // wait 10ms
  }
  saveData();

}

void
mscds(int num)
{
  int i;

  for (i=1; i<=num; ++i) {
    scds();
  }

}

static void
sim_mscds(void)  /* for simulation only */
{
  static int num=1;
  
  if (!get_int("How many data files? ",num,&num))
    return;
  mscds(num);
}


void  /* stop the data collection prematurely */
stopcd(void)
{
  save_data_flag = FALSE;
  logMsg("time=%d.%03d : Data collection interupted! %c\n", 
	 (int) data_collect_time,
	 ((int)(data_collect_time*1000))-((int) data_collect_time)*1000,
	 0x7,0,0,0);
}    

/*!*****************************************************************************
 *******************************************************************************
\note  outMenu
\date  April 1999
   
\remarks 

        a menu to make the output script stuff more visible

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
outMenu(void)

{

  int aux = 2;
  double temp;
  static char  fname[100] = "default.script";

  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return;
  }

  if (save_data_flag) {
    int ans = 0;
    printf("You cannot change output variables while data collection is running!\n");
    get_int("Do you want to abort on-going data collection? No=0, Yes=1",ans,&ans);
    if (ans == 1) 
      stopcd();
    else
      return;
  }

  AGAIN:
  
  printf("\n\n\nOUTPUT SCRIPT OPTIONS:\n\n");
  printf("        Sampling Rate                   ---> %d\n",1);
  printf("        Read Script File                ---> %d\n",2);
  printf("        Sampling Time                   ---> %d\n",3);
  printf("        Quit                            ---> q\n");
  printf("\n");
  if (!get_int("        ----> Input",aux,&aux)) return;
  
  if (aux > 3 || aux < 1) {
    
    goto AGAIN;
    
  } 
  
  
  if (aux == 1) {
    
    if (get_double("At which frequency [Hz] should data be stored?\0",
		   sampling_freq/(double)store_in_buffer_rate,
		   &temp))
      if (temp > sampling_freq) temp = sampling_freq;
      store_in_buffer_rate = sampling_freq/temp;
    
  } else if (aux == 2) {
    
  ONCE_MORE:

    /* get the file name */

    if (!get_string("Name of the Output Script File\0",script_name,fname)) 
      goto AGAIN;

    /* try to read this file */

    if (!readDataCollectScript(fname,TRUE)) 
      goto ONCE_MORE;

    strcpy(script_name,fname);
    
  } else if (aux == 3) {

    temp = ((double)n_save_data_points * store_in_buffer_rate) / 
      sampling_freq;
    if (get_double("How many seconds are to be stored?\0",
		   temp,&temp))
      n_save_data_points = 
	temp /((double) store_in_buffer_rate) * sampling_freq;
    
    /* ensure that we have an adequate data collection buffer */
    if (buffer != NULL) {
      my_free_fmatrix(buffer,1,n_rows,1,n_cols);
    } 
    
    if (n_cvars > 0 && n_save_data_points > 0) {
      n_rows = n_save_data_points;
      n_cols = n_cvars;
      buffer = my_fmatrix(1,n_rows,1,n_cols);
    }
    
  }
  
  goto AGAIN;

}

/*!*****************************************************************************
 *******************************************************************************
\note  addVarToCollect
\date  April 1999
   
\remarks 

        add a variable to the possible set of data collection variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ptr  : pointer to the variable
 \param[in]     name : name of variable
 \param[in]     units: units of variable
 \param[in]     type : data type of variable, e.g., int, float, double, etc.
 \param[in]     flag : update the date collection script

 ******************************************************************************/
void
addVarToCollect(char *vptr,char *name,char *units, int type, int flag)

{

  Cinfo *ptr;
  FILE  *outfile;
  char   string[100];

  if (!collect_data_initialized) {
    printf("Collect data is not initialized\n");
    return;
  }

  ptr = &vars;

  while (ptr != NULL) {
    /* does variable already exist? */
    if (strcmp(ptr->name,name)==0) {
      /* overwrite the pointer in case a task was re-initialized */
      ptr->ptr  = vptr;
      return;
    }
    if (ptr->nptr == NULL)
      break;
    ptr = (Cinfo *)ptr->nptr;
  }

  ptr->nptr = (char *) my_calloc(1,sizeof(Cinfo),MY_STOP);
  ptr = (Cinfo *)ptr->nptr;
  ptr->ptr  = vptr;
  ptr->nptr = NULL;
  ptr->type = type;
  strncpy(ptr->name,name,MAX_CHARS-1);
  ptr->name[MAX_CHARS-1] = '\0';
  strncpy(ptr->units,units,MAX_CHARS-1);
  ptr->units[MAX_CHARS-1] = '\0';


  if (flag) {
    
    sprintf(string,"%s%s_sample.script",PREFS,servo_name);
    outfile = fopen(string,"a+");
    if (outfile == NULL) {
      printf("Couldn't open file >%s< for append\n",string);
      return;
    }

    fprintf(outfile,"%s\n",name);
    fclose(outfile);

    /* read which variable are to be collected -- maybe there are some
       variable that we missed before */

    readDataCollectScript(script_name,FALSE);

  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  changeCollectFreq
\date  April 1999
   
\remarks 

        changes the data collection frequency

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     freq : what frequency has the data collection loop

 ******************************************************************************/
void 
changeCollectFreq( int freq )
{
  sampling_freq = freq;
}


/*!*****************************************************************************
 *******************************************************************************
\note  updateDataCollectScript
\date  April 1999
   
\remarks 

        changes the data collection frequency

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     freq : what frequency has the data collection loop

 ******************************************************************************/
void 
updateDataCollectScript( void )
{

  Cinfo *ptr;
  FILE *outfile;
  char string[100];
  
  /* dump all variables to the sample.script */

  sprintf(string,"%s%s_sample.script",PREFS,servo_name);
  outfile = fopen(string,"w");
  if (outfile == NULL) {
    printf("Couldn't open file >%s< for append\n",string);
    return;
  }
  
  ptr = &vars;
  while (ptr != NULL) {
    fprintf(outfile,"%s\n",ptr->name);
    ptr = (Cinfo *)ptr->nptr;
  }
  fclose(outfile);
  
  /* update the data collection */

  readDataCollectScript(script_name,FALSE);

}

/*!*****************************************************************************
 *******************************************************************************
\note  getDataCollectPtr
\date  Aug. 2010
   
\remarks 

 returns the pointer to a variable and its data type, if this variable exists
 in the data collection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name : name of variable
 \param[in]     n_char_units : pass sizeof(units) or an explicity integer
 \param[out]    vptr : ptr to variable (or NULL if not existant)
 \param[out]    type : data type (see SL_collect_data.h)
 \param[out]    units: units of variable (see SL_collect_data.h)

 returns FALSE if not found, and TRUE if found

 ******************************************************************************/
int
getDataCollectPtr( char *name, int n_char_units, void **vptr, int *type, 
		   char *units )
{

  Cinfo *ptr;
  char string[100];
  
  // search all variables for data collection
  ptr = &vars;
  while (ptr != NULL) {
    if (strcmp(ptr->name,name)==0) {
      *vptr = (void *)ptr->ptr;
      *type = ptr->type;
      strncpy(units,ptr->units,n_char_units);
      return TRUE;
    }
    ptr = (Cinfo *)ptr->nptr;
  }

  return FALSE;
}


