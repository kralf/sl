/*!=============================================================================
  ==============================================================================

  \file    SL_register.c

  \author  Stefan Schaal
  \date    August 1993

  ==============================================================================
  \remarks

  manages the reading of the SARCOS register files by calling
  an adapted version of their code for vxWorks

  ============================================================================*/

/* vxWorks includes */

#include "vxWorks.h"
#include "stdioLib.h"
#include "memLib.h"
#include "string.h"
#include "utility.h"
#include "usrLib.h"
#include "envLib.h"

#include <DextArm/At.h>
#include <DextArm/Ajc.h>
#include "SL.h"
#include "SL_motor_servo.h"

#define  TRUE  1
#define  FALSE 0

/* the keywords I am looking for */

#define N_KEYWORDS 9

char keywords[N_KEYWORDS][100] = {
  "version\0",
  "creation\0",
  "description\0",
  "robot\0",
  "purpose\0",
  "operator\0",
  "index\0",
  "registers\0",
  "end\0"};


/* note: the sequences MUST correspond to the SARCOS At.h file */

#define N_REGISTERS RegisterMAX

char registers[N_REGISTERS][100] = {
  "position.cross.gain\0",
  "torque.cross.gain\0",
  "position.error.gain\0",
  "velocity.error.gain\0",
  "torque.error.gain\0",
  "plus.velocity.gain\0",
  "actual.position.gain.one\0",
  "actual.position.bias.one\0",
  "actual.position.gain.two\0",
  "actual.position.bias.two\0",
  "actual.torque.gain\0",
  "actual.torque.bias.fine\0",
  "actual.torque.bias.course\0",
  "tap.gain\0",
  "tap.threshold\0",
  "valve.gain\0",
  "valve.command\0",
  "gravity.compensation\0",
  "desired.position\0",
  "desired.torque\0"};

char    *SARCOSDTSHOME;

/*!*****************************************************************************
 *******************************************************************************
\note  read_register_file
\date  10/20/91 
   
\remarks 

        reads in the register file an writes all values to the 
	registers on the robot. This program heavily depends on the
	correct keywords in the sarcos register file.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

      none

 ******************************************************************************/
int
read_register_file(char *fname_in)

{

  char *fname;
  char *longfname;
  char *path;
  char *line;
  FILE *infile,*dummyfile;
  int   c;
  int   nc;
  int   count_keywords;
  int   count_registers;
  int   count_joints;
  AjcRegister *ajc_registers;
  int   i,j,n;
  int   rc,start;
  int  **register_values;
  char *dummy;
  char *preferred_rf_file;
   AjcJoint ajc_joint[100];
   Error error;
   int   count_write;
   int   count_bad;
   char *aux_ptr;
   size_t number,length;
   char  joint_name[100];
   char  string[100];

   if (servo_enabled) {
     printf("Motor servo must not be running when loading registers\n");
     return FALSE;
   }

   length = sizeof(char);
   number = 1000;
   fname = (char *) calloc(number,length);
   longfname = (char *)calloc(number,length);
   path = (char *)calloc(number,length);
   dummy = (char *)calloc(number,length);
   preferred_rf_file = (char *)calloc(number,length);
   line = (char *)calloc(number,length);

   number = 100;
   length = sizeof(AjcRegister);
   ajc_registers = (AjcRegister *)calloc(number,length);

   number = 100;
   length = sizeof(int *);
   register_values = (int **)calloc(number,length);
   length = sizeof(int);
   for (i=0; i<100;++i) register_values[i] = (int *)calloc(number,length);


   /* try to locat register file by trying to read from preference 
      file */

   if (strcmp(fname_in,"default")==0) {

     sprintf(string,"%spreferred_register_file",PREFS);
     infile = fopen(string,"r");
     if (infile != NULL) {

       if (fscanf(infile,"%s",preferred_rf_file) == 1) {

	 strcpy(fname,preferred_rf_file);

       }

       fclose(infile);

     }

   } else {

     strcpy(preferred_rf_file,fname_in);
     strcpy(fname,fname_in);

   }

   /* get the filename for the register file */

   SARCOSDTSHOME = getenv("SARCOSDTSHOME");
   sprintf(path,"%s/register",SARCOSDTSHOME);
   printf("Set path to %s\n\n",path);

   if (strcmp(fname,preferred_rf_file) != 0) {

     printf("Type 'ls' to get directory list or 'quit' to quit\n");
     sprintf(fname,"animfigCalibration.rf");

   AGAIN:

     get_string("Input name of register file to download",fname,fname);

   }

   if (strcmp("ls",fname) == 0) {

     ls(path,FALSE); 
     sprintf(fname,"animfigRegistersCalibration.rf");

     goto AGAIN;

   } else if (strcmp("quit",fname) == 0) {

     return FALSE;

   }

   sprintf(longfname,"%s/%s",path,fname);
   printf("Reading file '%s' ...\n",fname);


   /* get a pointer for the register file */

   infile = fopen(longfname,"r");
   if (infile == NULL ) {

     printf("\nError: Cannot open file '%s'\n\n",longfname);

     goto AGAIN;

   }


   /* read the register file */

   /* ------------------------------------------------------------------*/
   /* first step: find out whether all keywords are satisfied */

   count_keywords = 0;
   c = '?';

   while (c != EOF) {

     nc = 0;
     while ((c = fgetc(infile)) != '\n') {

       if (c == EOF)  break;

       line[nc] = c;
       ++nc;

     }

     if (c == EOF) break;

     line[nc] = '\0';

     for (i=count_keywords; i< N_KEYWORDS; ++i) {
       if (strstr(line,keywords[i]) != NULL) {
	 if (count_keywords == 0) printf("\n\nFile Header:\n\n");
	 if (count_keywords < 6) printf("%s\n",line); 
	 ++count_keywords;
	 break;
       }
     }

   }

   rewind(infile);

   if (count_keywords != N_KEYWORDS) {
     printf("\nError: Invalid input file format\n\n");
     goto AGAIN;
   }


   /* ------------------------------------------------------------------*/
   /* second step: go to the index field and extract the registers which
      are included in this file */

   count_registers = 0;
   c = '?';
   start = FALSE;

   while (c != EOF) {

     nc = 0;
     while ((c = fgetc(infile)) != '\n') {

       if (c == EOF)  break;

       line[nc] = c;
       ++nc;

     }

     if (c == EOF) break;

     line[nc] = '\0';

     if (strstr(line,keywords[7]) != NULL) start = FALSE;

     if (start) {

       /* determine the current register */

       for (i=REGISTERSTART; i<RegisterMAX; ++i) {

	 if (strstr(line,registers[i-REGISTERSTART]) != NULL) {
	   ajc_registers[count_registers] = i;
	   ++count_registers;
	 }

       }

     }

     if (strstr(line,keywords[6]) != NULL) start = TRUE;

   }

   rewind(infile);


   printf("\nFound %d registers in file\n",count_registers);




   /* ------------------------------------------------------------------*/
   /* third: go to the registers field and extract the joints
      and register values  */

   count_joints = 0;
   c = '?';
   start = FALSE;

   while (c != EOF) {



     nc = 0;
     while ((c = fgetc(infile)) != '\n') {

       if (c == EOF)  break;

      line[nc] = c;
      ++nc;
      
    }
    
    if (c == EOF) break;
    
    line[nc] = '\0';


    
    if (strstr(line,keywords[8]) != NULL) start = FALSE;


    if (start) {

      /* extract the name of the joint and the register values */
      
      /* there is a tab between the joint specification  and the
	 first register: hope that argus continues this format */
      
      aux_ptr = strtok(line," ");
      if (aux_ptr == NULL) return (-999);
      sscanf(aux_ptr,"%s %d",
	     joint_name,&(register_values[count_joints][0]));
      
      for (n=1; n < count_registers; ++n) { 
	aux_ptr = strtok(NULL," ");
	if (aux_ptr == NULL) return (-999);
	sscanf(aux_ptr,"%d",&(register_values[count_joints][n])); 
      }
      
      /* get the ID of this joint */
      error = At_GetJointID ( joint_name, &(ajc_joint[count_joints] ));
      if (error) {
	printf("Invalid joint in register file: >%s<\n",joint_name);
      }

      ++count_joints;

    }
	    
    if (strstr(line,keywords[7]) != NULL) start = TRUE;
    
  }
  
  /* rm("temptemp"); */
  fclose(infile);
   

  printf("\nFound %d joints in file\n",count_registers);
  
  
  printf("\nStart downloading registers to robot(s) ...\n");


  count_write = 0;
  count_bad   = 0;

  for (i=0; i < count_joints; ++i) {
    for (j=0; j < count_registers; ++j) {

      ++count_write;
      error = Ajc_WriteFunction(1,PriorityHigh,&(ajc_joint[i]),ajc_registers[j],
			&(register_values[i][j]));
      if (error) {
	++count_bad;
	Error_Message(Error_Module(error),(char *)E_FAILURE);
	At_GetJointName ( ajc_joint[i], (const char **) &joint_name );
	printf("Error %ld for joint=%s, reg.= [%s], val.= %d\n",
	       error,joint_name,registers[ajc_registers[j]],register_values[i][j]);
      }
    }
  }

printf("%d registers written to Ajc boards, %d errors\n",
	 count_write,count_bad);

  free(fname);
  free(longfname);
  free(path);
  free(dummy);
  free(preferred_rf_file);
  free(line);
  free(ajc_registers);
  for (i=0; i<100;++i) free(register_values[i]);
  free(register_values);

  return TRUE; 
 
}

