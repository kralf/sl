/*!=============================================================================
  ==============================================================================

  \ingroup SLvision

  \file    SL_vision_servo.c

  \author  Stefan Schaal
  \date    June 1999

  ==============================================================================
  \remarks


  the (open loop) vision information. Since there is no data
  buffering in the QuickMag, it is necessary to poll the QuickMag
  connection all the time. The 712 transition module of the 
  QuickMag is used for communication.

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_vision_servo.h"
#include "SL_shared_memory.h"
#include "SL_oscilloscope.h"
#include "lwpr.h"
#include "SL_man.h"
#include "SL_common.h"
#include "SL_unix_common.h"

#define TIME_OUT_NS  100000

/* global variables */
int           servo_enabled;
double        servo_time;
double        vision_servo_time;
double        last_vision_servo_time;
int           vision_servo_rate = R60HZ;
int           vision_servo_calls;
char          current_pp_name[100];
int           vision_servo_initialized = FALSE;
int           vision_servo_errors;
int           count_all_frames;
int           count_lost_frames;
int           *blob_is_endeffector;
int           learn_transformation_flag = FALSE;
int           raw_blob_overwrite_flag=FALSE;
int           no_hardware_flag = FALSE;

/* local variables */
static int models_read = FALSE;

/* local functions */
static int  broadcast_blobs(void);
static int  receive_cartesian(void);
static int  learn_transformation(void );
static int  check_raw_blob_overwrite(void);
static int  checkForMessages(void);
static void reset_hardware_flag(void);
static int  visualize_active_blobs(void);



/* global functions */
void reset_learning( void );
void save_learning( void );
void set_learning( void );
void status(void);
int  stop(char *msg);



/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_servo
\date  June 1999
   
\remarks 

        initializes vision servo structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
init_vision_servo()
{
  int  j, i, k;
  char string[100];
  static int firsttime = TRUE;
  extern int  init_user_commands(void);
  
  if (vision_servo_initialized) {
    printf("Vision Servo is already initialized\n");
    return;
  }
  
  if (firsttime) {
    firsttime = FALSE;
    blob_is_endeffector = my_ivector(1,max_blobs);
  }

  /* the servo name */
  sprintf(servo_name,"vision");

  /* init the color vision */
  if (!init_vision_hardware())
    no_hardware_flag = TRUE;

  /* initialize shared memories and shared semaphores */
  if (!init_shared_memory())
    return;

   /* initialize system commands and user commands */
  if (!init_commands())
    return;

  if (!init_user_commands())
    return;

  /* init the vision processing */
  if (!init_vision_processing())
    return;

  /* init the coordinate transformations models */
  if (!init_learning())
    return;

  /* add variables to data collection */
  initCollectData(vision_servo_rate);

  for (i=1; i<=max_blobs; ++i) {
    printf("%d...",i);
    fflush(stdout);
    sprintf(string,"%s_stat",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_x",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_y",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_z",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.x[_Z_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_xd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_X_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_yd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Y_]),string,"m/s", DOUBLE,FALSE);
    sprintf(string,"%s_zd",blob_names[i]);
    addVarToCollect((char *)&(blobs[i].blob.xd[_Z_]),string,"m/s", DOUBLE,FALSE);

    sprintf(string,"%s_rstat1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry1",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][1].x[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_rstat2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].status),string,"-",CHAR,FALSE);
    sprintf(string,"%s_rx2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_ry2",blob_names[i]);
    addVarToCollect((char *)&(raw_blobs2D[i][2].x[_Y_]),string,"m", DOUBLE,FALSE);

  }

  addVarToCollect((char *)&(count_all_frames),"frame_counter","-",INT,FALSE);

  printf("done\n");

  updateDataCollectScript();

  // add to man pages 
  addToMan("dvs","disables the vision servo",dvs);
  addToMan("status","displays status information about servo",status);
  addToMan("reset_hardware_flag","resets the no-hardware flag",reset_hardware_flag);
  addToMan("reset_learning","resets the vision->3D models",reset_learning);
  addToMan("save_learning","saves learning models to disk",save_learning);
  addToMan("set_learning","allows selecting which blobs are used by learning",set_learning);

  /* initialize user specific things */
  if (!init_user_vision())
    return;

  /* if all worked out, we mark the servo as ready to go */
  vision_servo_initialized = TRUE;

  /* set oscilloscope to start value */  
  initOsc();
  setOsc(d2a_cv,0.0);
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  run_vision_servo
\date  June 1999
   
\remarks 

        executes the sequence of vision servo routines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
run_vision_servo(void)

{

  int i,j;
  int dticks;


  /*************************************************************************
   *  adjust time
   */
  
  ++vision_servo_calls;
  vision_servo_time += 1./(double)vision_servo_rate;
  servo_time = vision_servo_time;

  // check for missed calls to the servo
  dticks = round((vision_servo_time - last_vision_servo_time)*(double)vision_servo_rate);
  if (dticks != 1 && vision_servo_calls > 2) // need transient ticks to sync servos
    vision_servo_errors += abs(dticks-1);

  /*************************************************************************
   *  check for messages
   */
  
  checkForMessages();

  /*************************************************************************
   * this allows to overwrite the blobs, e.g., by simulated information
   */
  
  setOsc(d2a_cv,00.0);

  /* reset the blob status if there is no hardware */
  if (no_hardware_flag) {
    for (i=1; i<=max_blobs; ++i) {
      raw_blobs2D[i][1].status = 0;
      raw_blobs2D[i][2].status = 0;
    }
    count_all_frames = vision_servo_calls;
  }
  raw_blob_overwrite_flag = check_raw_blob_overwrite();
  
  /*************************************************************************
   * process the blobs (filtering, conversion into our coordinates
   */
  
  setOsc(d2a_cv,50.0);
  process_blobs(raw_blobs2D);
  
  /*************************************************************************
   * broadcast the the final blob information in shared memory
   */
  
  setOsc(d2a_cv,60.0);
  broadcast_blobs();
  
  /*************************************************************************
   * read the robot state
   */
  
  setOsc(d2a_cv,70.0);
  receive_cartesian();
  
  /*************************************************************************
   * learn the mapping
   */
  
  setOsc(d2a_cv,80.0);
  learn_transformation();
  
  /*************************************************************************
   * collect data
   */
  
  setOsc(d2a_cv,100.0);
  writeToBuffer();
  sendOscilloscopeData();

  /*************************************************************************
   * visuzalize active blobs
   */
  
  visualize_active_blobs();
  
  /*************************************************************************
   * end of program sequence
   */

  last_vision_servo_time = vision_servo_time;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  broadcast_blobs
\date  April 1999
   
\remarks 

        copies filtered blob information to shared memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
broadcast_blobs(void)
{
  static int count = 0;
  int i,j,k;
  char string[40];
  
  if (semTake(sm_vision_blobs_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++vision_servo_errors;
    
  } else {

    cSL_VisionBlob(blobs,sm_vision_blobs_data,max_blobs,DOUBLE2FLOAT);

    cSL_VisionBlobaux(raw_blobs2D,sm_vision_blobs_data_aux,
		      max_blobs,DOUBLE2FLOAT);
    
    for (i=1; i<=max_blobs; ++i) {
      if (sm_vision_blobs_data[i].status) {
	sm_vision_blobs->blobs[i] = sm_vision_blobs_data[i];
      }
      else
	sm_vision_blobs->blobs[i].status = sm_vision_blobs_data[i].status;

      sm_vision_blobs_aux->blobs[i] = sm_vision_blobs_data_aux[i];
    }
    
    /* check the post processing specifier */
    strcpy(string,sm_vision_blobs->pp_name);
    sm_vision_blobs->frame_counter = count_all_frames;
    if (strcmp(string,current_pp_name) != 0 && strcmp(string,"") != 0) {
      init_pp(string);
    }
    semGive(sm_vision_blobs_sem);

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_cartesian
\date  April 1999
   
\remarks 

        receives new sensory data 
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_cartesian(void)
{
  
  int i;
  int dticks;
  double ts;

  if (semTake(sm_cart_states_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++vision_servo_errors;
    return FALSE;
  } 

  memcpy((void *)(&sm_cart_states_data[1]),(const void*)(&sm_cart_states->state[1]),
	 sizeof(SL_fCstate)*n_endeffs);

  cSL_Cstate(cart_state,sm_cart_states_data,n_endeffs,FLOAT2DOUBLE);

  // get time stamp and adjust local time
  vision_servo_time = servo_time = sm_cart_states->ts;
  
  semGive(sm_cart_states_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  check_raw_blob_overwrite
\date  April 1999
   
\remarks 

        if semaphore is given, overwrite the raw blob information
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     returns TRUE only if the 3D blobs were overwritten -- for 2D blobs,
     no special action needs to be taken as processing just proceeds from
     this fake visual information

 ******************************************************************************/
static int 
check_raw_blob_overwrite(void)
{
  
  int i;
  static int last_frame_counter=0;
  int rc = FALSE;

  /* check whether there is an overwrite request */
  if (semTake(sm_raw_blobs_ready_sem,NO_WAIT) == ERROR) {
    return FALSE;
  } 

  /* check for 3D blob information */
  if (semTake(sm_raw_blobs_sem,ns2ticks(TIME_OUT_NS)) != ERROR) {
    
    for (i=1; i<=max_blobs; ++i) 
      if (sm_raw_blobs->blobs[i].status) {
	sm_raw_blobs_data[i] = sm_raw_blobs->blobs[i];
      }
    
    cBlob3D(raw_blobs,sm_raw_blobs_data,max_blobs,FLOAT2DOUBLE);
    
    if (count_all_frames > last_frame_counter) {
      last_frame_counter = count_all_frames;
    } else {
      ++count_all_frames; /* no vision input at the moment -> fake it */
      last_frame_counter = count_all_frames;
    }
  
    semGive(sm_raw_blobs_sem);

    rc = TRUE;
    
  } 

  /* check for 2D blob information */
  if (semTake(sm_raw_blobs2D_sem,ns2ticks(TIME_OUT_NS)) != ERROR) {

    for (i=1; i<=max_blobs*2; ++i) 
      if (sm_raw_blobs2D->blobs[i].status) {
	sm_raw_blobs2D_data[i] = sm_raw_blobs2D->blobs[i];
      }
    
    cBlob2D(raw_blobs2D,sm_raw_blobs2D_data,max_blobs,FLOAT2DOUBLE);
    
    if (count_all_frames > last_frame_counter) {
      last_frame_counter = count_all_frames;
    } else {
      if (!rc) { /* don't increment if already done for 3D blobs */
	++count_all_frames; /* no vision input at the moment -> fake it */
	last_frame_counter = count_all_frames;
      }
    }
    
    semGive(sm_raw_blobs2D_sem);

  } 



  return rc;
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_learning
\date  Feb 1999
\remarks 

initializes the learning models

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_learning( void )

{
  int i, j;

  if (models_read) {
    ;
  } else {
    if (!readLWPRScript(BLOB2ROBOT_SCRIPT,0,BLOB2ROBOT)) {
      printf("Error when reading script file >%s<\n",BLOB2ROBOT_SCRIPT);
      return FALSE;
    }
  }

  for (i=1; i<=max_blobs; ++i) {
    blob_is_endeffector[i] = FALSE;
  }

  models_read = TRUE;

  return TRUE;
  
}    

/*!*****************************************************************************
 *******************************************************************************
\note  reset_learning
\date  Feb 1999
\remarks 

resets the learning model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
reset_learning( void )

{
  int i, j;



  // make sure not to intefer with servo running
  sl_rt_mutex_lock(&mutex1);

  if (models_read) {
    deleteLWPR(BLOB2ROBOT);
  }
  if (!readLWPRScript(BLOB2ROBOT_SCRIPT,1,BLOB2ROBOT)) {

      printf("Error when reading script file >%s<\n",BLOB2ROBOT_SCRIPT);

  } else {

    for (i=1; i<=max_blobs; ++i) 
      blob_is_endeffector[i] = FALSE;

    models_read = TRUE;

    printf("Learning models were reset\n");
    
  }


  sl_rt_mutex_unlock(&mutex1);

}    

/*!*****************************************************************************
 *******************************************************************************
\note  save_learning
\date  Feb 1999
\remarks 

saves the learned model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
save_learning( void )

{
  int i, j;

  if (models_read) {
    writeLWPR(BLOB2ROBOT);
  }

}    

/*!*****************************************************************************
 *******************************************************************************
\note  set_learning
\date  Feb 1999
\remarks 

allows setting which endeffector corresponds to which blob

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
set_learning( void )

{
  int  i,j;
  char string[100];
  
  for (i=1; i<=max_blobs; ++i) {
    sprintf(string,"Which Endeffector Corresponds to %s",blob_names[i]);
    get_int(string, blob_is_endeffector[i],&(blob_is_endeffector[i]));
    if (blob_is_endeffector[i] < 1 || blob_is_endeffector[i] > n_endeffs)
      blob_is_endeffector[i] = FALSE;
  }
  
}    



/*!*****************************************************************************
 *******************************************************************************
\note  learn_transformation
\date  Feb 1999
\remarks 

learns the quickmag blob to robot cartesian space mapping

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
static int
learn_transformation(void )

{
  int  i,j;
  char string[100];
  double x[2+2+1];
  double y[N_CART+1];
  int    rfID;

  // the transformation can only be learned on the pysical robot
  if (!real_robot_flag)
    return TRUE;
  
  if (semTake(sm_learn_blob2body_sem,NO_WAIT) == ERROR)
    return TRUE;

  for (i=1; i<=max_blobs; ++i) {
    if (blob_is_endeffector[i] != 0 && 
	raw_blobs2D[i][1].status &&
	raw_blobs2D[i][2].status ) {

      x[1] = raw_blobs2D[i][1].x[1];
      x[2] = raw_blobs2D[i][1].x[2];
      x[3] = raw_blobs2D[i][2].x[1];
      x[4] = raw_blobs2D[i][2].x[2];

      for (j=_X_ ; j<= _Z_; ++j) {
	y[j] = cart_state[blob_is_endeffector[i]].x[j];
      }
      addDataToLWPR(BLOB2ROBOT, x, x, y,FALSE,&rfID);
    }
  }

  return TRUE;
  
}    



/*!*****************************************************************************
 *******************************************************************************
\note  dvs & disable_vision_servo
\date  April 1999
   
\remarks 

        disables the vision servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void 
dvs(void)
{
  disable_vision_servo();
}

void 
disable_vision_servo(void)
{
  int j;

  if ( servo_enabled == 1 )   {

    servo_enabled = 0;
    printf("Vision Servo Terminated\n");

    exit(1);
    
  } else
    fprintf( stderr, "vision servo is not on!\n" );
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  status
\date  August 7, 1992
   
\remarks 

        prints out all important variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
status(void)
{

  printf("\n");
  printf("            Time                   = %f\n",vision_servo_time);
  printf("            Servo Calls            = %d\n",vision_servo_calls);
  printf("            Servo Rate             = %d\n",vision_servo_rate);
  printf("            Servo Errors           = %d\n",vision_servo_errors);
  printf("            Servo Initialize       = %d\n",vision_servo_initialized);
  printf("            Servo Running          = %d\n",servo_enabled);
  printf("           #frames read            = %d\n",count_all_frames);
  printf("           #frames lost            = %d\n",count_lost_frames);
  printf("            vision_pp              = %s\n",current_pp_name);
  printf("            No Hardware Flag       = %d\n",no_hardware_flag);
#ifdef __XENO__
  extern long count_xenomai_mode_switches;
  extern int  delay_ns;
  printf("            Xeonmai Mode Swiches   = %ld\n",count_xenomai_mode_switches);
  printf("            Delay [ns]             = %d\n",delay_ns);
#endif
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  stop
\date  August 7, 1992 
   
\remarks 

       stops ongoing processing on this servo

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

    none

 ******************************************************************************/
int
stop(char *msg)
{

  int i;

  dvs();
  beep(1);
  printf("%s\n",msg);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  checkForMessages
\date  Nov. 2007
   
\remarks 

Messages can be given to the servo for hard-coded tasks.This allows
some information passing between the different processes on variables
of common interest, e.g., the endeffector specs, object information,
etc.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int
checkForMessages(void)
{
  int i,j;
  char name[20];

  // check whether a message is available
  if (semTake(sm_vision_message_ready_sem,NO_WAIT) == ERROR)
    return FALSE;


  // receive the message
  if (semTake(sm_vision_message_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++vision_servo_errors;
    printf("Couldn't take task message semaphore\n");
    return FALSE;
  }

  for (i=1; i<=sm_vision_message->n_msgs; ++i) {

    // get the name of this message
    strcpy(name,sm_vision_message->name[i]);

    // act according to the message name

    // ---------------------------------------------------------------------------
    if (strcmp(name,"status") == 0) { 

      status();

    }


  }

  // give back semaphore
  sm_vision_message->n_msgs = 0;
  sm_vision_message->n_bytes_used = 0;
  semGive(sm_vision_message_sem);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  reset_hardware_flag
\date  Oct 2011
   
\remarks 

 reset the no_hardware_flag to FALSE, such that the vision system can be used
 if it spits out data

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static void
reset_hardware_flag(void )
{
  no_hardware_flag = FALSE;
  printf("no_hardware_flag is reset to FALSE\n");
}

/*!*****************************************************************************
 *******************************************************************************
\note  visualize_active_blobs
\date  Oct 2011
   
\remarks 

 send all active blob positions to openGl for visualization
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
static int 
visualize_active_blobs(void)
{
  int i,j,k;
  struct {
    char     name[20];
    float    blob[N_CART+1];
  } blob_data;

  for (i=1; i<=max_blobs; ++i) {
    if (blobs[i].status == TRUE) {
      sprintf(blob_data.name,"blob-%d",i);
      for (j=1; j<=N_CART; ++j)
	blob_data.blob[j] = blobs[i].blob.x[j];

      sendUserGraphics(blob_data.name,&blob_data,sizeof(blob_data));      
    }
  }
    
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  sendUserGraphics
\date  Nov. 2007
   
\remarks 

      sends out information for user graphics to shared memory

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name   : name of graphics
 \param[in]     buf    : byte buffer with information
 \param[in]     n_bytes: number of bytest in buffer

 ******************************************************************************/
int
sendUserGraphics(char *name, void *buf, int n_bytes)
{
  int i,j;
  
  // send the user graphics data
  if (semTake(sm_user_graphics_sem,ns2ticks(1000000)) == ERROR) {
    return FALSE;
  }

  // first, check whether there is an older graphics object with the same
  // name -- it will be overwritten, as only the latest graphics objects
  // matters for visualization
  for (i=1; i<=sm_user_graphics->n_entries; ++i) {
    if (strcmp(name,sm_user_graphics->name[i]) == 0) {

      // just overwrite the data
      memcpy(sm_user_graphics->buf+sm_user_graphics->moff[i],buf,n_bytes);

      // give semaphores
      semGive(sm_user_graphics_sem);
      semGive(sm_user_graphics_ready_sem);

      return TRUE;

    }
  }

  // make sure the pointer offset buffer is correct
  if (sm_user_graphics->n_entries == 0) {
    sm_user_graphics->moff[1] = 0;
    sm_user_graphics->n_bytes_used = 0;
  }

  // check whether there is space for the graphics object
  if (sm_user_graphics->n_entries >= MAX_N_MESSAGES) {
    printf("User graphics buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  if (MAX_BYTES_USER_GRAPHICS-sm_user_graphics->n_bytes_used < n_bytes) {
    printf("User Graphics memory buffer exhausted in sendUserGraphics\n");
    semGive(sm_user_graphics_sem);
    return FALSE;
  }
  
  // update the logistics
  ++sm_user_graphics->n_entries;
  sm_user_graphics->n_bytes_used += n_bytes;
  
  // specify the name and info of this entry
  strcpy(sm_user_graphics->name[sm_user_graphics->n_entries],name);
  memcpy(sm_user_graphics->buf+sm_user_graphics->moff[sm_user_graphics->n_entries],buf,n_bytes);
  
  // prepare pointer buffer for next entry
  if (sm_user_graphics->n_entries < MAX_N_MESSAGES)
    sm_user_graphics->moff[sm_user_graphics->n_entries+1]=
      sm_user_graphics->moff[sm_user_graphics->n_entries]+n_bytes;

  // give semaphores
  semGive(sm_user_graphics_sem);
  semGive(sm_user_graphics_ready_sem);
  
  return TRUE;
}
