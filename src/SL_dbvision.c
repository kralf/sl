/*!=============================================================================
  ==============================================================================

  \file    SL_dbvision.c

  \author  Stefan Schaal
  \date    Oct. 2011

  ==============================================================================
  \remarks

  handles the I/O with the DBVision vision system of Ales Ude

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_vision_servo.h"
#ifdef __XENO__
#include "SL_serial_xeno.h"
#else
#include "SL_serial_unix.h"
#endif
#include "fcntl.h"

#define CAMERA_1  1
#define CAMERA_2  2
#define N_CAMERAS 2        /*!< binocular */

#define WAIT_IN_NS 100000

typedef struct Blob2DInfo {
  int status;
  int x;
  int y;
  int area;
  int left;
  int top;
  int right;
  int bottom;
  int aspect_ratio;
  int orient;
} Blob2DInfo;

typedef struct Frame {
  int counter;  /*!< the frame counter */
  Blob2DInfo **blobinfo;
} Frame;

static Frame           the_frame;
static int             last_counter;
static int             frame_counter;
static int             serial_fd;

// global variables

// local functions
static int  init_dbvision_interface(void);
static int  read_frame(char *,int );

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_hardware
\date  June 1999
   
\remarks 

        initializes communication with the DBVision system

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_vision_hardware(void)
{
  int i;

  printf("Initializing vision hardware ...");

  // allocate memory for the blobs
  the_frame.blobinfo = 
    my_calloc((max_blobs+1),sizeof(Blob2DInfo *),MY_STOP);
  
  for (i=1; i<=max_blobs; ++i)
    the_frame.blobinfo[i] = 
      my_calloc((N_CAMERAS+1),sizeof(Blob2DInfo),MY_STOP);
  
  // initialize the interface
  if (!init_dbvision_interface())
    return FALSE;

  printf("done\n");  

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  init_dbvision_interface
\date  Oct 2000
   
\remarks 

        initializes the interface, i.e., just a serial connection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int
init_dbvision_interface(void)
{

  serial_fd = open_serial(SERIALPORT1,BAUD115K,O_RDONLY);
  if (serial_fd < 0) {
    printf("Error when opening Serial Port\n");
    return FALSE;
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  acquire_blobs
\date  Oct 2000

\remarks 

read the current information about the blobs

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     blobs  : array of blob structures

 ******************************************************************************/
#define MAX_CHARS 10000
int
acquire_blobs(Blob2D blobs[][2+1])

{
  int    i,j,r,m;
  int    rc;
  static char buffer[MAX_CHARS];
  static int  n_buffer = 0; 
  char   buffer2[MAX_CHARS];
  int    n_buffer2 = 0;
  int    count = 0;
  int    run = TRUE;
  
  // reset all the blobs to be non existent
  for (i = 1; i<=max_blobs; ++i) {
    the_frame.blobinfo[i][CAMERA_1].status = FALSE;
    the_frame.blobinfo[i][CAMERA_2].status = FALSE;
  }

  while( run ) {  

    // check for data in the serial port buffer
    if ( (rc=check_serial(serial_fd)) > 0) {
      if (rc > MAX_CHARS-n_buffer) {
	rc = MAX_CHARS-n_buffer;
	printf("buffer overflow\n");
      }
      rc = read_serial(serial_fd,rc,&(buffer[n_buffer]));
      n_buffer += rc;
    }

    // look for a complete frame in the data
    for (j=n_buffer-1; j>=0; --j) {
      if (buffer[j] == '\n')  // found the end of a frame at j
	break;
    }

    for (i=j; i>=0; --i) {
      if (buffer[i] == '^') { // found the beginning of a frame at i
	run = FALSE;
	break;
      }
    }

    if (i<0 && j>=0) { // no beginning of frame found despite end of frame
      for (r = j+1; r<n_buffer; ++r)
	buffer[r-(j+1)] = buffer[r];
      n_buffer -= j+1;
      printf("discard early buffer\n");
    }

    if (!run)
      break;

    taskDelay(ns2ticks(WAIT_IN_NS));

    if (++count > 10000) {
      printf("Error: Timeout when reading from vision hardware\n");
      printf("Switch to no-hardware mode\n");
      no_hardware_flag = TRUE;
      return TRUE;
    }
  }

  // re-arrange the buffer
  for (r = i+1; r<j; ++r)
    buffer2[r-(i+1)] = buffer[r];
  buffer2[j-(i+1)] = '\0';
  n_buffer2 = j-(i+1);

  for (r = j+1; r<n_buffer; ++r)
    buffer[r-(j+1)] = buffer[r];
  n_buffer -= j+1;

  /* now we are at the beginning of a frame */
  last_counter = the_frame.counter;
  
   /* read the entire frame */
  rc = read_frame(buffer2,n_buffer2);

  ++frame_counter;

  if (vision_servo_calls < 10) {
    /* synchronize the v->frame_counter and the_frame.counter */
    frame_counter = the_frame.counter;
    last_counter = frame_counter - 1;
  }

  ++count_all_frames;
  
  /* frame_counter and the_frame.counter should always coincide,
     anything else counts as a lost frame */
  
  count_lost_frames += abs(frame_counter - the_frame.counter);
  frame_counter = the_frame.counter;

  /* was the frame counter advanced and was read_frame successful ?*/

  if ( (the_frame.counter == last_counter + 1) && rc ) {
    ;
  } else {
    
    /* reset all the blobs to be non existent */
    for (i = 1; i<=max_blobs; ++i) {
      the_frame.blobinfo[i][CAMERA_1].status = FALSE;
      the_frame.blobinfo[i][CAMERA_2].status = FALSE;
    }
    ++count_lost_frames;
  }
  
  /* copy the data into the global structures */

  for (i=1; i<=max_blobs; ++i) {
    /*
    if (the_frame.blobinfo[i][CAMERA_1].status &&
	the_frame.blobinfo[i][CAMERA_2].status) {

      blobs[i][CAMERA_1].status = blobs[i][CAMERA_2].status = TRUE;
      blobs[i][CAMERA_1].x[_X_] = the_frame.blobinfo[i][CAMERA_1].x;
      blobs[i][CAMERA_1].x[_Y_] = the_frame.blobinfo[i][CAMERA_1].y;
      blobs[i][CAMERA_2].x[_X_] = the_frame.blobinfo[i][CAMERA_2].x;
      blobs[i][CAMERA_2].x[_Y_] = the_frame.blobinfo[i][CAMERA_2].y;

    } else {

      blobs[i][CAMERA_1].status = blobs[i][CAMERA_2].status = FALSE;

      } */

    if (the_frame.blobinfo[i][CAMERA_1].status) {
      blobs[i][CAMERA_1].status = TRUE;
      blobs[i][CAMERA_1].x[_X_] = the_frame.blobinfo[i][CAMERA_1].x;
      blobs[i][CAMERA_1].x[_Y_] = the_frame.blobinfo[i][CAMERA_1].y;
    } else {
      blobs[i][CAMERA_1].status = FALSE;
    }
    if (the_frame.blobinfo[i][CAMERA_2].status) {
      blobs[i][CAMERA_2].status = TRUE;
      blobs[i][CAMERA_2].x[_X_] = the_frame.blobinfo[i][CAMERA_2].x;
      blobs[i][CAMERA_2].x[_Y_] = the_frame.blobinfo[i][CAMERA_2].y;
    } else {
      blobs[i][CAMERA_2].status = FALSE;
    }
  }

  return TRUE;

}  

/*!*****************************************************************************
 *******************************************************************************
\note  read_frame
\date  Oct 2000
   
\remarks 

        Reads a frame from the serial port. 
	Note: all date comes as ASCII characters!!! Weird .....

	The following characters demark the frame:
	start of frame   : ^
	end of frame     : \n
	start of camera 2: /
	end of channel   : ;
	end of number    : space
	
	For the following readings, the DBvision hardware needs to
	be set to DrScheme communication. This means:
	- report frame counter (otherwise this function reports errors)
	- only report x and y of the blobs, no other data
          (this could be changed if needed)
        - only allow one object per channel 
          (can be improved in the future, too, but requires to use
           temporal reasoning to avoid confusing two objects with
           the same color, and also occlusions)

	Currently, blob numbering coincides with the the color channel
        number.

        Thus, an empty frame would look like:
        1213;;;;/;;;;/

        A valid frame with one blob would looke like:
        4475460 356 269;;;;/238 174;;;;/

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     \param[in]  buffer   : charcter buffer with frame
     \param[in]  n_buffer : number of characters in frame

     returns the number of bytes read

 ******************************************************************************/
static int
read_frame(char *buffer, int n_buffer)

{
  int   i,j;
  int   start_camera[N_CAMERAS+1];
  char *startbuf;
  int   rc;
  char  store_char;
  int   iaux;
  char *cptr;

  /* where is the start of camera 2 data: at the first "/"  */
  for (j=0; j<n_buffer; ++j)
    if (buffer[j] == '/')
      break;

  if (j >= n_buffer)
    start_camera[CAMERA_2] = FALSE;
  else
    start_camera[CAMERA_2] = j;

  if (start_camera[CAMERA_2]==FALSE) {
    printf("Invalid Frame: / character not found: ");
    printf("Frame = >");
    for (i=0; i<n_buffer; ++i)
      printf("%c",buffer[i]);
    printf("<\n");
    return FALSE;
  }
  buffer[start_camera[CAMERA_2]] = '\0';
  ++start_camera[CAMERA_2];

  /* where is the start of camera 1 data: more tricky: it is 
     before the first ";", and after the first space, if blob1
     exists */
  for (j=0; j<n_buffer; ++j)
    if (buffer[j] == ';')
      break;

  if (j >= n_buffer) {
    printf("Invalid Frame: ; character not found: ");
    printf("Frame = >");
    for (i=0; i<n_buffer; ++i)
      printf("%c",buffer[i]);
    printf("<\n");
    return FALSE;
  } else
    iaux = j;

  for (j=0; j<n_buffer; ++j)
    if (buffer[j] == ' ')
      break;

  if (j >= n_buffer)
    start_camera[CAMERA_1]=iaux;
  else {
    start_camera[CAMERA_1]=j;
    if (iaux<start_camera[CAMERA_1])
      start_camera[CAMERA_1]=iaux;
  }
  store_char = buffer[start_camera[CAMERA_1]];
  buffer[start_camera[CAMERA_1]] = ' ';
  
  /* read the frame counter and the first blob of camera 1 */
  if (sscanf(buffer,"%d",&the_frame.counter) == 0) {
    printf("Invalid Frame: frame counter not found\n");
    return FALSE;
  }
  buffer[start_camera[CAMERA_1]] = store_char;

  /* loop through both cameras and parse the data */

  for (i=1; i<=N_CAMERAS; ++i) {
    startbuf=&(buffer[start_camera[i]]);
    for (j=1; j<=max_blobs; ++j) {

      /* find the next semicolon */
      cptr = strchr(startbuf,';');
      if (cptr==NULL)
	break;
      *cptr = '\0';

      /* read the data */
      if (sscanf(startbuf,"%d %d",&the_frame.blobinfo[j][i].x,
		 &the_frame.blobinfo[j][i].y) != 2)
	the_frame.blobinfo[j][i].status = FALSE;
      else
	the_frame.blobinfo[j][i].status = TRUE;

      startbuf = cptr+1;
	
    }
  }

  return TRUE;
  
}




