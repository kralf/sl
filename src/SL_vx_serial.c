/*!=============================================================================
  ==============================================================================

  \file    SL_vx_serial.c

  \author  Stefan Schaal
  \date    Oct. 2000

  ==============================================================================
  \remarks

  generic routines for managing serial communiction on vxWorks

  ============================================================================*/

#include "vxWorks.h"
#include "stdio.h"
#include "string.h"
#include "intLib.h"
#include "timers.h"
#include "ioLib.h"

/* private includes */
#include "SL_vx_serial.h"
#include "SL_man.h"

/* local variables */

/* global variables */

/* local functions */

/*!*****************************************************************************
 *******************************************************************************
\note  open_serial
\date  Oct 2000
   
\remarks 

        opens a specific serial port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : name of serial port
 \param[in]     baud  : baudrate
 \param[in]     mode  : 'rw' or 'ro'
 \param[in]     line  : FALSE/TRUE

     return serial_fd, i.e., the file descriptor, or FALSE

 ******************************************************************************/
int
open_serial(char *fname,int baud, char *mode, int line)
{
  int serial_fd=(int)NULL;

  if (strcmp("ro",mode)==0)
    serial_fd = open( fname, O_RDONLY, 666 );
  else if (strcmp("rw",mode)==0)
    serial_fd = open( fname, O_RDWR, 777 );

  if (serial_fd == (int)NULL) {
    printf("Can't open serial port %s for %s\n",fname,mode);
    return FALSE;
  }

  if (line) {
    if (ioctl( serial_fd, FIOSETOPTIONS, OPT_LINE ) == ERROR) {
      printf("Can't set serial port %s to LINE mode\n",fname);
      return FALSE;
    }
  } else {
    if (ioctl( serial_fd, FIOSETOPTIONS, OPT_RAW ) == ERROR) {
      printf("Can't set serial port %s to RAW mode\n",fname);
      return FALSE;
    }
  }

  if (ioctl( serial_fd, FIOBAUDRATE, baud ) == ERROR) {
    printf("Can't set serial port %s baud rate to %d\n",fname,baud);
    return FALSE;
  }
  
  clear_serial(serial_fd);

  return serial_fd;
  
}
/*!*****************************************************************************
 *******************************************************************************
\note  close_serial
\date  Oct 2000
   
\remarks 

        closes the serial connection

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor

 ******************************************************************************/
void
close_serial(int fd) 
{
  close(fd);
}

/*!*****************************************************************************
 *******************************************************************************
\note  clear_serial
\date  Oct 2000
   
\remarks 

        empties the serial buffer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor

 ******************************************************************************/
int
clear_serial(int fd) 
{
  if (ioctl(fd,FIOFLUSH,(int)NULL) == ERROR)
    return FALSE;

  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  read_serial
\date  Oct 2000
   
\remarks 

        reads into a buffer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor
 \param[in]     n_bytes: number of bytes to read
 \param[out]    buffer: buffer for read

     returns the number of bytes actually read

 ******************************************************************************/
int
read_serial(int fd,int n_bytes, char *buffer) 
{
  return read(fd, buffer, (size_t) n_bytes);
}

/*!*****************************************************************************
 *******************************************************************************
\note  write_serial
\date  Oct 2000
   
\remarks 

        write to the serial port

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor
 \param[in]     n_bytes: number of bytes to write
 \param[in]     buffer: buffer with bytes to write

     returns the number of bytes actually written

 ******************************************************************************/
int
write_serial(int fd,int n_bytes, char *buffer) 
{
  return write(fd, buffer, (size_t) n_bytes);
}

/*!*****************************************************************************
 *******************************************************************************
\note  check_serial
\date  Oct 2000
   
\remarks 

        checks the number of bytes in serial input buffer

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fd : file descriptor

     returns the number of bytes in buffer

 ******************************************************************************/
int
check_serial(int fd) 
{
  int n_bytes;

  ioctl(fd, FIONREAD, (int) (&n_bytes));

  return n_bytes;
}

