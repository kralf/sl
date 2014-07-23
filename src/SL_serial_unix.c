/*!=============================================================================
  ==============================================================================

  \file    SL_serial.c

  \author  Stefan Schaal
  \date    Oct 2011

  ==============================================================================
  \remarks

  generic routines for managing serial communiction

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "utility.h"
#include "SL_serial_unix.h"
#include "SL_man.h"
#include "termios.h"
#include "fcntl.h"
#include "sys/ioctl.h"
#include "unistd.h"





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
 \param[in]     baud  : baudrate (choose from termios.h, e.g., B38400)
 \param[in]     mode  : O_RDONLY or O_RDWR

 return serial_fd, i.e., the file descriptor, or FALSE

 ******************************************************************************/
int
open_serial(char *fname, int baud, int mode)
{
  int serial_fd;
  struct termios options;

  serial_fd = open( fname, mode  | O_NOCTTY | O_NDELAY );

  if (serial_fd == -1) {
    printf("Can't open serial port %s for mode %d\n",fname,mode);
    return serial_fd;
  }

  // get settings of the serial port
  tcgetattr(serial_fd, &options);

  // set baud rate
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // make this a raw communication
  cfmakeraw(&options);

  // and update the serial line
  tcsetattr(serial_fd, TCSANOW, &options);

  // clear the buffer
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

  if (!tcflush(fd,TCIOFLUSH))
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

  ioctl(fd,FIONREAD,&n_bytes);

  return n_bytes;
}

