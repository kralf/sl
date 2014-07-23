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
#include "SL_serial_xeno.h"
#include "SL_man.h"
#include "fcntl.h"
#include "unistd.h"
#include <rtdm/rtdm.h>
#include <rtdm/rtserial.h>

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
  struct rtser_config get_config;
  int rc;

  serial_fd = rt_dev_open( fname, mode  | O_NOCTTY | O_NDELAY );

  if (serial_fd < 0) {
    printf("Can't open serial port %s for mode %d (err=%d)\n",fname,mode,serial_fd);
    return FALSE;
  }

  // get settings of the serial port
  rc = rt_dev_ioctl(serial_fd, RTSER_RTIOC_GET_CONFIG, &get_config);
  if (rc) {
    printf("ERROR: cannot get RTSER_RTIOC_GET_CONFIG, status = %d, errno = %d, strerror = %s \n",
	   rc,errno,strerror(errno));
    return FALSE;
  }

  // set baud rate
  get_config.baud_rate = baud;

  // set parity and bits
  get_config.parity        = RTSER_NO_PARITY;
  get_config.data_bits     = RTSER_8_BITS;
  get_config.stop_bits     = RTSER_DEF_STOPB;
  get_config.rx_timeout    = 50*1000; // in ns
  get_config.tx_timeout    = 50*1000; // in ns
  get_config.event_timeout = 50*1000; // in ns
  get_config.event_mask    = RTSER_EVENT_RXPEND;

  get_config.config_mask = 
    RTSER_SET_BAUD | RTSER_SET_PARITY | RTSER_SET_DATA_BITS | RTSER_SET_STOP_BITS |
    RTSER_SET_TIMEOUT_RX | RTSER_SET_TIMEOUT_TX | RTSER_SET_TIMEOUT_EVENT | RTSER_SET_EVENT_MASK;

  // and update the serial line
  rc = rt_dev_ioctl(serial_fd, RTSER_RTIOC_SET_CONFIG, &get_config);
  if (rc) {
    printf("ERROR: cannot get RTSER_RTIOC_SET_CONFIG, status = %d, errno = %d, strerror = %s \n",
	   rc,errno,strerror(errno));
    return FALSE;
  }

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
  rt_dev_close(fd);
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
  int n_bytes;
  int n_bytes_read;
  char buf[10000];

  while(TRUE) {
    n_bytes = check_serial(fd);
    if (n_bytes > 10000) {
      n_bytes = 10000;
      n_bytes_read = read_serial(fd,n_bytes,buf);
    } else {
      n_bytes_read = read_serial(fd,n_bytes,buf);
      break;
    }
  }

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

  return rt_dev_read(fd, buffer, n_bytes);

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
  return rt_dev_write(fd, buffer, (size_t) n_bytes);
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
  int rc;
  struct rtser_event rx_event;

  rc = rt_dev_ioctl(fd, RTSER_RTIOC_WAIT_EVENT, &rx_event );
  if (rc) {
    if (rc != -ETIMEDOUT)
      printf("ERROR on RTSER_RTIOC_WAIT_EVENT, %s\n",strerror(-rc));
    return 0;
  }
  
  n_bytes = rx_event.rx_pending;

  return n_bytes;
}

