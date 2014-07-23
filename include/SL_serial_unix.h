/*!=============================================================================
  ==============================================================================

  \file    SL_serial_unix.h

  \author  Stefan Schaal
  \date    Sept. 2011

  ==============================================================================

  supports SL_serial_unix.c

  ============================================================================*/


#ifndef _SL_serial_unix_
#define _SL_serial_unix_

#include "termios.h"

#define BAUD9K    B9600
#define BAUD19K   B19200
#define BAUD38K   B38400
#define BAUD115K  B115200

#define SERIALPORT1 "/dev/ttyS0"
#define SERIALPORT2 "/dev/ttyS1"
#define SERIALPORT3 "/dev/ttyS2"
#define SERIALPORT4 "/dev/ttyS3"

#ifdef __cplusplus
extern "C" {
#endif

int  open_serial(char *fname,int baud, int mode);
void close_serial(int fd);
int  clear_serial(int fd);
int  read_serial(int fd,int n_bytes, char *buffer);
int  write_serial(int fd,int n_bytes, char *buffer);
int  check_serial(int fd);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_serial_unix_ */
