/*!=============================================================================
  ==============================================================================

  \file    SL_serial_xeno.h

  \author  Stefan Schaal
  \date    Sept. 2011

  ==============================================================================

  supports SL_serial_xeno.c

  ============================================================================*/


#ifndef _SL_serial_xeno_
#define _SL_serial_xeno_

#define BAUD9K    9600
#define BAUD19K   19200
#define BAUD38K   38400
#define BAUD115K  115200

#define SERIALPORT1 "rtser0"
#define SERIALPORT2 "rtser1"
#define SERIALPORT3 "rtser2"
#define SERIALPORT4 "rtser3"

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

#endif  /* _SL_serial_xeno_ */
