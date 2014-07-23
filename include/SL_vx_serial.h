/*!=============================================================================
  ==============================================================================

  \file    SL_vx_serial.h

  \author  Stefan Schaal
  \date    Oct. 2000

  ==============================================================================

      function for serial port communication

  ============================================================================*/


#ifndef _SL_vx_serial_
#define _SL_vx_serial_

#define BAUD9K    9600
#define BAUD19K   19200
#define BAUD38K   38400
#define BAUD115K  115200

#define SERIALPORT1 "/tyCo/0"
#define SERIALPORT2 "/tyCo/1"
#define SERIALPORT3 "/tyCo/2"
#define SERIALPORT4 "/tyCo/3"

#ifdef __cplusplus
extern "C" {
#endif

int  open_serial(char *fname,int baud,char *mode, int line);
void close_serial(int fd);
int  clear_serial(int fd);
int  read_serial(int fd,int n_bytes, char *buffer);
int  write_serial(int fd,int n_bytes, char *buffer);
int  check_serial(int fd);

#ifdef __cplusplus
}
#endif

#endif  /* _SL_vx_serial_ */
