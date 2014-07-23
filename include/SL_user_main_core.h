int i,j,n,c=0;
char argv_array[100][50];
char *argv_ptr[100];
int  graphics_flag = TRUE;
int  hold_flag = FALSE;
int  ros_flag = FALSE;
char string[100];
char xstring[100];
char gstring[100];
int  delta_width = 600;
int  geometry_argv;
int  servo_argv;
int  background_argv;
int  title_argv;
int  nice_argv;
int  stat_loc;
int  options = 0;
int  rc;
int  x,y,h,w;

Display *display;
int  screen_num;
int  display_width;
int  display_height;


// init the parameter pool
init_parameter_pool();

// connect to X server using the DISPLAY environment variable
if ( (display=XOpenDisplay(NULL)) == NULL ) {
  printf("Cannot connect to X servo %s\n",XDisplayName(NULL));
  exit(-1);
 }

// get screen size from display structure macro 
screen_num = DefaultScreen(display);
display_width = DisplayWidth(display, screen_num);
display_height = DisplayHeight(display, screen_num);

// assign the servo_name variable with the calling program as default
sprintf(servo_name,"%s",argv[0]);

// NOTE: the sequence of initialization of the servos
//       is important for the initial semaphore synchronization

// check for no-graphics flag
for (i=1; i<argc; ++i)
  if (strcmp(argv[i],"-ng")==0 ||  strcmp(argv[i],"-no-graphics")==0)
    graphics_flag = FALSE;

// check for hold flag
for (i=1; i<argc; ++i)
  if (strcmp(argv[i],"-hold")==0)
    hold_flag = TRUE;

// check for ros flag, but only if ROS_MASTER_URI is defined
if (getenv("ROS_MASTER_URI") != NULL)
  for (i=1; i<argc; ++i)
    if (strcmp(argv[i],"-ros")==0)
      ros_flag = TRUE;

// get the current process ID
parent_process_id = getpid();

// need the config file names initialized
setRealRobotOptions() ;

// build the command array
sprintf(argv_array[c++],"xterm");
sprintf(argv_array[c++],"-wf");
sprintf(argv_array[c++],"-leftbar");
sprintf(argv_array[c++],"-geometry");
geometry_argv = c;
sprintf(argv_array[c++],"90x10+0+0");
sprintf(argv_array[c++],"-bg");
background_argv = c;
sprintf(argv_array[c++],"red");
sprintf(argv_array[c++],"-fg");
sprintf(argv_array[c++],"black");
if (hold_flag)
  sprintf(argv_array[c++],"-hold");
sprintf(argv_array[c++],"-title");
title_argv = c;
sprintf(argv_array[c++],"%s",argv[0]);
sprintf(argv_array[c++],"-e");
sprintf(argv_array[c++],"env");
sprintf(argv_array[c++],"LD_LIBRARY_PATH=%s",getenv("LD_LIBRARY_PATH"));
sprintf(argv_array[c++],"nice");
sprintf(argv_array[c++],"-n");
nice_argv = c;
sprintf(argv_array[c++],"0");
servo_argv = c;
sprintf(argv_array[c++],"xdummy");
sprintf(argv_array[c++],"-pid");
sprintf(argv_array[c++],"%d",parent_process_id);
if (!graphics_flag) 
  sprintf(argv_array[c++],"-ng");


// detach process
#if sparc
#else
rc=daemon(1,0);
#endif

// initialize shared memories and shared semaphores
if (!init_shared_memory())
  exit(-1);

// signal handlers
installSignalHandlers();
  

