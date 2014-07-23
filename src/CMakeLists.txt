#############################################################################
#############################################################################
#
#  This is a CMakeList.txt file originally programmed for the CLMC/AMD labs
#  at the University of Southern California and the Max-Planck-Institute for
#  Intelligent Systems. We use a mixutre of explicit makefiles and cmake, but 
#  primarily we relay on cmake for all major compile dependencies. All our
#  software is provided under a slightly modified version of the LGPL license
#  to be found at http://www-clmc.usc.edu/software/license.
#
#  Copyright by Stefan Schaal, 2014
#
#############################################################################
#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# user defined cmake components

# set global compile type
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Optimization with debugging info
#set(CMAKE_BUILD_TYPE Release)       # Optimization 
#set(CMAKE_BUILD_TYPE Debug)         # Debug

# local defines
include_directories(BEFORE ../include)
include_directories(BEFORE ../src)


# ------------------------------------------------------------------------

set(SRCS_SL_COMMON
	SL_common.c
	SL_collect_data.c
	SL_shared_memory.c
	SL_filters.c
	SL_unix_common.c
	SL_terrains.c
	SL_oscilloscope.c
	SL_man.c 
	)	
if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_SL_COMMON
	SL_vx2xeno_wrappers.c
	SL_xeno_common.c
	${SRCS_SL_COMMON}
	)
else()	
  set(SRCS_SL_COMMON
	SL_vx2unix_wrappers.c
	${SRCS_SL_COMMON}
	)
endif()	


# ------------------------------------------------------------------------

set(SRCS_MOTOR_SERVO
	SL_controller.c
	SL_motor_servo.c
	SL_sensor_proc.c
	)

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_MOTOR_SERVO
	SL_motor_servo_xeno.c
	${SRCS_MOTOR_SERVO}
	)
else()
  set(SRCS_MOTOR_SERVO
	SL_motor_servo_unix.c
	${SRCS_MOTOR_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_TASK_SERVO
	SL_tasks.c 
	SL_task_servo.c 
	SL_goto_task.c
	SL_sine_task.c 
	SL_go_cart_task.c
	SL_objects.c
	SL_traj_task.c
	)

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_TASK_SERVO
	SL_task_servo_xeno.c
	${SRCS_TASK_SERVO}
	)
else()
  set(SRCS_TASK_SERVO
	SL_task_servo_unix.c
	${SRCS_TASK_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_SIM_SERVO
	SL_simulation_servo.c
	SL_userSimulation.c
	SL_objects.c
        SL_integrate.c
	) 

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_SIM_SERVO
	SL_simulation_servo_xeno.c
	${SRCS_SIM_SERVO}
	) 
else()
  set(SRCS_SIM_SERVO
	SL_simulation_servo_unix.c
	${SRCS_SIM_SERVO}
	) 
endif()

# ------------------------------------------------------------------------

set(SRCS_GL_SERVO
	SL_openGL_servo.c
	SL_openGL.c
	SL_openGL_oscilloscope.c
	SL_objects.c
	SL_userGraphics.c 
	)

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_GL_SERVO
	SL_openGL_servo_xeno.c
	${SRCS_GL_SERVO}
	)
else()
  set(SRCS_GL_SERVO
	SL_openGL_servo_unix.c
	${SRCS_GL_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_VISION_SERVO
	SL_vision_servo.c
	SL_dbvision.c
	SL_vision_proc.c
	) 

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_VISION_SERVO
	SL_vision_servo_xeno.c
	SL_serial_xeno.c
	${SRCS_VISION_SERVO}
	) 
else()
  set(SRCS_VISION_SERVO
	SL_vision_servo_unix.c
	SL_serial_unix.c
	${SRCS_VISION_SERVO}
	) 
endif()

# ------------------------------------------------------------------------

set(SRCS_ROS_SERVO
	SL_ros_servo.cpp
	SL_ros_communicator.cpp
	)

if($ENV{MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_ROS_SERVO
	SL_ros_servo_xeno.cpp
	${SRCS_ROS_SERVO}
	)
else()
  set(SRCS_ROS_SERVO
	SL_ros_servo_unix.cpp
	${SRCS_ROS_SERVO}
	)
endif()

set(ROS_PACKAGES roscpp sensor_msgs geometry_msgs)

# ------------------------------------------------------------------------

set(HEADERS
	  ../include/SL.h
	  ../include/SL_collect_data.h
	  ../include/SL_common.h 
	  ../include/SL_controller.h
	  ../include/SL_dynamics.h 
	  ../include/SL_filters.h 
	  ../include/SL_integrate.h
	  ../include/SL_kinematics.h 
	  ../include/SL_man.h
	  ../include/SL_motor_servo.h
	  ../include/SL_objects.h
	  ../include/SL_objects_defines.h
	  ../include/SL_openGL.h
	  ../include/SL_openGL_servo.h 
	  ../include/SL_oscilloscope.h
	  ../include/SL_ros_servo.h
	  ../include/SL_rt_mutex.h
	  ../include/SL_sensor_proc.h
	  ../include/SL_shared_memory.h
	  ../include/SL_simulation_servo.h
	  ../include/SL_system_headers.h
	  ../include/SL_task_servo.h
	  ../include/SL_tasks.h
	  ../include/SL_terrains.h
	  ../include/SL_unix_common.h
	  ../include/SL_userGraphics.h
	  ../include/SL_userSimulation.h
	  ../include/SL_user_common.h
	  ../include/SL_user_display_core.h
	  ../include/SL_user_idle_core.h
	  ../include/SL_user_main_core.h
	  ../include/SL_vision_servo.h
	  ../include/SL_serial_unix.h
	  ../include/SL_serial_xeno.h
	  ../include/SL_vx_wrappers.h
	  ../include/SL_xeno_common.h
	  ../include/SL_xeno_headers.h 
	)

# ------------------------------------------------------------------------

add_library(SLcommon ${SRCS_SL_COMMON})
add_library(SLtask ${SRCS_TASK_SERVO})
add_library(SLmotor ${SRCS_MOTOR_SERVO})
add_library(SLsimulation ${SRCS_SIM_SERVO})
add_library(SLopenGL ${SRCS_GL_SERVO})
add_library(SLvision ${SRCS_VISION_SERVO})

install(TARGETS SLcommon ARCHIVE DESTINATION ${LAB_LIBDIR})
install(TARGETS SLtask ARCHIVE DESTINATION ${LAB_LIBDIR})
install(TARGETS SLmotor ARCHIVE DESTINATION ${LAB_LIBDIR})
install(TARGETS SLsimulation ARCHIVE DESTINATION ${LAB_LIBDIR})
install(TARGETS SLopenGL ARCHIVE DESTINATION ${LAB_LIBDIR})
install(TARGETS SLvision ARCHIVE DESTINATION ${LAB_LIBDIR})

if(DEFINED ENV{ROS_ROOT})
  add_library(SLros ${SRCS_ROS_SERVO})
  install(TARGETS SLros ARCHIVE DESTINATION ${LAB_LIBDIR})
endif()


install(FILES ${HEADERS} DESTINATION ${LAB_INCLUDES})

