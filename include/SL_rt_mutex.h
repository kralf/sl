/*!=============================================================================
  ==============================================================================

  \file    SL_rt_mutex.h

  \author  Mrinal Kalakrishnan
  \date    Nov. 2010

  ==============================================================================
  \remarks

  This file defines a type "sl_rt_mutex", which is a wrapper for a real-time mutex.
  It uses Xenomai mutexes on the Xenomai RTOS, and pthread mutexes otherwise.
  We attempt to mimic the pthread_mutex_* api.

  Pthread condition functions are also wrapped as sl_rt_cond_*.

  Using this wrappers turned out to be more stable than using the xenomai
  posix-skin, which caused problems in 64-bit Xenomai.

  TODOs:
  Error codes are not converted yet. 
  Calls with timeout values are not implemented yet.

  ============================================================================*/

#ifndef SL_RT_MUTEX_H_
#define SL_RT_MUTEX_H_

#define SL_RT_MUTEX_WARNINGS 1

#include <stdio.h>

#ifdef __XENO__
#include <native/mutex.h>
#include <native/cond.h>
#include <native/timer.h>
#else
#include <pthread.h>
#include <sys/time.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __XENO__
  typedef RT_MUTEX sl_rt_mutex;
  typedef RT_COND sl_rt_cond;
#else
  typedef pthread_mutex_t sl_rt_mutex;
  typedef pthread_cond_t sl_rt_cond;
#endif

typedef unsigned long long sl_rt_time; //time in nanoseconds

static int sl_rt_mutex_init(sl_rt_mutex* mutex);
static int sl_rt_mutex_destroy(sl_rt_mutex* mutex);
static int sl_rt_mutex_lock(sl_rt_mutex* mutex);
static int sl_rt_mutex_trylock(sl_rt_mutex* mutex);
static int sl_rt_mutex_unlock(sl_rt_mutex* mutex);

static int sl_rt_cond_init(sl_rt_cond* cond);
static int sl_rt_cond_destroy(sl_rt_cond* cond);
static int sl_rt_cond_signal(sl_rt_cond* cond);
static int sl_rt_cond_broadcast(sl_rt_cond* cond);
static int sl_rt_cond_wait(sl_rt_cond* cond, sl_rt_mutex* mutex);
static int sl_rt_cond_timedwait(sl_rt_cond* cond, sl_rt_mutex* mutex, sl_rt_time timeout);
static int sl_rt_cond_timedwait_relative(sl_rt_cond* cond, sl_rt_mutex* mutex, sl_rt_time timeout);

static void sl_rt_warning(char* function_name, int error_code);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////////
// inline function definitions follow:

static inline int sl_rt_mutex_init(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_mutex_create(mutex, NULL);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_mutex_create", res);
  return res;
#else
  return pthread_mutex_init(mutex, NULL);
#endif
}

static inline int sl_rt_mutex_destroy(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_mutex_delete(mutex);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_mutex_delete", res);
  return res;
#else
  return pthread_mutex_destroy(mutex);
#endif
}

static inline int sl_rt_mutex_lock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_mutex_acquire(mutex, TM_INFINITE);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_mutex_acquire", res);
  return res;
#else
  return pthread_mutex_lock(mutex);
#endif
}

static inline int sl_rt_mutex_trylock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_mutex_acquire(mutex, TM_NONBLOCK);
  if (SL_RT_MUTEX_WARNINGS && res && res !=-EWOULDBLOCK)
    sl_rt_warning("rt_mutex_acquire", res);
  return res;
#else
  return pthread_mutex_trylock(mutex);
#endif
}

static inline int sl_rt_mutex_unlock(sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_mutex_release(mutex);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_mutex_unlock", res);
  return res;
#else
  return pthread_mutex_unlock(mutex);
#endif
}

static inline int sl_rt_cond_init(sl_rt_cond* cond)
{
#ifdef __XENO__
  int res = rt_cond_create(cond, NULL);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_cond_create", res);
  return res;
#else
  return pthread_cond_init(cond, NULL);
#endif
}

static inline int sl_rt_cond_destroy(sl_rt_cond* cond)
{
#ifdef __XENO__
  int res = rt_cond_delete(cond);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_cond_delete", res);
  return res;
#else
  return pthread_cond_destroy(cond);
#endif
}

static inline int sl_rt_cond_signal(sl_rt_cond* cond)
{
#ifdef __XENO__
  int res = rt_cond_signal(cond);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_cond_signal", res);
  return res;
#else
  return pthread_cond_signal(cond);
#endif
}

static inline int sl_rt_cond_broadcast(sl_rt_cond* cond)
{
#ifdef __XENO__
  int res = rt_cond_broadcast(cond);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_cond_signal", res);
  return res;
#else
  return pthread_cond_broadcast(cond);
#endif
}

static inline int sl_rt_cond_wait(sl_rt_cond* cond, sl_rt_mutex* mutex)
{
#ifdef __XENO__
  int res = rt_cond_wait(cond, mutex, TM_INFINITE);
  if (SL_RT_MUTEX_WARNINGS && res)
    sl_rt_warning("rt_cond_wait", res);
  return res;
#else
  return pthread_cond_wait(cond, mutex);
#endif
}

static inline int sl_rt_cond_timedwait(sl_rt_cond* cond, sl_rt_mutex* mutex, sl_rt_time timeout)
{
#ifdef __XENO__
  int res = rt_cond_wait_until(cond, mutex, timeout);
  if (SL_RT_MUTEX_WARNINGS && res && res!=-ETIMEDOUT)
    sl_rt_warning("rt_cond_wait_until", res);
  return res;
#else
  struct timespec ts;
  ts.tv_sec = (time_t) (timeout / 1000000000);
  ts.tv_nsec = (long) (timeout % 1000000000);
  return pthread_cond_timedwait(cond, mutex, &ts);
#endif
}

static inline int sl_rt_cond_timedwait_relative(sl_rt_cond* cond, sl_rt_mutex* mutex, sl_rt_time timeout)
{
#ifdef __XENO__
  RTIME abs_timeout = rt_timer_read() + timeout;
  return sl_rt_cond_timedwait(cond, mutex, abs_timeout);
#else
  struct timeval t;
  gettimeofday(&t, NULL);

  struct timespec ts;
  ts.tv_sec = (time_t) (timeout / 1000000000);
  ts.tv_nsec = (long) (timeout % 1000000000);

  ts.tv_sec += t.tv_sec;
  ts.tv_nsec += t.tv_usec*1000;
  if (ts.tv_nsec >= 1000000000)
  {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000;
  }
  return pthread_cond_timedwait(cond, mutex, &ts);

#endif
}

static inline void sl_rt_warning(char* function_name, int error_code)
{
  //char error_str[1000];
  //strerror_r(error_code, error_str, 1000);
  printf("ERROR: %s failed with error code: %d\n", function_name, error_code);
}

#endif /* SL_RT_MUTEX_H_ */
