#ifndef __ETHERCAT_TIMER_H__
#define __ETHERCAT_TIMER_H__

#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
int Timer_Init(void* func);
int Start_Timer(uint32_t time_ns);
int Stop_Timer(void);

#ifdef __cplusplus
}
#endif

#endif