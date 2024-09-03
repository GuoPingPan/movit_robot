#include "EtherCAT_Timer.h"
#include "string.h"

/******* 定时器相关 *******/

struct sigevent se;
struct itimerspec its;
timer_t timer;

int Timer_Init(void* func){
	se.sigev_notify = SIGEV_THREAD;
	se.sigev_value.sival_ptr = timer;
	se._sigev_un._sigev_thread._function = func;
	se._sigev_un._sigev_thread._attribute = NULL;

	int ret = timer_create(CLOCK_REALTIME, &se, &timer);
	return ret;
}

int Start_Timer(uint32_t time_ns){
	its.it_value.tv_sec = 0;
	its.it_value.tv_nsec = time_ns;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	int ret = timer_settime(timer, 0, &its, NULL);
	return ret;
}

int Stop_Timer(void){
    its.it_value.tv_sec = 0;
	its.it_value.tv_nsec = 0;
	its.it_interval.tv_sec = 0;
	its.it_interval.tv_nsec = 0;

	timer_settime(timer, 0, &its, NULL);

    int ret = timer_delete(timer);

	return ret;
}