
#ifndef _UTIMER_INCLUDE_
#define _UTIMER_INCLUDE_

/* function prototypes */
extern int utimer_init(int max);
extern void utimer_do_queue(uint32_t lap_time);

extern int utimer_start(int time, int auto_load, void *arg, void (*cb)(void *arg), void **timer_id);
extern void utimer_stop(void *timer_id);
extern int utimer_restart(void *timer_id);
extern void utimer_display();

#endif

/* end of file */
