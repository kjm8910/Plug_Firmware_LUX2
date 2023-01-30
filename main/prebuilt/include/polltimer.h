
#ifndef _POLLTIMER_INCLUDE_
#define _POLLTIMER_INCLUDE_

/* function prototype */

extern int polltimer_init(uint32_t *timer);
extern int polltimer_timeout(uint32_t *timer, uint32_t timeout_ms);
extern int polltimer_timeout_sec(uint32_t *timer, uint32_t timeout_sec);

#endif

/* end of file */
