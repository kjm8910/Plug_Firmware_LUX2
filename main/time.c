
#include "mosa.h"


void mosa_time_init(uint32_t *time)
{
    *time = mosa_time_get();
}


uint32_t mosa_time_elapsed(uint32_t *time)
{
    uint32_t curr_time;
    uint32_t diff_time;

    curr_time = mosa_time_get();
    diff_time = curr_time - *time;

    *time = curr_time;

    return (diff_time);
}


uint32_t mosa_time_get()
{
    /* esp_timer_get_time: get time in microseconds since boot */
    return (uint32_t)(esp_timer_get_time()/1000);
}

/* end of file */
