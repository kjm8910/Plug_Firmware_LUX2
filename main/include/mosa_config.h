
#ifndef _MOSA_CONFIG_INCLUDE_
#define _MOSA_CONFIG_INCLUDE_

#define MOSA_MAIN_TASK_STACK_SIZE            (1024*2)        // 2048 K
#define MOSA_MAIN_TASK_PRI                   10           


#define MOSA_MAIN_TIMER_TICK                 10
#define MOSA_MAIN_TIMER_MS_TO_TICK(m)        ((m)/MOSA_MAIN_TIMER_TICK)

#define MOSA_MAIN_TIMER_MS                   100
#define MOSA_MAIN_EVENT_TIMER                0x00000001


/* log TAG */
#define MOSA_LOG_TAG                         "mosa"
/* log function macro */
#define MOSA_LOG(format, ...)        ESP_LOGI(MOSA_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define MOSA_LOGW(format, ...)       ESP_LOGW(MOSA_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define MOSA_LOGE(format, ...)       ESP_LOGE(MOSA_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)

#endif          // _MOSA_CONFIG_INCLUDE_

/* end of file */
