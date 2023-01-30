
#ifndef _MOSA_INCLUDE_
#define _MOSA_INCLUDE_


/* Standard */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio_ext.h>
#include <stdint.h>
#include <time.h>
#include <ctype.h>
#include <sys/time.h>
#include <math.h>

#include "sdkconfig.h"  // <project>/build/config/sdkconfig.h


/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP32 */
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"


/* components */
#include "mos_pub.h"

#include "tracker_api.h"
#include "utimer.h"
#include "polltimer.h"


/* mos */
#include "mosa_config.h"
#include "mosa_extern.h"


/* function prototype */
#include "mosa_proto.h"

#endif          // _MOSA_INCLUDE_

/* end of file */
