
#include "mosa.h"
#include "imu_dist_loop.h"

TaskHandle_t        mosa_main_task_handle;
extern uint8_t flag_gnss_state = false;
extern double POS_Result[2];
extern uint8_t flag_power_off = false;
extern double Dist_IMU_DEG = 0.0;

/* application entry point */
void app_main(void)
{
    /* 1. setup application */
    APP_setup(0, mosa_power_off_cb);

    /* 1.1 mosa command */
    mosa_cmd_init();
    
    /* 2. initialize application */
    APP_init();

    APP_command_prompt("mosa");

    /* 2.1 setup IMU */
    IMU_setup(mosa_imu_push_cb);
    
    /* 2.2 setup GPS */
    GPS_setup(mosa_gps_push_cb);

    /* 2.3 initialize local */
    mosa_init();

    /* 2.4 initialize mos */
    mos_init();

    /* 3. start application */
    APP_start();
}


void mosa_power_off_cb(int plug_off)
{
    if (plug_off) {
        flag_power_off = true;
        printf("Power off Start ---------________-----------\n");
        //IMU_setup(mosa_imu_push_cb);
        mos_shutdown();
        if(flag_gnss_state == false){
            float f_acc[3] = {0, };
            float f_gyro[3] = {0, };
            uint32_t navi_lat, navi_lon;
            navi_lat = (uint32_t)((POS_Result[0]+Dist_IMU_DEG/2.0)*100000);
            navi_lon = (uint32_t)((POS_Result[1]+Dist_IMU_DEG/2.0)*100000);
            f_gyro[0] = 1.0/100.0;
            f_gyro[1] = 2.0/100.0;
            f_gyro[2] = 3.0/100.0;
            f_acc[0] = 1.0;
            f_acc[1] = 2.0;
            f_acc[2] = 3.0;
            GPS_navi_push(navi_lat, navi_lon, f_acc, f_gyro);
        }
    }

    MOSA_LOG("Plug Power %s", plug_off ? "OFF" : "ON");
}

void mosa_init()
{
    mosa_imu_init();

    /* mos_main_task */
    xTaskCreate(mosa_main_task, "mosa_main", MOSA_MAIN_TASK_STACK_SIZE,
            NULL, MOSA_MAIN_TASK_PRI, &mosa_main_task_handle);

}

void mosa_main_task(void *args)
{
    uint32_t event_bits;
    uint32_t timer;
    uint32_t counter_10ms, counter_1s;

    /* 10ms tick */
    utimer_start(MOSA_MAIN_TIMER_MS_TO_TICK(MOSA_MAIN_TIMER_MS), 1,
            NULL, mosa_main_timer_cb, 0);

    polltimer_init(&timer);
    counter_10ms = 0;
    counter_1s = 0;
    while (1) {
        /* get task event; no-wait(0) */
        xTaskNotifyWait(0, ULONG_MAX, &event_bits, 0);

        /* 10 ms ticks */
        if (event_bits & MOSA_MAIN_EVENT_TIMER) {
            counter_10ms++;  
        }
        if (polltimer_timeout(&timer, 1000)) {
            counter_1s++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void mosa_main_timer_cb(void *args)
{
    xTaskNotify(mosa_main_task_handle, MOSA_MAIN_EVENT_TIMER, eSetBits);
    
}




/* end of file */
