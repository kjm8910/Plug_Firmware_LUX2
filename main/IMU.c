
#include "mosa.h"
#include "imu_dist_loop.h"

float mosa_navi_acc[3];
float mosa_navi_gyro[3];
uint32_t mosa_imu_time_ms = 0;
extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;
extern double POS_Result[2];

void mosa_imu_init()
{
    mosa_time_init(&mosa_imu_time_ms);
}

extern double Dist_IMU_DEG;
void mosa_imu_push_cb(float f_acc[3], float f_gyro[3])
{
    uint32_t diff_time; /* in millisecond */
    int i;

    diff_time = mosa_time_elapsed(&mosa_imu_time_ms);
    diff_time = diff_time;      // not used ?

    Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, POS_Result, 
                        flag_power_off, flag_gnss_state, diff_time);
    for (i = 0; i < 3; i++) {
        mosa_navi_acc[i] = f_acc[i];
        mosa_navi_gyro[i] = f_acc[i];
    }
}

int mosa_imu_navi_get(float f_acc[3], float f_gyro[3])
{
    int i;

    for (i = 0; i < 3; i++) {
        f_acc[i] = mosa_navi_acc[i];
        f_gyro[i] = mosa_navi_gyro[i];
    }

    return (1);
}

/* end of file */
