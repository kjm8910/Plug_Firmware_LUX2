
#include "mosa.h"
#include "imu_dist_loop.h"

float mosa_navi_acc[3];
float mosa_navi_gyro[3];
uint32_t mosa_imu_time_ms = 0;
extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;
extern uint8_t flag_imu_state;
extern double POS_Result[2];
extern double Dist_IMU_DEG;
extern uint32_t time_imu;

extern float main_f_acc[3] = {0, };
extern float main_f_gyro[3] = {0, };
void mosa_imu_init()
{
    mosa_time_init(&mosa_imu_time_ms);
}


void mosa_imu_push_cb(float f_acc[3], float f_gyro[3])
{
    int i;
    uint32_t diff_time;
    diff_time = mosa_time_elapsed(&mosa_imu_time_ms);
    time_imu = diff_time;
    //printf("diff time %d\n", diff_time);
    //printf("$IMU %f\t%f\t%f\t%f\t%f\t%f\t\n",f_gyro[0],f_gyro[1],f_gyro[2], f_acc[0],f_acc[1],f_acc[2]);
    
    //Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, POS_Result, 
    //                flag_power_off, flag_gnss_state, diff_time);
    //mos_imu_add(f_acc, f_gyro);
    
    for (i = 0; i < 3; i++) {
        mosa_navi_acc[i] = f_acc[i];
        mosa_navi_gyro[i] = f_gyro[i];
    }
    memcpy(main_f_acc, f_acc, sizeof(main_f_acc));
    memcpy(main_f_gyro, f_gyro, sizeof(main_f_gyro));
    flag_imu_state = true;
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
