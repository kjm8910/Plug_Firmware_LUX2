#include "imu_dist_loop.h"
#include "mos.h"

extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;
extern double POS_Result[2];
extern double Dist_IMU_DEG;

int mos_imu_add(float f_acc[3], float f_gyro[3])
{

    //float f_acc[3], f_gyro[3];
    //printf("TEST\n");
    //mosa_imu_navi_get(f_acc,f_gyro);
    Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, POS_Result, 
            flag_power_off, flag_gnss_state, 10);
    
    return (1);
}


/* end of file */
