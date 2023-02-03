
#include "mos.h"
#include "imu_dist_loop.h"
extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;
extern double POS_Result[2];
extern double Dist_IMU_DEG;

int mos_init()
{

    return (1);
}



void mos_shutdown()
{
    
}

void mos_imu_dist(float f_acc[3], float f_gyro[3])
{
    Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, POS_Result, 
            flag_power_off, flag_gnss_state, 10);
}
/* end of file */
