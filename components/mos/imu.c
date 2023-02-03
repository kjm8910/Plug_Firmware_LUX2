#include "imu_dist_loop.h"
#include "mos.h"

extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;
extern double POS_Result[2];
extern double Dist_IMU_DEG;

int mos_imu_add(float f_acc[3], float f_gyro[3])
{

    mos_imu_dist(f_acc, f_gyro);
    
    return (1);
}


/* end of file */
