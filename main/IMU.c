
#include "mosa.h"
#include "imu_dist_loop.h"

float mosa_navi_acc[3];
float mosa_navi_gyro[3];

uint32_t mosa_imu_time_ms = 0;

extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;

extern double g_gnss_data[2];
extern double Dist_IMU_DEG;

void mosa_imu_init()
{
    mosa_time_init(&mosa_imu_time_ms);
}


void mosa_imu_push_cb(float f_acc[3], float f_gyro[3])
{
    int i;
    static int cnt = 0;

    cnt++;
    
    if(cnt >= 50){
        printf("$IMU %f %f %f %f %f %f\n",f_gyro[0],f_gyro[1],f_gyro[2], f_acc[0],f_acc[1],f_acc[2]);
        cnt = 0;
    }
    
    
    //Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, g_gnss_data, 
    //                flag_power_off, flag_gnss_state, time_imu);
    //mos_imu_add(f_acc, f_gyro);

    for (i = 0; i < 3; i++) {
        mosa_navi_acc[i] = f_acc[i];
        mosa_navi_gyro[i] = f_gyro[i];
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
int mosa_iope_navi(void){
    uint32_t diff_time;
    diff_time = mosa_time_elapsed(&mosa_imu_time_ms);
    
    float f_acc[3], f_gyro[3];
    mosa_imu_navi_get(f_acc, f_gyro);
    //printf("diff time %d\n", diff_time);
    //printf("IMU DATA %f %f %f\n",f_acc[0],f_acc[1], f_acc[2]);
    

    if(flag_power_off == true){
            uint32_t navi_lat, navi_lon;
            navi_lat = (uint32_t)((g_gnss_data[0]+Dist_IMU_DEG/2.0)*100000);
            navi_lon = (uint32_t)((g_gnss_data[1]+Dist_IMU_DEG/2.0)*100000);
            f_gyro[0] = 1.0/100.0;
            f_gyro[1] = 2.0/100.0;
            f_gyro[2] = 3.0/100.0;
            f_acc[0] = 1.0;
            f_acc[1] = 2.0;
            f_acc[2] = 3.0;
            GPS_navi_push(navi_lat, navi_lon, f_acc, f_gyro);
            return (1);
    }

    Dist_IMU_DEG = dist_Loop(f_acc, f_gyro, g_gnss_data,  
                flag_power_off, flag_gnss_state, diff_time);

    return (1);
}
/* end of file */
