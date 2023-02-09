
#include "mosa.h"
#include "imu_dist_loop.h"

/* every 1 second */
extern uint8_t flag_gnss_state;
extern uint8_t flag_power_off;
extern double g_gnss_data[2] = {0,0 };
extern double Dist_IMU_DEG;
void mosa_gps_push_cb(uint32_t lat, uint32_t lon, 
int32_t alt, uint32_t speed, uint32_t acc)
{
    printf("$GPS %d %d %d %d %d\n", lat, lon, alt, speed, acc);
    uint32_t navi_lat, navi_lon;
    float navi_acc[3], navi_gyro[3];
     /* get imu data */
    mosa_imu_navi_get(navi_acc, navi_gyro);
   
    if (lat == 0 || lon == 0) {
        navi_lat = 0;
        navi_lon = 0;
        if(flag_power_off == true){
            navi_lat = (uint32_t)((g_gnss_data[0]+Dist_IMU_DEG/2.0)*100000);
            navi_lon = (uint32_t)((g_gnss_data[1]+Dist_IMU_DEG/2.0)*100000);
            navi_gyro[0] = 1.0/100.0;
            navi_gyro[1] = 2.0/100.0;
            navi_gyro[2] = 3.0/100.0;
            navi_acc[0] = 1.0;
            navi_acc[1] = 2.0;
            navi_acc[2] = 3.0;
            GPS_navi_push(navi_lat, navi_lon, navi_acc, navi_gyro);
        }
        flag_gnss_state = false;
    } else {
        g_gnss_data[0] = (double)(lat)/100000.0;
        g_gnss_data[1] = (double)(lon)/100000.0; 
        navi_lat = lat;
        navi_lon = lon;
        flag_gnss_state = true;
    }
    /*
    if(flag_gnss_state == true) {
        printf("$TEST flag_power_off %d, flag_gnss_state %d \n", flag_power_off, flag_gnss_state);
        printf("$EST_STATE %d %d %f %f\n",navi_lat, navi_lon, POS_Result[0], POS_Result[1]);
    }*/
}


/* end of file */
