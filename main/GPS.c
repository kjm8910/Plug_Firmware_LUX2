
#include "mosa.h"
#include "imu_dist_loop.h"

/* every 1 second */
extern uint8_t flag_gnss_state;
extern uint8_t flag_power_off;
extern double g_gnss_data[2] = {0,0 };
extern double Dist_IMU_DEG;

// $DB acc de-acc start stop CL C
int Driving_Behavior_GPS(float speed);
int rapid_deceleration(float preSpeed, float curSpeed);

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
        Driving_Behavior_GPS((float)(speed/1000));
        
        
    }
}

int Driving_Behavior_GPS(float speed){
    
    static float preSpeed = 0.0;
    
    static int flag_accel = 0, flag_de_accel = 0, flag_start = 0, flag_stop = 0;
    if(rapid_deceleration(preSpeed, speed) == true) flag_accel++;

    preSpeed = speed;
    printf("$DB %d %d %d %d\n", flag_accel, flag_de_accel, flag_de_accel, flag_stop);
    return 0;
}

int rapid_deceleration(float preSpeed, float curSpeed){
    uint8_t flag = false;
    float del_speed;
    static int cnt_test = 0;
    del_speed = fabsf(preSpeed - curSpeed);
    if ((float)preSpeed*3.6 >= 6.0 && (float)preSpeed*3.6 <= 10.0){ //km/h
        
        if(del_speed >= 12){
            flag = true;
        }
    }
    else if ((float)preSpeed*3.6 <= 20.0){ //km/h
        if(del_speed >= 10){
            flag = true;
        }
    }
    else if ((float)preSpeed*3.6 > 20.0){ //km/h
        if(del_speed >= 8){
            flag = true;
        }
    return flag;
}


/* end of file */
