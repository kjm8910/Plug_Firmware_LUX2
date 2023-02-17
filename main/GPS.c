
#include "mosa.h"
#include "imu_dist_loop.h"

/* every 1 second */
extern uint8_t flag_gnss_state;
extern uint8_t flag_power_off;
extern double g_gnss_data[2] = {0,0 };
extern double Dist_IMU_DEG;
extern uint32_t gSpeed = 0.0;
//extern int cnt_test;
// $DB acc de-acc start stop CL C
int Driving_Behavior_GPS(float Speed, float preSpeed);
int rapid_acceleration(float preSpeed, float curSpeed);
int harsh_deceleration(float preSpeed, float curSpeed, float alpha);

void mosa_gps_push_cb(uint32_t lat, uint32_t lon, 
int32_t alt, uint32_t speed, uint32_t acc)
{
    static float preSpeed = 0.0, delV = 0.0;
    if (preSpeed != 0.0){
        delV = (float)(speed)/1000.0 - preSpeed;
    }
    else delV = 0.0;

    printf("$GPS %d %d %d %d %d %d\n", lat, lon, alt, speed, acc, delV);
    uint32_t navi_lat, navi_lon;
    float navi_acc[3], navi_gyro[3];
    
     /* get imu data */
    

    mosa_imu_navi_get(navi_acc, navi_gyro);
   
    if (lat == 0 || lon == 0) {
        navi_lat = 0;
        navi_lon = 0;
        /*/
        if(cnt_test < 25*100){
            navi_lat = 3712345 + cnt_test/100;
            navi_lon = 12700223 + cnt_test/100;
            GPS_navi_push(navi_lat, navi_lon, navi_acc, navi_gyro);
        }
        else if(cnt_test == 25*100){
        
        navi_lat = 0;
        navi_lon = 0;
        navi_gyro[0] = 1.0/100.0;
        navi_gyro[1] = 2.0/100.0;
        navi_gyro[2] = 3.0/100.0;
        navi_acc[0] = 1.0;
        navi_acc[1] = 2.0;
        navi_acc[2] = 3.0;
        GPS_navi_push(navi_lat, navi_lon, navi_acc, navi_gyro);
    }
    else if(cnt_test == 28*100){
        cnt_test++;
        navi_lat = 0;
        navi_lon = 0;
        navi_gyro[0] = 1.0/100.0;
        navi_gyro[1] = 2.0/100.0;
        navi_gyro[2] = 3.0/100.0;
        navi_acc[0] = 1.0;
        navi_acc[1] = 2.0;
        navi_acc[2] = 3.0;
        GPS_navi_push(navi_lat, navi_lon, navi_acc, navi_gyro);
    }*/

        if(flag_power_off == true && g_gnss_data[0] != 0){
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

        /*
        if(acc <= 15 && acc > 0){
            if(preSpeed != 0){
                Driving_Behavior_GPS((float)(speed/1000.0), preSpeed);
            }
            preSpeed = (float)(speed/1000.0);
            gSpeed = speed;
        }
        else{
            preSpeed = 0;
        }
        */
    }
}
extern uint8_t flag_rotation, flag_rotation_1, flag_rotation_2;
int Driving_Behavior_GPS(float Speed, float preSpeed){
    static uint8_t flag_accel = 0, flag_accel_1 = 0, flag_accel_2 = 0;
    static uint8_t flag_de_accel = 0, flag_de_accel_1 = 0, flag_de_accel_2 = 0;
    static uint8_t flag_start = 0, flag_start_1 = 0, flag_start_2 = 0;
    static uint8_t flag_stop_1 = 0, flag_stop_2 = 0, flag_stop = 0;
    static uint8_t flag_init = true;
    static int cnt_init = 0;
    float del_speed = 0.0;
    if(flag_init == true){
        // 급가속
        if(rapid_acceleration(preSpeed, Speed) == true){
            flag_accel++;
           //flag_init = false;
        }

        // 급감속
        if(harsh_deceleration(preSpeed, Speed, 0.5) == true){
            flag_de_accel++;
            //flag_init = false;
        }
        if(harsh_deceleration(preSpeed, Speed, 0.25) == true){
            flag_de_accel_1++;
        }
        if(harsh_deceleration(preSpeed, Speed, 0) == true){
            flag_de_accel_2++;
        }
        ////////////////////////////////////////////////////////////////////////////////////
        
        // 급출발
        if (preSpeed <= 10.0 && preSpeed >= 0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed >= 8.0){ // 10 -> 8
                flag_start++;
            }
        }
        // 급출발 _ 1
        if (preSpeed <= 10.0 && preSpeed >= 0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed >= 6.0){ // 10 -> 8
                flag_start_1++;
            }
        }
        // 급출발 _2
        if (preSpeed <= 10.0 && preSpeed >= 0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed >= 10.0){ // 10 -> 8
                flag_start_2++;
            }
        }

        // 급정지
        if (Speed <= 10.0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed <= -8.0){ // 14 -> 8
                flag_stop++;
            }
        }
        // 급정지
        if (Speed <= 10.0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed <= -14.0){ // 14 -> 8
                flag_stop_1++;
            }
        }
        // 급정지
        if (Speed <= 5.0){ //km/h
            del_speed = Speed - preSpeed;
            if(del_speed <= -6.0){ // 14 -> 8
                flag_stop_2++;
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////
    if(flag_init == false){
        cnt_init++;
        if(cnt_init >= 30){
            flag_init = true;
            cnt_init = 0;
        }
    }
    
    printf("$DB %d %d %d %d %d ", flag_accel, flag_de_accel, flag_start, flag_stop, flag_rotation);
    printf("%d %d %d %d ", flag_de_accel_1, flag_start_1, flag_stop_1, flag_rotation_1);
    printf("%d %d %d %d\n", flag_de_accel_2, flag_start_2, flag_stop_2, flag_rotation_2);
    return 0;
}
int harsh_deceleration(float preSpeed, float curSpeed, float alpha){
    // 급감속
    float del_speed = 0.0;
    if (curSpeed >= 6.0)
    {
        if (preSpeed > 10.0 && preSpeed <= 30.0){ //km/h
            del_speed = curSpeed - preSpeed;
            if(del_speed <= -8 + alpha){ // 14 -> 8
                return true;
            }
        }
        else if (preSpeed > 30.0 && preSpeed <= 50.0){ //km/h
            del_speed = curSpeed - preSpeed;
            if(del_speed <= -8 + alpha){ //15 -> 10
                return true;
            }
        }
        else if (preSpeed > 50.0 && preSpeed <= 300.0){ //km/h
            del_speed = curSpeed - preSpeed;
            if(del_speed <= -8 + alpha){ //15 -> 10
                return true;
            }
        }
    }
    return false;
}
int rapid_acceleration(float preSpeed, float curSpeed){

    uint8_t flag = false;
    float del_speed;
    static int cnt_test = 0;
    del_speed = curSpeed - preSpeed;
    if (preSpeed >= 6.0 && preSpeed <= 10.0 && preSpeed > 0){ //km/h
        if(del_speed >= 12){
            flag = true;
        }
    }
    else if (preSpeed <= 20.0){ //km/h
        if(del_speed >= 10){
            flag = true;
        }
    }
    else if (preSpeed > 20.0){ //km/h
        if(del_speed >= 8){
            flag = true;
        }
    }
    return flag;
}

/* end of file */
