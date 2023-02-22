
#include "mosa.h"
#include "imu_dist_loop.h"

float mosa_navi_acc[3];
float mosa_navi_gyro[3];

uint32_t mosa_imu_time_ms = 0;

extern uint8_t flag_power_off;
extern uint8_t flag_gnss_state;

extern double g_gnss_data[2];
extern double Dist_IMU_DEG;
extern uint32_t gSpeed;


extern int cnt_test = 0;

int Driving_Behavior_IMU(float speed, float accel[3], float gyro[3]);
int rapid_rotation(float gyro[3], float curSpeed, float alpha);
int rapid_u_turn(float gyro[3], float curSpeed);
void mosa_imu_init()
{
    mosa_time_init(&mosa_imu_time_ms);
}


void mosa_imu_push_cb(float f_acc[3], float f_gyro[3])
{
    int i;
    /*
    static int cnt = 0;

    cnt++;
    
    if(cnt >= 50){
        printf("$IMU %f %f %f %f %f %f\n",f_gyro[0],f_gyro[1],f_gyro[2], f_acc[0],f_acc[1],f_acc[2]);
        cnt = 0;
    }
    */
    for (i = 0; i < 3; i++) {
        mosa_navi_acc[i] = f_acc[i];
        mosa_navi_gyro[i] = f_gyro[i];
    }
    //Driving_Behavior_IMU((float)(gSpeed/1000.0), f_acc, f_gyro);
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
    uint32_t navi_lat, navi_lon;
    float f_acc[3], f_gyro[3];
    mosa_imu_navi_get(f_acc, f_gyro);
    //printf("diff time %d\n", diff_time);
    //printf("IMU DATA %f %f %f\n",f_acc[0],f_acc[1], f_acc[2]);
    /*
    cnt_test++;
    if(cnt_test < 25*100){
        navi_lat = 3712345 + cnt_test/100;
        navi_lon = 12700223 + cnt_test/100;
    }
    else if(cnt_test == 25*100){
        
        navi_lat = 0;
        navi_lon = 0;
        f_gyro[0] = 1.0/100.0;
        f_gyro[1] = 2.0/100.0;
        f_gyro[2] = 3.0/100.0;
        f_acc[0] = 1.0;
        f_acc[1] = 2.0;
        f_acc[2] = 3.0;
        GPS_navi_push(navi_lat, navi_lon, f_acc, f_gyro);
    }
    else if(cnt_test == 28*100){
        cnt_test++;
        navi_lat = 0;
        navi_lon = 0;
        f_gyro[0] = 1.0/100.0;
        f_gyro[1] = 2.0/100.0;
        f_gyro[2] = 3.0/100.0;
        f_acc[0] = 1.0;
        f_acc[1] = 2.0;
        f_acc[2] = 3.0;
        GPS_navi_push(navi_lat, navi_lon, f_acc, f_gyro);
    }
    */
    
    if(flag_power_off == true && g_gnss_data[0] != 0){
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
extern uint8_t flag_rotation = 0;
extern uint8_t flag_rotation_1 = 0;
extern uint8_t flag_rotation_2 = 0;
int Driving_Behavior_IMU(float speed, float accel[3], float gyro[3]){
    
    static int flag_u_turn = 0;
    static int flag_init = true;
    static int cnt_init = 0;

    /*
    if(rapid_u_turn(gyro, speed) == true){
        flag_u_turn++;
        flag_rotation--;
        if (flag_rotation < 0) flag_rotation = 0;
        flag_init = false;
    }*/
    if(flag_init == true && rapid_rotation(gyro, speed, -2) == true) {
        flag_rotation++;
        //flag_init = false;
    }
   if(flag_init == true && rapid_rotation(gyro, speed, -4) == true) {
        flag_rotation_1++;
        //flag_init = false;
    }
    if(flag_init == true && rapid_rotation(gyro, speed, -1) == true) {
        flag_rotation_2++;
        //flag_init = false;
    }
   if(flag_init == false){
        cnt_init++;
        if(cnt_init >= 30){
            flag_init = true;
            cnt_init = 0;
        }
    }
    return 0;
}

int rapid_rotation(float gyro[3], float curSpeed, float alpha){
    float norm_gyro = 0.0;
    static float buf_nGyro[200] = {0, };
    static float init_nGyro = 0.0;
    static int cnt_init = 0;
    float sum_nGyro = 0.0;
    int flag = false;

    norm_gyro = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    
    if(cnt_init >= 50){
        cnt_init = 51;
        norm_gyro -= init_nGyro/50;
        for(int i = 0;i<199;i++){
            buf_nGyro[i] = buf_nGyro[i+1];
            sum_nGyro += buf_nGyro[i]*0.01;
        }
        buf_nGyro[199] = norm_gyro;
        sum_nGyro += norm_gyro*0.01;

        if(curSpeed >= 15 && sum_nGyro >= (40.0 + alpha) && sum_nGyro < 160.0){
            flag = true;
            memset(buf_nGyro,0,sizeof(buf_nGyro));
        }
    }
    else{
        init_nGyro += norm_gyro;
        cnt_init++;
    }  
    return flag;
}

int rapid_u_turn(float gyro[3], float curSpeed){
    float norm_gyro = 0.0;
    static float init_nGyro = 0.0;
    static int cnt_init = 0;
    static float sum_nGyro = 0.0;
    static float buf_nGyro[2][255] = {0, };
    int flag = false;

    norm_gyro = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    if(cnt_init >= 50){
        cnt_init = 51;
        norm_gyro -= init_nGyro/50.0;
        for(int i = 0;i<254;i++){
            buf_nGyro[0][i] = buf_nGyro[0][i+1];
            sum_nGyro += buf_nGyro[0][i]*0.01;
        }
        buf_nGyro[0][254] = buf_nGyro[1][0];

        for(int i = 0;i<254;i++){
            buf_nGyro[1][i] = buf_nGyro[1][i+1];
            sum_nGyro += buf_nGyro[1][i]*0.01;
        }
        buf_nGyro[1][254] = norm_gyro;
        sum_nGyro += norm_gyro*0.01;
        if(curSpeed >= 25 && sum_nGyro >= 160.0){
            sum_nGyro = 0;
            memset(buf_nGyro,0,sizeof(buf_nGyro));
            flag = true;
        }
        else{
            init_nGyro += norm_gyro;
            cnt_init++;
        } 
    }
    return flag;
}
/* end of file */
