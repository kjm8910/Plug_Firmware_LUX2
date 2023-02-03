/*
    Distance Estimation Using IMU Only
    Email : jmkim@carrotins.com
    Designed By : Jong Myeong Kim(Finn), Digital Innovation
*/

// InPut : ACC, GYRO, GNSS
// OutPut : Filtering Data, Dist, etc
// https://www.notion.so/08aac645f9a143489406a3f515fd0cf6
// 프로토콜 참고

#include "imu_dist_loop.h"

extern double del_dist = 0.0;
extern double dist = 0.0;
extern uint8_t flag_car_stop = false;
extern float dt = 0.02;
extern double del_deg_pos[2] = {0,};

void imu_unit_conv(float *acc, float *gyro);
void gyro_offset_elimination(float *gyro);
void define_vehicle_stop(float *bias_gyro, float *pre_bias, 
                         uint8_t cnt_ars);
void Estimation_State(float *bias_gyro, float *pre_bias, uint8_t cnt_ars,
                      float *acc_ned, float *acc_ned_est, int32_t *gyro_lpf);
void imu_noise_filtering(float acc[3], float gyro[3]);

double dist_Loop(float acc[3], float gyro[3], double pos_gnss_data[2], 
                            uint8_t flag_plug_off, uint8_t flag_gnss_state, uint32_t diff_time){
    dt = (float)(diff_time) / 1000.0;
    //printf("$ACC %f\t%f\t%f\n",acc[0], acc[1], acc[2]);
    //printf("$GYRO %f\t%f\t%f\n",gyro[0], gyro[1], gyro[2]);
    static uint8_t flag_cross = false;
    static double Pos_IMU[2] = {0, };
    float euler[3] = {0, };              // Attitude(Roll, Pitch, Yaw)
    float acc_ned[3] = {0, };            // Acceleration NED Frame
    float acc_ned_est[3] = {0, };        // Acceleration NED Frame Excepted gravity 
    static int32_t time_sec = 0;
    double del_dist_deg = 0.0;
    // flag_plug_off true는 차량 전원이 차단되거나 상시 전원의 차량의 경우 슬립모드 진입순간에 발생함
    // gnss[0] == 0은 zeroGPS인 상황
    // 즉, zeroGPS인 상황에서 전원이 차단되어 마지막 데이터를 전송시키기 위한 상태
    static uint8_t cnt_test = 0;
    //Pos_IMU[0] = 37.12345;
    //Pos_IMU[1] = 127.0001;
    //memcpy(Pos_IMU, pos_gnss_data, sizeof(pos_gnss_data));
    if(flag_gnss_state == true){
        Pos_IMU[0] = pos_gnss_data[0];
        Pos_IMU[1] = pos_gnss_data[1];
        dist = 0.0;
        del_dist = 0.0;
    }
    
    if(flag_plug_off == true && flag_gnss_state == false){
        // 지하주차장의 경우 zeroGPS인 상황에서 짧게는 수백미터에서 길게는 수키로의 주행거리가 측정된다.
        // 전원 ON & zeroGPS -> IMU 기반의 이동거리 계산 후 전원 OFF면 마지막으로 전송된 GPS 
        // 데이터의 위경도 값에 계산된 Distance를 Degree로 변환하여
        // 더해준 후 서버로 전송하는 마지막 버퍼를 통해 서버로 전송

        // Total Distance limit => 1000 meter
        if(dist > 1000.0) dist = 1000.0;
        del_dist_deg = 0.00001 / 1.11 * dist; // meter to degree
        /* push adjusted navigation data */
        //printf("$IOPE %f %f\n", Pos_IMU[0], Pos_IMU[1]);
        return del_dist_deg;
    }
    
    // Main Loop For Distance Estimation
    // 0. Unit Conversation
    imu_unit_conv(acc, gyro);
    // 1. GYRO OFFSET Elimination
    gyro_offset_elimination(gyro);
    // 2. Filtering(IMU)
    imu_noise_filtering(acc, gyro);

    // 3. Attitude Reference System Using EKF
    float *x_est_ars;
    float qut[4] ={0, };
    int32_t gyro_est[3] = {0, };
    static float bias_gyro[3] = {0, };
    static float pre_bias[3] = {0, };
   
    gyro_est[0] = gyro[0] - (int32_t)(bias_gyro[0]*sf_gyro*DEG2RAD);
    gyro_est[1] = gyro[1] - (int32_t)(bias_gyro[1]*sf_gyro*DEG2RAD);
    gyro_est[2] = gyro[2] - (int32_t)(bias_gyro[2]*sf_gyro*DEG2RAD);
    
    x_est_ars = kf_ars_loop(acc, gyro_est);
    memcpy(qut, x_est_ars, sizeof(qut));
    memcpy(pre_bias, bias_gyro, sizeof(pre_bias));
    memcpy(bias_gyro, (x_est_ars + 4), sizeof(bias_gyro));

    static int cnt_ars = 0;
    if(cnt_ars < 300){
        cnt_ars++;
        flag_car_stop = true;
        return 0;
    }
    
    qut2euler(qut, euler);
    euler[0] *= RAD2DEG;
    euler[1] *= RAD2DEG;
    euler[2] *= RAD2DEG;
    
    bias_gyro[0] *= RAD2DEG;
    bias_gyro[1] *= RAD2DEG;
    bias_gyro[2] *= RAD2DEG;
    /*
    printf("$ARS %f %f %f %f %f %f\n", 
            euler[0], euler[1], euler[2],
            bias_gyro[0],bias_gyro[1],bias_gyro[2]);
    */
    float C_n2b[3][3] = {0, }, C_b2n[3][3] = {0, };
    qut2dcm(qut, C_n2b);
    Transpose_MAT_3(C_n2b, C_b2n);

    float facc_lpf[3] = {0, };
    facc_lpf[0] = (float)(acc[0]/sf_acc);
    facc_lpf[1] = (float)(acc[1]/sf_acc);
    facc_lpf[2] = (float)(acc[2]/sf_acc);

    // Body to NED(ACC)
    memset(acc_ned, 0, sizeof(acc_ned));
    dot_mat_vec(acc_ned, C_b2n, facc_lpf, 3);

    // 5. Calibration Accelerometer
    memset(acc_ned_est,0,sizeof(acc_ned_est));
    static float cNormAcc = 0.0;
    static float pNormAcc = 0.0;
    
    ACC_Calib(acc_ned, acc_ned_est, 25);
    /*
    printf("$ACC_NED %f %f %f %f %f %f", 
            acc_ned[0], acc_ned[1], acc_ned[2],
            acc_ned_est[0], acc_ned_est[1], acc_ned_est[2]);
    */
    pNormAcc = cNormAcc;
    cNormAcc = norm_vec(3, acc_ned_est);
    
    // 6. Estimation Pos & Vel Using RK4
    if (fabsf(cNormAcc - pNormAcc) < 2){
        Estimation_State(bias_gyro, pre_bias, cnt_ars,
                    acc_ned, acc_ned_est, gyro_est);
        
    }
    
    if (Pos_IMU[0] >= 30.0 && flag_gnss_state == false) {
        //double del_dist_deg = 0.0;
        static double dist_1sec = 0.0;
        static uint32_t one_sec = 0;
        //dist += del_dist;
        dist_1sec += del_dist;
        one_sec += diff_time;
        // Distance limit during 1sec => 8 meter
        if(one_sec > 990){
            if (dist_1sec > 10.0) dist_1sec = 10.0;
            
            dist += dist_1sec;
            dist_1sec = 0.0;
            one_sec = 0.0;
        } 

        static uint8_t cnt_print = 0;
        
        if (cnt_print > 50) {
            printf("IMU DIST : %f meter\n", dist);
            //printf("GPS TEST : %f %f \n", preGPS[0], curGPS[0]);
            cnt_print = 0;
        }
        else cnt_print++;
        del_dist = 0;
    }
    del_dist_deg = 0.00001 / 1.11 * (dist);// meter to degree
    return del_dist_deg;
}

void imu_unit_conv(float *acc, float *gyro){
    // IMU Unit Conversation(Ac 
    // g to m/s^2
    acc[0] *= Gravity;
    acc[1] *= Gravity;
    acc[2] *= Gravity;
    // Deg to Rad
    gyro[0] *= DEG2RAD;
    gyro[1] *= DEG2RAD;
    gyro[2] *= DEG2RAD;
}

void gyro_offset_elimination(float *gyro){
    // 1 - 1. OFFSET Elimination
    static int gyro_offset_cnt = 0;
    static float GYRO_OFFSET_VAL[3] = {0.0, };
    if (gyro_offset_cnt < GYRO_OFFSET_NUM){
        GYRO_OFFSET_VAL[0] += gyro[0];
        GYRO_OFFSET_VAL[1] += gyro[1];
        GYRO_OFFSET_VAL[2] += gyro[2];
        gyro_offset_cnt++;
        if(gyro_offset_cnt > 5){
            gyro[0] -= GYRO_OFFSET_VAL[0] / (float)gyro_offset_cnt;
            gyro[1] -= GYRO_OFFSET_VAL[1] / (float)gyro_offset_cnt;
            gyro[2] -= GYRO_OFFSET_VAL[2] / (float)gyro_offset_cnt;
        }
    }
    else{
      gyro[0] -= GYRO_OFFSET_VAL[0] / (float)gyro_offset_cnt;
      gyro[1] -= GYRO_OFFSET_VAL[1] / (float)gyro_offset_cnt;
      gyro[2] -= GYRO_OFFSET_VAL[2] / (float)gyro_offset_cnt;
    }
    
}

void define_vehicle_stop(float *bias_gyro, float *pre_bias, 
                         uint8_t cnt_ars){
    // differential current and previous estimation bias value
    static float buf[50] = {0, };
    static float cst_stop = 0.07;
    static uint8_t flag_stop_cst = false;
    static int cnt_diff_bias = 0;
    float diff_bias = 0.0;
    uint8_t N_ars = 50;
    //float diff_bias_2 = 0.0; 
    diff_bias = fabs(bias_gyro[0] - pre_bias[0]);//deg
    //diff_bias_2 = fabs(bias_gyro[1] - pre_bias[1]);//deg
    //printf("$bias diff %f\t%f\n", diff_bias, diff_bias_2);
    if(flag_stop_cst == false){
        //memmove(buf, buf+1, sizeof(buf)-sizeof(float));
        for(int i = 0; i < N_ars ;i++)
        {
            buf[i] = buf[i+1];
        }
        buf[49] = diff_bias;
        if (buf[0] != 0.0) {
            float mean_cst_stop = 0.0;
            for(int i = 0; i < N_ars;i++){
                mean_cst_stop += buf[i];
            }
             cst_stop = mean_cst_stop / (float)N_ars * 3.0;
             if(cst_stop < 0.02) cst_stop = 0.02;
             else if(cst_stop > 0.07) cst_stop = 0.07;
             flag_stop_cst = true;
             memset(buf, 0, sizeof(buf));
        }
    }
    
    if(diff_bias < cst_stop){ //0.02 IG, 0.04 Kasper, 0.05 ~ 0.07 QM3
        if(cnt_diff_bias >= N_ars){
            flag_car_stop = true;
            cnt_diff_bias = 0;
            //flag_stop_cst = false;
        }
        else cnt_diff_bias++;
    }
    else if(diff_bias > 0.25){ // 221205, Roation without Car moving 
        cnt_diff_bias = 0;
        flag_car_stop = false;
        cnt_ars-= 1;
        if(cnt_ars <0) cnt_ars = 0;
    }
    else {
        cnt_diff_bias = 0;
        flag_car_stop = false;
    }
}

void Estimation_State(float *bias_gyro, float *pre_bias, uint8_t cnt_ars,
                      float *acc_ned, float *acc_ned_est, int32_t *gyro_lpf){
    
    static double cX_est[6] = {37.41231*DEG2RAD,127.010203*DEG2RAD, 10.0, 0.0,0.0,0.0};
    static double pX_est[6] = {37.41231*DEG2RAD,127.010203*DEG2RAD, 10.0, 0.0,0.0,0.0};
    
     
    cX_est[2] = 10;
    pX_est[2] = 10;
    // 바이어스 차이를 활용한 정지 상태 정의 /////////////////////////////////
    define_vehicle_stop(bias_gyro, pre_bias, cnt_ars);
    //////////////////////////////////////////////////////////////////
    if(flag_car_stop == false){
        float nGyro = 0.0;                   // norm of Anguler rate
        float fgyro_lpf[3] = {0, };
        fgyro_lpf[0] = (float)(gyro_lpf[0]/sf_acc);
        fgyro_lpf[1] = (float)(gyro_lpf[1]/sf_acc);
        fgyro_lpf[2] = (float)(gyro_lpf[2]/sf_acc);
        nGyro = norm_vec(3, fgyro_lpf) * RAD2DEG; //RAD
        double norm_acc = sqrt(acc_ned[0]*acc_ned[0] + acc_ned[1]*acc_ned[1] + 
                                acc_ned[2]*acc_ned[2]);
        //printf("$ACC NED %f\t%f\t%f\t%f\t%f\t%f\t\n",acc_ned[0],acc_ned[1],acc_ned[2],
        //                        acc_ned_est[0],acc_ned_est[1],acc_ned_est[2]);
        if(nGyro > 40.0){
            del_dist = 0.0;
            cnt_ars = 0;
        }
        else if(acc_ned[2] > 9.81*0.94 && acc_ned[2] < 9.81*1.06){
            // Numerical Integration Using Runge Kutta 4th
            // Inuput : Acceleration
            // Output : Position & Velocity
            // cX_est[0 ~ 2] => Position(NED)
            // cX_est[3 ~ 5] => Velocity(NED)
            rk4_dist(cX_est, acc_ned, acc_ned_est);
            // 7. Calculation Distance
            // Input : Lat1, Long1, Lat2, Long2(degree)
            // Output : Distance (meter)
            del_dist = calculate_dist(cX_est[0], cX_est[1], 
                            pX_est[0], pX_est[1]);
        }
        else{
            del_dist = 0;
        }
        
    }
    else if(flag_car_stop == true){
        del_dist = 0.0;
        pX_est[3]  = 0.0;
        pX_est[4]  = 0.0;
        pX_est[5]  = 0.0;
        cnt_ars--;
        memcpy(cX_est, pX_est, sizeof(pX_est));
    }

    if(cnt_ars < 0) cnt_ars = 0;

    del_deg_pos[0] = (cX_est[0] - pX_est[0])*RAD2DEG;
    del_deg_pos[1] = (cX_est[1] - pX_est[1])*RAD2DEG;
    

    memcpy(pX_est, cX_est, sizeof(cX_est));
}

void imu_noise_filtering(float acc[3], float gyro[3]){
    //  a. Median Filter
    //  b. LPF(Low Pass Filter)
    static int32_t MedData_acc[3] = {0, };
    static int32_t MedData_gyro[3] = {0, };
    static int32_t acc_lpf[3] = {0, }, gyro_lpf[3] = {0, };
    // 2 - 1. Median Filter
    MedianFilter(acc, gyro, MedData_acc, MedData_gyro);
    MedData_gyro[0] = (int32_t)(gyro[0] * sf_gyro);
    MedData_gyro[1] = (int32_t)(gyro[1] * sf_gyro);
    MedData_gyro[2] = (int32_t)(gyro[2] * sf_gyro);

    // 2 - 2. Low Pass Filter
    if(acc_lpf[0] == 0 && gyro_lpf[0] == 0 ){//&& acc_lpf[1] == 0) {
        acc_lpf[0] = (int32_t)(acc[0] * sf_acc);
        acc_lpf[1] = (int32_t)(acc[1] * sf_acc);
        acc_lpf[2] = (int32_t)(acc[2] * sf_acc);

        gyro_lpf[0] = (int32_t)(gyro[0] * sf_gyro);
        gyro_lpf[1] = (int32_t)(gyro[1] * sf_gyro);
        gyro_lpf[2] = (int32_t)(gyro[2] * sf_gyro);
    }
    else{
        low_pass_filter(MedData_acc, acc_lpf);
        low_pass_filter(MedData_gyro, gyro_lpf);
        //high_pass_filter(MedData_gyro,gyro_lpf,pGyro);
        //memcpy(pGyro, MedData_gyro, sizeof(MedData_gyro));
    }
    memcpy(acc, acc_lpf, sizeof(acc_lpf));
    memcpy(gyro, gyro_lpf, sizeof(gyro_lpf));
    /*
    printf("$ACC %f %f %f\n",
            acc[0], acc[1], acc[2],
            acc_lpf[0],acc_lpf[1],acc_lpf[2]);
    printf("$GYRO %f %f %f\n",
            gyro[0], gyro[1], gyro[2],
            gyro_lpf[0],gyro_lpf[1],gyro_lpf[2]);
    */
}