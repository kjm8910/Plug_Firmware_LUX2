/*
    Project : IMU Only Position Estimation(IOPE)
    Email : jmkim@carrotins.com
    Designed By : Jong Myeong Kim(Finn), IoT Tech Team.
*/
/*
    <Define InPut Data>
    1. acc[3]
        accelerometer raw data
            unit : g
    2. gyro[3]
        gyro raw data
            unit : degree
    3. pos_gnss_data[2]
        GNSS Position Data(Lattitude, Longitude)
            unit : degree
    4. flag_gnss_state
        0 : no Fix
        1 : 3D Fix
    5. flag_plug_off
        true : Plug Power Off
        false : Plug Power On(Operating)
    6. diff_time(Delta Time)
        Differential Time
        uint : miliSecond(ms)

*/
#include "imu_dist_loop.h"

extern int64_t del_dist = 0.0;
extern int64_t dist = 0.0;
extern uint8_t flag_car_stop = false;
extern int32_t dt = 10;
extern int64_t del_deg_pos[2] = {0,};

double dist_Loop(float acc[3], float gyro[3], double pos_gnss_data[2], 
                            uint8_t flag_plug_off, uint8_t flag_gnss_state, int32_t diff_time){
    
   
    
    //printf("$ACC %f\t%f\t%f\n",acc[0], acc[1], acc[2]);
    //printf("$GYRO %f\t%f\t%f\n",gyro[0], gyro[1], gyro[2]);
    static int64_t Pos_IMU[2] = {0, };
    float euler[3] = {0, };              // Attitude(Roll, Pitch, Yaw)
    float acc_ned[3] = {0, };            // Acceleration NED Frame
    float acc_ned_est[3] = {0, };        // Acceleration NED Frame Excepted gravity 
    double del_dist_deg = 0.0;
    // flag_plug_off true는 차량 전원이 차단되거나 상시 전원의 차량의 경우 슬립모드 진입순간에 발생함
    // gnss[0] == 0은 zeroGPS인 상황
    // 즉, zeroGPS인 상황에서 전원이 차단되어 마지막 데이터를 전송시키기 위한 상태
    //static uint8_t cnt_test = 0;
    //Pos_IMU[0] = 37.12345;
    //Pos_IMU[1] = 127.0001;
    //memcpy(Pos_IMU, pos_gnss_data, sizeof(pos_gnss_data));

    // Time Check
    if (diff_time >= 9){
        dt = diff_time;
    }
    else if(diff_time < 9){
        return 0;
    } 
    else{
        return 0;
    }
    
    // GNSS Data Check
    if(flag_gnss_state == ThreeD_Fix){
        Pos_IMU[0] = (int64_t)(pos_gnss_data[0] * 1e+5);
        Pos_IMU[1] = (int64_t)(pos_gnss_data[1] * 1e+5);
        dist = 0.0;
        del_dist = 0.0;
    }
    
    if(flag_plug_off == plug_power_off && flag_gnss_state == no_Fix){
        // 지하주차장의 경우 zeroGPS인 상황에서 짧게는 수백미터에서 길게는 수키로의 주행거리가 측정된다.
        // 전원 ON & zeroGPS -> IMU 기반의 이동거리 계산 후 전원 OFF면 마지막으로 전송된 GPS 
        // 데이터의 위경도 값에 계산된 Distance를 Degree로 변환하여
        // 더해준 후 서버로 전송하는 마지막 버퍼를 통해 서버로 전송

        // Total Distance limit => 1000 meter
        // 최종 이동거리 제한은 추후 반영
        //if(dist > 1000.0) dist = 1000.0;
        del_dist_deg = 0.00001 / 1.11 * (double)(dist/1e+5); // meter to degree
        
        return del_dist_deg;
    }
    
    // Main Loop For Distance Estimation
    // 0. Unit Conversation
    imu_unit_conv(acc, gyro);
    // 1. GYRO OFFSET Elimination
    gyro_offset_elimination(gyro);
    // 2. Filtering(IMU)
    imu_noise_filtering(acc, gyro);
    
    int32_t acc_int[3] = {0, };
    int32_t gyro_int[3] = {0, };
    float2int32(acc, acc_int, 3);
    acc_int[0] = (int32_t)(acc[0]*1e+5);
    acc_int[1] = (int32_t)(acc[1]*1e+5);
    acc_int[2] = (int32_t)(acc[2]*1e+5);

    gyro_int[0] = (int32_t)(gyro[0]*1e+5);
    gyro_int[1] = (int32_t)(gyro[1]*1e+5);
    gyro_int[2] = (int32_t)(gyro[2]*1e+5);
    //TEST
    // 3. Attitude Reference System Using EKF
    float *x_est_ars;
    float qut[4] ={0, };
    int32_t gyro_est[3] = {0, };
    static int32_t bias_gyro_int[3] = {0, };
    static int32_t pre_bias_int[3] = {0, };
    float bias_gyro[3] = {0, };

    gyro_est[0] = gyro[0] - (bias_gyro[0]*DEG2RAD);
    gyro_est[1] = gyro[1] - (bias_gyro[1]*DEG2RAD);
    gyro_est[2] = gyro[2] - (bias_gyro[2]*DEG2RAD);
    
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

    // Body to NED(ACC)
    memset(acc_ned, 0, sizeof(acc_ned));
    dot_mat_vec(acc_ned, C_b2n, acc, 3);

    // 5. Calibration Accelerometer
    memset(acc_ned_est,0,sizeof(acc_ned_est));
    static float cNormAcc = 0.0;
    static float pNormAcc = 0.0;
    
    ACC_Calib(acc_ned, acc_ned_est, 100);
    /*
    printf("$ACC_NED %f %f %f %f %f %f\n", 
            acc_ned[0], acc_ned[1], acc_ned[2],
            acc_ned_est[0], acc_ned_est[1], acc_ned_est[2]);
    */
    pNormAcc = cNormAcc;
    cNormAcc = norm_vec(3, acc_ned_est);
    
    // 6. Estimation Pos & Vel Using RK4
    //printf("TEST %f\n", fabsf(cNormAcc - pNormAcc));
    if (fabsf(cNormAcc - pNormAcc) < 4){
        Estimation_State(bias_gyro, pre_bias, cnt_ars,
                    acc_ned, acc_ned_est, gyro_est);
        
    }
    
    if (Pos_IMU[0] >= 30.0 && flag_gnss_state == false) {
        static double dist_1sec = 0.0;
        static uint32_t one_sec = 0;
        dist_1sec += del_dist;
        one_sec += diff_time;
        // Distance limit during 1sec => 8 meter
        if(one_sec > 990){
           // printf("$TEST_IOPE %f\n",dist_1sec);
            if (dist_1sec > 10.0) dist_1sec = 10.0;
            
            dist += dist_1sec;
            dist_1sec = 0.0;
            one_sec = 0.0;
        } 

        static uint8_t cnt_print = 0;
        
        if (cnt_print >= 100) {
            printf("IMU DIST : %f meter\n", dist);
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
    static float buf[100] = {0, };
    static float cst_stop = 0.07;
    static uint8_t flag_stop_cst = false;
    static uint16_t cnt_diff_bias = 0;
    float diff_bias = 0.0;
    uint16_t N_ars = 100;
    diff_bias = fabs(bias_gyro[0] - pre_bias[0]);//deg

    if(flag_stop_cst == false){
        //memmove(buf, buf+1, sizeof(buf)-sizeof(float));
        for(int i = 0; i < N_ars ;i++)
        {
            buf[i] = buf[i+1];
        }
        buf[99] = diff_bias;
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
        if(cnt_diff_bias >= 150){
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
                      float *acc_ned, float *acc_ned_est, float *gyro){
    
    static double cX_est[6] = {37.41231*DEG2RAD,127.010203*DEG2RAD, 10.0, 0.0,0.0,0.0};
    static double pX_est[6] = {37.41231*DEG2RAD,127.010203*DEG2RAD, 10.0, 0.0,0.0,0.0};
    
    cX_est[2] = 10;
    pX_est[2] = 10;
    // 바이어스 차이를 활용한 정지 상태 정의 /////////////////////////////////
    define_vehicle_stop(bias_gyro, pre_bias, cnt_ars);
    //////////////////////////////////////////////////////////////////
    if(flag_car_stop == false){
        float nGyro = 0.0;                   // norm of Anguler rate
        nGyro = norm_vec(3, gyro) * RAD2DEG;
        double norm_acc = sqrt(acc_ned[0]*acc_ned[0] + acc_ned[1]*acc_ned[1] + 
                                acc_ned[2]*acc_ned[2]);
        //printf("$ACC NED %f\t%f\t%f\t%f\t%f\t%f\t\n",acc_ned[0],acc_ned[1],acc_ned[2],
        //                        acc_ned_est[0],acc_ned_est[1],acc_ned_est[2]);
        if(nGyro > 80.0){
            //printf("TEST LOOP 1 nGyro %f\n", nGyro);
            del_dist = 0.0;
            cnt_ars = 0;
            //if(cnt_ars < 0) cnt_ars = 0;
        }
        else if(acc_ned[2] > 9.81*0.94 && acc_ned[2] < 9.81*1.06){
            //printf("TEST LOOP 2\n");
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
            //printf("DIST del %f\n", del_dist);
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
    acc[0] = (float)(acc_lpf[0])/sf_acc;
    acc[1] = (float)(acc_lpf[1])/sf_acc;
    acc[2] = (float)(acc_lpf[2])/sf_acc;
    gyro[0] = (float)(gyro_lpf[0])/sf_gyro;
    gyro[1] = (float)(gyro_lpf[1])/sf_gyro;
    gyro[2] = (float)(gyro_lpf[2])/sf_gyro;
    /*
    printf("$ACC %f %f %f\n",
            acc[0], acc[1], acc[2],
            acc_lpf[0],acc_lpf[1],acc_lpf[2]);
    printf("$GYRO %f %f %f\n",
            gyro[0], gyro[1], gyro[2],
            gyro_lpf[0],gyro_lpf[1],gyro_lpf[2]);
    */
}


void float2int32(float *fdata, int32_t *i32data, int8_t data_size){

    int i = 0;
    for(i = 0; i < data_size; i++){
        *(i32data + i)  = (int32_t)(*(fdata + i)*1e+5);
    }
}