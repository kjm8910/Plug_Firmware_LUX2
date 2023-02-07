#include "imu_dist_loop.h"
// IMU Noise Reduction
// LPF : Low Pass Filter
// MAF : Moving Average Filter
// MED : Median Filter
// HPF : High Pass Filter

void GYRO_MED(float gyro[3], int32_t MedData_gyro[3]){
    static int32_t buf_gyro[N_Med][3] = {0, };
    static int32_t calib_gyro[3] = {0, };
    static int8_t  flag_calib_gyro = false;
    static int8_t cnt = 0;
    int32_t Data_X[N_Med] = {0, }, Data_Y[N_Med] = {0, }, Data_Z[N_Med] = {0, };
    int32_t temp_x, temp_y, temp_z;

    for (int i = 0; i < N_Med-1;i ++){
        temp_x = buf_gyro[i+1][0];
        buf_gyro[i+1][0] = buf_gyro[i][0];
        buf_gyro[i][0] = temp_x;

        temp_y = buf_gyro[i+1][1];
        buf_gyro[i+1][1] = buf_gyro[i][1];
        buf_gyro[i][1] = temp_y;

        temp_z = buf_gyro[i+1][2];
        buf_gyro[i+1][2] = buf_gyro[i][2];
        buf_gyro[i][2] = temp_z;
    }
      
    buf_gyro[N_Med - 1][0] = (int32_t)(gyro[0]*sf_gyro);
    buf_gyro[N_Med - 1][1] = (int32_t)(gyro[1]*sf_gyro);
    buf_gyro[N_Med - 1][2] = (int32_t)(gyro[2]*sf_gyro);

    if (flag_calib_gyro == false) {
        calib_gyro[0] += buf_gyro[N_Med - 1][0];
        calib_gyro[1] += buf_gyro[N_Med - 1][1];
        calib_gyro[2] += buf_gyro[N_Med - 1][2];  
        cnt++;  
    }
    int32_t mean_X = 0, mean_Y = 0, mean_Z = 0;
    if (buf_gyro[0][0] != 0){
        for (int i = 0; i < N_Med;i++){
            
            Data_X[i] = buf_gyro[i][0];
            Data_Y[i] = buf_gyro[i][1];
            Data_Z[i] = buf_gyro[i][2];
            mean_X += Data_X[i];
            mean_Y += Data_Y[i];
            mean_Z += Data_Z[i];
        }

        mean_X /= cnt;
        mean_Y /= cnt;
        mean_Z /= cnt;

        float mean_gyro = 0.0;

        mean_gyro = sqrtf((float)(mean_X*mean_X + mean_Y*mean_Y + mean_Z*mean_Z)/sf_gyro/sf_gyro);
        flag_calib_gyro = true;
         
        qsort(Data_X, sizeof(Data_X) / sizeof(float), sizeof(float), compare);
        qsort(Data_Y, sizeof(Data_Y) / sizeof(float), sizeof(float), compare);
        qsort(Data_Z, sizeof(Data_Z) / sizeof(float), sizeof(float), compare);
        
        int len = sizeof(Data_X) / sizeof(float) / 2 - 1;
        
        MedData_gyro[0] = Data_X[len] - mean_X;
        MedData_gyro[1] = Data_Y[len] - mean_Y;
        MedData_gyro[2] = Data_Z[len] - mean_Z;
    }
    else{
        MedData_gyro[0] = (int32_t)(gyro[0]*sf_gyro) - calib_gyro[0]/(int32_t)cnt;
        MedData_gyro[1] = (int32_t)(gyro[1]*sf_gyro) - calib_gyro[1]/(int32_t)cnt;
        MedData_gyro[2] = (int32_t)(gyro[2]*sf_gyro) - calib_gyro[2]/(int32_t)cnt;
    }

}

void ACCEL_MED(float acc[3], int32_t MedData_acc[3]){
    static int32_t buf_acc[N_Med][3] = {0, };
    int32_t Data_X[N_Med] = {0, }, Data_Y[N_Med] = {0, }, Data_Z[N_Med] = {0, };
    int32_t temp_x, temp_y, temp_z;

    for (int i = 0; i < N_Med-1;i ++){
        temp_x = buf_acc[i+1][0];
        buf_acc[i+1][0] = buf_acc[i][0];
        buf_acc[i][0] = temp_x;

        temp_y = buf_acc[i+1][1];
        buf_acc[i+1][1] = buf_acc[i][1];
        buf_acc[i][1] = temp_y;

        temp_z = buf_acc[i+1][2];
        buf_acc[i+1][2] = buf_acc[i][2];
        buf_acc[i][2] = temp_z;
    }
    buf_acc[N_Med - 1][0] = (int32_t)(acc[0]*sf_acc);
    buf_acc[N_Med - 1][1] = (int32_t)(acc[1]*sf_acc);
    buf_acc[N_Med - 1][2] = (int32_t)(acc[2]*sf_acc);
    
    if (buf_acc[0][0] != 0){
        for (int i = 0; i < N_Med;i++){
            Data_X[i] = buf_acc[i][0];
            Data_Y[i] = buf_acc[i][1];
            Data_Z[i] = buf_acc[i][2];
        }
      
        qsort(Data_X, sizeof(Data_X) / sizeof(int32_t), sizeof(float), compare);
        qsort(Data_Y, sizeof(Data_Y) / sizeof(int32_t), sizeof(float), compare);
        qsort(Data_Z, sizeof(Data_Z) / sizeof(int32_t), sizeof(float), compare);
        
        int len = sizeof(Data_X) / sizeof(float) / 2 - 1;
        MedData_acc[0] = (Data_X[len]);
        MedData_acc[1] = (Data_Y[len]);       
        MedData_acc[2] = (Data_Z[len]);
    }
    else {
        MedData_acc[0] = (int32_t)(acc[0]*sf_acc);
        MedData_acc[1] = (int32_t)(acc[1]*sf_acc);
        MedData_acc[2] = (int32_t)(acc[2]*sf_acc);
    }
}

void MedianFilter(float acc[3], float gyro[3], int32_t MedData_acc[3], int32_t MedData_gyro[3]) {
    //float gyro[3] = {0, }, acc[3] = {0, };
    //memcpy(gyro,stImuDatTemp.gyro, sizeof(gyro));
    //memcpy(acc,stImuDatTemp.acc, sizeof(acc));
    ACCEL_MED(acc, MedData_acc);
    GYRO_MED(gyro, MedData_gyro); 
    
}
int compare(const void *T1 , const void *T2) 
{ 
     if( *(float*)T1 > *(float*)T2 )
        return 1;
    else if( *(float*)T1 < *(float*)T2 )
        return -1;
    else
        return 0;
} 
void low_pass_filter(int32_t curData[3], int32_t preData[3]){
    float cData[3] = {0, }, pData[3] = {0,};
    float alpha = 0.95;
    for (int i = 0; i<3;i++){
        cData[i] = (float)(curData[i])/sf_acc;
        pData[i] = (float)(preData[i])/sf_acc;
    }

    if (preData[0] == 0){
        preData[0] = curData[0];
        preData[1] = curData[1];
        preData[2] = curData[2];
    }
    else{
        //float W_1 = (float)(LPF_ALPHA / (LPF_ALPHA + dt) );
        //float W_2 = (float)(dt / (LPF_ALPHA + dt));
        for (int i = 0; i < 3; i++){
            pData[i] = pData[i] * (1 - alpha) + cData[i] * alpha;
            preData[i] = (int32_t)(pData[i] * sf_acc);
        }
    }
}   

void high_pass_filter(int32_t CurData[3], int32_t preFData[3], int32_t preData[3]){
    
    int32_t beta = 0.6;

    if (preFData[0] == 0) {
        preData[0] = CurData[0];
        preData[1] = CurData[1];
        preData[2] = CurData[2];
    }
    else {
        preData[0] = (preFData[0]*beta + (CurData[0] - preData[0])*beta)/10;
        preData[1] = (preFData[1]*beta + (CurData[1] - preData[1])*beta)/10;
        preData[2] = (preFData[2]*beta + (CurData[2] - preData[2])*beta)/10;
    
    }
}

void ACC_Calib(float *acc_ned, float acc_est[3], int N_acc){
    
    static float cal_acc[3] = {0, };
    static uint8_t cnt_acc = 0;
    static uint8_t cnt_cal_reset = 0;
    static uint8_t flag_acc_done = false;
    float norm_acc = 0.0;
    
    norm_acc = sqrtf(acc_ned[0]*acc_ned[0] + acc_ned[1]*acc_ned[1]);

    acc_est[0] = 0.0;
    acc_est[1] = 0.0;
    acc_est[2] = 0.0;
    if (cnt_acc < N_acc && norm_acc < 0.05){
            cnt_acc += 1;
            cal_acc[0] += acc_ned[0];
            cal_acc[1] += acc_ned[1];
            cal_acc[2] += acc_ned[2];
        
            if (cnt_acc >= 5){
                acc_est[0] = acc_ned[0] - cal_acc[0]/(float)(cnt_acc);
                acc_est[1] = acc_ned[1] - cal_acc[1]/(float)(cnt_acc);
                acc_est[2] = acc_ned[2] - cal_acc[2]/(float)(cnt_acc);
            }
            if (cnt_acc == N_acc) flag_acc_done = true;
    }
    if (cnt_acc == N_acc){
        if (flag_acc_done == true){
            printf("Accel Calib Done!!! \n");
            flag_acc_done = false;
        }
        if (norm_acc > 0.5) cnt_cal_reset++;
        //221125 else 추가
        else{
            cnt_cal_reset--;
            if (cnt_cal_reset < 0) cnt_cal_reset = 0;
        }
        if(cnt_cal_reset > 500){
                cnt_acc = 20;
                cnt_cal_reset = 0;
                memcpy(cal_acc,0,sizeof(cal_acc));
        }
        else{
            acc_est[0] = acc_ned[0] - cal_acc[0]/(float)cnt_acc;
            acc_est[1] = acc_ned[1] - cal_acc[1]/(float)cnt_acc;
            acc_est[2] = acc_ned[2] - cal_acc[2]/(float)cnt_acc;
        }
        
    }
}