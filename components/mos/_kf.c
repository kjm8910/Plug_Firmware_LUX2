#include "_kf.h"
#include "imu_dist_loop.h"
kf_ars_t kf_ars;
extern float dt = 0.01;  
float* kf_ars_loop(int32_t acc_lpf[3],int32_t gyro_lpf[3], uint32_t diff_time){
    static uint8_t flag_ars_init = false; 
    dt =(float)(diff_time)/1000.0;
    if (flag_ars_init == false) {
      // 1. Initialization
      memset(&kf_ars,0, sizeof(kf_ars)); 
      init_kf_ars(acc_lpf);
      flag_ars_init = true;
      
    }
    else{
    // Quternion Update
        // 2. Propagation
        qut_discrete(gyro_lpf);
        cov_discrete(gyro_lpf);
        // 3. Gain
        Kalman_Gain(acc_lpf);
        // 4. Update
        Measurement_Update(acc_lpf);
    }

    return kf_ars.x_est;
    
}
void Measurement_Update(int32_t acc_lpf[3]){
    float qut[4] = {0, };
    memcpy(qut, kf_ars.x_est, sizeof(qut));
    float acc[3] = {0, };
    acc[0] = (float)(acc_lpf[0] / sf_acc);
    acc[1] = (float)(acc_lpf[1] / sf_acc);
    acc[2] = (float)(acc_lpf[2] / sf_acc);
    
    float C_b2n[3][3] = {0, }, C_n2b[3][3] = {0, };
    qut2dcm(qut, C_n2b);
    Transpose_MAT_3(C_n2b, C_b2n);
    
    float acc_ref[3] = {0,0,Gravity};
    memset(kf_ars.acc_ref_b, 0, sizeof(kf_ars.acc_ref_b));
    dot_mat_vec(kf_ars.acc_ref_b, C_n2b, acc_ref, 3);

    float I_66[6][6] = {0, };
    for(int i = 0; i < 6; i++){
        I_66[i][i] = 1.0;
    }
    float P_1[6][6] = {0, };
    for(int i = 0 ; i < 6; i++){
        for(int j = 0; j < 6; j++){
            for(int k = 0; k < 3; k++){
                P_1[i][j] += kf_ars.Gain[i][k]*kf_ars.Hk[k][j];
            }
        }
    }
    for(int i = 0; i < 6; i++){
            for(int j = 0; j < 6; j++){// 221219 j = 3 -> 6
                P_1[i][j] *= -1.0;
            }
        }
    float P_2[6][6] = {0, };
    float P_est_k[6][6] = {0, };
    Matrix_Add_float_66(P_2, I_66, P_1, 6);
    dot_mat_mat_66(P_est_k,P_2,kf_ars.P_est,6);
    memcpy(kf_ars.P_est, P_est_k, sizeof(kf_ars.P_est));
    ////////////////////////
    
    float err_mea[3] = {0, };
    err_mea[0] = acc[0] - kf_ars.acc_ref_b[0];
    err_mea[1] = acc[1] - kf_ars.acc_ref_b[1];
    err_mea[2] = acc[2] - kf_ars.acc_ref_b[2];

    memset(kf_ars.del_x, 0, sizeof(kf_ars.del_x));
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 3; j++){
            kf_ars.del_x[i] += kf_ars.Gain[i][j]*err_mea[j];
        }
    }
    
    // Quternion Update
    
    float OM[4][3] = {0, };
    float skew_qut[3][3] = {0, };
    float Eye[3][3] = {0, };
    float qut_att[3] = {0, };
    float rQut[4] = {0, };

    Eye[0][0] = 1.0;
    Eye[1][1] = 1.0;
    Eye[2][2] = 1.0;

    qut_att[0] = qut[0];
    qut_att[1] = qut[1];
    qut_att[2] = qut[2];

    skew_symmetirc(qut_att, skew_qut);
    
    for(int i = 0; i < 3;i++){
        for (int j = 0; j< 3;j++){
            OM[i][j] = qut[3]*Eye[i][j] + skew_qut[i][j];
        }
    }
    OM[3][0] = -qut[0];
    OM[3][1] = -qut[1];
    OM[3][2] = -qut[2];
    for(int i = 0; i < 4;i++){
        for(int j = 0; j<3; j++){
            rQut[i] += 0.5*OM[i][j]*kf_ars.del_x[j];
        }
        rQut[i] += qut[i];
    }
    quternion_normalization(rQut);
    
    //int32_t euler[3] = {0, };
    //qut2euler(qut,euler);
    //euler[2] = 0.1*euler[2];
    //euler2qut(euler, qut);
    kf_ars.x_est[0] = rQut[0];
    kf_ars.x_est[1] = rQut[1];
    kf_ars.x_est[2] = rQut[2];
    kf_ars.x_est[3] = rQut[3];

    kf_ars.x_est[4] += kf_ars.del_x[3];
    kf_ars.x_est[5] += kf_ars.del_x[4];
    kf_ars.x_est[6] += kf_ars.del_x[5];        
}
void Kalman_Gain(int32_t acc_lpf[3]){
    float Hk[3][6] = {0, }, H_T[6][3] = {0, };
    float acc_ref[3] = {0,0,Gravity};
    float C_n2b[3][3] = {0, }, C_b2n[3][3] = {0, };
    float acc_ref_b[3] = {0, };
    float skew_ref_acc[3][3] = {0, };

    float qut[4] = {0, };
    qut[0] = kf_ars.x_est[0];
    qut[1] = kf_ars.x_est[1];
    qut[2] = kf_ars.x_est[2];
    qut[3] = kf_ars.x_est[3];
    
    float acc[3] = {0, };
    acc[0] = (float)(acc_lpf[0] / sf_acc);
    acc[1] = (float)(acc_lpf[1] / sf_acc);
    acc[2] = (float)(acc_lpf[2] / sf_acc);
    float acc_norm = 0.0;
    acc_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + 
                    acc[2]*acc[2]);

    //WMEC, Windowed Measurement Error Cov
    //static int cnt_err = 0;
    static float err_acc_save[10] = {0, };
    float norm_err = 0.0;
    norm_err = Gravity - acc_norm;
    //memmove(err_acc_save, err_acc_save + 1, sizeof(err_acc_save));

    for (int i = 0 ; i < 9; i++)
    {
        err_acc_save[i] = err_acc_save[i+1];
    }
    err_acc_save[9] = powf(norm_err, 2);
    float R_k_[3][3] = {0, };
    if (err_acc_save[0] != 0) {
        float err_sum = 0.0, alpha = 1e-5, err = 0.0;
        for(int i = 0 ; i < 10; i++){
            err += err_acc_save[i];
        }
        err_sum = expf(alpha*err);

        // Update Measurement Cov(R)
        for(int i = 0 ;i < 3;i++)
        {        
            R_k_[i][i] = kf_ars.R_k[i][i] * err_sum;
        }
    }
    else{
        memcpy(R_k_, kf_ars.R_k, sizeof(R_k_));
    }
        /*
    for(int i = 0 ; i < 9; i++){
        temp1 = err_acc_save[i];
        err_acc_save[i] = err_acc_save[i+1];
        err_acc_save[i+1] = temp1;
    }
    err_acc_save[9] = norm_err*norm_err;
    float err_sum = 0.0, alpha = 1e+1, err;
    err = 0.0;
    for(int i = 0 ; i < 10; i++){
        err += err_acc_save[i];
    }
    err_sum = expf(alpha*err/10.0);
    */
    memset(C_n2b, 0, sizeof(C_n2b));
    memset(C_b2n, 0, sizeof(C_b2n));
    
    qut2dcm(qut, C_n2b);
    Transpose_MAT_3(C_n2b, C_b2n);
    dot_mat_vec(acc_ref_b, C_n2b, acc_ref, 3);
    skew_symmetirc(acc_ref_b, skew_ref_acc);
    for(int i = 0; i<3;i++){
        kf_ars.acc_ref_b[i] = acc_ref_b[i];
        for(int j = 0; j<3;j++){
            Hk[i][j] = skew_ref_acc[i][j];
            kf_ars.Hk[i][j] = Hk[i][j];
        }
    }
    
    for(int i = 0 ;i < 3; i++){
        for(int j = 0; j < 6;j++){
            H_T[j][i] = Hk[i][j];
        }
    }
    float Gain_1[6][3] = {0, }, Gain_2[3][6] = {0, };
    float Gain_3[3][3] = {0, }, Gain_3_inv[3][3] = {0, };
    for(int i = 0;i < 6; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 6; k++){
                Gain_1[i][j] += kf_ars.P_est[i][k]*H_T[k][j];
            }
        }
    }
    for(int i = 0;i < 3; i++){
        for(int j = 0; j < 6; j++){
            for(int k = 0; k < 6; k++){
                Gain_2[i][j] += Hk[i][k]*kf_ars.P_est[k][j];
            }
        }
    }
    
    for(int i = 0;i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 6; k++){
                Gain_3[i][j] += Gain_2[i][k]*H_T[k][j];
            }
        }
    }
    for(int i = 0;i < 3; i++){
        for(int j = 0; j < 3; j++){
            Gain_3[i][j] = Gain_3[i][j] + R_k_[i][j];
        }
    }
    float det_gain = det(Gain_3);
    memset(kf_ars.Gain, 0, sizeof(kf_ars.Gain));
    if(det_gain != 0.0)
    {
      inversMatrix(Gain_3, Gain_3_inv);
      //Matrix_Inverse(Gain_3,Gain_3_inv);
    
      
      for(int i = 0; i< 6;i++){
          for(int j = 0; j < 3;j++){
              for(int k = 0; k < 3; k++){
                  kf_ars.Gain[i][j] += Gain_1[i][k]*Gain_3_inv[k][j];
              }
          }
      }
    }

}
void qut_discrete(int32_t gyro[3]){
    
    float psi[3] = {0, }, qut[4] = {0, };
    float sin_psi, norm_gyro, cos_psi;
    float gx, gy, gz;
    //float omega[3] = {0, };

    gx = (float)(gyro[0]/sf_gyro);
    gy = (float)(gyro[1]/sf_gyro);
    gz = (float)(gyro[2]/sf_gyro);

    for(int i = 0;i < 4;i++){
        qut[i] = kf_ars.x_est[i];
    }
    
    norm_gyro = sqrtf((gx*gx + gy*gy + gz*gz));
    sin_psi = sinf(0.5*norm_gyro*dt) / norm_gyro;

    psi[0] = sin_psi * gx;
    psi[1] = sin_psi * gy;
    psi[2] = sin_psi * gz;

    cos_psi = cosf(0.5*norm_gyro*dt);
    float OM[4][4] = {0, }, skew_psi[3][3] = {0, };
    skew_symmetirc(psi, skew_psi);
    for(int i = 0;i< 3;i++){
        OM[i][i] = cos_psi;
        for(int j = 0;j<3;j++){
            OM[i][j] -= skew_psi[i][j];
        }
    }

    for(int i = 0;i<3;i++){
        OM[i][3] = psi[i];
        OM[3][i] = -psi[i];
    }
    OM[3][3] = cos_psi;
    
    float qut_k[4] = {0, };
    for(int i = 0; i < 4;i++){
        for(int j = 0;j<4;j++){
            qut_k[i] += OM[i][j]*qut[j];
        }
    }
    quternion_normalization(qut_k);

    kf_ars.x_est[0] = qut_k[0];
    kf_ars.x_est[1] = qut_k[1];
    kf_ars.x_est[2] = qut_k[2];
    kf_ars.x_est[3] = qut_k[3];

}


void init_kf_ars(int32_t acc[3]){
    
    int32_t att_euler[3] = {0, };
    float qut[4] = {0, }, bias_gyro[3] = {0, };
    
    acc2attitude(acc, att_euler);
    euler2qut(att_euler, qut);
    //###### State ###########################################
    kf_ars.x_est[0] = qut[0];
    kf_ars.x_est[1] = qut[1];
    kf_ars.x_est[2] = qut[2];
    kf_ars.x_est[3] = qut[3];
    
    kf_ars.x_est[4] = bias_gyro[0];
    kf_ars.x_est[5] = bias_gyro[1];
    kf_ars.x_est[6] = bias_gyro[2];
    //###### Covariance ######################################
    for(int i = 0; i < 3; i++){
        kf_ars.P_est[i][i] = powf(1.1 * DEG2RAD, 2.0);
        kf_ars.P_est[i+3][i+3] = powf(30*DEG2RAD/3600, 2.0);
    }
    //####### Error Covariance ##################################
    // State Errror Covariance
    float Qk_11[3][3] = {0, }, const_11;
    float Qk_12[3][3] = {0, }, const_12;
    float Qk_21[3][3] = {0, }, const_21;
    float Qk_22[3][3] = {0, }, const_22;
    float I_33[3][3] = {0, };
    for(int i = 0;i<3;i++){
        I_33[i][i] = 1.0;
        Qk_11[i][i] = I_33[i][i];
        Qk_12[i][i] = I_33[i][i];
        Qk_21[i][i] = I_33[i][i];
        Qk_22[i][i] = I_33[i][i];
    }
    const_11 = powf(gyro_sigma_noise, 2)*(dt) + 
            1/3.0*powf(gyro_sigma_bias, 2)*(dt*dt*dt);
    dot_mat_const(Qk_11, const_11, 3);
    const_12 = 1/2.0*powf(gyro_sigma_bias, 2)*(dt*dt);
    dot_mat_const(Qk_12, const_12, 3);
    const_21 = 1/2.0*powf(gyro_sigma_bias, 2)*(dt*dt);
    dot_mat_const(Qk_21, const_21, 3);
    const_22 = powf(gyro_sigma_bias, 2)*(dt);
    dot_mat_const(Qk_22, const_22, 3);
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
            kf_ars.Q_k[i][j] = Qk_11[i][j];
            kf_ars.Q_k[i][j+3] = Qk_12[i][j];
            kf_ars.Q_k[i+3][j] = Qk_21[i][j];
            kf_ars.Q_k[i+3][j+3] = Qk_22[i][j];
        }
    }
    // Measurement Errror Covariance
    kf_ars.R_k[0][0] = powf(acc_sigma_noise, 2);
    kf_ars.R_k[1][1] = powf(acc_sigma_noise, 2);
    kf_ars.R_k[2][2] = powf(acc_sigma_noise, 2);

    //return kf_ars;
    
}

void cov_discrete(int32_t gyro[3]){

    float Gamma[6][6] = {0, }, Gamma_T[6][6] = {0, };
    float I_33[3][3] = {0, };
    
    for(int i=0; i<3; i++){
        I_33[i][i] = 1.0;
    }
    for(int i=0; i<3; i++){
        Gamma[i][i]     = -1.0;
        Gamma[i+3][i+3] = 1.0;
    }
    
    float fGyro[3] = {0, };
    float Phi[6][6] = {0, }, Phi_T[6][6] = {0, };
    float Phi_11[3][3] = {0, };
    float skew_omega_1[3][3] = {0, };
    float skew_omega_2[3][3] = {0, };
    float const_omega_1 = 0.0, const_omega_2 = 0.0, const_omega_3 = 0.0;

    fGyro[0] = (float)(gyro[0] / sf_gyro);
    fGyro[1] = (float)(gyro[1] / sf_gyro);
    fGyro[2] = (float)(gyro[2] / sf_gyro);
    
    // Phi 11
    skew_symmetirc(fGyro, skew_omega_1);
    dot_mat_mat_33(skew_omega_2, skew_omega_1, skew_omega_1, 3);
    float norm_gyro = 0.0;
    norm_gyro = sqrtf(fGyro[0]*fGyro[0] + fGyro[1]*fGyro[1] + 
                    fGyro[2]*fGyro[2]);
    const_omega_1 = sinf(norm_gyro*dt)/norm_gyro;
    dot_mat_const(skew_omega_1, -1.0*const_omega_1, 3);

    
    const_omega_2 = (1.0 - cosf(norm_gyro*dt))/(norm_gyro*norm_gyro);
    dot_mat_const(skew_omega_2, const_omega_2, 3);
    
    Matrix_Subtract_float_33(Phi_11, I_33, skew_omega_1, 3);
    Matrix_Add_float_33(Phi_11, Phi_11, skew_omega_2, 3);
     

     // Phi 12
     float Phi_12[3][3] = {0, };
     skew_symmetirc(fGyro, skew_omega_1);
     dot_mat_mat_33(skew_omega_2, skew_omega_1, skew_omega_1, 3);
     dot_mat_const(skew_omega_1, const_omega_2, 3);
     float I_33_dt[3][3] = {0, };
     memcpy(I_33_dt, I_33, sizeof(I_33));
     dot_mat_const(I_33_dt, dt, 3);
     Matrix_Subtract_float_33(Phi_12, skew_omega_1, I_33_dt,3);
     const_omega_3 = (norm_gyro*dt - sinf(norm_gyro*dt)) / powf(norm_gyro,3);
     dot_mat_const(skew_omega_2, const_omega_3, 3);
     Matrix_Subtract_float_33(Phi_12, Phi_12, skew_omega_2, 3);
     
    for(int i = 0;i < 3;i++){
        for(int j = 0; j < 3; j++)
        {
            Phi[i][j] = Phi_11[i][j];
        }
        for(int k = 3; k < 6; k++){
            Phi[i][k] = Phi_12[i][k-3];
        }
    }
    for(int i = 3;i < 6;i++){
        Phi[i][i] = 1.0;
    }

    Transpose_MAT_6(Phi, Phi_T);
    Transpose_MAT_6(Gamma, Gamma_T);
    float P_k1[6][6] = {0, }, P_k2[6][6] = {0, };
    dot_mat_mat_66(P_k1, Phi, kf_ars.P_est, 6);
    dot_mat_mat_66(P_k2, P_k1, Phi_T, 6);

    float Q_k_2[6][6] = {0, };
    float Q_k_1[6][6] = {0, };
    dot_mat_mat_66(Q_k_1, Gamma, kf_ars.Q_k, 6);
    dot_mat_mat_66(Q_k_2, Q_k_1, Gamma_T, 6);

    //memcpy(Q_k, kf_ars.Q_k, sizeof(Q_k));
    Matrix_Add_float_66(kf_ars.P_est, P_k2, Q_k_2,6);
} 