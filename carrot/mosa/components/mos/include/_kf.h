#ifndef _KF_H
#define _KF_H

#include "imu_dist.h"
#include <stdint.h>
#include <stdbool.h>
// ### Sensor Spec.##################################
// Accelerometer
#define acc_sigma_noise         10.0*1e-3*Gravity
#define acc_sigma_bias          1.0*1e-3*Gravity
#define acc_sigma_sf            1000.0*1e-6
 // Gyro
#define gyro_sigma_noise        0.07 * DEG2RAD
#define gyro_sigma_bias         500.0/3600.0 * DEG2RAD 
#define gyro_sigma_sf           1000.0*1e-6

typedef struct {
    float del_x[6]; // [] x, y, z
    float x_est[7]; // [] x, y, z,

    float P_est[6][6];
    float Q_k[6][6];
    float R_k[3][3];
    float Gain[6][3]; // Kalman Gain
    float Hk[3][6];
    float acc_ref_b[3];

}kf_ars_t;

float SystemModel_ars(float q[4], float Omega[3]) ;
void qut_discrete(int32_t gyro[3]);
void Kalman_Gain(int32_t acc_lpf[3]);
void init_kf_ars(int32_t acc[3]);
void cov_discrete(int32_t gyro[3]);
float* kf_ars_loop(int32_t acc_lpf[3], int32_t gyro[3], uint32_t diff_time);
void Measurement_Update(int32_t acc_lpf[3]);
#endif