#ifndef IMU_DIST_LOOP_H
#define IMU_DIST_LOOP_H

#define LPF_ALPHA                           0.1
#define N_buf                               50
#define GYRO_OFFSET_NUM                     100

// Header ////////////////////////
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "imu_dist.h"
#include "_kf.h"
#include "_matrix.h"

///////////////////////////////////////

////////////////////////////////////

typedef struct {
    float   acc[3]; // [] x, y, z
    float   gyro[3]; // [] x, y, z
} IMU_DATA_T;

typedef struct {
    double  Lat;
    double  Long;
    float   Height;
    float   DOP;
} GNSS_DATA_T;

// Function
double dist_Loop(float acc[3], float gyro[3], double pos_gnss_data[2], 
                            uint8_t flag_plug_off, uint8_t flag_gnss_state, uint32_t diff_time);
void ACC_Calib(float *acc_ned, float acc_est[3], int N_acc);
void imu_unit_conv(float *acc, float *gyro);
void gyro_offset_elimination(float *gyro);
void define_vehicle_stop(float *bias_gyro, float *pre_bias, 
                         uint8_t cnt_ars);
void Estimation_State(float *bias_gyro, float *pre_bias, uint8_t cnt_ars,
                      float *acc_ned, float *acc_ned_est, float *gyro);
void imu_noise_filtering(float acc[3], float gyro[3]);


#endif