#ifndef IMU_DIST_LOOP_H
#define IMU_DIST_LOOP_H

#define LPF_ALPHA                           0.1
#define N_buf                               50
#define GYRO_OFFSET_NUM                     50

// Header ////////////////////////
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "imu_dist.h"
#include "_kf.h"
#include "_matrix.h"

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
#endif