
#ifndef _IMU_DIST_H

#define _IMU_DIST_H
#include <stdlib.h> 
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "_kf.h"
#include "_matrix.h"

/////////////////////////////////////////////////////////////////

#define RAD2DEG                             180.0 / 3.141592
#define DEG2RAD                             3.141592 / 180.0
#define Gravity                             9.80665   // m / s2
#define N_Med                               10
#define sf_acc                              1000000.0
#define sf_att                              1000000.0  
#define sf_dcm                              1000000.0
#define sf_state                            1000000.0
#define sf_gyro                             1000000.0
/////////////////////////////////////////////////////////////////
// WGS - 84 Parameter

#define a                       6378137.0           // Semi - major Axis, meter

#define f                       298.257223563       // Flattening Factor of the Earth

#define omega_e                 7.292115*1e-5       // Nominal Mean Angular Velocity of the ω Earth
                                                    // rad/sec
#define GM                      3.986004418*1e+14   // Geocentric Gravitational Constant 
                                                    // (Mass of Earth’s Atmosphere Included)
                                                    // meter^3 / sec^2
#define g0                      9.780325            // Gravity at equator, m/s^2
#define e                       0.0818              // earth eccentricity           

/////////////////////////////////////////////////////////////////
void low_pass_filter(int32_t curData[3], int32_t preData[3]);
void ComplementaryFilter(int32_t att_acc[3], int32_t att_comp[3]);
void acc2attitude(float acc[3], int32_t  arr[3]);
void gyro2attitude(int32_t gyro[3], int32_t  arr[3]);
void system_model(double xdot[6], double x[6], double u[3], uint8_t flag);
void MedianFilter(float acc[3], float gyro[3], int32_t MedData_acc[3], int32_t MedData_gyro[3]);
void GYRO_MED(float gyro[3], int32_t MedData_gyro[3]);
void ACCEL_MED(float acc[3], int32_t MedData_acc[3]);
int compare(const void *T1 , const void *T2);
double calculate_dist(double cLat, double cLong, double pLat, double pLong);
void high_pass_filter(int32_t CurData[3], int32_t preFData[3], int32_t preData[3]);
void rk4_dist(double rk4_x_est[6], float acc_ned[3], 
            float acc_ned_est[3]);
void qut2Rot(float *qut, float Rot[3][3]);
double meter2degree(double meter, double degree);

#endif