#ifndef _MATRIX_H
#define _MATRIX_H

#include "imu_dist.h"

#define inv_size            3

float vec_sum(float *vec, int num);
void identity_matrix(int **arr, int size);
float VecNorm_acc(int32_t data[3], int len);
float VecNorm_att(int32_t data[3], int len);
void NED2BODY_Euler(int32_t att[3], float DCM[3][3]);
void Transpose_MAT_3(float MAT[3][3], float MAT_T[3][3]);
void Transpose_MAT_6(float MAT[6][6], float MAT_T[6][6]);
float dot(float **MAT, int len_mat, float *vec, int len_vec);
void plus_vec(float *vec, float *vec1, float *vec2, int size);

void _rk4(float x[4],int32_t u[3], int len);
void euler2qut(int32_t att_comp[3], float qut[4]);
void quternion_normalization(float qut[4]);
void qut2dcm(float *qut, float DCM[3][3]);
void skew_symmetirc(float data[3], float MAT[3][3]);
void qut2euler(float q[4], float euler[3]);
void Matrix_Subtract_float_33(float RESULT[3][3], float MAT_1[3][3], 
                            float MAT_2[3][3], int8_t size);
void Matrix_Add_float_33(float RESULT[3][3], float MAT_1[3][3], 
                            float MAT_2[3][3], int8_t size);
void Matrix_Add_float_66(float RESULT[6][6], float MAT_1[6][6], 
                            float MAT_2[6][6], int8_t size);
void dot_mat_vec(float result[3], float MAT[3][3],
                    float vec[3], int len);
void dot_mat_mat_33(float R[3][3], float MAT_1[3][3], float MAT_2[3][3], int8_t len);
void dot_mat_const(float Mat[3][3], float val, int8_t len);
void dot_mat_mat_66(float R[6][6], float MAT_1[6][6], float MAT_2[6][6], int8_t len);

void Matrix_Inverse(float MAT[3][3], float Inv_Result[3][3]);
float norm_vec(int num, float *x);
void inversMatrix(float matrix[][3], float invers[][3]);
float det(float m[][3]);
void errCov2SD(float P_est[6][6], float SD[6]);
float mean_vec(int size, float *x);
void Sleep_Mode_discrimination(float acc[3], float val);

#endif