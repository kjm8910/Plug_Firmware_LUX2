#include "_matrix.h"
#include <stdlib.h>
extern float dt;

void identity_matrix(int **arr, int size)
{

    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            if (i == j)
                arr[i][j] = 1;
            else
                arr[i][j] = 0;
        }
    }
}

float VecNorm_acc(int32_t data[3], int len)
{

    float sumData, Result;
    float acc[3] = {
        0,
    };
    sumData = 0;

    acc[0] = (float)(data[0] / sf_acc);
    acc[1] = (float)(data[1] / sf_acc);
    acc[2] = (float)(data[2] / sf_acc);

    for (int i = 0; i < len; i++)
    {
        sumData += acc[i] * acc[i];
    }

    Result = sqrtf(sumData);

    return Result;
}
float VecNorm_att(int32_t data[3], int len)
{

    float sumData, Result;
    float att[3] = {
        0,
    };
    sumData = 0.0;

    att[0] = (float)(data[0] / sf_att);
    att[1] = (float)(data[1] / sf_att);
    att[2] = (float)(data[2] / sf_att);

    for (int i = 0; i < len; i++)
    {
        sumData += att[i] * att[i];
    }

    Result = sqrtf(sumData);

    return Result;
}

void plus_vec(float *vec, float *vec1, float *vec2, int size)
{
    // vec = malloc(sizeof(float)*size);
    for (int i = 0; i < size; i++)
    {
        *(vec + i) = *(vec1 + i) + *(vec2 + i);
    }
}

void skew_symmetirc(float data[3], float MAT[3][3])
{

    float x = data[0];
    float y = data[1];
    float z = data[2];
    memset(MAT[0], 0, sizeof(MAT[0]));
    memset(MAT[1], 0, sizeof(MAT[1]));
    memset(MAT[2], 0, sizeof(MAT[2]));

    MAT[0][0] = 0.0;
    MAT[0][1] = -z;
    MAT[0][2] = y;

    MAT[1][0] = z;
    MAT[1][1] = 0.0;
    MAT[1][2] = -x;

    MAT[2][0] = -y;
    MAT[2][1] = x;
    MAT[2][2] = 0.0;
}

void _rk4(float x[4], int32_t u[3], int len)
{
    float k1[4] = {
        0,
    },
          k2[4] = {
              0,
          };
    float k3[4] = {
        0,
    },
          k4[4] = {
              0,
          };
    float x_k1[4] = {
        0,
    },
          x_k2[4] = {
              0,
          };
    float x_k3[4] = {
        0,
    };
    int vec_size = len;
    float q[4] = {
        0,
    };
    q[0] = x[0];
    q[1] = x[1];
    q[2] = x[2];
    q[3] = x[3];

    float gyro[3] = {
        0,
    };
    gyro[0] = (float)(u[0] / sf_acc);
    gyro[1] = (float)(u[1] / sf_acc);
    gyro[2] = (float)(u[2] / sf_acc);

    float qdot[4] = {
        0,
    };
    *qdot = SystemModel_ars(q, gyro);
    for (int i = 0; i < vec_size; i++)
    {
        k1[i] = qdot[i] * 0.5 * dt;
    }
    plus_vec(x_k1, q, k1, vec_size);
    memset(qdot, 0, sizeof(qdot));
    *qdot = SystemModel_ars(x_k1, gyro);
    for (int i = 0; i < vec_size; i++)
    {
        k2[i] = qdot[i] * 0.5 * dt;
    }
    plus_vec(x_k2, q, k2, vec_size);

    *qdot = SystemModel_ars(x_k2, gyro);
    for (int i = 0; i < vec_size; i++)
    {
        k3[i] = qdot[i] * 0.5 * dt;
    }

    plus_vec(x_k3, q, k3, vec_size);
    *qdot = SystemModel_ars(x_k3, gyro);
    for (int i = 0; i < vec_size; i++)
    {
        k4[i] = qdot[i] * dt;
    }

    for (int i = 0; i < vec_size; i++)
    {
        x[i] = x[i] + (k1[i] + 2.0 * (k2[i] + k3[i]) + k4[i]) / 6.0;
    }
}

void Matrix_Subtract_float_33(float RESULT[3][3], float MAT_1[3][3],
                              float MAT_2[3][3], int8_t size)
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            RESULT[i][j] = MAT_1[i][j] - MAT_2[i][j];
        }
    }
}

void Matrix_Add_float_33(float RESULT[3][3], float MAT_1[3][3],
                         float MAT_2[3][3], int8_t size)
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            RESULT[i][j] = MAT_1[i][j] + MAT_2[i][j];
        }
    }
}
void Matrix_Add_float_66(float RESULT[6][6], float MAT_1[6][6],
                         float MAT_2[6][6], int8_t size)
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            RESULT[i][j] = MAT_1[i][j] + MAT_2[i][j];
        }
    }
}
void Transpose_MAT_3(float MAT[3][3], float MAT_T[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            MAT_T[i][j] = MAT[j][i];
        }
    }
}

void Transpose_MAT_6(float MAT[6][6], float MAT_T[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            MAT_T[i][j] = MAT[j][i];
        }
    }
}

void dot_mat_vec(float result[3], float MAT[3][3],
                 float vec[3], int len)
{

    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len; j++)
        {
            result[i] += MAT[i][j] * vec[j];
        }
    }
}

void dot_mat_mat_33(float R[3][3], float MAT_1[3][3], float MAT_2[3][3], int8_t len)
{
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len; j++)
        {
            for (int k = 0; k < len; k++)
            {
                R[i][j] += MAT_1[i][k] * MAT_2[k][j];
            }
        }
    }
}
void dot_mat_mat_66(float R[6][6], float MAT_1[6][6], float MAT_2[6][6], int8_t len)
{
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len; j++)
        {
            for (int k = 0; k < len; k++)
            {
                R[i][j] += MAT_1[i][k] * MAT_2[k][j];
            }
        }
    }
}
void dot_mat_const(float Mat[3][3], float val, int8_t len)
{

    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len; j++)
        {
            Mat[i][j] *= val;
        }
    }
}

void Matrix_Inverse(float MAT[3][3], float Inv_Result[3][3])
{
    // Determinant
    float sum = 0;
    // Gauian Jordan method
    // int size = 3;

    int size = 3;

    float **numPtr1 = malloc(sizeof(float) * size);
    for (int i = 0; i < size; i++)
    {
        numPtr1[i] = malloc(sizeof(float) * size);
    }
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            numPtr1[i][j] = MAT[i][j];
        }
    }

    float **numPtr2 = malloc(sizeof(float) * size);
    for (int i = 0; i < size; i++)
    {
        numPtr2[i] = malloc(sizeof(float) * size);
    }
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            if (i == j)
            {
                numPtr2[i][j] = 1.0;
            }
            else
                numPtr2[i][j] = 0.0;
        }
    }

    for (int p = 0; p < size; p++)
    {
        if (p < size - 1)
        {
            if (numPtr1[p][p] != 0)
            {
                float temp1 = 1.0 / numPtr1[p][p];
                for (int j = 0; j < size; j++)
                {
                    numPtr1[p][j] = temp1 * numPtr1[p][j];
                    numPtr2[p][j] = temp1 * numPtr2[p][j];
                }

                for (int i = p + 1; i < size; i++)
                {
                    if (numPtr1[i][p] != 0)
                    {
                        float temp2 = -(numPtr1[i][p] / numPtr1[p][p]);
                        for (int j = 0; j < size; j++)
                        {
                            numPtr1[i][j] = temp2 * numPtr1[p][j] + numPtr1[i][j];
                            numPtr2[i][j] = temp2 * numPtr2[p][j] + numPtr2[i][j];
                        }
                    }
                }
            }
        }
        if (p == size - 1)
        {
            if (numPtr1[p][p] != 0)
            {
                float temp1 = 1.0 / numPtr1[p][p];
                for (int j = 0; j < size; j++)
                {
                    numPtr1[p][j] = temp1 * numPtr1[p][j];
                    numPtr2[p][j] = temp1 * numPtr2[p][j];
                }
            }
        }
    }

    for (int i = size - 1; i >= 0; i--)
    {
        for (int j = size - 1; j >= 0; j--)
        {
            if (j < i && numPtr1[j][i] != 0)
            {
                double temp = -numPtr1[j][i];
                for (int p = 0; p < size; p++)
                {
                    numPtr1[j][p] = temp * numPtr1[i][p] + numPtr1[j][p];
                    numPtr2[j][p] = temp * numPtr2[i][p] + numPtr2[j][p];
                }
            }
            else
            {
                numPtr1[j][i] = numPtr1[j][i];
                numPtr2[j][i] = numPtr2[j][i];
            }
        }
    }

    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            sum += numPtr1[i][j];
        }
    }
    /*
    if (sum != size)
    {
        //return 0;
    }*/
    if (sum == size)
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                Inv_Result[i][j] = numPtr2[i][j];
            }
        }
    }
    for (int i = 0; i < size; i++)
    {
        free(numPtr1[i]);
    }
    free(numPtr1);

    for (int j = 0; j < size; j++)
    {
        free(numPtr2[j]);
    }
    free(numPtr2);
}

float norm_vec(int num, float *x)
{
    float norm_x = 0.0;

    for (int i = 0; i < num; i++)
    {
        norm_x += x[i] * x[i];
    }
    norm_x = sqrtf(norm_x);

    return norm_x;
}

void inversMatrix(float matrix[][3], float invers[][3])
{

    float determinant = det(matrix);
    
    if (determinant == 0.0)
    {
        memset(invers[0], 0, sizeof(invers[0]));
        memset(invers[1], 0, sizeof(invers[1]));
        memset(invers[2], 0, sizeof(invers[2]));
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            invers[i][j] = 1.0 / determinant *
                (matrix[(i + 1) % 3][(j + 1) % 3] *
                 matrix[(i + 2) % 3][(j + 2) % 3] - 
                 matrix[(i + 1) % 3][(j + 2) % 3] * 
                 matrix[(i + 2) % 3][(j + 1) % 3]);
        }
    }
}

float det(float m[][3])
{
    float d = 0.0;

    d = (m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1]);

    d -= (m[0][2] * m[1][1] * m[2][0] + m[2][1] * m[1][2] * m[0][0] + m[2][2] * m[1][0] * m[0][1]);

    return d;
}

void errCov2SD(float P_est[6][6], float SD[6]){
    for(int i = 0;i < 6;i++){
        SD[i] = sqrtf(P_est[i][i]);
    }
}

float mean_vec(int size, float *x){
    float Ave = 0.0;
    for(int i = 0;i<size;i++)
    {
        Ave += x[i];
    }
    Ave /= size;

    return Ave;
}

float vec_sum(float *vec, int num){
    float result = 0.0;
    for(int i = 0; i<num; i++){
        result += *(vec + i);
    }
   return result;
}
extern int flag_sleep_mode;
//extern int cnt_sleep_mode;
//extern float var_acc_ext;
void Sleep_Mode_discrimination(float acc[3], float val){
    static int cnt_wake_up = 0;
    static int cnt_sleep_mode = 0;
    static int sleep_mode_time = 0;
    static float buff_acc[50] = {0, };
    
    float var_acc = 0.0;

    // Calculation Norm
    float acc_norm = 0.0;
    
    acc_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + 
                    acc[2]*acc[2]);
    if (acc_norm > 0.5){
        //float tmp = buff_acc[0];
        for(int i = 0; i < 49; i++)
        {
            // ACC
            buff_acc[i] = buff_acc[i + 1];
        }
        buff_acc[49] = acc_norm;
        
        float mean_acc = 0.0;
        if (buff_acc[0] != 0.0){

            //Calculation Mean
            for(int i = 0; i < 50 ;i++){
                mean_acc += buff_acc[i];
            }
            mean_acc /= 50.0;
            
            // Variance
            for(int i = 0; i < 50 ;i++){
                var_acc += (buff_acc[i] - mean_acc) * 
                            (buff_acc[i] - mean_acc);
            }
            var_acc /= 50.0;
            int SleepTime = 10*60*100;// 10 Min X 60 Sec X  X 100Hz
            // Sleep Mode 
            sleep_mode_time++;
            if(flag_sleep_mode == false && 
            sleep_mode_time <= SleepTime && var_acc < 1e-4){
                cnt_sleep_mode++;
            }
            else if(sleep_mode_time > SleepTime ){
                cnt_sleep_mode = 0;
                sleep_mode_time = 0;
            }
            if(flag_sleep_mode == false && 
                cnt_sleep_mode > SleepTime*9/10){
                flag_sleep_mode = true;
                cnt_sleep_mode = 0;
            }
            else if(var_acc >= 0.0001*val){
                cnt_wake_up++;
                if (cnt_wake_up > 100) { // 1 sec X 100Hz
                    flag_sleep_mode = false;
                    cnt_wake_up = 0;
                    cnt_sleep_mode = 0;
                }
               
                
            }
        }
    }
}