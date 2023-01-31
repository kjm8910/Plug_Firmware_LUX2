#include "imu_dist.h"
/*
    No GPS -> IMU ONLY
*/

// IMU correct(LPF, MAF...)
// Numerical(RK4)
// Transform( B2E, q 2 euler...)
// Model
// 
// etc

// Data Input -> acc, gyro
// Data Filtering -> acc, gyro
// attitude -> complementary
// RK4 -> Position ->Dist
// Compare between RTK, GNSS and IMU
// OutPUT -> IMU DIST
extern float dt;
void acc2attitude(int32_t acc[3], int32_t  arr[3]){
    float ax, ay, az;
    float roll, pitch;

    ax = (float)(acc[0])/sf_acc;
    ay = (float)(acc[1])/sf_acc;
    az = (float)(acc[2])/sf_acc;

    roll = atan2f(az, ay); // Rad
    pitch = atan2f(sqrtf(ay*ay + az*az), ax);// Rad
    
    arr[0] = (int32_t)(roll*sf_att);
    arr[1] = (int32_t)(pitch*sf_att);
    arr[2] = 0;
}

void gyro2attitude(int32_t gyro[3], int32_t  arr[3]){
    float gx, gy, gz;
    //float roll, pitch, yaw;

    gx = (float)(gyro[0]/sf_gyro);
    gy = (float)(gyro[1]/sf_gyro);
    gz = (float)(gyro[2]/sf_gyro);

    arr[0] += (int32_t)(gx*dt*sf_att);
    arr[1] += (int32_t)(gy*dt*sf_att);
    arr[2] += (int32_t)(gz*dt*sf_att);
}

void ComplementaryFilter(int32_t att_acc[3], int32_t att_comp[3]){
    //float norm_att_comp, norm_att_acc;
    //norm_att_comp = VecNorm_att(att_comp,3);
    //norm_att_acc = VecNorm_acc(att_acc,3);
//    if (norm_att_comp == 0.0) return *att_acc;
//    else if (norm_att_acc == 0.0) return *att_comp;
    
    float beta, att[3] = {0, }, att_acc_f[3] = {0, };
    beta = 0.96;
    
    att[0] = (float)(att_comp[0] / sf_att);
    att[1] = (float)(att_comp[1] / sf_att);
    //att[2] = (float)(att_comp[2] / sf_att);
    
    att_acc_f[0] = (float)(att_acc[0] / sf_att);
    att_acc_f[1] = (float)(att_acc[1] / sf_att);
    att_acc_f[2] = (float)(att_acc[2] / sf_att);

    att_comp[0] = (int32_t)((att[0]*beta + att_acc_f[0]*(1 - beta))*sf_att);
    att_comp[1] = (int32_t)((att[1]*beta + att_acc_f[1]*(1 - beta))*sf_att);
    //att_comp[2] = (int32_t)(att[2]*sf_att);
}



void system_model(double xdot[6], double x[6], double u[3], uint8_t flag) {
    
    double Lat, Long, h;
    double vn, ve, vd;
    double g_hat;
    //

    Lat       = x[0];
    Long      = x[1];
    h         = x[2];

    vn        = x[3];
    ve        = x[4];
    vd        = x[5];

    double a_n, a_e, a_d;
    //float pos_ned_dot[3], v_ned_dot[3];
    double R_phi, R_lamda;
    R_phi       = a*(1 - e*e) / pow(1 - e*e*sin(Lat)*sin(Lat), 3/2);
    R_lamda     = a / pow(1 - pow(e*sin(Lat),2),1/2);

    a_n = u[0];
    a_e = u[1];
    a_d = u[2];
    
    if (flag == false){
        g_hat = 9.780327*(1+5.3024*1e-3*pow(sin(Lat),2)-5.8*1e-6*pow(sin(Lat*2.0),2))
        -(3.0877*1e-6-4.4*1e-9*pow(sin(Lat),2))*h+7.2*1e-14*h*h;
    }
    else{
        g_hat = 0.0;
    }
    // Position NED(Lat,Long, Height)
    xdot[0]   = vn/(R_phi+h);
    xdot[1]   = ve/((R_lamda+h)*cos(Lat));
    xdot[2]   = -vd;
    //Velocity
    xdot[3]   = -(ve/((R_lamda + h)*cos(Lat)) + 2.0*omega_e)*ve*sin(Lat)
                    + vn*vd/(R_phi+h) + a_n;
    xdot[4]   = (ve/((R_lamda+h)*cos(Lat)) + 2.0*omega_e)*vn*sin(Lat)
                    +ve*vd/(R_lamda+h) + 2.0*omega_e*vd*cos(Lat) + a_e;
    xdot[5]   = -ve*ve/(R_lamda+h) -vn*vn/(R_phi+h)
                    -2.0*omega_e*ve*cos(Lat)  + a_d + g_hat;
}


double calculate_dist(double cLat, double cLong, double pLat, double pLong){
    if (pLat == 0.0 || pLat == NAN || cLat == NAN) return 0.0;
    else {
        
        double dLat = (cLat - pLat);
        double dLon = (cLong - pLong);
        
        double Dist = 0.0;
        double C;
        C = sin(dLat/2.0)*sin(dLat/2.0)
        + cos(cLat)*cos(pLat)
            *sin(dLon/2.0)*sin(dLon/2.0);
        double CT;
        CT = 2.0*atan2(sqrt(C), sqrt(1 - C));
        Dist = (double)(a*CT);//meter

        if (Dist/(double)dt >= 10) Dist = 0.10;
        //else if(Dist <= 0.001) 
        //    Dist = 0.0;
        return Dist;
        
        /*
        double x, y, del_dist = 0;
        x = dLon*cosf((pLat + cLat)/2.0);
        y = dLat;
        del_dist = a * sqrtf(x*x + y*y);
        if (del_dist >= 0.27) del_dist = 0.2;
        return del_dist;*/
    }

}

void NED2BODY_Euler(int32_t att[3], float DCM[3][3]){
    float roll, pitch, yaw;
    
    roll = (float)(att[0]/sf_att);
    pitch = (float)(att[1]/sf_att);
    yaw = (float)(att[2]/sf_att);

    float c_r, s_r;
    float c_p, s_p;
    float c_y, s_y; 
    
    c_r    = cosf(roll);
    s_r    = sinf(roll);

    c_p   = cosf(pitch);
    s_p   = sinf(pitch);

    c_y     = cosf(yaw);
    s_y     = sinf(yaw);
    
    DCM[0][0] = c_y*c_p;
    DCM[1][0] = -s_y*c_r + c_y*s_p*s_r;
    DCM[2][0] = s_y*s_r + c_y*s_p*c_r;
    
    DCM[0][1] = s_y*c_p;
    DCM[1][1] = c_y*c_r + s_y*s_p*s_r;
    DCM[2][1] = -c_y*s_r + s_y*s_p*c_r;
    
    DCM[0][2] = -s_p;
    DCM[1][2] = c_p*s_r;
    DCM[2][2] = c_p*c_r;
}


void rk4_dist(double rk4_x_est[6], float acc_ned[3], 
            float acc_ned_est[3]){

    double k1[6] = {0, }, k2[6] = {0, };
    double k3[6] = {0, }, k4[6] = {0, };
    double x_k1[6] = {0, }, x_k2[6] = {0, };
    double x_k3[6] = {0, };
    int vec_size = 6;
    
    double acc[3] = {0,};

    uint8_t flag = true;
    float norm_acc_est = 0;
    norm_acc_est = norm_vec(3, acc_ned_est);
    if (fabs(acc_ned_est[2]) > 0.4 || norm_acc_est == 0.0){
        flag = false;
        acc[0] = (double)(acc_ned[0]); // acc_ned include gravity force
        acc[1] = (double)(acc_ned[1]);
        acc[2] = (double)(acc_ned[2]);
    }
    else {
        flag = true;
        acc[0] = (double)(acc_ned_est[0]);
        acc[1] = (double)(acc_ned_est[1]);
        acc[2] = (double)(acc_ned_est[2]);
    }
    system_model(k1, rk4_x_est, acc,flag);
    //for (int i = 0; i < vec_size;i++){
    //    k1[i] = k1[i] * dt;
    //}
    double k1_h[6] = {0, }, k2_h[6] = {0, };
    for (int i = 0; i < vec_size;i++){
        k1_h[i] = k1[i] * dt;
        x_k1[i] = rk4_x_est[i] + 0.5*k1_h[i];
    }
    //plus_vec(x_k1, rk4_x_est, k1_h, vec_size);
    
    system_model(k2, x_k1, acc,flag);
    for (int i = 0; i < vec_size;i++){
        k2_h[i] = k2[i] * dt;
        x_k2[i] = rk4_x_est[i] + 0.5*k2_h[i];
    }
    float k3_h[6] = {0, }, k4_h[6] = {0, };
    system_model(k3, x_k2, acc,flag);
    for (int i = 0; i < vec_size;i++){
        k3_h[i] = k3[i] * dt;
        x_k3[i] = rk4_x_est[i] + k3_h[i];
    }
    //plus_vec(x_k3, rk4_x_est, k3, vec_size);
    system_model(k4, x_k3, acc,flag);
    for (int i = 0; i < vec_size;i++){
        k4_h[i] = k4[i] * dt;
    }

    for (int i = 0; i < vec_size; i++){
        rk4_x_est[i] = rk4_x_est[i] + (k1_h[i]+ 2.0*(k2_h[i] + k3_h[i]) + k4_h[i])/6.0;
    }
    //rk4_x_est[0] *= RAD2DEG;
    //rk4_x_est[1] *= RAD2DEG;
    //return rk4_x_est;
    //for (int i = 0; i< 6 ;i++){
    //    x[i] = (int64_t)(rk4_x_est[i]*sf_state);
    //}
}


void euler2qut(int32_t att_euler[3], float qut[4]){

    float roll, pitch, yaw;

    roll = (float)(att_euler[0]/sf_att);
    pitch = (float)(att_euler[1]/sf_att);
    yaw = (float)(att_euler[2]/sf_att);
    
    qut[0] = sinf(roll/2.0) * cosf(pitch/2.0) * cosf(yaw/2.0) 
        - cosf(roll/2.0) * sinf(pitch/2.0) * sinf(yaw/2.0);
    qut[1] = cosf(roll/2.0) * sinf(pitch/2.0) * cosf(yaw/2.0) 
        + sinf(roll/2.0) * cosf(pitch/2.0) * sinf(yaw/2.0);
    qut[2] = cosf(roll/2.0) * cosf(pitch/2.0) * sinf(yaw/2.0) 
        - sinf(roll/2.0) * sinf(pitch/2.0) * cosf(yaw/2.0);
    qut[3] = cosf(roll/2.0) * cosf(pitch/2.0) * cosf(yaw/2.0) 
        + sinf(roll/2.0) * sinf(pitch/2.0) * sinf(yaw/2.0);

    quternion_normalization(qut);
}

void quternion_normalization(float qut[4]){
    float q_norm;

    q_norm = sqrtf(qut[0]*qut[0] + qut[1]*qut[1] + 
                    qut[2]*qut[2] + qut[3]*qut[3]);

    qut[0] /= q_norm;
    qut[1] /= q_norm;
    qut[2] /= q_norm;
    qut[3] /= q_norm;

}
void qut2euler(float q[4], float euler[3]){
        /*
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        */
        
        float x = q[0];
        float y = q[1];
        float z = q[2];
        float w = q[3];
        //float t0, t1,t2,t3,t4;
        float roll, pitch, yaw;
        // roll (x-axis rotation)
        float sinr_cosp = 2.0 * (w * x + y * z);
        float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = 2.0 * (w*y - z*x);
        // if (fabs(sinp) >= 1.0)
        //    pitch = copysign(3.14 / 2.0, sinp); // use 90 degrees if out of range
        // else
              
        pitch = asin(sinp);
        // yaw (z-axis rotation)
        float siny_cosp = 2.0 * (w*z + x*y);
        float cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
        yaw = atan2f(siny_cosp, cosy_cosp);
        
        euler[0] = roll;
        euler[1] = pitch;
        euler[2] = yaw;
        
}
void qut2dcm(float *qut, float DCM[3][3]){
    //memset(DCM, 0, sizeof(DCM));
    quternion_normalization(qut);
    float q1, q2, q3, q4;

    q1 = qut[0];
    q2 = qut[1];
    q3 = qut[2];
    q4 = qut[3];
    
    float OM[4][3] = {0, };
    float PHI[4][3] = {0, };
    float skew_qut[3][3] = {0, };
    float Eye[3][3] = {0, };

    Eye[0][0] = 1.0;
    Eye[1][1] = 1.0;
    Eye[2][2] = 1.0;
    float att_q[3] = {qut[0], qut[1], qut[2]};
    skew_symmetirc(att_q, skew_qut);
    
    for(int i = 0; i < 3;i++){
        for (int j = 0; j< 3;j++){
            OM[i][j] = q4*Eye[i][j] + skew_qut[i][j];
        }
    }
    OM[3][0] = -q1;
    OM[3][1] = -q2;
    OM[3][2] = -q3;

    for(int i = 0; i < 3;i++){
        for (int j = 0; j< 3;j++){
            PHI[i][j] = q4*Eye[i][j] - skew_qut[i][j];
        }
    }
    PHI[3][0] = -q1;
    PHI[3][1] = -q2;
    PHI[3][2] = -q3;
    
    float OM_T[3][4] = {0,};
    
    for(int i = 0; i < 3;i++){
        for(int j = 0; j< 4;j++){
            OM_T[i][j] = OM[j][i];
        }
    }
    
    for(int i = 0; i < 3;i++){
        for(int j = 0; j< 3;j++){
            for(int z = 0; z<4;z++){
                DCM[i][j] += OM_T[i][z]*PHI[z][j];
            }
        }
    }
    
}


void qut2Rot(float *qut, float Rot[3][3]){

    quternion_normalization(qut);
    float q1, q2, q3, q0;

    q1 = qut[0];
    q2 = qut[1];
    q3 = qut[2];
    q0 = qut[3];

    Rot[0][0] = 1.0 - 2.0*(q2*q2 + q3*q3);
    Rot[0][1] = 2.0*(q1*q2 - q0*q3);
    Rot[0][2] = 2.0*(q0*q2 + q1*q3);

    Rot[1][0] = 2.0*(q1*q2 + q0*q3);
    Rot[1][1] = 1.0 - 2.0*(q1*q1 + q3*q3);
    Rot[1][2] = 2.0*(q2*q3 - q0*q1);

    Rot[2][0] = 2.0*(q1*q3 - q0*q2);
    Rot[2][1] = 2.0*(q0*q1 + q2*q3);
    Rot[2][2] = 1.0 - 2.0*(q1*q1 + q2*q2);


}
