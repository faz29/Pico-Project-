
#include <stdint.h>
#include "hardware/i2c.h"

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(i2c_inst_t *i2cPort);

void MPU6050_Read_Accel(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_All(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


