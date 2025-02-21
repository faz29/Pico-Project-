
#include <stdint.h>
#include "hardware/i2c.h"


#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
//#define MPU6050_ADDR 0xD0
#define MPU6050_ADDR 0x68


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


