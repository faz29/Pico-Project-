#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <stdint.h>

#define I2C_PORT i2c0
#define MPU6050_SDA 20
#define MPU6050_SCL 21

#define UART_PORT uart1
#define UART_TX 4
#define UART_RX 5
#define BAUD_RATE 115200

#define M1_pin 10
#define M2_pin 12
#define M3_pin
#define M4_pin

typedef struct {

    double PID;
    double P;
    double I;
    double D;
    double Kp;
    double Ki;
    double Kd;
    double E;
    double prevE;
    double prevTime;
    double pulse;
    double pv;
    double sp;

} pid_vars;

//====================================================================================================
#define RAD_TO_DEG 57.295779513082320876798154814105
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
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

//====================================================================================================

void i2c_initialisation(i2c_inst_t *port,uint freq);

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data, int stop);  

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width,int ledPin);

void esc_calibration(int pwm_pin,uint chan,int ledPin, int arm_sleep);

void led_on(int ledPin, bool state);

void read_throttle(int pwm_pin, int* c, uint16_t* throttleVal);

void read_brake(int pwm_pin, int* c, uint16_t* brake);

void read_controller(uart_inst_t* uart_port,int pwm_pin, int* t,int* b,uint16_t* throttle, uint16_t* brake);

void PID(double* E, double Kp, double Ki, double Kd,double* pulse1, double* pulse2, double* blm1, double* blm2);

void PIDStruct(pid_vars *Data);

uint8_t MPU6050_Init(i2c_inst_t *i2cPort);

void MPU6050_Read_Accel(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

void MPU6050_Read_All(i2c_inst_t *i2cPort, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);












































    
// uint8_t index = 0;
// uint8_t dpad[0];

// uint16_t buttons[1];
// uint8_t buttons[1];

// int32_t axisX = 0;
// int32_t axisY = 0;
// int32_t axisRX = 0;
// int32_t axisRY = 0;
// int32_t brake = 0;

// uint16_t miscButtons = 0;
// int32_t gyroX = 0;
// int32_t gyroY = 0;
// int32_t gyroZ = 0;
// int32_t accelX = 0;
// int32_t accelY = 0;
// int32_t accelZ = 0;















//     for(int k=0; k<1000;k++){
//         int delay = i2c_start_measurement(ctx);
//         if (delay < 0) {
//                 printf("Failed to initiate measurement...\n");
//                 return -1;
//         }
//         // sleep_ms(5);
        
//         res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
//         if (res) {
//                 printf("Failed to read measurements...\n");
//                 return -1;
//         }
//         printf("Temperature: %.5f C, Pressure -101230: %.5f hPa\n", temp, pressure);
//         }

//         printf("Initialisation measurements done !\nNow starting average measurements ...");
//         sleep_ms(2000);

//         int total = 1000;
//     for(int k=0; k<total;k++){
//         int delay = i2c_start_measurement(ctx);
//         if (delay < 0) {
//                 printf("Failed to initiate measurement...\n");
//                 return -1;
//         }
//         //sleep_ms(5);
//         res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
//         if (res) {
//                 printf("Failed to read measurements...\n");
//                 return -1;
//         }
//         printf("Temperature: %.5f C, Pressure -101230: %.5f hPa\n", temp, pressure);
//         avgTemp +=temp;
//         avgPressure += pressure;
//         }
//         printf("\nAverage Temperature: %.5f C, Average Pressure Pressure: %.5f hPa\n", avgTemp, avgPressure);

//         avgPressure = avgPressure/total;
//         avgTemp = avgTemp/total;
//         printf("\nAverage Temperature: %.5f C, Average Pressure Pressure: %.5f hPa\n", avgTemp, avgPressure);
