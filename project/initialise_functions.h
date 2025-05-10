#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <stdint.h>

#define I2C_PORT i2c0
#define MPU6050_SDA 4
#define MPU6050_SCL 5

#define ICM20948_ADDR 0x68
#define REG_BANK_SEL  0x7F

#define UART_PORT uart1
#define UART_TX 8
#define UART_RX 9
#define BAUD_RATE 115200

#define M1_pin 10 //front right
#define M2_pin 12 //front left
#define M3_pin 14 //back right
#define M4_pin 16 //back left   

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

//-----IMU DEFINITIONS---------

#ifndef ICM20948_REG_H
#define ICM20948_REG_H

// Mag
#define WHO_AM_I_AK09916 0x01 
#define AK09916_ST1      0x10  
#define AK09916_XOUT_L   0x11  
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  
#define AK09916_CNTL     0x30  
#define AK09916_CNTL2    0x31  

// ICM20948
// User Bank 0
#define WHO_AM_I_ICM20948  0x00 
#define USER_CTRL          0x03  
#define LP_CONFIG		   0x05 
#define PWR_MGMT_1         0x06 
#define PWR_MGMT_2         0x07
#define INT_PIN_CFG        0x0F
#define INT_ENABLE         0x10
#define INT_ENABLE_1	   0x11 
#define INT_ENABLE_2	   0x12 
#define INT_ENABLE_3	   0x13 
#define I2C_MST_STATUS     0x17
#define INT_STATUS         0x19
#define INT_STATUS_1	   0x1A 
#define INT_STATUS_2	   0x1B 
#define INT_STATUS_3	   0x1C 
#define DELAY_TIMEH		   0x28	
#define DELAY_TIMEL		   0x29	
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38
#define TEMP_OUT_H         0x39
#define TEMP_OUT_L         0x3A
#define EXT_SENS_DATA_00   0x3B
#define EXT_SENS_DATA_01   0x3C
#define EXT_SENS_DATA_02   0x3D
#define EXT_SENS_DATA_03   0x3E
#define EXT_SENS_DATA_04   0x3F
#define EXT_SENS_DATA_05   0x40
#define EXT_SENS_DATA_06   0x41
#define EXT_SENS_DATA_07   0x42
#define EXT_SENS_DATA_08   0x43
#define EXT_SENS_DATA_09   0x44
#define EXT_SENS_DATA_10   0x45
#define EXT_SENS_DATA_11   0x46
#define EXT_SENS_DATA_12   0x47
#define EXT_SENS_DATA_13   0x48
#define EXT_SENS_DATA_14   0x49
#define EXT_SENS_DATA_15   0x4A
#define EXT_SENS_DATA_16   0x4B
#define EXT_SENS_DATA_17   0x4C
#define EXT_SENS_DATA_18   0x4D
#define EXT_SENS_DATA_19   0x4E
#define EXT_SENS_DATA_20   0x4F
#define EXT_SENS_DATA_21   0x50
#define EXT_SENS_DATA_22   0x51
#define EXT_SENS_DATA_23   0x52
#define FIFO_EN_1          0x66
#define FIFO_EN_2          0x67 
#define FIFO_RST		   0x68 
#define FIFO_MODE		   0x69 
#define FIFO_COUNTH        0x70
#define FIFO_COUNTL        0x71
#define FIFO_R_W           0x72
#define DATA_RDY_STATUS	   0x74 
#define FIFO_CFG		   0x76 
#define REG_BANK_SEL	   0x7F 

// User Bank 1
#define SELF_TEST_X_GYRO  			0x02
#define SELF_TEST_Y_GYRO  			0x03
#define SELF_TEST_Z_GYRO  			0x04
#define SELF_TEST_X_ACCEL 			0x0E
#define SELF_TEST_Y_ACCEL 			0x0F
#define SELF_TEST_Z_ACCEL 			0x10
#define XA_OFFSET_H       			0x14
#define XA_OFFSET_L       			0x15
#define YA_OFFSET_H       			0x17
#define YA_OFFSET_L       			0x18
#define ZA_OFFSET_H       			0x1A
#define ZA_OFFSET_L       			0x1B
#define TIMEBASE_CORRECTION_PLL		0x28

// User Bank 2
#define GYRO_SMPLRT_DIV        	0x00 
#define GYRO_CONFIG_1      		0x01 
#define GYRO_CONFIG_2      		0x02 
#define XG_OFFSET_H       		0x03  
#define XG_OFFSET_L       		0x04
#define YG_OFFSET_H       		0x05
#define YG_OFFSET_L       		0x06
#define ZG_OFFSET_H       		0x07
#define ZG_OFFSET_L       		0x08
#define ODR_ALIGN_EN			0x09 
#define ACCEL_SMPLRT_DIV_1     	0x10 
#define ACCEL_SMPLRT_DIV_2     	0x11 
#define ACCEL_INTEL_CTRL		0x12 
#define ACCEL_WOM_THR			0x13 
#define ACCEL_CONFIG      		0x14
#define ACCEL_CONFIG_2     		0x15 
#define FSYNC_CONFIG			0x52 
#define TEMP_CONFIG				0x53 
#define MOD_CTRL_USR			0x54 

// User Bank 3
#define I2C_MST_ODR_CONFIG		0x00 
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17

#endif // ICM20948_REG_H

#define sampleFreq	100.0f	
#define PI 3.14159624f


//struct to hold all the pid variables
typedef struct {
//make sure to iniitalise these values that need to be 0 in the main function
    double P;
    double I;   //init 0
    double D;
    double Kp;
    double Ki;
    double Kd;
    double E;      //init 0
    double prevE;   //init 0
    double prevTime;    //init 0
    double pulse;   //init 0
    double filterD;
    double pv;
    double sp;

    double gyropv;
    
    int max;
    float r;


    float alpha;
    float filteredE;
    float prevfilteredE;
} pid_vars;

//struct to hold controller data
typedef struct{

    uint32_t brake;
    uint32_t throttle;
    int32_t rx;
    int32_t ry;
    int32_t lx;
    int32_t ly;    

} controller_vars;

//===================================================================================================
// IMU2096 structures 
typedef struct icm20948_config {
    // usual addr
    // addr_accel_gyro:  0x68
    // addr_mag:         0x0C
    uint8_t    addr_accel_gyro;
    uint8_t    addr_mag;
    // example
    // i2c_inst_t icm20948_i2c = {i2c0_hw, false}
    i2c_inst_t *i2c;
} icm20948_config_t;

typedef struct icm20984_data {
    // 0: x, 1: y, 2: z
    int16_t accel_raw[3];
    int16_t accel_bias[3];
    int16_t gyro_raw[3];
    int16_t gyro_bias[3];
    int16_t mag_raw[3];
    int16_t mag_bias[3];
    float temp_c;
} icm20984_data_t;

typedef struct madgwick_ahrs {
    volatile float beta;
    volatile float q[4];
} madgwick_ahrs_t;

//====================================================================================================
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

    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;
    double Mx;
    double My;
    double Mz;

    float Temperature;

    double roll;
    double pitch;
    double yaw;

    double KLMroll;
    double KLMpitch;
    double KLMyaw;


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

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width);

void led_on(int ledPin, bool state);

void read_controller_data_structure(uart_inst_t* uart_port, controller_vars *cdata);

void PIDStruct(pid_vars *Data);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

int icm20948_init(icm20948_config_t *config);

void icm20948_cal_gyro(icm20948_config_t *config, int16_t gyro_bias[3]);

void icm20948_cal_accel(icm20948_config_t *config, int16_t accel_bias[3]);

void icm20948_read_raw_accel(icm20948_config_t *config, int16_t accel[3]);

void icm20948_read_raw_gyro(icm20948_config_t *config, int16_t gyro[3]);

void icm20948_read_raw_mag(icm20948_config_t *config, int16_t mag[3]);

void icm20948_Read_All(MPU6050_t *DataStruct,icm20948_config_t *config,icm20984_data_t *data);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, madgwick_ahrs_t *quat);

void ToEulerAngles(madgwick_ahrs_t *quat, MPU6050_t *DataStruct);

































    
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
