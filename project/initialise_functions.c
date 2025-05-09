#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>
#include "initialise_functions.h"

//====================================================================================================

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void i2c_initialisation(i2c_inst_t *port,uint freq){
// I2C Initialisation. Using it at 400Khz.
    i2c_init(port, freq);

    gpio_set_function(MPU6050_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA);
    gpio_pull_up(MPU6050_SCL);


int P;
    if (port==i2c0){
       P = 0;
    }
    if (port==i2c1){
       P = 1;
    }

    printf("I2C Initialised at %dkHz on port %p !!\n", freq/1000, port);

}

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data_bits, int stop_bits){    
    
    //set up the UART TX and RX pins
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart_port,tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart_port,rx_pin));    

    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, data_bits, stop_bits, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_port, false);

    uart_init(uart_port, uart_Brate);
    printf("UART initialised at %d baud rate with TX on pin %d and RX on pin %d !\n", uart_Brate, tx_pin, rx_pin);
}

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width){
    gpio_set_function(pwm_pin,GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pwm_pin);

    //clk div of 125 gives a frequency of 1MHz meaning a per tick width of 1us.
    pwm_set_clkdiv(slice, 125);
    pwm_set_wrap(slice, 1000);
    pwm_set_chan_level(slice,chan,pulse_width); //maybe moved to main loop
    pwm_set_enabled(slice,true);    
    printf("PWM initialised on pin %d !\n",pwm_pin);

    sleep_ms(100);
}

void led_on(int ledPin, bool state){
    gpio_init(ledPin);
    gpio_set_dir(ledPin,GPIO_OUT);
    gpio_put(ledPin,state);

}

void read_controller_data_structure(uart_inst_t* uart_port, controller_vars *cdata){    

    uint8_t check[1];
    uint8_t controllerArray[25];   

    if (uart_is_readable(uart_port) == true){
        // while (true) {
            led_on(7,true);
            uart_read_blocking(uart_port, check, 1);
            if (check[0] == 0xFF) {
    //        }
        uart_read_blocking(uart_port, controllerArray, 24);
        check[0] = cdata->throttle = cdata->brake = cdata->rx = cdata->ry = cdata->lx = cdata->ly = 0;

        for (int j = 0; j<4; j++){
            cdata->throttle += (controllerArray[j]<<(8*(3-j))); 
            }
        for (int j = 4; j<8; j++){
            cdata->brake += (controllerArray[j]<<(8*(7-j)));
        }

        for (int j = 8; j<12; j++){
            cdata->rx += (controllerArray[j]<<(8*(11-j)));
        }
        for (int j = 12; j<16; j++){
            cdata->ry += (controllerArray[j]<<(8*(15-j)));
        }
        for (int j = 16; j<20; j++){
            cdata->lx += (controllerArray[j]<<(8*(19-j)));
        }
        for (int j = 20; j<24; j++){
            cdata->ly += (controllerArray[j]<<(8*(23-j)));
        }
    
}
}
    
}

void PIDStruct(pid_vars *pid){
    double dt, dE, FdE;
    uint64_t currTime = time_us_64();
    pid->E = pid->sp-pid->pv;

    pid->filteredE = (1 - pid->alpha) * pid->filteredE + pid->alpha * pid->E;

    dt = (currTime-pid->prevTime)/1000000.0;
    if (dt < 0.000001) {dt = 0.000001;}   
    // dE = pid->E - pid->prevE;
    FdE = pid->filteredE - pid->prevfilteredE;

    pid->P = pid->Kp*(pid->E);

    if(pid->pulse > 0 && pid->pulse < pid->max){pid->I += pid->Ki*pid->E*dt;}

    //double Derivative = dE / dt;
    double FDeriv = FdE/dt;
    
    // r = smoothing factor for lowpass filter on derivative term
    pid->filterD = pid->filterD * (1.0 - pid->r) + pid->r * FDeriv;
    
    pid->D = pid->Kd*pid->filterD;
    
    pid->pulse = pid->P + pid->I + pid->D;

    pid->prevTime = currTime;
    pid->prevE = pid->E;
    pid->prevfilteredE = pid->filteredE;

}

//ICM20948 Functions

int icm20948_init(icm20948_config_t *config) {
    uint8_t reg[2], buf;

    // wake up accel/gyro!
    // first write register then, write value
    reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // auto select clock source
    reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // disable accel/gyro once
    reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    sleep_ms(10);
    
    // enable accel/gyro (again)
    reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // check if the accel/gyro could be accessed
    reg[0] = WHO_AM_I_ICM20948;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_accel_gyro, &buf, 1, false);
#ifndef NDEBUG    
    printf("AG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0xEA) return -1;

    // switch to user bank 2
    reg[0] = REG_BANK_SEL; reg[1] = 0x20;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // gyro config
    //
    // set full scale to +-
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    //
    // set gyro output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 11 = 100
    reg[0] = GYRO_SMPLRT_DIV; reg[1] = 0x0A;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // accel config
    //
    // set full scale to +-2g
    // set noise bandwidth to 136Hz
    reg[0] = ACCEL_CONFIG; reg[1] = 0x11;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    //
    // set accel output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    // 16 bits for ACCEL_SMPLRT_DIV
    reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 0x0A;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    
    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);
    
    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = INT_PIN_CFG; reg[1] = 0x02;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, reg, 2, false);

    // check if the mag could be accessed
    reg[0] = 0x01;
    i2c_write_blocking(config->i2c, config->addr_mag, reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_mag, &buf, 1, false);
#ifndef NDEBUG
    printf("MAG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0x09) return -1;

    // config mag
    //
    // set mag mode, to measure continuously in 100Hz
    reg[0] = AK09916_CNTL2; reg[1] = 0x08;
    i2c_write_blocking(config->i2c, config->addr_mag, reg, 2, false);

    return 0;
}

void icm20948_Read_All(MPU6050_t *DataStruct, icm20948_config_t *config, icm20984_data_t *data){
    uint8_t buf[6];
    int16_t accel[3], gyro[3], mag[3];

    // accel: 2 bytes each axis
    uint8_t reg = ACCEL_XOUT_H; 
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    for (uint8_t i = 0; i < 3; i++) accel[i] -= data->accel_bias[i];

    // gyro: 2byte each axis
    reg = GYRO_XOUT_H;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    for (uint8_t i = 0; i < 3; i++) gyro[i] -= data->gyro_bias[i];

    reg = AK09916_XOUT_L;
    i2c_write_blocking(config->i2c, config->addr_mag, &reg, 1,true);
    i2c_read_blocking(config->i2c, config->addr_mag, buf, 8, false);

    for (int i = 0; i < 3; i++) mag[i] = (buf[(i * 2) + 1] << 8 | buf[(i * 2)]);
    for (uint8_t i = 0; i < 3; i++) mag[i] -= data->mag_bias[i];

    DataStruct->Accel_X_RAW = (int16_t)(accel[0]);
    DataStruct->Accel_Y_RAW = (int16_t)(accel[1]);
    DataStruct->Accel_Z_RAW = (int16_t)(accel[2]);
    DataStruct->Gyro_X_RAW = (int16_t)(gyro[0]);
    DataStruct->Gyro_Y_RAW = (int16_t)(gyro[1]);
    DataStruct->Gyro_Z_RAW = (int16_t)(gyro[2]);
    DataStruct->Mag_X_RAW = (int16_t)(mag[0]);
    DataStruct->Mag_Y_RAW = (int16_t)(mag[1]);
    DataStruct->Mag_Z_RAW = (int16_t)(mag[2]);


    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0f;

    DataStruct->Mx = DataStruct->Mag_X_RAW * 0.15f;
    DataStruct->My = DataStruct->Mag_Y_RAW * 0.15f;
    DataStruct->Mz = DataStruct->Mag_Z_RAW * 0.15f;

    double dt = (double) (time_us_32()-timer)/1E6;
    timer = time_us_32();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->pitch > 90) || (pitch > 90 && DataStruct->pitch < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KLMpitch = pitch;
    } else {
        DataStruct->KLMpitch = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->pitch) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KLMroll = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);
}

//calibration functions

void icm20948_cal_gyro(icm20948_config_t *config, int16_t gyro_bias[3]) {
    int16_t buf[3] = {0};
    int32_t bias[3] = {0};

    for (uint8_t i = 0; i < 100; i++) {
        icm20948_read_raw_gyro(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            bias[j] += buf[j];
        }
        sleep_ms(25);
    }
    for (uint8_t i = 0; i < 3; i++) gyro_bias[i] = (int16_t)(bias[i] / 200);
    
    return;
}

void icm20948_cal_accel(icm20948_config_t *config, int16_t accel_bias[3]) {
    int16_t buf[3] = {0};
    int32_t bias[3] = {0};

    for (uint8_t i = 0; i < 100; i++) {
        icm20948_read_raw_accel(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            if (j == 2) bias[j] += (buf[j] - 16384);
            else bias[j] += buf[j];
        }
        sleep_ms(25);
    }
    for (uint8_t i = 0; i < 3; i++) accel_bias[i] = (int16_t)(bias[i] / 200);
    return;
}

void icm20948_cal_mag_simple(icm20948_config_t *config, int16_t mag_bias[3]) {
    int16_t buf[3] = {0}, max[3] = {0}, min[3] = {0};
#ifndef NDEBUG
    printf("mag calibration: \nswing sensor for 360 deg\n");
#endif
    for (uint8_t i = 0; i < 1000; i++) {
        icm20948_read_raw_mag(config, buf);
        for (uint8_t j = 0; j < 3; j++) {
            if (buf[j] > max[j]) max[j] = buf[j];
            if (buf[j] < min[j]) min[j] = buf[j];
        }
        sleep_ms(10);
    }
    for (uint8_t i = 0; i < 3; i++) mag_bias[i] = (max[i] + min[i]) / 2;
    return;
}


//reading raw data

void icm20948_read_raw_accel(icm20948_config_t *config, int16_t accel[3]) {
    uint8_t buf[6];

    // accel: 2 bytes each axis
    uint8_t reg = ACCEL_XOUT_H;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);

    return;
}

void icm20948_read_raw_gyro(icm20948_config_t *config, int16_t gyro[3]) {
    uint8_t buf[6];

    // gyro: 2byte each axis
    uint8_t reg = GYRO_XOUT_H;
    i2c_write_blocking(config->i2c, config->addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    return;
}

void icm20948_read_raw_mag(icm20948_config_t *config, int16_t mag[3]) {
    uint8_t buf[8];

    uint8_t reg = AK09916_XOUT_L;
    i2c_write_blocking(config->i2c, config->addr_mag, &reg, 1,true);
    i2c_read_blocking(config->i2c, config->addr_mag, buf, 8, false);

    for (int i = 0; i < 3; i++) mag[i] = (buf[(i * 2) + 1] << 8 | buf[(i * 2)]);

#ifndef NDEBUG
    if ((buf[6] & 0x08) == 0x08) printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt){


    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;

}


//================================================================================================//    
//madgwick stuff

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, madgwick_ahrs_t *quat)
{
    float q1 = quat->q[0], q2 = quat->q[1], q3 = quat->q[2], q4 = quat->q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;


    // Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - quat->beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - quat->beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - quat->beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - quat->beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    quat->q[0] = q1 * norm;
    quat->q[1] = q2 * norm;
    quat->q[2] = q3 * norm;
    quat->q[3] = q4 * norm;
}

void ToEulerAngles(madgwick_ahrs_t *quat,MPU6050_t *DataStruct) {

    //w = 0, x = 1, y =2, z = 3    

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat->q[0] * quat->q[1] + quat->q[2] * quat->q[3]);
    double cosr_cosp = 1 - 2 * (quat->q[1] * quat->q[1] + quat->q[2] * quat->q[2]);
    DataStruct->roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat->q[0] * quat->q[2] - quat->q[3] * quat->q[1]);
    if (fabs(sinp) >= 1)
        DataStruct->pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        DataStruct->pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat->q[0] * quat->q[3] + quat->q[1] * quat->q[2]);
    double cosy_cosp = 1 - 2 * (quat->q[2] * quat->q[2] + quat->q[3] * quat->q[3]);
    DataStruct->yaw = atan2(siny_cosp, cosy_cosp) ;

    DataStruct->yaw += 0.8;
    if (DataStruct->yaw > PI) {
        DataStruct->yaw -= 2 * PI;
    }


}


//============================================
//old functions:

// void read_controller(uart_inst_t* uart_port,int pwm_pin, int* t,int* b,uint16_t* throttle, uint16_t* brake){    

//     //old function not needed
//     uint8_t check[1];
//     uint8_t controllerArray[9];
//     uint32_t brakeJoined;
//     uint32_t throttleJoined;    
//     uint slice = pwm_gpio_to_slice_num(pwm_pin);

//     if (uart_is_readable(uart_port) == true){
//         // while (true) {
//             led_on(7,true);
//             uart_read_blocking(uart_port, check, 1);
//             if (check[0] == 0xFF) {

//     //        }

//         uart_read_blocking(uart_port, controllerArray, 8);
//     check[0] = throttleJoined = brakeJoined = 0;

//     for (int j = 0; j<4; j++){
//         throttleJoined += (controllerArray[j]<<(8*(3-j))); 
//         }
//     for (int j = 4; j<8; j++){
//         brakeJoined += (controllerArray[j]<<(8*(7-j)));
//     }
    
//     *throttle = 125 + (125*throttleJoined)/1020;
//     *brake = 125 + (125*brakeJoined)/1020;

//     *t = throttleJoined;
//     *b = brakeJoined;    
// }
// }
    
// }


// void PID(double* E, double Kp, double Ki, double Kd,double* pulse1, double* pulse2, double* blm1, double* blm2){
//   //old function
//     double PID,P,D,dE,dt;
//     static double I = 0;
//     static double prevE = 0;
//     static uint64_t prevTime = 0;    
//     uint64_t currTime = time_us_64();
//     dt = (currTime-prevTime)/1000000.0;
//     if (dt < 0.000001) {dt = 0.000001;}   
//     dE = *E - prevE;
//     P = Kp*(*E);

//     if(*pulse1>0 && *pulse1<85){
//     I += Ki*(*E)*dt;}
//     double r = 0.01;

//     double Derivative = dE / dt;

//     static double filterD = 0;

//     filterD = filterD * (1.0 - r) + r * Derivative;
    
//     D = Kd*(filterD);      

//     *pulse1 = *blm1 + P + I + D; 
//     if(*pulse1>135){*pulse1 = 135;}
//     if(*pulse1<0){*pulse1 = 0;}

//     *pulse2 = (*blm2) - (P + I + D);
//     if(*pulse2>135){*pulse2 = 135;}
//     if(*pulse2<0){*pulse2 = 0;}

//     prevTime = currTime;
//     prevE = *E;
//         // printf("M1 PID: P=%f, I=%f, D=%f, PID=%f, pulse=%f\n", P, I, D, PID, *pulse1);
//         // printf("M2 PID: P=%f, I=%f, D=%f, PID=%f, pulse=%f\n", P, I, D, PID, *pulse2);

// }


// void esc_calibration(int pwm_pin,uint chan,int ledPin, int arm_sleep){
//     uint slice = pwm_gpio_to_slice_num(pwm_pin);
//     //arm sequence: zero throttle (125us), small delay, then full (250us), small delay then zero throttle.
//     pwm_set_chan_level(slice,chan,125); 
//     sleep_ms(arm_sleep); 
//     pwm_set_chan_level(slice,chan,250);
//     sleep_ms(arm_sleep);
//     pwm_set_chan_level(slice,chan,125); 

//     //turn on Led when arming is complete
//     gpio_init(ledPin);
//     gpio_set_dir(ledPin,GPIO_OUT);
//     gpio_put(ledPin,1);
//     printf("Arm sequence complete!\n");
// }
