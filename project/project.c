#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <time.h>
#include <math.h>

#include "pico_sensor_lib.h"
#include "initialise_functions.h"

i2c_inst_t icm20948_i2c = {i2c0_hw, false};
icm20948_config_t config = {0x68, 0x0C, &icm20948_i2c};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.005f, {1.0f, 0.0f, 0.0f, 0.0f}};


int main() {
    int pwm;
    float AngX, AngY;   
    MPU6050_t MPU6050;

    //initialise all structures to 0
    controller_vars cdata = {0};    
    pid_vars yaw = {0};
    pid_vars roll = {0};
    pid_vars pitch = {0};     

    stdio_init_all();
    
    i2c_initialisation(I2C_PORT,400*1000);

    uart_initialisation(UART_PORT,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    sleep_ms(100);
    pwm_initialisation(M1_pin,0,125);
    pwm_initialisation(M2_pin,0,125);
    pwm_initialisation(M3_pin,0,125);
    pwm_initialisation(M4_pin,0,125);

    sleep_ms(300);

    uint sliceM1 = pwm_gpio_to_slice_num(M1_pin);
    uint sliceM2 = pwm_gpio_to_slice_num(M2_pin);
    uint sliceM3 = pwm_gpio_to_slice_num(M3_pin);
    uint sliceM4 = pwm_gpio_to_slice_num(M4_pin);

    printf("Hello, world! \n");

    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");

    //icm20948_cal_gyro(&config, &data.gyro_bias[0]);
    //icm20948_cal_accel(&config, &data.accel_bias[0]);
    
    data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    //icm20948_cal_mag(&config, &data.mag_bias[0]);

    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
    printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);
    printf("calibrated mag: %d %d %d\n", data.mag_bias[0], data.mag_bias[1], data.mag_bias[2]);

    sleep_ms(500);
    int interval = 100000; 

    uint32_t ledCurr = time_us_32();
    uint32_t ledPrev = 0;
    bool ledState = false;

    double m1, m2, m3, m4;
    m1 = m2 = m3 = m4 = 0;

    int mmax = 200;
    int mmin = 125;

    // roll.Kp = 0.7;
    // roll.Kd = 1.5;
    // roll.Ki = 0;

    roll.Kp = 0.2;  
    roll.Kd = 0;
    roll.Ki = 0;

//     pitch.Kp = 1.068;
//     pitch.Kd = 0.006192;
//     pitch.Ki = 1.919;

    pitch.Kp = 0.2;
    pitch.Kd = 0;
    pitch.Ki = 0;

    // yaw.Kp = 0.7447;
    // yaw.Kd = 0.004319;   
    // yaw.Ki = 1.339;

    yaw.Kp = 0;
    yaw.Kd = 0;   
    yaw.Ki = 0;

    roll.r = pitch.r = 0.1;
    roll.alpha = pitch.alpha = 0.3;

    yaw.r = 0.001;

    float loop_speed, loop_time;
    uint32_t poll_start = 0;
    uint32_t end = 0;
    
    float time_now = 0;
    float time_former = 0;
    float deltat = 0;

    float roll_gcomp = 0;
    float rollphi = 0;
    float Kg = 1;
    float mmin_comp, gCompFactor; 
    float pitchRad, rollRad;

    while (true) {
        
        //loop poll timer
        //poll_start = time_us_32();

        icm20948_Read_All(&MPU6050,&config,&data);
    
        time_now = time_us_32();
        deltat = (float)(time_now - time_former) / 1000000.0f;
        time_former = time_now;

        MadgwickQuaternionUpdate(MPU6050.Ax * 9.8, MPU6050.Ay * 9.8, MPU6050.Az * 9.8,
                                MPU6050.Gx, MPU6050.Gy, MPU6050.Gz,
                                MPU6050.Mz, MPU6050.Mz, MPU6050.Mz, 
                                deltat, &filter);         
        
        ToEulerAngles(&filter, &MPU6050);

        roll.pv = MPU6050.KLMroll;
        pitch.pv = MPU6050.KLMpitch;        
        yaw.pv = MPU6050.yaw;

        PIDStruct(&roll);
        PIDStruct(&pitch);
        PIDStruct(&yaw);


        // rollphi = roll.pv*M_PI/180;
        // roll_gcomp = sinf(rollphi);

        rollRad  = roll.pv  * (M_PI / 180.0f);
        pitchRad = pitch.pv * (M_PI / 180.0f);
        gCompFactor = 1.0f / (cosf(rollRad) * cosf(pitchRad));

        mmin_comp = 120 * gCompFactor;

        // m1 = mmin - pitch.pulse - roll.pulse - yaw.pulse;   // Front left motor 
        // m3 = mmin - pitch.pulse + roll.pulse + yaw.pulse;   // Front right motor 
        // m2 = mmin + pitch.pulse - roll.pulse + yaw.pulse;   // Back left motor
        // m4 = mmin + pitch.pulse + roll.pulse - yaw.pulse;   // Back right motor
        
        m1 = mmin_comp - pitch.pulse - roll.pulse - yaw.pulse;   // Front left motor 
        m3 = mmin_comp - pitch.pulse + roll.pulse + yaw.pulse;   // Front right motor 
        m2 = mmin_comp + pitch.pulse - roll.pulse + yaw.pulse;   // Back left motor
        m4 = mmin_comp + pitch.pulse + roll.pulse - yaw.pulse;   // Back right motor


        if(m1 > mmax) { m1 = mmax; }
        if(m1 < mmin) { m1 = mmin; }
        
        if(m2 > mmax) { m2 = mmax; }
        if(m2 < mmin) { m2 = mmin; }

        if(m3 > mmax) { m3 = mmax; }
        if(m3 < mmin) { m3 = mmin; }

        if(m4 > mmax) { m4 = mmax; }
        if(m4 < mmin) { m4 = mmin; }

        pwm_set_chan_level(sliceM1,0,m1);
        pwm_set_chan_level(sliceM2,0,m2);
        pwm_set_chan_level(sliceM3,0,m3);
        pwm_set_chan_level(sliceM4,0,m4);

        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;
                led_on(18,ledState);
            
                // printf("\nAccel: X=%+2.5f Y=%+2.5f Z=%+2.5f| ", MPU6050.Ax,MPU6050.Ay,MPU6050.Az);
                // printf("\nGyro: X=%+2.5f Y=%+2.5f Z=%+2.5f | ", MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);

                printf("\n-----------------------------------------------\n");
                printf("Roll Pitch and Yaw \n");
                printf("Roll sp: %f Roll: %f    Pitch sp: %f Pitch: %f       ",roll.sp, MPU6050.KLMroll, pitch.sp, MPU6050.KLMpitch);
                // printf("Yaw sp: %f Yaw: %f\n", yaw.sp, MPU6050.yaw* 180.0 / PI);
                // printf("\nquaternions are %f, %f, %f, %f\n", filter.q[0],filter.q[1],filter.q[2],filter.q[3]);
                // printf("\n-----------------------------------------------\n");

                printf("\n      Roll:   P: %f           D: %f          I: %f",roll.P,roll.D,roll.I);
                // printf("\n      Pitch:  P: %f           D: %f          I: %f",pitch.P,pitch.D,pitch.I);
                // printf("\n      Yaw:    P: %f           D: %f          I: %f", yaw.P, yaw.D, yaw.I);
                printf("\nM1 (front left): %f       M2 (back left): %f       \nM3 (front right): %f       M4 (back right): %f",m1,m2,m3,m4);
                printf("\n-----------------------------------------------\n");

        }
        ledCurr = time_us_32();


// printf("\nM1: enter pulse width between 125 and 250 us");
// scanf("%d",&pwm);

// pwm_set_chan_level(sliceM1,0,pwm);

// printf("\nM2: enter pulse width between 125 and 250 us");
// scanf("%d",&pwm);

// pwm_set_chan_level(sliceM2,0,pwm);

// printf("\nM3: enter pulse width between 125 and 250 us");
// scanf("%d",&pwm);


// pwm_set_chan_level(sliceM3,0,pwm);

// printf("\nM4: enter pulse width between 125 and 250 us");
// scanf("%d",&pwm);
// pwm_set_chan_level(sliceM4,0,pwm);

// printf("motor identification complete !\n");

// ledState = !ledState;

// led_on(18,ledState);



}
        
}



//==========================================================================//
//manual setting of PWM
//disable uart input if using this

// printf("enter pulse width between 125 and 250 us");
// scanf("%d",&pwm);

// pwm_set_chan_level(sliceM1,0,pwm);
// pwm_set_chan_level(sliceM2,0,pwm);
// pwm_set_chan_level(sliceM3,0,pwm);
// pwm_set_chan_level(sliceM4,0,pwm);

// ledState = !ledState;

// led_on(18,ledState);

//==========================================================================//


// //measure Poll Rate
// end = time_us_32();
// loop_time = (float)(end-start)/1E6;

// loop_speed = 1/loop_time;
// //printf("\n%f Hz",loop_speed);