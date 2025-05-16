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
madgwick_ahrs_t filter = {0.005,0.001,{1.0f, 0.0f, 0.0f, 0.0f}};
Kalman_2t kf;

 
int main() {
    int pwm;
    float AngX, AngY;   
    MPU6050_t MPU6050;

    //initialise all structures to 0
    controller_vars cdata = {0};    
    pid_vars yaw = {0};
    pid_vars roll = {0};
    pid_vars pitch = {0};     

    //all protocall initialisations
    stdio_init_all();
    i2c_initialisation(I2C_PORT,400*1000);
    uart_initialisation(UART_PORT,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    sleep_ms(100);
    // pwm_initialisation(M1_pin,0,125);
    // pwm_initialisation(M2_pin,0,125);
    // pwm_initialisation(M3_pin,0,125);
    // pwm_initialisation(M4_pin,0,125);

    //motor calibration
    pwm_initialisation(M1_pin,0,250);
    pwm_initialisation(M2_pin,0,250);
    pwm_initialisation(M3_pin,0,250);
    pwm_initialisation(M4_pin,0,250);
    sleep_ms(6000);

    pwm_initialisation(M1_pin,0,125);
    pwm_initialisation(M2_pin,0,125);
    pwm_initialisation(M3_pin,0,125);
    pwm_initialisation(M4_pin,0,125);
    sleep_ms(9000);

    sleep_ms(2000);

    uint sliceM1 = pwm_gpio_to_slice_num(M1_pin);
    uint sliceM2 = pwm_gpio_to_slice_num(M2_pin);
    uint sliceM3 = pwm_gpio_to_slice_num(M3_pin);
    uint sliceM4 = pwm_gpio_to_slice_num(M4_pin);

    printf("Attempting to communicate with ICM20948 \n");

    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");

    //sensor biases set manually
    data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    data.accel_bias[0] = -561; data.accel_bias[1] = 282; data.accel_bias[2] = 902;
    data.gyro_bias[0] = 1; data.gyro_bias[1] = 62; data.gyro_bias[2] = 72;
    
    float mag_soft[3][3] = {
    { 1.246254f,  -0.036161f,  0.213955f },   // X row
    { -0.036161f,  1.306548f,  0.074080f },   // Y row
    { 0.213955f,   0.074080f,  0.998964f }    // Z row

    //calibration functions
    //icm20948_cal_gyro(&config, &data.gyro_bias[0]);
    //icm20948_cal_accel(&config, &data.accel_bias[0]);
    //icm20948_cal_mag_simple(&config, &data.mag_bias[0]);
    

};
    
    //printing biases
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

    int mmax = 220;
    int mmin = 130;
    float thrust = 0.0;

    //PID coefficients
    roll.Kp = 3.6;
    roll.Kd = 1.3;
    roll.Ki = 0;

    pitch.Kp = 3.8;
    pitch.Kd = 1.5;
    pitch.Ki = 0;

    yaw.Kp = 0; 
    yaw.Kd = 0; 
    yaw.Ki = 0; 

    roll.r = 1;
    pitch.r = 1;
    yaw.r = 0.95;

    //initialise kalman for Yaw
    float initial_heading = 0.0f; 
    float initial_rate    = 0.0f;   
    KalmanInit(&kf, 0.00007f, initial_heading, initial_rate);

    float loop_speed, loop_time;
    uint32_t poll_start = 0;
    uint32_t end = 0;
    
    float time_now = 0;
    float time_former = 0;
    float deltat = 0;

    float smoothYaw = 0.0f;
    const float alpha = 0.6f;  

    while (true) {
        
        //Read controller and IMU
        read_controller_data_structure(UART_PORT,&cdata);
        icm20948_Read_All(&MPU6050,&config,&data,mag_soft);
        
        //controller garbage value handling
        if(cdata.brake>1030){cdata.brake = 0;}
        if(cdata.throttle>1030){cdata.throttle = 0;}
        if(cdata.lx<-530){cdata.lx = 0;}
        if(cdata.lx>530){cdata.lx = 0;}
        if(cdata.rx<-530){cdata.rx = 0;}
        if(cdata.ry>530){cdata.ry = 0;}

        //controller deadzone to stop drift
        if(cdata.lx<50 && cdata.lx > -50){cdata.lx = 0;}
        if(cdata.ly<50 && cdata.ly > -50){cdata.ly = 0;}
        if(cdata.rx<50 && cdata.rx > -50){cdata.rx = 0;}
  
        //controller variables to control motion
        float throttle = 0.1 *(float)cdata.throttle/1024; 
        float brake =  0.1 *(float)cdata.brake/1024; 
        float set_roll = 0.15 *(float)cdata.lx/1024;
        float set_pitch = 0.15 *(float)cdata.ly/1024;;
        float cont_yaw = 120 * (float)cdata.rx/1024;

        //handling out of bound values
        if(throttle>1){throttle = 0;}
        if(brake>1){brake = 0;}

        roll.sp = roll.sp + set_roll;
        if (roll.sp >45){roll.sp = 45;}
        if(roll.sp <-45){roll.sp = -45;}

        pitch.sp = pitch.sp - set_pitch;
        if(pitch.sp >45){pitch.sp = 45;}
        if(pitch.sp <-45){pitch.sp = -45;}

        thrust = thrust + throttle - brake;
        if (thrust > 220){thrust = 220;}
        if (thrust < 130){thrust = 130;}

        //time measurement and calculation
        time_now = time_us_32();
        deltat = (float)(time_now - time_former) / 1000000.0f;
        time_former = time_now;

        //Yaw 2x2 Kalman filter
        kf.A[0][1] = deltat;           // update A’s dt term
        KalmanPredict(&kf);
        float magYaw = atan2(MPU6050.My,MPU6050.Mx); /* tilt-compensated heading in degrees */
        KalmanUpdate(&kf, magYaw, MPU6050.Gz);

        //set calculated yaw
        MPU6050.yaw = alpha * MPU6050.yaw + (1.0f - alpha) * kf.X[0];

        //setting yaw to between 180 and -180
        if      (MPU6050.yaw >  180.0f) MPU6050.yaw -= 360.0f;
        else if (MPU6050.yaw < -180.0f) MPU6050.yaw += 360.0f;

        //setting roll pitch and yaw for use in the PID loops
        roll.pv = MPU6050.KLMroll;
        pitch.pv = MPU6050.KLMpitch;        
        yaw.pv = MPU6050.yaw* 180.0 / PI;

        //using gyroscope for rate of change of axis
        roll.gyropv = -MPU6050.Gx;  
        pitch.gyropv = -MPU6050.Gy;
        yaw.gyropv = MPU6050.Gz;

        //call PID function to get values
        PIDStruct(&roll);
        PIDStruct(&pitch);
        PIDStruct(&yaw);

        //set motor values
        m1 = thrust - pitch.pulse - roll.pulse - yaw.pulse - cont_yaw;   // Front left motor 
        m3 = thrust - pitch.pulse + roll.pulse + yaw.pulse + cont_yaw;   // Front right motor 
        m2 = thrust + pitch.pulse - roll.pulse + yaw.pulse + cont_yaw;   // Back left motor
        m4 = thrust + pitch.pulse + roll.pulse - yaw.pulse - cont_yaw;   // Back right motor

        //handle out of bound motor values
        if(m1 > mmax) { m1 = mmax; }
        if(m1 < mmin) { m1 = mmin; }
        
        if(m2 > mmax) { m2 = mmax; }
        if(m2 < mmin) { m2 = mmin; }

        if(m3 > mmax) { m3 = mmax; }
        if(m3 < mmin) { m3 = mmin; }

        if(m4 > mmax) { m4 = mmax; }
        if(m4 < mmin) { m4 = mmin; }

        //send pulse width modlulation values to each motor
        pwm_set_chan_level(sliceM1,0,m1);
        pwm_set_chan_level(sliceM2,0,m2);
        pwm_set_chan_level(sliceM3,0,m3);
        pwm_set_chan_level(sliceM4,0,m4);


        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;
                led_on(18,ledState);

                //Accel Gyro and Mag values

                // printf("\nAccel: X=%+2.5f Y=%+2.5f Z=%+2.5f| ", MPU6050.Ax,MPU6050.Ay,MPU6050.Az);
                // printf("\nGyro: X=%+2.5f Y=%+2.5f Z=%+2.5f | ", MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);
                // printf("\nMag: X=%+2.5f Y=%+2.5f Z=%+2.5f | \n", MPU6050.Mx,MPU6050.My,MPU6050.Mz);

                //yaw pitch and roll values

                // printf("\n-----------------------------------------------\n");
                // printf("Roll Pitch and Yaw \n");
                // printf("Roll sp: %f Roll: %f    Pitch sp: %f Pitch: %f       ",roll.sp, MPU6050.KLMroll, pitch.sp, MPU6050.KLMpitch);
                // printf("Yaw sp: %f Yaw: %f\n", yaw.sp, MPU6050.yaw* 180.0 / PI);
                // printf("\n-----------------------------------------------\n");


                //print PID Values

                //printf("\n      Roll:   P: %f           D: %f          I: %f",roll.P,roll.D,roll.I);
                //printf("\n      Pitch:  P: %f           D: %f          I: %f",pitch.P,pitch.D,pitch.I);
                // printf("\n      Yaw:    P: %f           D: %f          I: %f", yaw.P, yaw.D, yaw.I);
                // printf("\nM1 (front left): %f       M2 (back left): %f       \nM3 (front right): %f       M4 (back right): %f",m1,m2,m3,m4);
                // printf("\n-----------------------------------------------\n");


                // printf("\nController values\n");

                // printf("Throttle: %f\n", throttle);
                // printf("Brake: %d\n", cdata.brake);
                // printf("Roll: %d\n", cdata.lx);
                // printf("Pitch: %d\n", cdata.ly);
                // printf("yaw from controller: %d\n", cdata.rx);
                // printf("yaw control: %f\n", cont_yaw);
                // printf("\nThrust: %f\n ",thrust);
                
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