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

    // pwm_initialisation(M1_pin,0,250);
    // pwm_initialisation(M2_pin,0,250);
    // pwm_initialisation(M3_pin,0,250);
    // pwm_initialisation(M4_pin,0,250);

    sleep_ms(2000);

    uint sliceM1 = pwm_gpio_to_slice_num(M1_pin);
    uint sliceM2 = pwm_gpio_to_slice_num(M2_pin);
    uint sliceM3 = pwm_gpio_to_slice_num(M3_pin);
    uint sliceM4 = pwm_gpio_to_slice_num(M4_pin);

    printf("Hello, world! \n");

    MPU6050_Init(I2C_PORT);

    printf("MPU6050 initialised !\n");
        
    sleep_ms(150);
 
    int interval = 100000; 

    uint32_t ledCurr = time_us_32();
    uint32_t ledPrev = 0;
    bool ledState = false;

    double m1, m2, m3, m4;
    m1 = m2 = m3 = m4 = 0;

//     Kp = 0.11;
//     Ki = 0.64;
//     Kd = 0.25;
    
    int mmax = 200;
    int mmin = 130;

    roll.Kp = 0.0;
    roll.Kd = 0.0;
    roll.Ki = 0.0;

    pitch.Kp = 0.12;
    pitch.Kd = 0.3;
    pitch.Ki = 0.0;

    roll.r = pitch.r = yaw.r = 0.01;

    yaw.Kp = 0.0;
    yaw.Kd = 0.0;   
    yaw.Ki = 0.0;

    float loop_speed, loop_time;
    uint32_t start = 0;
    uint32_t end = 0;

    float phi, gcomp, Kg;
    Kg = 53;

    while (true) {
        
        //loop poll timer
        start = time_us_32();

        MPU6050_Read_All(I2C_PORT,&MPU6050);

        roll.pv = MPU6050.KalmanAngleX;
        pitch.pv = MPU6050.KalmanAngleY; 
        yaw.pv = MPU6050.Gz;

        PIDStruct(&roll);
        PIDStruct(&pitch);
        PIDStruct(&yaw);
        
        phi = pitch.pv * (3.14/180.0f);      // convert to radians
        gcomp = Kg * sinf(phi); //gravity compensation term

        m1 = mmin - pitch.pulse + roll.pulse + yaw.pulse;   // Front right motor 
        m2 = mmin - pitch.pulse - roll.pulse - yaw.pulse;   // Front left motor 
        m3 = mmin + pitch.pulse + roll.pulse - yaw.pulse;   // Back right motor
        m4 = mmin + pitch.pulse - roll.pulse + yaw.pulse;   // Back left motor

        m1 = mmin - pitch.pulse + roll.pulse + yaw.pulse + gcomp;
        m2 = mmin - pitch.pulse - roll.pulse - yaw.pulse + gcomp;
        m3 = mmin + pitch.pulse + roll.pulse - yaw.pulse - gcomp;
        m4 = mmin + pitch.pulse - roll.pulse + yaw.pulse - gcomp;

        m1 = mmin + gcomp;
        m2 = mmin + gcomp;
        m3 = mmin - gcomp;
        m4 = mmin - gcomp;
        
        if(m1 > mmax) { m1 = mmax; }
        if(m1 < mmin) { m1 = mmin; }
        
        if(m2 > mmax) { m2 = mmax; }
        if(m2 < mmin) { m2 = mmin; }

        if(m3 > mmax) { m3 = mmax; }
        if(m3 < mmin) { m3 = mmin; }

        if(m4 > mmax) { m4 = mmax; }
        if(m4 < mmin) { m4 = mmin; }


//comment this out if using manual speeds
        pwm_set_chan_level(sliceM1,0,m1);
        pwm_set_chan_level(sliceM2,0,m2);
        pwm_set_chan_level(sliceM3,0,m3);
        pwm_set_chan_level(sliceM4,0,m4);

        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;
                led_on(18,ledState);

                printf("\n-----------------------------------------------\n");
                printf("Roll sp: %f Roll: %f    Pitch sp: %f Pitch: %f       ",roll.sp, roll.pv, pitch.sp, pitch.pv);
                printf("Yaw sp: %f Yaw Rate: %f\n", yaw.sp, yaw.pv);
                printf("\n              Roll:\nP: %f         D: %f          I: %f\n",roll.P,roll.D,roll.I);
                printf("\n              Pitch:\nP: %f        D: %f          I: %f\n",pitch.P,pitch.D,pitch.I);
                printf("\n              Yaw:\nP: %f          D: %f          I: %f\n", yaw.P, yaw.D, yaw.I);
                printf("\nM1 (front right): %f       M2 (front left): %f       \n\nM3 (backright): %f       M4 (backleft): %f",m1,m2,m3,m4);
                printf("\n-----------------------------------------------\n");

        }
        ledCurr = time_us_32();

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


//==========================================================================//

        // printf("Pitch Pv: %f\n",pitch.pv);
        // printf("\nenter Kg Value");
        // scanf("%f",&Kg);/