#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h" //likely not needed
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/multicore.h" // potentially run sensors and flight control code different cores
#include "hardware/irq.h"   //potential use interrupts for certain actions e.g. landing 
#include "hardware/pwm.h"
#include <time.h>
#include <math.h>

#include "pico_sensor_lib.h"
#include "initialise_functions.h"


int main() {
    int pwm;
    float AngX, AngY;
    MPU6050_t MPU6050;

    pid_vars pidM1;
    pid_vars pidM2;
    
    stdio_init_all();
    
    sleep_ms(150);
    i2c_initialisation(I2C_PORT,400*1000);

    uart_initialisation(UART_PORT,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    pwm_initialisation(M1_pin,0,125,6);
    pwm_initialisation(M2_pin,0,125,6);

    uint sliceM1 = pwm_gpio_to_slice_num(M1_pin);
    uint sliceM2 = pwm_gpio_to_slice_num(M2_pin);

    //esc_calibration(M1_pin,0,6,500);

    printf("Hello, world! \n");

    MPU6050_Init(I2C_PORT);

    printf("MPU6050 initialised !\n");

    float temp, pressure, humidity;
    float avgTemp, avgPressure, absPressure;
    avgTemp = avgPressure = 0;
        
    sleep_ms(150);


    int interval = 100000; 

    uint32_t ledCurr = time_us_32();
    uint32_t ledPrev = 0;
    bool ledState = false;

    // float ErrorM1, ErrorM2, setp, pvar, Kp,Ki,Kd,pulseM1,pulseM2,currAngle,addAngle, angle, throttleM1, throttleM2, baselineM1, baselineM2;
    double ErrorM1, ErrorM2, setp, pvar, Kp,Ki,Kd,pulseM1,pulseM2,currAngle,addAngle, angle, throttleM1, throttleM2, baselineM1, baselineM2;

    angle = setp = addAngle = ErrorM1 = ErrorM2 = throttleM1 = throttleM2 = baselineM1 = baselineM2 = pulseM1 = pulseM2 = 0;

    int UP,DOWN,maxstep;
    maxstep = 10;
    UP = DOWN = 0;
    uint16_t throttleVal, brakeVal;
    
    Kp = 0.11;
    Ki = 0.64;
    Kd = 0.25;

    while (true) {

        absPressure = (pressure - avgPressure);
        MPU6050_Read_All(I2C_PORT,&MPU6050);
            AngX = MPU6050.KalmanAngleX;
            AngY = MPU6050.KalmanAngleY;

        read_controller(UART_PORT,M1_pin,&UP,&DOWN,&throttleVal,&brakeVal);

        pvar = (double)AngX;
        angle = (((double)DOWN-(double)UP)*0.005);
        addAngle = (angle/1020);
        setp +=  addAngle;

        ErrorM1 = pvar - setp;
        ErrorM2 = setp - pvar;

        baselineM1 = sin((pvar*2*3.14)/360)*50;
        baselineM2 = (-1)*sin((pvar*2*3.14)/360)*75;

        if(AngX<0){baselineM1 = 0;}           
        if(AngX>0){baselineM2 = 0;}

        PID(&ErrorM1,Kp,Ki,Kd,&pulseM1,&pulseM2,&baselineM1,&baselineM2);
    
        throttleM1 = 140 + pulseM1;     //new pid function
        throttleM2 = 140 + pulseM2;

        if(throttleM1>190){throttleM1 = 190;}
        if(throttleM2>190){throttleM2 = 190;}

        pwm_set_chan_level(sliceM1,0,throttleM1);
        pwm_set_chan_level(sliceM2,0,throttleM2);

        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;

                led_on(6,ledState);
                printf("\nTemp: %.5f C, Pressure: %.5f hPa, Throttle: %d, Brake: %d, X: %f, Y: %f,\n", temp,absPressure,UP,DOWN,AngX,AngY);
                printf("\nSetpoint: %f",setp); 
                printf("\nThrottle M1: %f",throttleM1);   
                printf("\nError M1: %f",ErrorM1);
                printf("\nThrottle M2: %f",throttleM2);   
                printf("\nError M2: %f",ErrorM2);
                printf("\nCurrent Angle: %f",pvar);
                printf("\nPulse M1: %f",pulseM1);
                printf("\nPulse M2: %f",pulseM2);

        }
        ledCurr = time_us_32();

}
}


//==========================================================================//
//manual setting of PWM
//disable uart input if using this

    //     printf("enter pulse width between 125 and 250 us");
    //     scanf("%d",&pwm);
        
    //    pwm_set_chan_level(sliceM1,0,pwm);

//==========================================================================//

