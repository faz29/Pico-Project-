#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/interp.h" //likely not needed
#include "hardware/timer.h"
#include "hardware/watchdog.h" //likely not needed
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h" //likely not needed -- only used for controlling LED
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/multicore.h" // potentially run sensors and flight control code different cores
#include "hardware/irq.h"   //potential use interrupts for certain actions e.g. landing 
#include "hardware/pwm.h"
#include <time.h>

#include "pico_sensor_lib.h"
#include "initialise_functions.h"


int main() {
    int pause;
    // int throttleVal;
    float AngX, AngY;
    MPU6050_t MPU6050;
    
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

    // void *ctx = NULL;
    // int res = i2c_init_sensor(get_i2c_sensor_type("DPS310"), I2C_PORT, 0x77, &ctx);
    // if (res) {
    //     printf("DPS310 Failed to initialize ...\n");
    //     return -1;
    // }
    printf("DPS310 initialized successfully!\n");

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

    float ErrorM1, ErrorM2, setp, pvar, Kp,Ki,Kd,pulseM1,pulseM2,currAngle,addAngle;
    float angle, throttleM1, throttleM2;

    angle = setp = addAngle = ErrorM1 = ErrorM2 = throttleM1 = throttleM2 = 0;

    int UP,DOWN,maxstep;
    maxstep = 3;
    UP = DOWN = 0;
    uint16_t throttleVal, brakeVal;
    
    Kp = 0.002;
    // Ki = 0.0028797695;
    // Kd = 10.5;

    while (true) {

        // res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
        // if (res) {
        //         printf("Failed to read measurements...\n");
        //         return -1;
        // }
        absPressure = (pressure - avgPressure);
        //fflush(stdout);
        MPU6050_Read_All(I2C_PORT,&MPU6050);
            AngX = MPU6050.KalmanAngleX;
            AngY = MPU6050.KalmanAngleY;

        read_controller(UART_PORT,M1_pin,&UP,&DOWN,&throttleVal,&brakeVal);

        pvar = AngX/1.00000000;
        angle = ((DOWN*1.00000-UP*1.0000)*0.005);
        addAngle = (angle/1020);
        setp +=  addAngle;

        // if(setp>100 || setp<-100){
        //     setp = 0;}
        // if (setp>50 && setp<100){ 
        //     setp =50;}
        // if(setp<-50 && setp>-100){
        //     setp = -50;}

        ErrorM1 = pvar-setp;
        ErrorM2 = setp - pvar;

        PID(&ErrorM1,&setp,&pvar,Kp,Ki,Kd,&maxstep,&pulseM1);
        PID(&ErrorM2,&setp,&pvar,Kp,Ki,Kd,&maxstep,&pulseM2);

        if(pulseM1>35){
            pulseM1 = 35;
        }
        if(pulseM1<10){
            pulseM1 =10;
        }
        
        throttleM1 = 125 + pulseM1;  

        if(pulseM2>125){
            pulseM2 = 125;
        }
        if(pulseM2<10){
            pulseM2 =10;
        }
        
        throttleM2 = 125 + pulseM2;  

        pwm_set_chan_level(sliceM1,0,throttleM1);
        pwm_set_chan_level(sliceM2  ,0,throttleM2);


        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;

                led_on(6,ledState);
                printf("Temp: %.5f C, Pressure: %.5f hPa, Throttle: %d, Brake: %d, X: %f, Y: %f,\n", temp,absPressure,UP,DOWN,AngX,AngY);
                printf("\nSetpoint: %f\n",setp); 
                printf("\nPulse M1: %f\n",throttleM1);   
                printf("\nError M1: %f\n",ErrorM1);
                printf("\nPulse M2: %f\n",throttleM2);   
                printf("\nError M2: %f\n",ErrorM2);
                printf("\ncurrent Angle: %f\n",pvar);


        }
        ledCurr = time_us_32();


//==========================================================================//
//manual setting of PWM
//disable uart input if using this

//         printf("enter pulse width between 125 and 250 us");
//         scanf("%d",&pause);
        
//        pwm_set_chan_level(slice,0,pause);

//==========================================================================//





}
}



