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
    uint slice = pwm_gpio_to_slice_num(M1_pin);

    esc_calibration(M1_pin,0,6,500);

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

    float Error, setp, pvar, Kp,Ki,Kd,pulse,currAngle,addAngle;
    float angle, throttle;

    angle = setp = addAngle = Error = throttle = 0;

    int UP,DOWN,maxstep;
    maxstep = 3;
    UP = DOWN = 0;
    uint16_t throttleVal, brakeVal;
    
    Kp= 3;
    Ki = 0.003;
    Kd= 1.5;

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

        
        pvar = AngY/1.00000000;
        angle = ((DOWN*1.00000-UP*1.0000)*0.005);
        addAngle = (angle/1020);
        setp +=  addAngle;


        if(setp>100 || setp<-100){
            setp = 0;}
        if (setp>50 && setp<100){ 
            setp =50;}
        if(setp<-50 && setp>-100){
            setp = -50;}

        //Error = (setp - currAngle);


        PID(&Error,&setp,&pvar,Kp,Ki,Kd,&maxstep,&pulse);

        if(pulse>125){
            pulse = 125;
        }
        if(pulse<10){
            pulse =10;
        }
        
        throttle = 125 +pulse;  

        pwm_set_chan_level(slice,0,throttle);

        if((ledCurr-ledPrev)>=interval){
                ledPrev = ledCurr;
                ledState = !ledState;

                led_on(6,ledState);
                printf("Temp: %.5f C, Pressure: %.5f hPa, Throttle: %d, Brake: %d, X: %f, Y: %f,\n", temp,absPressure,UP,DOWN,AngX,AngY);
                printf("\nSetpoint: %f\n",setp); 
                printf("\nPulse: %f\n",throttle);   
                printf("\nError: %f\n",Error);
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



