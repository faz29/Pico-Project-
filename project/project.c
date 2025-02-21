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
    int throttleVal;
    
    stdio_init_all();
    
    sleep_ms(2000);
    i2c_initialisation(i2c0,400*1000,DPS310_SDA,DPS310_SCL);

    //wifi_chip_initialisation();

    uart_initialisation(uart0,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    pwm_initialisation(M1_pin,0,125,6);
    uint slice = pwm_gpio_to_slice_num(M1_pin);

    esc_calibration(M1_pin,0,6,500);

    printf("Hello, world! \n");


    void *ctx = NULL;
    int res = i2c_init_sensor(get_i2c_sensor_type("DPS310"), i2c0, 0x77, &ctx);
    if (res) {
        printf("Failed to initialize sensor...\n");
        return -1;
    }
    printf("Sensor initialized successfully!\n");

    float temp, pressure, humidity;
    float avgTemp, avgPressure, absPressure;
    avgTemp = avgPressure = 0;
        
    sleep_ms(2000);
    int LED = 0;
    int LED_ticks = 1000;

    int interval = 100000;

    long ledCurr = clock();
    long ledPrev = 0;
    bool ledState = false;

    while (true) {

        // if((ledCurr-ledPrev)>=interval){
        //         ledPrev = ledCurr;
        //         if (ledState ==false){
        //                 ledState = true;
        //         }
        //         if (ledState == true){
        //                 ledState = false;
        //         }
        //         led_on(6,ledState);
        // }

        res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
        if (res) {
                printf("Failed to read measurements...\n");
                return -1;
        }
        absPressure = (pressure - avgPressure);
        //fflush(stdout);

        int c,d;
        uint16_t throttleVal, brakeVal;
        // read_throttle(M1_pin,&c,&throttleVal);
        // read_brake(M1_pin,&d,&brakeVal);

        throttleResponse(M1_pin,&c,&d,&throttleVal, &brakeVal);

       printf("Temperature: %.5f C, Absolute Pressure: %.5f hPa, Throttle: %d, Brake: %d\n", temp,absPressure,c,d);



//==========================================================================//
//manual setting of PWM
//disable uart input if using this

//         printf("enter pulse width between 125 and 250 us");
//         scanf("%d",&pause);
        
//        pwm_set_chan_level(slice,0,pause);

//==========================================================================//





}
}



