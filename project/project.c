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

#include "initialise_functions.h"


int main() {
    stdio_init_all();

    sleep_ms(10000);
    i2c_initialisation(i2c0,400*1000,MPU6050_SDA,MPU6050_SCL);

    wifi_chip_initialisation();
    uart_initialisation(uart0,BAUD_RATE, UART_TX, UART_RX,3, 1);
    
    //uart_initialisation(uart0,BAUD_RATE,UART_TX,UART_RX);

    printf("Hello, world! \n");
    
    uint8_t BT_BUFFER[0];


    int i = 0;
    uint8_t buf[100];

    while (true) {
       
       
        //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
        //sleep_ms(250);
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        //sleep_ms(500);

        // printf("enter pulse width between 125 and 250 us");
        // scanf("%d",&pause);
        
       // pwm_set_chan_level(slice,0,pause)        
            
        uart_read_blocking(uart0,buf,100);
        //printf("%c%c%c \n",buf[0],buf[1],buf[2]);
        for (int i = 0; i<100;i++){
            printf("%c",buf[i]);
        }
        printf("\n");


        


        
     //   sleep_ms(150);
    }    
}



