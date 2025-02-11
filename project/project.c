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
    int pause;
    int throttleVal;
    
    stdio_init_all();

    sleep_ms(5000);
    i2c_initialisation(i2c0,400*1000,MPU6050_SDA,MPU6050_SCL);

    //wifi_chip_initialisation();

    uart_initialisation(uart0,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    pwm_initialisation(M1_pin,0,0,6);
    uint slice = pwm_gpio_to_slice_num(M1_pin);

    //esc_calibration(M1_pin,0,6,500);

    printf("Hello, world! \n");
    

    uint8_t index = 0;
    uint8_t dpad[0];

    //uint16_t buttons[1];
    uint8_t buttons[1];

    int32_t axisX = 0;
    int32_t axisY = 0;
    int32_t axisRX = 0;
    int32_t axisRY = 0;
    int32_t brake = 0;


    uint8_t throttleArray[5];
    uint8_t* throttlePtr = throttleArray;
    uint8_t buffer[1];
    
    uint32_t joined = 0;

    uint16_t miscButtons = 0;
    int32_t gyroX = 0;
    int32_t gyroY = 0;
    int32_t gyroZ = 0;
    int32_t accelX = 0;
    int32_t accelY = 0;
    int32_t accelZ = 0;
  

    while (true) {
        //printf("\e[1;1H\e[2J");
        //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
        //sleep_ms(250);  
        //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        //sleep_ms(500);
        gpio_init(PICO_DEFAULT_LED_PIN_INVERTED);
        gpio_put(PICO_DEFAULT_LED_PIN_INVERTED, 1);

//==========================================================================//
//manual setting of PWM// 
/*
        printf("enter pulse width between 125 and 250 us");
        scanf("%d",&pause);
        
       pwm_set_chan_level(slice,0,pause);
*/
//==========================================================================//

            //working code to read 1 byte
        // uart_read_blocking(uart0,dpad,1);
        // //printf("%c%c%c \n",buf[0],buf[1],buf[2]);
        // //for (int i = 0; i<sizeof(buf);i++){
        // printf("%d",dpad[0]);
        //}

/*      uart_read_blocking(uart0,buttons,1);
        uart_read_blocking(uart0,buttons,1);

        printf("%d   %d",buttons[0],buttons[1]);

        printf("\n");
*/

    if (uart_is_readable(uart0) == true){


        while (true) {
            uart_read_blocking(uart0, buffer, 1);
            if (buffer[0] == 0xFF) {
                break;
            }

        }

         uart_read_blocking(uart0, throttleArray, 4);


        // for (int i=0; i<10; i++){
        //     if (buffer[i] == 0xFF){
        //         for (int j = 0; j < 5; j++) {
        //             uart_read_blocking(uart0, &throttleArray[j], 1);
        //         }
        //         break;
        //         }
        //     }
    //}
    memset(buffer, 0, sizeof(buffer));

    joined = 0;
    for (int j = 0; j<4; j++){
        //printf("%d ",throttleArray[j]);
        joined += (throttleArray[j]<<(8*(3-j))); 
        }

    printf("\n");
    
    printf("\nthrottle: %d\n\n",joined);
    
    throttleVal = 125 + (125*joined)/1020;

    pwm_set_chan_level(slice,0,throttleVal);


    }
    





}
}



