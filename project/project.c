#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/interp.h" //likely not needed
#include "hardware/timer.h"
#include "hardware/watchdog.h" //likely not needed
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h" //likely not needed
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/multicore.h" // potentially run sensor and flight control code different cores
#include "hardware/irq.h"


#include "initialise_functions.h"



int main() {
    stdio_init_all();

    sleep_ms(10000);
    i2c_initialisation(i2c0,400*1000,MPU6050_SDA,MPU6050_SCL);

    wifi_chip_initialisation();

    uart_initialisation(uart0, BAUD_RATE, UART_TX, UART_RX, DATA_BITS, STOP_BITS);

    printf("Hello, world! \n");
    uart_is_enabled(uart0);
    
    uint8_t buf[1];

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
        sleep_ms(250);
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        sleep_ms(500);

        
        //read uart 
        uart_read_blocking(uart0, buf, 10);
        
        
        printf("data stored in buf:  char: %c    int: %d   \n",buf[0],buf[0]);
        sleep_ms(1000);
    }
}



