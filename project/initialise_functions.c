#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"

void i2c_initialisation(i2c_inst_t *port,uint freq, int SDA_pin,int SCL_pin){
// I2C Initialisation. Using it at 400Khz.
    i2c_init(port, freq);
    gpio_set_function(SDA_pin, GPIO_FUNC_I2C);
    gpio_set_function(SCL_pin, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_pin);
    gpio_pull_up(SCL_pin);

int P;
    if (port==i2c0){
       P = 0;
    }
    if (port==i2c1){
       P = 1;
    }

    printf("I2C Initialised at %dkHz on port %d with SDA on pin %d and SCL on pin %d !!\n", freq/1000, P,SDA_pin,SCL_pin);

}

uint8_t wifi_chip_initialisation(void){
        // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }   
    else{
        printf("Wifi chip initialised !!\n");
    }
}

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data, int stop){
    //set up the UART TX and RX pins
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart_port,tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart_port,rx_pin));    

    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, data, stop, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_port, true);

    uart_init(uart_port, uart_Brate);
    printf("UART initialised at %d baud rate with TX on pin %d and RX on pin %d !!\n", uart_Brate, tx_pin, rx_pin);


}