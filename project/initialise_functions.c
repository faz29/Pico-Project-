#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"



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
    if (cyw43_arch_init()==-1) {
        printf("Wi-Fi init failed\n");
        return -1;
    }   
    else{
        printf("Wifi chip initialised !\n");
    }
}

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data_bits, int stop_bits){    
    
    //set up the UART TX and RX pins
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart_port,tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart_port,rx_pin));    

    
    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, data_bits, stop_bits, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_port, false);

    uart_init(uart_port, uart_Brate);
    printf("UART initialised at %d baud rate with TX on pin %d and RX on pin %d !\n", uart_Brate, tx_pin, rx_pin);
}

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width){
    gpio_set_function(pwm_pin,GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pwm_pin);

    //clk div of 125 gives a frequency of 1MHz meaning a per tick width of 1us.
    pwm_set_clkdiv(slice, 125);
    pwm_set_wrap(slice, 1000);
    pwm_set_chan_level(slice,chan,pulse_width); //maybe moved to main loop
    pwm_set_enabled(slice,true);    
    printf("PWM initialised on pin %d !\n",pwm_pin);
}
void arm_sequence(int pwm_pin,uint chan,int ledPin, int arm_sleep){
    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    //arm sequence: zero throttle (125us), small delay, then full (250us), small delay then zero throttle.
    pwm_set_chan_level(slice,chan,125); 
    sleep_ms(arm_sleep); 
    pwm_set_chan_level(slice,chan,250);
    sleep_ms(arm_sleep);
    pwm_set_chan_level(slice,chan,125); 

    //turn on Led when arming is complete
    gpio_init(ledPin);
    gpio_set_dir(ledPin,GPIO_OUT);
    gpio_put(ledPin,1);
    printf("Arm sequence complete!\n");
}

    

