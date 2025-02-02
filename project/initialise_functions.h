#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

//#define I2C_PORT i2c0
#define MPU6050_SDA 8
#define MPU6050_SCL 9

//#define Magnetomer_SDA 
//#define Magnetomer_SCL

#define UART_TX 16
#define UART_RX 17
#define BAUD_RATE 115200


void i2c_initialisation(i2c_inst_t *port,uint freq, int SDA_pin,int SCL_pin);

//likely not used gpio25 can be used for the LED
uint8_t wifi_chip_initialisation(void);

void uart_initialisation(uart_inst_t *uart_port, int uart_Brate, int tx_pin, int rx_pin); 
