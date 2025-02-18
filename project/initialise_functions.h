#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

//#define I2C_PORT i2c0
#define MPU6050_SDA 8
#define MPU6050_SCL 9

#define DPS310_SDA 12
#define DPS310_SCL 13

//#define Magnetomer_SDA 
//#define Magnetomer_SCL

#define UART_TX 16
#define UART_RX 17
#define BAUD_RATE 115200

#define M1_pin 10

void i2c_initialisation(i2c_inst_t *port,uint freq, int SDA_pin,int SCL_pin);

//likely not used gpio25 can be used for the LED
uint8_t wifi_chip_initialisation(void);

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data, int stop);  

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width,int ledPin);

void esc_calibration(int pwm_pin,uint chan,int ledPin, int arm_sleep);

void led_on(int ledPin, bool state);

void led_blink(int pin, int ticks);

void read_throttle(int pwm_pin, int* c, uint16_t* throttleVal);

void read_brake(int pwm_pin, int* c, uint16_t* brake);

void throttleResponse(int pwm_pin, int* t,int* b,uint16_t* throttle, uint16_t* brake);



    
uint8_t buffer[1];

//uint8_t index = 0;
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

uint32_t joined = 0;

uint16_t miscButtons = 0;
int32_t gyroX = 0;
int32_t gyroY = 0;
int32_t gyroZ = 0;
int32_t accelX = 0;
int32_t accelY = 0;
int32_t accelZ = 0;















//     for(int k=0; k<1000;k++){
//         int delay = i2c_start_measurement(ctx);
//         if (delay < 0) {
//                 printf("Failed to initiate measurement...\n");
//                 return -1;
//         }
//         // sleep_ms(5);
        
//         res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
//         if (res) {
//                 printf("Failed to read measurements...\n");
//                 return -1;
//         }
//         printf("Temperature: %.5f C, Pressure -101230: %.5f hPa\n", temp, pressure);
//         }

//         printf("Initialisation measurements done !\nNow starting average measurements ...");
//         sleep_ms(2000);

//         int total = 1000;
//     for(int k=0; k<total;k++){
//         int delay = i2c_start_measurement(ctx);
//         if (delay < 0) {
//                 printf("Failed to initiate measurement...\n");
//                 return -1;
//         }
//         //sleep_ms(5);
//         res = i2c_read_measurement(ctx, &temp, &pressure, &humidity);
//         if (res) {
//                 printf("Failed to read measurements...\n");
//                 return -1;
//         }
//         printf("Temperature: %.5f C, Pressure -101230: %.5f hPa\n", temp, pressure);
//         avgTemp +=temp;
//         avgPressure += pressure;
//         }
//         printf("\nAverage Temperature: %.5f C, Average Pressure Pressure: %.5f hPa\n", avgTemp, avgPressure);

//         avgPressure = avgPressure/total;
//         avgTemp = avgTemp/total;
//         printf("\nAverage Temperature: %.5f C, Average Pressure Pressure: %.5f hPa\n", avgTemp, avgPressure);
