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

    sleep_ms(5000);
    i2c_initialisation(i2c0,400*1000,MPU6050_SDA,MPU6050_SCL);

    //wifi_chip_initialisation();

<<<<<<< HEAD
    uart_initialisation(uart0,BAUD_RATE, UART_TX, UART_RX,8, 1);
    
    pwm_initialisation(10,0,0);

    arm_sequence(10,0,6,500);
=======
    uart_initialisation(uart0, BAUD_RATE, UART_TX, UART_RX, DATA_BITS, STOP_BITS);
>>>>>>> 6ded87106021ef35ab460688d599c3402e2b9255

    printf("Hello, world! \n");
    uart_is_enabled(uart0);
    

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
    
    uint32_t joined = 0;

    uint16_t miscButtons = 0;
    int32_t gyroX = 0;
    int32_t gyroY = 0;
    int32_t gyroZ = 0;
    int32_t accelX = 0;
    int32_t accelY = 0;
    int32_t accelZ = 0;
    



    while (true) {
        //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
        //sleep_ms(250);  
        //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        //sleep_ms(500);
        gpio_init(PICO_DEFAULT_LED_PIN_INVERTED);
        gpio_put(PICO_DEFAULT_LED_PIN_INVERTED, 1);

        // printf("enter pulse width between 125 and 250 us");
        // scanf("%d",&pause);
        
<<<<<<< HEAD
       // pwm_set_chan_level(slice,0,pause)        

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
    uart_read_blocking(uart0,throttleArray,5);

    //if (tempThrottle[0] == 0xFF){   //this will test the first bit incase picked up in the middle of transmission

        //art_read_blocking(uart0,(tempPtr),4);

        // *throttlePtr = (throttleArray[1]>>0)&0xFF;
        //   printf("%d ",*throttlePtr);
  
        //  *(throttlePtr+1) = (throttleArray[2]>>8)&0xFF;
        //   printf("%d ", *(throttlePtr+1));
      
        // *(throttlePtr+2) = (throttleArray[3] >>16)&0xFF;
        //   printf("%d ",*(throttlePtr + 2));
      
        //  *(throttlePtr+3) = (throttleArray[4]>>24);
        //   printf("%d ",*(throttlePtr+3));
      
          for (int j = 1; j<5; j++){
            joined = joined + (*(throttlePtr+j)<<(8*j)); 
                }
            
          printf("\n\n%d",joined);

      // }
      // else
      //   {
      //   break;
      //   }
          // uart_read_blocking(uart0,(throttlePtr+2),1);
          // uart_read_blocking(uart0,(throttlePtr+3),1);
          // uart_read_blocking(uart0,(throttlePtr+4),1);

=======
        //read uart 
        uart_read_blocking(uart0, buf, 10);
        
        
        printf("data stored in buf:  char: %c    int: %d   \n",buf[0],buf[0]);
        sleep_ms(1000);
>>>>>>> 6ded87106021ef35ab460688d599c3402e2b9255
    }





    sleep_ms(500);
  }
}



