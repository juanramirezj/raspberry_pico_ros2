/*****
 * Motor Driver using DRV8833 Motor Driver Circuit
 *****/

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

int main() {
    const uint AIN1_PIN = 3;
    const uint AIN2_PIN = 2;
    const uint BIN1_PIN = 4;
    const uint BIN2_PIN = 5;

    const uint STBY_PIN = 6;

    // Initialize GPIO function and set Pin as output
    gpio_init(AIN1_PIN);
    gpio_set_dir(AIN1_PIN, GPIO_OUT);
        
    gpio_init(AIN2_PIN);
    gpio_set_dir(AIN2_PIN, GPIO_OUT);
    
    gpio_init(BIN1_PIN);
    gpio_set_dir(BIN1_PIN, GPIO_OUT);
    
    gpio_init(BIN2_PIN);
    gpio_set_dir(BIN2_PIN, GPIO_OUT);
    
    gpio_init(STBY_PIN);
    gpio_set_dir(STBY_PIN, GPIO_OUT);


    gpio_put(STBY_PIN, 1);

    while(true)
    {

        gpio_put(AIN1_PIN, 1);
        gpio_put(AIN2_PIN, 0);
        sleep_ms(1000);

        gpio_put(AIN1_PIN, 0);
        gpio_put(AIN2_PIN, 1);
        sleep_ms(1000);

        gpio_put(AIN1_PIN, 0);
        gpio_put(AIN2_PIN, 0);
        sleep_ms(2000);
        


        gpio_put(BIN1_PIN, 1);
        gpio_put(BIN2_PIN, 0);
        sleep_ms(1000);

        gpio_put(BIN1_PIN, 0);
        gpio_put(BIN2_PIN, 1);
        sleep_ms(1000);

        gpio_put(BIN1_PIN, 0);
        gpio_put(BIN2_PIN, 0);
        sleep_ms(2000);

    }

}