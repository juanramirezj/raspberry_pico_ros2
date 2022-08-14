// Some hardware interrupt testing with Raspberry Pico
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// Debounce control
unsigned long time;
const int delayTime = 50; // Delay for every push button may vary


static char event_str[128];
void gpio_event_string(char *buf, uint32_t events);

void button_callback(uint gpio, uint32_t events)
{   
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) 
    {
        // Recommend to not to change the position of this line
        time = to_ms_since_boot(get_absolute_time());

        // Put the GPIO event(s) that just happened into event_str
        // so we can print it
        gpio_event_string(event_str, events);
        printf("GPIO %d %s\n\r", gpio, event_str); 
    }
} 

void button_callback2(uint gpio, uint32_t events)
{   
    printf("Interrupt 2 occured gpio=%i str=\n\r",gpio);     
} 

void main() 
{   
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    const uint inputSW = 3;
    const uint inputSW2 = 0;

    time = to_ms_since_boot(get_absolute_time());

    int led_status = true;
    int evt = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    // int evt = GPIO_IRQ_EDGE_FALL;
    
    stdio_init_all();

    // LED  
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Button
    gpio_init(inputSW);
    gpio_set_dir(inputSW, GPIO_IN);
    gpio_pull_up(inputSW);


    // Button 2
    gpio_init(inputSW2);
    gpio_set_dir(inputSW2, GPIO_IN);
    gpio_pull_up(inputSW2);

    gpio_set_irq_enabled_with_callback(inputSW, evt, true, &button_callback);   
    gpio_set_irq_enabled_with_callback(inputSW2, evt, true, &button_callback);   

    while(true)
    {
        printf("Print\n\r");
        if( led_status)
            gpio_put(LED_PIN, 1);
        else
            gpio_put(LED_PIN, 0);

        led_status = !led_status;
        sleep_ms(1000);   
    }
}

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}
