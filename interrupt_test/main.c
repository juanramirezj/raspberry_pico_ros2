// Some hardware interrupt testing with Raspberry Pico
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// Debounce control
unsigned long time;
const int delayTime = 50; // Delay for every push button may vary

const uint32_t period_timer = 500; //ms


static char event_str[128];
void gpio_event_string(char *buf, uint32_t events);
const uint wheelL_pin = 3;
const uint wheelR_pin = 0;
uint wheelL_ticks = 0;
uint wheelR_ticks = 0;
uint wheelL_hz;
uint wheelR_hz;

bool repeating_timer_callback(struct repeating_timer *t)
{
    wheelL_hz = wheelL_ticks * 1000 / period_timer;
    wheelR_hz = wheelR_ticks * 1000 / period_timer;
    wheelL_ticks = 0;
    wheelR_ticks = 0;
    
    return true;
}


void button_callback(uint gpio, uint32_t events)
{   
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) 
    {
        // Recommend to not to change the position of this line
        time = to_ms_since_boot(get_absolute_time());

        // Put the GPIO event(s) that just happened into event_str
        // so we can print it
        gpio_event_string(event_str, events);
        // printf("GPIO %d %s\n\r", gpio, event_str); 
        if( gpio == wheelL_pin) wheelL_ticks++;
        if( gpio == wheelR_pin) wheelR_ticks++;
    }
} 


void main() 
{   
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    const uint inputSW = 3;
    const uint inputSW2 = 0;   

    time = to_ms_since_boot(get_absolute_time());

    // Create a repeating timer that calls repeating_timer callback.
    // It makes the rpm calculation
    struct repeating_timer timer;
    add_repeating_timer_ms(period_timer, repeating_timer_callback, NULL, &timer);

    int led_status = true;
    int evt = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    // int evt = GPIO_IRQ_EDGE_FALL;
    
    stdio_init_all();

    // LED  
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // wheelL
    gpio_init(wheelL_pin);
    gpio_set_dir(wheelL_pin, GPIO_IN);
    gpio_pull_up(wheelL_pin);


    // wheelR
    gpio_init(wheelR_pin);
    gpio_set_dir(wheelR_pin, GPIO_IN);
    gpio_pull_up(wheelR_pin);

    gpio_set_irq_enabled_with_callback(wheelL_pin, evt, true, &button_callback);   
    gpio_set_irq_enabled_with_callback(wheelR_pin, evt, true, &button_callback);   

    while(true)
    {
        printf("Print L hz=%u  R hz=%u\n\r", wheelL_hz, wheelR_hz);
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
