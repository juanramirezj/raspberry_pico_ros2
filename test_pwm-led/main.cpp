// Output PWM signal on pins 0 and 1

#include <stdio.h>


#include "pico/stdlib.h"
#include "hardware/pwm.h"

/*
GPIO    0  	1  2  3  4  5  6  7  8  9  10  11  12  13  14  15
PWM
Channel 0A 	0B 1A 1B 2A 2B 3A 3B 4A 4B 5A  5B  6A  6B  7A  7B

GPIO   16  17  18 19 20 21 22 23 24 25 26  27  28  29

PWM
Channel 0A 0B  1A 1B 2A 2B 3A 3B 4A 4B 5A  5B  6A  6B
*/


uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d);
uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, int d);



uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d)
{
 uint32_t clock = 125000000; // default clock 125 MHz
 uint32_t divider16 = clock / f / 4096 + 
                           (clock % (f * 4096) != 0);
 if (divider16 / 16 == 0)
 divider16 = 16;
 uint32_t wrap = clock * 16 / divider16 / f - 1;
 pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                     divider16 & 0xF);
 pwm_set_wrap(slice_num, wrap);
 pwm_set_chan_level(slice_num, chan, wrap * d / 100);
 return wrap;
}

uint32_t pwm_get_wrap(uint slice_num)
{  
    valid_params_if(PWM, slice_num >= 0 &&  slice_num < NUM_PWM_SLICES);  
    return pwm_hw->slice[slice_num].top;  
} 

void pwm_set_duty(uint slice_num, uint chan, int d)  
{  
    pwm_set_chan_level(slice_num,chan,pwm_get_wrap(slice_num)*d/100);  
} 


int main()  
{  
    gpio_set_function(25, GPIO_FUNC_PWM);  
    uint slice_num = pwm_gpio_to_slice_num(25);  
    uint chan = pwm_gpio_to_channel(25);  
    
    pwm_set_freq_duty(slice_num, chan, 2000, 0);  
    pwm_set_enabled(slice_num, true);  
    while (true)  
    {  
        for (int d = 0; d <= 100; d++)  
        {  
            pwm_set_duty(slice_num, chan, d);  
            sleep_ms(50);  
        }  
    } 
 } 
 