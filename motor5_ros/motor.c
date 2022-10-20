/*****
 * Motor Driver using DRV8833 Motor Driver Circuit
 *****/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <inttypes.h>

#include "motor.h"
#include "ssd1306.h"

const uint STBY_PIN = 6;
BiMotor mot1;
BiMotor mot2;
volatile uint64_t t1 = 0;
volatile uint64_t t2 = 0;    
const uint mot1_speed = 27;
const uint mot2_speed = 26;
const int32_t delay_ms_timer = 1000;
struct repeating_timer timer;

volatile int32_t speed1;
volatile int32_t speed2;


void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    if(leftSpeed >=0)
    {
      BiMotorspeed(&mot1,  leftSpeed, true);
    }
    else
    {
      BiMotorspeed(&mot1, -leftSpeed, false);
    }

    if(rightSpeed >=0)
    {
      BiMotorspeed(&mot2,  rightSpeed, true);
    }
    else
    {
      BiMotorspeed(&mot2,  -rightSpeed, false);
    }
}

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d)
{
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}

uint32_t pwm_get_wrap(uint slice_num)
{
    valid_params_if(PWM, slice_num >= 0 && slice_num < NUM_PWM_SLICES);
    return pwm_hw->slice[slice_num].top;
}

void pwm_set_duty(uint slice_num, uint chan, int d)
{
    pwm_set_chan_level(slice_num, chan, pwm_get_wrap(slice_num) * d / 100);
}

void BiMotorInit(BiMotor *m, uint gpioForward, uint gpioBackward, uint freq)
{
    gpio_set_function(gpioForward, GPIO_FUNC_PWM);
    m->gpioForward = gpioForward;
    m->slice = pwm_gpio_to_slice_num(gpioForward);
    m->Fchan = pwm_gpio_to_channel(gpioForward);

    gpio_set_function(gpioBackward, GPIO_FUNC_PWM);
    m->gpioBackward = gpioBackward;
    m->Bchan = pwm_gpio_to_channel(gpioBackward);

    m->freq = freq;
    m->speed = 0;
    m->forward = true;
    m->resolution = pwm_set_freq_duty(m->slice, m->Fchan, m->freq, 0);
    pwm_set_duty(m->slice, m->Bchan, 0);
    m->on = false;
}

void BiMotorspeed(BiMotor *m, int s, bool forward)
{
    if (forward)
    {
        pwm_set_duty(m->slice, m->Bchan, 0);
        pwm_set_duty(m->slice, m->Fchan, s);
        m->forward = true;
    }
    else
    {
        pwm_set_duty(m->slice, m->Fchan, 0);
        pwm_set_duty(m->slice, m->Bchan, s);
        m->forward = false;
    }
    m->speed = s;
}

void BiMotorOn(BiMotor *m)
{
    pwm_set_enabled(m->slice, true);
    m->on = true;
}

void BiMotorOff(BiMotor *m)
{
    pwm_set_enabled(m->slice, false);
    m->on = false;
}




void MyIRQHandler(uint gpio, uint32_t events)
{
    
    if( gpio == mot1_speed)
    {
        t1++;
    }
    else if( gpio == mot2_speed)
    {
        t2++;       
    }
    // printf("gpio=%u t1=%" PRIu64 "t2=%" PRIu64 "\n\r", gpio, t1,t2);
}

bool MyTimerHandler(struct repeating_timer *t)
{
    static uint64_t t10 = 0;
    static uint64_t t20 = 0;
    speed1 = (t1-t10)/(delay_ms_timer/1000);
    speed2 = (t2-t20)/(delay_ms_timer/1000);
    t10 = t1;
    t20 = t2;
    // t1 = 0;
    // t2 = 0;
    return true;
}

/* Wrap the encoder reading function */
uint64_t readEncoder(int i) {
    if (i == LEFT) return t1;
    else if (i == RIGHT) return t2;
    else return 7;
}

void resetEncoders()
{
    t1 = 0;
    t2 = 0;
}

int motor_init_all()
{
    stdio_init_all();

    // Timer

    add_repeating_timer_ms(delay_ms_timer, MyTimerHandler, NULL, &timer);

    //Interrupts for encoder
    gpio_set_function(mot1_speed, GPIO_FUNC_SIO);
    gpio_set_function(mot2_speed, GPIO_FUNC_SIO);
    gpio_set_dir(mot1_speed, false);
    gpio_pull_down(mot1_speed);
    gpio_set_dir(mot2_speed, false);
    gpio_pull_down(mot2_speed);

    gpio_set_irq_enabled_with_callback( mot1_speed, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &MyIRQHandler);
    gpio_set_irq_enabled_with_callback( mot2_speed, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &MyIRQHandler);


    BiMotorInit(&mot1, 3, 2, 2000);
    BiMotorInit(&mot2, 4, 5, 2000);
    
    gpio_init(STBY_PIN);
    gpio_set_dir(STBY_PIN, GPIO_OUT);
    gpio_put(STBY_PIN, 1);
    
    BiMotorOn(&mot1);
    BiMotorOn(&mot2);
    
    return 0;
}

void test_motor()
{
    debug_txt("Mini1");

    //blink_led(4);
    BiMotorspeed(&mot1, 100, true);
    BiMotorspeed(&mot2, 100, true);
    sleep_ms(2000);
    setMotorSpeeds(65,-90);   
    debug_txt("Slp 2s");
    sleep_ms(4000);
    printf("Speed 1= %i  Speed 2= %i\n", speed1,speed2);

    debug_txt("Mini2");
    //blink_led(5);
    BiMotorspeed(&mot1, 60, true);
    BiMotorspeed(&mot2, 60, true);
    sleep_ms(2000);   
    printf("Speed 1= %i  Speed 2= %i\n", speed1,speed2);

    debug_txt("Mini3");
    //blink_led(6);
    BiMotorspeed(&mot1, 100, false);
    BiMotorspeed(&mot2, 100, false);
    sleep_ms(2000);
    printf("Speed 1= %i  Speed 2= %i\n", speed1,speed2);

    debug_txt("Mini4");
    //blink_led(7);
    BiMotorspeed(&mot1, 60, false);
    BiMotorspeed(&mot2, 60, false);
    sleep_ms(2000);               
    printf("Speed 1= %i  Speed 2= %i\n", speed1,speed2);

    debug_txt("Mini5");
    sleep_ms(4000);
    BiMotorspeed(&mot1, 0, true);
    BiMotorspeed(&mot2, 0, true);    

    debug_txt("Speed");
    while(true)
    {
      setMotorSpeeds(65,-65);     
      printf("Speed 1= %i  Speed 2= %i\n\r", speed1,speed2);
      sleep_ms(1000);
    }
}