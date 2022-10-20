#ifndef MOTOR_H
#define MOTOR_H

#define LEFT 1
#define RIGHT 2
#define MAX_PWM 100 // 255

typedef struct
{
    uint gpioForward;
    uint gpioBackward;
    uint slice;
    uint Fchan;
    uint Bchan;
    bool forward;
    uint speed;
    uint freq;
    uint resolution;
    bool on;
} BiMotor;



extern const uint STBY_PIN;

extern volatile int32_t speed1;
extern volatile int32_t speed2;

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);
uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, int d);
extern BiMotor mot1;
extern BiMotor mot2;

void BiMotorInit(BiMotor *m, uint gpioForward, uint gpioBackward, uint freq);
void BiMotorspeed(BiMotor *m, int s, bool forward);
void BiMotorOn(BiMotor *m);
void BiMotorOff(BiMotor *m);
void MyIRQHandler(uint gpio, uint32_t events);
bool MyTimerHandler(struct repeating_timer *t);
int motor_init_all();
void test_motor();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
uint64_t readEncoder(int i);
void resetEncoders();
void blink_led(int n);
#endif