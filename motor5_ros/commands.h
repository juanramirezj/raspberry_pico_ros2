/* Define single-letter commands that will be sent by the PC over the
   serial link. 
   This is used for testing purposes because the main program will receive commands by ROS2 topics
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'

#endif