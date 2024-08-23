/*
 * servo.h
 *
 *  Created on: 2021Äê8ÔÂ5ÈÕ
 *      Author: 95159
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"

#define S3010_PWM_CH ATOM3_CH1_P33_5
#define S3010_HZ 50


typedef struct ServoPID
{
    float error;
    float preError;
    float prepreError;
    uint32 taw;

    uint32 kp;
    uint32 kd_in;
    uint32 kd_out;

    uint32 kp_ramp;
    uint32 kp_onRamp;
    uint32 kp_min;
    uint32 kp_max;
    uint32 kd_min;
    uint32 kd_max;
    uint32 differential;
    int PID_Out;

} ServoPID;
void servo_init(void);
int Calc_ServoPID(ServoPID *pp);
void servoPID_init(void);

extern uint16 SERVO_PWM_MID;
extern uint16 SERVO_PWM_MAX;
extern uint16 SERVO_PWM_MIN;
extern uint16 servoPWM;
extern ServoPID servoPID;
#endif /* CODE_SERVO_H_ */
