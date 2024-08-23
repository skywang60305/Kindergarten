/*
 * servo.c
 *
 *  Created on: 2021年8月5日
 *      Author: 95159
 */

#include "servo.h"
uint16 SERVO_PWM_MID = 700;
uint16 SERVO_PWM_MAX = 75;
uint16 SERVO_PWM_MIN = 75;
uint16 servoPWM = 0;
ServoPID servoPID;
// fuzzy_PID servoPID_fuzzy;
// 可以选用不用模糊
uint8 flag_servoPID_fuzzy = 1;
void servo_init()
{
    servoPID_init();
    deviationParam_init();
#ifndef BOOM7_QT_DEBUG
    gtm_pwm_init(S3010_PWM_CH, S3010_HZ, SERVO_PWM_MID);
#endif
}

// 这个函数本来是有问题的，里面的限幅 是把 无符号和有符号数 进行比较，这样会把有符号数转换成无符号数，然后比较，这样会有很大的BUG
// 因为  如果是 负数， 比如-1 那就会变成一个很大的值了。这还比较各啥？所以说最后代码中少出现减法的比较
// 转向 的D项 电磁加上陀螺仪
int Calc_ServoPID(ServoPID *pp) // 舵机的 KP=1不写死
{
    float kp, kd;
    float error;
    static float d_error = 0;
    error = pp->error;
    d_error = (pp->error - pp->preError) * (1 - pp->taw / 10.0) + // 不完全微分   (pp->error - pp->preError)是这次的偏差变化量，d_error是之前的偏差输出量  taw是两个的置信度
              d_error * (pp->taw / 10.0);
    pp->prepreError = pp->preError;
    pp->preError = error;
    kp = (float)(pp->kp / 100.0);
    if (IF.ramp == 1)
    {
        kp = kp * servoPID.kp_ramp / 100;
    }
    else if (IF.ramp == 2)
    {
        kp = kp * servoPID.kp_onRamp / 100;
    }
    if ((d_error > 0 && error < 0) || (d_error < 0 && error > 0))
    {
        kd = (float)(pp->kd_out / 100.0);
    }
    else
    {
        kd = (float)(pp->kd_in / 100.0);
    }
    if ((IF.ramp && IF.ramp != 3) || IF.annulus == AL5 || IF.annulus == AR5 || IF.annulusDelay)
    {
        kd = 0;
    }

    pp->PID_Out = (int)(kp * error + kd * d_error);
    // pp->PID_Out = (kp * error + kd * d_error);

    if (pp->PID_Out < -(int)SERVO_PWM_MAX)
    {
        pp->PID_Out = -(int)SERVO_PWM_MAX;
    }
    else if (pp->PID_Out > (int)SERVO_PWM_MIN)
    {
        pp->PID_Out = (int)SERVO_PWM_MIN;
    }
    return (pp->PID_Out);
}

void servoPID_init()
{
    // 动态参数初始化
    servoPID.kp_max = 100;     // 最高速度时的kp
    servoPID.kp_min = 100;     // 最低速度时的kp
    servoPID.kp_ramp = 110;    // 坡道kp
    servoPID.kp_onRamp = 100;  // 坡道kp
    servoPID.kd_max = 300;     // 最高速度时的kd
    servoPID.kd_min = 300;     // 最低速度时的kd
    servoPID.differential = 0; // 出入弯的kd差

    // 车辆运行时实际使用的偏差参数初始化，都初始化为最低
    servoPID.kp = servoPID.kp_min;     // 最终用于PD控制运算的p
    servoPID.kd_in = servoPID.kd_min;  // 最终用于PD控制运算的入弯p
    servoPID.kd_out = servoPID.kd_min; // 最终用于PD控制运算的出弯p

    // 偏差计算初始化
    servoPID.error = 0;       // 当前偏差
    servoPID.preError = 0;    // 前一次偏差
    servoPID.prepreError = 0; // 前前一次偏差
    servoPID.taw = 5;         // 前一次和前前一次的置信度
    servoPID.PID_Out = 0;     // 最终舵机打脚值
}
