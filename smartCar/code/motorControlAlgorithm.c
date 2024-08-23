/*
 * myMotor.c
 *
 *  Created on: 2022年5月10日
 *      Author: 95159
 */
#include "motorControlAlgorithm.h"

MotorControlAlgorithm motorControlAlgorithm = PositionalPID;

// 位置式
PID_Calc positionalPIDLeft, positionalPIDRight;
PID_Param positionalPIDParam;

// 增量式
PID_Calc incrementalPIDLeft, incrementalPIDRight;
PID_Param incrementalPIDParam;

// 变结构
PID_Calc variableStructurePIDLeft, variableStructurePIDRight;
Param_VariableStructurePID variableStructurePIDParam;

// 祖传
PID_Calc zjutPIDLeft, zjutPIDRight;
Param_ZJUTPID zjutPIDParam;

uint32 cnt_motorPID_period = 0;
double motor_sumError_left = 0;
double motor_sumError_right = 0;

void motorPIDInit()
{
    // 位置式的
    positionalPIDLeft.target = 0;
    positionalPIDLeft.feedBack = 0;
    positionalPIDLeft.error = 0;
    positionalPIDLeft.preError = 0;
    positionalPIDLeft.prePreError = 0;
    positionalPIDLeft.integral = 0;
    positionalPIDLeft.derivative = 0;
    positionalPIDLeft.output = 0;

    positionalPIDRight.target = 0;
    positionalPIDRight.feedBack = 0;
    positionalPIDRight.error = 0;
    positionalPIDRight.preError = 0;
    positionalPIDRight.prePreError = 0;
    positionalPIDRight.integral = 0;
    positionalPIDRight.derivative = 0;
    positionalPIDRight.output = 0;

    // 增量式的
    incrementalPIDLeft.target = 0;
    incrementalPIDLeft.feedBack = 0;
    incrementalPIDLeft.error = 0;
    incrementalPIDLeft.preError = 0;
    incrementalPIDLeft.prePreError = 0;
    incrementalPIDLeft.integral = 0;
    incrementalPIDLeft.derivative = 0;
    incrementalPIDLeft.output = 0;

    incrementalPIDRight.target = 0;
    incrementalPIDRight.feedBack = 0;
    incrementalPIDRight.error = 0;
    incrementalPIDRight.preError = 0;
    incrementalPIDRight.prePreError = 0;
    incrementalPIDRight.integral = 0;
    incrementalPIDRight.derivative = 0;
    incrementalPIDRight.output = 0;

    // 变结构的
    variableStructurePIDLeft.target = 0;
    variableStructurePIDLeft.feedBack = 0;
    variableStructurePIDLeft.error = 0;
    variableStructurePIDLeft.preError = 0;
    variableStructurePIDLeft.prePreError = 0;
    variableStructurePIDLeft.integral = 0;
    variableStructurePIDLeft.derivative = 0;
    variableStructurePIDLeft.output = 0;

    variableStructurePIDRight.target = 0;
    variableStructurePIDRight.feedBack = 0;
    variableStructurePIDRight.error = 0;
    variableStructurePIDRight.preError = 0;
    variableStructurePIDRight.prePreError = 0;
    variableStructurePIDRight.integral = 0;
    variableStructurePIDRight.derivative = 0;
    variableStructurePIDRight.output = 0;

    // 祖传的
    zjutPIDLeft.target = 0;
    zjutPIDLeft.feedBack = 0;
    zjutPIDLeft.error = 0;
    zjutPIDLeft.preError = 0;
    zjutPIDLeft.prePreError = 0;
    zjutPIDLeft.integral = 0;
    zjutPIDLeft.derivative = 0;
    zjutPIDLeft.output = 0;

    zjutPIDRight.target = 0;
    zjutPIDRight.feedBack = 0;
    zjutPIDRight.error = 0;
    zjutPIDRight.preError = 0;
    zjutPIDRight.prePreError = 0;
    zjutPIDRight.integral = 0;
    zjutPIDRight.derivative = 0;
    zjutPIDRight.output = 0;
}

void motorPIDParamInit()
{
    // 位置式的
    positionalPIDParam.kp = 150;
    positionalPIDParam.ki = 10;
    positionalPIDParam.kd = 100;
    positionalPIDParam.i_limit = 400;

    // 增量式的,没用到积分限幅，不过还是赋个值
    incrementalPIDParam.kp = 150;
    incrementalPIDParam.ki = 10;
    incrementalPIDParam.kd = 100;
    incrementalPIDParam.i_limit = 400;

    // 变结构的,不积分限幅，参数不复杂，看注释
    variableStructurePIDParam.kp_base = 30;     // 参数p的基准，真正的p随变差变化，偏差越大p越大，体现在公式中
    variableStructurePIDParam.range_kp = 120;   // 参数p的变化范围
    variableStructurePIDParam.decayRate_kp = 8; // 公式中指数的衰减速率，越大，指数项衰减越快，误差每变大1p加的越多
                                                // 实际上，这个参数决定了偏差在什么范围之外，k给满
    variableStructurePIDParam.ki_base = 13;
    variableStructurePIDParam.decayRate_ki = 5;

    zjutPIDParam.kp = 180;
    zjutPIDParam.ki = 9;
    zjutPIDParam.kd = 150;
    zjutPIDParam.change_ki = 10;
    zjutPIDParam.change_kib = 4;
}

int calcMotorPID_positional(PID_Calc *PID, PID_Param *param)
{

    PID->error = PID->target - PID->feedBack;                                     // 偏差
    PID->derivative = (PID->error - PID->preError) * 0.5 + PID->derivative * 0.5; // 微分量,不完全微分，给微分项加点惯性
    PID->integral += PID->error;

    if (PID == &positionalPIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &positionalPIDRight)
        motor_sumError_right += fabs(PID->error);

    PID->output = PID->error * (float)param->kp / 10 +      // 比例
                  PID->derivative * (float)param->kd / 10 + // 微分
                  PID->integral * (float)param->ki / 10;    // 积分

    if (PID->integral > param->i_limit) // 积分限幅
        PID->integral = param->i_limit;
    if (PID->integral < -param->i_limit) // 积分限幅
        PID->integral = -param->i_limit;
    PID->preError = PID->error; // 前一个误差值

    return (int)PID->output;
}

int calcMotorPID_Incremental(PID_Calc *PID, PID_Param *param)
{
    float ep, ei, ed;
    PID->error = PID->target - PID->feedBack; // 偏差

    if (PID == &incrementalPIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &incrementalPIDRight)
        motor_sumError_right += fabs(PID->error);

    ep = PID->error - PID->preError;
    ei = PID->error;
    ed = PID->error - 2 * PID->preError + PID->prePreError;
    PID->output = ep * (float)param->kp / 10 + // 比例
                  ed * (float)param->kd / 10 + // 微分
                  ei * (float)param->ki / 10;  // 积分
    PID->prePreError = PID->preError;          // 前一个误差值
    PID->preError = PID->error;                // 前一个误差值
    return (int)PID->output;
}

// 变结构PID
int calcMotorPID_VariableStructure(PID_Calc *PID, Param_VariableStructurePID *param)
{
    float kp, ki;
    PID->error = PID->target - PID->feedBack; // 偏差
    PID->integral += PID->error;

    if (PID == &variableStructurePIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &variableStructurePIDRight)
        motor_sumError_right += fabs(PID->error);

    kp = (float)param->kp_base + (float)param->range_kp * (1 - 1 / (exp((float)param->decayRate_kp / 100 * fabs(PID->error))));
    ki = (float)param->ki_base / exp((float)param->decayRate_ki / 100 * fabs(PID->error));

    PID->output = PID->error * kp / 10 +   // 比例
                  PID->integral * ki / 10; // 积分

    return (int)PID->output;
}

// 祖传的PID，积分分离+遇限削弱积分+不完全微分
int calcMotorPID_ZJUT(PID_Calc *PID, Param_ZJUTPID *param)
{
    float kp, ki, kd;
    float once_i;
    // 参数设定，记得float转换
    kp = (float)param->kp / 10;
    if (PID->error + PID->preError >= 0) // 变积分控制（加速时?即偏差较大的时候，削弱积分的效果,不然容易超调）(这次目标-这次实际值+上次目标-上次实际值)>0即认为加速
        ki = ((float)param->change_ki / 10.0) - ((float)param->change_ki / 10.0) / (1 + exp((float)param->change_kib - 0.2 * fabs(PID->error)));
    else // 减速时? (偏差出现正负变化即偏差很小，这时候加大积分作用，因为偏差小)  不用变速积分
        ki = (float)param->ki / 10;
    kd = (float)param->kd / 10;

    // 偏差
    PID->error = PID->target - PID->feedBack;
    // 不完全微分,这次的和上次的共同作用
    PID->derivative = (PID->error - PID->preError) * 0.5 + PID->derivative * 0.5;
    // 本次应该累积的积分值
    once_i = ki * 0.5 * (PID->error + PID->preError);
    // 积分分离，在这里先算一次输出，这里是有i项的，但是i项是否累积在后面做决策,便于代码编写，该函数内积分项是乘上系数的值
    PID->output = kp * PID->error + PID->integral + kd * PID->derivative;

    // 看是否已经超出执行器的工作范畴，超过的话不进行使溢出更加严重的偏差累积
    // 未超出
    if (PID->output > MOTOR_PWM_MIN && PID->output < MOTOR_PWM_MAX)
    {
        PID->output += once_i;
        if (PID->output > MOTOR_PWM_MAX)
        {
            float temp = PID->output - MOTOR_PWM_MAX; // temp是积分过饱和的部分
            once_i -= temp;                           // 把积分的量 砍掉过饱和的部分
            PID->output = MOTOR_PWM_MAX;              // 占空比设为满
        }
        else if (PID->output < MOTOR_PWM_MIN)
        {
            float temp = PID->output - MOTOR_PWM_MIN;
            once_i -= temp;
            PID->output = MOTOR_PWM_MIN;
        }
        PID->integral += once_i;
    }
    // 超出，只累积与饱和方向相反的误差
    if ((PID->output >= MOTOR_PWM_MAX && once_i < 0) || (PID->output <= MOTOR_PWM_MIN && once_i > 0))
    {
        PID->integral += once_i;
        PID->output += once_i;
    }

    PID->preError = PID->error;
    return (int)PID->output;
}
