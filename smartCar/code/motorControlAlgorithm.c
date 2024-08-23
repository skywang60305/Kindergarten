/*
 * myMotor.c
 *
 *  Created on: 2022��5��10��
 *      Author: 95159
 */
#include "motorControlAlgorithm.h"

MotorControlAlgorithm motorControlAlgorithm = PositionalPID;

// λ��ʽ
PID_Calc positionalPIDLeft, positionalPIDRight;
PID_Param positionalPIDParam;

// ����ʽ
PID_Calc incrementalPIDLeft, incrementalPIDRight;
PID_Param incrementalPIDParam;

// ��ṹ
PID_Calc variableStructurePIDLeft, variableStructurePIDRight;
Param_VariableStructurePID variableStructurePIDParam;

// �洫
PID_Calc zjutPIDLeft, zjutPIDRight;
Param_ZJUTPID zjutPIDParam;

uint32 cnt_motorPID_period = 0;
double motor_sumError_left = 0;
double motor_sumError_right = 0;

void motorPIDInit()
{
    // λ��ʽ��
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

    // ����ʽ��
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

    // ��ṹ��
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

    // �洫��
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
    // λ��ʽ��
    positionalPIDParam.kp = 150;
    positionalPIDParam.ki = 10;
    positionalPIDParam.kd = 100;
    positionalPIDParam.i_limit = 400;

    // ����ʽ��,û�õ������޷����������Ǹ���ֵ
    incrementalPIDParam.kp = 150;
    incrementalPIDParam.ki = 10;
    incrementalPIDParam.kd = 100;
    incrementalPIDParam.i_limit = 400;

    // ��ṹ��,�������޷������������ӣ���ע��
    variableStructurePIDParam.kp_base = 30;     // ����p�Ļ�׼��������p����仯��ƫ��Խ��pԽ�������ڹ�ʽ��
    variableStructurePIDParam.range_kp = 120;   // ����p�ı仯��Χ
    variableStructurePIDParam.decayRate_kp = 8; // ��ʽ��ָ����˥�����ʣ�Խ��ָ����˥��Խ�죬���ÿ���1p�ӵ�Խ��
                                                // ʵ���ϣ��������������ƫ����ʲô��Χ֮�⣬k����
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

    PID->error = PID->target - PID->feedBack;                                     // ƫ��
    PID->derivative = (PID->error - PID->preError) * 0.5 + PID->derivative * 0.5; // ΢����,����ȫ΢�֣���΢����ӵ����
    PID->integral += PID->error;

    if (PID == &positionalPIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &positionalPIDRight)
        motor_sumError_right += fabs(PID->error);

    PID->output = PID->error * (float)param->kp / 10 +      // ����
                  PID->derivative * (float)param->kd / 10 + // ΢��
                  PID->integral * (float)param->ki / 10;    // ����

    if (PID->integral > param->i_limit) // �����޷�
        PID->integral = param->i_limit;
    if (PID->integral < -param->i_limit) // �����޷�
        PID->integral = -param->i_limit;
    PID->preError = PID->error; // ǰһ�����ֵ

    return (int)PID->output;
}

int calcMotorPID_Incremental(PID_Calc *PID, PID_Param *param)
{
    float ep, ei, ed;
    PID->error = PID->target - PID->feedBack; // ƫ��

    if (PID == &incrementalPIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &incrementalPIDRight)
        motor_sumError_right += fabs(PID->error);

    ep = PID->error - PID->preError;
    ei = PID->error;
    ed = PID->error - 2 * PID->preError + PID->prePreError;
    PID->output = ep * (float)param->kp / 10 + // ����
                  ed * (float)param->kd / 10 + // ΢��
                  ei * (float)param->ki / 10;  // ����
    PID->prePreError = PID->preError;          // ǰһ�����ֵ
    PID->preError = PID->error;                // ǰһ�����ֵ
    return (int)PID->output;
}

// ��ṹPID
int calcMotorPID_VariableStructure(PID_Calc *PID, Param_VariableStructurePID *param)
{
    float kp, ki;
    PID->error = PID->target - PID->feedBack; // ƫ��
    PID->integral += PID->error;

    if (PID == &variableStructurePIDLeft)
        motor_sumError_left += fabs(PID->error);
    if (PID == &variableStructurePIDRight)
        motor_sumError_right += fabs(PID->error);

    kp = (float)param->kp_base + (float)param->range_kp * (1 - 1 / (exp((float)param->decayRate_kp / 100 * fabs(PID->error))));
    ki = (float)param->ki_base / exp((float)param->decayRate_ki / 100 * fabs(PID->error));

    PID->output = PID->error * kp / 10 +   // ����
                  PID->integral * ki / 10; // ����

    return (int)PID->output;
}

// �洫��PID�����ַ���+������������+����ȫ΢��
int calcMotorPID_ZJUT(PID_Calc *PID, Param_ZJUTPID *param)
{
    float kp, ki, kd;
    float once_i;
    // �����趨���ǵ�floatת��
    kp = (float)param->kp / 10;
    if (PID->error + PID->preError >= 0) // ����ֿ��ƣ�����ʱ?��ƫ��ϴ��ʱ���������ֵ�Ч��,��Ȼ���׳�����(���Ŀ��-���ʵ��ֵ+�ϴ�Ŀ��-�ϴ�ʵ��ֵ)>0����Ϊ����
        ki = ((float)param->change_ki / 10.0) - ((float)param->change_ki / 10.0) / (1 + exp((float)param->change_kib - 0.2 * fabs(PID->error)));
    else // ����ʱ? (ƫ����������仯��ƫ���С����ʱ��Ӵ�������ã���Ϊƫ��С)  ���ñ��ٻ���
        ki = (float)param->ki / 10;
    kd = (float)param->kd / 10;

    // ƫ��
    PID->error = PID->target - PID->feedBack;
    // ����ȫ΢��,��εĺ��ϴεĹ�ͬ����
    PID->derivative = (PID->error - PID->preError) * 0.5 + PID->derivative * 0.5;
    // ����Ӧ���ۻ��Ļ���ֵ
    once_i = ki * 0.5 * (PID->error + PID->preError);
    // ���ַ��룬����������һ���������������i��ģ�����i���Ƿ��ۻ��ں���������,���ڴ����д���ú����ڻ������ǳ���ϵ����ֵ
    PID->output = kp * PID->error + PID->integral + kd * PID->derivative;

    // ���Ƿ��Ѿ�����ִ�����Ĺ������룬�����Ļ�������ʹ����������ص�ƫ���ۻ�
    // δ����
    if (PID->output > MOTOR_PWM_MIN && PID->output < MOTOR_PWM_MAX)
    {
        PID->output += once_i;
        if (PID->output > MOTOR_PWM_MAX)
        {
            float temp = PID->output - MOTOR_PWM_MAX; // temp�ǻ��ֹ����͵Ĳ���
            once_i -= temp;                           // �ѻ��ֵ��� ���������͵Ĳ���
            PID->output = MOTOR_PWM_MAX;              // ռ�ձ���Ϊ��
        }
        else if (PID->output < MOTOR_PWM_MIN)
        {
            float temp = PID->output - MOTOR_PWM_MIN;
            once_i -= temp;
            PID->output = MOTOR_PWM_MIN;
        }
        PID->integral += once_i;
    }
    // ������ֻ�ۻ��뱥�ͷ����෴�����
    if ((PID->output >= MOTOR_PWM_MAX && once_i < 0) || (PID->output <= MOTOR_PWM_MIN && once_i > 0))
    {
        PID->integral += once_i;
        PID->output += once_i;
    }

    PID->preError = PID->error;
    return (int)PID->output;
}
