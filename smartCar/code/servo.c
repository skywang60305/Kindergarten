/*
 * servo.c
 *
 *  Created on: 2021��8��5��
 *      Author: 95159
 */

#include "servo.h"
uint16 SERVO_PWM_MID = 700;
uint16 SERVO_PWM_MAX = 75;
uint16 SERVO_PWM_MIN = 75;
uint16 servoPWM = 0;
ServoPID servoPID;
// fuzzy_PID servoPID_fuzzy;
// ����ѡ�ò���ģ��
uint8 flag_servoPID_fuzzy = 1;
void servo_init()
{
    servoPID_init();
    deviationParam_init();
#ifndef BOOM7_QT_DEBUG
    gtm_pwm_init(S3010_PWM_CH, S3010_HZ, SERVO_PWM_MID);
#endif
}

// �������������������ģ�������޷� �ǰ� �޷��ź��з����� ���бȽϣ���������з�����ת�����޷�������Ȼ��Ƚϣ��������кܴ��BUG
// ��Ϊ  ����� ������ ����-1 �Ǿͻ���һ���ܴ��ֵ�ˡ��⻹�Ƚϸ�ɶ������˵���������ٳ��ּ����ıȽ�
// ת�� ��D�� ��ż���������
int Calc_ServoPID(ServoPID *pp) // ����� KP=1��д��
{
    float kp, kd;
    float error;
    static float d_error = 0;
    error = pp->error;
    d_error = (pp->error - pp->preError) * (1 - pp->taw / 10.0) + // ����ȫ΢��   (pp->error - pp->preError)����ε�ƫ��仯����d_error��֮ǰ��ƫ�������  taw�����������Ŷ�
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
    // ��̬������ʼ��
    servoPID.kp_max = 100;     // ����ٶ�ʱ��kp
    servoPID.kp_min = 100;     // ����ٶ�ʱ��kp
    servoPID.kp_ramp = 110;    // �µ�kp
    servoPID.kp_onRamp = 100;  // �µ�kp
    servoPID.kd_max = 300;     // ����ٶ�ʱ��kd
    servoPID.kd_min = 300;     // ����ٶ�ʱ��kd
    servoPID.differential = 0; // �������kd��

    // ��������ʱʵ��ʹ�õ�ƫ�������ʼ��������ʼ��Ϊ���
    servoPID.kp = servoPID.kp_min;     // ��������PD���������p
    servoPID.kd_in = servoPID.kd_min;  // ��������PD�������������p
    servoPID.kd_out = servoPID.kd_min; // ��������PD��������ĳ���p

    // ƫ������ʼ��
    servoPID.error = 0;       // ��ǰƫ��
    servoPID.preError = 0;    // ǰһ��ƫ��
    servoPID.prepreError = 0; // ǰǰһ��ƫ��
    servoPID.taw = 5;         // ǰһ�κ�ǰǰһ�ε����Ŷ�
    servoPID.PID_Out = 0;     // ���ն�����ֵ
}
