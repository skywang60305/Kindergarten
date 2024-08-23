/*
 * motor.c
 *
 *  Created on: 2021��8��10��
 *      Author: 95159
 */
#include "motor.h"

extern EulerAngleTypedef SystemAttitudeRate; // ��̬���ٶ�

int carStallCnt;
extern DeviationVAR DeviationVar;
extern volatile StopReason stopReason;

void motor_init() // ����������ʱ��ֻ�ܶ�һ���࣬��һ����0
{
    // ���������ʼ��
    motorPIDInit();
    motorPIDParamInit();
    ADRC_Init(&ADRC_Speed_Controller_l, &ADRC_Speed_Controller_r);

    // ���ʹ��
    gpio_init(MOTOR_EN, GPO, 1, GPO_PUSH_PULL);

    // ������ʹ��
    gpt12_init(QUAD_MODULD_L, QUAD_MODULD_L_PIN0, QUAD_MODULD_L_PIN1); // ��ʼ��������
    gpt12_clear(QUAD_MODULD_L);
    gpt12_init(QUAD_MODULD_R, QUAD_MODULD_R_PIN0, QUAD_MODULD_R_PIN1); // ��ʼ��������
    gpt12_clear(QUAD_MODULD_R);

#ifdef ZJUT_MOTORDRIVER
    pwm_init(MOTOR_PWM_FORWARD_L, MOTOR_HZ, 0);
    pwm_init(MOTOR_PWM_BACK_L, MOTOR_HZ, 0);
    pwm_init(MOTOR_PWM_FORWARD_R, MOTOR_HZ, 0);
    pwm_init(MOTOR_PWM_BACK_R, MOTOR_HZ, 0);
#endif

#ifdef SEEKFREE_MOTORDRIVER
    gpio_init(MOTOR_DIR_R, GPO, VALUE_DIR_FOWARD_R, GPO_PUSH_PULL);
    gpio_init(MOTOR_DIR_L, GPO, VALUE_DIR_FOWARD_L, GPO_PUSH_PULL);
    pwm_init(MOTOR_PWM_R, MOTOR_HZ, 0);
    pwm_init(MOTOR_PWM_L, MOTOR_HZ, 0);
#endif
}

void stopMotor()
{
    DISABLE_MOTOR;
#ifdef ZJUT_MOTORDRIVER
    pwm_set_duty(MOTOR_PWM_FORWARD_L, 0); // ��ߵ����ת
    pwm_set_duty(MOTOR_PWM_BACK_L, 0);    // ��ߵ����ת
    pwm_set_duty(MOTOR_PWM_FORWARD_R, 0); // �ұߵ����ת
    pwm_set_duty(MOTOR_PWM_BACK_R, 0);    // �ұߵ����ת
#endif
#ifdef SEEKFREE_MOTORDRIVER
    pwm_set_duty(MOTOR_PWM_L, 0); // ��ߵ��
    pwm_set_duty(MOTOR_PWM_R, 0); // �ұߵ��
#endif
}

void getEncoder()
{
    SI.varL[2] = SI.varL[1];
    SI.varL[1] = SI.varL[0];
    SI.nowSpeedL = (int)encoder_get_count(QUAD_MODULD_L) * 2.5 * 0.5;
    gpt12_clear(QUAD_MODULD_L);

    SI.varR[2] = SI.varR[1];
    SI.varR[1] = SI.varR[0];
    SI.nowSpeedR = -(int)encoder_get_count(QUAD_MODULD_R) * 2.5 * 0.5;
    gpt12_clear(QUAD_MODULD_R);
    //    send_uint16_data(uint16 data1,uint16 data2,uint16 data3,uint16 data4)

    // ԭ����
    //    SI.varL[0] = SI.nowSpeedL;
    //
    //    SI.varR[0] = SI.nowSpeedR;

    // ��ֵ�˲�
    SI.varL[0] = (SI.nowSpeedL + SI.varL[1] + SI.varL[2]) / 3;
    SI.varR[0] = (SI.nowSpeedR + SI.varR[1] + SI.varR[2]) / 3;

    // ������
    //    SI.varL[0] = kalman_filter_L(SI.nowSpeedL);
    //
    //    SI.varR[0] = kalman_filter_R(SI.nowSpeedR);

    // ��ͨ�˲�
    // ��ֹƵ�� ��������
    //    float cutoffFreq = 10, Ts = 0.01;
    //    float a = 1/(1+Ts * 2 * PI * cutoffFreq);
    //    a = 0.1;
    //    SI.varL[0] = a*SI.varL[0] + (1-a)*SI.nowSpeedL;
    //    SI.varR[0] = a*SI.varR[0] + (1-a)*SI.nowSpeedR;
}

void setMotorPWM() // ���ٿ����д���BUG  ����Ӧ�ó��ڶ�����Ǹ�PWM֮ǰ���޷���-�޷���
{
    float differential;
    float PWM;
    float MAX_PWM;

    // ChangeVar�Ƿ�ʹ��ͨ�����뿪��������          //�������Ҫ�Լ������� ����
    differential = (float)SI.differential / 100.0;
    // ���Ƕ��PID�����ֵ �����Ƕ�������PWM�����ȥ��D���ֵ
    PWM = (float)(SERVO_PWM_MID) - (float)(servoPWM); // �޷���-�޷���Ҫ���� ֮ǰ���������������
    // ����Ƕ����һ�������ŵļ���ֵ
    MAX_PWM = (float)(SERVO_PWM_MAX + SERVO_PWM_MIN) / 2;

    static uint8 cnt = 0;
    if (cnt > 20)
        return;
    if (runState != RUNNING) // ɲ�� ͣ�� ������ʱ ��ʱ�������ٶ�Ϊ0
    {
        SI.aimSpeedL = SI.aimSpeedR = 0;
        if (carBrakeFinished())
            cnt++;
        if (cnt > 20)
        {
            stopMotor();
            return;
        }
    }
    else // ���������ָ�����,���ּ��٣����ֲ���Ҫ����Ϊһ�������ǲ���򻬵�
    {
        cnt_motorPID_period++;
        // �������ּ���
        if (PWM < 0 && (DeviationVar.nowDeviation > 5 || DeviationVar.nowDeviation < -5)) // �����PWM��MAX_PWM ���Ƕ����ת��֮��ĳ� ��P��ʱ���ֵ����ΪD��仯̫��
        {
            SI.aimSpeedR = SI.aimSpeed;
            // ��ʽ   ���ֵ�����=Ŀ���ٶ�*( 1 - ���PID���/����������������*���ٵı�����P)
            // ��ʵ����   ������������һ��֮���һ��ϵ���� ���ڰ�����ģ��
            SI.aimSpeedL = (int)(SI.aimSpeed + (float)PWM / MAX_PWM * differential * SI.aimSpeed);
        }
        else if (PWM > 0 && (DeviationVar.nowDeviation > 5 || DeviationVar.nowDeviation < -5))
        {
            SI.aimSpeedL = SI.aimSpeed;
            SI.aimSpeedR = (int)(SI.aimSpeed - (float)PWM / MAX_PWM * differential * SI.aimSpeed);
        }
        else
        {
            SI.aimSpeedL = SI.aimSpeedR = SI.aimSpeed;
        }
    }

    SI.aimSpeedL = (SI.aimSpeedL + SI.preAimSpeedL + SI.prepreAimSpeedL) / 3;
    SI.aimSpeedR = (SI.aimSpeedR + SI.preAimSpeedR + SI.prepreAimSpeedR) / 3;

    SI.preAimSpeedL = SI.aimSpeedL;
    SI.prepreAimSpeedL = SI.preAimSpeedL;

    SI.preAimSpeedR = SI.aimSpeedR;
    SI.prepreAimSpeedR = SI.preAimSpeedR;

    if (visualScope_enable && runState != BRAKING)
        printf("%d,%d,%d,%d\n", SI.varL[0], SI.aimSpeedL, SI.varR[0], SI.aimSpeedR);

    // PID����
    switch (motorControlAlgorithm)
    {
    case PositionalPID:
        positionalPIDLeft.target = (float)SI.aimSpeedL;
        positionalPIDRight.target = (float)SI.aimSpeedR;
        positionalPIDLeft.feedBack = (float)SI.varL[0];
        positionalPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_positional(&positionalPIDLeft, &positionalPIDParam);
        SI.motorPWMR = calcMotorPID_positional(&positionalPIDRight, &positionalPIDParam);
        break;
    case IncrementalPID:
        incrementalPIDLeft.target = (float)SI.aimSpeedL;
        incrementalPIDRight.target = (float)SI.aimSpeedR;
        incrementalPIDLeft.feedBack = (float)SI.varL[0];
        incrementalPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML += calcMotorPID_Incremental(&incrementalPIDLeft, &incrementalPIDParam);
        SI.motorPWMR += calcMotorPID_Incremental(&incrementalPIDRight, &incrementalPIDParam);
        break;
    case VariableStructurePID:
        variableStructurePIDLeft.target = (float)SI.aimSpeedL;
        variableStructurePIDRight.target = (float)SI.aimSpeedR;
        variableStructurePIDLeft.feedBack = (float)SI.varL[0];
        variableStructurePIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_VariableStructure(&variableStructurePIDLeft, &variableStructurePIDParam);
        SI.motorPWMR = calcMotorPID_VariableStructure(&variableStructurePIDRight, &variableStructurePIDParam);
        break;
    case ZjutPID:
        zjutPIDLeft.target = (float)SI.aimSpeedL;
        zjutPIDRight.target = (float)SI.aimSpeedR;
        zjutPIDLeft.feedBack = (float)SI.varL[0];
        zjutPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_ZJUT(&zjutPIDLeft, &zjutPIDParam);
        SI.motorPWMR = calcMotorPID_ZJUT(&zjutPIDRight, &zjutPIDParam);
        break;
    case ADRC:
        ADRC_Control(&ADRC_Speed_Controller_l, (float)SI.aimSpeedL, (float)SI.varL[0]);
        ADRC_Control(&ADRC_Speed_Controller_r, (float)SI.aimSpeedR, (float)SI.varR[0]);
        SI.motorPWML = ADRC_Speed_Controller_l.u / 10.0;
        SI.motorPWMR = ADRC_Speed_Controller_r.u / 10.0;
        break;
    };

    if (SI.motorPWML > MOTOR_PWM_MAX)
        SI.motorPWML = MOTOR_PWM_MAX;
    if (SI.motorPWMR > MOTOR_PWM_MAX)
        SI.motorPWMR = MOTOR_PWM_MAX;
    if (SI.motorPWML < MOTOR_PWM_MIN)
        SI.motorPWML = MOTOR_PWM_MIN;
    if (SI.motorPWMR < MOTOR_PWM_MIN)
        SI.motorPWMR = MOTOR_PWM_MIN;

    if (runState == BRAKING) //  ����Ϊɲ����ʱ�� ��ռ�ձ�������-500���ϣ�Ӧ���������ӻ��ƣ�ͣ��ʱ��0
    {
        if (SI.motorPWML < -500)
            SI.motorPWML = -500;
        if (SI.motorPWMR < -500)
            SI.motorPWMR = -500;
    }

    //    SI.motorPWML = SI.motorPWML * 0.75;
    //    SI.motorPWMR = SI.motorPWMR * 0.75;

    // ˫�������
    if ((SI.varL[0] < 80 && SI.motorPWML > 450) || (SI.varR[0] < 80 && SI.motorPWMR > 450)) // ��ת����
    {
        carStallCnt++;
        if (carStallCnt > 500)
            carStallCnt = 500;
    }
    else
    {
        carStallCnt = 0;
    }

    if (carStallCnt > 60)
    {
        runState = BRAKING;
        stopReason = StallProtect;
#ifdef ZJUT_MOTORDRIVER
        pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
        pwm_set_duty(MOTOR_PWM_BACK_L, 0);
        pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
        pwm_set_duty(MOTOR_PWM_BACK_R, 0);
#endif
#ifdef SEEKFREE_MOTORDRIVER
        pwm_set_duty(MOTOR_PWM_R, 0);
        pwm_set_duty(MOTOR_PWM_L, 0);
#endif
    }
    else
    {
#ifdef ZJUT_MOTORDRIVER
        if (SI.motorPWML >= 0)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 10 * SI.motorPWML);
            pwm_set_duty(MOTOR_PWM_BACK_L, 0);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_BACK_L, 10 * (-SI.motorPWML));
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
        }

        if (SI.motorPWMR >= 0)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 10 * SI.motorPWMR);
            pwm_set_duty(MOTOR_PWM_BACK_R, 0);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_BACK_R, 10 * (-SI.motorPWMR));
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
        }
#endif
#ifdef SEEKFREE_MOTORDRIVER
        if (SI.motorPWML >= 0)
        {
            pwm_set_duty(MOTOR_PWM_L, 10 * SI.motorPWML);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_FOWARD_L);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_L, 10 * (-SI.motorPWML));
            gpio_set(MOTOR_DIR_L, VALUE_DIR_BACK_L);
        }

        if (SI.motorPWMR >= 0)
        {
            pwm_set_duty(MOTOR_PWM_R, 10 * SI.motorPWMR);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_FOWARD_R);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_R, 10 * (-SI.motorPWMR));
            gpio_set(MOTOR_DIR_R, VALUE_DIR_BACK_R);
        }
#endif
    }
}

uint8 carBrakeFinished()
{
    if ((SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0]) <= 0)
        return 1;
    return 0;
}

void adjustMotorPID_visualScope(uint32 aimSpeedL, uint32 aimSpeedR)
{
    int aimSpL = (int)aimSpeedL - 300;
    int aimSpR = (int)aimSpeedR - 300;
    getEncoder();
    switch (motorControlAlgorithm)
    {
    case PositionalPID:
        positionalPIDLeft.target = (float)aimSpL;
        positionalPIDRight.target = (float)aimSpR;
        positionalPIDLeft.feedBack = (float)SI.varL[0];
        positionalPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_positional(&positionalPIDLeft, &positionalPIDParam);
        SI.motorPWMR = calcMotorPID_positional(&positionalPIDRight, &positionalPIDParam);
        break;
    case IncrementalPID:
        incrementalPIDLeft.target = (float)aimSpL;
        incrementalPIDRight.target = (float)aimSpR;
        incrementalPIDLeft.feedBack = (float)SI.varL[0];
        incrementalPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML += calcMotorPID_Incremental(&incrementalPIDLeft, &incrementalPIDParam);
        SI.motorPWMR += calcMotorPID_Incremental(&incrementalPIDRight, &incrementalPIDParam);
        break;
    case VariableStructurePID:
        variableStructurePIDLeft.target = (float)aimSpL;
        variableStructurePIDRight.target = (float)aimSpR;
        variableStructurePIDLeft.feedBack = (float)SI.varL[0];
        variableStructurePIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_VariableStructure(&variableStructurePIDLeft, &variableStructurePIDParam);
        SI.motorPWMR = calcMotorPID_VariableStructure(&variableStructurePIDRight, &variableStructurePIDParam);
        break;
    case ZjutPID:
        zjutPIDLeft.target = (float)aimSpL;
        zjutPIDRight.target = (float)aimSpR;
        zjutPIDLeft.feedBack = (float)SI.varL[0];
        zjutPIDRight.feedBack = (float)SI.varR[0];
        SI.motorPWML = calcMotorPID_ZJUT(&zjutPIDLeft, &zjutPIDParam);
        SI.motorPWMR = calcMotorPID_ZJUT(&zjutPIDRight, &zjutPIDParam);
        break;
    case ADRC:
        // һ��һ����
        // ��һ���������ٸ���΢����
        //            Fhan_ADRC(&ADRC_Speed_Controller_l, (float)aimSpL);
        //            send_int16_data(aimSpL, ADRC_Speed_Controller_l.x1, ADRC_Speed_Controller_l.x2, ADRC_Speed_Controller_l.fh);
        // �ڶ�����������״̬�۲���
        //            ADRC_Speed_Controller_l.y = SI.varL[0];
        //            ESO_ADRC(&ADRC_Speed_Controller_l);
        //            send_int16_data(ADRC_Speed_Controller_l.y, ADRC_Speed_Controller_l.z1, ADRC_Speed_Controller_l.z2, ADRC_Speed_Controller_l.z3);
        // ��󣬵�PD
        ADRC_Control(&ADRC_Speed_Controller_l, (float)aimSpL, (float)SI.varL[0]);
        ADRC_Control(&ADRC_Speed_Controller_r, (float)aimSpR, (float)SI.varR[0]);
        //            send_int16_data(ADRC_Speed_Controller_l.y, ADRC_Speed_Controller_l.z1, ADRC_Speed_Controller_l.z2, ADRC_Speed_Controller_l.z3);
        SI.motorPWML = ADRC_Speed_Controller_l.u / 10;
        SI.motorPWMR = ADRC_Speed_Controller_r.u / 10;
        send_int16_data(SI.varL[0], aimSpL, ADRC_Speed_Controller_l.z3, SI.motorPWML);
        break;
    }
    if (SI.motorPWML > MOTOR_PWM_MAX)
        SI.motorPWML = MOTOR_PWM_MAX;
    if (SI.motorPWMR > MOTOR_PWM_MAX)
        SI.motorPWMR = MOTOR_PWM_MAX;
    if (SI.motorPWML < MOTOR_PWM_MIN)
        SI.motorPWML = MOTOR_PWM_MIN;
    if (SI.motorPWMR < MOTOR_PWM_MIN)
        SI.motorPWMR = MOTOR_PWM_MIN;
    if ((SI.varL[0] < 80 && SI.motorPWML > 450) || (SI.varR[0] < 80 && SI.motorPWMR > 450)) // ��ת����
    {
        carStallCnt++;
        if (carStallCnt > 500)
            carStallCnt = 500;
    }
    else
    {
        carStallCnt = 0;
    }

    //    SI.motorPWML = SI.motorPWML * 0.75;
    //    SI.motorPWMR = SI.motorPWMR * 0.75;

    if (carStallCnt > 60)
    {
        runState = BRAKING;
        stopReason = StallProtect;
#ifdef ZJUT_MOTORDRIVER
        pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
        pwm_set_duty(MOTOR_PWM_BACK_L, 0);
        pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
        pwm_set_duty(MOTOR_PWM_BACK_R, 0);
#endif
#ifdef SEEKFREE_MOTORDRIVER
        pwm_set_duty(MOTOR_PWM_R, 0);
        pwm_set_duty(MOTOR_PWM_L, 0);
#endif
    }
    else
    {
#ifdef ZJUT_MOTORDRIVER
        if (SI.motorPWML >= 0)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 10 * SI.motorPWML);
            pwm_set_duty(MOTOR_PWM_BACK_L, 0);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_BACK_L, 10 * (-SI.motorPWML));
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
        }

        if (SI.motorPWMR >= 0)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 10 * SI.motorPWMR);
            pwm_set_duty(MOTOR_PWM_BACK_R, 0);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_BACK_R, 10 * (-SI.motorPWMR));
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
        }
#endif
#ifdef SEEKFREE_MOTORDRIVER
        if (SI.motorPWML >= 0)
        {
            pwm_set_duty(MOTOR_PWM_L, 10 * SI.motorPWML);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_FOWARD_L);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_L, 10 * (-SI.motorPWML));
            gpio_set(MOTOR_DIR_L, VALUE_DIR_BACK_L);
        }

        if (SI.motorPWMR >= 0)
        {
            pwm_set_duty(MOTOR_PWM_R, 10 * SI.motorPWMR);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_FOWARD_R);
        }
        else
        {
            pwm_set_duty(MOTOR_PWM_R, 10 * (-SI.motorPWMR));
            gpio_set(MOTOR_DIR_R, VALUE_DIR_BACK_R);
        }
#endif
    }
    //    if(motorControlAlgorithm != ADRC)
    //        send_int16_data(aimSpL,SI.varL[0],aimSpR,SI.varR[0]);
    printf("%d,%d,%d,%d\n", SI.varL[0], aimSpL, SI.varR[0], aimSpR);
}

void motorDriver_test()
{
    lcd_clear_all(WHITE);
    tft180_set_color(RED, WHITE);
    lcd_showstr(0, 1, "Press");
    lcd_showstr(0, 2, "KEY_B:left forward");
    lcd_showstr(0, 3, "KEY_U:left backward");
    lcd_showstr(0, 4, "KEY_R:right forward");
    lcd_showstr(0, 5, "KEY_D:right backward");

//    lcd_showstr(0, 7, "Press KEY_L to exit");
#ifdef SEEKFREE_MOTORDRIVER
    while (keymsg.key != KEY_L)
    {
        if (keymsg.key == KEY_B)
        {
            pwm_set_duty(MOTOR_PWM_L, 2000);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_FOWARD_L);
            pwm_set_duty(MOTOR_PWM_R, 0);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_FOWARD_R);
        }
        if (keymsg.key == KEY_U)
        {
            pwm_set_duty(MOTOR_PWM_L, 2000);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_BACK_L);
            pwm_set_duty(MOTOR_PWM_R, 0);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_FOWARD_R);
        }
        if (keymsg.key == KEY_R)
        {
            pwm_set_duty(MOTOR_PWM_L, 0);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_FOWARD_L);
            pwm_set_duty(MOTOR_PWM_R, 2000);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_FOWARD_R);
        }
        if (keymsg.key == KEY_D)
        {
            pwm_set_duty(MOTOR_PWM_L, 0);
            gpio_set(MOTOR_DIR_L, VALUE_DIR_FOWARD_L);
            pwm_set_duty(MOTOR_PWM_R, 2000);
            gpio_set(MOTOR_DIR_R, VALUE_DIR_BACK_R);
        }
        //        lcd_showstr(0, 6, "nowSL:");
        //        lcd_showint16(50, 6, SI.nowSpeedL);
        //        lcd_showstr(0, 7, "nowSR:");
        //        lcd_showint16(50, 7, SI.nowSpeedR);
        tft180_show_string(0, 6 * 16, "nowSL:");
        tft180_show_int(100, 6 * 16, SI.nowSpeedL, 4);
        tft180_show_string(0, 7 * 16, "nowSR:");
        tft180_show_int(100, 7 * 16, SI.nowSpeedR, 4);
    }
#endif
#ifdef ZJUT_MOTORDRIVER
    while (keymsg.key != KEY_L)
    {
        if (keymsg.key == KEY_B)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 8000);
            pwm_set_duty(MOTOR_PWM_BACK_L, 0);
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
            pwm_set_duty(MOTOR_PWM_BACK_R, 0);

            lcd_showstr(0 , 7,"B");
        }
        if (keymsg.key == KEY_U)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
            pwm_set_duty(MOTOR_PWM_BACK_L, 8000);
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
            pwm_set_duty(MOTOR_PWM_BACK_R, 0);
            lcd_showstr(0 , 7,"U");
        }
        if (keymsg.key == KEY_R)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
            pwm_set_duty(MOTOR_PWM_BACK_L, 0);
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 8000);
            pwm_set_duty(MOTOR_PWM_BACK_R, 0);
            lcd_showstr(0 , 7,"R");
        }
        if (keymsg.key == KEY_D)
        {
            pwm_set_duty(MOTOR_PWM_FORWARD_L, 0);
            pwm_set_duty(MOTOR_PWM_BACK_L, 0);
            pwm_set_duty(MOTOR_PWM_FORWARD_R, 0);
            pwm_set_duty(MOTOR_PWM_BACK_R, 8000);
            lcd_showstr(0 , 7,"D");
        }
        lcd_showstr(0, 1, "nowSL:");
        lcd_showint32(48, 1, SI.nowSpeedL);
        lcd_showstr(80, 1, "nowSR:");
        lcd_showint32(128, 1, SI.nowSpeedR);
    }
#endif
    //    pwm_set_duty(MOTOR_PWM_L       ,  0                 );
    //    pwm_set_duty(MOTOR_PWM_R       ,  0                 );
    lcd_clear_all(WHITE);
}

/*****************************************************
 *�������ƣ�kalman_filter
 *�������ܣ�ADC_�˲�
 *��ڲ�����ADC_Value
 *���ڲ�����kalman_adc
 *****************************************************/
int kalman_filter_L(int nowSpeed_Value)
{
    float x_k1_k1, x_k_k1;
    static float lastSpeed_L;
    float Z_k;
    static float P_k1_k1_L;

    static float Q_L = 0.0001; // Q�����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    static float R_L = 0.005;  // R������������R���󣬶�̬��Ӧ�����������ȶ��Ա��
    static float Kg_L = 0;
    static float P_k_k1_L = 1;

    float kalman_speed;
    static float kalman_speed_old_L = 0;
    Z_k = (float)nowSpeed_Value;
    x_k1_k1 = kalman_speed_old_L;

    x_k_k1 = x_k1_k1;
    P_k_k1_L = P_k1_k1_L + Q_L;

    Kg_L = P_k_k1_L / (P_k_k1_L + R_L);

    kalman_speed = x_k_k1 + Kg_L * (Z_k - kalman_speed_old_L);
    P_k1_k1_L = (1 - Kg_L) * P_k_k1_L;
    P_k_k1_L = P_k1_k1_L;

    lastSpeed_L = (float)nowSpeed_Value;
    kalman_speed_old_L = kalman_speed;

    return kalman_speed;
}

int kalman_filter_R(int nowSpeed_Value)
{
    float x_k1_k1, x_k_k1;
    static float lastSpeed_R;
    float Z_k;
    static float P_k1_k1_L;

    static float Q_L = 0.0001; // Q�����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    static float R_L = 0.005;  // R������������R���󣬶�̬��Ӧ�����������ȶ��Ա��
    static float Kg_L = 0;
    static float P_k_k1_L = 1;

    float kalman_speed;
    static float kalman_speed_old_R = 0;
    Z_k = (float)nowSpeed_Value;
    x_k1_k1 = kalman_speed_old_R;

    x_k_k1 = x_k1_k1;
    P_k_k1_L = P_k1_k1_L + Q_L;

    Kg_L = P_k_k1_L / (P_k_k1_L + R_L);

    kalman_speed = x_k_k1 + Kg_L * (Z_k - kalman_speed_old_R);
    P_k1_k1_L = (1 - Kg_L) * P_k_k1_L;
    P_k_k1_L = P_k1_k1_L;

    lastSpeed_R = (float)nowSpeed_Value;
    kalman_speed_old_R = kalman_speed;

    return kalman_speed;
}
