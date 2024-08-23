/*
 * motor.h
 *
 *  Created on: 2021��8��10��
 *      Author: 95159
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#if 1
#define ZJUT_MOTORDRIVER
#else
#define SEEKFREE_MOTORDRIVER
#endif

#define MOTOR_HZ 17 * 1000
#define MOTOR_EN P22_1

#ifdef ZJUT_MOTORDRIVER
#define MOTOR_PWM_FORWARD_R ATOM0_CH2_P21_4
#define MOTOR_PWM_BACK_R ATOM0_CH3_P21_5
#define MOTOR_PWM_FORWARD_L ATOM1_CH1_P21_3
#define MOTOR_PWM_BACK_L ATOM0_CH0_P21_2 
#endif

#ifdef SEEKFREE_MOTORDRIVER
#define MOTOR_PWM_L ATOM1_CH1_P21_3
#define MOTOR_DIR_L P21_2
#define MOTOR_PWM_R ATOM0_CH2_P21_4
#define MOTOR_DIR_R P21_5

#define VALUE_DIR_FOWARD_L 1
#define VALUE_DIR_BACK_L 0
#define VALUE_DIR_FOWARD_R 0
#define VALUE_DIR_BACK_R 1
#endif

#define QUAD_MODULD_L TIM2_ENCODER
#define QUAD_MODULD_L_PIN0 TIM2_ENCODER_CH1_P33_7
#define QUAD_MODULD_L_PIN1 TIM2_ENCODER_CH2_P33_6

#define QUAD_MODULD_R TIM5_ENCODER
#define QUAD_MODULD_R_PIN0 TIM5_ENCODER_CH1_P10_3
#define QUAD_MODULD_R_PIN1 TIM5_ENCODER_CH2_P10_1

////�������
#define MPIDL motorPIDLeft
#define MPIDR motorPIDRight

#define MOTOR_PWM_MAX 999  // �޷�
#define MOTOR_PWM_MIN -999 // �޷�

// ���ʹ��
#define ENABLE_MOTOR gpio_set(MOTOR_EN, 1)
#define DISABLE_MOTOR gpio_set(MOTOR_EN, 0)

#define TRACK 0.164       // �־࣬��
#define SPD_CONVERT 57.8f // 1m/s 5783������/s

void motorPID_init(void);
void motor_init(void);
void setMotorPWM(void);
void stopMotor(void);
void getEncoder(void);
int kalman_filter_L(int nowSpeed_Value);
int kalman_filter_R(int nowSpeed_Value);
void getTrueSpeed();
void adjustMotorPID_visualScope(uint32 aimSpeedL, uint32 aimSpeedR);
void motorDriver_test();
uint8 carBrakeFinished();
#endif /* CODE_MOTOR_H_ */
