/*
 * speed.h
 *
 *  Created on: 2021��8��11��
 *      Author: 95159
 */

#ifndef CODE_SPEED_H_
#define CODE_SPEED_H_

#include "zf_common_headfile.h"
#define SI speedInfo
#define SP speedParam
#define TDP trackDectionParam

typedef struct SpeedInfo
{
    int varL[3];   // ������ֵ
    int varR[3];   // ������ֵ
    int nowSpeedL; // ����ʵ���ٶ�
    int nowSpeedR; // ����ʵ���ٶ�
    int aimSpeed;  // Ŀ���ٶ�
    int aimSpeedL; // ����Ŀ���ٶȣ����ϲ��ٵģ�
    int aimSpeedR; // ����Ŀ���ٶȣ����ϲ��ٵģ�
    int preAimSpeedL;
    int prepreAimSpeedL;
    int preAimSpeedR;
    int prepreAimSpeedR;
    int motorPWML;       // ����PWM
    int motorPWMR;       // ����PWM
    uint16 differential; // ����
    uint8 realSpeedTop;
} SpeedInfo;

typedef enum SpeedType
{
    NORMAL_SHIFT, // ���ι�ʽ
    FULL_ACCELE,  // ֱ��ȫ��
    BRAKE,        // ɲ��
} SpeedType;

typedef enum BrakeType
{
    STRIGHT_BRAKE_NORMAL, // ɲ����NORMAL
    STRIGHT_BRAKE_CURVE,  // ɲ����CURVE
    MINSP_BRAKE,
} BrakeType;

typedef struct SpeedParam
{
    uint16 maxSpeed; // ֱ������ٶ�   ��ֱ���ٶ�
    uint16 minSpeed; // ��С�ٶ� (ȫ����)

    uint16 normalSpeed; // normalShift����ʱ�����������ٶȣ�variableSpeedʱ�Ǽ���״̬�µ�����ٶ�
    uint16 curveSpeed;  // ���ι�ʽ�������ٶ�

    uint16 speedK;         // ɲ��ϵ��
    uint16 speedK2;        // ���ι�ʽϵ��
    uint16 annulusSpeedK2; // Բ�����ι�ʽϵ��

    // ��Ԫ�ص��ٶ�
    uint16 constantSpeed;    // ����
    uint16 annulusSpeed;     // Բ���ٶ�
    uint16 annulusMinSpeed;  // Բ����С�ٶ�
    uint16 outUCrossSpeed;   // ����ʮ�ֳ��ڼ���
    uint16 forkSpeed;        // �����ٶ�
    uint16 rampUpSpeed[3];   // ���µ��ٶ�
    uint16 rampOnSpeed[3];   // �����ٶ�
    uint16 rampDownSpeed[3]; // �����ٶ�
    uint16 outGarageSpeed;   // �����ٶ�
    uint16 garageSpeed;      // �е������ʱ��ɲ���ٶ�

    uint16 switchSpeedTop;   // normalShift����ʱ�л�����ٵĵ�����
    uint16 strightSpeedCut;  // ��ֱ��ɲ������
    uint16 brakeEdge_normal; // ɲ��ʱ����ǰ�ٶ������ٶȼ���brakeEdgeʱ����ִ��ɲ��
    uint16 brakeEdge_curve;
    uint16 brakeEdge_min;
} SpeedParam;

typedef struct TrackDectionParam
{
    uint16 strightAD;    // ��ֱ�����ж�
    uint16 enterCurveAD; // ������ٵ��ж�

    uint16 brakeTime; // ɲ��ʱ��
    uint16 limitTime; // ������ʱ�䣨û�õ���

    uint16 brakeTest; // ɲ������
    uint16 brakeTop_1;
    uint16 brakeTop_2;
    uint16 brakeTop_3;
    uint16 brakeTop_4;
    uint16 brakeTop_0;

    uint16 brakeSpeedTop_1; // ����ɲ��ʱ��TOP��
    uint16 brakeSpeedTop_2;
    uint16 brakeSpeedTop_3;
    uint16 brakeSpeedTop_4;
    uint16 brakeSpeedTop_0;

} TrackDectionParam;
void speed_init(void);
void speedParam_init(void);
void SpeedInfo_Init(void);
void trackDectionParam_init(void);
SpeedType getSpeedType(void);
uint8 isBrakeFinished(BrakeType brakeType);
int getAimBrakeSpeed();
uint8 getSpeedTopByTFMINI();

int getAimSpeed(void);
int getAimSpeed_constantSpeed();
int getAimSpeed_normalShift();
int getAimSpeed_variableSpeed();
int myNewGetAimSpeed_variableSpeed();

extern volatile SpeedParam SP;
extern volatile SpeedInfo SI;
extern SpeedType speedType;
extern BrakeType brakeType;
extern TrackDectionParam trackDectionParam;
extern uint8 limitFlag;
#endif /* CODE_SPEED_H_ */
