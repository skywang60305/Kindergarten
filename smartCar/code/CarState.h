/*
 * CarState.h
 *
 *  Created on: 2023��10��19��
 *      Author: 86135
 */

#ifndef CARSTATE_H_
#define CARSTATE_H_
#include "zf_common_headfile.h"
#define programRunTime PRT
typedef enum RunState
{
    STOP,
    READY,
    RUNNING,
    BRAKING,
} RunState;

/* С������ģʽö������ */
typedef enum RunMode
{
    NORMAL_RUN,
    TIMING_RUN,
    ADC_RUN,
} RunMode;

typedef enum RunSpeedMode
{
    VARIABLE_SPEED,
    CONSTANT_SPEED,
    NORMAL_SHIFT_SPEED,
    ADC_SPEED,
} RunSpeedMode;

////ͣ��ԭ��
// typedef enum StopReason
//{
//     HaventStarted,
//     TimeUp,
//     RunOutLine,
//     EnterGarage,
//     StallProtect,
// } StopReason;

typedef struct ProgramRunTime
{
    uint16 whileTime;
    uint16 _whileTime;
    uint16 maxWhileTime;
    uint16 aveWhileTime;
    uint16 goTime;
    uint16 maxGoTime;
    uint16 aveGoTime;
    uint16 allTime;
    uint16 sdSaveTime;
    uint16 vcanTime;
} ProgramRunTime;

typedef struct
{
    uint32 AUTO_EXP_TEMP;
    uint32 EXP_TIME_TEMP;
    uint32 FPS_TEMP;
    uint32 GAIN_TEMP;
    // uint32_t UD_OFFSET;    //����ƫ��
} CAMERA_DEBUG_VAR;

typedef enum
{
    WAITING,
    WORKING,
} CoreStatus;

#define BEEP_ENABLE P33_8
#define BUZZER_ON gpio_set_level(BEEP_ENABLE, 1)
#define BUZZER_OFF gpio_set_level(BEEP_ENABLE, 0)

#define BEEP(i)              \
    do                       \
    {                        \
        BUZZER_ON;           \
        buzzerTime = i / 10; \
    } while (0);

void time_init();
void cameraDebugVarInit();
void cameraVarInitFromEeprom();
extern volatile RunState runState;
extern volatile StopReason stopReason;
extern volatile RunMode runMode;
extern volatile RunSpeedMode runSpeedMode;
extern int16 buzzerTime;
extern CoreStatus cpu0;
extern CoreStatus cpu1;
extern ProgramRunTime programRunTime;
extern uint16 over3msTimeCnt;
extern uint16 over5msTimeCnt;
extern uint16 over8msTimeCnt;
extern uint16 sdSaveCnt;        //  SD��Ĵ���
extern uint16 programCnt;       //  ����while�Ĵ���
extern uint16 programOver18Cnt; //  ���򳬹�18MS�Ĵ���
extern float sdStability;       //  SD�ȶ���
extern float programStability;  //  �����ȶ���
extern uint16 maxAllTime;       //  ���й����е����whileʱ��
extern ProgramRunTime programRunTime;
// extern volatile RunState runState;
// extern volatile StopReason stopReason;
// extern volatile RunMode runMode;
// extern volatile RunSpeedMode runSpeedMode;
extern CAMERA_DEBUG_VAR cameraDebugVAR;
extern uint8 img_prepared;
extern uint32 tickCnt;

#endif /* CODE_CARSTATE_H_ */
