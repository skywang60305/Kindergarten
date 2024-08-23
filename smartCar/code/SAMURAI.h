/*
 * SAUMRAI.h
 *
 *  Created on: 2020年11月18日
 *      Author: Samurai
 */

#ifndef CODE_SAMURAI_H_
#define CODE_SAMURAI_H_
#include "zf_common_headfile.h"

// #define programRunTime PRT
// #define SW_UP       1-gpio_get(P11_2)
// #define SW_DOWN     1-gpio_get(P11_9)
// #define SW_LEFT     1-gpio_get(P13_3)
// #define SW_RIGHT    1-gpio_get(P11_3)
// #define SW_OK       1-gpio_get(P11_6)
//
// #define BUZZER_ON   gpio_set(BEEP_ENABLE, 1)
// #define BUZZER_OFF  gpio_set(BEEP_ENABLE, 0)
//
// #def ine BEEP(i)         \
//do {                    \
//    BUZZER_ON;          \
//    buzzerTime = i / 10;\
//} while (0);
//
// typedef enum RunState
//{
//     STOP,
//     READY,
//     RUNNING,
//     BRAKING,
// } RunState;
//
//
///* 小车运行模式枚举类型 */
// typedef enum RunMode
//{
//     NORMAL_RUN,
//     TIMING_RUN,
// } RunMode;
//
// typedef enum DevMode
//{
//     OLD_MODE,
//     NEW_MODE,
// }DevMode;
//
// typedef enum RunSpeedMode
//{
//     VARIABLE_SPEED,
//     CONSTANT_SPEED,
//     NORMAL_SHIFT_SPEED,
//     ADC_SPEED,
// } RunSpeedMode;
//
// typedef struct ProgramRunTime
//{
//     uint16 whileTime;
//     uint16 _whileTime;
//     uint16 maxWhileTime;
//     uint16 aveWhileTime;
//     uint16 goTime;
//     uint16 maxGoTime;
//     uint16 aveGoTime;
//     uint16 allTime;
//     uint16 sdSaveTime;
//     uint16 vcanTime;
// } ProgramRunTime;
//
// typedef struct{
//     uint32 AUTO_EXP_TEMP;
//     uint32 EXP_TIME_TEMP;
//     uint32 FPS_TEMP;
//     uint32 GAIN_TEMP;
//     //uint32_t UD_OFFSET;    //向下偏移
// }CAMERA_DEBUG_VAR;
//
// typedef enum
//{
//     WAITING,
//     WORKING,
// }CoreStatus;
//
// #define  BEEP_ENABLE    P22_3//P33_8
// #define  LED3_on        gpio_init(P33_11, GPO, 0, PUSHPULL)
// #define  LED3_off       gpio_init(P33_11, GPO, 1, PUSHPULL)
// #define  LED3_toggle    gpio_toggle(P33_11)
// #define  LED4_on        gpio_init(P33_12, GPO, 0, PUSHPULL)
// #define  LED4_off       gpio_init(P33_12, GPO, 1, PUSHPULL)
//
// #define  LED_all_off    LED1_off,LED2_off,LED3_off,LED4_off
// #define  LED_all_on     LED1_on,LED2_on,LED3_on,LED4_on
//
// #define  ERROR1         while(1){LED1_on;}
// #define  ERROR2         while(1){LED2_on;}
// #define  ERROR3         while(1){LED3_on;}
// #define  ERROR4         while(1){LED4_on;}
//
// void init_all(void);
// void time_init(void);
// void waitFinishGetImg(void);
// void startGetImg(void);
// void NVIC_init(void);
// void PTS_init(void);
// void cameraVarInitFromEeprom(void);
// void push(uint8 x,uint8 y);
// Point pop(void);
// uint8 isEmpty(void);
// void cameraDebugVarInit(void);
// void myMemcpy(uint8 *distBuff,uint8 *regionBuff,uint32 size);
// void mySystickDelay_us(uint32 time_us);
//
// extern CoreStatus cpu0;
// extern CoreStatus cpu1;
// extern ProgramRunTime programRunTime;
// extern uint16 over3msTimeCnt;
// extern uint16 over5msTimeCnt;
// extern uint16 over8msTimeCnt;
// extern uint16 sdSaveCnt;                    //  SD存的次数
// extern uint16 programCnt;                    //  程序while的次数
// extern uint16 programOver18Cnt;              //  程序超过18MS的次数
// extern float sdStability;                    //  SD稳定度
// extern float programStability;               //  程序稳定度
// extern uint16 maxAllTime;                    //  运行过程中的最大while时间
// extern ProgramRunTime programRunTime;
// extern volatile RunState runState;
// extern volatile StopReason stopReason;
// extern volatile RunMode runMode;
// extern volatile RunSpeedMode runSpeedMode;
// extern uint8 img_prepared;
// extern uint32 tickCnt;
// extern CAMERA_DEBUG_VAR cameraDebugVAR;
#endif /* CODE_SAMURAI_H_ */
