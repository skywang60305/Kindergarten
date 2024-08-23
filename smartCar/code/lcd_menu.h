/*
 * lcd_menu.h
 *
 *  Created on: 2021��8��11��
 *      Author: 95159
 */

#ifndef CODE_LCD_MENU_H_
#define CODE_LCD_MENU_H_

#include "zf_common_headfile.h"

/******************************* �˵��ṹ�嶨�� *******************************/
typedef struct
{

    uint8 ExitMark; // �˳��˵�(0-���˳���1-�˳�)

    uint8 Cursor; // ���ֵ(��ǰ���λ��)

    uint8 PageNo; // �˵�ҳ(��ʾ��ʼ��)

    uint8 Index; // �˵�����(��ǰѡ��Ĳ˵���)

    uint8 DispNum; // ��ʾ����(ÿҳ�������ڲ˵���)

    uint8 MaxPage; // ���ҳ��(����ж�������ʾҳ)

} MENU_PRMT; // �˵�����

typedef struct
{

    uint8 *MenuName; // �˵���Ŀ����

    void (*ItemHook)(void); // Ҫ���еĲ˵�����

    uint16 *DebugParam; // Ҫ���ԵĲ���

} MENU_TABLE; // �˵�ִ��
// �����˿ڵ�ö��

typedef struct
{

    uint8 x;
    uint8 y;

} Site_t;

void Menu_RunMode(void);
void Menu_NORMAL_RUN(void);
void Menu_TIMING_RUN(void);
void Menu_CONSTANT_SPEED(void);
void Menu_VARIABLE_SPEED(void);
void Menu_NORMAL_SHIFT_SPEED(void);

void Menu_Servo(void);
void Menu_SERVO_PWM_MID(void);
void Menu_SERVO_PWM_MIN(void);
void Menu_SERVO_PWM_MAX(void);

void Menu_Element(void);

void Menu_Garage(void);
void Menu_UCross();
void Menu_Annulus(void);
void Menu_Ann50();
void Menu_Ann60();
void Menu_Ann70();
void Menu_Ann80();
void Menu_Ann90();
void Menu_Ann100();
void Menu_AnnulusSize(void);
void chooseAnnulusSize(void);
void annulusParamInitFromEeprom();

void Menu_Fork(void);

void Menu_Ramp(void);
void Menu_GentleRamp(void);
void Menu_BigRamp(void);
void Menu_MidRamp(void);
void Menu_SmallRamp(void);
void Menu_RampSize(void);
void chooseRampSize(void);
void rampParamInitFromEeprom(void);

void Menu_SpeedPara(void);
void Menu_TrackDetection(void);

void Menu_Motor(void);
void Menu_motorControlAlgorithm(void);
void Menu_visualScope(void);
void Menu_CHANGE_AIMSPEED_LEFT(void);
void Menu_CHANGE_AIMSPEED_RIGHT(void);
void Menu_motorParam(void);

void Menu_Sun(void);

void Menu_Camera(void);

void Menu_sdCardParam(void);
void Menu_Sd(void);
void Menu_allmapMode(void);
void Menu_imgGrayMode(void);

void Menu_ReadFlash(void);
void readFlash(uint8 flashNum);
void Menu_ReadFlash_1(void);
void Menu_ReadFlash_2(void);
void Menu_ReadFlash_3(void);
void Menu_ReadFlash_4(void);
void Menu_ReadFlash_5(void);

void Menu_WriteFlash(void);
void writeFlash(uint8 flashNum);
void Menu_WriteFlash_1(void);
void Menu_WriteFlash_2(void);
void Menu_WriteFlash_3(void);
void Menu_WriteFlash_4(void);
void Menu_WriteFlash_5(void);

void MainMenu_Set(void);
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key);
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page);
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
KEY_e KeySan(void);
KEY_e KeySan_motor(void);
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key);
void adjustParam(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor);
void Menu_Null(void);
void Write_EEPROM(void);
void Read_EEPROM(void);

extern uint32 visualScope_enable;
extern uint32 aimSpeedL_visualScope;
extern uint32 aimSpeedR_visualScope;
#endif /* CODE_LCD_MENU_H_ */
