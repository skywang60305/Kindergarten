/*
 * lcd_menu.h
 *
 *  Created on: 2021年8月11日
 *      Author: 95159
 */

#ifndef CODE_LCD_MENU_H_
#define CODE_LCD_MENU_H_

#include "zf_common_headfile.h"

/******************************* 菜单结构体定义 *******************************/
typedef struct
{

    uint8 ExitMark; // 退出菜单(0-不退出，1-退出)

    uint8 Cursor; // 光标值(当前光标位置)

    uint8 PageNo; // 菜单页(显示开始项)

    uint8 Index; // 菜单索引(当前选择的菜单项)

    uint8 DispNum; // 显示项数(每页可以现在菜单项)

    uint8 MaxPage; // 最大页数(最大有多少种显示页)

} MENU_PRMT; // 菜单参数

typedef struct
{

    uint8 *MenuName; // 菜单项目名称

    void (*ItemHook)(void); // 要运行的菜单函数

    uint16 *DebugParam; // 要调试的参数

} MENU_TABLE; // 菜单执行
// 按键端口的枚举

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
