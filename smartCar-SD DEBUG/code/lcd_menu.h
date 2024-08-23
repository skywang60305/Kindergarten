/*
 * lcd_menu.h
 *
 *  Created on: 2021年8月11日
 *      Author: 95159
 */

#ifndef CODE_LCD_MENU_H_
#define CODE_LCD_MENU_H_

#include "zf_common_headfile.h"
#include "key.h"
#include "switch.h"

//#include "zf_device_tft180.h"
//#include "zf_driver_delay.h"
//#include "zf_common_font.h"
//#include "zf_driver_flash.h"
//#include "zf_device_mt9v03x.h"
/******************************* 菜单结构体定义 *******************************/
typedef struct{

    uint8 ExitMark;     // 退出菜单(0-不退出，1-退出)

    uint8 Cursor;       // 光标值(当前光标位置)

    uint8 PageNo;       // 菜单页(显示开始项)

    uint8 Index;        // 菜单索引(当前选择的菜单项)

    uint8 DispNum;      // 显示项数(每页可以现在菜单项)

    uint8 MaxPage;      // 最大页数(最大有多少种显示页)

}MENU_PRMT;      // 菜单参数

typedef enum
{
    Main_Menu,   //主菜单
    Menu_1st,    //一级菜单
}Menu_Flag;


typedef struct{

    uint8 *MenuName;      // 菜单项目名称

    void(*ItemHook)(void);  // 要运行的菜单函数

    uint16 *DebugParam;     // 要调试的参数

}MENU_TABLE;     // 菜单执行
//按键端口的枚举

/*BEEP文件*/
#define BEEP_ENABLE P33_10
void beep_on();
void beep_off();
void beep_init();
void beep();
void BEEP_FLAG();


typedef struct{

    uint8 x;
    uint8 y;

}Site_t;

void Menu_RunMode(void);
void Menu_exp7(void);
void Menu_exp8(void);
void Menu_Camera(void);
void Menu_Image(void);
void image_show_gray(void);
void image_show_binary(void);
void image_show_all(void);
void Ostu_Compress(uint8 start_rows, uint8 end_rows);

void MainMenu_Set(void);
uint8 Menu_Move(MENU_PRMT *prmt,KEY_e key);
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page);
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
KEY_e KeySan(void);
uint8 Menu_Move(MENU_PRMT *prmt,KEY_e key);
void adjustParam(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor);
void Menu_Null(void);
void Write_EEPROM(void);
void Read_EEPROM(void);


#endif /* CODE_LCD_MENU_H_ */
