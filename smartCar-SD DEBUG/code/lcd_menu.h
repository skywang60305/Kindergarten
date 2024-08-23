/*
 * lcd_menu.h
 *
 *  Created on: 2021��8��11��
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
/******************************* �˵��ṹ�嶨�� *******************************/
typedef struct{

    uint8 ExitMark;     // �˳��˵�(0-���˳���1-�˳�)

    uint8 Cursor;       // ���ֵ(��ǰ���λ��)

    uint8 PageNo;       // �˵�ҳ(��ʾ��ʼ��)

    uint8 Index;        // �˵�����(��ǰѡ��Ĳ˵���)

    uint8 DispNum;      // ��ʾ����(ÿҳ�������ڲ˵���)

    uint8 MaxPage;      // ���ҳ��(����ж�������ʾҳ)

}MENU_PRMT;      // �˵�����

typedef enum
{
    Main_Menu,   //���˵�
    Menu_1st,    //һ���˵�
}Menu_Flag;


typedef struct{

    uint8 *MenuName;      // �˵���Ŀ����

    void(*ItemHook)(void);  // Ҫ���еĲ˵�����

    uint16 *DebugParam;     // Ҫ���ԵĲ���

}MENU_TABLE;     // �˵�ִ��
//�����˿ڵ�ö��

/*BEEP�ļ�*/
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
