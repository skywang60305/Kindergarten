/*
 * lcd.h
 *
 *  Created on: 2020��11��19��
 *      Author: Samurai
 */

#ifndef CODE_LCD_H_
#define CODE_LCD_H_

#include "zf_common_headfile.h"

typedef enum
{
    LCD_EVENT1          =0,
    LCD_EVENT2          =1,
    LCD_EVENT3          =2,
    LCD_EVENT4          =3,
    LCD_EVENT5          =4,
    LCD_MAX             =5,
}LCD_SHOW_MODE;


/*BOOM5��LCD��ʾ״̬���Ѿ����������˵���ʾ��ʽ�ˣ��Ժ��������ͺ�*/
/*�÷�:����������ʾ,������Ӧ�ĺ����з���Ҫִ�е���ʾ����,Ȼ����ͷ�ļ����Ӧ��д��ע��*/
void displayLCD_callBackVersion(void);
void callBackDisplayLCD( void (*event)() );
void keyEventOfMyCallBack(void (*event)());

void displayLCD_Event1(void);
void displayLCD_Event2(void);
void displayLCD_Event3(void);
void displayLCD_Event4(void);
void displayLCD_Event5(void);

void keyB_Event1(void);
void keyL_Event1(void);
void keyU_Event1(void);
void keyR_Event1(void);
void keyD_Event1(void);

void keyB_Event2(void);
void keyL_Event2(void);
void keyU_Event2(void);
void keyR_Event2(void);
void keyD_Event2(void);

void keyB_Event3(void);
void keyL_Event3(void);
void keyU_Event3(void);
void keyR_Event3(void);
void keyD_Event3(void);

void keyB_Event4(void);
void keyL_Event4(void);
void keyU_Event4(void);
void keyR_Event4(void);
void keyD_Event4(void);

void keyB_Event5(void);
void keyL_Event5(void);
void keyR_Event5(void);
void keyU_Event5(void);
void keyD_Event5(void);

/*����ĺ�����Ҫ��,������д�Լ�����ʾ������������Ŀ��,25���ջ����ò����*/

void myShowFunc1(void);
void myShowFunc2(void);
void myShowFunc3(void);

extern volatile LCD_SHOW_MODE LCD_STATE;
#endif /* CODE_LCD_H_ */
