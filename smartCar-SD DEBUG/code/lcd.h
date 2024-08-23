/*
 * lcd.h
 *
 *  Created on: 2020年11月19日
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


/*BOOM5的LCD显示状态机已经是最简捷明了的显示方式了，以后就用这个就好*/
/*用法:想在哪里显示,就往对应的函数中放入要执行的显示函数,然后在头文件里对应的写好注释*/
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

/*上面的函数不要动,在下面写自己的显示函数放入上面的框架,25个空基本用不完的*/

void myShowFunc1(void);
void myShowFunc2(void);
void myShowFunc3(void);

extern volatile LCD_SHOW_MODE LCD_STATE;
#endif /* CODE_LCD_H_ */
