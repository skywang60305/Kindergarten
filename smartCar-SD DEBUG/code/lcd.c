/*
 * lcd.c
 *
 *  Created on: 2020年11月19日
 *      Author: Samurai
 */

#include "lcd.h"

uint32 lcd_flag = 0;        //为 1 时开始运行lcd回调函数
volatile LCD_SHOW_MODE LCD_STATE=LCD_EVENT1;
//清屏幕用的  全局的static只有该文件能用
static uint8 keyCallBackClearFlag[2]={0};
static uint8 LcdClearFlag[2]={0};

/*
 * LCD显示函数，BOOM5回调函数版本
 */
void displayLCD_callBackVersion()
{
    LcdClearFlag[1]=LcdClearFlag[0];
    if(LCD_STATE==LCD_EVENT1)
        callBackDisplayLCD(displayLCD_Event1);
    else if(LCD_STATE==LCD_EVENT2)
        callBackDisplayLCD(displayLCD_Event2);
    else if(LCD_STATE==LCD_EVENT3)
        callBackDisplayLCD(displayLCD_Event3);
    else if(LCD_STATE==LCD_EVENT4)
        callBackDisplayLCD(displayLCD_Event4);
    else if(LCD_STATE==LCD_EVENT5)
        callBackDisplayLCD(displayLCD_Event5);
}

//回调函数
void callBackDisplayLCD(void (*event)(void))
{
    //清屏
    LCD_STATE=LCD_MAX;
    LcdClearFlag[0]=LCD_STATE;
    if(LcdClearFlag[0]!=LcdClearFlag[1])
        tft180_clear();
    //执行回调函数
    event();
}

//使用有参数的回调函数时,回调注册函数中必须也加入参数,注意回调函数只适用于同类型的操作
//回调注册函数
void keyEventOfMyCallBack(void (*event)(void))
{
    //执行共有的操作
    keyCallBackClearFlag[1]=keyCallBackClearFlag[0];
    keyCallBackClearFlag[0]=keymsg.key;
    if(keyCallBackClearFlag[0]!=keyCallBackClearFlag[1])
       tft180_clear();
    //执行各自的操作
    event();
}

void displayLCD_Event1()
{
    if(lcd_flag == 1)
    {
        switch (keymsg.key)
    {
//        case KEY_B:
//            keyEventOfMyCallBack(keyB_Event1);
//            break;
        case KEY_L:
            keyEventOfMyCallBack(keyL_Event1);
            break;
        case KEY_R:
            keyEventOfMyCallBack(keyR_Event1);
            break;
        case KEY_U:
            keyEventOfMyCallBack(keyU_Event1);
            break;
        case KEY_D:
            keyEventOfMyCallBack(keyD_Event1);
            break;
        default:
            keyEventOfMyCallBack(keyB_Event1);
            break;
    }
    }
}

void displayLCD_Event2()
{
    switch (keymsg.key)
    {
//        case KEY_B:
//            keyEventOfMyCallBack(keyB_Event2);
//            break;
        case KEY_L:
            keyEventOfMyCallBack(keyL_Event2);
            break;
        case KEY_R:
            keyEventOfMyCallBack(keyR_Event2);
            break;
        case KEY_U:
            keyEventOfMyCallBack(keyU_Event2);
            break;
        case KEY_D:
            keyEventOfMyCallBack(keyD_Event2);
            break;
        default:
            keyEventOfMyCallBack(keyB_Event2);
            break;
    }
}


void displayLCD_Event3()
{
    switch (keymsg.key)
    {
//        case KEY_B:
//            keyEventOfMyCallBack(keyB_Event3);
//            break;
        case KEY_L:
            keyEventOfMyCallBack(keyL_Event3);
            break;
        case KEY_R:
            keyEventOfMyCallBack(keyR_Event3);
            break;
        case KEY_U:
            keyEventOfMyCallBack(keyU_Event3);
            break;
        case KEY_D:
            keyEventOfMyCallBack(keyD_Event3);
            break;
        default:
            keyEventOfMyCallBack(keyB_Event3);
            break;
    }
}

void displayLCD_Event4()
{
    switch (keymsg.key)
    {
//        case KEY_B:
//            keyEventOfMyCallBack(keyB_Event4);
//            break;
        case KEY_L:
            keyEventOfMyCallBack(keyL_Event4);
            break;
        case KEY_R:
            keyEventOfMyCallBack(keyR_Event4);
            break;
        case KEY_U:
            keyEventOfMyCallBack(keyU_Event4);
            break;
        case KEY_D:
            keyEventOfMyCallBack(keyD_Event4);
            break;
        default:
            keyEventOfMyCallBack(keyB_Event4);
            break;
    }
}

void displayLCD_Event5()
{
    switch (keymsg.key)
    {
//        case KEY_B:
//            keyEventOfMyCallBack(keyB_Event1);
//            break;
        case KEY_L:
            keyEventOfMyCallBack(keyL_Event1);
            break;
        case KEY_R:
            keyEventOfMyCallBack(keyR_Event5);
            break;
        case KEY_U:
            keyEventOfMyCallBack(keyB_Event5);
            break;
        case KEY_D:
            keyEventOfMyCallBack(keyB_Event5);
            break;
        default:
            keyEventOfMyCallBack(keyB_Event5);
            break;
    }
}


/**********************************显示页面1*************************************/
void keyB_Event1()
{
    myShowFunc1();
}

void keyL_Event1()
{
    myShowFunc2();
}

void keyU_Event1()
{
}

void keyR_Event1()
{
}

void keyD_Event1()
{
}

/**********************************显示页面2*************************************/
void keyB_Event2()
{
    myShowFunc3();
}

void keyL_Event2()
{
}

void keyU_Event2()
{
}

void keyR_Event2()
{
}

void keyD_Event2()
{
}

/**********************************显示页面3*************************************/
void keyB_Event3()
{
}

void keyL_Event3()
{
}

void keyU_Event3()
{
}

void keyR_Event3()
{
}

void keyD_Event3()
{
}

/**********************************显示页面4*************************************/
void keyB_Event4()
{
}

void keyL_Event4()
{
}

void keyU_Event4()
{
}

void keyR_Event4()
{
}

void keyD_Event4()
{
}

/**********************************显示页面5*************************************/
void keyB_Event5()
{
}
void keyL_Event5()
{
}

void keyU_Event5()
{
}

void keyR_Event5()
{
}

void keyD_Event5()
{
}

///////////////////////我是分界线，在下面写自己的lcd显示函数/////////////////////////////////////////
void myShowFunc1()
{
    tft180_show_string(50,2,"myShowFunc1");
}

void myShowFunc2()
{
    tft180_show_string(50,2,"myShowFunc2");
}

void myShowFunc3()
{
    tft180_show_string(50,2,"myShowFunc3");
}
