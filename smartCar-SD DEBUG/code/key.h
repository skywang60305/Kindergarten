/*
 * key.h
 *
 *  Created on: 2020年11月19日
 *      Author: Samurai
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

//#include "zf_common_headfile.h"
#include "zf_device_key.h"
#include "zf_driver_delay.h"
#include "zf_device_tft180.h"
//下面是定义按键的时间，单位为 ： 10ms（中断时间）
#define KEY_DOWN_TIME           10       //消抖确认按下时间
#define KEY_HOLD_TIME           50      //长按hold确认时间，最多253，否则需要修改 keytime 的类型
                                        //如果按键一直按下去，则每隔 KEY_HOLD_TIME - KEY_DOWN_TIME 时间会发送一个 KEY_HOLD 消息

//定义按键消息FIFO大小
#define KEY_MSG_FIFO_SIZE       20      //最多 255，否则需要修改key_msg_front/key_msg_rear类型

//按键端口的枚举
typedef enum
{
    KEY_U,  //上P20_6
    KEY_D,  //下P20_7

    KEY_L,  //左P11_2
    KEY_R,  //右P11_3

    KEY_RUN,  //开始

    KEY_STOP,   //停止

    KEY_MAX,
} KEY_e;


//key状态枚举定义
typedef enum
{
    KEY_DOWN  =   0,         //按键按下时对应电平
    KEY_UP    =   1,         //按键弹起时对应电平

    KEY_HOLD,               //长按按键(用于定时按键扫描)

} KEY_STATUS_e;


//按键消息结构体
typedef struct
{
    KEY_e           key;        //按键编号
    KEY_STATUS_e    status;     //按键状态
} KEY_MSG_t;



KEY_STATUS_e    key_get(KEY_e key);             //检测key状态（不带延时消抖）
KEY_STATUS_e    key_check(KEY_e key);           //检测key状态（带延时消抖）

//定时扫描按键
void    KEY_init(KEY_e key);
uint8   get_key_msg(KEY_MSG_t *keymsg);         //获取按键消息，返回1表示有按键消息，0为无按键消息
void    key_IRQHandler(void);                   //需要定时扫描的中断服务函数（定时时间为10ms）
void    key_check_m(KEY_MSG_t keymsg);


extern volatile KEY_MSG_t keymsg;
#endif /* CODE_KEY_H_ */
