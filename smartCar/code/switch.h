/*
 * switch.h
 *
 *  Created on: 2020年11月18日
 *      Author: Samurai
 */

#ifndef CODE_SWITCH_H_
#define CODE_SWITCH_H_
#include "zf_common_headfile.h"
// 小主板
// #define  GET_SWITCH8()  !gpio_get(P33_9)
// #define  GET_SWITCH7()  !gpio_get(P33_10)
// #define  GET_SWITCH6()  !gpio_get(P33_11)
// #define  GET_SWITCH5()  !gpio_get(P33_12)
// #define  GET_SWITCH4()  !gpio_get(P33_13)
// #define  GET_SWITCH3()  !gpio_get(P32_4)
// #define  GET_SWITCH2()  !gpio_get(P23_1)
// #define  GET_SWITCH1()  !gpio_get(P22_0)

#define GET_SWITCH8() !gpio_get(P13_0)
#define GET_SWITCH7() !gpio_get(P13_2)
#define GET_SWITCH6() !gpio_get(P11_2)
#define GET_SWITCH5() !gpio_get(P13_1)
#define GET_SWITCH4() !gpio_get(P02_6)
#define GET_SWITCH3() !gpio_get(P02_5)
#define GET_SWITCH2() !gpio_get(P10_2)
#define GET_SWITCH1() !gpio_get(P02_4)
// 开关端口的枚举
typedef enum
{
  SWITCH_1,
  SWITCH_2,
  SWITCH_3,
  SWITCH_4,
  SWITCH_5,
  SWITCH_6,
  SWITCH_7,
  SWITCH_8,

  SWITCH_MAX,
} SWITCH_e;

typedef enum
{
  SWITCH_ON = 0,  // 按键按下时对应电平
  SWITCH_OFF = 1, // 按键弹起时对应电平
} SWITCH_STATUS_e;

typedef enum
{
  VHIGH = 0,
  VMEDIUM,
  VLOW,

} VOICE_TYPE_e;

/*
 *  供外部调用的函数接口声明
 */
extern void switch_init();
extern SWITCH_STATUS_e Switch_Get(SWITCH_e key);

#endif /* CODE_SWITCH_H_ */
