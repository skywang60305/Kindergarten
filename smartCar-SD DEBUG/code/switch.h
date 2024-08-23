/*
 * switch.h
 *
 *  Created on: 2020年11月18日
 *      Author: Samurai
 */

#ifndef CODE_SWITCH_H_
#define CODE_SWITCH_H_
#include "zf_common_headfile.h"

#define  GET_SWITCH1()  !gpio_get_level(P33_11)
#define  GET_SWITCH2()  !gpio_get_level(P33_12)

//开关端口的枚举
typedef enum
{
    SWITCH_1,
    SWITCH_2,
    SWITCH_MAX,
} SWITCH_e;

typedef enum
{
    SWITCH_ON = 0,         //拨码开启时对应电平
    SWITCH_OFF = 1,        //拨码关闭时对应电平
} SWITCH_STATUS_e;

typedef enum
{
  VHIGH = 0,
  VMEDIUM,
  VLOW,
}VOICE_TYPE_e;

/*
 *  供外部调用的函数接口声明
 */
void              switch_init();
SWITCH_STATUS_e   Switch_Get(SWITCH_e key);


#endif /* CODE_SWITCH_H_ */
