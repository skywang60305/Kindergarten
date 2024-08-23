/*
 * switch.h
 *
 *  Created on: 2020��11��18��
 *      Author: Samurai
 */

#ifndef CODE_SWITCH_H_
#define CODE_SWITCH_H_
#include "zf_common_headfile.h"

#define  GET_SWITCH1()  !gpio_get_level(P33_11)
#define  GET_SWITCH2()  !gpio_get_level(P33_12)

//���ض˿ڵ�ö��
typedef enum
{
    SWITCH_1,
    SWITCH_2,
    SWITCH_MAX,
} SWITCH_e;

typedef enum
{
    SWITCH_ON = 0,         //���뿪��ʱ��Ӧ��ƽ
    SWITCH_OFF = 1,        //����ر�ʱ��Ӧ��ƽ
} SWITCH_STATUS_e;

typedef enum
{
  VHIGH = 0,
  VMEDIUM,
  VLOW,
}VOICE_TYPE_e;

/*
 *  ���ⲿ���õĺ����ӿ�����
 */
void              switch_init();
SWITCH_STATUS_e   Switch_Get(SWITCH_e key);


#endif /* CODE_SWITCH_H_ */
