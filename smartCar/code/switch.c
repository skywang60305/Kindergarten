/*
 * switch.c
 *
 *  Created on: 2020��11��18��
 *      Author: Samurai
 */

#include "switch.h"

/* ����SWITCH��Ŷ�Ӧ�Ĺܽ� */
// С����
// PIN_enum SWITCH_PTxn[SWITCH_MAX] = {P33_9, P33_10, P33_11, P33_12, P33_13, P32_4, P23_1, P22_0};
gpio_pin_enum SWITCH_PTxn[SWITCH_MAX] = {P02_4, P10_2, P02_5, P02_6, P13_3, P10_2, P10_1, P10_3};

/******************************************************************************
 *  @brief      ��ʼ��switch�˿�
 *****************************************************************************/
void switch_init()
{
    uint8 i = SWITCH_MAX;

    // ��ʼ��ȫ�� ����
    while (i--)
    {
        gpio_init(SWITCH_PTxn[i], GPI, 0, GPO_PUSH_PULL);
    }
}

/******************************************************************************
 *  @brief      ��ȡswitch״̬
 *  @param      SWITCH_e         SWITCH���
 *  @return     SWITCH_STATUS_e    SWITCH״̬(SWITCH_ON,SWITCH_OFF)
 ******************************************************************************/
SWITCH_STATUS_e Switch_Get(SWITCH_e i)
{
    if (gpio_get(SWITCH_PTxn[i]) == SWITCH_OFF)
    {
        return SWITCH_OFF;
    }
    return SWITCH_ON;
}
