/*
 * key.h
 *
 *  Created on: 2020��11��19��
 *      Author: Samurai
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

//#include "zf_common_headfile.h"
#include "zf_device_key.h"
#include "zf_driver_delay.h"
#include "zf_device_tft180.h"
//�����Ƕ��尴����ʱ�䣬��λΪ �� 10ms���ж�ʱ�䣩
#define KEY_DOWN_TIME           10       //����ȷ�ϰ���ʱ��
#define KEY_HOLD_TIME           50      //����holdȷ��ʱ�䣬���253��������Ҫ�޸� keytime ������
                                        //�������һֱ����ȥ����ÿ�� KEY_HOLD_TIME - KEY_DOWN_TIME ʱ��ᷢ��һ�� KEY_HOLD ��Ϣ

//���尴����ϢFIFO��С
#define KEY_MSG_FIFO_SIZE       20      //��� 255��������Ҫ�޸�key_msg_front/key_msg_rear����

//�����˿ڵ�ö��
typedef enum
{
    KEY_U,  //��P20_6
    KEY_D,  //��P20_7

    KEY_L,  //��P11_2
    KEY_R,  //��P11_3

    KEY_RUN,  //��ʼ

    KEY_STOP,   //ֹͣ

    KEY_MAX,
} KEY_e;


//key״̬ö�ٶ���
typedef enum
{
    KEY_DOWN  =   0,         //��������ʱ��Ӧ��ƽ
    KEY_UP    =   1,         //��������ʱ��Ӧ��ƽ

    KEY_HOLD,               //��������(���ڶ�ʱ����ɨ��)

} KEY_STATUS_e;


//������Ϣ�ṹ��
typedef struct
{
    KEY_e           key;        //�������
    KEY_STATUS_e    status;     //����״̬
} KEY_MSG_t;



KEY_STATUS_e    key_get(KEY_e key);             //���key״̬��������ʱ������
KEY_STATUS_e    key_check(KEY_e key);           //���key״̬������ʱ������

//��ʱɨ�谴��
void    KEY_init(KEY_e key);
uint8   get_key_msg(KEY_MSG_t *keymsg);         //��ȡ������Ϣ������1��ʾ�а�����Ϣ��0Ϊ�ް�����Ϣ
void    key_IRQHandler(void);                   //��Ҫ��ʱɨ����жϷ���������ʱʱ��Ϊ10ms��
void    key_check_m(KEY_MSG_t keymsg);


extern volatile KEY_MSG_t keymsg;
#endif /* CODE_KEY_H_ */
