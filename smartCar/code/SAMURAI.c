/*
 * SAMURAI.c
 *
 *  Created on: 2020��12��17��
 *      Author: Samurai
 */

#include "SAMURAI.h"
// extern int16 MT9V03X_CFG[CONFIG_FINISH][2];
// extern uint32 runTimeTemp;
//
//
//
// volatile RunState runState = STOP;
// volatile StopReason stopReason;
// volatile RunMode runMode = NORMAL_RUN;
// volatile RunSpeedMode runSpeedMode = CONSTANT_SPEED;
// CAMERA_DEBUG_VAR cameraDebugVAR;
//
// ProgramRunTime programRunTime;
// uint32 runTimeTemp=0;
// CoreStatus cpu0 = WORKING;
// CoreStatus cpu1 = WAITING;
// uint8 call_bmx055_flag = 0;
// uint16 sdSaveCnt=0;                //  SD��Ĵ���
// uint16 over3msTimeCnt=0;
// uint16 over5msTimeCnt=0;
// uint16 over8msTimeCnt=0;
//
// uint16 programCnt=0;               //  ����while�Ĵ���
// uint16 programOver18Cnt=0;         //  ���򳬹�18MS�Ĵ���
// float sdStability=0;                 //  SD�ȶ���
// float programStability=0;            //  �����ȶ���
// uint16 maxAllTime=0;               //  ���й����е����whileʱ��
// uint8 img_prepared = 0;
// uint32 tickCnt;
// void init_all()
//{
//     /*******������ʼ��*********/
//
//     uart_init(UART_3, 115200, UART3_TX_P15_6, UART3_RX_P15_7);
//     gpio_init(BEEP_ENABLE, GPO, 0, PUSHPULL);
//     NVIC_init();
//     lcd_init();
//     lcd_showstr(0,0,"LCD_INIT");
//     switch_init();
//
//     speed_init();
//     lcd_showstr(0,7,"MOTOR_INIT");
//     lcd_showstr(0,1,"SWITCH_INIT");
//     time_init();
//     lcd_showstr(0,2,"TIME_INIT");
//     key_init(KEY_MAX);
//     lcd_showstr(0,3,"KEY_INIT");
//     /*********������***********/
//     simiic_init();
//     while(!BMX055_init());
//     GyroOffset_init();
//     lcd_showstr(0,4,"BMX055_INIT");
//     if(GET_SWITCH4())
//     {
//         while(!sd_init());
//         lcd_showstr(0,5,"SD CARD INITED");
//     }
//     else
//     {
//         lcd_showstr(0,5,"NO SD CARD");
//     }
//
//     /***********���������**********/
//     servo_init();
//     lcd_showstr(0,6,"SERVO_INIT");
//
//     mt9v03x_init();
//     lcd_showstr(80,0,"CAMERA_INIT");
//
//     cameraDebugVarInit();
//     standard();
//     enableInterrupts();
//     MainMenu_Set();
//     cameraVarInitFromEeprom();
//     mt9v03x_init();
//     wdog_init_ms(500);
//     wdog_enable();
// }
//
// void time_init()
//{
//     PRT.maxWhileTime = 0;
//     PRT.aveWhileTime = 0;
//     PRT.whileTime = 0;
//     PRT._whileTime = 0;
//     PRT.maxGoTime = 0;
//     PRT.aveGoTime = 0;
//     PRT.goTime = 0;
//     PRT.allTime=0;
//     PRT.sdSaveTime=0;
//     PRT.vcanTime=0;
// }
//
// void waitFinishGetImg()
//{
//     while(!mt9v03x_finish_flag)
//     {
//
//     }
// }
//
// void startGetImg()
//{
//     mt9v03x_finish_flag = 0;
// }
// void NVIC_init()
//{
//     pit_interrupt_ms(CCU6_0, PIT_CH0, 4);
//     pit_interrupt_ms(CCU6_0, PIT_CH1, 2);
//     pit_interrupt_us(CCU6_1, PIT_CH0, 10);
//     enableInterrupts();
// }
//
// void dealTime()
//{
//     if (PRT.whileTime > PRT.maxWhileTime)
//     {
//         PRT.maxWhileTime = PRT.whileTime;
//     }
//     if (PRT.goTime > PRT.maxGoTime)
//     {
//         PRT.maxGoTime = PRT.goTime;
//     }
// }
//
//
// void cameraDebugVarInit()
//{
//     cameraDebugVAR.AUTO_EXP_TEMP=0;
//     cameraDebugVAR.EXP_TIME_TEMP=450;
//     cameraDebugVAR.FPS_TEMP=100;
//     cameraDebugVAR.GAIN_TEMP=32;
// }
//
//
// void cameraVarInitFromEeprom()
//{
//     //�Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
//     //һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
//     MT9V03X_CFG[0][1]=(int16)cameraDebugVAR.AUTO_EXP_TEMP;
//
//     //�ع�ʱ��   ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
//     MT9V03X_CFG[1][1]=(int16)cameraDebugVAR.EXP_TIME_TEMP;
//
//     //ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
//     MT9V03X_CFG[2][1]=(int16)cameraDebugVAR.FPS_TEMP;
//
//     //ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�
//     MT9V03X_CFG[7][1]=(int16)cameraDebugVAR.GAIN_TEMP;
//
// }
//
// void myMemcpy(uint8 *distBuff,uint8 *regionBuff,uint32 size)
//{
//     for(uint32 i=0;i<size;i++)
//     {
//         distBuff[i]=regionBuff[i];
//     }
// }
//
//
// void mySystickDelay_us(uint32 time_us)
//{
//     tickCnt = 0;
//     uint8 a;
//     while(tickCnt<time_us)
//     {
//         a+=1; //��ֹ����
//     }
// }
