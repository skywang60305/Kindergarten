/*
 * SAMURAI.c
 *
 *  Created on: 2020年12月17日
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
// uint16 sdSaveCnt=0;                //  SD存的次数
// uint16 over3msTimeCnt=0;
// uint16 over5msTimeCnt=0;
// uint16 over8msTimeCnt=0;
//
// uint16 programCnt=0;               //  程序while的次数
// uint16 programOver18Cnt=0;         //  程序超过18MS的次数
// float sdStability=0;                 //  SD稳定度
// float programStability=0;            //  程序稳定度
// uint16 maxAllTime=0;               //  运行过程中的最大while时间
// uint8 img_prepared = 0;
// uint32 tickCnt;
// void init_all()
//{
//     /*******基础初始化*********/
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
//     /*********陀螺仪***********/
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
//     /***********电机编码器**********/
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
//     //自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
//     //一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
//     MT9V03X_CFG[0][1]=(int16)cameraDebugVAR.AUTO_EXP_TEMP;
//
//     //曝光时间   摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
//     MT9V03X_CFG[1][1]=(int16)cameraDebugVAR.EXP_TIME_TEMP;
//
//     //图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
//     MT9V03X_CFG[2][1]=(int16)cameraDebugVAR.FPS_TEMP;
//
//     //图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
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
//         a+=1; //防止卡死
//     }
// }
