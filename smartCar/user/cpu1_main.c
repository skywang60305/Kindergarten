/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中
//  **************************** 代码区域 ****************************
void init_all()
{
    /*******基础初始化*********/
    //    uart_init(UART_3, 115200, UART3_TX_P15_6, UART3_RX_P15_7);
    gpio_init(BEEP_ENABLE, GPO, 0, GPO_PUSH_PULL);
    // 中断定时器设置
    pit_ms_init(CCU60_CH0, 4);
    pit_ms_init(CCU60_CH1, 2);
    pit_ms_init(CCU61_CH0, 10);
    tft180_init();
    lcd_showstr(0, 0, "LCD_INIT");
    switch_init();

    speed_init();
    lcd_showstr(0, 7, "MOTOR_INIT");
    lcd_showstr(0, 1, "SWITCH_INIT");
    time_init();
    lcd_showstr(0, 2, "TIME_INIT");
    key_init(KEY_MAX);
    lcd_showstr(0, 3, "KEY_INIT");
    /*********陀螺仪***********/
    //    simiic_init();
    //    while(!BMX055_init());
    //    GyroOffset_init();
    //    lcd_showstr(0,4,"BMX055_INIT");
    if (GET_SWITCH4())
    {
        while (!sd_init())
            ;
        lcd_showstr(0, 5, "SD CARD INITED");
    }
    else
    {
        lcd_showstr(0, 5, "NO SD CARD");
    }

    /***********电机编码器**********/
    servo_init();
    lcd_showstr(0, 6, "SERVO_INIT");
    system_delay_ms(500);
    mt9v03x_init();
    lcd_showstr(0, 0, "CAMERA_INIT");

    cameraDebugVarInit();
    standard();
    enableInterrupts();

    MainMenu_Set();
    cameraVarInitFromEeprom();
    mt9v03x_init();
    //    wdog_init_ms(500);
    //    wdog_enable();
}
void waitFinishGetImg()
{
    while (!mt9v03x_finish_flag)
    {
    }
}

void startGetImg()
{
    mt9v03x_finish_flag = 0;
}

void core1_main(void)
{
    disableInterrupts();
    // 用户在此处调用各种初始化函数等

    motor_init();
    init_all();
    enableInterrupts();
    //pwm_set_duty(ATOM0_CH2_P21_4, 8000);

    while (TRUE)
    {
        system_start();
        programCnt++;
        waitFinishGetImg();
        memcpy(imgGray[0], mt9v03x_image[0], sizeof(mt9v03x_image));
        startGetImg();
        PRT.goTime = (uint32)system_getval_us();
        go();
        PRT.goTime = (uint32)system_getval_us() - PRT.goTime;
        SI.aimSpeed = getAimSpeed();
        img_prepared = 1;
        PRT.whileTime = (uint32)system_getval_us();

        PRT._whileTime = (uint32)system_getval_us();
        if (GET_SWITCH2() || LCD_STATE == LCD_SHOW_STOP_INFORM)
        {
            programStability = 1.0 - ((float)(programOver18Cnt)) / 1.0 / ((float)(programCnt)); //  程序稳定性
            sdStability = 1.0 - ((float)(over5msTimeCnt)) / 1.0 / ((float)(sdSaveCnt));         //  SD稳定性
            displayLCD_callBackVersion();                                                       // 回调函数版本
        }
        if (LCD_STATE != LCD_SHOW_STOP_INFORM) // 1.24  当停车时，不再计算程序稳定性
            if (maxAllTime < PRT._whileTime)   // 跑的过程中while一次最大时间
                maxAllTime = PRT._whileTime;
        wdog_feed();
        showbasemap();
    }
}

#pragma section all restore
