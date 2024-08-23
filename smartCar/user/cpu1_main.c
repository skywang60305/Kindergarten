/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��
//  **************************** �������� ****************************
void init_all()
{
    /*******������ʼ��*********/
    //    uart_init(UART_3, 115200, UART3_TX_P15_6, UART3_RX_P15_7);
    gpio_init(BEEP_ENABLE, GPO, 0, GPO_PUSH_PULL);
    // �ж϶�ʱ������
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
    /*********������***********/
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

    /***********���������**********/
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
    // �û��ڴ˴����ø��ֳ�ʼ��������

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
            programStability = 1.0 - ((float)(programOver18Cnt)) / 1.0 / ((float)(programCnt)); //  �����ȶ���
            sdStability = 1.0 - ((float)(over5msTimeCnt)) / 1.0 / ((float)(sdSaveCnt));         //  SD�ȶ���
            displayLCD_callBackVersion();                                                       // �ص������汾
        }
        if (LCD_STATE != LCD_SHOW_STOP_INFORM) // 1.24  ��ͣ��ʱ�����ټ�������ȶ���
            if (maxAllTime < PRT._whileTime)   // �ܵĹ�����whileһ�����ʱ��
                maxAllTime = PRT._whileTime;
        wdog_feed();
        showbasemap();
    }
}

#pragma section all restore
