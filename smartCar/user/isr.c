

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            isr
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        tasking v6.3r1
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
//��isr.c���жϺ�������������ĵڶ��������̶�Ϊ0���벻Ҫ���ģ���ʹ����CPU1�����ж�Ҳ��Ҫ���ģ���ҪCPU1�����ж�ֻ��Ҫ��isr_config.h���޸Ķ�Ӧ�ĺ궨�弴��
volatile uint32 runTime = 0;
volatile KEY_e last = 0;
volatile uint8 ExitMenu_flag=0;
volatile uint32 runningTime=0;
volatile uint8 runReadyFlag=0;
volatile int startCarTimeCnt=0;  //100*10=1s
//int16 buzzerTime = 0;
volatile uint16 TFMINI_PLUS_DISTANCE;
volatile uint16 TFMINI_PLUS_STRENGTH;
//PIT�жϺ���  ʾ��
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts();//�����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);
    /***************************************����ɨ��*************************************/
    //����ɨ��
    key_IRQHandler();         //�Ѱ�����Ϣ����FIFO
    while(get_key_msg(&keymsg)); //�յ�ʱ��ͻ�������
    //���������⴦��   ������Ҫ�� �������� ��Ȼ���ظ�ִ��
    if(keymsg.key==KEY_RUN && keymsg.status==KEY_DOWN )
    {
        runState=READY;     //��������ʼ����ɻ���run

        keymsg.status=KEY_UP;

    }
    else if(keymsg.key==KEY_STOP && keymsg.status==KEY_DOWN  )
    {
        if(!ExitMenu_flag)
        {
            ExitMenu_flag=1;
        }
        else
        {
            LCD_STATE=(LCD_SHOW_MODE)((LCD_STATE+1)%6);
        }
        runState=STOP;
        keymsg.status=KEY_UP;
    }
    else if(keymsg.status==KEY_DOWN)
    {
        MapClearFlag = 1;
        last = keymsg.key;
        keymsg.status=KEY_UP;
    }
    /********************�������**********************/
    if(runState==READY && 0==runReadyFlag)   //����������
    {
        startCarTimeCnt++;
        if(startCarTimeCnt>=65530)    //��ֹ���
        {
            startCarTimeCnt=65530;
        }

        if(startCarTimeCnt/2.5>=200)        //��ʱ����
        {
            runState = RUNNING;
            programCnt = 0;
            runReadyFlag=1;
            sector= SECTOR_START;
        }
    }
    else if(runState==STOP && 0==runReadyFlag)
    {
        //BEEP(200);
    }

    if(runState==RUNNING)
    {
        getEncoder();
        setMotorPWM();
        runningTime+=1;                 //  ������ʱ�������ߺͳ������⴦���,һ��ʱ�����ٶ�С��ĳֵʱb+20
        if(runningTime>=65530)         //  ��ֹ���
        {
            runningTime=65530;
        }
        if(runMode==TIMING_RUN && runningTime/2.5>=runTime*10)  //��ʱ��   1S->0.1S
        {
            runState=BRAKING;
            stopReason=TimeUp;
        }
    }
    else if(runState==BRAKING)//ͣ��ʱ��ʾ��Ϣ Բ������ɶ��
    {
        getEncoder();
        setMotorPWM();
        if(SI.nowSpeedL<5 && SI.nowSpeedR<5)
        {
            LCD_STATE=LCD_SHOW_STOP_INFORM;
        }
    }
    else if(visualScope_enable && !ExitMenu_flag)
    {
        adjustMotorPID_visualScope(aimSpeedL_visualScope,aimSpeedR_visualScope);
    }
    else
    {
        getEncoder();
    }

    if (buzzerTime > 0)
    {
        buzzerTime-=10;
        if (buzzerTime == 0)
        {
            BUZZER_OFF;
        }
    }
}


IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);
//    dl1b_get_distance();
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);
}
// **************************** PIT�жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        wireless_module_uart_handler(); // ����ģ��ͳһ�ص�����
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);             // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

//��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//  enableInterrupts();//�����ж�Ƕ��
//  if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//ͨ��2�ж�
//  {
//      CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//  }
//  if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//ͨ��6�ж�
//  {
//      CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//  }
//}


IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** �ⲿ�жϺ��� ****************************

// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_dma_handler();       // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


//IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
//{
//    enableInterrupts();//�����ж�Ƕ��
//    if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//ͨ��3�ж�
//    {
//        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
//        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();
//        else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_vsync();
//        else if (CAMERA_BIN       == camera_type)   ov7725_vsync();
//
//    }
//    if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//ͨ��7�ж�
//    {
//        CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);
//
//    }
//}
//
//
//
//IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
//{
//    enableInterrupts();//�����ж�Ƕ��
//
//    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_dma();
//    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_dma();
//    else if (CAMERA_BIN       == camera_type)   ov7725_dma();
//}

//�����жϺ���  ʾ��
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    camera_uart_handler(); // ����ͷ��������ͳһ�ص�����
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart3_handle);
    static uint8 num = 0;
    static uint8 previousData = 0;
    static uint8 TFMINI_PLUS_DATA[9];
    static uint8 flag = 0;
    uint8 nowData = 0;

    nowData = uart_read_byte(UART_3);
    if (flag == 0 && previousData == 0x59 && nowData == 0x59)
    {
        flag = 1;
        TFMINI_PLUS_DATA[0] = TFMINI_PLUS_DATA[1] = 0x59;
        num = 2;
    }
    else if (flag == 1)
    {
        TFMINI_PLUS_DATA[num++] = nowData;
        if (num == 9)
        {
            uint8 checkSum = 0;
            for (uint8 i = 0; i < 8; ++i)
            {
                checkSum += TFMINI_PLUS_DATA[i];
            }
            if (checkSum == TFMINI_PLUS_DATA[8])
            {
                TFMINI_PLUS_DISTANCE = TFMINI_PLUS_DATA[2] | (TFMINI_PLUS_DATA[3] << 8);
                TFMINI_PLUS_STRENGTH = TFMINI_PLUS_DATA[4] | (TFMINI_PLUS_DATA[5] << 8);
            }
            flag = 0;
        }
    }
    previousData = nowData;

}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
