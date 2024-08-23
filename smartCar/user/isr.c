

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        tasking v6.3r1
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
//在isr.c的中断函数，函数定义的第二个参数固定为0，请不要更改，即使你用CPU1处理中断也不要更改，需要CPU1处理中断只需要在isr_config.h内修改对应的宏定义即可
volatile uint32 runTime = 0;
volatile KEY_e last = 0;
volatile uint8 ExitMenu_flag=0;
volatile uint32 runningTime=0;
volatile uint8 runReadyFlag=0;
volatile int startCarTimeCnt=0;  //100*10=1s
//int16 buzzerTime = 0;
volatile uint16 TFMINI_PLUS_DISTANCE;
volatile uint16 TFMINI_PLUS_STRENGTH;
//PIT中断函数  示例
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    /***************************************按键扫描*************************************/
    //按键扫描
    key_IRQHandler();         //把按键消息送入FIFO
    while(get_key_msg(&keymsg)); //空的时候就会跳出来
    //按键的特殊处理   处理完要把 按键弹起 不然会重复执行
    if(keymsg.key==KEY_RUN && keymsg.status==KEY_DOWN )
    {
        runState=READY;     //编码器初始化完成会变成run

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
    /********************电机控制**********************/
    if(runState==READY && 0==runReadyFlag)   //用来发车的
    {
        startCarTimeCnt++;
        if(startCarTimeCnt>=65530)    //防止溢出
        {
            startCarTimeCnt=65530;
        }

        if(startCarTimeCnt/2.5>=200)        //延时发车
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
        runningTime+=1;                 //  用来延时判起跑线和出库特殊处理的,一定时间内速度小于某值时b+20
        if(runningTime>=65530)         //  防止溢出
        {
            runningTime=65530;
        }
        if(runMode==TIMING_RUN && runningTime/2.5>=runTime*10)  //计时跑   1S->0.1S
        {
            runState=BRAKING;
            stopReason=TimeUp;
        }
    }
    else if(runState==BRAKING)//停车时显示信息 圆环个数啥的
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
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);
//    dl1b_get_distance();
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);
}
// **************************** PIT中断函数 ****************************

// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // 开启中断嵌套
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        wireless_module_uart_handler(); // 无线模块统一回调函数
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);             // 开启中断嵌套
    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//  enableInterrupts();//开启中断嵌套
//  if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//  }
//  if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//  }
//}


IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // 开启中断嵌套
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // 摄像头触发采集统一回调函数
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** 外部中断函数 ****************************

// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套
    camera_dma_handler();       // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


//IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
//{
//    enableInterrupts();//开启中断嵌套
//    if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
//    {
//        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
//        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();
//        else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_vsync();
//        else if (CAMERA_BIN       == camera_type)   ov7725_vsync();
//
//    }
//    if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
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
//    enableInterrupts();//开启中断嵌套
//
//    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_dma();
//    else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_dma();
//    else if (CAMERA_BIN       == camera_type)   ov7725_dma();
//}

//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    camera_uart_handler(); // 摄像头参数配置统一回调函数
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
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
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
