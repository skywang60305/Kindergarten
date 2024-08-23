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
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 工程导入到软件之后 应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level 处设置优化等级
// 一般默认新建立的工程都会默认开2级优化 因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因为需要我们自己手动调用enableInterrupts();来开启中断的响应。

int core0_main(void)
{
    clock_init(); // 获取时钟频率  务必保留
    debug_init();
    enableInterrupts();
    static int skip = 10;
    static int keep = 10;
    while (TRUE)
    {
        while (img_prepared == 0)
            system_delay(10, 1);

        img_prepared = 0;
        cpu0 = WORKING;
        system_delay(10, 1);

        if (GET_SWITCH4())
        {
            if (skip > 0)
            {
                skip--;
                continue;
            }
            if (LCD_STATE == LCD_SHOW_STOP_INFORM)
            {
                if (keep > 0)
                    keep--;
                else
                    sd_stop();
                continue;
            }

            PRT.sdSaveTime = (uint32)system_getval();
            sd_write();
            PRT.sdSaveTime = (uint32)system_getval() - PRT.sdSaveTime;
            sdSaveCnt++;

            if (PRT.sdSaveTime > 3000)
                over3msTimeCnt++;

            if (PRT.sdSaveTime > 5000)
                over5msTimeCnt++;

            if (PRT.sdSaveTime > 8000)
                over8msTimeCnt++;

            if (PRT.sdSaveTime > 65000) // 清空所有次数
            {
                over3msTimeCnt = 0;
                over5msTimeCnt = 0;
                over8msTimeCnt = 0;
                sdSaveCnt = 0;
                sdStability = 0;
            }
        }
        cpu0 = WAITING;
        system_delay(10, 1);
    }
    return 0;
}

#pragma section all restore
