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
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// ���̵��뵽���֮�� Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
// ����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
// Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level �������Ż��ȼ�
// һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż� ��˴��Ҳ��������Ϊ2���Ż�

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ���Ϊ��Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��

int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��  ��ر���
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

            if (PRT.sdSaveTime > 65000) // ������д���
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
