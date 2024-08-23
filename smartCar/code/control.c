/*************************************************
Copyright (C), 2020-2022, Tech. Co., Ltd.
File name: control.c
Author: Samurai ������ BOOM7
Version: unknown
Date: 2021/12/15
Description: ƫ�����
Others: ѡ�߷�ʽ��֮ǰ�в�ͬ��kb��pd�޸�Ϊ���ٶ����
Function List:
    �ԣ�ֱ�ӿ�ͷ�ļ�
History: ��2021/2/24��ʼ��¼���ļ����޸�
1. Date:2021/2/24
Author:BOOM7 SAMURAI
Modification:����̬kb��Ϊ����ʱʹ�ã�����ʱ�ù̶�����
2. ...
*************************************************/

#include "control.h"
extern EulerAngleTypedef angle;
float kbleft[2] = {0};
float kbright[2] = {0};
float kbmid[2] = {0};

ControlLine controlLine = BOTHLINE; // ö�ٱ�����������
DeviationVAR DeviationVar;
DeviationParamKB KbParam;

int leftnum = 0;
int rightnum = 0;
int midnum = 0;
int top;
int top_temp;

float kk, bb, dd;

uint8 needToOutGarage = 1;
uint8 outGarageFlag = 0; // �����־
uint8 outGarageSpecialKkTime = 110;
uint8 annulusNum = 2;
uint8 rampNum = 2;

uint16 param_angle_roll = 600;

/*************************************************
Function: deviationParam_init()
Description: ƫ����������ʼ��
Input: null
Output: null
Return: null
Others: ��Щ������Ҫ�����������ǿ���̬kb��pd�ܵ�ʱ�򣬲�
        ���õĻ�Ч�����ܲ���ȫ��һ��kb��
Author: BOOM7 SAMURAI_WRY
*************************************************/
void deviationParam_init()
{
    KbParam.baseB = 220;
    KbParam.proportion_max = 450;
    KbParam.proportion_min = 400;
    KbParam.proportion_dd_Ramp = 100;
    KbParam.proportion_dd_ucross = 80;
    KbParam.proportion_enterAnnulus[0] = 380;
    KbParam.proportion_isAnnulus[0] = 380;
    KbParam.proportion_outAnnulus[0] = 380;

    KbParam.proportion_enterAnnulus[1] = 380;
    KbParam.proportion_isAnnulus[1] = 380;
    KbParam.proportion_outAnnulus[1] = 380;

    KbParam.proportion_enterAnnulus[2] = 380;
    KbParam.proportion_isAnnulus[2] = 380;
    KbParam.proportion_outAnnulus[2] = 380;

    KbParam.proportion_enterAnnulus[3] = 380;
    KbParam.proportion_isAnnulus[3] = 380;
    KbParam.proportion_outAnnulus[3] = 380;

    KbParam.proportion_enterAnnulus[4] = 380;
    KbParam.proportion_isAnnulus[4] = 380;
    KbParam.proportion_outAnnulus[4] = 380;

    KbParam.proportion_ramp = 300;
    KbParam.proportion_onRamp = 250;
    KbParam.proportion_fork = 100;
    KbParam.proportion_enterGarage = 110;
    KbParam.proportion_outGarage = 95;
}

/*************************************************
Function: directionControl()
Description: ����ƫ��
Input: null
Output: ����Ĵ��
Return: null
Others: ����ע��һ�������������
Author: BOOM7 SAMURAI_WRY
*************************************************/
void directionControl()
{
    // ������������ߣ�ÿ�������ߵĵ�����¼����
    II.midnum = getRampLine(kbmid, MIDLINE);
    II.leftnum = getLine(kbleft, LEFTLINE);
    II.rightnum = getLine(kbright, RIGHTLINE);
    // ȡ������ͼ�ĵ���
    uint16 lnum = II.lnum_all;
    uint16 rnum = II.rnum_all;
    // ��ʼѡ��������
    // ����Ԫ�ؾ�����Ҫֱ�Ӷ��������ߣ�����趨�Ļ���Ҫ��֤����״̬�����߶���Ҫ������
    // ����ѡ�߹���һ�����������ͼ������һ������Կ����ߵĵ��������ǳ����Լ�ѡ��
    // ����Ԫ�ص�ʱ������ֻҪ���������ֹ���ͺ���

    // ���ȴ����½���
    // �����ߵ������پͰ�����ͼ�ĵ�������
    if (II.leftnum < 10)
        lnum = 0;
    if (II.rightnum < 10)
        rnum = 0;
    if (II.leftnum < 10 && II.rightnum < 10)
        lnum = rnum = 0;
    if (II.line_forbid == LEFTLINE)
        lnum = II.leftnum = 0;
    else if (II.line_forbid == RIGHTLINE)
        rnum = II.rightnum = 0;
    else if (II.line_forbid == BOTHLINE)
        lnum = rnum = II.leftnum = II.rightnum = 0;

    // ����Ԫ������ѡȡ
    if (IF.annulus && IF.annulus != AA && IF.annulus != AL4 && IF.annulus != AR4) // ��
    {
        if (II.line_forbid == NONELINE)
        {
            if (IF.annulus > 6)
                controlLine = LEFTLINE;
            else if (IF.annulus > 0)
                controlLine = RIGHTLINE;
        }
        else
            controlLine = NONELINE;
    }
    else if (IF.fork == FR && startAddLine_fork)
    {

        //        if(II.rightnum>35) controlLine = RIGHTLINE;
        //        else controlLine = chooseControlLineByLineNum(II.leftnum, II.rightnum);

        if (II.leftnum > 10)
            controlLine = LEFTLINE;
        else if (II.rightnum > 35)
            controlLine = RIGHTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.fork == FL && startAddLine_fork)
    {
        //        if(II.leftnum>35) controlLine = RIGHTLINE;
        //        else controlLine = chooseControlLineByLineNum(II.leftnum, II.rightnum);

        if (II.rightnum > 10)
            controlLine = RIGHTLINE;
        else if (II.leftnum > 35)
            controlLine = LEFTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.crossroad == CL2)
    {
        if (II.rightnum > 10 && II.leftnum > 10)
            controlLine = BOTHLINE;
        else if (II.leftnum > 10)
            controlLine = LEFTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.crossroad == CR2)
    {
        if (II.rightnum > 10 && II.leftnum > 10)
            controlLine = BOTHLINE;
        else if (II.rightnum > 10)
            controlLine = RIGHTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.crossroad == UL2)
    {
        if (II.leftnum > 10)
            controlLine = LEFTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.crossroad == UR2)
    {
        if (II.rightnum > 10)
            controlLine = RIGHTLINE;
        else
            controlLine = NONELINE;
    }
    else if (IF.crossroad || !outGarageFlag || (IF.garage > 0 && IF.garage <= 6) || IF.annulus == AL4 || IF.annulus == AR4) // ʮ�ֺ͵���ʮ�ֺ�����
        controlLine = chooseControlLineByLineNum(II.leftnum, II.rightnum);
    else // �������������ͼ����
        controlLine = chooseControlLineByMapNum(lnum, rnum);
    float dist_bottom = get_dist_bottom();

    getDevParam(&kk, &bb, &dd);
#ifndef BOOM7_QT_DEBUG
    if (GET_SWITCH3())
        adjustParamByAimspeed(&kk);
#else
    if (imgSizeInfo == _47_60)
        adjustParamByAimspeed(&kk);
#endif

    if (IF.ramp || IF.rampDelay) // ��
    {
        controlLine = getRampControlLine();
        // ȷ��������ϵ��
        float p_roll;
        //        if(IF.ramp == 2)    p_roll= -(float)param_angle_roll / 100;
        //        else p_roll = 0;
        // �õĻ�������
        p_roll = 0;
        // �޿������򴿷�����
        if (controlLine == NONELINE)
        {
            // ���¹���˥����һЩ
            if (angle.Pitch > 10)
                DeviationVar.nowDeviation = DeviationVar.lastDeviation * 0.8;
            else
                DeviationVar.nowDeviation = DeviationVar.lastDeviation * 0.66;
        }
        else
        {
            DeviationVar.K = kbmid[0];
            DeviationVar.B = kbmid[1];
            DeviationVar.nowDeviation = DeviationVar.K * kk + dist_bottom * dd + angle.Roll * p_roll;
            DeviationVar.DK = DeviationVar.K * 100;

#ifdef BOOM7_QT_DEBUG
            qout << DeviationVar.K << "*" << kk << "+" << dist_bottom << "*" << dd << "+" << p_roll << "*" << angle.Roll << "=" << DeviationVar.nowDeviation;
            qout << "part_:k: " << DeviationVar.K * kk << "pard_dd: " << dist_bottom * dd << "part_roll" << angle.Roll * p_roll;
#endif
        }
    }
    else if (controlLine == BOTHLINE)
    {
        DeviationVar.K = (kbleft[0] + kbright[0]) / 2;
        DeviationVar.B = (kbleft[1] + kbright[1]) / 2;
        DeviationVar.nowDeviation = DeviationVar.K * kk + DeviationVar.B * bb + dist_bottom * dd;
        DeviationVar.DK = DeviationVar.K * 100;
#ifdef BOOM7_QT_DEBUG
        qout << DeviationVar.K << "*" << kk << "+" << DeviationVar.B << "*" << bb << "+" << dist_bottom << "*" << dd << "=" << DeviationVar.nowDeviation;
#endif
    }
    else if (controlLine == LEFTLINE)
    {
        //        qout<<getCurvatureRadius(LEFTLINE);
        DeviationVar.K = kbleft[0];
        DeviationVar.B = kbleft[1];
        DeviationVar.nowDeviation = DeviationVar.K * kk + DeviationVar.B * bb + dist_bottom * dd;
        DeviationVar.DK = DeviationVar.K * 100;
#ifdef BOOM7_QT_DEBUG
        qout << DeviationVar.K << "*" << kk << "+" << DeviationVar.B << "*" << bb << "+" << dist_bottom << "*" << dd << "=" << DeviationVar.nowDeviation;
#endif
    }
    else if (controlLine == RIGHTLINE)
    {
        DeviationVar.K = kbright[0];
        DeviationVar.B = kbright[1];
        DeviationVar.nowDeviation = DeviationVar.K * kk + DeviationVar.B * bb + dist_bottom * dd;
        DeviationVar.DK = DeviationVar.K * 100;
#ifdef BOOM7_QT_DEBUG
        qout << DeviationVar.K << "*" << kk << "+" << DeviationVar.B << "*" << bb << "+" << dist_bottom << "*" << dd << "=" << DeviationVar.nowDeviation;
#endif
    }
    else if (controlLine == NONELINE)
    {
        DeviationVar.nowDeviation = DeviationVar.lastDeviation;
    }

    int limitedAnnulusDev;
    //    if(DeviationVar.nowDeviation > SERVO_PWM_MAX) limitedAnnulusDev = SERVO_PWM_MAX;
    //    else if(DeviationVar.nowDeviation < -SERVO_PWM_MAX) limitedAnnulusDev = -SERVO_PWM_MAX;
    /*    else */ limitedAnnulusDev = DeviationVar.nowDeviation;
    if (AD.flag == 1)
    {

        AD.sumDev += limitedAnnulusDev;
        AD.sumDK += DeviationVar.DK;
        ++AD.cnt;
    }
    else if (AD.flag == 3)
    {
        DeviationVar.nowDeviation = AD.sumDev / AD.cnt;
        DeviationVar.DK = AD.sumDK / AD.cnt;
    }
    else if (AD.flag == 4)
    {
        DeviationVar.DK = AD.sumDK / AD.cnt;
    }

    if ((!(DeviationVar.nowDeviation == DeviationVar.nowDeviation && DeviationVar.K == DeviationVar.K && DeviationVar.B == DeviationVar.B)) || (((IF.annulus == AL1 && startAddLine_annulus) || IF.annulus == AL2 || (IF.annulus == AR1 && startAddLine_annulus) || IF.annulus == AR2) && !II.annulusDealFlag))
    {
        controlLine = NONELINE;
        DeviationVar.nowDeviation = DeviationVar.lastDeviation;
        DeviationVar.K = DeviationVar.lastK;
        DeviationVar.B = DeviationVar.lastB;
    }

    /*******************************************************/
    DeviationVar.lastDeviation = DeviationVar.nowDeviation;
    DeviationVar.lastK = DeviationVar.K;
    DeviationVar.lastB = DeviationVar.B;
    servoPID.error = DeviationVar.nowDeviation;
    Calc_ServoPID(&servoPID);
    /***************************************************************/

    servoPWM = SERVO_PWM_MID - servoPID.PID_Out;
#ifndef BOOM7_QT_DEBUG
    if (!GET_SWITCH7())
        pwm_duty(S3010_PWM_CH, servoPWM);
#endif
}

/*************************************************
Function: decideChoosingRule(uint8 lnum, uint8 rnum, uint8 lpnum, uint8 rpnum)
Description: ����ѡ�ߵĹ���
Input: ����ͼ�ĵ��������ҿ������ߵĵ���
Output: ����Ĵ��
Return: ����1������ͼ�����У�����2�ÿ����ߵ����У�����0��ѡ�ˣ�ֱ����һ֡ƫ�������
Others: ��Ҫ�Բ�����������д��ѡ�߹����ǿ���ֱ�Ӹ���
Author: BOOM7 SAMURAI_WRY
*************************************************/
// uint8 decideChoosingRule(uint8 lineNumL, uint8 lineNumR, uint8 mapNumL, uint8 mapNumR)
//{

//    return 0;
//}

/*************************************************
Function: chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR)
Description: ͨ�������ߵĵ�����ѡ��
Input: �����ߵĵ���
Output: null
Return: �����ߵ�ѡ��
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
ControlLine chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR)
{
#ifdef BOOM7_QT_DEBUG
    qout << "chooseControlLineByLineNum" << lineNumL << lineNumR;
#endif
    // �����ߵ���������һ��ֵ��������һ֡ƫ��
    if (lineNumL < 10)
        lineNumL = 0;
    if (lineNumR < 10)
        lineNumR = 0;
    if (lineNumL == 0 && lineNumR > 0)
        return RIGHTLINE;
    else if (lineNumL > 0 && lineNumR == 0)
        return LEFTLINE;
    else if (lineNumL < 10 && lineNumR < 10)
        return NONELINE;
    else if (lineNumL < 30 && lineNumR < 30)
    {
        if (lineNumL > lineNumR + 15)
            return LEFTLINE;
        else if (lineNumR > lineNumL + 15)
            return RIGHTLINE;
        else
            return BOTHLINE;
    }
    else
    {
        // �����ߵ���֮�Ƚϴ��򵥱��߿���
        float ldr = (float)lineNumL / (float)lineNumR;
        float rdl = (float)lineNumR / (float)lineNumL;
        if (!lineNumR)
            return LEFTLINE;
        if (!lineNumL)
            return RIGHTLINE;
        if (lineNumL > lineNumR && ldr > 1.8)
        {
            //            if(BSI.status==VERY_DANGEROUS_RIGHT)return BOTHLINE;
            return LEFTLINE;
        }
        if (lineNumR > lineNumL && rdl > 1.8)
        {
            //            if(BSI.status==VERY_DANGEROUS_LEFT)return BOTHLINE;
            return RIGHTLINE;
        }
        return BOTHLINE;
    }
    return NONELINE;
}

/*************************************************
Function: chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR)
Description: ͨ������ͼ�ĵ�����ѡ��
Input: �����ߵĵ���
Output: null
Return: �����ߵ�ѡ��
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
ControlLine chooseControlLineByMapNum(uint16 mapNumL, uint16 mapNumR)
{
#ifdef BOOM7_QT_DEBUG
    qout << "chooseControlLineByMapNum" << mapNumL << mapNumR;
#endif
    if (mapNumL > mapNumR + 600)
        return LEFTLINE;
    else if (mapNumR > mapNumL + 600)
        return RIGHTLINE;
    else
    {
        if (mapNumR >= 100 && mapNumL >= 100)
        {
            uint8 part_l = 0, part_r = 0;
            for (uint8 i = 0; i < II.num_lm; i++)
                if (II.lnum[i] > II.lnum[part_l])
                    part_l = i;
            for (uint8 i = 0; i < II.num_rm; i++)
                if (II.rnum[i] > II.rnum[part_r])
                    part_r = i;
            if (II.start_lm[part_l] > 20 && II.start_rm[0] < 5)
                return RIGHTLINE;
            if (II.start_rm[part_r] > 20 && II.start_lm[0] < 5)
                return LEFTLINE;
            return BOTHLINE;
        }
        else if (mapNumL >= mapNumR && ((mapNumR <= 100 && mapNumL >= 100) || II.leftnum > 15))
            return LEFTLINE;
        else if (mapNumR >= mapNumL && ((mapNumL <= 100 && mapNumR >= 100) || II.rightnum > 15))
            return RIGHTLINE;
        else
            return NONELINE;
    }
    return NONELINE;
}

/*************************************************
Function: getRampLine(float kb[2], ControlLine cl)
Description: �������ߵ���͸�������
Input: ���б�ʽؾ������
Output: null
Return: ���ߵ���
Others: null
Author: zjut
*************************************************/
uint8 getRampLine(float kb[2], ControlLine cl)
{
    float sumx = 0;
    float sumy = 0;
    float sumxy = 0;
    float sumx2 = 0;
    uint8 n = 0;

    float x;
    float y;
    uint8 lasty = 255;
    uint8 gap;
    memset(midline, 0, sizeof(midline));
    uint8 top = YM / 4;
    if (IF.ramp == 3)
        top = YM / 2;
    for (uint8 i = 0; i < top; ++i)
    {
        if (i < 5)
            gap = 2;
        else
            gap = 1;
        if (lasty != 255)
            if (II.midline[i] > II.midline[lasty] + gap || II.midline[i] + gap < II.midline[lasty])
                continue;
        midline[i] = II.midline[i];
    }

    // �µ�2״̬���ѿ�����������һ�����
    if (IF.ramp == 2)
    {
#define JUDGE_NUM 15
#define THRESHOLD_WIDTH 3
        uint8 midline_judgeWidth[JUDGE_NUM] = {0};
        uint8 cnt = 0;
        for (uint8 i = 0; i < top; i++)
        {
            if (midline[i])
                midline_judgeWidth[cnt++] = midline[i];
            if (cnt == JUDGE_NUM)
                break;
        }

        uint8 min = XX, max = 0, width = 0;
        for (uint8 i = 0; i < JUDGE_NUM; i++)
        {
            if (midline_judgeWidth[i] < min)
                min = midline_judgeWidth[i];
            if (midline_judgeWidth[i] > max)
                max = midline_judgeWidth[i];
        }
        width = max - min + 1;
#ifdef BOOM7_QT_DEBUG
        qout << "min" << min << "max" << max << "width" << width;
#endif
        if (width > THRESHOLD_WIDTH)
        {
#ifdef BOOM7_QT_DEBUG
            qout << "too wide";
#endif
            n = 0;
            return n;
        }
    }
    for (uint8 i = 0; i < top; i++)
        if (midline[i] != 0)
        {
            ++n;
            y = ((float)II.midline[i] - myCarMid) * k1[i];
            x = k2[i];
            sumx += x;
            sumy += y;
            sumxy += x * y;
            sumx2 += x * x;
            lasty = i;
        }
    kb[0] = (n * sumxy - sumx * sumy) / (n * sumx2 - sumx * sumx);
    kb[1] = sumy / n - kb[0] * sumx / n;
    return n;
}

/*************************************************
Function: getLine(float kb[2], ControlLine cl)
Description: �����ߵ���͸�������
Input: ���б�ʽؾ��������ĳ��������
Output: null
Return: �ÿ����ߵĵ���
Others: null
Author: zjut
*************************************************/
uint8 getLine(float kb[2], ControlLine cl)
{
    float sumx = 0;
    float sumy = 0;
    float sumxy = 0;
    float sumx2 = 0;
    uint8 n = 0;
    float x;
    float y;
    if (cl == LEFTLINE)
        for (uint8 i = II.startRow; i < II.endRow; ++i)
            for (uint8 j = 0; j <= XX; ++j)
            {
                if (leftmap[i][j] == 2 || leftmap[i][j] == 3)
                {
                    leftmap[i][j] = 4;
                    ++n;
                    y = (float)((int)j - (int)leftline[i]) * k1[i];

                    x = k2[i];
                    sumx += x;
                    sumy += y;
                    sumxy += x * y;
                    sumx2 += x * x;
                }
            }
    else if (cl == RIGHTLINE)
        for (uint8 i = II.startRow; i < II.endRow; ++i)
            for (int j = XX; j >= 0; --j)
            {
                if (rightmap[i][j] == 2 || rightmap[i][j] == 3)
                {
                    rightmap[i][j] = 4;
                    ++n;
                    y = ((float)j - (float)rightline[i]) * k1[i];
                    x = k2[i];
                    sumx += x;
                    sumy += y;
                    sumxy += x * y;
                    sumx2 += x * x;
                }
            }
    else if (cl == MIDLINE)
        for (uint8 i = II.startRow; i < II.endRow - 10; ++i)
        {
            ++n;
            y = ((float)II.midline[i] - myCarMid) * k1[i];
            x = k2[i];
            sumx += x;
            sumy += y;
            sumxy += x * y;
            sumx2 += x * x;
        }

    kb[0] = (n * sumxy - sumx * sumy) / (n * sumx2 - sumx * sumx);
    kb[1] = sumy / n - kb[0] * sumx / n;
    return n;
}

/*************************************************
Function: uint8 get_K(float *k, ControlLine cl)
Description: �����ߵ���͸�������,�Ǹ����ԣ�ֻ���б��
Input: ���б�ʽؾ��������ĳ��������
Output: null
Return: �ÿ����ߵĵ���
Others: null
Author: zjut
*************************************************/
uint8 get_K(float *k, ControlLine cl)
{
    float sumx = 0;
    float sumy = 0;
    float sumxy = 0;
    float sumx2 = 0;
    uint8 n = 0;
    float x;
    float y;

    if (cl == LEFTLINE)
        for (uint8 i = 0; i < II.endRow; ++i)
            for (uint8 j = 0; j < XX; ++j)
            {
                if (leftmap[i][j] == 2 || leftmap[i][j] == 3)
                {
                    leftmap[i][j] = 4;
                    ++n;
                    y = (float)((int)j - (int)leftline[i]) * k1[i];
                    x = k2[i];
                    sumx += x;
                    sumy += y;
                    sumxy += x * y;
                    sumx2 += x * x;
                }
            }
    else if (cl == RIGHTLINE)
        for (uint8 i = 0; i < II.endRow; ++i)
            for (uint8 j = XX; j > 0; --j)
            {
                if (rightmap[i][j] == 2 || rightmap[i][j] == 3)
                {
                    rightmap[i][j] = 4;
                    ++n;
                    y = ((float)j - (float)rightline[i]) * k1[i];
                    x = k2[i];
                    sumx += x;
                    sumy += y;
                    sumxy += x * y;
                    sumx2 += x * x;
                }
            }
    *k = (n * sumxy - sumx * sumy) / (n * sumx2 - sumx * sumx);
    return n;
}

float get_dist_bottom()
{
    uint8 line = 0;
    float ave_dist = 0;
    uint8 b_row = 20;
    uint8 cnt_left = 0, cnt_right = 0, cnt_mid = 0;

    if (IF.ramp)
    {
        b_row = YM / 4;
        for (uint8 row = 0; row < b_row; row++)
            if (midline[row])
                cnt_mid++;
        for (uint8 row = 0; row < b_row; row++)
            if (midline[row])
                ave_dist += ((float)midline[row] - myCarMid) * k1[row] / (float)cnt_mid;
    }
    else
    {
        for (uint8 i = 0; i < b_row; ++i)
            for (uint8 j = 0; j < XX; ++j)
            {
                if (leftmap[i][j] == 4)
                    cnt_left++;
                if (rightmap[i][j] == 4)
                    cnt_right++;
            }

        if (cnt_left > 3 && cnt_right > 3)
        {
            if (cnt_left > cnt_right)
                line = goLeft;
            else
                line = goRight;
        }
        else if (cnt_left > 3)
            line = goLeft;
        else if (cnt_right > 3)
            line = goRight;

        if (line == goLeft)
        {
            for (uint8 row = 0; row < b_row; row++)
                for (uint8 col = 0; col < XM; col++)
                    if (leftmap[row][col] == 4)
                        ave_dist += ((float)col - leftline[row]) * k1[row] / (float)cnt_left;
        }
        else if (line == goRight)
        {
            for (uint8 row = 0; row < b_row; row++)
                for (uint8 col = 0; col < XM; col++)
                    if (rightmap[row][col] == 4)
                        ave_dist += ((float)col - rightline[row]) * k1[row] / (float)cnt_right;
        }
        else
            ave_dist = 0;
    }
    //    qout<<ave_dist;
    return ave_dist;
}
/*************************************************
Function: getAveDeviation(ControlLine cl)
Description: ��ȡĿ���߶����׼�ߵ�ƽ��ʵ�ʾ���ƫ��
Input: Ŀ���߶�
Output: null
Return: ƽ��ƫ��
Others: null
Author: zjut
*************************************************/
float getAveDeviation(ControlLine cl) // leftmap, leftline
{
    float sum = 0;
    float x;
    int n = 0;
    uint8 d_line = 30;
    uint8 stop = II.top - 1 < 59 ? II.top - 1 : 59;
    if (cl == LEFTLINE)
        for (uint8 j = d_line; j < stop; ++j)
            for (uint8 i = 0; i < XM; ++i)
                if (leftmap[j][i] >= 2)
                {
                    ++n;
                    x = (i - leftline[j]) * k1[j];
                    sum += x;
                }
    if (cl == RIGHTLINE)
        for (uint8 j = d_line; j < stop; ++j)
            for (int8 i = XX; i >= 0; --i)
                if (rightmap[j][i] >= 2)
                {
                    ++n;
                    x = (i - rightline[j]) * k1[j];
                    sum += x;
                }
    if (n == 0)
        return 10000;
    else
        return (sum / n);
}
#ifdef BOOM7_QT_DEBUG
extern int32 nowSpeedL_SD, nowSpeedR_SD;
#endif
/*************************************************
Function: getDevParam(float* kk, float* bb)
Description: ��ȡ����
Input: ����kb�ĵ�ַ
Output: ����Ŀǰ����Ԫ�ط�����Ӧ�Ĳ���
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void getDevParam(float *kk, float *bb, float *dd)
{
    float proportion = (float)KbParam.proportion_min / 100.000; // Ĭ��ʹ��k_min;

    if (IF.annulus == AL1 || IF.annulus == AL2 || IF.annulus == AR1 || IF.annulus == AR2)
        proportion = (float)KbParam.proportion_enterAnnulus[(judgedAnnulusNum - 1) % annulusNum] / 100.000;
    else if (IF.annulus == AL3 || IF.annulus == AR3)
        proportion = (float)KbParam.proportion_isAnnulus[(judgedAnnulusNum - 1) % annulusNum] / 100.000;
    else if (IF.annulus == AL4 || IF.annulus == AL5 || IF.annulus == AR4 || IF.annulus == AR5 || IF.annulus == AL6 || IF.annulus == AR6)
        proportion = (float)KbParam.proportion_outAnnulus[(judgedAnnulusNum - 1) % annulusNum] / 100.000;
    else if (IF.fork)
        proportion *= (float)KbParam.proportion_fork / 100.000;
    else if (IF.ramp == 1 || (IF.ramp == 3 && angle.Pitch < -10))
        proportion = (float)KbParam.proportion_ramp / 100.000;
    else if (IF.ramp == 2)
        proportion = (float)KbParam.proportion_onRamp / 100.000;
    else if (!outGarageFlag)
        proportion *= (float)KbParam.proportion_outGarage / 100.000;
    //    else if(IF.garage)
    //        proportion *= (float)KbParam.proportion_enterGarage / 100.000;

    *bb = KbParam.baseB / 100.000;
    *kk = *bb * proportion * 10.000;
    if (IF.crossroad == UL1 || IF.crossroad == UR1 || IF.crossroad == UL2 || IF.crossroad == UR2)
        *dd = *bb * KbParam.proportion_dd_ucross / 100;
    else if (IF.ramp || IF.rampDelay)
        *dd = *bb * KbParam.proportion_dd_Ramp / 100;
    else
        *dd = 0;
    // qout<<"final kk:"<<*kk<<"bb"<<*bb;
}

/*************************************************
Function: adjustParamByAimspeed(float* kk)
Description: ��̬kb��pd����������Ե���ʱ����ֹײ·��Բ�����������,����ʱ�ټӲ������ܻ�����
Input: ����kk�ĵ�ַ
Output: ���ݵ�ǰĿ���ٶ��޸Ĳ���ֵ
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void adjustParamByAimspeed(float *kk)
{
    // �µ�p������
    // Ȼ������ٶȼ����Ӧpd����������һ�κ�������������

    int nowSpeed = SI.aimSpeed;

    // �����޷�,Ϊ����pd�����𵴣�ʵ�ʾ��Ǹ߹�ĳһ�ٶȺ����������
    nowSpeed = nowSpeed > MAXSPEED ? MAXSPEED : nowSpeed;
    nowSpeed = nowSpeed < MINSPEED ? MINSPEED : nowSpeed;
    // ��ȡһ�κ���б��
    float k_p = (float)(servoPID.kp_max - servoPID.kp_min) / (MAXSPEED - MINSPEED);
    float k_d = (float)(servoPID.kd_max - servoPID.kd_min) / (MAXSPEED - MINSPEED);

    // ���ﶯ̬PD��˼·Ӧ�������⣬�ҵ������ʵd��max��min��һ����ֵ��

    // ���㵱ǰ�ٶ��µ�pd��p������ģʽ���䣬d��k�����ٶ�����
    // ���ճ�����˼���Ļ��������ٶȿ�֮��ƫ��Ҫ��Ĵ�Щ������pd�������෴�ģ��ٶȴ�ҪС��
    // �ٶȴ��ʱ����֡ͼƬ��ƫ���΢�ֱ����ʱ�������d�Ӵ������ײ�����
    // ����pҲ�ǣ��ᵼ����֡ͼƬƫ���΢�ֱ����Ȼ�ᶶ
    servoPID.kp = servoPID.kp_min + (nowSpeed - MINSPEED) * k_p;
    // һ��һ����������d��ͬ�Ļ�������Ҫ�ȳ���d��Щ�����ܴ�̫�࣬��Ȼֱ���������Գƻᶶ
    servoPID.kd_in = (servoPID.kd_min + (MAXSPEED - nowSpeed) * k_d) * (100 + servoPID.differential) / 100;
    servoPID.kd_out = (servoPID.kd_min + (MAXSPEED - nowSpeed) * k_d) * (100 - servoPID.differential) / 100;
    // ��������̬k,���k_k�൱���ٶ�ÿ��1��k��Ҫͬ���������ٲŽ�Ϊ����
    float k_k = (float)(KbParam.proportion_max - KbParam.proportion_min) * KbParam.baseB / (MAXSPEED - MINSPEED) / 1000;
    // ��̬k����һ�κ�������Ϊ��ͬԪ�ص�k��һ�����ڼӵ�ʱ���Ե�ǰk��Ϊ����
    if (!IF.garage)
        *kk += k_k * (nowSpeed - MINSPEED);
    // ���㵱ǰ�ٶ��µ�pd
    servoPID.kp = servoPID.kp_min + (nowSpeed - MINSPEED) * k_p;
}

ControlLine getRampControlLine()
{
    ControlLine ctl = NONELINE;
#ifdef BOOM7_QT_DEBUG
    qout << "getRampControlLine" << midnum;
#endif
    if (II.midnum < 10)
        ctl = NONELINE;
    else
        ctl = MIDLINE;
    return ctl;
}

// �����ʣ��������񲻺�׼
// float getCurvatureRadius(ControlLine cl)
//{
//     if(cl == LEFTLINE)
//     {
//         Point high,low,mid;
//         high.y = 0;low.y = YY;
//         for(uint8 i=0;i<=II.top;i++)
//         {
//             uint8 col = getMapXMin_Row(i,leftmap,4);
//             if(col != XM)
//             {
//                 if(i >= high.y)
//                 {
//                     high.y = i;high.x = col;
//                 }
//                 if(i <= low.y)
//                 {
//                     low.y = i;low.x = col;
//                 }
//             }
//         }

//        if(high.y <= low.y) return 0;
//        qout<<"high:"<<high.x<<high.y;
//        qout<<"low:"<<low.x<<low.y;
//        //��ֹ�������õ�
//        mid.y = high.y * 0.5 + low.y * 0.5;
//        //�Ȱ������͵��ҳ���
//        high = getRealPoint(high);
//        low = getRealPoint(low);
//        mid.y = high.y * 0.5 + low.y * 0.5;
//        for(uint8 row = 0;row<YM;row++)
//            if(k2[row] > mid.y)
//            {
//                mid.y = row;
//                break;
//            }
//        mid.x = getMapXMin_Row(mid.y,leftmap,4);
//        mid = getRealPoint(mid);

//        qout<<"high:"<<high.x<<high.y;
//        qout<<"mid:"<<mid.x<<mid.y;
//        qout<<"low:"<<low.x<<low.y;
//        double a[2],b[2],c[2];
//        a[0] = high.x;a[1] = high.y;
//        b[0] = mid.x; b[1] = mid.y;
//        c[0] = low.x; c[1] = low.y;
//        return curvature(a,b,c);
////        if(getMapXMax_Row())
//    }
//    else if(cl == RIGHTLINE)
//    {

//    }

//    return 0;
//}

// int collinear(double a[2],double b[2],double c[2])//�ж������Ƿ��ߣ����߷���1
//{
//     double k1,k2;
//     double kx1,ky1,kx2,ky2;
//     if(a[0]==b[0]&&b[0]==c[0])  return 1;//��������궼��ȣ�����
//     else
//     {
//         kx1=b[0]-a[0];

//        kx2=b[0]-c[0];
//        ky1=b[1]-a[1];
//        ky2=b[1]-a[1];
//        k1=ky1/kx1;
//        k2=ky2/kx2;
//        if(k1==k2) return 1;//AB��BCб����ȣ�����
//        else  return 0;//������
//    }
//}
// double curvature(double a[2],double b[2],double c[2])//doubleΪ�������ͣ�
//{                                                    //����a[2]Ϊ��a��������Ϣ��a[0]Ϊa��x���꣬a[1]Ϊa��y����
//    double radius;//���ʰ뾶
//    if(collinear(a,b,c)==1)//�ж������Ƿ���
//    {
//        radius=10000;//���㹲��ʱ��������Ϊĳ��ֵ��0
//    }
//    else
//    {
//        double dis,dis1,dis2,dis3;//����
//        double A,A_duijiao;//abȷ���ı�����Ӧ�Ľ�A��cosֵ
//        qout<<a[0]<<a[1]<<b[0]<<b[1]<<c[0]<<c[1];
//        qout<<distance(a[0],a[1],c[0],c[1]);
//        dis1=distance(a[0],a[1],c[0],c[1]);
//        dis2=distance(a[0],a[1],b[0],b[1]);
//        dis3=distance(b[0],b[1],c[0],c[1]);
//        qout<<dis1<<dis2<<dis3;
//        dis=dis2*dis2+dis3*dis3-dis1*dis1;
//        A=acos(dis/(2*dis2*dis3));//���Ҷ���
//        qout<<A<<A * PI / 180;
//        A_duijiao = 2*PI-2*A;
//        radius =0.5 * dis1 / sin(0.5 * A_duijiao);
//        qout<<1/radius;
//    }
//    return radius;
//}
