/*************************************************
Copyright (C), 2020-2022, Tech. Co., Ltd.
File name: control.c
Author: Samurai 王若愚 BOOM7
Version: unknown
Date: 2021/12/15
Description: 偏差计算
Others: 选线方式与之前有不同，kb和pd修改为与速度相关
Function List:
    略，直接看头文件
History: 于2021/2/24开始记录本文件的修改
1. Date:2021/2/24
Author:BOOM7 SAMURAI
Modification:将动态kb改为低速时使用，高速时用固定参数
2. ...
*************************************************/

#include "control.h"
extern EulerAngleTypedef angle;
float kbleft[2] = {0};
float kbright[2] = {0};
float kbmid[2] = {0};

ControlLine controlLine = BOTHLINE; // 枚举变量属于整型
DeviationVAR DeviationVar;
DeviationParamKB KbParam;

int leftnum = 0;
int rightnum = 0;
int midnum = 0;
int top;
int top_temp;

float kk, bb, dd;

uint8 needToOutGarage = 1;
uint8 outGarageFlag = 0; // 出库标志
uint8 outGarageSpecialKkTime = 110;
uint8 annulusNum = 2;
uint8 rampNum = 2;

uint16 param_angle_roll = 600;

/*************************************************
Function: deviationParam_init()
Description: 偏差计算参数初始化
Input: null
Output: null
Return: null
Others: 这些参数主要靠调，尤其是开动态kb和pd跑的时候，不
        调好的话效果可能不如全用一个kb了
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
Description: 计算偏差
Input: null
Output: 舵机的打脚
Return: null
Others: 按照注释一点点把流程理清楚
Author: BOOM7 SAMURAI_WRY
*************************************************/
void directionControl()
{
    // 首先拟合三条线，每条控制线的点数记录下来
    II.midnum = getRampLine(kbmid, MIDLINE);
    II.leftnum = getLine(kbleft, LEFTLINE);
    II.rightnum = getLine(kbright, RIGHTLINE);
    // 取出左右图的点数
    uint16 lnum = II.lnum_all;
    uint16 rnum = II.rnum_all;
    // 开始选控制线线
    // 特殊元素尽量不要直接定死控制线，如果设定的话就要保证整个状态那条线都不要出问题
    // 两种选线规则，一个是针对左右图点数，一个是针对控制线的点数，都是程序自己选的
    // 特殊元素的时候我们只要给定用哪种规则就好了

    // 首先处理下禁线
    // 控制线点数过少就把左右图的点数清零
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

    // 特殊元素特殊选取
    if (IF.annulus && IF.annulus != AA && IF.annulus != AL4 && IF.annulus != AR4) // 环
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
    else if (IF.crossroad || !outGarageFlag || (IF.garage > 0 && IF.garage <= 6) || IF.annulus == AL4 || IF.annulus == AR4) // 十字和单边十字和三叉
        controlLine = chooseControlLineByLineNum(II.leftnum, II.rightnum);
    else // 其他情况用左右图点数
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

    if (IF.ramp || IF.rampDelay) // 坡
    {
        controlLine = getRampControlLine();
        // 确定翻滚角系数
        float p_roll;
        //        if(IF.ramp == 2)    p_roll= -(float)param_angle_roll / 100;
        //        else p_roll = 0;
        // 用的话按上面
        p_roll = 0;
        // 无控制线则纯翻滚角
        if (controlLine == NONELINE)
        {
            // 上坡过程衰减慢一些
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
Description: 决定选线的规则
Input: 左右图的点数，左右控制线线的点数
Output: 舵机的打脚
Return: 返回1用左右图点数判，返回2用控制线点数判，返回0不选了，直接上一帧偏差安排上了
Others: 必要性不大，有需求再写，选线规则还是可以直接给的
Author: BOOM7 SAMURAI_WRY
*************************************************/
// uint8 decideChoosingRule(uint8 lineNumL, uint8 lineNumR, uint8 mapNumL, uint8 mapNumR)
//{

//    return 0;
//}

/*************************************************
Function: chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR)
Description: 通过控制线的点数来选线
Input: 左右线的点数
Output: null
Return: 控制线的选择
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
ControlLine chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR)
{
#ifdef BOOM7_QT_DEBUG
    qout << "chooseControlLineByLineNum" << lineNumL << lineNumR;
#endif
    // 两边线点数都少于一定值，则用上一帧偏差
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
        // 两边线点数之比较大，则单边线控制
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
Description: 通过左右图的点数来选线
Input: 左右线的点数
Output: null
Return: 控制线的选择
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
Description: 过坡中线的逆透视与拟合
Input: 存放斜率截距的数组
Output: null
Return: 中线点数
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

    // 坡道2状态，把控制线限制在一定宽度
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
Description: 控制线的逆透视与拟合
Input: 存放斜率截距的数组与某条控制线
Output: null
Return: 该控制线的点数
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
Description: 控制线的逆透视与拟合,是个尝试，只拟合斜率
Input: 存放斜率截距的数组与某条控制线
Output: null
Return: 该控制线的点数
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
Description: 获取目标线段与标准线的平均实际距离偏差
Input: 目标线段
Output: null
Return: 平均偏差
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
Description: 获取参数
Input: 参数kb的地址
Output: 根据目前所在元素返回相应的参数
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void getDevParam(float *kk, float *bb, float *dd)
{
    float proportion = (float)KbParam.proportion_min / 100.000; // 默认使用k_min;

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
Description: 动态kb与pd，基本是针对低速时，防止撞路肩对参数进行修整,高速时再加参数可能会震荡了
Input: 参数kk的地址
Output: 根据当前目标速度修改参数值
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void adjustParamByAimspeed(float *kk)
{
    // 坡道p单独设
    // 然后根据速度计算对应pd参数，先用一次函数这样来试试

    int nowSpeed = SI.aimSpeed;

    // 参数限幅,为避免pd过大震荡，实际就是高过某一速度后参数不变了
    nowSpeed = nowSpeed > MAXSPEED ? MAXSPEED : nowSpeed;
    nowSpeed = nowSpeed < MINSPEED ? MINSPEED : nowSpeed;
    // 获取一次函数斜率
    float k_p = (float)(servoPID.kp_max - servoPID.kp_min) / (MAXSPEED - MINSPEED);
    float k_d = (float)(servoPID.kd_max - servoPID.kd_min) / (MAXSPEED - MINSPEED);

    // 这里动态PD的思路应该有问题，我到最后其实d的max和min是一样的值。

    // 计算当前速度下的pd，p按变速模式来变，d和k按照速度来变
    // 按照常理来思考的话，好像速度快之后偏差要算的大些，但是pd这里是相反的，速度大要小。
    // 速度大的时候，两帧图片的偏差的微分变大，这时候如果把d加大则容易产生震荡
    // 增加p也是，会导致两帧图片偏差的微分变大，自然会抖
    servoPID.kp = servoPID.kp_min + (nowSpeed - MINSPEED) * k_p;
    // 一般一样，出入弯d不同的话，入弯要比出弯d大些，不能大太多，不然直道参数不对称会抖
    servoPID.kd_in = (servoPID.kd_min + (MAXSPEED - nowSpeed) * k_d) * (100 + servoPID.differential) / 100;
    servoPID.kd_out = (servoPID.kd_min + (MAXSPEED - nowSpeed) * k_d) * (100 - servoPID.differential) / 100;
    // 接下来动态k,求得k_k相当于速度每快1，k需要同比增长多少才较为合适
    float k_k = (float)(KbParam.proportion_max - KbParam.proportion_min) * KbParam.baseB / (MAXSPEED - MINSPEED) / 1000;
    // 动态k按照一次函数，因为不同元素的k不一样，在加的时候以当前k作为基数
    if (!IF.garage)
        *kk += k_k * (nowSpeed - MINSPEED);
    // 计算当前速度下的pd
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

// 求曲率，不过好像不很准
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
//        //防止出问题用的
//        mid.y = high.y * 0.5 + low.y * 0.5;
//        //先把最高最低点找出来
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

// int collinear(double a[2],double b[2],double c[2])//判断三点是否共线，共线返回1
//{
//     double k1,k2;
//     double kx1,ky1,kx2,ky2;
//     if(a[0]==b[0]&&b[0]==c[0])  return 1;//三点横坐标都相等，共线
//     else
//     {
//         kx1=b[0]-a[0];

//        kx2=b[0]-c[0];
//        ky1=b[1]-a[1];
//        ky2=b[1]-a[1];
//        k1=ky1/kx1;
//        k2=ky2/kx2;
//        if(k1==k2) return 1;//AB与BC斜率相等，共线
//        else  return 0;//不共线
//    }
//}
// double curvature(double a[2],double b[2],double c[2])//double为数据类型，
//{                                                    //数组a[2]为点a的坐标信息，a[0]为a的x坐标，a[1]为a的y坐标
//    double radius;//曲率半径
//    if(collinear(a,b,c)==1)//判断三点是否共线
//    {
//        radius=10000;//三点共线时将曲率设为某个值，0
//    }
//    else
//    {
//        double dis,dis1,dis2,dis3;//距离
//        double A,A_duijiao;//ab确定的边所对应的角A的cos值
//        qout<<a[0]<<a[1]<<b[0]<<b[1]<<c[0]<<c[1];
//        qout<<distance(a[0],a[1],c[0],c[1]);
//        dis1=distance(a[0],a[1],c[0],c[1]);
//        dis2=distance(a[0],a[1],b[0],b[1]);
//        dis3=distance(b[0],b[1],c[0],c[1]);
//        qout<<dis1<<dis2<<dis3;
//        dis=dis2*dis2+dis3*dis3-dis1*dis1;
//        A=acos(dis/(2*dis2*dis3));//余弦定理
//        qout<<A<<A * PI / 180;
//        A_duijiao = 2*PI-2*A;
//        radius =0.5 * dis1 / sin(0.5 * A_duijiao);
//        qout<<1/radius;
//    }
//    return radius;
//}
