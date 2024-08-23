/*************************************************
Copyright (C), 2020-2022, Tech. Co., Ltd.
File name: deal_img.c
Author: Samurai 王若愚 BOOM7
Version: unknown
Date: 2022/3/18
Description: 辅助功能
            （1）车身安全性检测
            （2）行车记录仪
            （1）车身安全性检测
            （1）车身安全性检测
1. Date: 2022/3/18
Author: Samurai_WRY
Modification: 开始写一些没啥用的辅助功能：
    （1）行车记录仪，记录之前的弯道方向，以及元素。
    （2）车身危险性评估放到这个文件来，也算是辅助功能
1. Date: 2022/3/25
Author: Samurai_WRY
Modification: 行车记录仪的道路类型记录，写成状态机，减少代码量，方便维护

2. Date: 2022/4/1
Author: Samurai_WRY
Modification: 路况记录仪初步完成，一些状态变normal的条件，写成，某状态持续一段时间后不变，就变成normal
                还有一些，在图像不符合当前状态的时候，调成normal
*************************************************/
#include "accessibility.h"
#ifdef BOOM7_QT_DEBUG
#include "mainwindow.h"
extern MainWindow *m;
#endif
int servoLimit_veryDanger = 5;
BodyworkSafetyIndex BSI_INIT, BSI;
RoadRecorder roadRecorder;
ElementRecorder elementRecorder;
RoadType roadType_now = unKnown;
// 评估路况的指标
int low2carmid, mid2carmid, high2carmid, k_ml, k_hm, k_ml_abs, k_hm_abs, k, k_abs, sumOfCenter, range_midline;
int low = 0, mid = 40, high = 50;

int release = 0;
/*************************************************
Function: void drivingRecorder_init()
Description: 初始化行车记录仪
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void drivingRecorder_init()
{
    roadRecorder.now = unKnown;
    roadRecorder.before = unKnown;
    roadRecorder.next = unKnown;
    for (uint8 i = 0; i < 100; i++)
        roadRecorder.all[i] = unKnown;
    roadRecorder.top = -1;
    roadRecorder.time = 0;
    roadRecorder.isSCurve = 0;

    if (needToOutGarage)
        elementRecorder.now = outGarage;
    else
        elementRecorder.now = null;
    elementRecorder.before = null;
    elementRecorder.next = null;
    for (uint8 i = 0; i < 100; i++)
        elementRecorder.all[i] = null;
    elementRecorder.top = -1;
}

/*************************************************
Function: void drivingRecorder_record()
Description: 行车记录仪记录
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void drivingRecorder_record()
{
    recordElementType();
    recordRoadType();
}
/*************************************************
Function: roadType_change(RoadType now),elementType_change(ElementType now)
Description: 行车记录仪改变
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void roadType_change(RoadType now)
{
    release = 0;
    roadRecorder.all[++roadRecorder.top] = now;
    roadRecorder.before = roadRecorder.now;
    roadRecorder.now = now;
    roadRecorder.time = 0;
}

void elementType_change(ElementType now)
{
    elementRecorder.all[++elementRecorder.top] = now;
    elementRecorder.before = elementRecorder.now;
    elementRecorder.now = now;
}

/*************************************************
Function: getElementType(),getRoadType()
Description: 行车记录仪记录
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void recordElementType()
{
    ElementType now = null;
    if (IF.annulus)
        now = annulus;
    else if (IF.crossroad)
        switch (IF.crossroad)
        {
        case CR1:
            now = cross;
            break;
        case CL1:
            now = cross;
            break;
        case CR2:
            now = cross;
            break;
        case CL2:
            now = cross;
            break;
        case CM1:
            now = cross;
            break;
        case CM2:
            now = cross;
            break;
        case UL1:
            now = ucross;
            break;
        case UL2:
            now = ucross;
            break;
        case UR1:
            now = ucross;
            break;
        case UR2:
            now = ucross;
            break;
        }
    else if (IF.fork)
        now = fork;
    else if (IF.garage)
        switch (IF.crossroad)
        {
        case 253:
            now = outGarage;
            break;
        case 254:
            now = outGarage;
            break;
        case 255:
            now = outGarage;
            break;
        case GL1:
            now = enterGarage;
            break;
        case CR1:
            now = enterGarage;
            break;
        case GL2:
            now = enterGarage;
            break;
        case GR2:
            now = enterGarage;
            break;
        case GL3:
            now = enterGarage;
            break;
        case GR3:
            now = enterGarage;
            break;
        }
    else if (IF.ramp)
        now = Ramp;
    if (now != elementRecorder.now)
        elementType_change(now);
}

/*************************************************
Function: void recordRoadType()
Description: 路况记录状态机
Input: null
Output: null
Return: null
Others: 状态转换表：
        normal,longlongStraight,longStraight,shortStraight->all except outCorner
        enterCorner -> inCorner,outCorner,normal
        inCornerv -> outCorner,normal
        outCorner -> all

        每个状态转换到normal的条件是不同的，转换到normal相当于是找不到别的典型路况，只能先转normal，
        跑normalshift，需要具体分析
        各类直道之间的状态转换，与其他状态转直道不同
Author: BOOM7 SAMURAI_WRY
*************************************************/
void recordRoadType()
{
    // 先算avedeviation，拿来判出弯和入弯的
    DeviationVar.aveDeviation = DeviationVar.nowDeviation;

    //    if(DeviationVar.aveDeviation>SERVO_PWM_MAX) DeviationVar.aveDeviation = SERVO_PWM_MAX;
    //    if(DeviationVar.aveDeviation<-SERVO_PWM_MIN) DeviationVar.aveDeviation = -SERVO_PWM_MIN;
    DeviationVar.absDeviation[4] = DeviationVar.absDeviation[3];
    DeviationVar.absDeviation[3] = DeviationVar.absDeviation[2];
    DeviationVar.absDeviation[2] = DeviationVar.absDeviation[1];
    DeviationVar.absDeviation[1] = DeviationVar.absDeviation[0];
    DeviationVar.absDeviation[0] = abs((int)DeviationVar.aveDeviation);
    // DeviationVar.absDeviation[0] = abs((int)DeviationVar.nowDeviation);

    // diff是偏差的变化 是这一次减上一次
    DeviationVar.diffDeviation[4] = DeviationVar.diffDeviation[3];
    DeviationVar.diffDeviation[3] = DeviationVar.diffDeviation[2];
    DeviationVar.diffDeviation[2] = DeviationVar.diffDeviation[1];
    DeviationVar.diffDeviation[1] = DeviationVar.diffDeviation[0];
    DeviationVar.diffDeviation[0] = DeviationVar.absDeviation[0] - DeviationVar.absDeviation[1];

    // 下面有些指标是抄杭电的，他们判道路类型用中线为主，我们没有系统的判，速度规划用AveDeviation，现在想试试系统的判会不会好些

    // 相邻两行中线偏差和
    sumOfCenter = 0;
    // 中线所占横向范围
    range_midline = 0;
    int max = 0, min = XM;
    for (uint8 j = 0; j < 40; j++)
    {
        sumOfCenter += (II.midline[j + 1] - II.midline[j]);
        if (II.midline[j] < min)
            min = II.midline[j];
        if (II.midline[j] > max)
            max = II.midline[j];
    }
    range_midline = max - min;
    sumOfCenter = abs(sumOfCenter);

    // 底部，中间和高处各找一点，计算在横向实际差了多少cm，杭电是用点数的，而且是取固定行，我们不太适合
    // 因为我们没对中线很多的处理过，中线很容易不完整，不能取固定行，要找正确的行，做差会因找的行不同参数不同，改进一下

    min = XM;
    // 底部点，有时候最低行会没有扫到线，处理下
    for (uint8 i = 0; i < 5; i++)
        if (II.midfindflag[i])
        {
            low = i;
            break;
        }
    for (uint8 i = 0; i < 5; i++)
        if (II.leftfindflag[i] && II.rightfindflag[i])
        {
            low = i;
            break;
        }
    if (low == 0 && !II.midfindflag[0])
    {
        II.midfindflag[0] = 1;
        II.midline[0] = LAST_II.midline[0];
    }
    // 中间点，图像中部图像宽度最小的那行
    for (uint8 row = 35; row < 45 && row < II.breakY - 2; row++)
        if (II.leftfindflag[row] && II.rightfindflag[row])
        {
            uint8 width = II.rightline[row] - II.leftline[row];
            if (width < min)
            {
                mid = row;
                min = width;
            }
        }
    if (mid > II.breakY)
        mid = II.breakY;
    // 高处点，图像高部与中部点差距最大的
    if (high > II.breakY)
        high = II.breakY;
    max = 0;
    for (uint8 row = mid; row < II.breakY && row < mid + 10; row++)
        if (II.midfindflag[row])
        {
            int diff = abs((int)II.midline[row] - (int)II.midline[mid]);
            if (diff > max)
            {
                max = diff;
                high = row;
            }
        }
    // 找完点开始算评估路况的一些指标
    low2carmid = (int)II.midline[low] - myCarMid;
    mid2carmid = (int)II.midline[mid] - myCarMid;
    high2carmid = (int)II.midline[high] - myCarMid;
    k_ml = (int)(k1[low] * low2carmid - k1[mid] * mid2carmid);
    k_hm = (int)(k1[mid] * mid2carmid - k1[high] * high2carmid);
    k_ml_abs = abs(k_ml);
    k_hm_abs = abs(k_hm);
    k = k_ml + k_hm;
    k_abs = abs(k);
#ifdef BOOM7_QT_DEBUG_RECORDER
    qout << low2carmid << mid2carmid << high2carmid;
    qout << "low" << low << "mid" << mid << "high" << high;
    qout << "range_midline" << range_midline << "sumOfCenter" << sumOfCenter;
    qout << "k_ml" << k_ml << "k_hm" << k_hm;
#endif
    // 接下来判定几种典型的道路类型，不要求每帧都判出来当前是什么路况，针对状态转换的图判，准一点，作为状态转换的判据就好
    // 初始化这一帧的道路类型为未知
    roadType_now = unKnown;
    // 元素内直接赋值
    if (IF.annulus == AL1 || IF.annulus == AL2)
        roadType_now = enterLeftCorner;
    else if (IF.annulus == AL3)
        roadType_now = inLeftCorner;
    else if (IF.annulus == AL4 || IF.annulus == AL5 || IF.annulus == AL6)
        roadType_now = outLeftCorner;
    else if (IF.annulus == AR1 || IF.annulus == AR2)
        roadType_now = enterRightCorner;
    else if (IF.annulus == AR3)
        roadType_now = inRightCorner;
    else if (IF.annulus == AR4 || IF.annulus == AR5 || IF.annulus == AR6)
        roadType_now = outRightCorner;
    else if ((IF.garage == 254 || IF.garage == 255) && garageDirectionFlag == goLeft)
        roadType_now = outLeftCorner;
    else if ((IF.garage == 254 || IF.garage == 255) && garageDirectionFlag == goRight)
        roadType_now = outRightCorner;
    else if (IF.annulusDelay)
        roadType_now = unKnown;
    else if (IF.ramp || IF.rampDelay)
        roadType_now = normal;
    else if (IF.fork == FR)
    {
        if (!II.num_lm)
            roadType_now = inRightCorner;
        else
            roadType_now = enterRightCorner;
    }
    else if (!IF.fork && LAST_IF.fork == FR)
        roadType_now = outRightCorner;
    else if (IF.fork == FL)
    {
        if (!II.num_rm)
            roadType_now = inLeftCorner;
        else
            roadType_now = enterLeftCorner;
    }
    else if (!IF.fork && LAST_IF.fork == FL)
        roadType_now = outLeftCorner;
    else
    {
        switch (roadRecorder.now)
        {
        case normal:
            roadType_now = roadTypeJudge_normal();
            break;
        case longStraight:
            roadType_now = roadTypeJudge_lStraight();
            break;
        case shortStraight:
            roadType_now = roadTypeJudge_sStraight();
            break;
        case enterLeftCorner:
            roadType_now = roadTypeJudge_enterCorner(goLeft);
            break;
        case inLeftCorner:
            roadType_now = roadTypeJudge_inCorner(goLeft);
            break;
        case outLeftCorner:
            roadType_now = roadTypeJudge_outCorner(goLeft);
            break;
        case enterRightCorner:
            roadType_now = roadTypeJudge_enterCorner(goRight);
            break;
        case inRightCorner:
            roadType_now = roadTypeJudge_inCorner(goRight);
            break;
        case outRightCorner:
            roadType_now = roadTypeJudge_outCorner(goRight);
            break;
        case unKnown:
            roadType_now = roadTypeJudge_normal();
            break;
        }
    }
    if (roadRecorder.now != roadType_now && roadType_now != unKnown)
        roadType_change(roadType_now);
    roadRecorder.time++;

    // 判下s弯,判出s弯后10帧之内为s弯状态
    static int sCurveDelay = 10;
    if (roadRecorder.now != longStraight && roadRecorder.now != shortStraight && !IF.crossroad && !IF.ramp && !IF.annulus && !IF.garage)
    {
        uint8 sCurve = isSCurve();
        if (sCurve)
        {
            sCurveDelay = 10;
            roadRecorder.isSCurve = 1;
        }
        else if (sCurveDelay)
        {
            sCurveDelay--;
            roadRecorder.isSCurve = 1;
        }
        else
            roadRecorder.isSCurve = 0;
    }
    else
    {
        sCurveDelay = 10;
        roadRecorder.isSCurve = 0;
    }
}

/*************************************************
Function: uint8 aveDev_same_trend_in_n_frames(uint8 dir,uint8 n)
Description: 判断几帧内是否偏差递增或递减
Input: null
Output: null
Return: 是否偏差递增或递减
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 aveDev_same_trend_in_n_frames(uint8 dir, uint8 n)
{
    if (dir == goUp)
    {
        for (uint8 i = 0; i < n; i++)
            if (DeviationVar.diffDeviation[i] < 0)
                return 0;
        return 1;
    }
    else if (dir == goDown)
    {
        for (uint8 i = 0; i < n; i++)
            if (DeviationVar.diffDeviation[i] > 0)
                return 0;
        return 1;
    }
    return 0;
}

/*************************************************
Function: RoadType roadTypeJudge_normal()
Description: 判断是否该从一般路况转换至特殊路况
Input: null
Output: null
Return: 是否该从一般路况转换至特殊路况
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_normal()
{
    uint8 straight = isStraight();
    if (straight == 1)
        return longStraight;
    else if (straight == 2)
        return shortStraight;
    uint8 enterCorner = isEnterCorner();
    if (enterCorner == goLeft)
        return enterLeftCorner;
    else if (enterCorner == goRight)
        return enterRightCorner;
    uint8 inCorner = isInCorner();
    if (inCorner == goLeft)
        return inLeftCorner;
    else if (inCorner == goRight)
        return inRightCorner;
    return unKnown;
}

/*************************************************
Function: RoadType roadTypeJudge_lStraight()
Description: 判断是否该从长直道状态转换
Input: null
Output: null
Return: 是否该从长直道状态转换
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_lStraight()
{
    if (k_ml_abs <= 20 && DeviationVar.aveDeviation < 15 && range_midline <= 10 && (BSI.status == SAFE || BSI.status == VERY_SAFE || BSI.status == NORMAL)) // 直道
    {
        if (BSI.status != NORMAL && sumOfCenter <= 4 && (k2[II.speedTop] > 120 || II.speedTop == YM))
            return longStraight;
        // 短直道
        else if (k2[II.speedTop] > 100)
            return shortStraight;
    }

    uint8 enterCorner = isEnterCorner();
    if (enterCorner == goLeft)
        return enterLeftCorner;
    else if (enterCorner == goRight)
        return enterRightCorner;
    uint8 inCorner = isInCorner();
    if (inCorner == goLeft)
        return inLeftCorner;
    else if (inCorner == goRight)
        return inRightCorner;
    if (DeviationVar.aveDeviation > 30 || k2[II.speedTop] < 100)
        return normal;
    return unKnown;
}

/*************************************************
Function: RoadType roadTypeJudge_lStraight()
Description: 判断是否该从短直道状态转换
Input: null
Output: null
Return: 是否该从短直道状态转换
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_sStraight()
{
    uint8 straight = isStraight();
    if (straight == 1)
        return longStraight;
    else if (straight == 2)
        return shortStraight;
    uint8 enterCorner = isEnterCorner();
    if (enterCorner == goLeft)
        return enterLeftCorner;
    else if (enterCorner == goRight)
        return enterRightCorner;
    uint8 inCorner = isInCorner();
    if (inCorner == goLeft)
        return inLeftCorner;
    else if (inCorner == goRight)
        return inRightCorner;
    if (DeviationVar.aveDeviation > 30 || k2[II.speedTop] < 100)
        return normal;
    return unKnown;
}

/*************************************************
Function: RoadType roadTypeJudge_inCorner(uint8 dir)
Description: 判断是否该从弯内转换状态
Input: null
Output: null
Return: 是否该从出弯转换状态
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_enterCorner(uint8 dir)
{
    if (dir == goRight)
    {
        if (isInCorner() == goRight)
            return inRightCorner;
        if (isEnterCorner() == goLeft)
            return enterLeftCorner;
        if (k2[II.speedTop] > 160 || II.speedTop == YM || II.rnum_all > II.lnum_all + 200)
            return normal;
    }
    else if (dir == goLeft)
    {
        if (isInCorner() == goLeft)
            return inLeftCorner;
        if (isEnterCorner() == goRight)
            return enterRightCorner;
        if (k2[II.speedTop] > 160 || II.speedTop == YM || II.lnum_all > II.rnum_all + 200)
            return normal;
    }
    return unKnown;
}

/*************************************************
Function: RoadType roadTypeJudge_inCorner(uint8 dir)
Description: 判断是否该从弯内转换状态
Input: null
Output: null
Return: 是否该从出弯转换状态
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_inCorner(uint8 dir)
{

    if (dir == goLeft)
    {
        if (isOutCorner(goLeft))
            return outLeftCorner;
        if (isEnterCorner() == goRight)
            return enterRightCorner;
        if (DeviationVar.absDeviation[0] < 20 && (k2[II.speedTop] > 100 || II.speedTop == YM || (II.breakY > 50 && II.searchLineMid > myCarMid - 4)))
            return normal;
    }
    else if (dir == goRight)
    {
        if (isOutCorner(goRight))
            return outRightCorner;
        if (isEnterCorner() == goLeft)
            return enterLeftCorner;
        if (DeviationVar.absDeviation[0] < 20 && (k2[II.speedTop] > 100 || II.speedTop == YM || (II.breakY > 50 && II.searchLineMid < myCarMid + 4)))
            return normal;
    }
    return unKnown;
}

/*************************************************
Function: RoadType roadTypeJudge_outCorner(uint8 dir)
Description: 判断是否该从出弯转换状态
Input: null
Output: null
Return: 是否该从出弯转换状态
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
RoadType roadTypeJudge_outCorner(uint8 dir)
{
    ++release;
    if (dir == goRight)
    {
        uint8 straight = isStraight();
        if (straight == 1)
            return longStraight;
        else if (straight == 2)
            return shortStraight;
        uint8 enterCorner = isEnterCorner();
        if (enterCorner == goLeft)
            return enterLeftCorner;
        else if (enterCorner == goRight)
            return enterRightCorner;
        uint8 inCorner = isInCorner();
        if (inCorner == goLeft)
            return inLeftCorner;
        else if (inCorner == goRight)
            return inRightCorner;
    }
    else if (dir == goLeft)
    {
        uint8 straight = isStraight();
        if (straight == 1)
            return longStraight;
        else if (straight == 2)
            return shortStraight;
        uint8 enterCorner = isEnterCorner();
        if (enterCorner == goLeft)
            return enterLeftCorner;
        else if (enterCorner == goRight)
            return enterRightCorner;
        uint8 inCorner = isInCorner();
        if (inCorner == goLeft)
            return inLeftCorner;
        else if (inCorner == goRight)
            return inRightCorner;
    }
    if (release > 20)
    {
        release = 0;
        return normal;
    }
    return unKnown;
}

/*************************************************
Function: uint8 isStraight()
Description: 判直道
Input: null
Output: null
Return: 是否是直道
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 isStraight()
{
    if (k_ml_abs <= 10 && DeviationVar.aveDeviation < 10 && (BSI.status == SAFE || BSI.status == VERY_SAFE || BSI.status == NORMAL)) // 直道
    {
        if (BSI.status != NORMAL && IF.crossroad && (k2[II.speedTop] > 140 || II.speedTop == YM))
            return 1;
        if (BSI.status != NORMAL && sumOfCenter <= 4 && range_midline <= 5 && (k2[II.speedTop] > 140 || II.speedTop == YM))
            return 1;
        // 短直道
        else if (k_hm_abs < 30 && (k2[II.speedTop] > 100 || II.speedTop == YM))
            return 2;
    }
    return 0;
}

/*************************************************
Function: uint8 isEnterCorner()
Description: 判断是否为入弯
Input: 方向
Output: null
Return: null
Others: 是否为入弯
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 isEnterCorner()
{
    static uint8 cnt = 0;                                                                                                                         // 52
    if (k_abs > 25 && k2[II.speedTop] <= 120 && ((high - mid > 1 && k_hm_abs > 20) || (k_ml_abs > 15 && aveDev_same_trend_in_n_frames(goUp, 3)))) // 确定有弯
    {
        cnt++;
        if (cnt > 2 && k > 0 && II.num_lm == 1 && II.num_rm == 1 && II.start_lm[0] < 10 && II.start_rm[0] < 10 && II.rnum_all > 500)
            return goLeft;
        if (cnt > 2 && k < 0 && II.num_lm == 1 && II.num_rm == 1 && II.start_lm[0] < 10 && II.start_rm[0] < 10 && II.lnum_all > 500)
            return goRight;
    }
    else
        cnt = 0;
    return 0;
}

/*************************************************
Function: uint8 isInCorner()
Description: 判断是否在弯道内
Input: 方向
Output: null
Return: null
Others: 是否在弯道内
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 isInCorner()
{
    if (aveDev_same_trend_in_n_frames(goDown, 3))
        return 0;
    if (k_abs > 25 && k2[high] <= 120)
    {
        if (k > 0 && II.lnum_all < 50 && abs(kbright[0]) > 0.8)
            return goLeft;
        if (k < 0 && II.rnum_all < 50 && abs(kbleft[0]) > 0.8)
            return goRight;
    }
    if (k_abs > 30 && k2[high] <= 120 /*&& !II.dnum_all && !II.inum_all */ && DeviationVar.absDeviation[0] > 30)
    {
        if (k > 0 && II.lnum_all < 50 && II.rnum_all > 750 && abs(kbright[0]) > 0.8)
            return goLeft;
        if (k < 0 && II.rnum_all < 50 && II.lnum_all > 750 && abs(kbleft[0]) > 0.8)
            return goRight;
    }
    return 0;
}

/*************************************************
Function: uint8 isOutCorner(uint8 dir)
Description: 判断是否为出弯
Input: 方向
Output: null
Return: null
Others: 是否在弯道内
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 isOutCorner(uint8 dir)
{
    if (aveDev_same_trend_in_n_frames(goDown, 3) && (II.breakY > 52 || II.top > 55))
    {
        if ((k > 0 && dir == goLeft && abs(kbright[0]) < 0.5) || (k < 0 && dir == goRight && abs(kbleft[0]) < 0.5))
            return 1;
    }
    return 0;
}

/*************************************************
Function: uint8 isSCurve()
Description: 判断是否为s弯
Input: 方向
Output: null
Return: null
Others: 是否为s弯
Author: BOOM7 SAMURAI_WRY
*************************************************/
uint8 isSCurve()
{
    int p1, p2, p3;
    int dist1, dist2, dist3;
    int maxDist1 = 0, maxDist2 = 0, maxDist3 = 0;
    for (uint8 i = low + 1; i < mid; i++)
    {
        dist1 = abs((int)II.midline[low] - (int)II.midline[i]);
        dist2 = abs((int)II.midline[mid] - (int)II.midline[i]);
        dist3 = abs((int)II.midline[high] - (int)II.midline[i]);
        // 注意一个>= 两个 >，我想找纵向距离也最远的
        if (dist1 >= maxDist1)
        {
            maxDist1 = dist1;
            p1 = i;
        }
        if (dist2 > maxDist2)
        {
            maxDist2 = dist2;
            p2 = i;
        }
        if (dist3 > maxDist3)
        {
            maxDist3 = dist3;
            p3 = i;
        }
    }
#ifdef BOOM7_QT_DEBUG_RECORDER
    qout << "p1" << p1 << "dist" << maxDist1 << "|| p2" << p2 << "dist" << maxDist2 << "|| p3" << p3 << "dist" << maxDist3;
#endif
    // 接下来对点进行分析
    int point_low = YM, point_high = YM, point_turning = YM;
    if (maxDist1 > 3 && maxDist3 > 3 && abs(p1 - p3) < 15)
    {
        point_low = low;
        point_high = high;
        point_turning = (p1 + p3) / 2;
    }
    if (maxDist1 > 3 && maxDist2 > 3 && abs(p1 - p2) < 10)
    {
        point_low = low;
        point_high = mid;
        point_turning = (p1 + p2) / 2;
    }
//    if(maxDist2>3 && maxDist3>maxDist2 && p2 == p3 && p2>10)
//    {
//        point_low = low;
//        point_high = mid;
//        point_turning = (p1 + p2)/2;
//    }
#ifdef BOOM7_QT_DEBUG_RECORDER
    qout << "p_l" << point_low << "p_turn" << point_turning << "p_h" << point_high;
#endif
    if (point_low == YM)
        return 0;
    uint8 trend_low2turning = getMidlineTurningTrend(point_low, point_turning);
    uint8 trend_turning2high = getMidlineTurningTrend(point_turning, point_high);
#ifdef BOOM7_QT_DEBUG_RECORDER
    qout << "trend_l2t" << trend_low2turning << "trend_t2h" << trend_turning2high;
#endif
    if (trend_low2turning && trend_turning2high && trend_low2turning != trend_turning2high)
        return 1;
    return 0;
}

uint8 getMidlineTurningTrend(int p_low, int p_high)
{
    uint8 aimTrend = 0, judgedTrend = 0;
    if (II.midline[p_low] < II.midline[p_high])
        aimTrend = goRight;
    else if (II.midline[p_low] > II.midline[p_high])
        aimTrend = goLeft;
    else
        return 0;

    int cnt_increasing = 0, cnt_decreasing = 0;
    for (uint8 row = p_low; row < p_high; row++)
    {
        if (II.midline[row + 1] > II.midline[row])
            cnt_increasing++;
        if (II.midline[row + 1] < II.midline[row])
            cnt_decreasing++;
    }
    if (cnt_increasing > cnt_decreasing)
        if (cnt_decreasing == 0 || (float)cnt_increasing / (float)cnt_decreasing > 1.5)
            judgedTrend = goRight;
    if (cnt_increasing < cnt_decreasing)
        if (cnt_increasing == 0 || (float)cnt_decreasing / (float)cnt_increasing > 1.5)
            judgedTrend = goLeft;
    if (judgedTrend == aimTrend)
        return aimTrend;
    return 0;
}

/*************************************************
Function: float getRiskLevel(uint8 dir)
Description: 计算车身危险度
Input: 方向
Output: null
Return: 车身危险度
Others: 算车身线与相应map间覆盖的面积
Author: BOOM7 SAMURAI_WRY
*************************************************/
float getRiskLevel(uint8 dir)
{
    float ret = 0;
    if (dir == goLeft)
    {
        for (uint8 row = 0; row < YM / 3 * 2; row++)
        {
            if (II.leftdrawflag[row])
                ret += (bodyworkLine_left[row] - getMapXMax_Row(row, leftmap, 3)) * k1[row] * k2[row];
            else if (II.leftfindflag[row])
                ret += (bodyworkLine_left[row] - II.leftline[row]) * k1[row] * k2[row];
            else
            {
                uint8 col = getMapXMax_Row(row, leftmap, 1);
                if (col != XM)
                    ret += (bodyworkLine_left[row] - col - 1) * k1[row] * k2[row];
                else
                    ret += bodyworkLine_left[row] * k1[row] * k2[row];
            }
        }
    }
    if (dir == goRight)
    {
        for (uint8 row = 0; row < YM / 3 * 2; row++)
        {
            if (II.rightdrawflag[row])
                ret -= (bodyworkLine_right[row] - getMapXMin_Row(row, rightmap, 3)) * k1[row] * k2[row];
            else if (II.rightfindflag[row])
                ret -= (bodyworkLine_right[row] - II.rightline[row]) * k1[row] * k2[row];
            else
            {
                uint8 col = getMapXMin_Row(row, rightmap, 1);
                if (col != XM)
                    ret -= (bodyworkLine_right[row] - col + 1) * k1[row] * k2[row];
                else
                    ret -= (bodyworkLine_right[row] - XX) * k1[row] * k2[row];
            }
        }
    }
    return ret / 100; // 单位是平方分米
}

/*************************************************
Function: bodyworkSafetyAssess()
Description: 车身安全评估
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
*************************************************/
void bodyworkSafetyAssess()
{
    // 找车身线与basemap和控制线的角点
    uint8 left = YM, right = YM;
    for (uint8 row = 0; row < YM; row++)
    {
        if ((basemap[row][(int)(bodyworkLine_left[row] + 1)] || leftmap[row][(int)(bodyworkLine_left[row] + 1)]) && left == YM)
            left = row;
        if ((basemap[row][(int)(bodyworkLine_right[row] - 1)] || rightmap[row][(int)(bodyworkLine_right[row] - 1)]) && right == YM)
            right = row;
    }
    // 交点在2.2m以上，非常安全

    if (left == YM && right == YM)
        BSI.status = VERY_SAFE;
    else
    {
        uint8 dir, lower;

        if (left > right)
        {
            dir = goRight;
            lower = right;
        }
        else
        {
            dir = goLeft;
            lower = left;
        }
        //        qout<<dir<<lower<<k2[lower]<<bodyworkLine_right[lower]+0.5<<basemap[lower][(int)(bodyworkLine_right[lower]+0.5)]<<rightmap[lower][(int)(bodyworkLine_right[lower]+0.5)];
        if (k2[lower] > 200)
            BSI.status = VERY_SAFE;
        else if (k2[lower] > 80)
            BSI.status = SAFE;
        else if (k2[lower] > 40)
            BSI.status = NORMAL;
        else if (k2[lower] > 20)
        {
            if (dir == goLeft)
                BSI.status = DANGEROUS_LEFT;
            else if (dir == goRight)
                BSI.status = DANGEROUS_RIGHT;
        }
        else if (k2[lower] > 0)
        {
            if (dir == goLeft)
                BSI.status = VERY_DANGEROUS_LEFT;
            else if (dir == goRight)
                BSI.status = VERY_DANGEROUS_RIGHT;
        }
    }
}
