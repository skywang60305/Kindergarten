/*
 * speed.c
 *
 *  Created on: 2021年8月11日
 *      Author: 95159
 */

#include "speed.h"
#ifdef BOOM7_QT_DEBUG
#include "mainwindow.h"
extern int32 nowSpeedL_SD;
extern int32 nowSpeedR_SD;
extern MainWindow *m;
#endif
extern EulerAngleTypedef angle;

// from deal_img.c
extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern IMG_INFO II;
extern uint8 judgedRampNum;
extern AnnulusDEV AD;

// from control.c
extern DeviationVAR DeviationVar;
extern uint8 rampNum;

volatile SpeedParam speedParam;
volatile SpeedInfo speedInfo;
SpeedType speedType = FULL_ACCELE;
BrakeType brakeType = STRIGHT_BRAKE_NORMAL;
uint8 brakeTime = 0;
uint16 speedUpTime = 0;
float speed_accele = 0;
TrackDectionParam trackDectionParam;
uint8 limitFlag = 0;
int maxSpeed;
/*************************************************
Function: speedParam_init()
Description: 速度参数结构体初始化
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void speedParam_init()
{
    // 匀速
    SP.constantSpeed = 160;

    // 变速，这里是国赛最好成绩的速度
    // 跑不到不要硬给，慢慢来
    // speedk这种只有参考价值，自己要重新调过
    SP.maxSpeed = 300; // 直道最大速度
    SP.minSpeed = 200;
    SP.normalSpeed = 250;
    SP.curveSpeed = 215;
    // 变速参数
    SP.speedK = 75;  // 二次公式系数，用于刹车，越小刹的越快
    SP.speedK2 = 75; // 用于入弯
    SP.annulusSpeedK2 = 85;
    SP.strightSpeedCut = 20;
    SP.switchSpeedTop = 55;
    // 元素
    for (int i = 0; i < 3; i++)
    {
        SP.rampUpSpeed[i] = 150;
        SP.rampOnSpeed[i] = 120;
        SP.rampDownSpeed[i] = 80;
    }
    SP.annulusSpeed = 215;

    SP.annulusMinSpeed = 200;
    SP.garageSpeed = 180; // 出库
    SP.forkSpeed = 160;
    SP.outUCrossSpeed = 160;
    SP.outGarageSpeed = 160;

    SP.brakeEdge_normal = 30;
    SP.brakeEdge_curve = 30;
    SP.brakeEdge_min = 30;
}

/*************************************************
Function: trackDectionParam_init()
Description: 速度规划结构体初始化
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void trackDectionParam_init()
{
    TDP.strightAD = 15;    // 入直道的判断
    TDP.enterCurveAD = 15; // 入弯减速的判断

    TDP.brakeTime = 12;
    TDP.limitTime = 20;

    // brakeTest给0就用1的参数，直道能拉的最快，给1234时，top和speed递减，就是刹的越来越晚
    // 不过给1234对应的maxspeed会被brakeSpeed限幅，最高速度也会低，总而言之就是调几套合适的刹车模式
    // 其实没啥用，删了吧
    TDP.brakeTest = 0;
    TDP.brakeTop_1 = 57;
    TDP.brakeTop_2 = 56;
    TDP.brakeTop_3 = 52;
    TDP.brakeTop_4 = 51;
    TDP.brakeTop_0 = 49;

    TDP.brakeSpeedTop_1 = 54;
    TDP.brakeSpeedTop_2 = 52;
    TDP.brakeSpeedTop_3 = 51;
    TDP.brakeSpeedTop_4 = 49;
    TDP.brakeSpeedTop_0 = 47;
}

/*************************************************
Function: SpeedInfo_Init()
Description: 速度信息结构体初始化
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void SpeedInfo_Init()
{
    SI.varL[0] = 0;
    SI.varL[1] = 0;
    SI.varL[2] = 0;

    SI.varR[0] = 0;
    SI.varR[1] = 0;
    SI.varR[2] = 0;

    SI.nowSpeedL = 0;
    SI.nowSpeedR = 0;
    SI.aimSpeedL = 0;
    SI.preAimSpeedL = 0;
    SI.prepreAimSpeedL = 0;
    SI.aimSpeedR = 0;
    SI.preAimSpeedR = 0;
    SI.prepreAimSpeedR = 0;
    SI.aimSpeed = 0;

    SI.motorPWML = 0;
    SI.motorPWMR = 0;
    SI.differential = 38;
    SI.realSpeedTop = YM;
}

/*************************************************
Function: speed_init()
Description: 速度方面大初始化
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void speed_init()
{
    speedParam_init();
    SpeedInfo_Init();
    trackDectionParam_init();
}

/*************************************************
Function: getAimBrakeSpeed()
Description: 获取刹车速度
Input: null
Output: null
Return: 刹车速度
Others: 根据不同刹车类型确定speedcut，就是减速值，然后在二次公式基础上减掉，就是刹车的速度
Author: ZJUT
 *************************************************/
int getAimBrakeSpeed()
{
    int brakeSpeed;
    int speedCut;
    int maxsp;

    uint8 speedTop_TFMINI = getSpeedTopByTFMINI();
    uint8 realSpeedTop = II.speedTop < speedTop_TFMINI ? II.speedTop : speedTop_TFMINI;
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("realSpeedTop: ", realSpeedTop);
#endif
    switch (brakeType)
    {
    case STRIGHT_BRAKE_NORMAL:
        speedCut = SP.strightSpeedCut;
        if (II.speedTop >= SP.switchSpeedTop)
            maxsp = (int16)(SP.normalSpeed - (SP.normalSpeed - SP.curveSpeed) * (YM - II.speedTop) / (YM - SP.switchSpeedTop));
        else
            maxsp = SP.curveSpeed;
        break;
    case STRIGHT_BRAKE_CURVE:
        speedCut = SP.strightSpeedCut;
        maxsp = SP.curveSpeed;
        break;
    case MINSP_BRAKE:
        speedCut = SP.strightSpeedCut;
        maxsp = SP.minSpeed;
        break;
    }
    brakeSpeed = (int)(maxsp - (maxsp - SP.minSpeed) * DeviationVar.DK * DeviationVar.DK / SP.speedK / SP.speedK) - speedCut;
    if (brakeSpeed + speedCut < SP.minSpeed)
        brakeSpeed = SP.minSpeed - speedCut;
    if (brakeSpeed + speedCut > SP.normalSpeed)
        brakeSpeed = SP.normalSpeed - speedCut;
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getAimBrakeSpeed()", brakeSpeed);
#endif
    return brakeSpeed;
}

/*************************************************
Function: isBrakeFinished(int aimSpeed)
Description: 判断刹车是否完成
Input: null
Output: null
Return: 刹车是否完成
Others: null
Author: ZJUT
 *************************************************/
uint8 isBrakeFinished(BrakeType brakeType)
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("isBrakeFinished()");
#endif
    int speed_expected;
    switch (brakeType)
    {
    case STRIGHT_BRAKE_NORMAL:
        // 直道减速
        if (II.speedTop >= SP.switchSpeedTop)
            speed_expected = (int16)(SP.normalSpeed - (SP.normalSpeed - SP.curveSpeed) * (YM - II.speedTop) / (YM - SP.switchSpeedTop));
        else
            speed_expected = SP.curveSpeed;
        break;
    case STRIGHT_BRAKE_CURVE:
        // 短直道减速
        speed_expected = SP.curveSpeed;
        break;
    case MINSP_BRAKE:
        speed_expected = SP.minSpeed;
        break;
    }
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("expect:", speed_expected);
    m->setText_sub("now   :", maxSpeed);
#endif
    if (maxSpeed <= speed_expected)
        return 1;
    return 0;
}

uint8 getSpeedTopByTFMINI()
{
    uint8 speedTop = YM;
    if (!TFMINI_PLUS_DISTANCE || TFMINI_PLUS_DISTANCE > k2[YY])
        return YM;
    for (int row = YY; row > 0; row--)
        if (k2[row] < TFMINI_PLUS_DISTANCE)
        {
            speedTop = row;
            break;
        }
    return speedTop;
}

/*************************************************
Function: getAimSpeed()
Description: 获取目标速度
Input: null
Output: null
Return: 目标速度
Others: null
Author: ZJUT BOOM7_WRY
 *************************************************/
int getAimSpeed()
{
#ifndef BOOM7_QT_DEBUG
    maxSpeed = SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0]; // 当前两个轮子中速度大的那个
#else
    maxSpeed = nowSpeedL_SD > nowSpeedR_SD ? nowSpeedL_SD : nowSpeedR_SD; // 当前两个轮子中速度大的那个
#endif
    if (runSpeedMode == CONSTANT_SPEED) // 匀速
        return getAimSpeed_constantSpeed();
    else if (runSpeedMode == NORMAL_SHIFT_SPEED) // 二次公式
        return getAimSpeed_normalShift();
    else // 变速
        return getAimSpeed_variableSpeed();
}

/*************************************************
Function: getAimSpeed_constantSpeed()
Description: 目标速度获取--constantSpeed
Input: null
Output: null
Return: 匀速跑目标速度
Others: null
Author: ZJUT BOOM7_WRY
 *************************************************/
int getAimSpeed_constantSpeed()
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getAimSpeed_constantSpeed()");
#endif
    int aimSpeed = SP.constantSpeed;
    if (IF.ramp)
    {
        // 当设定速度小于坡道速度的时候，直接用设定的速度，不然上坡下坡和车抖动时变成加速了
        if (SP.constantSpeed < SP.rampUpSpeed[(judgedRampNum - 1) % rampNum])
            aimSpeed = SP.constantSpeed;
        if (1 == IF.ramp)
            aimSpeed = SP.rampUpSpeed[(judgedRampNum - 1) % rampNum];
        else if (2 == IF.ramp)
            aimSpeed = SP.rampOnSpeed[(judgedRampNum - 1) % rampNum];
        else if (3 == IF.ramp)
            aimSpeed = SP.rampDownSpeed[(judgedRampNum - 1) % rampNum];
    }
    //    else if(!IF.ramp && (angle.Pitch>8 || angle.Pitch<-6))    //因为一般下坡都是 提前结束掉状态，所以加一个新标志，当角度在一定范围内时使用
    //        aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%judgedRampNum];   //也用下坡速度就行了  解决了下坡戳出去的问题，因为下坡变速了，甚至判成长直道了
    else if (253 == IF.garage || 254 == IF.garage || 255 == IF.garage)
        // 出库
        aimSpeed = SP.outGarageSpeed;
    else if (1 == IF.garage || 2 == IF.garage || 3 == IF.garage || 4 == IF.garage)
        // 判出入库后刹车
        aimSpeed = SP.garageSpeed;
    else if (5 == IF.garage || 6 == IF.garage)
        // 入库后刹死
        aimSpeed = 40;
    else if (IF.annulusAnticipation)
    {
        if (SP.annulusSpeed < SP.constantSpeed)
            aimSpeed = SP.annulusSpeed;
        else
            aimSpeed = SP.constantSpeed;
    }
    else if (IF.annulus)
        aimSpeed = SP.annulusMinSpeed;
    else if (IF.crossroad == UL2 || IF.crossroad == UR2 || IF.crossroad == UL3 || IF.crossroad == UR3)
        aimSpeed = SP.outUCrossSpeed;
    else if (IF.fork)
        aimSpeed = SP.forkSpeed;
    else
        aimSpeed = SP.constantSpeed;
    return aimSpeed;
}

/*************************************************
Function: getAimSpeed_normalShift()
Description: 目标速度获取--normalShift
Input: null
Output: null
Return: 纯二次公式跑的目标速度，不过速度快了很难跑出很好的路径
Others: 对normalshift进行修改，之前只针对偏差计算减速量，现在多加上speedtop，能让他在入弯之前就刹到低一点的速度，让路径好一些
        只针对偏差计算减速量，在入弯那一段是很不明显的
        修改：用speedTop区分入弯还是小s，speedTop大于一定值的时候根据speedTop变速，小于的时候根据偏差变速
Author: ZJUT BOOM7_WRY
 *************************************************/
int getAimSpeed_normalShift()
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getAimSpeed_normalShift()");
#endif
    int aimSpeed = SP.minSpeed;
    uint8 speedTop_TFMINI = getSpeedTopByTFMINI();
    SI.realSpeedTop = II.speedTop < speedTop_TFMINI ? II.speedTop : speedTop_TFMINI;

    uint16 referenceSpeed;
    uint16 minSpeed;
    uint16 speedK2;
    if (IF.ramp)
    {
        if (1 == IF.ramp)
            aimSpeed = SP.rampUpSpeed[(judgedRampNum - 1) % rampNum];
        else if (2 == IF.ramp)
            aimSpeed = SP.rampOnSpeed[(judgedRampNum - 1) % rampNum];
        else if (3 == IF.ramp)
            aimSpeed = SP.rampDownSpeed[(judgedRampNum - 1) % rampNum];
    }
    else if (253 == IF.garage || 254 == IF.garage || 255 == IF.garage)
        // 出库
        aimSpeed = SP.outGarageSpeed;
    else if (GL1 == IF.garage || GR1 == IF.garage || GL2 == IF.garage || GR2 == IF.garage)
        // 判出入库后刹车
        aimSpeed = SP.garageSpeed;
    else if (GL3 == IF.garage || GR3 == IF.garage)
        // 入库后缓慢移动至判刹停
        aimSpeed = 40;
    else if (IF.annulusAnticipation)
        aimSpeed = SP.annulusSpeed;
    else if (IF.crossroad == UL1 || IF.crossroad == UR1 || IF.crossroad == UL2 || IF.crossroad == UR2 || IF.crossroad == UL3 || IF.crossroad == UR3)
        aimSpeed = SP.outUCrossSpeed;
    else
    {
        if (IF.annulus)
        {
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("annulus");
#endif
            referenceSpeed = SP.annulusSpeed;
            minSpeed = SP.annulusMinSpeed;
            speedK2 = SP.annulusSpeedK2;
            aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * DeviationVar.DK * DeviationVar.DK / speedK2 / speedK2);
        }
        else if (SI.realSpeedTop >= SP.switchSpeedTop && !roadRecorder.isSCurve)
        {
            referenceSpeed = SP.normalSpeed;
            if (SI.realSpeedTop != II.speedTop)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("range:normal~rampUp");
#endif
                minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // 前面有坡
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
            }
            else
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("range:normal~curve");
#endif
                minSpeed = SP.curveSpeed;
                // 公式： 最高速 - 变速范围 * speedTop到YM的差 / 设定speedTop到YM的差
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - SP.switchSpeedTop));
            }
        }
        else
        {
            referenceSpeed = SP.curveSpeed;
            if (SI.realSpeedTop != II.speedTop)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("range:curve~rampUp");
#endif
                minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // 前面有坡
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
            }
            else
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("range:curve~min");
#endif
                minSpeed = SP.minSpeed;
                speedK2 = SP.speedK2;
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * DeviationVar.DK * DeviationVar.DK / speedK2 / speedK2);
                //                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * DeviationVar.nowDeviation * DeviationVar.nowDeviation / speedK2 / speedK2);
            }
        }

        // 圆环的6状态对速度限一次幅
        if (IF.annulusDelay)
        {
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("annulusDelay");
#endif
            float DK = AD.sumDK / AD.cnt;
            int16 annulusSpeed = (int16)(SP.annulusSpeed - (SP.annulusSpeed - SP.annulusMinSpeed) * DK * DK / SP.annulusSpeedK2 / SP.annulusSpeedK2);
            annulusSpeed = annulusSpeed > SP.annulusMinSpeed ? annulusSpeed : SP.annulusMinSpeed;
            aimSpeed = aimSpeed < annulusSpeed ? aimSpeed : annulusSpeed;
        }
        /*速度大限幅*/
        if (aimSpeed < minSpeed)
            aimSpeed = minSpeed;
        if (SI.realSpeedTop >= SP.switchSpeedTop && aimSpeed > SP.normalSpeed)
            aimSpeed = SP.normalSpeed;
        if (SI.realSpeedTop < SP.switchSpeedTop && aimSpeed > SP.curveSpeed)
            aimSpeed = SP.curveSpeed;
    }
    return aimSpeed;
}

/*************************************************
Function: getAimSpeed_variableSpeed()
Description: 目标速度获取--variableSpeed
Input: null
Output: null
Return: 变速跑的目标速度
Others: null
        变速还是大状态机状态不断切换就好
        几个要求：
        1.在入弯前就要刹车到过这个弯的目标速度，不然路径不好
        2.最好能够50的弯，速度刚好是minspeed
        3.speedTop一样的时候，区分直道和和小s，小s的速度求稳比直道慢一点

        预期的效果：


        入弯：
        根据speedTop来吧
        弯内：
        一种曲率一种速度，其实逆透视回去之后，应该可以算出来弯的半径的，试一下，杭电他们用曲率变速的
        ，我们是偏差，各有利弊吧，先把曲率算出来再说
        我们的图像压缩过之后曲率算不准，很烦人，还是用斜率或者偏差变速吧

        入三叉：
        修正speedtop并用normalshift算速度。

        入圆环：
        判到圆环就压速度到环内最高速度以下，之后就和弯内是一样的

        小s：
        避免小s中speedtop过高导致normalshift算出来速度很快，在路况判定中对小s进行了单独判定，判出小s则
        对速度做限制。

        经过思考，不要HALF_ACCELE和EXIT_CURVE_ACCELE了，这两个都是我修改后的normalshift能直接做到的：
        变速相对于新版normalshift，对速度规划的更加细致一些，确实新版normalshift直接把mormalspeed给到
        300，直道也能冲的很快，但是会导致那种不长不短的直道或者小s，冲的太快了
        这套变速就是在normalshift上，经过判定后再允许冲刺

Author: ZJUT BOOM7_WRY
 *************************************************/
int getAimSpeed_variableSpeed()
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getAimSpeed_variableSpeed()");
#endif
    int aimSpeed = SP.minSpeed;
    uint8 speedTop_TFMINI = getSpeedTopByTFMINI();
    SI.realSpeedTop = II.speedTop < speedTop_TFMINI ? II.speedTop : speedTop_TFMINI;

    if (IF.ramp) // 坡道
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("rampSpeed");
#endif
        if (1 == IF.ramp)
            aimSpeed = SP.rampUpSpeed[(judgedRampNum - 1) % rampNum];
        else if (2 == IF.ramp)
            aimSpeed = SP.rampOnSpeed[(judgedRampNum - 1) % rampNum];
        else if (3 == IF.ramp)
            aimSpeed = SP.rampDownSpeed[(judgedRampNum - 1) % rampNum];
        speedType = NORMAL_SHIFT;
    }
    else if (253 == IF.garage || 254 == IF.garage || 255 == IF.garage) // 出库
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("EXIT_CURVE_ACCELE");
#endif
        aimSpeed = SP.outGarageSpeed;
        speedType = NORMAL_SHIFT;
    }
    else if (1 == IF.garage || 2 == IF.garage || 3 == IF.garage || 4 == IF.garage) // 判出入库后刹车
    {
        // 高速判库则降低入库速度
        if (!LAST_IF.garage && maxSpeed > 260)
            SP.garageSpeed -= SP.strightSpeedCut;
        aimSpeed = SP.garageSpeed;
    }
    else if (5 == IF.garage || 6 == IF.garage) // 入库后慢慢调节至刹死
        aimSpeed = 40;
    else if (IF.crossroad == UL1 || IF.crossroad == UR1)
        aimSpeed = SP.minSpeed;
    else if (IF.crossroad == UL2 || IF.crossroad == UL3 || IF.crossroad == UR2 || IF.crossroad == UR3)
    {
        speedType = NORMAL_SHIFT;
        aimSpeed = SP.outUCrossSpeed;
    }
    else if (IF.annulusAnticipation) // 圆环预判
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("annulusAnticipationSpeed");
#endif
        aimSpeed = SP.annulusSpeed;
        speedType = NORMAL_SHIFT;
    }
    else // 正常变速
    {
        speedType = getSpeedType();
        switch (speedType)
        {
        case NORMAL_SHIFT:
        {
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case NORMAL_SHIFT");
#endif
            uint16 referenceSpeed;
            uint16 minSpeed;
            uint16 speedK2;
            if (IF.annulus)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("annulus");
#endif
                referenceSpeed = SP.annulusSpeed;
                minSpeed = SP.annulusMinSpeed;
                speedK2 = SP.annulusSpeedK2;
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * DeviationVar.DK * DeviationVar.DK / speedK2 / speedK2);
            }
            else if (SI.realSpeedTop >= SP.switchSpeedTop && !roadRecorder.isSCurve)
            {
                referenceSpeed = SP.normalSpeed;
                if (SI.realSpeedTop != II.speedTop)
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("range:normal~rampUp");
#endif
                    minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // 前面有坡
                    aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("range:normal~curve");
#endif
                    minSpeed = SP.curveSpeed;
                    // 公式： 最高速 - 变速范围 * speedTop到YM的差 / 设定speedTop到YM的差
                    aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - SP.switchSpeedTop));
                }
            }
            else
            {
                referenceSpeed = SP.curveSpeed;
                if (SI.realSpeedTop != II.speedTop)
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("range:curve~rampUp");
#endif
                    minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // 前面有坡
                    aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("range:curve~min");
#endif
                    minSpeed = SP.minSpeed;
                    speedK2 = SP.speedK2;
                    aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * DeviationVar.DK * DeviationVar.DK / speedK2 / speedK2);
                }
            }
            // 圆环的6状态对速度限一次幅
            if (IF.annulusDelay)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("NORMAL_SHIFT annulusDelay");
#endif
                float DK = AD.sumDK / AD.cnt;
                int16 annulusSpeed = (int16)(SP.annulusSpeed - (SP.annulusSpeed - SP.annulusMinSpeed) * DK * DK / SP.annulusSpeedK2 / SP.annulusSpeedK2);
                annulusSpeed = annulusSpeed > SP.annulusMinSpeed ? annulusSpeed : SP.annulusMinSpeed;
                aimSpeed = aimSpeed < annulusSpeed ? aimSpeed : annulusSpeed;
            }
            /*速度大限幅*/
            if (aimSpeed < minSpeed)
                aimSpeed = minSpeed;
            if (SI.realSpeedTop >= SP.switchSpeedTop && aimSpeed > SP.normalSpeed)
                aimSpeed = SP.normalSpeed;
            if (SI.realSpeedTop < SP.switchSpeedTop && aimSpeed > SP.curveSpeed)
                aimSpeed = SP.curveSpeed;

            break;
        }
        // 长直道拉满
        case FULL_ACCELE:
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case FULL_ACCELE");
#endif
            if (roadRecorder.now == longStraight)
                aimSpeed = SP.maxSpeed;
            else
                aimSpeed = SP.normalSpeed;
            // 是否可以在这里调一下电机的pid参数
            break;
            // 减速
        case BRAKE:
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case BRAKE");
#endif
            aimSpeed = getAimBrakeSpeed();
            // 减速完成后一段时间变normalshift
            if (isBrakeFinished(brakeType))
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("++brakeTime");
#endif
                ++brakeTime;
            }
            if (brakeTime >= TDP.brakeTime)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("BRAKE->NORMAL_SHIFT");
#endif
                brakeTime = 0;
                speedUpTime = 0;
                speedType = NORMAL_SHIFT;
            }
            break;
        default:
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case default");
#endif
            aimSpeed = SP.minSpeed;
            break;
        }

        int16 maxSpeed = SP.maxSpeed;
        if (aimSpeed > maxSpeed)
            aimSpeed = maxSpeed;
    }
    return aimSpeed;
}

/*************************************************
Function: SpeedType getSpeedType()
Description: 速度规划
Input: null
Output: null
Return: 目前速度类型
Others: null
Author: ZJUT BOOM7_WRY
 *************************************************/
SpeedType getSpeedType()
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getSpeedType()");
#endif
    /************************** 速度规划 ******************************/
    uint16 brakeTop, brakeSpeedTop;
    switch (TDP.brakeTest)
    {
    case 1:
        brakeTop = TDP.brakeTop_1;
        brakeSpeedTop = TDP.brakeSpeedTop_1;
        break;
    case 2:
        brakeTop = TDP.brakeTop_2;
        brakeSpeedTop = TDP.brakeSpeedTop_2;
        break;
    case 3:
        brakeTop = TDP.brakeTop_3;
        brakeSpeedTop = TDP.brakeSpeedTop_3;
        break;
    case 4:
        brakeTop = TDP.brakeTop_4;
        brakeSpeedTop = TDP.brakeSpeedTop_4;
        break;
    default:
        brakeTop = TDP.brakeTop_1;
        brakeSpeedTop = TDP.brakeSpeedTop_1;
        break;
    }
    if (IF.annulusDelay) // 也就是圆环6状态 出环必有直道 圆环6状态改为出弯加速
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("annulusDelay");
#endif
        speedType = NORMAL_SHIFT;
        speedUpTime = 0;
    }
    else if (IF.annulus)
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("annulus");
#endif
        speedType = NORMAL_SHIFT;
        speedUpTime = 0;
        if (IF.annulus && LAST_IF.annulus == 0)
        {
            limitFlag = 1;
        }
    }
    else if (IF.rampDelay)
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("rampDelay");
#endif
        speedType = NORMAL_SHIFT; // rampDelay时给他normalshift,这样可以更快的判出弯加速
        speedUpTime = 0;
    }
    else if (IF.fork)
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("fork");
#endif
        speedType = NORMAL_SHIFT; // fork时给normalshift,最高速度限制一下
        speedUpTime = 0;
    }
    else if (IF.crossroad == UL2 || IF.crossroad == UR2)
    // 往返十字2状态给出弯加速
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("unilateralCross NORMAL_SHIFT");
#endif
        speedUpTime = 0;
        speedType = NORMAL_SHIFT;
    }

    else
    {
        switch (speedType)
        {
        // 长直冲刺
        case FULL_ACCELE:
            // 结束长直冲刺的各种判据，结果有3种，刹车到normalspeed、minspeed、curvespeed

            // 刹车到curve
            if ((roadRecorder.now == enterLeftCorner || roadRecorder.now == enterRightCorner || roadRecorder.now == inLeftCorner || roadRecorder.now == inRightCorner))
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("enter or in corner");
#endif
                if (maxSpeed >= SP.curveSpeed)
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("BRAKE");
#endif
                    speedType = BRAKE;
                    brakeType = STRIGHT_BRAKE_CURVE;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("NORMAL");
#endif
                    speedType = NORMAL_SHIFT;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
            }
            // 刹车到normal
            else if (roadRecorder.now == shortStraight || II.breakY <= brakeTop || SI.realSpeedTop <= brakeSpeedTop || II.breakY <= TDP.brakeTop_0 || SI.realSpeedTop <= TDP.brakeSpeedTop_0 || DeviationVar.absDeviation[0] >= TDP.strightAD)
            {
                if (maxSpeed > SP.normalSpeed + SP.brakeEdge_normal)
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("Brake to normal");
#endif
                    speedType = BRAKE;
                    brakeType = STRIGHT_BRAKE_NORMAL;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("Turn to normal");
#endif
                    speedType = NORMAL_SHIFT;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
            }
            else if (II.speedTop != SI.realSpeedTop)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("ramp anticipation");
#endif
                if (maxSpeed > SP.minSpeed + SP.brakeEdge_min)
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("Brake to min");
#endif
                    speedType = BRAKE;
                    brakeType = MINSP_BRAKE;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("Turn to normal");
#endif
                    speedType = NORMAL_SHIFT;
                    speedUpTime = 0;
                    limitFlag = 1;
                }
            }

            break;
        case NORMAL_SHIFT:
            // 长直加速判的严点，因为速度会给的特别大
            if ((II.breakY >= YY && SI.realSpeedTop >= YY - 1 && II.searchLineMid > myCarMid - 5 && II.searchLineMid <= II.searchLineMid + 5 &&
                 DeviationVar.absDeviation[0] <= TDP.strightAD &&
                 DeviationVar.absDeviation[1] <= TDP.strightAD &&
                 DeviationVar.absDeviation[2] <= TDP.strightAD) &&
                II.speedTop == SI.realSpeedTop)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("++speedUpTime");
#endif
                ++speedUpTime;
            }
            else if (speedUpTime)
                speedUpTime = 0;

            if (speedUpTime >= 3 && 0 == IF.rampDelay) // 防止下坡提前解除误判长直
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("FULL_ACCELE");
#endif
                speedType = FULL_ACCELE;
            }
            break;
        case BRAKE:
            if ((II.breakY >= YY - 1 && SI.realSpeedTop >= YY - 2 && II.searchLineMid > myCarMid - 5 && II.searchLineMid <= II.searchLineMid + 5 &&
                 DeviationVar.absDeviation[0] <= TDP.strightAD &&
                 DeviationVar.absDeviation[1] <= TDP.strightAD &&
                 DeviationVar.absDeviation[2] <= TDP.strightAD) &&
                II.speedTop == SI.realSpeedTop)
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("BRAKE ++speedUpTime");
#endif
                ++speedUpTime;
            }
            if (speedUpTime >= 6 && 0 == IF.rampDelay) // 防止下坡提前解除误判长直
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("BRAKE TO FULL_ACCELE");
#endif
                speedType = FULL_ACCELE;
                brakeTime = 0;
            }
            break;
        }
    }
    return speedType;
}
