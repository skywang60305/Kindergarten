/*
 * speed.c
 *
 *  Created on: 2021��8��11��
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
Description: �ٶȲ����ṹ���ʼ��
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void speedParam_init()
{
    // ����
    SP.constantSpeed = 160;

    // ���٣������ǹ�����óɼ����ٶ�
    // �ܲ�����ҪӲ����������
    // speedk����ֻ�вο���ֵ���Լ�Ҫ���µ���
    SP.maxSpeed = 300; // ֱ������ٶ�
    SP.minSpeed = 200;
    SP.normalSpeed = 250;
    SP.curveSpeed = 215;
    // ���ٲ���
    SP.speedK = 75;  // ���ι�ʽϵ��������ɲ����ԽСɲ��Խ��
    SP.speedK2 = 75; // ��������
    SP.annulusSpeedK2 = 85;
    SP.strightSpeedCut = 20;
    SP.switchSpeedTop = 55;
    // Ԫ��
    for (int i = 0; i < 3; i++)
    {
        SP.rampUpSpeed[i] = 150;
        SP.rampOnSpeed[i] = 120;
        SP.rampDownSpeed[i] = 80;
    }
    SP.annulusSpeed = 215;

    SP.annulusMinSpeed = 200;
    SP.garageSpeed = 180; // ����
    SP.forkSpeed = 160;
    SP.outUCrossSpeed = 160;
    SP.outGarageSpeed = 160;

    SP.brakeEdge_normal = 30;
    SP.brakeEdge_curve = 30;
    SP.brakeEdge_min = 30;
}

/*************************************************
Function: trackDectionParam_init()
Description: �ٶȹ滮�ṹ���ʼ��
Input: null
Output: null
Return: null
Others: null
Author: ZJUT
 *************************************************/
void trackDectionParam_init()
{
    TDP.strightAD = 15;    // ��ֱ�����ж�
    TDP.enterCurveAD = 15; // ������ٵ��ж�

    TDP.brakeTime = 12;
    TDP.limitTime = 20;

    // brakeTest��0����1�Ĳ�����ֱ����������죬��1234ʱ��top��speed�ݼ�������ɲ��Խ��Խ��
    // ������1234��Ӧ��maxspeed�ᱻbrakeSpeed�޷�������ٶ�Ҳ��ͣ��ܶ���֮���ǵ����׺��ʵ�ɲ��ģʽ
    // ��ʵûɶ�ã�ɾ�˰�
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
Description: �ٶ���Ϣ�ṹ���ʼ��
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
Description: �ٶȷ�����ʼ��
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
Description: ��ȡɲ���ٶ�
Input: null
Output: null
Return: ɲ���ٶ�
Others: ���ݲ�ͬɲ������ȷ��speedcut�����Ǽ���ֵ��Ȼ���ڶ��ι�ʽ�����ϼ���������ɲ�����ٶ�
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
Description: �ж�ɲ���Ƿ����
Input: null
Output: null
Return: ɲ���Ƿ����
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
        // ֱ������
        if (II.speedTop >= SP.switchSpeedTop)
            speed_expected = (int16)(SP.normalSpeed - (SP.normalSpeed - SP.curveSpeed) * (YM - II.speedTop) / (YM - SP.switchSpeedTop));
        else
            speed_expected = SP.curveSpeed;
        break;
    case STRIGHT_BRAKE_CURVE:
        // ��ֱ������
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
Description: ��ȡĿ���ٶ�
Input: null
Output: null
Return: Ŀ���ٶ�
Others: null
Author: ZJUT BOOM7_WRY
 *************************************************/
int getAimSpeed()
{
#ifndef BOOM7_QT_DEBUG
    maxSpeed = SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0]; // ��ǰ�����������ٶȴ���Ǹ�
#else
    maxSpeed = nowSpeedL_SD > nowSpeedR_SD ? nowSpeedL_SD : nowSpeedR_SD; // ��ǰ�����������ٶȴ���Ǹ�
#endif
    if (runSpeedMode == CONSTANT_SPEED) // ����
        return getAimSpeed_constantSpeed();
    else if (runSpeedMode == NORMAL_SHIFT_SPEED) // ���ι�ʽ
        return getAimSpeed_normalShift();
    else // ����
        return getAimSpeed_variableSpeed();
}

/*************************************************
Function: getAimSpeed_constantSpeed()
Description: Ŀ���ٶȻ�ȡ--constantSpeed
Input: null
Output: null
Return: ������Ŀ���ٶ�
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
        // ���趨�ٶ�С���µ��ٶȵ�ʱ��ֱ�����趨���ٶȣ���Ȼ�������ºͳ�����ʱ��ɼ�����
        if (SP.constantSpeed < SP.rampUpSpeed[(judgedRampNum - 1) % rampNum])
            aimSpeed = SP.constantSpeed;
        if (1 == IF.ramp)
            aimSpeed = SP.rampUpSpeed[(judgedRampNum - 1) % rampNum];
        else if (2 == IF.ramp)
            aimSpeed = SP.rampOnSpeed[(judgedRampNum - 1) % rampNum];
        else if (3 == IF.ramp)
            aimSpeed = SP.rampDownSpeed[(judgedRampNum - 1) % rampNum];
    }
    //    else if(!IF.ramp && (angle.Pitch>8 || angle.Pitch<-6))    //��Ϊһ�����¶��� ��ǰ������״̬�����Լ�һ���±�־�����Ƕ���һ����Χ��ʱʹ��
    //        aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%judgedRampNum];   //Ҳ�������ٶȾ�����  ��������´���ȥ�����⣬��Ϊ���±����ˣ������гɳ�ֱ����
    else if (253 == IF.garage || 254 == IF.garage || 255 == IF.garage)
        // ����
        aimSpeed = SP.outGarageSpeed;
    else if (1 == IF.garage || 2 == IF.garage || 3 == IF.garage || 4 == IF.garage)
        // �г�����ɲ��
        aimSpeed = SP.garageSpeed;
    else if (5 == IF.garage || 6 == IF.garage)
        // ����ɲ��
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
Description: Ŀ���ٶȻ�ȡ--normalShift
Input: null
Output: null
Return: �����ι�ʽ�ܵ�Ŀ���ٶȣ������ٶȿ��˺����ܳ��ܺõ�·��
Others: ��normalshift�����޸ģ�֮ǰֻ���ƫ���������������ڶ����speedtop��������������֮ǰ��ɲ����һ����ٶȣ���·����һЩ
        ֻ���ƫ��������������������һ���Ǻܲ����Ե�
        �޸ģ���speedTop�������仹��Сs��speedTop����һ��ֵ��ʱ�����speedTop���٣�С�ڵ�ʱ�����ƫ�����
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
        // ����
        aimSpeed = SP.outGarageSpeed;
    else if (GL1 == IF.garage || GR1 == IF.garage || GL2 == IF.garage || GR2 == IF.garage)
        // �г�����ɲ��
        aimSpeed = SP.garageSpeed;
    else if (GL3 == IF.garage || GR3 == IF.garage)
        // �������ƶ�����ɲͣ
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
                minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // ǰ������
                aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
            }
            else
            {
#ifdef BOOM7_QT_DEBUG
                m->setText_sub("range:normal~curve");
#endif
                minSpeed = SP.curveSpeed;
                // ��ʽ�� ����� - ���ٷ�Χ * speedTop��YM�Ĳ� / �趨speedTop��YM�Ĳ�
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
                minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // ǰ������
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

        // Բ����6״̬���ٶ���һ�η�
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
        /*�ٶȴ��޷�*/
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
Description: Ŀ���ٶȻ�ȡ--variableSpeed
Input: null
Output: null
Return: �����ܵ�Ŀ���ٶ�
Others: null
        ���ٻ��Ǵ�״̬��״̬�����л��ͺ�
        ����Ҫ��
        1.������ǰ��Ҫɲ������������Ŀ���ٶȣ���Ȼ·������
        2.����ܹ�50���䣬�ٶȸպ���minspeed
        3.speedTopһ����ʱ������ֱ���ͺ�Сs��Сs���ٶ����ȱ�ֱ����һ��

        Ԥ�ڵ�Ч����


        ���䣺
        ����speedTop����
        ���ڣ�
        һ������һ���ٶȣ���ʵ��͸�ӻ�ȥ֮��Ӧ�ÿ����������İ뾶�ģ���һ�£��������������ʱ��ٵ�
        ��������ƫ��������װɣ��Ȱ������������˵
        ���ǵ�ͼ��ѹ����֮�������㲻׼���ܷ��ˣ�������б�ʻ���ƫ����ٰ�

        �����棺
        ����speedtop����normalshift���ٶȡ�

        ��Բ����
        �е�Բ����ѹ�ٶȵ���������ٶ����£�֮��ͺ�������һ����

        Сs��
        ����Сs��speedtop���ߵ���normalshift������ٶȺܿ죬��·���ж��ж�Сs�����˵����ж����г�Сs��
        ���ٶ������ơ�

        ����˼������ҪHALF_ACCELE��EXIT_CURVE_ACCELE�ˣ��������������޸ĺ��normalshift��ֱ�������ģ�
        ����������°�normalshift�����ٶȹ滮�ĸ���ϸ��һЩ��ȷʵ�°�normalshiftֱ�Ӱ�mormalspeed����
        300��ֱ��Ҳ�ܳ�ĺܿ죬���ǻᵼ�����ֲ������̵�ֱ������Сs�����̫����
        ���ױ��پ�����normalshift�ϣ������ж�����������

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

    if (IF.ramp) // �µ�
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
    else if (253 == IF.garage || 254 == IF.garage || 255 == IF.garage) // ����
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("EXIT_CURVE_ACCELE");
#endif
        aimSpeed = SP.outGarageSpeed;
        speedType = NORMAL_SHIFT;
    }
    else if (1 == IF.garage || 2 == IF.garage || 3 == IF.garage || 4 == IF.garage) // �г�����ɲ��
    {
        // �����п��򽵵�����ٶ�
        if (!LAST_IF.garage && maxSpeed > 260)
            SP.garageSpeed -= SP.strightSpeedCut;
        aimSpeed = SP.garageSpeed;
    }
    else if (5 == IF.garage || 6 == IF.garage) // ��������������ɲ��
        aimSpeed = 40;
    else if (IF.crossroad == UL1 || IF.crossroad == UR1)
        aimSpeed = SP.minSpeed;
    else if (IF.crossroad == UL2 || IF.crossroad == UL3 || IF.crossroad == UR2 || IF.crossroad == UR3)
    {
        speedType = NORMAL_SHIFT;
        aimSpeed = SP.outUCrossSpeed;
    }
    else if (IF.annulusAnticipation) // Բ��Ԥ��
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("annulusAnticipationSpeed");
#endif
        aimSpeed = SP.annulusSpeed;
        speedType = NORMAL_SHIFT;
    }
    else // ��������
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
                    minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // ǰ������
                    aimSpeed = (int16)(referenceSpeed - (referenceSpeed - minSpeed) * (YM - SI.realSpeedTop) / (YM - 30));
                }
                else
                {
#ifdef BOOM7_QT_DEBUG
                    m->setText_sub("range:normal~curve");
#endif
                    minSpeed = SP.curveSpeed;
                    // ��ʽ�� ����� - ���ٷ�Χ * speedTop��YM�Ĳ� / �趨speedTop��YM�Ĳ�
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
                    minSpeed = SP.rampUpSpeed[(judgedRampNum) % rampNum]; // ǰ������
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
            // Բ����6״̬���ٶ���һ�η�
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
            /*�ٶȴ��޷�*/
            if (aimSpeed < minSpeed)
                aimSpeed = minSpeed;
            if (SI.realSpeedTop >= SP.switchSpeedTop && aimSpeed > SP.normalSpeed)
                aimSpeed = SP.normalSpeed;
            if (SI.realSpeedTop < SP.switchSpeedTop && aimSpeed > SP.curveSpeed)
                aimSpeed = SP.curveSpeed;

            break;
        }
        // ��ֱ������
        case FULL_ACCELE:
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case FULL_ACCELE");
#endif
            if (roadRecorder.now == longStraight)
                aimSpeed = SP.maxSpeed;
            else
                aimSpeed = SP.normalSpeed;
            // �Ƿ�����������һ�µ����pid����
            break;
            // ����
        case BRAKE:
#ifdef BOOM7_QT_DEBUG
            m->setText_sub("case BRAKE");
#endif
            aimSpeed = getAimBrakeSpeed();
            // ������ɺ�һ��ʱ���normalshift
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
Description: �ٶȹ滮
Input: null
Output: null
Return: Ŀǰ�ٶ�����
Others: null
Author: ZJUT BOOM7_WRY
 *************************************************/
SpeedType getSpeedType()
{
#ifdef BOOM7_QT_DEBUG
    m->setText_sub("getSpeedType()");
#endif
    /************************** �ٶȹ滮 ******************************/
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
    if (IF.annulusDelay) // Ҳ����Բ��6״̬ ��������ֱ�� Բ��6״̬��Ϊ�������
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
        speedType = NORMAL_SHIFT; // rampDelayʱ����normalshift,�������Ը�����г������
        speedUpTime = 0;
    }
    else if (IF.fork)
    {
#ifdef BOOM7_QT_DEBUG
        m->setText_sub("fork");
#endif
        speedType = NORMAL_SHIFT; // forkʱ��normalshift,����ٶ�����һ��
        speedUpTime = 0;
    }
    else if (IF.crossroad == UL2 || IF.crossroad == UR2)
    // ����ʮ��2״̬���������
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
        // ��ֱ���
        case FULL_ACCELE:
            // ������ֱ��̵ĸ����оݣ������3�֣�ɲ����normalspeed��minspeed��curvespeed

            // ɲ����curve
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
            // ɲ����normal
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
            // ��ֱ�����е��ϵ㣬��Ϊ�ٶȻ�����ر��
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

            if (speedUpTime >= 3 && 0 == IF.rampDelay) // ��ֹ������ǰ������г�ֱ
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
            if (speedUpTime >= 6 && 0 == IF.rampDelay) // ��ֹ������ǰ������г�ֱ
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
