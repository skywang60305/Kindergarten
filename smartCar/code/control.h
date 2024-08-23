#ifndef CONTROL_H
#define CONTROL_H
#include "zf_common_headfile.h"

#ifdef BOOM7_QT_DEBUG
typedef enum ControlLine
{
    BOTHLINE,
    LEFTLINE,
    RIGHTLINE,
    MIDLINE,
    NONELINE,
    CONTROLLINE
} ControlLine;
#endif

#define MAXSPEED 200 // ����ܱ�ס��������٣��ܱ�ס�������ȶ������·�����ٶ�,��������ٶ��Ѿ����ܿ�����������ˣ�Ҳ��û�ж�̬�ı�Ҫ�ˣ�
#define MINSPEED 150 // �ܵĺ�����·��������ͬ������ٶ�,Ҳ����˵˿��û�д򻬣���������Ҳ��ȫ�����ϣ��ͳ�����̥����е�ʹ��붼�йص�

typedef struct DeviationVAR
{
    float K;
    float B;
    float DK;
    float lastK;
    float lastB;
    float nowDeviation;
    float lastDeviation;
    float aveDeviation;
    float aveDeviationLeft;
    float aveDeviationRight;
    int absDeviation[5];
    int diffDeviation[5];
} DeviationVAR;

typedef struct DeviationParamKB
{
    uint16 baseB;
    uint16 proportion_dd_Ramp;
    uint16 proportion_dd_ucross;
    uint16 proportion_max;
    uint16 proportion_min;
    uint16 proportion_enterAnnulus[5];
    uint16 proportion_isAnnulus[5];
    uint16 proportion_outAnnulus[5];
    uint16 proportion_ramp;
    uint16 proportion_onRamp;
    uint16 proportion_fork;
    uint16 proportion_enterGarage;
    uint16 proportion_outGarage;
} DeviationParamKB;

void deviationParam_init(void);

/*
 * �����Ǽ���
 */
void directionControl(void);

/*
 * ������ѡȡ
 */
ControlLine chooseControlLineByLineNum(uint8 lineNumL, uint8 lineNumR);
ControlLine chooseControlLineByMapNum(uint16 mapNumL, uint16 mapNumR);
ControlLine getRampControlLine();

/*
 * ������͸�ӱ任������ʵ����ϵ�µ�ֱ�߷���
 */
uint8 getLine(float kb[2], ControlLine cl);
uint8 get_K(float *k, ControlLine cl);
float get_dist_bottom();
uint8 getRampLine(float kb[2], ControlLine cl);
float getAveDeviation(ControlLine cl);
void getDevParam(float *kk, float *bb, float *dd);
void adjustParamByAimspeed(float *kk);

// float getCurvatureRadius(ControlLine cl);
// int collinear(double a[2],double b[2],double c[2]);//�ж������Ƿ��ߣ����߷���1
// double curvature(double a[2],double b[2],double c[2]);

extern uint8 annulusNum;
extern uint8 rampNum;
extern ControlLine controlLine;
extern DeviationVAR DeviationVar;
extern DeviationParamKB KbParam;
extern uint8 needToOutGarage;
extern float kk, bb;
extern float kbleft[2];
extern float kbright[2];
extern float kbmid[2];
#endif // CONTROL_H
