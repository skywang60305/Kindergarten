#ifndef ACCESSIBILITY_H
#define ACCESSIBILITY_H
#include "zf_common_headfile.h"

typedef enum
{
    NORMAL = 0,
    VERY_SAFE = 1,
    SAFE = 2,
    DANGEROUS_LEFT = 3,
    DANGEROUS_RIGHT = 4,
    VERY_DANGEROUS_LEFT = 5,
    VERY_DANGEROUS_RIGHT = 6,
} SafetyStatus;

typedef struct BodyworkSafetyIndex
{
    uint8 areaLeft;
    uint8 areaRight;
    SafetyStatus status;
} BodyworkSafetyIndex;

typedef enum
{
    longStraight,
    shortStraight,
    enterLeftCorner,
    inLeftCorner,
    outLeftCorner,
    enterRightCorner,
    inRightCorner,
    outRightCorner,
    normal,
    unKnown
} RoadType;

typedef enum
{
    annulus,
    fork,
    Ramp,
    ucross,
    cross,
    outGarage,
    enterGarage,
    null
} ElementType;
// enterAnnulus,
// inAnnulus,
// outAnnulus,
// fork,
// RampUp,
// RampOn,
// RampDown,
// inUcross,
// outUcross,
// inCross,
// outCross,
// outGarage,
// enterGarage,
// null
typedef struct
{
    RoadType now;
    RoadType before;
    RoadType next;
    RoadType all[100];
    int top;
    int time;
    int isSCurve;
} RoadRecorder;

typedef struct
{
    ElementType now;
    ElementType before;
    ElementType next;
    ElementType all[100];
    int top;
} ElementRecorder;
/*
 * 行车记录仪
 */
void drivingRecorder_init();
void drivingRecorder_record();
void recordElementType();
void recordRoadType();
void roadType_change(RoadType now);
void elementType_change(ElementType now);
uint8 aveDev_same_trend_in_n_frames(uint8 dir, uint8 n);

RoadType roadTypeJudge_normal();
RoadType roadTypeJudge_lStraight();
RoadType roadTypeJudge_sStraight();
RoadType roadTypeJudge_enterCorner(uint8 dir);
RoadType roadTypeJudge_outCorner(uint8 dir);
RoadType roadTypeJudge_inCorner(uint8 dir);
/*
 * 路况类型判断
 */
uint8 isStraight();
uint8 isEnterCorner();
uint8 isInCorner();
uint8 isOutCorner(uint8 dir);
uint8 isSCurve();
uint8 getMidlineTurningTrend(int p_low, int p_high);
/*
 * 车身及稳定性判定
 */
float getRiskLevel(uint8 dir);
void bodyworkSafetyAssess();

extern BodyworkSafetyIndex BSI_INIT, BSI;
extern int servoLimit_veryDanger;
extern RoadRecorder roadRecorder;
extern ElementRecorder elementRecorder;
extern RoadType roadType_now;
#endif // ACCESSIBILITY_H
