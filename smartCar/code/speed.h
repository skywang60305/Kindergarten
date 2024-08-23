/*
 * speed.h
 *
 *  Created on: 2021年8月11日
 *      Author: 95159
 */

#ifndef CODE_SPEED_H_
#define CODE_SPEED_H_

#include "zf_common_headfile.h"
#define SI speedInfo
#define SP speedParam
#define TDP trackDectionParam

typedef struct SpeedInfo
{
    int varL[3];   // 编码器值
    int varR[3];   // 编码器值
    int nowSpeedL; // 左轮实际速度
    int nowSpeedR; // 右轮实际速度
    int aimSpeed;  // 目标速度
    int aimSpeedL; // 左轮目标速度（算上差速的）
    int aimSpeedR; // 右轮目标速度（算上差速的）
    int preAimSpeedL;
    int prepreAimSpeedL;
    int preAimSpeedR;
    int prepreAimSpeedR;
    int motorPWML;       // 左轮PWM
    int motorPWMR;       // 右轮PWM
    uint16 differential; // 差速
    uint8 realSpeedTop;
} SpeedInfo;

typedef enum SpeedType
{
    NORMAL_SHIFT, // 二次公式
    FULL_ACCELE,  // 直线全速
    BRAKE,        // 刹车
} SpeedType;

typedef enum BrakeType
{
    STRIGHT_BRAKE_NORMAL, // 刹车到NORMAL
    STRIGHT_BRAKE_CURVE,  // 刹车到CURVE
    MINSP_BRAKE,
} BrakeType;

typedef struct SpeedParam
{
    uint16 maxSpeed; // 直道最大速度   长直道速度
    uint16 minSpeed; // 最小速度 (全部的)

    uint16 normalSpeed; // normalShift变速时非弯道的最高速度，variableSpeed时非加速状态下的最高速度
    uint16 curveSpeed;  // 二次公式弯道最高速度

    uint16 speedK;         // 刹车系数
    uint16 speedK2;        // 二次公式系数
    uint16 annulusSpeedK2; // 圆环二次公式系数

    // 各元素的速度
    uint16 constantSpeed;    // 匀速
    uint16 annulusSpeed;     // 圆环速度
    uint16 annulusMinSpeed;  // 圆环最小速度
    uint16 outUCrossSpeed;   // 往返十字出口减速
    uint16 forkSpeed;        // 三叉速度
    uint16 rampUpSpeed[3];   // 过坡道速度
    uint16 rampOnSpeed[3];   // 坡上速度
    uint16 rampDownSpeed[3]; // 下坡速度
    uint16 outGarageSpeed;   // 出库速度
    uint16 garageSpeed;      // 判到入库速时的刹车速度

    uint16 switchSpeedTop;   // normalShift变速时切换最高速的的行数
    uint16 strightSpeedCut;  // 长直道刹车减速
    uint16 brakeEdge_normal; // 刹车时，当前速度期望速度加上brakeEdge时，才执行刹车
    uint16 brakeEdge_curve;
    uint16 brakeEdge_min;
} SpeedParam;

typedef struct TrackDectionParam
{
    uint16 strightAD;    // 入直道的判断
    uint16 enterCurveAD; // 入弯减速的判断

    uint16 brakeTime; // 刹车时间
    uint16 limitTime; // 砍积分时间（没用到）

    uint16 brakeTest; // 刹车类型
    uint16 brakeTop_1;
    uint16 brakeTop_2;
    uint16 brakeTop_3;
    uint16 brakeTop_4;
    uint16 brakeTop_0;

    uint16 brakeSpeedTop_1; // 各种刹车时的TOP点
    uint16 brakeSpeedTop_2;
    uint16 brakeSpeedTop_3;
    uint16 brakeSpeedTop_4;
    uint16 brakeSpeedTop_0;

} TrackDectionParam;
void speed_init(void);
void speedParam_init(void);
void SpeedInfo_Init(void);
void trackDectionParam_init(void);
SpeedType getSpeedType(void);
uint8 isBrakeFinished(BrakeType brakeType);
int getAimBrakeSpeed();
uint8 getSpeedTopByTFMINI();

int getAimSpeed(void);
int getAimSpeed_constantSpeed();
int getAimSpeed_normalShift();
int getAimSpeed_variableSpeed();
int myNewGetAimSpeed_variableSpeed();

extern volatile SpeedParam SP;
extern volatile SpeedInfo SI;
extern SpeedType speedType;
extern BrakeType brakeType;
extern TrackDectionParam trackDectionParam;
extern uint8 limitFlag;
#endif /* CODE_SPEED_H_ */
