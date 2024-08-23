#ifndef _ATTITUDE_CALCULATION_H
#define _ATTITUDE_CALCULATION_H
#include "math.h"
#define  PI            3.14159265358979f
#define  PERIODHZ      (float)(500)       /////闂佹彃娲﹂悧杈紣閹寸姴鑺�
#define  PERIODS       (float)(0.002)           ////闂佹彃娲﹂悧�?嶅川閵�?�附鍩�

typedef struct{
  float W;
  float X;
  float Y;
  float Z;
}QuaternionTypedef;

typedef struct{
  float Pitch;  //濞ｅ浂鍨拠婵堟喆閿燂拷
  float Yaw;    //闁�?�绻�?�崺鍛喆閿燂拷
  float Roll;   //缂傚牏绮划瀵告喆閿燂拷
}EulerAngleTypedef;


typedef struct{
  float Xdata;
  float Ydata;
  float Zdata;
}AttitudeDatatypedef;

extern QuaternionTypedef    Quaternion;   //闁搞儲绋戦崢鎾�?�极閿燂�?
extern EulerAngleTypedef    EulerAngle;   //婵炲柌鍕�?�欓悷娆欐�??
extern QuaternionTypedef    AxisAngle;    //閺�?�偟顥愰～锟�?
extern EulerAngleTypedef    EulerAngleRate;//鐟滅増鎸告晶鐘测枎瑜庢刊铏�?�喆閹烘鎷烽悢宄邦�?

extern QuaternionTypedef    MeaQuaternion;
extern EulerAngleTypedef    MeaEulerAngle;
extern QuaternionTypedef    MeaAxisAngle;

extern QuaternionTypedef    ErrQuaternion;
extern EulerAngleTypedef    ErrEulerAngle;
extern QuaternionTypedef    ErrAxisAngle;
extern AttitudeDatatypedef         Acc;
extern AttitudeDatatypedef         Gyro;

extern void Quaternion_init(void);

extern void Attitude_UpdateGyro(void);

extern void Attitude_UpdateAcc(void);

#endif
