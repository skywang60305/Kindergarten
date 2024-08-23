/*
 * myMotor.h
 *
 *  Created on: 2022年5月10日
 *      Author: 95159
 */

#ifndef CODE_MOTORCONTROLALGORITHM_H_
#define CODE_MOTORCONTROLALGORITHM_H_

#include "zf_common_headfile.h"

typedef enum MotorControlAlgorithm
{
        PositionalPID,
        IncrementalPID,
        VariableStructurePID,
        ZjutPID,
        ADRC,
} MotorControlAlgorithm;

// 单纯的PID,位置式增量式都可以用
typedef struct
{
        float target;
        float feedBack;
        float error;
        float preError;
        float prePreError;

        float integral;
        float derivative;

        float output;
} PID_Calc;

typedef struct
{
        uint16 kp;      // 比例参数
        uint16 ki;      // 积分参数
        uint16 kd;      // 微分参数
        uint16 i_limit; // 积分限幅
} PID_Param;

typedef struct
{
        uint16 kp_base;
        uint16 ki_base;
        uint16 range_kp;
        uint16 decayRate_kp;
        uint16 decayRate_ki;
} Param_VariableStructurePID;

typedef struct
{
        uint16 kp;
        uint16 ki;
        uint16 kd;
        uint16 change_ki;
        uint16 change_kib;
} Param_ZJUTPID;

void motorPIDInit();
void motorPIDParamInit();

int calcMotorPID_positional(PID_Calc *PID, PID_Param *param);
int calcMotorPID_Incremental(PID_Calc *PID, PID_Param *param);
int calcMotorPID_VariableStructure(PID_Calc *PID, Param_VariableStructurePID *param);
int calcMotorPID_ZJUT(PID_Calc *PID, Param_ZJUTPID *param);

extern MotorControlAlgorithm motorControlAlgorithm;
extern PID_Calc positionalPIDLeft, positionalPIDRight;
extern PID_Param positionalPIDParam;

extern PID_Calc incrementalPIDLeft, incrementalPIDRight;
extern PID_Param incrementalPIDParam;

extern PID_Calc variableStructurePIDLeft, variableStructurePIDRight;
extern Param_VariableStructurePID variableStructurePIDParam;

extern PID_Calc zjutPIDLeft, zjutPIDRight;
extern Param_ZJUTPID zjutPIDParam;

extern uint32 cnt_motorPID_period;
extern double motor_sumError_left;
extern double motor_sumError_right;

#endif /* CODE_MOTORCONTROLALGORITHM_H_ */
