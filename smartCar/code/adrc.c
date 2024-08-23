/*
 * adrc.c
 *
 *  Created on: 2022年7月26日
 *      Author: TinyHang
 */
/**
 * @file ADRC.c
 * @brief
 * @author Wcp (1042249823@qq.com)
 * @version 1.0
 * @date 2022-01-07
 *
 * @copyright Copyright (c) 2022  ILCE
 *
 * @par   LOG:
 *        Date            Version         Author          Description
 *      2022-01-07
 */
#include "ADRC.h"
#include <stdbool.h>

ADRC_Param adrcParam;

/**  左轮ADRC控制  */
Fhan_Data ADRC_Speed_Controller_l; /*!< 左轮 */
/**  右轮ADRC控制  */
Fhan_Data ADRC_Speed_Controller_r; /*!< 右轮 */

/**  ADRC参数  */
// 这里是空载的参数
float ADRC_Unit[15] =
    /*      TD跟踪微分器               扩张+状态观测ESO           扰动补偿                 非线性组合*/
    /*r           h      N0     beta_01  beta_02   beta_03       I b0       beta_0   P beta_1   D beta_2    N1    C     alpha1  alpha2 ZETA(线性区间)*/
    {15000000.0f, 0.004, 1.0f, 240.05f, 39200.8f, 512000.46f, 1.5f, 0.0f, 80.0f, 0.05f, 5.0f, 5.0f, 1.00f, 1.00f, 20.0f};
//{10000000.0f, 0.004, 1.0f,  100.0f,  1000.0f,  3000.0f,      10.0f,      0.0f,    200.0f,    0.10f,      5.0f, 5.0f, 1.00f,  1.00f, 20.0f};

Butter_Parameter ADRC_Div_LPF_Parameter = {
    // 200---20hz
    1, -1.14298050254, 0.4128015980962,
    0.06745527388907, 0.1349105477781, 0.06745527388907};
float ADRC_LPF(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    /* 加速度计Butterworth滤波 */
    /* 获取最新x(n) */
    Buffer->Input_Butter[2] = curr_inputer;
    /* Butterworth滤波 */
    Buffer->Output_Butter[2] =
        Parameter->b[0] * Buffer->Input_Butter[2] + Parameter->b[1] * Buffer->Input_Butter[1] + Parameter->b[2] * Buffer->Input_Butter[0] - Parameter->a[1] * Buffer->Output_Butter[1] - Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) 序列保存 */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) 序列保存 */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    return (Buffer->Output_Butter[2]);
}

/**
 * @brief  限幅输出
 * @param  amt              Param Describtion:需要限幅的数
 * @param  low              Param Describtion:下限
 * @param  high             Param Describtion:上限
 * @return float            Return Describtion:限幅后数值
 */
float Constrain_Float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * @brief  限幅输出
 * @param  amt              Param Describtion:需要限幅的数
 * @param  low              Param Describtion:下限
 * @param  high             Param Describtion:上限
 * @return int              Return Describtion:限幅后数值
 */
int32 Constrain_Int32(int32 amt, int32 low, int32 high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * @brief  符号函数
 * @param  Input            Param Describtion:x
 * @return int16          Return Describtion:y
 */
int16 Sign_ADRC(float Input)
{
    int16 output = 0;
    if (Input > 1E-6f)
        output = 1;
    else if (Input < -1E-6f)
        output = -1;
    else
        output = 0;
    return output;
}

int16 Fsg_ADRC(float x, float d)
{
    int16 output = 0;
    output = (Sign_ADRC(x + d) - Sign_ADRC(x - d)) * 1.0f / 2 * 1.0f;
    return output;
}

/**
 * @brief  初始化ADRC参数
 * @param  fhan_Input1      Param Describtion:需要初始化的结构体
 * @param  fhan_Input2      Param Describtion:需要初始化的结构体
 * @param  fhan_Input3      Param Describtion:需要初始化的结构体
 * @param  fhan_Input4      Param Describtion:需要初始化的结构体
 */
void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2)
{
    fhan_Input1->r = ADRC_Unit[0];
    fhan_Input1->h = ADRC_Unit[1];
    fhan_Input1->N0 = (uint16)(ADRC_Unit[2]);
    fhan_Input1->beta_01 = ADRC_Unit[3];
    fhan_Input1->beta_02 = ADRC_Unit[4];
    fhan_Input1->beta_03 = ADRC_Unit[5];
    fhan_Input1->b0 = ADRC_Unit[6];
    fhan_Input1->beta_0 = ADRC_Unit[7];
    fhan_Input1->beta_1 = ADRC_Unit[8];
    fhan_Input1->beta_2 = ADRC_Unit[9];
    fhan_Input1->N1 = (uint16)(ADRC_Unit[10]);
    fhan_Input1->c = ADRC_Unit[11];

    fhan_Input1->alpha1 = ADRC_Unit[12];
    fhan_Input1->alpha2 = ADRC_Unit[13];
    fhan_Input1->zeta = ADRC_Unit[14];

    fhan_Input2->r = ADRC_Unit[0];
    fhan_Input2->h = ADRC_Unit[1];
    fhan_Input2->N0 = (uint16)(ADRC_Unit[2]);
    fhan_Input2->beta_01 = ADRC_Unit[3];
    fhan_Input2->beta_02 = ADRC_Unit[4];
    fhan_Input2->beta_03 = ADRC_Unit[5];
    fhan_Input2->b0 = ADRC_Unit[6];
    fhan_Input2->beta_0 = ADRC_Unit[7];
    fhan_Input2->beta_1 = ADRC_Unit[8];
    fhan_Input2->beta_2 = ADRC_Unit[9];
    fhan_Input2->N1 = (uint16)(ADRC_Unit[10]);
    fhan_Input2->c = ADRC_Unit[11];

    fhan_Input2->alpha1 = ADRC_Unit[12];
    fhan_Input2->alpha2 = ADRC_Unit[13];
    fhan_Input2->zeta = ADRC_Unit[14];

    // 调参的数量级弄对，然后细调
    // r:100
    adrcParam.r = ADRC_Unit[0] / 100000;
    adrcParam.beta01 = ADRC_Unit[3];
    adrcParam.beta02 = ADRC_Unit[4] / 100;
    adrcParam.beta03 = ADRC_Unit[5] / 1000;
    adrcParam.beta1 = ADRC_Unit[8];
    adrcParam.beta2 = ADRC_Unit[9] * 100;
    adrcParam.b0 = ADRC_Unit[6];
}

/**
 * @brief  ADRC最速跟踪微分器TD，改进的算法fhan
 * @param  fhan_Input       Param Describtion:所用ADRC结构体
 * @param  expect_ADRC      Param Describtion:期望值
 */
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC) // 安排ADRC过度过程
{
    float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
    float x1_delta = 0; // ADRC状态跟踪误差项
    // 读取参数
    fhan_Input->r = (float)adrcParam.r * 100000;

    x1_delta = fhan_Input->x1 - expect_ADRC;                    // 用x1-v(k)替代x1得到离散更新公式
    fhan_Input->h0 = fhan_Input->N0 * fhan_Input->h * 1.0f;     // 用h0替代h，解决最速跟踪微分器速度超调问题
    d = fhan_Input->r * fhan_Input->h0 * fhan_Input->h0 * 1.0f; // d=rh^2;
    a0 = fhan_Input->h0 * fhan_Input->x2 * 1.0f;                // a0=h*x2
    y = x1_delta + a0;                                          // y=x1+a0
    a1 = sqrt(d * (d + 8 * abs(y))) * 1.0f;                     // a1=sqrt(d*(d+8*abs(y))])
    a2 = (a0 + Sign_ADRC(y) * (a1 - d)) * 1.0f / 2.0f;          // a2=a0+sign(y)*(a1-d)/2;
    a = (a0 + y) * Fsg_ADRC(y, d) * 1.0f + a2 * (1 - Fsg_ADRC(y, d)) * 1.0f;
    fhan_Input->fh = -fhan_Input->r * (a / d) * 1.0f * Fsg_ADRC(a, d) - fhan_Input->r * 1.0f * Sign_ADRC(a) * (1 - Fsg_ADRC(a, d)); // 得到最速微分加速度跟踪量
    fhan_Input->x1 += fhan_Input->h * fhan_Input->x2 * 1.0f;                                                                        // 跟新最速跟踪状态量x1
    fhan_Input->x2 += fhan_Input->h * fhan_Input->fh * 1.0f;                                                                        // 跟新最速跟踪状态量微分x2
}

/**
 * @brief  原点附近有连线性段的连续幂次函数
 * @param  e                Param Describtion:输入自变量x
 * @param  alpha            Param Describtion:非线性程度(alpha<1时)
 * @param  zeta             Param Describtion:线性区间(y在|zeta|范围内为一次线性函数)
 * @return float            Return Describtion:输出值y
 */
float Fal_ADRC(float e, float alpha, float zeta)
{
    int16 s = 0;
    float fal_output = 0;
    s = (Sign_ADRC(e + zeta) - Sign_ADRC(e - zeta)) * 1.0f / 2.0f;
    fal_output = e * 1.0f * s / (powf(zeta, 1 - alpha)) + powf(abs(e), alpha) * Sign_ADRC(e) * (1 - s) * 1.0f;
    return fal_output;
}

/**
 * @brief  扩张状态观测器
 * @param  fhan_Input       Param Describtion:?
 */
void ESO_ADRC(Fhan_Data *fhan_Input)
{
    fhan_Input->e = fhan_Input->z1 - fhan_Input->y; // 状态误差

    // 读取参数
    fhan_Input->beta_01 = (float)adrcParam.beta01;
    fhan_Input->beta_02 = (float)adrcParam.beta02 * 100;
    fhan_Input->beta_03 = (float)adrcParam.beta03 * 1000;

    fhan_Input->fe = Fal_ADRC(fhan_Input->e, 0.5, fhan_Input->N0 * fhan_Input->h); // 非线性函数，提取跟踪状态与当前状态误差
    fhan_Input->fe1 = Fal_ADRC(fhan_Input->e, 0.25, fhan_Input->N0 * fhan_Input->h);

    /*************扩展状态量更新**********/
    fhan_Input->z1 += fhan_Input->h * (fhan_Input->z2 - fhan_Input->beta_01 * fhan_Input->e);
    fhan_Input->z2 += fhan_Input->h * (fhan_Input->z3 - fhan_Input->beta_02 * fhan_Input->fe + fhan_Input->b0 * fhan_Input->u);
    fhan_Input->z3 += fhan_Input->h * (-fhan_Input->beta_03 * fhan_Input->fe1);
}

/**
 * @brief  非线性PID
 * @param  fhan_Input       Param Describtion:
 */
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{

    // 读取参数
    fhan_Input->beta_1 = (float)adrcParam.beta1;
    fhan_Input->beta_2 = (float)adrcParam.beta2 / 100;

    fhan_Input->P = fhan_Input->beta_1 * Fal_ADRC(fhan_Input->e1, 0.8f, 20.01f); // 小误差大增益x
    fhan_Input->D = fhan_Input->beta_2 * fhan_Input->e2;

    fhan_Input->u0 = fhan_Input->P - fhan_Input->D + fhan_Input->I;
    fhan_Input->u0 = Constrain_Float(fhan_Input->u0, -9500, 9500);
}

/**
 * @brief  ADRC控制过程
 * @param  fhan_Input       Param Describtion:略
 * @param  expect_ADRC      Param Describtion:期望
 * @param  feedback_ADRC    Param Describtion:反馈
 */
void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC)
{
    /*自抗扰控制器第1步*/
    /*****
  安排过度过程，输入为期望给定，
  由TD跟踪微分器得到：
  过度期望信号x1，过度期望微分信号x2
     ******/
    Fhan_ADRC(fhan_Input, expect_ADRC);
    /*自抗扰控制器第2步*/

    /************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
    fhan_Input->y = feedback_ADRC;
    //    fhan_Input->z1=feedback_ADRC;
    /*****
  扩张状态观测器，得到反馈信号的扩张状态：
  1、状态信号z1；
  2、状态速度信号z2；
  3、状态加速度信号z3。
  其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
  经过非线性函数映射，乘以beta系数后，
  组合得到未加入状态加速度估计扰动补偿的原始控制量u
     *********/
    ESO_ADRC(fhan_Input);
    /*自抗扰控制器第3步*/
    /********状态误差反馈率***/
    fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h; // 状态积分项
    fhan_Input->e1 = fhan_Input->x1 - fhan_Input->z1; // 状态偏差项
    fhan_Input->e2 = fhan_Input->x2 - fhan_Input->z2; // 状态微分项，

    Nolinear_Conbination_ADRC(fhan_Input);
    //    /**********扰动补偿*******/
    fhan_Input->b0 = (float)adrcParam.b0;
    //    fhan_Input->b0 = 0;
    fhan_Input->u = fhan_Input->u0;
    //    fhan_Input->u = fhan_Input->u0 - fhan_Input->z3 / fhan_Input->b0 * 1.0f; //
    fhan_Input->u = Constrain_Float(fhan_Input->u, -6500, 6500);
}
