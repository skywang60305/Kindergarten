/*
 * adrc.c
 *
 *  Created on: 2022��7��26��
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

/**  ����ADRC����  */
Fhan_Data ADRC_Speed_Controller_l; /*!< ���� */
/**  ����ADRC����  */
Fhan_Data ADRC_Speed_Controller_r; /*!< ���� */

/**  ADRC����  */
// �����ǿ��صĲ���
float ADRC_Unit[15] =
    /*      TD����΢����               ����+״̬�۲�ESO           �Ŷ�����                 ���������*/
    /*r           h      N0     beta_01  beta_02   beta_03       I b0       beta_0   P beta_1   D beta_2    N1    C     alpha1  alpha2 ZETA(��������)*/
    {15000000.0f, 0.004, 1.0f, 240.05f, 39200.8f, 512000.46f, 1.5f, 0.0f, 80.0f, 0.05f, 5.0f, 5.0f, 1.00f, 1.00f, 20.0f};
//{10000000.0f, 0.004, 1.0f,  100.0f,  1000.0f,  3000.0f,      10.0f,      0.0f,    200.0f,    0.10f,      5.0f, 5.0f, 1.00f,  1.00f, 20.0f};

Butter_Parameter ADRC_Div_LPF_Parameter = {
    // 200---20hz
    1, -1.14298050254, 0.4128015980962,
    0.06745527388907, 0.1349105477781, 0.06745527388907};
float ADRC_LPF(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    /* ���ٶȼ�Butterworth�˲� */
    /* ��ȡ����x(n) */
    Buffer->Input_Butter[2] = curr_inputer;
    /* Butterworth�˲� */
    Buffer->Output_Butter[2] =
        Parameter->b[0] * Buffer->Input_Butter[2] + Parameter->b[1] * Buffer->Input_Butter[1] + Parameter->b[2] * Buffer->Input_Butter[0] - Parameter->a[1] * Buffer->Output_Butter[1] - Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) ���б��� */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) ���б��� */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    return (Buffer->Output_Butter[2]);
}

/**
 * @brief  �޷����
 * @param  amt              Param Describtion:��Ҫ�޷�����
 * @param  low              Param Describtion:����
 * @param  high             Param Describtion:����
 * @return float            Return Describtion:�޷�����ֵ
 */
float Constrain_Float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * @brief  �޷����
 * @param  amt              Param Describtion:��Ҫ�޷�����
 * @param  low              Param Describtion:����
 * @param  high             Param Describtion:����
 * @return int              Return Describtion:�޷�����ֵ
 */
int32 Constrain_Int32(int32 amt, int32 low, int32 high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * @brief  ���ź���
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
 * @brief  ��ʼ��ADRC����
 * @param  fhan_Input1      Param Describtion:��Ҫ��ʼ���Ľṹ��
 * @param  fhan_Input2      Param Describtion:��Ҫ��ʼ���Ľṹ��
 * @param  fhan_Input3      Param Describtion:��Ҫ��ʼ���Ľṹ��
 * @param  fhan_Input4      Param Describtion:��Ҫ��ʼ���Ľṹ��
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

    // ���ε�������Ū�ԣ�Ȼ��ϸ��
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
 * @brief  ADRC���ٸ���΢����TD���Ľ����㷨fhan
 * @param  fhan_Input       Param Describtion:����ADRC�ṹ��
 * @param  expect_ADRC      Param Describtion:����ֵ
 */
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC) // ����ADRC���ȹ���
{
    float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
    float x1_delta = 0; // ADRC״̬���������
    // ��ȡ����
    fhan_Input->r = (float)adrcParam.r * 100000;

    x1_delta = fhan_Input->x1 - expect_ADRC;                    // ��x1-v(k)���x1�õ���ɢ���¹�ʽ
    fhan_Input->h0 = fhan_Input->N0 * fhan_Input->h * 1.0f;     // ��h0���h��������ٸ���΢�����ٶȳ�������
    d = fhan_Input->r * fhan_Input->h0 * fhan_Input->h0 * 1.0f; // d=rh^2;
    a0 = fhan_Input->h0 * fhan_Input->x2 * 1.0f;                // a0=h*x2
    y = x1_delta + a0;                                          // y=x1+a0
    a1 = sqrt(d * (d + 8 * abs(y))) * 1.0f;                     // a1=sqrt(d*(d+8*abs(y))])
    a2 = (a0 + Sign_ADRC(y) * (a1 - d)) * 1.0f / 2.0f;          // a2=a0+sign(y)*(a1-d)/2;
    a = (a0 + y) * Fsg_ADRC(y, d) * 1.0f + a2 * (1 - Fsg_ADRC(y, d)) * 1.0f;
    fhan_Input->fh = -fhan_Input->r * (a / d) * 1.0f * Fsg_ADRC(a, d) - fhan_Input->r * 1.0f * Sign_ADRC(a) * (1 - Fsg_ADRC(a, d)); // �õ�����΢�ּ��ٶȸ�����
    fhan_Input->x1 += fhan_Input->h * fhan_Input->x2 * 1.0f;                                                                        // �������ٸ���״̬��x1
    fhan_Input->x2 += fhan_Input->h * fhan_Input->fh * 1.0f;                                                                        // �������ٸ���״̬��΢��x2
}

/**
 * @brief  ԭ�㸽���������Զε������ݴκ���
 * @param  e                Param Describtion:�����Ա���x
 * @param  alpha            Param Describtion:�����Գ̶�(alpha<1ʱ)
 * @param  zeta             Param Describtion:��������(y��|zeta|��Χ��Ϊһ�����Ժ���)
 * @return float            Return Describtion:���ֵy
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
 * @brief  ����״̬�۲���
 * @param  fhan_Input       Param Describtion:?
 */
void ESO_ADRC(Fhan_Data *fhan_Input)
{
    fhan_Input->e = fhan_Input->z1 - fhan_Input->y; // ״̬���

    // ��ȡ����
    fhan_Input->beta_01 = (float)adrcParam.beta01;
    fhan_Input->beta_02 = (float)adrcParam.beta02 * 100;
    fhan_Input->beta_03 = (float)adrcParam.beta03 * 1000;

    fhan_Input->fe = Fal_ADRC(fhan_Input->e, 0.5, fhan_Input->N0 * fhan_Input->h); // �����Ժ�������ȡ����״̬�뵱ǰ״̬���
    fhan_Input->fe1 = Fal_ADRC(fhan_Input->e, 0.25, fhan_Input->N0 * fhan_Input->h);

    /*************��չ״̬������**********/
    fhan_Input->z1 += fhan_Input->h * (fhan_Input->z2 - fhan_Input->beta_01 * fhan_Input->e);
    fhan_Input->z2 += fhan_Input->h * (fhan_Input->z3 - fhan_Input->beta_02 * fhan_Input->fe + fhan_Input->b0 * fhan_Input->u);
    fhan_Input->z3 += fhan_Input->h * (-fhan_Input->beta_03 * fhan_Input->fe1);
}

/**
 * @brief  ������PID
 * @param  fhan_Input       Param Describtion:
 */
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{

    // ��ȡ����
    fhan_Input->beta_1 = (float)adrcParam.beta1;
    fhan_Input->beta_2 = (float)adrcParam.beta2 / 100;

    fhan_Input->P = fhan_Input->beta_1 * Fal_ADRC(fhan_Input->e1, 0.8f, 20.01f); // С��������x
    fhan_Input->D = fhan_Input->beta_2 * fhan_Input->e2;

    fhan_Input->u0 = fhan_Input->P - fhan_Input->D + fhan_Input->I;
    fhan_Input->u0 = Constrain_Float(fhan_Input->u0, -9500, 9500);
}

/**
 * @brief  ADRC���ƹ���
 * @param  fhan_Input       Param Describtion:��
 * @param  expect_ADRC      Param Describtion:����
 * @param  feedback_ADRC    Param Describtion:����
 */
void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC)
{
    /*�Կ��ſ�������1��*/
    /*****
  ���Ź��ȹ��̣�����Ϊ����������
  ��TD����΢�����õ���
  ���������ź�x1����������΢���ź�x2
     ******/
    Fhan_ADRC(fhan_Input, expect_ADRC);
    /*�Կ��ſ�������2��*/

    /************ϵͳ���ֵΪ��������״̬������ESO����״̬�۲���������*********/
    fhan_Input->y = feedback_ADRC;
    //    fhan_Input->z1=feedback_ADRC;
    /*****
  ����״̬�۲������õ������źŵ�����״̬��
  1��״̬�ź�z1��
  2��״̬�ٶ��ź�z2��
  3��״̬���ٶ��ź�z3��
  ����z1��z2������Ϊ״̬������TD΢�ָ������õ���x1,x2�����
  ���������Ժ���ӳ�䣬����betaϵ����
  ��ϵõ�δ����״̬���ٶȹ����Ŷ�������ԭʼ������u
     *********/
    ESO_ADRC(fhan_Input);
    /*�Կ��ſ�������3��*/
    /********״̬������***/
    fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h; // ״̬������
    fhan_Input->e1 = fhan_Input->x1 - fhan_Input->z1; // ״̬ƫ����
    fhan_Input->e2 = fhan_Input->x2 - fhan_Input->z2; // ״̬΢���

    Nolinear_Conbination_ADRC(fhan_Input);
    //    /**********�Ŷ�����*******/
    fhan_Input->b0 = (float)adrcParam.b0;
    //    fhan_Input->b0 = 0;
    fhan_Input->u = fhan_Input->u0;
    //    fhan_Input->u = fhan_Input->u0 - fhan_Input->z3 / fhan_Input->b0 * 1.0f; //
    fhan_Input->u = Constrain_Float(fhan_Input->u, -6500, 6500);
}
