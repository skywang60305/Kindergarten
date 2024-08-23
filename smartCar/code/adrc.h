#ifndef _ADRC_H_
#define _ADRC_H_

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>

// ��Ҫ��Ҫ���ļ�������
typedef struct
{
  uint16 r;
  uint16 beta01;
  uint16 beta02;
  uint16 beta03;
  uint16 b0;
  uint16 beta1;
  uint16 beta2;
} ADRC_Param;

typedef struct
{
  float Input_Butter[3];
  float Output_Butter[3];
} Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
} Butter_Parameter;

typedef struct
{
  /*****���Ź��ȹ���*******/
  float x1;  // ����΢����״̬��
  float x2;  // ����΢����״̬��΢����
  float r;   // ʱ��߶�
  float h;   // ADRCϵͳ����ʱ��
  uint16 N0; // ����΢��������ٶȳ���h0=N*h (���� ���ɼ����㵽�趨ֵ)

  float h0;
  float fh; // ����΢�ּ��ٶȸ�����

  /*****����״̬�۲���*******/
  /******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
  float z1;
  float z2;
  float z3; // ���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
  float e;  // ϵͳ״̬���
  float y;  // ϵͳ�����
  float fe;
  float fe1;
  float beta_01;
  float beta_02;
  float beta_03;

  /**********ϵͳ״̬������*********/
  float e0; // ״̬��������
  float e1; // ״̬ƫ��
  float e2; // ״̬��΢����
  float u0; // ���������ϵͳ���
  float u;  // ���Ŷ�����������
  float Last_e1;
  float P;
  float I;
  float D;
  uint16 nei;
  uint16 wai;
  Butter_BufferData ADRC_LPF_Buffer; // ��������ͨ�����������
  /*********��һ�������ʽ*********/
  float beta_0; // ����
  float beta_1; // ��������ϲ���
  float beta_2; // u0=beta_1*e1+beta_2*e2+(beta_0*e0);

  /*********�ڶ��������ʽ*********/
  float alpha1; // u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
  float alpha2; // 0<alpha1<1<alpha2
  float zeta;   // ���Զε����䳤��

  /*********�����������ʽ*********/
  float h1;  // u0=-fhan(e1,e2,r,h1);
  uint16 N1; // ����΢��������ٶȳ���h0=N*h

  /*********�����������ʽ*********/
  float c;  // u0=-fhan(e1,c*e2*e2,r,h1);
  float b0; // �Ŷ�����

} Fhan_Data;

extern Fhan_Data ADRC_Speed_Controller_l;
extern Fhan_Data ADRC_Speed_Controller_r;
extern Fhan_Data ADRC_ControlOutL;
extern Fhan_Data ADRC_ControlOutR;

int32 Constrain_Int32(int32 amt, int32 low, int32 high);

void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2);

void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC);
void ESO_ADRC(Fhan_Data *fhan_Input);
void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback);

float Fal_ADRC(float e, float alpha, float zeta);

/**  ����ADRC����  */
extern Fhan_Data ADRC_Speed_Controller_l; /*!< ���� */
/**  ����ADRC����a  */
extern Fhan_Data ADRC_Speed_Controller_r; /*!< ���� */

extern ADRC_Param adrcParam;
#endif
