#include "CarState.h"

volatile RunState runState = STOP;
volatile StopReason stopReason;
volatile RunMode runMode = NORMAL_RUN;
volatile RunSpeedMode runSpeedMode = CONSTANT_SPEED;
int16 buzzerTime = 0;
CAMERA_DEBUG_VAR cameraDebugVAR;

ProgramRunTime programRunTime;
uint32 runTimeTemp = 0;
CoreStatus cpu0 = WORKING;
CoreStatus cpu1 = WAITING;
uint8 call_bmx055_flag = 0;
uint16 sdSaveCnt = 0; //  SD��Ĵ���
uint16 over3msTimeCnt = 0;
uint16 over5msTimeCnt = 0;
uint16 over8msTimeCnt = 0;

uint16 programCnt = 0;       //  ����while�Ĵ���
uint16 programOver18Cnt = 0; //  ���򳬹�18MS�Ĵ���
float sdStability = 0;       //  SD�ȶ���
float programStability = 0;  //  �����ȶ���
uint16 maxAllTime = 0;       //  ���й����е����whileʱ��
uint8 img_prepared = 0;
uint32 tickCnt;

void time_init()
{
    PRT.maxWhileTime = 0;
    PRT.aveWhileTime = 0;
    PRT.whileTime = 0;
    PRT._whileTime = 0;
    PRT.maxGoTime = 0;
    PRT.aveGoTime = 0;
    PRT.goTime = 0;
    PRT.allTime = 0;
    PRT.sdSaveTime = 0;
    PRT.vcanTime = 0;
}

void cameraDebugVarInit()
{
    cameraDebugVAR.AUTO_EXP_TEMP = 0;
    cameraDebugVAR.EXP_TIME_TEMP = 512;
    cameraDebugVAR.FPS_TEMP = 50;
    cameraDebugVAR.GAIN_TEMP = 32;
}
// int16 mt9v03x_set_confing_buffer[MT9V03X_CONFIG_FINISH][2];
void cameraVarInitFromEeprom()
{
    // �Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
    // һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
    mt9v03x_set_confing_buffer[1][1] = (int16)cameraDebugVAR.AUTO_EXP_TEMP;

    // �ع�ʱ��   ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
    mt9v03x_set_confing_buffer[2][1] = (int16)cameraDebugVAR.EXP_TIME_TEMP;

    // ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
    mt9v03x_set_confing_buffer[3][1] = (int16)cameraDebugVAR.FPS_TEMP;

    // ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�
    mt9v03x_set_confing_buffer[8][1] = (int16)cameraDebugVAR.GAIN_TEMP;
}
