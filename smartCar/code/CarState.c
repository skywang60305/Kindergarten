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
uint16 sdSaveCnt = 0; //  SD存的次数
uint16 over3msTimeCnt = 0;
uint16 over5msTimeCnt = 0;
uint16 over8msTimeCnt = 0;

uint16 programCnt = 0;       //  程序while的次数
uint16 programOver18Cnt = 0; //  程序超过18MS的次数
float sdStability = 0;       //  SD稳定度
float programStability = 0;  //  程序稳定度
uint16 maxAllTime = 0;       //  运行过程中的最大while时间
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
    // 自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    // 一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
    mt9v03x_set_confing_buffer[1][1] = (int16)cameraDebugVAR.AUTO_EXP_TEMP;

    // 曝光时间   摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    mt9v03x_set_confing_buffer[2][1] = (int16)cameraDebugVAR.EXP_TIME_TEMP;

    // 图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    mt9v03x_set_confing_buffer[3][1] = (int16)cameraDebugVAR.FPS_TEMP;

    // 图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    mt9v03x_set_confing_buffer[8][1] = (int16)cameraDebugVAR.GAIN_TEMP;
}
