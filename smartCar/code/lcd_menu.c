/*
 * lcd_menu.c
 *
 *  Created on: 2021��8��11��
 *      Author: 95159
 */

#define PAGE_DISP_NUM 7
#include "lcd_menu.h"

uint32 annulusSize[5] = {70, 70, 70, 70, 70};
uint32 proportion_enter_50 = 380;
uint32 proportion_in_50 = 380;
uint32 proportion_out_50 = 380;

uint32 proportion_enter_60 = 380;
uint32 proportion_in_60 = 380;
uint32 proportion_out_60 = 380;

uint32 proportion_enter_70 = 380;
uint32 proportion_in_70 = 380;
uint32 proportion_out_70 = 380;

uint32 proportion_enter_80 = 380;
uint32 proportion_in_80 = 380;
uint32 proportion_out_80 = 380;

uint32 proportion_enter_90 = 380;
uint32 proportion_in_90 = 380;
uint32 proportion_out_90 = 380;

uint32 proportion_enter_100 = 380;
uint32 proportion_in_100 = 380;
uint32 proportion_out_100 = 380;

// �µ�����궨��ͱ��������֣����û�ʱ����ˣ�4�֣�ƽ�ºͼ��£�������ֱ������������������4���ٶ�
// �����ٶȵ�ʱ���������ǰ�ǲ��ǳ�ֱ�Լ���һ�¾��У����³�ֱ���°�rampup����㣬���¿��Լ��ٵ�
#define RAMP_GENTLE 40
#define RAMP_BIG 30
#define RAMP_MID 20
#define RAMP_SMALL 10

uint32 rampSize[3] = {RAMP_MID, RAMP_MID, RAMP_MID};
// ����ֱ
uint32 rampUp_gentle = 150;
uint32 rampOn_gentle = 120;
uint32 rampDown_gentle = 80;

// ������
uint32 rampUp_big = 150;
uint32 rampOn_big = 120;
uint32 rampDown_big = 80;

// �⣬ֱ
uint32 rampUp_mid = 160;
uint32 rampOn_mid = 90;
uint32 rampDown_mid = 180;

// �⣬��
uint32 rampUp_small = 150;
uint32 rampOn_small = 90;
uint32 rampDown_small = 140;

uint32 visualScope_enable = 0;
uint32 aimSpeedL_visualScope = 300;
uint32 aimSpeedR_visualScope = 300;
uint32 addspeed = 10;

// ��Ҫ���ԵĲ�������������uint8
// ������һҳһҳ��̵� 8λ�ı���������������Ϳ��ܻ���ɴ���һ��16λ������ʱ�� ��һҳ
// ֻ����һ���������� �ͻῨס ����Ҫ���͸ɴ�ȫ��32λ�� 8λ�ĵ��η�Χ�͵�255Ҳ�е�С
uint32 *EEPROM_DATA[] =
    {
        // for Menu_RunMode
        (uint32 *)&runTime,
        (uint32 *)&runMode,
        // for Menu_SpeedRunModePara
        (uint32 *)&runSpeedMode,
        // for Menu_Servo
        (uint32 *)&KbParam.proportion_dd_Ramp,
        (uint32 *)&KbParam.proportion_dd_ucross,
        (uint32 *)&KbParam.baseB,
        (uint32 *)&KbParam.proportion_max,
        (uint32 *)&KbParam.proportion_min,
        (uint32 *)&servoPID.kp_ramp,
        (uint32 *)&servoPID.kp_onRamp,
        (uint32 *)&servoPID.kp,
        (uint32 *)&servoPID.kp_max,
        (uint32 *)&servoPID.kp_min,
        (uint32 *)&servoPID.kd_in,
        (uint32 *)&servoPID.kd_out,
        (uint32 *)&servoPID.kd_max,
        (uint32 *)&servoPID.kd_min,
        (uint32 *)&servoPID.differential,
        (uint32 *)&servoPID.taw,
        (uint32 *)&SERVO_PWM_MID,
        (uint32 *)&SERVO_PWM_MAX,
        (uint32 *)&SERVO_PWM_MIN,
        // for Menu_Fork
        (uint32 *)&KbParam.proportion_fork,
        (uint32 *)&KbParam.proportion_onRamp,
        (uint32 *)&dir_fork,
        // for Menu_Garage
        (uint32 *)&needToOutGarage,
        (uint32 *)&judgedCnt,
        (uint32 *)&garageDirectionFlag,
        (uint32 *)&KbParam.proportion_outGarage,
        (uint32 *)&KbParam.proportion_enterGarage,
        (uint32 *)&SP.garageSpeed,
        (uint32 *)&stopCarCnt,
        // for Menu_SpeedPara
        (uint32 *)&SP.constantSpeed,
        (uint32 *)&SP.maxSpeed,
        (uint32 *)&SP.minSpeed,
        (uint32 *)&SP.curveSpeed,
        (uint32 *)&SP.normalSpeed,
        (uint32 *)&SP.forkSpeed,
        (uint32 *)&SP.strightSpeedCut,
        (uint32 *)&SP.switchSpeedTop,
        (uint32 *)&SP.speedK,
        (uint32 *)&SP.speedK2,
        (uint32 *)&SP.annulusSpeedK2,
        (uint32 *)&SP.brakeEdge_normal,
        (uint32 *)&SP.brakeEdge_curve,
        (uint32 *)&SP.brakeEdge_min,
        (uint32 *)&SP.outUCrossSpeed,
        (uint32 *)&SI.differential,
        (uint32 *)&SP.outGarageSpeed,
        // for Menu_TrackDetection
        (uint32 *)&TDP.brakeTest,
        (uint32 *)&TDP.strightAD,
        (uint32 *)&TDP.enterCurveAD,
        (uint32 *)&TDP.brakeTop_1,
        (uint32 *)&TDP.brakeTop_2,
        (uint32 *)&TDP.brakeTop_3,
        (uint32 *)&TDP.brakeTop_4,
        (uint32 *)&TDP.brakeTop_0,
        (uint32 *)&TDP.brakeSpeedTop_1,
        (uint32 *)&TDP.brakeSpeedTop_2,
        (uint32 *)&TDP.brakeSpeedTop_3,
        (uint32 *)&TDP.brakeSpeedTop_4,
        (uint32 *)&TDP.brakeSpeedTop_0,
        (uint32 *)&TDP.brakeTime,
        // for Menu_Annulus
        (uint32 *)&annulusNum,
        (uint32 *)&annulusSize[0],
        (uint32 *)&annulusSize[1],
        (uint32 *)&annulusSize[2],
        (uint32 *)&annulusSize[3],
        (uint32 *)&annulusSize[4],
        (uint32 *)&SP.annulusMinSpeed,
        (uint32 *)&SP.annulusSpeed,
        (uint32 *)&proportion_enter_50,
        (uint32 *)&proportion_in_50,
        (uint32 *)&proportion_out_50,
        (uint32 *)&proportion_enter_60,
        (uint32 *)&proportion_in_60,
        (uint32 *)&proportion_out_60,
        (uint32 *)&proportion_enter_70,
        (uint32 *)&proportion_in_70,
        (uint32 *)&proportion_out_70,
        (uint32 *)&proportion_enter_80,
        (uint32 *)&proportion_in_80,
        (uint32 *)&proportion_out_80,
        (uint32 *)&proportion_enter_90,
        (uint32 *)&proportion_in_90,
        (uint32 *)&proportion_out_90,
        (uint32 *)&proportion_enter_100,
        (uint32 *)&proportion_in_100,
        (uint32 *)&proportion_out_100,
        // for Menu_Ramp
        (uint32 *)&rampNum,
        (uint32 *)&rampSize[0],
        (uint32 *)&rampSize[1],
        (uint32 *)&rampSize[2],
        (uint32 *)&rampUp_gentle,
        (uint32 *)&rampOn_gentle,
        (uint32 *)&rampDown_gentle,
        (uint32 *)&rampUp_big,
        (uint32 *)&rampOn_big,
        (uint32 *)&rampDown_big,
        (uint32 *)&rampUp_mid,
        (uint32 *)&rampOn_mid,
        (uint32 *)&rampDown_mid,
        (uint32 *)&rampUp_small,
        (uint32 *)&rampOn_small,
        (uint32 *)&rampDown_small,
        (uint32 *)&KbParam.proportion_ramp,
        (uint32 *)&threshold_TFmini,
        // for Menu_Motor
        (uint32 *)&motorControlAlgorithm,
        (uint32 *)&positionalPIDParam.kp,
        (uint32 *)&positionalPIDParam.ki,
        (uint32 *)&positionalPIDParam.kd,
        (uint32 *)&positionalPIDParam.i_limit,

        (uint32 *)&incrementalPIDParam.kp,
        (uint32 *)&incrementalPIDParam.ki,
        (uint32 *)&incrementalPIDParam.kd,
        (uint32 *)&incrementalPIDParam.i_limit,

        (uint32 *)&variableStructurePIDParam.decayRate_ki,
        (uint32 *)&variableStructurePIDParam.decayRate_kp,
        (uint32 *)&variableStructurePIDParam.ki_base,
        (uint32 *)&variableStructurePIDParam.kp_base,
        (uint32 *)&variableStructurePIDParam.range_kp,

        (uint32 *)&zjutPIDParam.kp,
        (uint32 *)&zjutPIDParam.ki,
        (uint32 *)&zjutPIDParam.kd,
        (uint32 *)&zjutPIDParam.change_ki,
        (uint32 *)&zjutPIDParam.change_kib,

        (uint32 *)&adrcParam.r,
        (uint32 *)&adrcParam.beta01,
        (uint32 *)&adrcParam.beta02,
        (uint32 *)&adrcParam.beta03,
        (uint32 *)&adrcParam.beta1,
        (uint32 *)&adrcParam.beta2,
        (uint32 *)&adrcParam.b0,

        // for Menu_Sun
        (uint32 *)&OSTU_MIN,
        (uint32 *)&OSTU_MAX,
        // for Menu_Camera
        (uint32 *)&cameraDebugVAR.AUTO_EXP_TEMP,
        (uint32 *)&cameraDebugVAR.EXP_TIME_TEMP,
        (uint32 *)&cameraDebugVAR.FPS_TEMP,
        (uint32 *)&cameraDebugVAR.GAIN_TEMP,
        (uint32 *)&highOstuGain,
        (uint32 *)&highOstuRow,
        (uint32 *)&drawLine_distance_fork,
        (uint32 *)&drawLine_distance_annulus,
        (uint32 *)&drawLine_distance_ucross,
        (uint32 *)&drawLine_distance_enterGarage,
        (uint32 *)&brake_distance_ucross,
        (uint32 *)&ucross_manual_setting,
        (uint32 *)&dir_ucross_manual_setting_1,
        (uint32 *)&dir_ucross_manual_setting_2,
        (uint32 *)&zoom,
        (uint32 *)&rigor_ucross_both,
        (uint32 *)&rigor_ucross_oneSide,
};

//----------------------------------   ���˵�   -------------------------------

// һ���˵�
MENU_PRMT MainMenu_Prmt;

MENU_TABLE MainMenu_Table[] =
    {
        {(uint8 *)"out Garage?    ", Menu_Null, (uint32 *)&needToOutGarage},
        {(uint8 *)"1.RunMode      ", Menu_RunMode, NULL},        //  �ܵ�ģʽ
        {(uint8 *)"2.Servo        ", Menu_Servo, NULL},          //  ���
        {(uint8 *)"3.Element      ", Menu_Element, NULL},        //  ���
        {(uint8 *)"4.SpeedPara    ", Menu_SpeedPara, NULL},      //  �ٶȲ���
        {(uint8 *)"5.SpJudge      ", Menu_TrackDetection, NULL}, //  ɲ������
        {(uint8 *)"6.Motor        ", Menu_Motor, NULL},          //  ���
                                                                 //        {(uint8 *)"7.SunParam     ", Menu_Sun, NULL},             //  �������
        {(uint8 *)"7.CameraParam  ", Menu_Camera, NULL},         //  ����ͷ����
        {(uint8 *)"8.sdCard       ", Menu_sdCardParam, NULL},
        {(uint8 *)"9.WriteFlash   ", Menu_WriteFlash, NULL}, //  ��FLASH
        {(uint8 *)"10.ReadFlash   ", Menu_ReadFlash, NULL},  //  ��FLASH
};

// �����˵�1  С��ģʽѡ��
MENU_PRMT RunMode_Prmt;

MENU_TABLE RunMode_MenuTable[] =
    {
        {(uint8 *)"1.runTime *0.1s ", Menu_Null, (uint32 *)&runTime}, // ��ʱ������ʱ��
        {(uint8 *)"2.NORMAL_RUN    ", Menu_NORMAL_RUN, NULL},         // ����ʱ��
        {(uint8 *)"3.TIMING_RUN    ", Menu_TIMING_RUN, NULL},         // ��ʱ��
        {(uint8 *)" SpeedRunMode   ", Menu_Null, NULL},               // ƫ����㷽ʽ
        {(uint8 *)"1.VARIABLE      ", Menu_VARIABLE_SPEED, NULL},     //  ����
        {(uint8 *)"2.CONSTANT      ", Menu_CONSTANT_SPEED, NULL},     //  ����
        {(uint8 *)"3.NORMAL_SHIFT  ", Menu_NORMAL_SHIFT_SPEED, NULL}, //  ���ι�ʽ
};

void Menu_RunMode(void)
{
    lcd_clear_all(WHITE);
    Site_t site;
    site.x = 120;
    site.y = (2 + runMode) * 16;
    m_lcd_showstr(site.x, site.y, "  * ", RED, WHITE);
    site.x = 120;
    site.y = (5 + runSpeedMode) * 16;
    lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
    m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    uint8 menuNum;
    menuNum = sizeof(RunMode_MenuTable) / sizeof(RunMode_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=   RunMode   =- ", &RunMode_Prmt, RunMode_MenuTable, menuNum);
}

void Menu_NORMAL_RUN()
{
    if (runMode != NORMAL_RUN)
    {
        Site_t site;
        site.x = 120;
        site.y = (2 + runMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runMode = NORMAL_RUN;

        site.x = 120;
        site.y = (2 + runMode) * 16;
        m_lcd_showstr(site.x, site.y, "  * ", RED, WHITE);
    }
}

// �������ö�ʱ��
void Menu_TIMING_RUN()
{
    if (runMode != TIMING_RUN)
    {
        Site_t site;
        site.x = 120;
        site.y = (2 + runMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runMode = TIMING_RUN;

        site.x = 120;
        site.y = (2 + runMode) * 16;
        m_lcd_showstr(site.x, site.y, "  * ", RED, WHITE);
    }
}

// ��������  �ܵ�ģʽ �� ������
void Menu_CONSTANT_SPEED()
{
    if (runSpeedMode != CONSTANT_SPEED)
    {
        Site_t site;
        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runSpeedMode = CONSTANT_SPEED;

        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}

// ������
void Menu_VARIABLE_SPEED()
{
    if (runSpeedMode != VARIABLE_SPEED)
    {
        Site_t site;
        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runSpeedMode = VARIABLE_SPEED;

        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}
// ���ι�ʽ��
void Menu_NORMAL_SHIFT_SPEED()
{
    if (runSpeedMode != NORMAL_SHIFT_SPEED)
    {
        Site_t site;
        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runSpeedMode = NORMAL_SHIFT_SPEED;

        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}

void Menu_ADC_SPEED()
{
    if (runSpeedMode != ADC_SPEED)
    {
        Site_t site;
        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        lcd_clear(site.x, site.y, TFT_X_MAX - 1, site.y + 15, WHITE);
        runSpeedMode = ADC_SPEED;

        site.x = 120;
        site.y = (5 + runSpeedMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}

// �����˵�  �������
MENU_PRMT Servo_Prmt;

MENU_TABLE Servo_MenuTable[] =
    {
        {(uint8 *)"1.baseB        ", Menu_Null, (uint32 *)&KbParam.baseB},
        {(uint8 *)"2.k_min        ", Menu_Null, (uint32 *)&KbParam.proportion_min},
        {(uint8 *)"3.k_max        ", Menu_Null, (uint32 *)&KbParam.proportion_max},
        {(uint8 *)"5.p_max        ", Menu_Null, (uint32 *)&servoPID.kp_max},
        {(uint8 *)"6.p_min        ", Menu_Null, (uint32 *)&servoPID.kp_min},
        {(uint8 *)"7.d_max        ", Menu_Null, (uint32 *)&servoPID.kd_max},
        {(uint8 *)"8.d_min        ", Menu_Null, (uint32 *)&servoPID.kd_min},
        {(uint8 *)"9.differential", Menu_Null, (uint32 *)&servoPID.differential},
        {(uint8 *)"10.taw         ", Menu_Null, (uint32 *)&servoPID.taw},
        {(uint8 *)"11.SERVO_MID   ", Menu_SERVO_PWM_MID, (uint32 *)&SERVO_PWM_MID},
        {(uint8 *)"12.SERVO_MAX   ", Menu_SERVO_PWM_MAX, (uint32 *)&SERVO_PWM_MAX},
        {(uint8 *)"13.SERVO_MIN   ", Menu_SERVO_PWM_MIN, (uint32 *)&SERVO_PWM_MIN},
};

void Menu_Servo(void)
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(Servo_MenuTable) / sizeof(Servo_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=  ServoParam  =- ", &Servo_Prmt, Servo_MenuTable, menuNum);
}

void Menu_SERVO_PWM_MID(void)
{
    Site_t site;
    site.x = 120;
    site.y = (1 + Servo_Prmt.Cursor) * 16;

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);

    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            SERVO_PWM_MID += 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID);
            break;

        case KEY_D:
            SERVO_PWM_MID -= 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID);
            break;

        case KEY_L:
            SERVO_PWM_MID -= 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID);
            break;

        case KEY_R:
            SERVO_PWM_MID += 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID);
            break;

        default:
            break;
        }
    } while (keymsg.key != KEY_B);

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MID), 4);
    pwm_duty(S3010_PWM_CH, SERVO_PWM_MID);
}

void Menu_SERVO_PWM_MIN(void)
{
    Site_t site;
    site.x = 120;
    site.y = (1 + Servo_Prmt.Cursor) * 16;

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);

    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            SERVO_PWM_MIN += 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID - SERVO_PWM_MIN);
            break;

        case KEY_D:
            SERVO_PWM_MIN -= 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID - SERVO_PWM_MIN);
            break;

        case KEY_L:
            SERVO_PWM_MIN -= 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID - SERVO_PWM_MIN);
            break;

        case KEY_R:
            SERVO_PWM_MIN += 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID - SERVO_PWM_MIN);
            break;

        default:
            break;
        }
    } while (keymsg.key != KEY_B);

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MIN), 4);
    pwm_duty(S3010_PWM_CH, SERVO_PWM_MID - SERVO_PWM_MIN);
}

void Menu_SERVO_PWM_MAX(void)
{
    Site_t site;
    site.x = 120;
    site.y = (1 + Servo_Prmt.Cursor) * 16;

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);

    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            SERVO_PWM_MAX += 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID + SERVO_PWM_MAX);
            break;

        case KEY_D:
            SERVO_PWM_MAX -= 1;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID + SERVO_PWM_MAX);
            break;

        case KEY_L:
            SERVO_PWM_MAX -= 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID + SERVO_PWM_MAX);
            break;

        case KEY_R:
            SERVO_PWM_MAX += 10;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);
            pwm_duty(S3010_PWM_CH, SERVO_PWM_MID + SERVO_PWM_MAX);
            break;

        default:
            break;
        }
    } while (keymsg.key != KEY_B);

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(SERVO_PWM_MAX), 4);
    pwm_duty(S3010_PWM_CH, SERVO_PWM_MID + SERVO_PWM_MAX);
}

// �����˵�  �������
MENU_PRMT Element_Prmt;

MENU_TABLE Element_MenuTable[] =
    {
        {(uint8 *)"1.Garage       ", Menu_Garage, NULL},      //  ����
        {(uint8 *)"2.AnnulusSize  ", Menu_AnnulusSize, NULL}, //  Բ��
        {(uint8 *)"3.AnnulusParam ", Menu_Annulus, NULL},     //  Բ��
        {(uint8 *)"4.Ramp         ", Menu_Ramp, NULL},        //  �µ�
        {(uint8 *)"5.RampSize     ", Menu_RampSize, NULL},    //  �µ�
        {(uint8 *)"6.fork         ", Menu_Fork, NULL},        //  ����
        {(uint8 *)"8.Ucross       ", Menu_UCross, NULL},      //  ����

};

void Menu_Element(void)
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(Element_MenuTable) / sizeof(Element_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=   Element   =- ", &Element_Prmt, Element_MenuTable, menuNum);
}

MENU_PRMT Garage_Prmt;

MENU_TABLE Garage_MenuTable[] =
    {
        {(uint8 *)"dir  1L 2R       ", Menu_Null, (uint32 *)&garageDirectionFlag},
        {(uint8 *)"2 - numOfTurns   ", Menu_Null, (uint32 *)&judgedCnt},
        {(uint8 *)"OUT garageK      ", Menu_Null, (uint32 *)&KbParam.proportion_outGarage},
        {(uint8 *)"IN garageK       ", Menu_Null, (uint32 *)&KbParam.proportion_enterGarage},
        {(uint8 *)"drawLine_distance", Menu_Null, (uint32 *)&drawLine_distance_enterGarage},
        {(uint8 *)"garageSpeed      ", Menu_Null, (uint32 *)&SP.garageSpeed},
};

// �����˵�  �������
void Menu_Garage()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Garage_MenuTable) / sizeof(Garage_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= Garage =-   ", &Garage_Prmt, Garage_MenuTable, menuNum);
}

// �����˵�  �ٶȲ���
MENU_PRMT SpeedPara_Prmt;

MENU_TABLE SpeedPara_MenuTable_Constant[] =
    {
        {(uint8 *)"0.constant     ", Menu_Null, (uint32 *)&SP.constantSpeed},
        {(uint8 *)"1.annulus      ", Menu_Null, (uint32 *)&SP.annulusMinSpeed},
        {(uint8 *)"2.forkSpeed    ", Menu_Null, (uint32 *)&SP.forkSpeed},
        {(uint8 *)"4.outgarageSp  ", Menu_Null, (uint32 *)&SP.outGarageSpeed},
        {(uint8 *)"5.inGarageSp   ", Menu_Null, (uint32 *)&SP.garageSpeed},
        {(uint8 *)"6.ucrossSpeed  ", Menu_Null, (uint32 *)&SP.outUCrossSpeed},
        {(uint8 *)"7.differential ", Menu_Null, (uint32 *)&SI.differential},
};

MENU_TABLE SpeedPara_MenuTable_NormalShift[] =
    {
        {(uint8 *)"0.minSpeed     ", Menu_Null, (uint32 *)&SP.minSpeed},
        {(uint8 *)"1.curveSpeed   ", Menu_Null, (uint32 *)&SP.curveSpeed},
        {(uint8 *)"1.normalSpeed  ", Menu_Null, (uint32 *)&SP.normalSpeed},
        {(uint8 *)"2.annulusMin   ", Menu_Null, (uint32 *)&SP.annulusMinSpeed},
        {(uint8 *)"3.annulusMax   ", Menu_Null, (uint32 *)&SP.annulusSpeed},
        {(uint8 *)"4.outgarageSp  ", Menu_Null, (uint32 *)&SP.outGarageSpeed},
        {(uint8 *)"5.inGarageSp   ", Menu_Null, (uint32 *)&SP.garageSpeed},
        {(uint8 *)"6.ucrossSpeed  ", Menu_Null, (uint32 *)&SP.outUCrossSpeed},
        {(uint8 *)"7.forkSpeed    ", Menu_Null, (uint32 *)&SP.forkSpeed},
        {(uint8 *)"8.switchSpeedTo", Menu_Null, (uint32 *)&SP.switchSpeedTop},
        {(uint8 *)"8.speedK       ", Menu_Null, (uint32 *)&SP.speedK},
        {(uint8 *)"9.speedK2      ", Menu_Null, (uint32 *)&SP.speedK2},
        {(uint8 *)"10.annulusSK2  ", Menu_Null, (uint32 *)&SP.annulusSpeedK2},
        {(uint8 *)"11.differential", Menu_Null, (uint32 *)&SI.differential},
};

MENU_TABLE SpeedPara_MenuTable_Variable[] =
    {
        {(uint8 *)"0.maxSpeed      ", Menu_Null, (uint32 *)&SP.maxSpeed},
        {(uint8 *)"1.normalSpeed   ", Menu_Null, (uint32 *)&SP.normalSpeed},
        {(uint8 *)"2.curveSpeed    ", Menu_Null, (uint32 *)&SP.curveSpeed},
        {(uint8 *)"3.minSpeed      ", Menu_Null, (uint32 *)&SP.minSpeed},
        {(uint8 *)"4.annulusSpeed  ", Menu_Null, (uint32 *)&SP.annulusSpeed},
        {(uint8 *)"5.annulusMinSP  ", Menu_Null, (uint32 *)&SP.annulusMinSpeed},
        {(uint8 *)"6.garageSpeed   ", Menu_Null, (uint32 *)&SP.garageSpeed},
        {(uint8 *)"7.ucrossSpeed   ", Menu_Null, (uint32 *)&SP.outUCrossSpeed},
        {(uint8 *)"8.forkSpeed    ", Menu_Null, (uint32 *)&SP.forkSpeed},

        {(uint8 *)"9.speedK        ", Menu_Null, (uint32 *)&SP.speedK},
        {(uint8 *)"10.speedK2       ", Menu_Null, (uint32 *)&SP.speedK2},
        {(uint8 *)"11.annulusSK2    ", Menu_Null, (uint32 *)&SP.annulusSpeedK2},

        {(uint8 *)"12.brakeEdge_nor", Menu_Null, (uint32 *)&SP.brakeEdge_normal},
        {(uint8 *)"13.brakeEdge_cur", Menu_Null, (uint32 *)&SP.brakeEdge_curve},
        {(uint8 *)"14.brakeEdge_min", Menu_Null, (uint32 *)&SP.brakeEdge_min},
        {(uint8 *)"15.strCut       ", Menu_Null, (uint32 *)&SP.strightSpeedCut},

        {(uint8 *)"16.differential ", Menu_Null, (uint32 *)&SI.differential},
};

void Menu_SpeedPara()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    switch (runSpeedMode)
    {
    case CONSTANT_SPEED:
        menuNum = sizeof(SpeedPara_MenuTable_Constant) / sizeof(SpeedPara_MenuTable_Constant[0]); // �˵�����
        Menu_Process((uint8 *)"  -= Constant =-   ", &SpeedPara_Prmt, SpeedPara_MenuTable_Constant, menuNum);
        break;
    case NORMAL_SHIFT_SPEED:
        menuNum = sizeof(SpeedPara_MenuTable_NormalShift) / sizeof(SpeedPara_MenuTable_NormalShift[0]); // �˵�����
        Menu_Process((uint8 *)"-= Normal  shift =-", &SpeedPara_Prmt, SpeedPara_MenuTable_NormalShift, menuNum);
        break;
    case VARIABLE_SPEED:
        menuNum = sizeof(SpeedPara_MenuTable_Variable) / sizeof(SpeedPara_MenuTable_Variable[0]); // �˵�����
        Menu_Process((uint8 *)"  -= Variable =-   ", &SpeedPara_Prmt, SpeedPara_MenuTable_Variable, menuNum);
        break;
    default:
        menuNum = sizeof(SpeedPara_MenuTable_Variable) / sizeof(SpeedPara_MenuTable_Variable[0]); // �˵�����
        Menu_Process((uint8 *)"  -= error =-  ", &SpeedPara_Prmt, SpeedPara_MenuTable_Variable, menuNum);
        break;
    }
}

// �����˵�  ɲ��/���� �ж� ����
MENU_PRMT TrackDetection_Prmt;

MENU_TABLE TrackDetection_MenuTable[] =
    {
        {(uint8 *)"1.brakeTest    ", Menu_Null, (uint32 *)&TDP.brakeTest},
        {(uint8 *)"2.strightAD    ", Menu_Null, (uint32 *)&TDP.strightAD},
        {(uint8 *)"3.enterCurveAD ", Menu_Null, (uint32 *)&TDP.enterCurveAD},
        {(uint8 *)"4.brakeTop_1   ", Menu_Null, (uint32 *)&TDP.brakeTop_1},
        {(uint8 *)"5.brakeST_1    ", Menu_Null, (uint32 *)&TDP.brakeSpeedTop_1},
        {(uint8 *)"6.brakeTop_2   ", Menu_Null, (uint32 *)&TDP.brakeTop_2},
        {(uint8 *)"7.brakeST_2    ", Menu_Null, (uint32 *)&TDP.brakeSpeedTop_2},
        {(uint8 *)"8.brakeTop_3   ", Menu_Null, (uint32 *)&TDP.brakeTop_3},
        {(uint8 *)"9.brakeST_3    ", Menu_Null, (uint32 *)&TDP.brakeSpeedTop_3},
        {(uint8 *)"10.brakeTop_4  ", Menu_Null, (uint32 *)&TDP.brakeTop_4},
        {(uint8 *)"11.brakeST_4   ", Menu_Null, (uint32 *)&TDP.brakeSpeedTop_4},
        {(uint8 *)"12.brakeTop_0  ", Menu_Null, (uint32 *)&TDP.brakeTop_0},
        {(uint8 *)"13.brakeST_0   ", Menu_Null, (uint32 *)&TDP.brakeSpeedTop_0},
        {(uint8 *)"14.brakeTime   ", Menu_Null, (uint32 *)&TDP.brakeTime},
        //{"25.limitTime   ", Menu_Null, &TDP.limitTime},
};

void Menu_TrackDetection(void)
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(TrackDetection_MenuTable) / sizeof(TrackDetection_MenuTable[0]);

    Menu_Process((uint8 *)"  -=  SpJudge  =- ", &TrackDetection_Prmt, TrackDetection_MenuTable, menuNum);
}

// �����˵�  Բ������ѡ��
MENU_PRMT AnnulusSize_Prmt;

MENU_TABLE AnnulusSize_MenuTable[] =
    {
        {(uint8 *)"annulusNum    ", Menu_Null, (uint32 *)&annulusNum},
        {(uint8 *)"first         ", chooseAnnulusSize, (uint32 *)&annulusSize[0]},
        {(uint8 *)"second        ", chooseAnnulusSize, (uint32 *)&annulusSize[1]},
        {(uint8 *)"third         ", chooseAnnulusSize, (uint32 *)&annulusSize[2]},
        {(uint8 *)"fourth        ", chooseAnnulusSize, (uint32 *)&annulusSize[3]},
        {(uint8 *)"fifth         ", chooseAnnulusSize, (uint32 *)&annulusSize[4]},
};

void Menu_AnnulusSize()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(AnnulusSize_MenuTable) / sizeof(AnnulusSize_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"  -= AnnulusSize=-  ", &AnnulusSize_Prmt, AnnulusSize_MenuTable, menuNum);
}

void chooseAnnulusSize()
{
    Site_t site;
    site.x = 120;
    site.y = (1 + AnnulusSize_Prmt.Cursor) * 16;
    uint8 index = AnnulusSize_Prmt.Cursor - 1;

    tft180_set_color(WHITE, RED);
    tft180_show_uint(site.x, site.y, annulusSize[index], 4);

    do
    {
        KeySan();
        switch (keymsg.key)
        {
        case KEY_U:
            if (annulusSize[index] < 100)
                annulusSize[index] += 10;
            //                tft180_set_color(WHITE, RED);
            //                tft180_show_uint(site.x, site.y, annulusSize[index], 4);
            break;
        case KEY_D:
            if (annulusSize[index] > 50)
                annulusSize[index] -= 10;
            //                tft180_set_color(WHITE, RED);
            //                tft180_show_uint(site.x, site.y, annulusSize[index], 4);
            break;
        case KEY_L:
            if (annulusSize[index] > 50)
                annulusSize[index] -= 10;
            //                tft180_set_color(WHITE, RED);
            //                tft180_show_uint(site.x, site.y, annulusSize[index], 4);
            break;

        case KEY_R:
            if (annulusSize[index] < 100)
                annulusSize[index] += 10;
            //                tft180_set_color(WHITE, RED);
            //                tft180_show_uint(site.x, site.y, annulusSize[index], 4);
            break;
        default:
            break;
        }
        tft180_set_color(WHITE, RED);
        tft180_show_uint(site.x, site.y, annulusSize[index], 4);
    } while (keymsg.key != KEY_B);

    tft180_set_color(WHITE, RED);
    tft180_show_uint(site.x, site.y, annulusSize[index], 4);
}

void annulusParamInitFromEeprom()
{
    for (uint8 i = 0; i < annulusNum; i++)
    {
        switch (annulusSize[i])
        {
        case 50:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_50;
            KbParam.proportion_isAnnulus[i] = proportion_in_50;
            KbParam.proportion_outAnnulus[i] = proportion_out_50;
            break;
        case 60:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_60;
            KbParam.proportion_isAnnulus[i] = proportion_in_60;
            KbParam.proportion_outAnnulus[i] = proportion_out_60;
            break;
        case 70:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_70;
            KbParam.proportion_isAnnulus[i] = proportion_in_70;
            KbParam.proportion_outAnnulus[i] = proportion_out_70;
            break;
        case 80:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_80;
            KbParam.proportion_isAnnulus[i] = proportion_in_80;
            KbParam.proportion_outAnnulus[i] = proportion_out_80;
            break;
        case 90:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_90;
            KbParam.proportion_isAnnulus[i] = proportion_in_90;
            KbParam.proportion_outAnnulus[i] = proportion_out_90;
            break;
        case 100:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_100;
            KbParam.proportion_isAnnulus[i] = proportion_in_100;
            KbParam.proportion_outAnnulus[i] = proportion_out_100;
            break;
        default:
            KbParam.proportion_enterAnnulus[i] = proportion_enter_70;
            KbParam.proportion_isAnnulus[i] = proportion_in_70;
            KbParam.proportion_outAnnulus[i] = proportion_out_70;
            break;
        }
    }
}

// �����˵�  Բ������
MENU_PRMT Annulus_Prmt;

MENU_TABLE Annulus_MenuTable[] =
    {
        {(uint8 *)"0.drawLineDistance", Menu_Null, (uint32 *)&drawLine_distance_annulus},
        {(uint8 *)"1.param 50        ", Menu_Ann50, NULL},
        {(uint8 *)"1.param 60        ", Menu_Ann60, NULL},
        {(uint8 *)"1.param 70        ", Menu_Ann70, NULL},
        {(uint8 *)"1.param 80        ", Menu_Ann80, NULL},
        {(uint8 *)"1.param 90        ", Menu_Ann90, NULL},
        {(uint8 *)"1.param 100       ", Menu_Ann100, NULL},
};

void Menu_Annulus()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Annulus_MenuTable) / sizeof(Annulus_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= Annulus =-   ", &Annulus_Prmt, Annulus_MenuTable, menuNum);
}

MENU_PRMT Ann50_Prmt;

MENU_TABLE Ann50_MenuTable[] =
    {
        {(uint8 *)"1.enterYH50    ", Menu_Null, (uint32 *)&proportion_enter_50},
        {(uint8 *)"1.isYH50       ", Menu_Null, (uint32 *)&proportion_in_50},
        {(uint8 *)"1.outYH50      ", Menu_Null, (uint32 *)&proportion_out_50},
};

void Menu_Ann50()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann50_MenuTable) / sizeof(Ann50_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 50 =-   ", &Ann50_Prmt, Ann50_MenuTable, menuNum);
}

MENU_PRMT Ann60_Prmt;

MENU_TABLE Ann60_MenuTable[] =
    {
        {(uint8 *)"1.enterYH60    ", Menu_Null, (uint32 *)&proportion_enter_60},
        {(uint8 *)"1.isYH60       ", Menu_Null, (uint32 *)&proportion_in_60},
        {(uint8 *)"1.outYH60      ", Menu_Null, (uint32 *)&proportion_out_60},
};

void Menu_Ann60()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann60_MenuTable) / sizeof(Ann60_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 60 =-   ", &Ann60_Prmt, Ann60_MenuTable, menuNum);
}

MENU_PRMT Ann70_Prmt;

MENU_TABLE Ann70_MenuTable[] =
    {
        {(uint8 *)"1.enterYH70    ", Menu_Null, (uint32 *)&proportion_enter_70},
        {(uint8 *)"1.isYH70       ", Menu_Null, (uint32 *)&proportion_in_70},
        {(uint8 *)"1.outYH70      ", Menu_Null, (uint32 *)&proportion_out_70},
};

void Menu_Ann70()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann70_MenuTable) / sizeof(Ann70_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 70 =-   ", &Ann70_Prmt, Ann70_MenuTable, menuNum);
}

MENU_PRMT Ann80_Prmt;

MENU_TABLE Ann80_MenuTable[] =
    {
        {(uint8 *)"1.enterYH80    ", Menu_Null, (uint32 *)&proportion_enter_80},
        {(uint8 *)"1.isYH80       ", Menu_Null, (uint32 *)&proportion_in_80},
        {(uint8 *)"1.outYH80      ", Menu_Null, (uint32 *)&proportion_out_80},
};

void Menu_Ann80()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann80_MenuTable) / sizeof(Ann80_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 80 =-   ", &Ann80_Prmt, Ann80_MenuTable, menuNum);
}

MENU_PRMT Ann90_Prmt;

MENU_TABLE Ann90_MenuTable[] =
    {
        {(uint8 *)"1.enterYH90    ", Menu_Null, (uint32 *)&proportion_enter_90},
        {(uint8 *)"1.isYH90       ", Menu_Null, (uint32 *)&proportion_in_90},
        {(uint8 *)"1.outYH90      ", Menu_Null, (uint32 *)&proportion_out_90},
};

void Menu_Ann90()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann90_MenuTable) / sizeof(Ann90_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 90 =-   ", &Ann80_Prmt, Ann80_MenuTable, menuNum);
}

MENU_PRMT Ann100_Prmt;

MENU_TABLE Ann100_MenuTable[] =
    {
        {(uint8 *)"1.enterYH100    ", Menu_Null, (uint32 *)&proportion_enter_100},
        {(uint8 *)"1.isYH100       ", Menu_Null, (uint32 *)&proportion_in_100},
        {(uint8 *)"1.outYH100      ", Menu_Null, (uint32 *)&proportion_out_100},
};

void Menu_Ann100()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ann100_MenuTable) / sizeof(Ann100_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= 100 =-   ", &Ann100_Prmt, Ann100_MenuTable, menuNum);
}

// �����˵�  �������
MENU_PRMT Fork_Prmt;

MENU_TABLE Fork_MenuTable[] =
    {
        {(uint8 *)"first dir      ", Menu_Null, (uint32 *)&dir_fork},
        {(uint8 *)"drawLineDistance", Menu_Null, (uint32 *)&drawLine_distance_fork},
        {(uint8 *)"forkK          ", Menu_Null, (uint32 *)&KbParam.proportion_fork},
        {(uint8 *)"forkSpeed      ", Menu_Null, (uint32 *)&SP.forkSpeed},
        {(uint8 *)"zoom           ", Menu_Null, (uint32 *)&zoom},
};

void Menu_Fork()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Fork_MenuTable) / sizeof(Fork_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"    -= Fork =-    ", &Fork_Prmt, Fork_MenuTable, menuNum);
}

// �����˵�  �µ�����ѡ��
MENU_PRMT RampSize_Prmt;

MENU_TABLE RampSize_MenuTable[] =
    {
        {(uint8 *)"rampNum       ", Menu_Null, (uint32 *)&rampNum},
        {(uint8 *)"first         ", chooseRampSize, (uint32 *)&rampSize[0]},
        {(uint8 *)"second        ", chooseRampSize, (uint32 *)&rampSize[1]},
        {(uint8 *)"third         ", chooseRampSize, (uint32 *)&rampSize[2]},
        {(uint8 *)"10:sharp  cur ", Menu_Null, NULL},
        {(uint8 *)"20:sharp  str ", Menu_Null, NULL},
        {(uint8 *)"30:gentle cur ", Menu_Null, NULL},
        {(uint8 *)"40:gentle str ", Menu_Null, NULL},
};

void Menu_RampSize()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(RampSize_MenuTable) / sizeof(RampSize_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)" -= RampSize=- ", &RampSize_Prmt, RampSize_MenuTable, menuNum);
}

void chooseRampSize()
{
    Site_t site;
    site.x = 120;
    site.y = (1 + RampSize_Prmt.Cursor) * 16;
    uint8 index = RampSize_Prmt.Cursor - 1;

    tft180_set_color(WHITE, RED);
    tft180_show_uint(site.x, site.y, rampSize[index], 4);
    do
    {
        KeySan();
        switch (keymsg.key)
        {
        case KEY_U:
            if (rampSize[index] < 40)
                rampSize[index] += 10;
            break;
        case KEY_D:
            if (rampSize[index] > 10)
                rampSize[index] -= 10;
            break;
        case KEY_L:
            if (rampSize[index] > 10)
                rampSize[index] -= 10;
            break;

        case KEY_R:
            if (rampSize[index] < 40)
                rampSize[index] += 10;
            break;
        default:
            break;
        }
        tft180_set_color(WHITE, RED);
        tft180_show_uint(site.x, site.y, rampSize[index], 4);
    } while (keymsg.key != KEY_B);

    tft180_set_color(WHITE, RED);
    tft180_show_uint(site.x, site.y, rampSize[index], 4);
}

void rampParamInitFromEeprom()
{
    for (uint8 i = 0; i < rampNum; i++)
    {
        switch (rampSize[i])
        {
        case 40:
            SP.rampUpSpeed[i] = rampUp_gentle;
            SP.rampOnSpeed[i] = rampOn_gentle;
            SP.rampDownSpeed[i] = rampOn_gentle;
            break;
        case 30:
            SP.rampUpSpeed[i] = rampUp_big;
            SP.rampOnSpeed[i] = rampOn_big;
            SP.rampDownSpeed[i] = rampDown_big;
            break;
        case 20:
            SP.rampUpSpeed[i] = rampUp_mid;
            SP.rampOnSpeed[i] = rampOn_mid;
            SP.rampDownSpeed[i] = rampDown_mid;
            break;
        case 10:
            SP.rampUpSpeed[i] = rampUp_small;
            SP.rampOnSpeed[i] = rampOn_small;
            SP.rampDownSpeed[i] = rampDown_small;
            break;
        default:
            SP.rampUpSpeed[i] = rampUp_small;
            SP.rampOnSpeed[i] = rampUp_small;
            SP.rampDownSpeed[i] = rampUp_small;
            break;
        }
    }
}

MENU_PRMT UCross_Prmt;

MENU_TABLE UCross_MenuTable[] =
    {

        {(uint8 *)"0.manual set? ", Menu_Null, (uint32 *)&ucross_manual_setting},
        {(uint8 *)"1.manual dir1 ", Menu_Null, (uint32 *)&dir_ucross_manual_setting_1},
        {(uint8 *)"2.manual dir2 ", Menu_Null, (uint32 *)&dir_ucross_manual_setting_2},
        {(uint8 *)"3.brake Dist  ", Menu_Null, (uint32 *)&brake_distance_ucross},
        {(uint8 *)"4.drawlineDist", Menu_Null, (uint32 *)&drawLine_distance_ucross},
        {(uint8 *)"5.ucross_dd   ", Menu_Null, (uint32 *)&KbParam.proportion_dd_ucross},
        {(uint8 *)"6.UcrossSpeed ", Menu_Null, (uint32 *)&SP.outUCrossSpeed},
        {(uint8 *)"7.rigor_both  ", Menu_Null, (uint32 *)&rigor_ucross_both},
        {(uint8 *)"8.rigorOneSide", Menu_Null, (uint32 *)&rigor_ucross_oneSide},
};

void Menu_UCross()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(UCross_MenuTable) / sizeof(UCross_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"    -= UCross =-", &UCross_Prmt, UCross_MenuTable, menuNum);
}

// �����˵�  �µ�����
MENU_PRMT Ramp_Prmt;

MENU_TABLE Ramp_MenuTable[] =
    {

        {(uint8 *)"0.enterRampK   ", Menu_Null, (uint32 *)&KbParam.proportion_ramp},
        {(uint8 *)"1.enterRampP   ", Menu_Null, (uint32 *)&servoPID.kp_ramp},
        {(uint8 *)"2.onRampK      ", Menu_Null, (uint32 *)&KbParam.proportion_onRamp},
        {(uint8 *)"3.onRampP      ", Menu_Null, (uint32 *)&servoPID.kp_onRamp},
        {(uint8 *)"4.dd           ", Menu_Null, (uint32 *)&KbParam.proportion_dd_Ramp},
        {(uint8 *)"4.thre_TFmini  ", Menu_Null, (uint32 *)&threshold_TFmini},
        {(uint8 *)"5.param 40     ", Menu_GentleRamp, NULL},
        {(uint8 *)"5.param 30     ", Menu_BigRamp, NULL},
        {(uint8 *)"6.param 20     ", Menu_MidRamp, NULL},
        {(uint8 *)"7.param 10     ", Menu_SmallRamp, NULL},
};

void Menu_Ramp()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Ramp_MenuTable) / sizeof(Ramp_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"    -= Ramp =-", &Ramp_Prmt, Ramp_MenuTable, menuNum);
}

MENU_PRMT GentleRamp_Prmt;

MENU_TABLE GentleRamp_MenuTable[] =
    {
        {(uint8 *)"1.RampUp      ", Menu_Null, (uint32 *)&rampUp_gentle},
        {(uint8 *)"1.RampOn      ", Menu_Null, (uint32 *)&rampOn_gentle},
        {(uint8 *)"1.RampDown    ", Menu_Null, (uint32 *)&rampDown_gentle},
};

void Menu_GentleRamp()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(GentleRamp_MenuTable) / sizeof(GentleRamp_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"  -= Gen Str =-  ", &GentleRamp_Prmt, GentleRamp_MenuTable, menuNum);
}

MENU_PRMT BigRamp_Prmt;

MENU_TABLE BigRamp_MenuTable[] =
    {
        {(uint8 *)"1.RampUp      ", Menu_Null, (uint32 *)&rampUp_big},
        {(uint8 *)"1.RampOn      ", Menu_Null, (uint32 *)&rampOn_big},
        {(uint8 *)"1.RampDown    ", Menu_Null, (uint32 *)&rampDown_big},
};

void Menu_BigRamp()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(BigRamp_MenuTable) / sizeof(BigRamp_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"  -= Gen Cur =-  ", &BigRamp_Prmt, BigRamp_MenuTable, menuNum);
}

MENU_PRMT MidRamp_Prmt;

MENU_TABLE MidRamp_MenuTable[] =
    {
        {(uint8 *)"1.RampUp      ", Menu_Null, (uint32 *)&rampUp_mid},
        {(uint8 *)"1.RampOn      ", Menu_Null, (uint32 *)&rampOn_mid},
        {(uint8 *)"1.RampDown    ", Menu_Null, (uint32 *)&rampDown_mid},
};

void Menu_MidRamp()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(MidRamp_MenuTable) / sizeof(MidRamp_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"  -= Sha Str =-  ", &MidRamp_Prmt, MidRamp_MenuTable, menuNum);
}

MENU_PRMT SmallRamp_Prmt;

MENU_TABLE SmallRamp_MenuTable[] =
    {
        {(uint8 *)"1.RampUp      ", Menu_Null, (uint32 *)&rampUp_small},
        {(uint8 *)"1.RampOn      ", Menu_Null, (uint32 *)&rampOn_small},
        {(uint8 *)"1.RampDown    ", Menu_Null, (uint32 *)&rampDown_small},
};

void Menu_SmallRamp()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(SmallRamp_MenuTable) / sizeof(SmallRamp_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"  -= Sha Cur =-  ", &SmallRamp_Prmt, SmallRamp_MenuTable, menuNum);
}

// �����˵�7  �������

MENU_PRMT Motor_Prmt;

MENU_TABLE Motor_MenuTable[] =
    {
        {(uint8 *)"0.visualScope  ", Menu_visualScope, NULL},
        {(uint8 *)"1.CtrlAlgorithm", Menu_motorControlAlgorithm, NULL},
        {(uint8 *)"2.param        ", Menu_motorParam, NULL},
        {(uint8 *)"2.driver_test  ", motorDriver_test, NULL},

};

void Menu_Motor()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(Motor_MenuTable) / sizeof(Motor_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= Motor =-   ", &Motor_Prmt, Motor_MenuTable, menuNum);
}

MENU_PRMT motorControlAlgorithm_Prmt;

MENU_TABLE motorControlAlgorithm_MenuTable[] =
    {
        {(uint8 *)"choose         ", Menu_Null, (uint32 *)&motorControlAlgorithm},
        {(uint8 *)"Positional  : 0", Menu_Null, NULL},
        {(uint8 *)"Incremental : 1", Menu_Null, NULL},
        {(uint8 *)"Vari Struct : 2", Menu_Null, NULL},
        {(uint8 *)"Zjut        : 3", Menu_Null, NULL},
        {(uint8 *)"ADRC        : 4", Menu_Null, NULL},
};

void Menu_motorControlAlgorithm()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(motorControlAlgorithm_MenuTable) / sizeof(motorControlAlgorithm_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -= Algorithm =- ", &motorControlAlgorithm_Prmt, motorControlAlgorithm_MenuTable, menuNum);
}

MENU_PRMT visualScope_Prmt;

MENU_TABLE visualScope_MenuTable[] =
    {
        {(uint8 *)"0.scopeEnable  ", Menu_Null, (uint32 *)&visualScope_enable},
        {(uint8 *)"1.aimSpeedL+300", Menu_CHANGE_AIMSPEED_LEFT, (uint32 *)&aimSpeedL_visualScope},
        {(uint8 *)"2.aimSpeedR+300", Menu_CHANGE_AIMSPEED_RIGHT, (uint32 *)&aimSpeedR_visualScope},
        {(uint8 *)"2.addspeed     ", Menu_Null, (uint32 *)&addspeed},
};

void Menu_visualScope()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;

    menuNum = sizeof(visualScope_MenuTable) / sizeof(visualScope_MenuTable[0]); // �˵�����

    Menu_Process((uint8 *)"   -= Motor =-   ", &visualScope_Prmt, visualScope_MenuTable, menuNum);
}

void Menu_CHANGE_AIMSPEED_LEFT(void)
{
    Site_t site;
    site.x = 120;
    site.y = (1 + visualScope_Prmt.Cursor) * 16;
    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);

    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            aimSpeedL_visualScope += addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);
            break;

        case KEY_D:
            aimSpeedL_visualScope -= addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);
            break;

        case KEY_L:
            aimSpeedL_visualScope -= addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);
            break;

        case KEY_R:
            aimSpeedL_visualScope += addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);
            break;

        default:
            break;
        }
        tft180_show_string(0, 7 * 16, "varL:");
        tft180_show_int(50, 7 * 16, SI.varL[0], 4);
    } while (keymsg.key != KEY_B);
    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(aimSpeedL_visualScope), 4);
}

void Menu_CHANGE_AIMSPEED_RIGHT(void)
{
    Site_t site;
    site.x = 120;
    site.y = (1 + visualScope_Prmt.Cursor) * 16;
    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);

    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            aimSpeedR_visualScope += addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);
            break;

        case KEY_D:
            aimSpeedR_visualScope -= addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);
            break;

        case KEY_L:
            aimSpeedR_visualScope -= addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);
            break;

        case KEY_R:
            aimSpeedR_visualScope += addspeed;
            tft180_set_color(RED, WHITE);
            tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);
            break;

        default:
            break;
            tft180_show_string(0, 7 * 16, "varR:");
            tft180_show_int(50, 7 * 16, SI.varR[0], 4);
        }
    } while (keymsg.key != KEY_B);

    tft180_set_color(RED, WHITE);
    tft180_show_int(site.x, site.y, (int)(aimSpeedR_visualScope), 4);
}

MENU_PRMT motorParam_PositionalPID_Prmt;

MENU_TABLE motorParam_PositionalPID_MenuTable[] =
    {
        {(uint8 *)"0.p            ", Menu_Null, (uint32 *)&positionalPIDParam.kp},
        {(uint8 *)"1.i            ", Menu_Null, (uint32 *)&positionalPIDParam.ki},
        {(uint8 *)"2.d            ", Menu_Null, (uint32 *)&positionalPIDParam.kd},
        {(uint8 *)"3.i_limit      ", Menu_Null, (uint32 *)&positionalPIDParam.i_limit},
};

MENU_PRMT motorParam_IncrementalPID_Prmt;

MENU_TABLE motorParam_IncrementalPID_MenuTable[] =
    {
        {(uint8 *)"0.p            ", Menu_Null, (uint32 *)&incrementalPIDParam.kp},
        {(uint8 *)"1.i            ", Menu_Null, (uint32 *)&incrementalPIDParam.ki},
        {(uint8 *)"2.d            ", Menu_Null, (uint32 *)&incrementalPIDParam.kd},
        //        {(uint8 *)"2.i_limit      ", Menu_Null, (uint32 *)&incrementalPIDParam.i_limit},
};

MENU_PRMT motorParam_VariableStructurePID_Prmt;

MENU_TABLE motorParam_VariableStructurePID_MenuTable[] =
    {
        {(uint8 *)"0.p_base       ", Menu_Null, (uint32 *)&variableStructurePIDParam.kp_base},
        {(uint8 *)"1.p_range      ", Menu_Null, (uint32 *)&variableStructurePIDParam.range_kp},
        {(uint8 *)"2.i_base       ", Menu_Null, (uint32 *)&variableStructurePIDParam.ki_base},
        {(uint8 *)"2.decay_kp     ", Menu_Null, (uint32 *)&variableStructurePIDParam.decayRate_kp},
        {(uint8 *)"2.decay_ki     ", Menu_Null, (uint32 *)&variableStructurePIDParam.decayRate_ki},
};

MENU_PRMT motorParam_ZjutPID_Prmt;

MENU_TABLE motorParam_ZjutPID_MenuTable[] =
    {
        {(uint8 *)"0.p            ", Menu_Null, (uint32 *)&zjutPIDParam.kp},
        {(uint8 *)"1.i            ", Menu_Null, (uint32 *)&zjutPIDParam.ki},
        {(uint8 *)"2.d            ", Menu_Null, (uint32 *)&zjutPIDParam.kd},
        {(uint8 *)"3.change_ki    ", Menu_Null, (uint32 *)&zjutPIDParam.change_ki},
        {(uint8 *)"4.change_kib   ", Menu_Null, (uint32 *)&zjutPIDParam.change_kib},
};

MENU_PRMT motorParam_ADRC_Prmt;

MENU_TABLE motorParam_ADRC_MenuTable[] =
    {
        {(uint8 *)"0.r            ", Menu_Null, (uint32 *)&adrcParam.r},
        {(uint8 *)"1.beta01       ", Menu_Null, (uint32 *)&adrcParam.beta01},
        {(uint8 *)"2.beta02       ", Menu_Null, (uint32 *)&adrcParam.beta02},
        {(uint8 *)"3.beta03       ", Menu_Null, (uint32 *)&adrcParam.beta03},
        {(uint8 *)"4.beta1        ", Menu_Null, (uint32 *)&adrcParam.beta1},
        {(uint8 *)"3.beta2        ", Menu_Null, (uint32 *)&adrcParam.beta2},
        {(uint8 *)"4.b0           ", Menu_Null, (uint32 *)&adrcParam.b0},
};

void Menu_motorParam()
{
    lcd_clear_all(WHITE);

    uint8 menuNum;
    switch (motorControlAlgorithm)
    {
    case PositionalPID:
        menuNum = sizeof(motorParam_PositionalPID_MenuTable) / sizeof(motorParam_PositionalPID_MenuTable[0]); // �˵�����
        Menu_Process((uint8 *)" PositionalPID ", &motorParam_PositionalPID_Prmt, motorParam_PositionalPID_MenuTable, menuNum);
        break;
    case IncrementalPID:
        menuNum = sizeof(motorParam_IncrementalPID_MenuTable) / sizeof(motorParam_IncrementalPID_MenuTable[0]); // �˵�����
        Menu_Process((uint8 *)" -= IncrementalPID =- ", &motorParam_IncrementalPID_Prmt, motorParam_IncrementalPID_MenuTable, menuNum);
        break;
    case VariableStructurePID:
        menuNum = sizeof(motorParam_VariableStructurePID_MenuTable) / sizeof(motorParam_VariableStructurePID_MenuTable[0]); // �˵�����
        Menu_Process((uint8 *)"VariableStructure", &motorParam_VariableStructurePID_Prmt, motorParam_VariableStructurePID_MenuTable, menuNum);
        break;
    case ZjutPID:
        menuNum = sizeof(motorParam_ZjutPID_MenuTable) / sizeof(motorParam_ZjutPID_MenuTable[0]); // �˵�����
        Menu_Process((uint8 *)"    -=  ZjutPID  =-     ", &motorParam_ZjutPID_Prmt, motorParam_ZjutPID_MenuTable, menuNum);
        break;
    case ADRC:
        menuNum = sizeof(motorParam_ADRC_MenuTable) / sizeof(motorParam_ADRC_MenuTable[0]); // �˵�����
        Menu_Process((uint8 *)"      -=  ADRC  =-       ", &motorParam_ADRC_Prmt, motorParam_ADRC_MenuTable, menuNum);
        break;
    };
}

// �����˵�  ͼ����ֵ
MENU_PRMT Sun_Prmt;

MENU_TABLE Sun_MenuTable[] =
    {
        {(uint8 *)"1.OSTU_Min      ", Menu_Null, (uint32 *)&OSTU_MIN}, //  ��������ֵ
        {(uint8 *)"2.OSTU_Max      ", Menu_Null, (uint32 *)&OSTU_MAX}, //  ��������ֵ
        {(uint8 *)"3.Sobel_Edge    ", Menu_Null, (uint32 *)&sobel_threshold},
        {(uint8 *)"3.highOstuGain  ", Menu_Null, (uint32 *)&highOstuGain},
        {(uint8 *)"3.highOstuRow   ", Menu_Null, (uint32 *)&highOstuRow}};

void Menu_Sun()
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(Sun_MenuTable) / sizeof(Sun_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -= SunParam =- ", &Sun_Prmt, Sun_MenuTable, menuNum);
}

// �����˵�  ����ͷ��������
MENU_PRMT Camera_Prmt;

MENU_TABLE Camera_MenuTable[] =
    {

        {(uint8 *)"1.AUTO_EXP     ", Menu_Null, (uint32 *)&cameraDebugVAR.AUTO_EXP_TEMP},
        {(uint8 *)"2.EXP_TIME     ", Menu_Null, (uint32 *)&cameraDebugVAR.EXP_TIME_TEMP},
        {(uint8 *)"3.FPS          ", Menu_Null, (uint32 *)&cameraDebugVAR.FPS_TEMP},
        {(uint8 *)"4.GAIN         ", Menu_Null, (uint32 *)&cameraDebugVAR.GAIN_TEMP},
        {(uint8 *)"5.OSTU_Min      ", Menu_Null, (uint32 *)&OSTU_MIN}, //  ��������ֵ
};

void Menu_Camera()
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(Camera_MenuTable) / sizeof(Camera_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -= CameraParam =- ", &Camera_Prmt, Camera_MenuTable, menuNum);
}

MENU_PRMT sdCard_Prmt;

MENU_TABLE sdCard_MenuTable[] =
    {
        {(uint8 *)"1.Review   ", Menu_Sd, NULL},
        {(uint8 *)"2.allmpMode", Menu_allmapMode, NULL},  // ������
        {(uint8 *)"3.imgGrMode", Menu_imgGrayMode, NULL}, // ֻ��Ҷ�ͼ
        {(uint8 *)"4.clear    ", sd_clear, NULL},
};

void Menu_sdCardParam(void)
{
    lcd_clear_all(WHITE);
    Site_t site;

    site.x = 120;
    site.y = (2 + sdcardParam.recordImgMode) * 16;
    m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);

    uint8 menuNum;
    menuNum = sizeof(sdCard_MenuTable) / sizeof(sdCard_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=   SdMode   =- ", &sdCard_Prmt, sdCard_MenuTable, menuNum);
}

void Menu_Sd(void)
{
    sdreverse();
}

// �����ݺ�allmap
void Menu_allmapMode()
{
    if (sdcardParam.recordImgMode != allmapMode)
    {
        Site_t site;
        site.x = 120;
        site.y = (2 + sdcardParam.recordImgMode) * 16;
        lcd_clear(site.x, site.y, 159, site.y + 15, WHITE);
        sdcardParam.recordImgMode = allmapMode;
        *((uint8 *)&sdInfo) = allmapMode;
        site.x = 120;
        site.y = (2 + sdcardParam.recordImgMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}

// ֻ��Ҷ�ͼ
void Menu_imgGrayMode()
{
    if (sdcardParam.recordImgMode != imgGrayMode)
    {
        Site_t site;
        site.x = 120;
        site.y = (2 + sdcardParam.recordImgMode) * 16;
        lcd_clear(site.x, site.y, 159, site.y + 15, WHITE);
        sdcardParam.recordImgMode = imgGrayMode;
        *((uint8 *)&sdInfo) = imgGrayMode;
        site.x = 120;
        site.y = (2 + sdcardParam.recordImgMode) * 16;
        m_lcd_showstr(site.x, site.y, "  *  ", RED, WHITE);
    }
}

MENU_PRMT ReadFlash_Prmt;

MENU_TABLE ReadFlash_MenuTable[] =
    {
        {(uint8 *)"1.low    Constant   ", Menu_ReadFlash_1, NULL},
        {(uint8 *)"2.high   Constant   ", Menu_ReadFlash_2, NULL},
        {(uint8 *)"3.mid    Normalshift", Menu_ReadFlash_3, NULL},
        {(uint8 *)"4.mid    Variable   ", Menu_ReadFlash_4, NULL},
        {(uint8 *)"5.high   Variable   ", Menu_ReadFlash_5, NULL},
};

void Menu_ReadFlash(void)
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(ReadFlash_MenuTable) / sizeof(ReadFlash_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=  Read_Flash  =- ", &ReadFlash_Prmt, ReadFlash_MenuTable, menuNum);
}

void Menu_ReadFlash_1()
{
    lcd_clear_all(WHITE);
    readFlash(1);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "READ IS OK", RED, WHITE);
}

void Menu_ReadFlash_2()
{
    lcd_clear_all(WHITE);
    readFlash(2);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "READ IS OK", RED, WHITE);
}

void Menu_ReadFlash_3()
{
    lcd_clear_all(WHITE);
    readFlash(3);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "READ IS OK", RED, WHITE);
}

void Menu_ReadFlash_4()
{
    lcd_clear_all(WHITE);
    readFlash(4);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "READ IS OK", RED, WHITE);
}

void Menu_ReadFlash_5()
{
    lcd_clear_all(WHITE);
    readFlash(5);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "READ IS OK", RED, WHITE);
}

void readFlash(uint8 flashNum)
{
    uint16 EEPROM_DATA_NUM = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    flash_buffer_clear();                   // �������
    flash_read_page_to_buffer(0, flashNum); // �����ݴӻ�����������
    tft180_set_color(RED, WHITE);
    tft180_show_string(0, 0, "reading...");
    tft180_show_uint(0, 16, EEPROM_DATA_NUM, 4);

    for (uint16 i = 0; i < EEPROM_DATA_NUM; ++i)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA[i] = temp_vaule;
    }
}
MENU_PRMT WriteFlash_Prmt;

MENU_TABLE WriteFlash_MenuTable[] =
    {
        {(uint8 *)"1.low    Constant   ", Menu_WriteFlash_1, NULL},
        {(uint8 *)"2.high   Constant   ", Menu_WriteFlash_2, NULL},
        {(uint8 *)"3.mid    Normalshift", Menu_WriteFlash_3, NULL},
        {(uint8 *)"4.mid    Variable   ", Menu_WriteFlash_4, NULL},
        {(uint8 *)"5.high   Variable   ", Menu_WriteFlash_5, NULL},
};

void Menu_WriteFlash(void)
{
    lcd_clear_all(WHITE);
    uint8 menuNum;
    menuNum = sizeof(WriteFlash_MenuTable) / sizeof(WriteFlash_MenuTable[0]); // �˵�����
    Menu_Process((uint8 *)" -=  WriteFlash  =- ", &WriteFlash_Prmt, WriteFlash_MenuTable, menuNum);
}

void Menu_WriteFlash_1()
{
    lcd_clear_all(WHITE);
    writeFlash(1);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "WRITE IS OK", RED, WHITE);
}

void Menu_WriteFlash_2()
{
    lcd_clear_all(WHITE);
    writeFlash(2);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "WRITE IS OK", RED, WHITE);
}

void Menu_WriteFlash_3()
{
    lcd_clear_all(WHITE);
    writeFlash(3);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "WRITE IS OK", RED, WHITE);
}

void Menu_WriteFlash_4()
{
    lcd_clear_all(WHITE);
    writeFlash(4);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "WRITE IS OK", RED, WHITE);
}

void Menu_WriteFlash_5()
{
    lcd_clear_all(WHITE);
    writeFlash(5);
    system_delay_ms(500);
    m_lcd_showstr(0, 0, "WRITE IS OK", RED, WHITE);
}

void writeFlash(uint8 flashNum)
{
    // һ����12������
    // ÿ��������1024ҳ
    // һ����96KB
    // һҳֻ��8���ֽڣ�������32λ��4��16λ
    uint16 EEPROM_DATA_NUM = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    uint32 pageNum = 0;
    uint32 sectorNum = flashNum;
    // ��鵱ǰҳ�Ƿ������ݣ��������������Ҫ��������������
    //    eeprom_erase_sector(sectorNum);
    flash_erase_page(0, flashNum);
    for (uint16 i = 0; i < EEPROM_DATA_NUM; ++i)
    {
        flash_write_page(sectorNum, pageNum, EEPROM_DATA[i], sizeof(EEPROM_DATA[i]));
        pageNum = pageNum + 1;
        if (pageNum == 1024)
        {
            sectorNum = sectorNum + 1;
            pageNum = 0;
            flash_erase_page(0, flashNum);
        }
    }
    m_lcd_showstr(0, 0, "WRITE IS OK!", RED, WHITE);
}
/******************************************************************************
 * FunctionName   : MainMenu_Set()
 * Description    : ��������
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void MainMenu_Set(void)
{
    if (GET_SWITCH1())
    {
        Read_EEPROM();
    }
    lcd_clear_all(WHITE);
    ExitMenu_flag = 0;
    lcd_clear_all(WHITE);
    uint8 menuNum = sizeof(MainMenu_Table) / sizeof(MainMenu_Table[0]); // �˵�����
    Menu_Process((uint8 *)" -=   Setting   =- ", &MainMenu_Prmt, MainMenu_Table, menuNum);
    Write_EEPROM(); // ������д��EEPROM����
    annulusParamInitFromEeprom();
    rampParamInitFromEeprom();
    lcd_clear_all(WHITE);
}

/******************************************************************************
 * FunctionName   : Menu_Process()
 * Description    : ����˵���
 * EntryParameter : menuName - �˵����ƣ�prmt - �˵�������table - �˵�����, num - �˵�����
 * ReturnValue    : None
 ******************************************************************************/
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)
{
    KEY_e key;
    Site_t site;

    uint8 page; // ��ʾ�˵���Ҫ��ҳ��

    if (num - PAGE_DISP_NUM <= 0)
        page = 1;
    else
    {
        page = num - PAGE_DISP_NUM + 1;
        num = PAGE_DISP_NUM;
    }

    // ��ʾ������ҳ������
    Menu_PrmtInit(prmt, num, page);

    do
    {
        tft180_set_color(RED, WHITE);
        tft180_show_string((uint16)0, (uint16)0, (const char *)menuName); // ��ʾ�˵�����
        // ��ʾ�˵���
        Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
        key = KeySan(); // ��ȡ����

        if (Menu_Move(prmt, key) == 0) // �˵��ƶ� ����ȷ�ϼ�
        {
            // �жϴ˲˵���������Ҫ���ڵĲ��� ��������������
            if (table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
            {
                site.x = 120;
                site.y = (1 + prmt->Cursor) * 16;

                tft180_set_color(WHITE, RED);
                tft180_show_uint(site.x, site.y, (uint16) * (table[prmt->Index].DebugParam), 4);
                adjustParam(site, table[prmt->Index].DebugParam, 4, WHITE, RED);
            }
            // ���ǲ������ڵĻ���ִ�в˵�����
            else
            {
                table[prmt->Index].ItemHook(); // ִ����Ӧ��
            }
        }
    } while (prmt->ExitMark == 0 && ExitMenu_flag == 0);

    lcd_clear_all(WHITE);
}

/******************************************************************************
 * FunctionName   : Menu_PrmtInit()
 * Description    : ��ʼ���˵�����
 * EntryParameter : prmt - �˵�����, num - ÿҳ��ʾ����, page - �����ʾҳ��
 * ReturnValue    : None
 *******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
    prmt->ExitMark = 0; // ����˳��˵���־

    prmt->Cursor = 0;    // �������
    prmt->PageNo = 0;    // ҳ����
    prmt->Index = 0;     // ��������
    prmt->DispNum = num; // ҳ�����ʾ��Ŀ��

    prmt->MaxPage = page; // ���ҳ��
}

/******************************************************************************
 * FunctionName   : Menu_Display()
 * Description    : ��ʾ�˵���
 * EntryParameter : page - ��ʾҳ��dispNum - ÿһҳ����ʾ�cursor - ���λ��
 * ReturnValue    : None
 *******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor)
{
    uint8 i;
    Site_t site;

    for (i = 0; i < dispNum; i++)
    {
        if (cursor == i)
        {
            /* ������ʾ��ǰ���ѡ�в˵��� */
            site.x = 0;
            site.y = (i + 1) * 16;
            tft180_set_color(WHITE, BLUE);
            tft180_show_string(site.x, site.y, (const char *)menuTable[pageNo + i].MenuName);
            /* ���˲˵�������Ҫ���Ĳ���������ʾ�ò��� */
            if (menuTable[pageNo + i].DebugParam != NULL)
            {
                site.x = 120;

                uint16 num_t = (*(menuTable[pageNo + i].DebugParam));
                tft180_set_color(RED, WHITE);
                tft180_show_uint(site.x, site.y, num_t, 4);
            }
        }
        else
        {
            /* ������ʾ����˵��� */
            site.x = 0;
            site.y = (i + 1) * 16;
            tft180_set_color(BLUE, WHITE);
            tft180_show_string(site.x, site.y, (const char *)menuTable[pageNo + i].MenuName);
            //            m_lcd_showstr((uint16)site.x, (uint16)site.y, (const int8 *)menuTable[pageNo + i].MenuName, BLUE, WHITE);
            /* ���˲˵�������Ҫ���Ĳ���������ʾ�ò��� */
            if (menuTable[pageNo + i].DebugParam != NULL)
            {
                site.x = 120;

                uint16 num_t = (*(menuTable[pageNo + i].DebugParam));
                tft180_set_color(RED, WHITE);
                tft180_show_uint(site.x, site.y, num_t, 4);
                // lcd_showuint16_setColor(site.x, site.y, num_t, RED, WHITE);
            }
            /*else if (showDirFlag && pageNo + i == 0)
                        {
                            showDir();
                        }*/
        }
    }
}

/******************************************************************************
 * FunctionName   : KeySan()
 * Description    : ������ȡ
 * EntryParameter : None
 * ReturnValue    : ����ֵ
 *******************************************************************************/
KEY_e KeySan(void)
{
    while (keymsg.status == KEY_UP && !ExitMenu_flag)
    {
    }
    keymsg.status = KEY_UP;
    return keymsg.key;
}
/******************************************************************************
 * FunctionName   : KeySan()
 * Description    : ������ȡ
 * EntryParameter : None
 * ReturnValue    : ����ֵ
 *******************************************************************************/
KEY_e KeySan_motor(void)
{
    while (keymsg.status == KEY_UP && !ExitMenu_flag)
    {
        lcd_showstr(0, 7, "varL:");
        lcd_showint16(50, 7, SI.varL[0]);
    }
    keymsg.status = KEY_UP;
    return keymsg.key;
}
/******************************************************************************
 * FunctionName   : Menu_Move()
 * Description    : �˵��ƶ�
 * EntryParameter :  prmt - �˵�����, key - ����ֵ
 * ReturnValue    : ��ȷ�Ϸ���0�����򷵻�1
 ******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key)
{
    uint8 rValue = 1;

    switch (key)
    {
    case KEY_U: // ����
    {
        if (prmt->Cursor != 0) // ��겻�ڶ���
        {
            prmt->Cursor--; // �������
        }
        else // ����ڶ���
        {
            if (prmt->PageNo != 0) // ҳ��û�е���С
            {
                prmt->PageNo--; // ���Ϸ�
            }
            else
            {
                prmt->Cursor = prmt->DispNum - 1; // ��굽��
                prmt->PageNo = prmt->MaxPage - 1; // ���ҳ
            }
        }
        break;
    }

    case KEY_D: // ����
    {
        if (prmt->Cursor < prmt->DispNum - 1) // ���û�е��ף��ƶ����
        {
            prmt->Cursor++; // ��������ƶ�
        }
        else // ��굽��
        {
            if (prmt->PageNo < prmt->MaxPage - 1) // ҳ��û�е��ף�ҳ���ƶ�
            {
                prmt->PageNo++; // �·�һҳ
            }
            else // ҳ��͹�궼���ף����ؿ�ʼҳ
            {
                prmt->Cursor = 0;
                prmt->PageNo = 0;
            }
        }
        break;
    }
    case KEY_B: // ȷ��
    {
        prmt->Index = prmt->Cursor + prmt->PageNo; // ����ִ���������
        rValue = 0;

        break;
    }
    case KEY_L: // ��������ϼ��˵�
    {
        // prmt->Cursor = 0;
        // prmt->PageNo = 0;
        prmt->ExitMark = 1;

        break;
    }
    case KEY_R: // �Ҽ������ײ�
    {
        prmt->Cursor = prmt->DispNum - 1; // ��굽��
        prmt->PageNo = prmt->MaxPage - 1; // ���ҳ

        break;
    }

    default:
        break;
    }
    return rValue; // ����ִ������
}

void adjustParam(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor)
{
    do
    {
        KeySan();

        switch (keymsg.key)
        {
        case KEY_U:
            if (*param <= 65534)
                (*param)++;
            break;

        case KEY_D:
            if (*param >= 1)
                (*param)--;
            break;

        case KEY_L:
            if (*param >= 10)
                (*param) -= 10;
            break;

        case KEY_R:
            if (*param <= 65525)
                (*param) += 10;
            break;

        default:
            break;
        }
        tft180_show_uint(site.x, site.y, *param, 4);
    } while (keymsg.key != KEY_B);
}

void Menu_Null()
{
    // DELAY_MS(100);
}

void Write_EEPROM(void) // һҳ��һ������
{
    // һ����12������
    // ÿ��������1024ҳ
    // һ����96KB
    // һҳֻ��8���ֽڣ�������32λ��4��16λ
    uint16 EEPROM_DATA_NUM = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    //    uint32 pageNum = 0;
    //    uint32 sectorNum = 0;

    // ��鵱ǰҳ�Ƿ������ݣ��������������Ҫ��������������
    if (flash_check(0, 11))      // �ж��Ƿ�������
        flash_erase_page(0, 11); // ������һҳ

    for (uint16 i = 0; i < EEPROM_DATA_NUM; ++i)
    {
        flash_union_buffer[i].uint32_type = (uint32)*EEPROM_DATA[i];
    }
    flash_write_page_from_buffer(0, 11);
    //        flash_write_page(sectorNum, pageNum, EEPROM_DATA[i], 32);
    //        pageNum = pageNum + 1;
    //        if (pageNum == 11)
    //        {
    //            sectorNum = sectorNum + 1;
    //            pageNum = 0;
    //            //          if(flash_check(sectorNum, pageNum))
    //            //          {
    //            flash_erase_page(sectorNum,pageNum);
    //            //          }
    //        }
    flash_buffer_clear(); // �������

    tft180_show_string(0, 0, "WRITE IS OK!");
    system_delay_ms(500);
}

void Read_EEPROM(void)
{
    uint16 EEPROM_DATA_NUM = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    //    uint32 pageNum = 0;
    //    uint32 sectorNum = 0;
    ////    uint32 data_temp;
    //
    //    tft180_set_color(RED, WHITE);
    //    tft180_show_string(0, 0, "reading...");
    //    tft180_set_color(RED, WHITE);
    //    tft180_show_uint(0, 16, EEPROM_DATA_NUM, 4);

    flash_buffer_clear();             // �������
    flash_read_page_to_buffer(0, 11); // �����ݴӻ�����������
    tft180_show_string(0, 0, "reading...");
    tft180_set_color(RED, WHITE);
    tft180_show_uint(0, 16, EEPROM_DATA_NUM, 4);
    system_delay_ms(1000);
    for (uint16 i = 0; i < EEPROM_DATA_NUM; ++i)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA[i] = temp_vaule;

        //        pageNum = pageNum + 1;
        //        if (pageNum == 11)
        //        {
        //            pageNum = 0;
        ////            sectorNum = sectorNum + 1;
        //        }
    }
}
