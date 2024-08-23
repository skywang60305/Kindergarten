/*
 * img2sd.c
 *
 *  Created on: 2021��8��4��
 *      Author: 95159
 */
#include "img2sd.h"
extern EulerAngleTypedef angle;

sdCardParam sdcardParam = {
    SDOFF,
    allmapMode,
    AutoMode,
    SECTOR_START,
    0,
    0,
    FORBID,
    6,
};
#define BYTE0(x) (*((char *)(&x)))
#define BYTE1(x) (*((char *)(&x) + 1))
#define BYTE2(x) (*((char *)(&x) + 2))
#define BYTE3(x) (*((char *)(&x) + 3))

uint16 sdInfo = SDOFF << 8 | imgGrayMode;

uint8 isOK;
uint16 sector;
uint8 paper1_2 = 1;
void allmap2sd(uint8 *sdBuff)
{
    uint8 *ptr = &allmap[0][0];
    uint8 *ptrEnd = &allmap[YY][XX] + 1;
    //    uint8 *ptr = &basemap[0][0];
    //    uint8 *ptrEnd = &basemap[YY][XX] + 1;
    uint8 i = 0;
    while (ptr != ptrEnd)
    {
        *sdBuff |= !*ptr++ << i++;
        if (i == 8)
        {
            i = 0;
            ++sdBuff;
        }
    }
}

void sd2allmap(uint8 *sdBuff)
{
    uint8 i = 0;
    uint8 *ptr = allmap[0];
    uint8 *ptrEnd = &allmap[YY][XX] + 1;
    uint8 *ptrImg = &sdBuff[159];
    while (ptr != ptrEnd)
    {
        if (*ptrImg & 0x01 << i++)
        {
            *ptr++ = 0;
        }
        else
        {
            *ptr++ = 255;
        }
        if (i == 8)
        {
            i = 0;
            ++ptrImg;
        }
    }
}
uint8 sd_init(void)
{
    uint8 status = 1;
    isOK = 0;
    status = sd_initialize();

    if (status == 0)
    {
        isOK = 1;
        sector = SECTOR_START;
    }

    return isOK;
}
uint8 sd_write(void)
{
    if (isOK == 0)
    {
        return 0;
    }

    if (sdcardParam.recordImgMode == allmapMode)
    {
        uint8 sdBuff[512] = {0};
        uint16 sdNum = 0;
        sdBuff[sdNum++] = 'b'; //->0
        sdBuff[sdNum++] = 'o'; //->1
        sdBuff[sdNum++] = 'o'; //->2
        sdBuff[sdNum++] = 'm'; //->3
        sdBuff[sdNum++] = '7'; //->4

        int16 pitch = (int16)(angle.Pitch * 100);
        sdBuff[sdNum++] = BYTE1(pitch);
        sdBuff[sdNum++] = BYTE0(pitch); //->6
        int16 yaw = (int16)(angle.Yaw * 100);
        sdBuff[sdNum++] = BYTE1(yaw);
        sdBuff[sdNum++] = BYTE0(yaw); //->8
        int16 roll = (int16)(angle.Roll * 100);
        sdBuff[sdNum++] = BYTE1(roll);
        sdBuff[sdNum++] = BYTE0(roll); //->10
        sdBuff[sdNum++] = BYTE1(SI.aimSpeedL);
        sdBuff[sdNum++] = BYTE0(SI.aimSpeedL);
        sdBuff[sdNum++] = BYTE1(SI.aimSpeedR);
        sdBuff[sdNum++] = BYTE0(SI.aimSpeedR);
        sdBuff[sdNum++] = BYTE1(SI.varL[0]);
        sdBuff[sdNum++] = BYTE0(SI.varL[0]);
        sdBuff[sdNum++] = BYTE1(SI.varR[0]);
        sdBuff[sdNum++] = BYTE0(SI.varR[0]); //->18
        int16 dev = (int16)(DeviationVar.nowDeviation);
        sdBuff[sdNum++] = BYTE1(dev);
        sdBuff[sdNum++] = BYTE0(dev); //->20
        sdBuff[sdNum++] = BYTE1(PRT.whileTime);
        sdBuff[sdNum++] = BYTE0(PRT.whileTime);
        sdBuff[sdNum++] = BYTE1(servoPID.PID_Out);
        sdBuff[sdNum++] = BYTE0(servoPID.PID_Out);
        sdBuff[sdNum++] = BYTE1(SI.motorPWML);
        sdBuff[sdNum++] = BYTE0(SI.motorPWML);
        sdBuff[sdNum++] = BYTE1(SI.motorPWMR);
        sdBuff[sdNum++] = BYTE0(SI.motorPWMR); //->28
        uint8 sp;
        switch (speedType)
        {
        case NORMAL_SHIFT:
            sp = 0;
            break;
        case FULL_ACCELE:
            sp = 1;
            break;
        case BRAKE:
            sp = 2;
            break;
        default:
            break;
        }
        sdBuff[sdNum++] = BYTE0(sp); //->29
        switch (brakeType)
        {
        case STRIGHT_BRAKE_NORMAL:
            sp = 0;
            break;
        case STRIGHT_BRAKE_CURVE:
            sp = 1;
            break;
        case MINSP_BRAKE:
            sp = 2;
            break;
        default:
            break;
        }
        sdBuff[sdNum++] = BYTE0(sp); //->30

        sdBuff[sdNum++] = BYTE0(II.top);
        sdBuff[sdNum++] = BYTE0(II.speedTop); //->32

        sdBuff[sdNum++] = BYTE1(PRT._whileTime);
        sdBuff[sdNum++] = BYTE0(PRT._whileTime); //->34

        sdBuff[sdNum++] = BYTE1(TFMINI_PLUS_DISTANCE);
        sdBuff[sdNum++] = BYTE0(TFMINI_PLUS_DISTANCE); //->36
        //        sdBuff[sdNum++] = BYTE0(L_AD[0]);
        //        sdBuff[sdNum++] = BYTE0(L_AD[1]);
        //        sdBuff[sdNum++] = BYTE0(L_AD[2]);
        //        sdBuff[sdNum++] = BYTE0(L_AD[3]);
        //        sdBuff[sdNum++] = BYTE0(L_AD[4]);

        sdNum = 159;
        allmap2sd(sdBuff + sdNum);
        sd_writeDisk(sdBuff, sector++, 1);
    }
    else if (sdcardParam.recordImgMode == imgGrayMode)
    {
        sd_writeDisk(imgGray[0], sector, 23);
        sector += 23;
    }
    return 1;
}

void sd_stop()
{
    uint16 sdNum = 0;
    uint8 sdBuff[4] = {0};
    sdBuff[sdNum++] = 's'; //->0
    sdBuff[sdNum++] = 't'; //->1
    sdBuff[sdNum++] = 'o'; //->2
    sdBuff[sdNum++] = 'p'; //->3
    sd_writeDisk(sdBuff, sector++, 1);
}

uint8 SDread(uint16 sector)
{
    if (0 == isOK)
    {
        return 0;
    }
    uint8 sdBuff[512] = {0};
    sd_readDisk(sdBuff, sector, 1);
    if (!(sdBuff[0] == 'b' && sdBuff[1] == 'o' &&
          sdBuff[2] == 'o' && sdBuff[3] == 'm' && sdBuff[4] == '7'))
    {
        return 0;
    }
    SI.aimSpeedL = (sdBuff[11] << 8) + (sdBuff[12]);
    SI.aimSpeedR = (sdBuff[13] << 8) + (sdBuff[14]);
    DeviationVar.nowDeviation = (sdBuff[19] << 8) + (sdBuff[20]);
    servoPID.PID_Out = (sdBuff[23] << 8) + (sdBuff[24]);
    speedType = sdBuff[29];
    brakeType = sdBuff[30];
    II.top = sdBuff[31];
    II.speedTop = sdBuff[32];

    sd2allmap(sdBuff);
    return 1;
}
void sdreverse(void)
{
    sdcardParam.currentSector = SECTOR_START;
    lcd_clear_all(WHITE);
    while (1)
    {
        KEY_e key;
        key = KeySan();
        switch (key)
        {
        case KEY_U:
            if (sdcardParam.currentSector > 0)
            {
                sdcardParam.currentSector--;
            }
            SDread(sdcardParam.currentSector);
            if (paper1_2 == 1)
                showpaper1();
            else
                showpaper2();
            showallmap();
            break;
        case KEY_L:
            do
            {
                SDread(sdcardParam.currentSector);
                if (paper1_2 == 1)
                    showpaper1();
                else
                    showpaper2();
                showallmap();
                sdcardParam.currentSector++;
            } while (KEY_UP == keymsg.status && keymsg.key == KEY_L);
            break;
        case KEY_R:
            SDread(sdcardParam.currentSector);
            paper1_2 = 2;
            lcd_clear_all(WHITE);
            showpaper2();
            showallmap();
            break;
        case KEY_D:
            sdcardParam.currentSector++;
            SDread(sdcardParam.currentSector);
            if (paper1_2 == 1)
                showpaper1();
            else
                showpaper2();
            showallmap();
            break;
        case KEY_B:
            SDread(sdcardParam.currentSector);
            paper1_2 = 1;
            lcd_clear_all(WHITE);
            showpaper1();
            showallmap();
            break;
        default:
            break;
        }
    }
}
void showpaper1()
{
    //    lcd_clear_all(WHITE);
    tft180_set_color(BLUE, WHITE);
    tft180_show_string(100, 0, "aimS:");
    tft180_show_int(100, 16, SI.aimSpeed, 4);
    tft180_show_string(100, 32, "nowS:");
    tft180_show_int(100, 48, SI.nowSpeedL, 4);
    tft180_show_string(100, 64, " nowD:");
    tft180_show_int(100, 80, DeviationVar.nowDeviation, 4);
    tft180_show_string(100, 96, "servo:");
}
void showpaper2()
{
    //    lcd_clear_all(WHITE);
    tft180_set_color(BLUE, WHITE);
    tft180_show_string(100, 0, "top:");
    tft180_show_int(100, 15, II.top, 4);
    tft180_show_string(100, 30, "Stop:");
    tft180_show_int(100, 45, II.speedTop, 4);
    tft180_show_string(100, 59, "Stype:");
    switch (speedType)
    {
    case NORMAL_SHIFT:
        m_lcd_showstr(100, 75, "NORMAL", BLUE, WHITE);
        break;
    case FULL_ACCELE:
        m_lcd_showstr(100, 75, "FULLA", BLUE, WHITE);
        break;
    case BRAKE:
        m_lcd_showstr(100, 75, "BRAKE", BLUE, WHITE);
        break;
    default:
        break;
    }
    m_lcd_showstr(100, 89, "Btype:", BLUE, WHITE);
    switch (brakeType)
    {
    case STRIGHT_BRAKE_NORMAL:
        if (speedType == BRAKE)
            m_lcd_showstr(100, 105, "to nor", RED, WHITE);
        else
            m_lcd_showstr(100, 105, "NONE  ", BLUE, WHITE);
        break;
    case STRIGHT_BRAKE_CURVE:
        if (speedType == BRAKE)
            m_lcd_showstr(100, 105, "to cur", RED, WHITE);
        else
            m_lcd_showstr(100, 105, "NONE  ", BLUE, WHITE);
        break;
    case MINSP_BRAKE:
        if (speedType == BRAKE)
            m_lcd_showstr(100, 105, "to min", RED, WHITE);
        else
            m_lcd_showstr(100, 105, "NONE  ", BLUE, WHITE);
        break;
    default:
        break;
    }
}

void sd_clear()
{
    lcd_clear_all(WHITE);
    lcd_showstr(20, 1, "clearing sector:");
    for (uint16 sec = 0; sec < 5000; sec++)
    {
        sd_stop();
        sector += 1;
        if (2 * sec % 100 == 0)
            lcd_showuint16(20, 2, 2 * sec);
    }
    sector = 0;
    lcd_clear_all(WHITE);
}
