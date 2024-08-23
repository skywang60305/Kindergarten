/*
 * lcd.c
 *
 *  Created on: 2020年11月19日
 *      Author: Samurai
 */

#include "lcd.h"

// extern int32 Curvature;

volatile LCD_SHOW_MODE LCD_STATE = LCD_SHOW_MAP;
KEY_e key_t[2] = {KEY_B};
uint8 display_flag[2] = {0};
static uint8 keyShowlearFlag[2] = {0};
static uint8 keyCallBackClearFlag[2] = {0};
static uint8 LcdClearFlag[2] = {0}; // 清屏幕用的  全局的static只有该文件能用
extern Difference;
uint8 MapClearFlag = 0;
/*
 * LCD显示函数，用的BOOM5版本
 */

void displayLCD_callBackVersion() // 回调函数版本
{
    LcdClearFlag[1] = LcdClearFlag[0];
    if (LCD_STATE == LCD_SHOW_MAP) // 显示搜图及相关信息
    {

        callBackDisplayLCD(displayLCD_Map);
    }
    else if (LCD_STATE == LCD_SHOW_LINES) // 显示扫线及相关信息
    {
        callBackDisplayLCD(displayLCD_Lines);
    }
    else if (LCD_STATE == LCD_SHOW_DEBUG)
    {
        callBackDisplayLCD(displayLCD_Debug); // 调整标准线和逆透视  调整CCD
    }
    //    else if(LCD_STATE==LCD_SHOW_ANGLE)
    //    {
    //        callBackDisplayLCD(displayLCD_AngleAndSpeed);       //显示角度和速度等
    //    }
    //    else if(LCD_STATE==LCD_SHOW_STABILITY)
    //    {
    //        callBackDisplayLCD(displayLCD_STABILITY);           //显示时间和稳定性
    //    }
    else if (LCD_STATE == LCD_SHOW_STOP_INFORM) // 停车后显示（速度小于5） 各特殊要素的个数，不参与按键的那个显示 在ISR中可以看到
    {
        callBackDisplayLCD(displayLCD_StopInform);
    }
    //    else
    //    {
    //        callBackDisplayLCD(displayLCD_NULL);                //清除状态
    //    }
}

void callBackDisplayLCD(void (*event)(void)) // LCD的回调函数版本
{
    // 清屏
    LcdClearFlag[0] = LCD_STATE;
    if (LcdClearFlag[0] != LcdClearFlag[1])
    {
        tft180_full(WHITE);
    }
    // 执行回调函数
    event();
}

// 使用有参数的回调函数时,回调注册函数中必须也加入参数,注意回调函数只适用于同类型的操作
// 回调注册函数
void keyEventOfMyCallBack(void (*event)(void))
{
    // 执行共有的操作
    keyCallBackClearFlag[1] = keyCallBackClearFlag[0];
    keyCallBackClearFlag[0] = keymsg.key;
    if (keyCallBackClearFlag[0] != keyCallBackClearFlag[1])
    {
        tft180_full(WHITE);
    }
    // 执行各自的操作
    event();
}
//////////////////////////////显示图像信息///////////////////////////////////
void displayLCD_Map()
{
    if (MapClearFlag)
    {
        tft180_full(WHITE);
        MapClearFlag = 0;
    }
    switch (keymsg.key)
    {
    case KEY_B:
    {
        keyEventOfMyCallBack(showbasemap); // 显示basemap
        break;
    }
    case KEY_L:
    {
        keyEventOfMyCallBack(showleftmap); // 显示leftmap
        break;
    }
    case KEY_R:
    {
        keyEventOfMyCallBack(showrightmap); // 显示rightmap
        break;
    }
    case KEY_U:
    {
        keyEventOfMyCallBack(showdeletemap); // 显示deletemap
        break;
    }
    case KEY_D:
    {
        keyEventOfMyCallBack(showinsidemap); // 显示insidemap
        break;
    }
    default:
    {
        keyEventOfMyCallBack(showbasemap);
        break;
    }
    }
}
/////////////////////////////////////显示扫线信息////////////////////////
void displayLCD_Lines()
{
    if (MapClearFlag)
    {
        tft180_full(WHITE);
        MapClearFlag = 0;
    }
    switch (keymsg.key)
    {
    case KEY_B:
    {
        keyEventOfMyCallBack(keyB_ShowLinesEvent); // 显示left,rightline
        break;
    }
    case KEY_L:
    {
        keyEventOfMyCallBack(keyL_ShowLinesEvent); // 显示leftline
        break;
    }
    case KEY_R:
    {
        keyEventOfMyCallBack(keyR_ShowLinesEvent); // 显示rightline
        break;
    }
    case KEY_U:
    {
        keyEventOfMyCallBack(keyU_ShowLinesEvent); // 显示二值化图
        break;
    }
        //        case KEY_D:
        //        {
        //            keyEventOfMyCallBack(keyD_ShowLinesEvent);       //显示RI
        //            break;
        //        }
        //        default:
        //        {
        //            keyEventOfMyCallBack(keyB_ShowLinesEvent);
        //            break;
        //        }
        //
    }
}

void displayLCD_Debug()
{
    switch (keymsg.key)
    {
    case KEY_B:
    {
        keyEventOfMyCallBack(keyB_ShowDebugEvent);
        break;
    }
    case KEY_L:
    {
        keyEventOfMyCallBack(keyL_ShowDebugEvent);
        break;
    }
    case KEY_R:
    {
        keyEventOfMyCallBack(keyR_ShowDebugEvent);
        break;
    }
    case KEY_U:
    {
        keyEventOfMyCallBack(keyU_ShowDebugEvent);
        break;
    }
        //        case KEY_D:
        //        {
        //            keyEventOfMyCallBack(keyD_ShowDebugEvent);
        //            break;
        //        }
        //        default:
        //        {
        //            keyEventOfMyCallBack(keyB_ShowDebugEvent);
        //            break;
        //        }
    }
}
// void displayLCD_AngleAndSpeed()
//{
//     switch (keymsg.key)
//     {
//         case KEY_B:
//         {
//             keyEventOfMyCallBack(keyB_ShowAngleAndSpeedEvent);  //显示角度参数
//             break;
//         }
//         case KEY_L:
//         {
//             keyEventOfMyCallBack(keyL_ShowAngleAndSpeedEvent);  //速度参数
//             break;
//         }
//         case KEY_R:
//         {
//             keyEventOfMyCallBack(keyR_ShowAngleAndSpeedEvent);  //备用
//             break;
//         }
//         case KEY_U:
//         {
//             keyEventOfMyCallBack(keyU_ShowAngleAndSpeedEvent);  //备用
//             break;
//         }
//         case KEY_D:
//         {
//             keyEventOfMyCallBack(keyD_ShowAngleAndSpeedEvent);  //备用
//             break;
//         }
//         default:
//         {
//             keyEventOfMyCallBack(keyB_ShowAngleAndSpeedEvent);
//             break;
//         }
//     }
// }

// void displayLCD_STABILITY()  //稳定性分析  显示时间  和跳频次数    加入计算所有sd卡次数和所有超过18ms的次数,算成百分比
//{
//     switch (keymsg.key)
//     {
//         case KEY_B:
//         {
//             keyEventOfMyCallBack(keyB_ShowStabilityEvent);      //显示程序稳定性
//             break;
//         }
//         case KEY_L:
//         {
//             keyEventOfMyCallBack(keyL_ShowStabilityEvent);           //SD卡稳定性  用5ms的来计算
//             break;
//         }
//         case KEY_R:
//         {
//             keyEventOfMyCallBack(keyR_ShowStabilityEvent);         //显示波形时间
//             break;
//         }
//         case KEY_U:
//         {
//             keyEventOfMyCallBack(keyU_ShowStabilityEvent);      //备用
//             break;
//         }
//         case KEY_D:
//         {
//             keyEventOfMyCallBack(keyB_ShowStabilityEvent);      //备用
//             break;
//         }
//         default:
//         {
//             keyEventOfMyCallBack(keyB_ShowStabilityEvent);
//             break;
//         }
//     }
// }
//
void displayLCD_StopInform()
{

    //    float programTemp=programStability;
    float sdTemp = sdStability;

    tft180_set_color(BLACK, WHITE);
    tft180_show_string(0, 0, "yh:"); // 圆环
    tft180_show_uint(60, 0, judgedAnnulusNum, 1);

    tft180_show_string(0, 1 * 16, "ramp:"); // 坡道
    tft180_show_uint(60, 1 * 16, judgedRampNum, 1);

    //    tft180_show_string(0, 30, BLACK, WHITE, "Bramp");                        //大坡
    //    lcd_showint16(60, 30, BLACK, WHITE,bigRampNum );

    // tft180_show_string  (0, 2 * 16, "Cross:");                        //十字
    // tft180_show_uint    (60,2 * 16, judgedCrossroadsNum, 1);

    //    tft180_show_string(0, 45, "Program");                      //程序稳定性
    //    tft180_set_color(BLUE, WHITE);
    //    tft180_show_float(60, 45, programTemp*100.0 ,4,2 );

    tft180_set_color(BLACK, WHITE);
    //    tft180_show_string(110, 45, "%");
    tft180_show_string(0, 5 * 16, "SD"); // SD卡稳定性

    tft180_set_color(BLUE, WHITE);
    tft180_show_float(60, 5 * 16, sdTemp * 100.0, 4, 2);

    //    tft180_set_color(BLACK, WHITE);
    //    tft180_show_string(110, 60, "%");
    //    tft180_show_string(0, 75, "MaxT");                         //最大时间  MS

    //    tft180_set_color(BLUE, WHITE);
    //    tft180_show_int(60, 75,(uint16)(maxAllTime/1000), 4);

    //    tft180_show_string(0, 90, "aveError", BLACK, WHITE);                         //最大时间  MS
    //    tft180_show_float(80, 90, BLUE, WHITE,motor_sumError_left/cnt_motorPID_period, 2 ,2);
    //    tft180_show_float(120, 90, BLUE, WHITE,motor_sumError_right/cnt_motorPID_period, 2 ,2);
    tft180_set_color(BLUE, WHITE);
    tft180_show_string(0, 6 * 16, "Time: "); // 最大时间  MS
    tft180_show_int(40, 6 * 16, runningTime / 2.5, 4);
    tft180_set_color(BLACK, RED);
    tft180_show_string(0, 7 * 16, "stop:"); // 停车原因
    switch (stopReason)
    {
    case HaventStarted:
        tft180_show_string(60, 7 * 16, "HaventStarted");
        break;
    case TimeUp:
        tft180_show_string(60, 7 * 16, "TimeUp");
        break;
    case RunOutLine:
        tft180_show_string(60, 7 * 16, "RunOutLine");
        break;
    case EnterGarage:
        tft180_show_string(60, 7 * 16, "EnterGarage");
        break;
    case StallProtect:
        tft180_show_string(60, 7 * 16, "StallProtect");
        break;
    }
}
//
// void displayLCD_NULL()
//{
//    tft180_show_string(40, 0, "Please "  , RED, WHITE);
//    tft180_show_string(40, 20, "Close" , RED, WHITE);
//    tft180_show_string(40, 40, "LCD" , RED, WHITE);
//    tft180_show_string(40, 60, "Switch" , RED, WHITE);
//    tft180_show_string(40, 80, "To" , RED, WHITE);
//    tft180_show_string(40, 100, "Run", RED, WHITE);
//
//}
//
///**********************************显示搜图*************************************/
// void keyB_ShowMapEvent()
//{
//     showbasemap();
// }
//
// void keyL_ShowMapEvent()
//{
//     showleftmap();
// }
//
// void keyU_ShowMapEvent()
//{
//     showdeletemap();
// }
//
// void keyR_ShowMapEvent()
//{
//     showrightmap();
// }
//
// void keyD_ShowMapEvent()
//{
//     showinsidemap();
// }

///**********************************显示扫线*************************************/
void keyB_ShowLinesEvent() // 显示两线
{
    showBothLine();
}
void keyL_ShowLinesEvent() // 显示左线
{
    showLeftLine();
}

void keyU_ShowLinesEvent() // 显示最终控制线
{
    showControlLine();
}

void keyR_ShowLinesEvent() // 显示右线
{
    showRightLine();
}
//
// void keyD_ShowLinesEvent()    //显示图像信息
//{
//    showImgInfo();
//}
//
///**********************************显示调试信息*************************************/
void keyB_ShowDebugEvent()
{
    //    这里给MT9V03X_H * 2，可以把二值化图也打出来，因为两数组地址相邻
    //    tft180_displayimage03x((const uint8 *)imgGRAY[0], 160, 128);
    tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 60);
}
//
void keyL_ShowDebugEvent()
{
    showBMXData();
}

void keyU_ShowDebugEvent()
{
    showImgInfo();
}

void keyR_ShowDebugEvent()
{
    //    adc_Info();
}
//
// void keyD_ShowDebugEvent()
//{
//    showDynamicParam();
//}
//
///**********************************显示角度与速度信息*************************************/
// void keyB_ShowAngleAndSpeedEvent()
//{
//     showBMXData();
// }
//
// void keyL_ShowAngleAndSpeedEvent()
//{
//     showMotorPIDParam();
// }
//
// void keyU_ShowAngleAndSpeedEvent()
//{
//     showTFMiniData();
// }
//
// void keyR_ShowAngleAndSpeedEvent()
//{
//     showinfo();
// }
//
// void keyD_ShowAngleAndSpeedEvent()
//{
//     showSpeedInfo();
// }
//
///**********************************显示各种稳定度*************************************/
// void keyB_ShowStabilityEvent()
//{
//     tft180_show_string(60, 0, "  -= Program =-  ", RED, WHITE);
//
//     tft180_show_string(0, 16, "Per", BLACK, WHITE);
//     tft180_show_float(60, 16, BLUE, WHITE,programStability*100.0 ,4,2 );
//     tft180_show_string(110, 16, "%", BLACK, WHITE);
//
//     tft180_show_string(0, 32, "while", BLACK, WHITE);
//     tft180_show_uint(70, 32,  PRT.whileTime, BLUE, WHITE);
//
//     tft180_show_string(0, 48, "all", BLACK, WHITE);
//     tft180_show_uint(70, 48,  PRT.allTime, BLUE, WHITE);
//
//     tft180_show_string(0, 64, "sd", BLACK, WHITE);
//     tft180_show_uint(70, 64,  PRT.sdSaveTime, BLUE, WHITE);
//
//     tft180_show_string(0, 80, "proCnt", BLACK, WHITE);
//     tft180_show_uint(70, 80,  programCnt, BLUE, WHITE);
//
//     //    tft180_show_string(0, 96, "overCnt", BLACK, WHITE);
//     //    tft180_show_uint(70, 96,  programOver18Cnt, BLUE, WHITE);
//
//     tft180_show_string(0, 112, "goTime", BLACK, WHITE);
//     tft180_show_uint(70, 112,PRT.goTime, BLUE, WHITE);
//
//     tft180_show_string(0, 96, "_while", BLACK, WHITE);
//     tft180_show_uint(70, 96,PRT._whileTime, BLUE, WHITE);
//
// }
// void keyL_ShowStabilityEvent()
//{
//
//     tft180_show_string(60, 0, "SD", RED, WHITE);
//
//     tft180_show_string(0, 16, "Per", BLACK, WHITE);
//     tft180_show_float(60, 16, BLUE, WHITE,sdStability*100.0 ,4,2 );
//     tft180_show_string(110, 16, "%", BLACK, WHITE);
//
//     tft180_show_string(0, 32, "sd", BLACK, WHITE);
//     tft180_show_uint(70, 32, PRT.sdSaveTime, BLUE, WHITE);
//
//     tft180_show_string(0, 48, "while", BLACK, WHITE);
//     tft180_show_uint(70, 48,  PRT.whileTime, BLUE, WHITE);
//
//     tft180_show_string(0, 64, "3Cnt", BLACK, WHITE);
//     tft180_show_uint(70, 64, over3msTimeCnt, BLUE, WHITE);
//
//     tft180_show_string(0, 80, "5Cnt", BLACK, WHITE);
//     tft180_show_uint(70, 80, over5msTimeCnt, BLUE, WHITE);
//
//     tft180_show_string(0, 96, "8Cnt", BLACK, WHITE);
//     tft180_show_uint(70, 96, over8msTimeCnt, BLUE, WHITE);
//
//     tft180_show_string(0, 112, "saveCnt", BLACK, WHITE);
//     tft180_show_uint(70, 112, sdSaveCnt, BLUE, WHITE);
//
//
// }
//
void adc_Info() // 电磁参数
{
    tft180_set_color(RED, WHITE);
    tft180_show_string(60, 0, "ADC");
    //    tft180_show_string  (0,  1 * 16, "ADC: 0  1  2  3  4");
    //    tft180_show_uint    (30,  2 * 16, ADC[0], 3);
    //    tft180_show_uint    (55,  2 * 16, ADC[1], 3);
    //    tft180_show_uint    (80,  2 * 16, ADC[2], 3);
    //    tft180_show_uint    (105,  2 * 16, ADC[3], 3);
    //    tft180_show_uint    (130,  2 * 16, ADC[4], 3);
    //
    //    tft180_show_string  (0,  3 * 16, "nor: 0  1  2  3  4");
    //    tft180_show_uint    (30,  4 * 16, ADC_normalization[0], 3);
    //    tft180_show_uint    (55,  4 * 16, ADC_normalization[1], 3);
    //    tft180_show_uint    (80,  4 * 16, ADC_normalization[2], 3);
    //    tft180_show_uint    (105, 4 * 16, ADC_normalization[3], 3);
    //    tft180_show_uint    (130, 4 * 16, ADC_normalization[4], 3);
    // tft180_show_string(0,2,"img_Dif");  tft180_show_float(80,2,DeviationVar.nowDeviation,2,3);
    // tft180_show_string(0,3,"adc_Dif");  lcd_showint32(80,3,Difference,3);
    // tft180_show_string(0,3,"error");     lcd_showint16(80,3,error);
    // tft180_show_string(0,4,"IF.noWay");tft180_show_uint(80,4,IF.noWay);//断路标志
    // tft180_show_string(0,5,"ADC_nor[0]");tft180_show_uint(80,5,ADC_normalization[0]);
    // tft180_show_string(0,6,"ADC_nor[1]");tft180_show_uint(80,6,ADC_normalization[1]);
    // tft180_show_string(0,7,"ADC_nor[2]");tft180_show_uint(80,7,ADC_normalization[2]);
}
//
// void keyR_ShowStabilityEvent()//波形时间
//{
//
//}
//
// void keyD_ShowStabilityEvent()
//{
//
//}
//
///////////////////////////////我是分界线，在下面写自己的lcd显示函数//////////////////////////////////////////////////
// 二值化图
// void showbinarymap(const uint8 (*p)[60])
//{
//     tft180_set_region(0,0, 159, YY);
//     for(int i=0;i<CAMERA_H;i++)
//     {
//         for(int j=14;j<174;j++)
//         {
//             if(p[i][j]==0)
//             {
//                 tft180_write_16bit_data(BLACK);
//             }
//             else if(p[i][j]==255)
//             {
//                 tft180_write_16bit_data(WHITE);
//             }
//         }
//     }
// }
//
///********************************显示搜图的图像等***************************************/
void showallmap()
{
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (!(j >= 0 && ((uint8)floor(leftline[j]) == i || (uint8)floor(rightline[j]) == i)))
                {
                    if (allmap[j][i])
                    {
                        color = WHITE;
                    }
                    else
                    {
                        color = BLACK;
                    }
                }
                else
                {
                    color = PURPLE;
                }
                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}

void showbasemap()
{
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    tft180_show_string(102, 0, "BaseMap");
    showIF(102, 1 * 16);
    tft180_show_string(102, 3 * 16, "top");
    tft180_show_uint(102, 4 * 16, II.top, 4);
    tft180_show_string(102, 5 * 16, "SpTop");
    tft180_show_uint(102, 6 * 16, II.speedTop, 4);
    tft180_show_int(102, 7 * 16, II.bnum_all, 4);
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {

                if (j != II.startRow && j != II.endRow)
                {
                    if (!(j >= 0 && ((uint8)floor(leftline[j]) == i || (uint8)floor(rightline[j]) == i)))
                    {
                        if (!basemap[j][i])
                        {
                            if (leftmap[j][i] == 2 || rightmap[j][i] == 2)
                            {
                                color = RED;
                            }
                            else if (leftmap[j][i] >= 3 || rightmap[j][i] >= 3)
                            {
                                color = BLUE;
                            }
                            else if (IF.ramp && j < YM / 3 && i == midline[j] / 2)
                            {
                                color = GREEN;
                            }
                            else
                            {
                                color = WHITE;
                            }
                        }
                        else
                        {
                            color = BLACK;
                        }
                    }
                    else
                    {
                        color = PURPLE;
                    }
                }
                else
                {
                    color = YELLOW;
                }

                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}

void showleftmap()
{
    tft180_show_string(102, 0, "LeftMap");
    tft180_show_string(102, 1 * 16, "num_lm");
    tft180_show_uint(102, 2 * 16, II.num_lm, 4);
    tft180_show_string(102, 3 * 16, "lnum");
    tft180_show_uint(102, 4 * 16, II.lnum_all, 4);
    tft180_show_string(102, 5 * 16, "angleL");
    tft180_show_uint(102, 6 * 16, (int)II.angleL, 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (!leftmap[j][i])
                {
                    color = WHITE;
                }
                else if (leftmap[j][i] == 1)
                {
                    color = BLACK;
                }
                else if (leftmap[j][i] == 2)
                {
                    color = RED;
                }
                else if (leftmap[j][i] >= 3)
                {
                    color = BLUE;
                }
                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}

void showrightmap()
{
    tft180_show_string(102, 0, "RighMap");
    tft180_show_string(102, 1 * 16, "num_rm");
    tft180_show_uint(102, 2 * 16, II.num_rm, 4);
    tft180_show_string(102, 3 * 16, "rnum");
    tft180_show_uint(102, 4 * 16, II.rnum_all, 4);
    tft180_show_string(102, 5 * 16, "angleR");
    tft180_show_uint(102, 6 * 16, (int)II.angleR, 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (!rightmap[j][i])
                {
                    color = WHITE;
                }
                else if (rightmap[j][i] == 1)
                {
                    color = BLACK;
                }
                else if (rightmap[j][i] == 2)
                {
                    color = RED;
                }
                else if (rightmap[j][i] >= 3)
                {
                    color = BLUE;
                }
                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}

void showdeletemap()
{
    tft180_show_string(102, 0, "deleMap");
    tft180_show_string(102, 1 * 16, "dnum");
    tft180_show_uint(102, 2 * 16, II.dnum_all, 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (!deletemap[j][i])
                {
                    color = WHITE;
                }
                else if (deletemap[j][i] == 1)
                {
                    color = BLACK;
                }
                else if (deletemap[j][i] == 2)
                {
                    color = RED;
                }
                else if (deletemap[j][i] == 254)
                {
                    color = BLACK;
                }
                else if (deletemap[j][i] == 253)
                {
                    color = BLUE;
                }
                else
                {
                    color = CYAN;
                }
                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}

void showinsidemap()
{
    tft180_show_string(102, 0, "insiMap");
    tft180_show_string(102, 1 * 16, "sta_Lin");
    tft180_show_uint(102, 2 * 16, twentyCmGarageJudge(1), 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (!insidemap[j][i])
                {
                    color = WHITE;
                }
                else if (insidemap[j][i] == 1)
                {
                    color = BLACK;
                }
                else if (insidemap[j][i] == 2)
                {
                    color = RED;
                }
                else
                {
                    color = CYAN;
                }
                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}
//
///**********************************扫线方面显示函数**************************************/
/*显示两线 左为红 右为蓝 中为绿*/
void showBothLine()
{
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    tft180_show_string(100, 0, "Mid:");
    tft180_show_uint(100, 1 * 16, II.searchLineMid, 4);
    tft180_show_string(100, 2 * 16, "BreakY:");
    tft180_show_uint(100, 3 * 16, II.breakY, 4);
    tft180_show_string(100, 4 * 16, "RowL:");
    tft180_show_uint(100, 5 * 16, II.leftSearchRow, 4);
    tft180_show_string(100, 6 * 16, "RowR:");
    tft180_show_uint(100, 7 * 16, II.rightSearchRow, 4);
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (int i = YY, flag = 0; i >= 0; i--)
    {
        do
        {
            flag = 1 - flag;
            if (i == II.leftSearchRow || i == II.rightSearchRow)
            {
                if (i == II.leftSearchRow && i != II.rightSearchRow)
                {
                    for (int j = 0; j < II.searchLineMid; j++)
                    {
                        tft180_write_16bit_data(PURPLE);
                        tft180_write_16bit_data(PURPLE);
                    }
                    for (int j = II.searchLineMid; j < XM; j++)
                    {
                        tft180_write_16bit_data(WHITE);
                        tft180_write_16bit_data(WHITE);
                    }
                }
                else if (i != II.leftSearchRow && i == II.rightSearchRow)
                {
                    for (int j = 0; j < II.searchLineMid; j++)
                    {
                        tft180_write_16bit_data(WHITE);
                        tft180_write_16bit_data(WHITE);
                    }
                    for (int j = II.searchLineMid; j < XM; j++)
                    {
                        tft180_write_16bit_data(PURPLE);
                        tft180_write_16bit_data(PURPLE);
                    }
                }
                else
                {
                    for (int j = 0; j < XM; j++)
                    {
                        tft180_write_16bit_data(PURPLE);
                        tft180_write_16bit_data(PURPLE);
                    }
                }
                continue;
            }
            for (int j = 0; j < XM; j++)
            {
                if (j == II.searchLineMid)
                {
                    tft180_write_16bit_data(PINK);
                    tft180_write_16bit_data(PINK);
                }
                else if (j == II.leftline[i] && II.leftfindflag[i])
                {
                    tft180_write_16bit_data(RED);
                    tft180_write_16bit_data(RED);
                }
                else if (j == II.rightline[i] && II.rightfindflag[i])
                {
                    tft180_write_16bit_data(BLUE);
                    tft180_write_16bit_data(BLUE);
                }
                else if (j == II.midline[i] && II.midfindflag[i])
                {
                    tft180_write_16bit_data(GREEN);
                    tft180_write_16bit_data(GREEN);
                }
                else if (i == II.bottomlineLeft[j])
                {
                    tft180_write_16bit_data(GRAY);
                }
                else if (i == II.bottomlineRight[j])
                {
                    tft180_write_16bit_data(MAGENTA);
                    tft180_write_16bit_data(MAGENTA);
                }
                else if (i == II.highlineLeft[j])
                {
                    tft180_write_16bit_data(GRAY);
                    tft180_write_16bit_data(GRAY);
                }
                else if (i == II.highlineRight[j])
                {
                    tft180_write_16bit_data(MAGENTA);
                    tft180_write_16bit_data(MAGENTA);
                }
                else
                {
                    tft180_write_16bit_data(WHITE);
                    tft180_write_16bit_data(WHITE);
                }
            }

        } while (flag);
    }
    TFT180_CS(1);
}

void showLeftLine()
{
    tft180_show_string(102, 0, "LeftLin");
    tft180_show_string(102, 1 * 16, "num_lm");
    tft180_show_uint(102, 2 * 16, II.num_lm, 4);
    tft180_show_string(102, 3 * 16, "lnum");
    tft180_show_uint(102, 4 * 16, II.lnum_all, 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (int i = YY, flag = 0; i >= 0; i--)
    {
        do
        {
            flag = 1 - flag;
            if (i == II.leftSearchRow)
            {

                for (int j = 0; j < II.searchLineMid; j++)
                {
                    tft180_write_16bit_data(PURPLE);
                    tft180_write_16bit_data(PURPLE);
                }
                for (int j = II.searchLineMid; j < XM; j++)
                {
                    tft180_write_16bit_data(WHITE);
                    tft180_write_16bit_data(WHITE);
                }
                continue;
            }
            for (int j = 0; j < XM; j++)
            {
                if (j == II.searchLineMid)
                {
                    tft180_write_16bit_data(PINK);
                    tft180_write_16bit_data(PINK);
                }
                else if (j == II.leftline[i] && II.leftfindflag[i])
                {
                    tft180_write_16bit_data(RED);
                    tft180_write_16bit_data(RED);
                }
                else if (i == II.bottomlineLeft[j])
                {
                    tft180_write_16bit_data(GRAY);
                }
                else if (i == II.highlineLeft[j])
                {
                    tft180_write_16bit_data(GRAY);
                    tft180_write_16bit_data(GRAY);
                }
                else
                {
                    tft180_write_16bit_data(WHITE);
                    tft180_write_16bit_data(WHITE);
                }
            }

        } while (flag);
    }
    TFT180_CS(1);
}

void showRightLine()
{
    tft180_show_string(102, 0, "RigLine");
    tft180_show_string(102, 1 * 16, "num_rm");
    tft180_show_uint(102, 2 * 16, II.num_rm, 4);
    tft180_show_string(102, 3 * 16, "rnum");
    tft180_show_uint(102, 4 * 16, II.rnum_all, 4);
    for (uint8 i = 0; i < 8; i++)
    {
        tft180_show_string(94, i * 16, "|");
    }
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (int i = YY, flag = 0; i >= 0; i--)
    {
        do
        {
            flag = 1 - flag;
            if (i == II.rightSearchRow)
            {
                for (int j = 0; j < II.searchLineMid; j++)
                {
                    tft180_write_16bit_data(WHITE);
                    tft180_write_16bit_data(WHITE);
                }
                for (int j = II.searchLineMid; j < XM; j++)
                {
                    tft180_write_16bit_data(PURPLE);
                    tft180_write_16bit_data(PURPLE);
                }
                continue;
            }
            for (int j = 0; j < XM; j++)
            {
                if (j == II.searchLineMid)
                {
                    tft180_write_16bit_data(PINK);
                    tft180_write_16bit_data(PINK);
                }
                else if (j == II.rightline[i] && II.rightfindflag[i])
                {
                    tft180_write_16bit_data(BLUE);
                    tft180_write_16bit_data(BLUE);
                }
                else if (i == II.bottomlineRight[j])
                {
                    tft180_write_16bit_data(MAGENTA);
                    tft180_write_16bit_data(MAGENTA);
                }
                else if (i == II.highlineRight[j])
                {
                    tft180_write_16bit_data(MAGENTA);
                    tft180_write_16bit_data(MAGENTA);
                }
                else
                {
                    tft180_write_16bit_data(WHITE);
                    tft180_write_16bit_data(WHITE);
                }
            }
        } while (flag);
    }
    TFT180_CS(1);
}

void showControlLine()
{
    uint16 color;
    TFT180_CS(0);
    tft180_set_region(0, 4, (XM << 1) - 1, (YM << 1) + 3);
    for (uint8 j = YY, flag = 0; j < YM; --j)
    {
        do
        {
            for (uint8 i = 0; i < XM; ++i)
            {
                if (j != II.startRow && j != II.endRow)
                {
                    if (!(j >= 0 && ((uint8)floor(leftline[j]) == i || (uint8)floor(rightline[j]) == i)))
                    {
                        if (leftmap[j][i] == 4 || rightmap[j][i] == 4)
                        {
                            color = BLUE;
                        }
                        else
                        {
                            color = WHITE;
                        }
                    }
                    else
                    {
                        color = PURPLE;
                    }
                }
                else
                {
                    color = YELLOW;
                }

                tft180_write_16bit_data(color);
                tft180_write_16bit_data(color);
            }
            flag = 1 - flag;
        } while (flag);
    }
    TFT180_CS(1);
}
//
void showImgInfo()
{
    tft180_show_string(0, 0, "  -=ImgInfo=- ");

    tft180_show_string(0, 1 * 16, "Dev  :");
    tft180_show_float(48, 1 * 16, DeviationVar.nowDeviation, 3, 2);
    //    tft180_show_string(0, 2 * 16, 'Cur  :');
    //    tft180_show_int(48, 2 * 16, Curvature, 4);
    //    //
    //    //    //显示电磁偏差
    //    tft180_show_string(0, 3 * 16, "adcdif:");
    //    tft180_show_float(80, 3 * 16, new_dif, 3, 2);
    //    tft180_show_string(0, 4 * 16, "adc_error:");
    //    tft180_show_float(80, 4 * 16, adc_new_error, 3, 2);
    //    tft180_show_string(0,1,"Line :");
    //    if(controlLine==0)
    //        tft180_show_string(48,1,"BOTH");
    //    else if(controlLine==1)
    //        tft180_show_string(48,1,"LEFT");
    //    else if(controlLine==2)
    //        tft180_show_string(48,1,"RIGHT");
    //    else if(controlLine==3)
    //        tft180_show_string(48,1,"MID");
    //    else if(controlLine==4)
    //        tft180_show_string(48,1,"NOPE");

    //
    //    tft180_show_string(88,1," OUT:");
    ////    lcd_showint8(128,1,(int8)servoPID.PID_Out);
    //
    //    tft180_show_uint(48,3,II.speedTop);
    //    tft180_show_uint(48,4,II.annulusTop);
    //    tft180_show_string(0,3,"SpTop:");
    //    tft180_show_string(0,4,"AnTop:");
    //    tft180_show_uint(48,3,II.speedTop);
    //    tft180_show_uint(48,4,II.annulusTop);
    //    tft180_show_string(0,5," -=   ImgFlag   =- ");
    //    tft180_show_string(0,6,"Annu :");
    //    tft180_show_uint(40,6,IF.annulus);
    //    tft180_show_string(80,6,"Gara:");
    //    tft180_show_uint(128,6,IF.garage);
    //    tft180_show_string(0,7,"Ramp:");
    //    tft180_show_uint(40,7,IF.ramp);
    //    tft180_show_string(80,7,"Fork:");
    //    tft180_show_uint(128,7,IF.fork);
}

void showIF(uint8 x, uint8 y)
{
    tft180_show_string(x, y, "IF:");
    if (IF.annulusDelay)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "AnnDela");
    }
    else if (IF.annulus)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "Annulus");
    }
    else if (IF.crossroad)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "cross");
    }
    else if (IF.fork)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "fork");
    }
    else if (IF.garage == 254 || IF.garage == 255)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "outGara");
    }
    else if (IF.garage)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "enterGa");
    }
    else if (IF.rampDelay)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "rampDel");
    }
    else if (IF.ramp)
    {
        lcd_clear(x, y + 16, 150, y + 32, WHITE);
        tft180_show_string(x, y + 15, "ramp");
    }
    //    else if (IF.noWay)
    //    {
    //        lcd_clear(x, y + 16, 150, y + 32, WHITE);
    //        tft180_show_string(x, y + 15, "noWay");
    //    }
    //    else if (IF.obstacle)
    //    {
    //        lcd_clear(x, y + 16, 150, y + 32, WHITE);
    //        tft180_show_string(x, y + 15, "obstacl");
    //    }
    else
    {
        lcd_clear(x, y + 16, 159, y + 32, WHITE);
        tft180_show_string(x, y + 15, "none");
    }
}
//
// void reverseViewMeasurement()
//{
//    showallmap();
//    uint8 endL=0,endR=0,endM=0;
//    for(uint8 i=0;i<YM;i++)
//    {
//        if(allmap[i][23]==0)
//        {
//            endM=i;
//            break;
//        }
//    }
//    for(uint8 i=0;i<YM;i++)
//    {
//        if(allmap[i][21]==0)
//        {
//            endL=i;
//            break;
//        }
//    }
//    for(uint8 i=0;i<YM;i++)
//    {
//        if(allmap[i][25]==0)
//        {
//            endR=i;
//            break;
//        }
//    }
//    tft180_show_string(102,0,"Measure");
//    tft180_show_string(102,1,"nowRow:");
//    tft180_show_string(102,2,"L:");
//    tft180_show_uint(102,3,endL);
//    tft180_show_string(102,4,"M:");
//    tft180_show_uint(102,5,endM);
//    tft180_show_string(102,6,"R:");
//    tft180_show_uint(102,7,endR);
//    for(uint8 i=0;i<8;i++)
//    {
//        tft180_show_string(94,i,"|");
//    }
//}
//
////测这套车模的标准差速值
// void speedDiffMeasurement(void)
//{
//     float PWM;
//     float MAX_PWM;
//     PWM = (float)(SERVO_PWM_MID) - (float)(servoPWM);
//     MAX_PWM = (float)(SERVO_PWM_MAX + SERVO_PWM_MIN) / 2;
//     float speed_high, speed_low,differential;
//     if(SI.varL[0]==0 && SI.varR[0]==0)
//     {
//         speed_high = 0;
//         speed_low = 0;
//         differential = 0;
//     }
//     if(SI.varL[0]>SI.varR[0])
//     {
//         speed_high = (float)SI.varL[0];
//         speed_low = (float)SI.varR[0];
//         differential = (float)((speed_high-speed_low)/speed_high/(PWM / MAX_PWM))*100;
//     }
//     else if(SI.varL[0]>=SI.varR[0])
//     {
//         speed_high = (float)SI.varR[0];
//         speed_low = (float)SI.varL[0];
//         differential = (float)((speed_high-speed_low)/speed_high/(PWM / MAX_PWM))*100;
//     }
//
//     tft180_show_string(0,0,"  spDiffMeasure  ");
//     tft180_show_string(0,1,"sp_high  :");
//     lcd_showint16(80,1,(int)speed_high);
//     tft180_show_string(0,2,"sp_low   :");
//     lcd_showint16(80,2,(int)speed_low);
//     tft180_show_string(0,3,"diff     :");
//     tft180_show_float(80,3,differential,2,2);
// }
//
// void showDynamicParam()
//{
//     tft180_show_string(0,0," -= DynamicParam =- ");
//     tft180_show_string(0,1,"k:");
//     tft180_show_uint(40,1,kk/bb*10);
//     tft180_show_string(0,2,"b:");
//     tft180_show_uint(40,2,bb*100);
//     tft180_show_string(0,3,"p:");
//     tft180_show_uint(40,3,servoPID.kp*100);
//     tft180_show_string(0,3,"din:");
//     tft180_show_uint(40,3,servoPID.kd_in);
//     tft180_show_string(0,4,"dout:");
//     tft180_show_uint(40,4,servoPID.kd_out);
//     tft180_show_string(80,1,"maxsp");
//     tft180_show_uint(120,1,MAXSPEED);
//     tft180_show_string(80,2,"minsp");
//     tft180_show_uint(120,2,MINSPEED);
//     tft180_show_string(80,3,"nowL");
//     lcd_showint16(120,3,SI.varL[0]);
//     tft180_show_string(80,4,"nowR");
//     lcd_showint16(120,4,SI.varR[0]);
// }
//
// void showSpeedInfo()
//{
//     tft180_show_string(0, 0, "    -= Speed =-  ", RED, WHITE);
//
//     tft180_show_string(0, 16, "SpTop :",BLACK, WHITE);
//     tft180_show_uint(64, 16, II.speedTop,BLUE, WHITE );
//     tft180_show_string(0, 32, "AnTop :", BLACK, WHITE);
//     tft180_show_uint(64, 32,II.annulusTop , BLUE, WHITE);
//
//
//     tft180_show_string(0, 48, "Encoder:", BLACK, WHITE);
//     tft180_show_string(64,48,"L:",BLACK, WHITE);
//     m_lcd_showint32(80, 48, BLUE, WHITE,SI.varL[0],4 );
//     tft180_show_string(112,48,"R:",BLACK, WHITE);
//     m_lcd_showint32(128, 48, BLUE, WHITE,SI.varR[0],4 );
//
//     tft180_show_string(0, 64, "aimSp :", BLACK, WHITE);
//     m_lcd_showint32(80, 64, BLUE, WHITE,SI.aimSpeedL,4 );
//     m_lcd_showint32(120, 64, BLUE, WHITE,SI.aimSpeedR,4 );
//
//     tft180_show_string(0, 96, "pwm   :", BLACK, WHITE);
//     m_lcd_showint32(80, 96, BLUE, WHITE,SI.motorPWML,4 );
//     m_lcd_showint32(120, 96, BLUE, WHITE,SI.motorPWMR,4 );
// }
//
//
// void showMotorPIDParam()
//{
//     float kp,ki,error;
//     switch (motorControlAlgorithm){
//         case PositionalPID:
//             tft180_show_string(0, 0, " -=  PositionalPID =- ", RED, WHITE);
//             tft180_show_string(0, 16, "p        :",BLACK, WHITE);
//             tft180_show_uint(96, 16, positionalPIDParam.kp,BLUE, WHITE );
//             tft180_show_string(0, 32, "i        :",BLACK, WHITE);
//             tft180_show_uint(96, 32, positionalPIDParam.ki,BLUE, WHITE );
//             tft180_show_string(0, 48, "d        :", BLACK, WHITE);
//             tft180_show_uint(96, 48, positionalPIDParam.kd , BLUE, WHITE);
//             tft180_show_string(0, 64, "i_limit  :", BLACK, WHITE);
//             tft180_show_uint(96, 64, positionalPIDParam.i_limit , BLUE, WHITE);
//             break;
//         case IncrementalPID:
//             tft180_show_string(0, 0, " -=  IncrementalPID =- ", RED, WHITE);
//             tft180_show_string(0, 16, "p        :",BLACK, WHITE);
//             tft180_show_uint(96, 16, incrementalPIDParam.kp,BLUE, WHITE );
//             tft180_show_string(0, 32, "i        :",BLACK, WHITE);
//             tft180_show_uint(96, 32, incrementalPIDParam.ki,BLUE, WHITE );
//             tft180_show_string(0, 48, "d        :", BLACK, WHITE);
//             tft180_show_uint(96, 48, incrementalPIDParam.kd , BLUE, WHITE);
//             break;
//         case VariableStructurePID:
//
//             tft180_show_string(0, 0, "-=VStructurePID=-", RED, WHITE);
//             tft180_show_string(0, 16, "error: 0   50   100",BLACK, WHITE);
//             tft180_show_string(0, 32, "p    :",BLACK, WHITE);
//             tft180_show_string(0, 48, "i    :",BLACK, WHITE);
//             error = 0;
//             kp = (float)variableStructurePIDParam.kp_base + (float)variableStructurePIDParam.range_kp * (1 - 1 / (exp((float)variableStructurePIDParam.decayRate_kp/100 * fabs(error))));
//             ki = (float)variableStructurePIDParam.ki_base / exp((float)variableStructurePIDParam.decayRate_ki/100 * fabs(error));
//             tft180_show_uint(48, 32, (int)kp,BLUE, WHITE );
//             tft180_show_uint(48, 48, (int)ki,BLUE, WHITE );
//             error = 50;
//             kp = (float)variableStructurePIDParam.kp_base + (float)variableStructurePIDParam.range_kp * (1 - 1 / (exp((float)variableStructurePIDParam.decayRate_kp/100 * fabs(error))));
//             ki = (float)variableStructurePIDParam.ki_base / exp((float)variableStructurePIDParam.decayRate_ki/100 * fabs(error));
//             tft180_show_uint(80, 32, (int)kp,BLUE, WHITE );
//             tft180_show_uint(80, 48, (int)ki,BLUE, WHITE );
//             error = 100;
//             kp = (float)variableStructurePIDParam.kp_base + (float)variableStructurePIDParam.range_kp * (1 - 1 / (exp((float)variableStructurePIDParam.decayRate_kp/100 * fabs(error))));
//             ki = (float)variableStructurePIDParam.ki_base / exp((float)variableStructurePIDParam.decayRate_ki/100 * fabs(error));
//             tft180_show_uint(112, 32, (int)kp,BLUE, WHITE );
//             tft180_show_uint(112, 48, (int)ki,BLUE, WHITE );
//             break;
//     };
//
//
//     //    tft180_show_string(0, 16, "p max    :",BLACK, WHITE);
//     //    tft180_show_uint(96, 16, motor_kp_right_max,BLUE, WHITE );
//     //    tft180_show_string(0, 32, "p min    :",BLACK, WHITE);
//     //    tft180_show_uint(96, 32, motor_kp_right_min,BLUE, WHITE );
//     //    tft180_show_string(0, 48, "change_ki:", BLACK, WHITE);
//     //    tft180_show_uint(96, 48,motor_change_ki_right , BLUE, WHITE);
//     //    tft180_show_string(0, 64, "d        :", BLACK, WHITE);
//     //    tft180_show_uint(96, 64,motor_kd_right , BLUE, WHITE);
//     //
//     //    tft180_show_string(0,80, "change_kpb:",BLACK, WHITE);
//     //    tft180_show_uint(96, 80, motor_change_kpb,BLUE, WHITE );
//     //    tft180_show_string(0, 96,"change_kib:", BLACK, WHITE);
//     //    tft180_show_uint(96, 96,motor_change_kib , BLUE, WHITE);
// }
//
void showBMXData(void)
{
    tft180_show_string(0, 0, "   -= BMX =-   ");

    tft180_show_string(48, 1 * 16, "angle");
    tft180_show_string(98, 1 * 16, "acc");

    tft180_show_string(0, 2 * 16, "pitch:");
    tft180_show_string(0, 3 * 16, "yaw:");
    tft180_show_string(0, 4 * 16, "roll:");

    //    tft180_show_float(98, 2 * 16, SystemAttitudeRate.Pitch, 2, 2);
    //    tft180_show_float(98, 3 * 16, SystemAttitudeRate.Yaw, 2, 2);
    //    tft180_show_float(98, 4 * 16, SystemAttitudeRate.Roll, 2, 2);
    //
    //    tft180_show_float(48, 2 * 16, angle.Pitch, 2, 2);
    //    tft180_show_float(48, 3 * 16, angle.Yaw, 2, 2);
    //    tft180_show_float(48, 4 * 16, angle.Roll, 2, 2);

    tft180_show_string(0, 5 * 16, " -= tof =- ");
    tft180_show_string(0, 6 * 16, "Distance:");
    tft180_show_int(98, 6 * 16, dl1b_distance_mm, 4);
}
//
// void showTFMiniData(void)
//{
//    tft180_show_string(0,0,"  -= TFMINI =-  ");
////    tft180_show_string(0,1,"Distance:");
////    tft180_show_uint(80,1,TFMINI_PLUS_DISTANCE);
////    tft180_show_string(0,2,"Strength:");
////    tft180_show_uint(80,2,TFMINI_PLUS_STRENGTH);
//}
//
//
//
