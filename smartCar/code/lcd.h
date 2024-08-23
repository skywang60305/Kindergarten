/*
 * lcd.h
 *
 *  Created on: 2020年11月19日
 *      Author: Samurai
 */

#ifndef CODE_LCD_H_
#define CODE_LCD_H_

#include "zf_common_headfile.h"

/*BOOM5的LCD显示状态机已经是最简捷明了的显示方式了，以后就用这个就好*/
/*用法:想在哪里显示,就往对应的函数中放入要执行的显示函数,然后在头文件里对应的写好注释*/
void displayLCD_callBackVersion(void);
void callBackDisplayLCD(void (*event)());
void keyEventOfMyCallBack(void (*event)());

void displayLCD_Map(void);
void displayLCD_Lines(void);
void displayLCD_Debug(void);
void displayLCD_AngleAndSpeed(void);
void displayLCD_STABILITY(void);
void displayLCD_StopInform(void);
void displayLCD_NULL(void);

void keyB_ShowMapEvent(void); // void showbasemap();
void keyL_ShowMapEvent(void); // void showleftmap();
void keyU_ShowMapEvent(void); // void showdeletemap();
void keyR_ShowMapEvent(void); // void showrightmap();
void keyD_ShowMapEvent(void); // void showinsidemap();

void keyB_ShowLinesEvent(void); // void showBothLine();
void keyL_ShowLinesEvent(void); // void showLeftLine();
void keyU_ShowLinesEvent(void); // void showControlLine();
void keyR_ShowLinesEvent(void); // void showRightLine();void showLineInfoRight();
void keyD_ShowLinesEvent(void); // void showImgInfo();

void keyB_ShowDebugEvent(void);
void keyL_ShowDebugEvent(void);
void keyU_ShowDebugEvent(void);
void keyR_ShowDebugEvent(void);
void keyD_ShowDebugEvent(void);

void keyB_ShowAngleAndSpeedEvent(void);
void keyL_ShowAngleAndSpeedEvent(void);
void keyU_ShowAngleAndSpeedEvent(void);
void keyR_ShowAngleAndSpeedEvent(void);
void keyD_ShowAngleAndSpeedEvent(void);

void keyB_ShowStabilityEvent(void);
void keyL_ShowStabilityEvent(void);
void keyR_ShowStabilityEvent(void);
void keyU_ShowStabilityEvent(void);
void keyD_ShowStabilityEvent(void);

/*显示内容不打算大改的话,上面的函数不要动,在下面写自己的显示函数放入上面的框架,25个空基本用不完的*/
// 二值化图
void showbinarymap(const uint8 (*p)[60]);
// 搜图图像
void showallmap(void);
void showbasemap(void);
void showleftmap(void);
void showrightmap(void);
void showdeletemap(void);
void showinsidemap(void);
// 扫线图像及控制线
void showBothLine(void);
void showLeftLine(void);
void showRightLine(void);
void showControlLine(void);
// 图像信息
void showImgInfo(void);
void showIF(uint8 x, uint8 y);
// 车辆参数测量显示
void reverseViewMeasurement(void);
void speedDiffMeasurement(void);
void showDynamicParam(void);
// 显示速度信息
void showSpeedInfo(void);
// 显示角度信息
void showBMXData(void);
void showMotorPIDParam();
// 显示TFMini信息
void showTFMiniData(void);
extern volatile LCD_SHOW_MODE LCD_STATE;
extern uint8 MapClearFlag;
#endif /* CODE_LCD_H_ */
