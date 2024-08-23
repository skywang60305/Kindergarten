/*
 * lcd.h
 *
 *  Created on: 2020��11��19��
 *      Author: Samurai
 */

#ifndef CODE_LCD_H_
#define CODE_LCD_H_

#include "zf_common_headfile.h"

/*BOOM5��LCD��ʾ״̬���Ѿ����������˵���ʾ��ʽ�ˣ��Ժ��������ͺ�*/
/*�÷�:����������ʾ,������Ӧ�ĺ����з���Ҫִ�е���ʾ����,Ȼ����ͷ�ļ����Ӧ��д��ע��*/
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

/*��ʾ���ݲ������ĵĻ�,����ĺ�����Ҫ��,������д�Լ�����ʾ������������Ŀ��,25���ջ����ò����*/
// ��ֵ��ͼ
void showbinarymap(const uint8 (*p)[60]);
// ��ͼͼ��
void showallmap(void);
void showbasemap(void);
void showleftmap(void);
void showrightmap(void);
void showdeletemap(void);
void showinsidemap(void);
// ɨ��ͼ�񼰿�����
void showBothLine(void);
void showLeftLine(void);
void showRightLine(void);
void showControlLine(void);
// ͼ����Ϣ
void showImgInfo(void);
void showIF(uint8 x, uint8 y);
// ��������������ʾ
void reverseViewMeasurement(void);
void speedDiffMeasurement(void);
void showDynamicParam(void);
// ��ʾ�ٶ���Ϣ
void showSpeedInfo(void);
// ��ʾ�Ƕ���Ϣ
void showBMXData(void);
void showMotorPIDParam();
// ��ʾTFMini��Ϣ
void showTFMiniData(void);
extern volatile LCD_SHOW_MODE LCD_STATE;
extern uint8 MapClearFlag;
#endif /* CODE_LCD_H_ */
