/*
 * common.h
 *
 *  Created on: 2023��10��19��
 *      Author: 86135
 */

#ifndef CODE_COMMON_H_
#define CODE_COMMON_H_

#define PAGE_DISP_NUM 7
// typedef struct Point
//{
//     int8 x;
//     int8 y;
// }Point;         //��ʱд������ļ���
//
// typedef enum ControlLine
//{
//     BOTHLINE,
//     LEFTLINE,
//     RIGHTLINE,
//     MIDLINE,
//     NONELINE,
//     CONTROLLINE
// } ControlLine;  //��ʱд������ļ���
//
//
//------------USER
typedef struct Point
{
    int16 x;
    int16 y;
} Point;

typedef struct PointF
{
    float x;
    float y;
} PointF;

// typedef struct Stack
// {
//     int32 top;
//     int32 MAX;
//     Point *data;
// }Stack;

// ͣ��ԭ��
typedef enum StopReason
{
    HaventStarted,
    TimeUp,
    RunOutLine,
    EnterGarage,
    StallProtect,
} StopReason;

typedef enum
{
    LCD_SHOW_MAP = 0,
    LCD_SHOW_LINES = 1,
    LCD_SHOW_DEBUG = 2,
    LCD_SHOW_ANGLE = 3,       // �ǶȺ��ٶ���Ϣ
    LCD_SHOW_STABILITY = 4,   // SD�ͳ�����ȶ���
    LCD_SHOW_NONE = 5,        // ����
    LCD_SHOW_STOP_INFORM = 6, // ͣ����ʾ��Ϣ

} LCD_SHOW_MODE;

typedef enum
{
    LCD_SHOWIMG = 0,
    LCD_SHOWINFO = 1,
} LCD_IMG_MODE;

typedef enum
{
    KEY_U, // ��
    KEY_D, // ��

    KEY_L, // ��
    KEY_R, // ��

    KEY_B, // ѡ��

    KEY_RUN,  // ��ʼ
    KEY_STOP, // ֹͣ

    KEY_MAX,
} KEY_e;

typedef enum ControlLine
{
    BOTHLINE,
    LEFTLINE,
    RIGHTLINE,
    MIDLINE,
    NONELINE,
    CONTROLLINE
} ControlLine;

#endif /* CODE_COMMON_H_ */
