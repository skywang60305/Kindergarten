/*
 * common.h
 *
 *  Created on: 2023年10月19日
 *      Author: 86135
 */

#ifndef CODE_COMMON_H_
#define CODE_COMMON_H_

#define PAGE_DISP_NUM 7
// typedef struct Point
//{
//     int8 x;
//     int8 y;
// }Point;         //暂时写在这个文件中
//
// typedef enum ControlLine
//{
//     BOTHLINE,
//     LEFTLINE,
//     RIGHTLINE,
//     MIDLINE,
//     NONELINE,
//     CONTROLLINE
// } ControlLine;  //暂时写在这个文件中
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

// 停车原因
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
    LCD_SHOW_ANGLE = 3,       // 角度和速度信息
    LCD_SHOW_STABILITY = 4,   // SD和程序的稳定度
    LCD_SHOW_NONE = 5,        // 清屏
    LCD_SHOW_STOP_INFORM = 6, // 停车显示信息

} LCD_SHOW_MODE;

typedef enum
{
    LCD_SHOWIMG = 0,
    LCD_SHOWINFO = 1,
} LCD_IMG_MODE;

typedef enum
{
    KEY_U, // 上
    KEY_D, // 下

    KEY_L, // 左
    KEY_R, // 右

    KEY_B, // 选择

    KEY_RUN,  // 开始
    KEY_STOP, // 停止

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
