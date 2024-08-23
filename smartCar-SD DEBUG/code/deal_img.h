/*
 * deal_img.h
 *
 *  Created on: 2022年7月27日
 *      Author: Samurai
 */

#ifndef CODE_DEAL_IMG_H_
#define CODE_DEAL_IMG_H_

#include "zf_common_headfile.h"

#define CAMERA_H MT9V03X_H
#define CAMERA_W MT9V03X_W

#define XM 47
#define YM 60
#define XX 46
#define YY 59

//横向压缩比例
#define HORIZON_TIMES 4
//黑和白的像素值，避免大伙分不清这里宏定义一下
#define PIXEL_WHITE 255
#define PIXEL_BLACK 0

typedef enum{
    Eightneighboor,
    normal,
}Search_way;

typedef struct{
    uint8 Left;
    uint8 Right;
}FLAG;



void binaryAlgorithm();
uint8 getOstuThreshold(uint8 *image, uint32 col, uint32 startRows,uint32 endRows,uint8 GrayScale_Max,uint8 GrayScale_Min);
void imgBinarize(unsigned char* org_in, unsigned char* ostu_out,unsigned char th_ostu);
void horizonCompress(uint8 tempimgbuff[], uint8 img[]);
void searchline(Search_way way);
void Get_Image(void);

extern uint8 imgGray[MT9V03X_H][MT9V03X_W];
extern uint8 imgBinary[MT9V03X_H][MT9V03X_W];
extern uint8 allmap[YM][XM];
extern uint32 L_line[MT9V03X_H];
extern uint32 R_line[MT9V03X_H];



#endif /* CODE_DEAL_IMG_H_ */
