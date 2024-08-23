/*
 * deal_img.c
 *
 *  Created on: 2022年7月27日
 *      Author: Samurai
 */

#include "deal_img.h"
//灰度图,二值化图
uint8 imgGray[MT9V03X_H][MT9V03X_W];
uint8 imgBinary[MT9V03X_H][MT9V03X_W];
//压缩图
uint8 allmap[YM][XM];
//大津法阈值
uint8 OSTU_THRESHOLD;
//大津法阈值限幅
uint8 OSTU_MIN = 0;
uint8 OSTU_MAX = 255;
//左右扫线保存
uint32 L_line[MT9V03X_H] = {0};
uint32 R_line[MT9V03X_H] = {0};
//标志位
FLAG flag;

void binaryAlgorithm()
{
    OSTU_THRESHOLD = getOstuThreshold(mt9v03x_image[0],CAMERA_W,0,YM,(uint8)OSTU_MAX,(uint8)OSTU_MIN);
    imgBinarize(imgGray[0], imgBinary[0],OSTU_THRESHOLD);
    horizonCompress(allmap[0], imgBinary[0]);
    uint8 tmp;
    for(uint8 j = 0;j < YM/2;j++)
    {
        for(uint8 i=0;i<XM;i++)
        {
            tmp = allmap[j][i];
            allmap[j][i] = allmap[YY-j][i];
            allmap[YY-j][i] = tmp;
        }
    }
}

uint8 getOstuThreshold(uint8 *image, uint32 col, uint32 startRows,uint32 endRows,uint8 GrayScale_Max,uint8 GrayScale_Min)
{
#define GrayScale 256
    uint32 width = col;
    uint32 pixelCount[GrayScale];
    float pixelPro[GrayScale];
    uint32 i, j, pixelSum = width * (endRows-startRows);
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数   //即获得直方图隔行获得
    for (i = startRows; i < endRows; i+=1)
        for (j = 0; j < width; j+=1)
        {
            pixelCount[(uint32)data[i * width + j]]++;   //将当前的点的像素值作为计数数组的下标
            gray_sum+=(uint32)data[i * width + j];       //灰度值总和
        }

    //计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = u0tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = GrayScale_Min; j < GrayScale_Max; j++)
    {
        w0 += pixelPro[j];         //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * (u0 - u) * (u0-u) + w1 * (u1-u)*(u1-u);    //块一丢丢
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)j;
        }
        if (deltaTmp < deltaMax)
            break;
    }
    if (threshold<OSTU_MIN) threshold=OSTU_MIN;
    if (threshold>OSTU_MAX) threshold=OSTU_MAX;
    return threshold;
}

#define MAX_COLS CAMERA_W    /*图像水平分辨率*/
#define MAX_ROWS CAMERA_H    /*图像竖直分辨率*/
void imgBinarize(unsigned char* org_in, unsigned char* ostu_out,unsigned char th_ostu)
{
    unsigned int i, col;
    unsigned int start_pixel, end_pixel;
    col = 0;
    start_pixel = 0;
    end_pixel =MAX_ROWS * MAX_COLS;

    unsigned int dealMax=end_pixel;
    for (i = start_pixel; i < dealMax; i++)
    {
        col ++;
        if(org_in[i] >= th_ostu) /*等号是否需要根据小车情况进行处理，区别不大*/
            ostu_out[i-start_pixel] = PIXEL_WHITE;    /*置白*/
        else
            ostu_out[i-start_pixel] = PIXEL_BLACK;      /*置黑*/
    }
}
#undef MAX_COLS
#undef MAX_ROWS

void horizonCompress(uint8 tempimgbuff[], uint8 img[])
{
    int dealMax = CAMERA_H*CAMERA_W/HORIZON_TIMES;
    for(int i = 0;i < dealMax;i++)
    {
        uint8 blackNumCnt=0;
        for(int index=0;index<HORIZON_TIMES;index++)
            if( (*(img+i*HORIZON_TIMES+index))<255 )
                blackNumCnt++;
        if(blackNumCnt >=2) tempimgbuff[i]=PIXEL_BLACK;         //2个黑点置黑色
        else tempimgbuff[i]=PIXEL_WHITE;
    }
}

void searchline(Search_way way)
{
   uint8 mid = MT9V03X_W/2;
   if(way == normal)
   {
       for(int i = MT9V03X_H;i > 0;i--)
       {
           for(int j = mid;j > 0;j--)
           {
               if(imgBinary[i][j]==PIXEL_BLACK && imgBinary[i][j - 1]==PIXEL_BLACK)
               {
                   L_line[i] = j;
                   flag.Left = 1;
                   break;
               }
               if(!flag.Left)
               {
                   L_line[i] = 0;
               }
           }
           for(int j = mid;j < MT9V03X_W - 1;j++)
           {
               if(imgBinary[i][j]==PIXEL_BLACK && imgBinary[i][j + 1]==PIXEL_BLACK)
               {
                   R_line[i] = j;
                   flag.Right = 1;
                   break;
               }
               if(!flag.Right)
               {
                   R_line[i] = MT9V03X_W - 1;
               }
           }
           flag.Left = 0;
           flag.Right = 0;
           imgBinary[i][(R_line[i]+L_line[i])/2] = PIXEL_BLACK;
       }
   }
   else if(way == Eightneighboor)
   {




   }




}

void Get_Image(void)
{
    if(mt9v03x_finish_flag)
    {
        for(int i = MT9V03X_H - 1;i >= 0;i--)
        {
            for(int j = 0;j < MT9V03X_W;j++)
            {
                imgGray[i][j]=mt9v03x_image[i][j];
            }
        }
        mt9v03x_finish_flag = 0;
        tft180_show_gray_image(0, 0, imgGray, MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 0);
        sd_write();
    }
    binaryAlgorithm();
}



