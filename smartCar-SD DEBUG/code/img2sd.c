/*
 * img2sd.c
 *
 *  Created on: 2022年7月27日
 *      Author: Samurai
 */
#include "img2sd.h"

uint8 isOK;
uint16 sector;

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
    while(ptr != ptrEnd)
    {
        if(*ptrImg & 0x01 << i++)
        {
            *ptr++ = 0;
        }
        else
        {
            *ptr++ = 255;
        }
        if(i == 8)
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

    if(status == 0)
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


////    存灰度图就这么存就可以
//    sd_writeDisk(imgGray[0], sector, 23);
//    sector += 23;

    //自己想存什么，就在这里sd_writeDisk就好了
    uint8 sdBuff[4] = {0};
    uint16 sdNum = 0;
//    sdBuff[sdNum++] = 'b';                                              //->0
//    sdBuff[sdNum++] = 'o';                                              //->1
//    sdBuff[sdNum++] = 'o';                                              //->2
//    sdBuff[sdNum++] = 'm';
//    sdBuff[sdNum++] = '7';
//    sd_writeDisk(sdBuff, sector++, 1);
    sd_writeDisk(imgGray[0], sector, 23);
        sector += 23;
    return 1;
}
