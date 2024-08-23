/*
 * img2sd.h
 *
 *  Created on: 2021Äê8ÔÂ4ÈÕ
 *      Author: 95159
 */

#ifndef CODE_IMG2SD_H_
#define CODE_IMG2SD_H_

#include "zf_common_headfile.h"


#define SECTOR_START 4096
#define SECTOR_END 300000
#define SECTOR_SIZE 512

typedef enum SDMode
{
    Record,
    Review,
    SDOFF,
} SDMode;

typedef enum RecordImgMode
{
    allmapMode,
    imgGrayMode,
} RecordImgMode;

typedef enum ReviewOperateMode
{
    AutoMode,
    ManualMode,
} ReviewOperateMode;

typedef enum OperateModeShift
{
    ALLOW,
    FORBID,
} OperateModeShift;

typedef struct sdCardParam
{
    SDMode sdMode;
    RecordImgMode recordImgMode;
    ReviewOperateMode reviewOperateMode;
    uint16 currentSector;
    uint8 getSectorData_start_flag;
    uint8 isEnd;
    OperateModeShift operateShift;
    uint16 reviewSpeed;
} sdCardParam;

extern sdCardParam sdcardParam;

uint8 sd_init(void);
uint8 sd_write(void);
uint8 sd_read(uint16 sector);
uint8 SDread( uint16 sector);
void sdreverse(void);
void AutoAdjustSector(void);
void ManualAdjustSector(void);
void sd_stop(void);
void showpaper1(void);
void showpaper2(void);
void sd_clear();
extern uint16 sector;
extern uint16 sdInfo;
#endif /* CODE_IMG2SD_H_ */
