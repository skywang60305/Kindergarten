/*
 * img2sd.h
 *
 *  Created on: 2022Äê7ÔÂ27ÈÕ
 *      Author: Samurai
 */

#ifndef CODE_IMG2SD_H_
#define CODE_IMG2SD_H_

#include "zf_common_headfile.h"

#define SECTOR_START 4096
#define SECTOR_END 300000
#define SECTOR_SIZE 512


uint8 sd_init(void);
uint8 sd_write(void);
void allmap2sd(uint8 *sdBuff);
void sd2allmap(uint8 *sdBuff);

#endif /* CODE_IMG2SD_H_ */
