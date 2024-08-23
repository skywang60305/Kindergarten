/*
 * spi_sd.h
 *
 *  Created on: 2021Äê8ÔÂ4ÈÕ
 *      Author: 95159
 */

#ifndef CODE_SPI_SD_H_
#define CODE_SPI_SD_H_

#include "zf_common_headfile.h"

uint8 sd_initialize(void);
uint32 sd_getSectorCount(void);
uint8 sd_writeDisk(uint8 *buf, uint32 sector, uint8 cnt);
uint8 sd_readDisk(uint8 *buf, uint32 sector, uint8 cnt);



#endif /* CODE_SPI_SD_H_ */


