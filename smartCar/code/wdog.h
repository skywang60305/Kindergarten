/*
 * wdog.h
 *
 *  Created on: 2020Äê5ÔÂ18ÈÕ
 *      Author: Zcc
 */

#ifndef WDOG_H_
#define WDOG_H_

#include "zf_common_headfile.h"
#include "IfxScuWdt.h"

void wdog_init_ms(uint16 ms);
void wdog_feed(void);
void wdog_disable(void);
void wdog_enable(void);
#endif /* CODE_WDOG_H_ */
