/*
 * VCAN_VisualScope.h
 *
 *  Created on: 2021��9��6��
 *      Author: 1
 */

#ifndef CODE_VCAN_VISUALSCOPE_H_
#define CODE_VCAN_VISUALSCOPE_H_
#include "zf_common_headfile.h"

#define UART_PIN UART_3
// ʹ��ǰ��Ҫ��ʼ��uart

void send_uint8_data(uint8 data1, uint8 data2, uint8 data3, uint8 data4);

void send_uint16_data(uint16 data1, uint16 data2, uint16 data3, uint16 data4);

void send_int16_data(int16 data1, int16 data2, int16 data3, int16 data4);
#endif /* CODE_VCAN_VISUALSCOPE_H_ */
