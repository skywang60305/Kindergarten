/*
 * @Author: Justin
 * @Date: 2023-02-05 15:06:50
 * @LastEditors: Justin
 * @LastEditTime: 2023-02-08 12:57:10
 * @FilePath: \PopCorn-soft\project\code\VOFA.h
 * @Description:
 */
#if 1
#ifndef VOFA_H_
#define VOFA_H_
#include "zf_common_headfile.h"

// uart_index_enum Chal;

#define my_send_char(chr) uart_putchar(Chal, chr)

void VOFAInit();

void justf();

int Print(const char *str, ...);

// int myfunc_str_to_int(char *str,int n);

// void uart_rx_interrupt_handler (void);
// void GetRecData();

#endif
#endif
