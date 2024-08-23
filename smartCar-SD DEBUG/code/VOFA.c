/*
 * author 哆啦A梦
 * date: 2022/10/21
 * description: 上位机传参 推荐使用vofa 因为界面好看，频率可以拉的很高，看曲线什么的比山外方便多了。
 * 山外那个太丑了 而且目前的justfloat是 可变参数 比较方便
 *
 */
#include <stdarg.h>
#include "zf_common_headfile.h"   //这里替换为你的无线串口头文件 我使用的是zf 无线串口
#include "VOFA.h"

/**
 * @description: 数据发送接口封装，请将自己的串口发送字符串函数封装到函数里面
 *               协议打包函数会在数据打包完成后调用这个函数将数据发送出去
 * @param
 *      buf : 需要发送的字符串
 *      len : 字符串长度
 * @return:
 *
 * @demo
 *      uart_send_buf_fcn(send_buf, 8);  //将send_buf这个字符串发送出去，发送长度为8
 */
static inline void uart_send_buf_fcn(uint8_t *buf, uint16_t len) //根据编译器不同 内联函数关键字可能不同,去掉inline效率会略微低一些
{
    for(int i=0;i<len;i++)
    {
        wireless_uart_send_byte(*buf++);    //这里使用的是无线串口  如果是蓝牙则修改成蓝牙的
    }
}

/**
 * @description: 伏特加上位机 JustFloat 模式，可变长度协议发送
 * @param
 *      len : 需要发送的数据个数
 *      ... : 需要发送的内容(必须为浮点数传参)！！！！！！(必须为浮点数传参)如果是其他类型
 *      强转成float即可
 * @return:
 *
 * @demo
 *      vodka_JustFloat_send(5, data1, data2, 3.14f, data4, data5);
 *      vodka_JustFloat_send(1,(float)int类型数据);
 */
float justfloat_send_buf[kWaveNumMax+1] = {0};
void vodka_JustFloat_send(int len,...)
{
    float* pfloat = justfloat_send_buf; //浮点数
    int32_t* pend = NULL; //帧尾
    uint8_t* psend = (uint8_t*)justfloat_send_buf; //发送指针
    if(len > kWaveNumMax) return;
    va_list float_data;
    va_start(float_data, len);
    for (int i = 0; i < len; i++)
    {
        *pfloat++ =  (float)va_arg(float_data, double);
    }
    va_end(float_data);
    pend = (int*)pfloat;
    *pend = (int)(0x7f800000);
    /* 这里调用串口字符串将psend发生出去 */
    uart_send_buf_fcn(psend, (len+1)*sizeof(float));
}


