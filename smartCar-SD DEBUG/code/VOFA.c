/*
 * author ����A��
 * date: 2022/10/21
 * description: ��λ������ �Ƽ�ʹ��vofa ��Ϊ����ÿ���Ƶ�ʿ������ĺܸߣ�������ʲô�ı�ɽ�ⷽ����ˡ�
 * ɽ���Ǹ�̫���� ����Ŀǰ��justfloat�� �ɱ���� �ȽϷ���
 *
 */
#include <stdarg.h>
#include "zf_common_headfile.h"   //�����滻Ϊ������ߴ���ͷ�ļ� ��ʹ�õ���zf ���ߴ���
#include "VOFA.h"

/**
 * @description: ���ݷ��ͽӿڷ�װ���뽫�Լ��Ĵ��ڷ����ַ���������װ����������
 *               Э���������������ݴ����ɺ����������������ݷ��ͳ�ȥ
 * @param
 *      buf : ��Ҫ���͵��ַ���
 *      len : �ַ�������
 * @return:
 *
 * @demo
 *      uart_send_buf_fcn(send_buf, 8);  //��send_buf����ַ������ͳ�ȥ�����ͳ���Ϊ8
 */
static inline void uart_send_buf_fcn(uint8_t *buf, uint16_t len) //���ݱ�������ͬ ���������ؼ��ֿ��ܲ�ͬ,ȥ��inlineЧ�ʻ���΢��һЩ
{
    for(int i=0;i<len;i++)
    {
        wireless_uart_send_byte(*buf++);    //����ʹ�õ������ߴ���  ������������޸ĳ�������
    }
}

/**
 * @description: ���ؼ���λ�� JustFloat ģʽ���ɱ䳤��Э�鷢��
 * @param
 *      len : ��Ҫ���͵����ݸ���
 *      ... : ��Ҫ���͵�����(����Ϊ����������)������������(����Ϊ����������)�������������
 *      ǿת��float����
 * @return:
 *
 * @demo
 *      vodka_JustFloat_send(5, data1, data2, 3.14f, data4, data5);
 *      vodka_JustFloat_send(1,(float)int��������);
 */
float justfloat_send_buf[kWaveNumMax+1] = {0};
void vodka_JustFloat_send(int len,...)
{
    float* pfloat = justfloat_send_buf; //������
    int32_t* pend = NULL; //֡β
    uint8_t* psend = (uint8_t*)justfloat_send_buf; //����ָ��
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
    /* ������ô����ַ�����psend������ȥ */
    uart_send_buf_fcn(psend, (len+1)*sizeof(float));
}


