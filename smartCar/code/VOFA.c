/*
 * @Author: Justin
 * @Date: 2023-01-30 13:08:19
 * @LastEditors: Justin
 * @LastEditTime: 2023-01-30 17:28:19
 * @FilePath: \PopCorn-soft\project\code\VOFA.c
 * @Description:  ��Ҫ��vofa��firewater��ͨ��Э��
 */
#if 1
#include "VOFA.h"

unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};

uart_index_enum Chal;
uint8 uart_get_data[64]; // ���ڽ������ݻ�����
uint8 fifo_get_data[64]; // fifo �������������

uint8 get_data = 0;         // �������ݱ���
uint32 fifo_data_count = 0; // fifo ���ݸ���

fifo_struct uart_data_fifo;

void VOFAInit()
{

    //    bluetooth_ch9141_init();
    Chal = UART_3;
}
vuint32 Recdata = 0;
uint8 dat = 0;
// void uart_rx_interrupt_handler (void)
//{
//     uart_query_byte(UART_0, &dat);
//     fifo_write_buffer(&uart_data_fifo, &dat, 1);
// }
// void GetRecData(){
//     fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
//     if(fifo_data_count != 0)                                                // ��ȡ��������
//     {
//         fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//         Recdata = myfunc_str_to_int(&fifo_get_data,fifo_data_count);
//         Print("%d\n",Recdata);
//     }
// }
//  ����m^n
unsigned long m_pow_n(unsigned long m, unsigned long n)
{
    unsigned long i = 0, ret = 1;
    if (n < 0)
        return 0;
    for (i = 0; i < n; i++)
    {
        ret *= m;
    }
    return ret;
}

// ����ֵΪ��ӡ�ַ��ĸ���
// ֧��%d��%o, %x��%s��%c��%f��ֻ��ӡ6λ���֣�
void justf()
{
    for (int i = 0; i < 4; i++)
    {
        my_send_char(tail[i]);
    }
}

int Print(const char *str, ...)
{
    if (str == NULL)
        return -1;

    unsigned int ret_num = 0;    // ���ش�ӡ�ַ��ĸ���
    char *pStr = (char *)str;    // ָ��str
    int ArgIntVal = 0;           // ��������
    unsigned long ArgHexVal = 0; // ��ʮ������
    char *ArgStrVal = NULL;      // �����ַ���
    double ArgFloVal = 0.0;      // ���ܸ�����
    unsigned long val_seg = 0;   // �����з�
    unsigned long val_temp = 0;  // ��ʱ��������
    int cnt = 0;                 // ���ݳ��ȼ���
                                 //    int i = 0;

    va_list pArgs;        // ����va_list����ָ�룬���ڴ洢�����ĵ�ַ
    va_start(pArgs, str); // ��ʼ��pArgs
    while (*pStr != '\0')
    {
        switch (*pStr)
        {
        case ' ':
            my_send_char(*pStr);
            ret_num++;
            break;
        case '\t':
            my_send_char(*pStr);
            ret_num += 4;
            break;
        case '\r':
            my_send_char(*pStr);
            ret_num++;
            break;
        case '\n':
            my_send_char(*pStr);
            ret_num++;
            break;
        case '%':
            pStr++;
            // % ��ʽ����
            switch (*pStr)
            {
            case '%':
                my_send_char('%'); // %%�����%
                ret_num++;
                pStr++;
                continue;
            case 'c':
                ArgIntVal = va_arg(pArgs, int); // %c�����char
                my_send_char((char)ArgIntVal);
                ret_num++;
                pStr++;
                continue;
            case 'd':
                // ��������
                ArgIntVal = va_arg(pArgs, int);
                if (ArgIntVal < 0) // ���Ϊ������ӡ������
                {
                    ArgIntVal = -ArgIntVal; // ȡ�෴��

                    my_send_char('-');
                    ret_num++;
                }
                val_seg = ArgIntVal; // ��ֵ�� val_seg��������
                // ����ArgIntVal����
                if (ArgIntVal)
                {
                    while (val_seg)
                    {
                        cnt++;
                        val_seg /= 10;
                    }
                }
                else
                    cnt = 1; // ����0�ĳ���Ϊ1

                ret_num += cnt; // �ַ��������������ĳ���

                // ������תΪ�����ַ���ӡ
                while (cnt)
                {
                    val_seg = ArgIntVal / m_pow_n(10, cnt - 1);
                    ArgIntVal %= m_pow_n(10, cnt - 1);
                    my_send_char((char)val_seg + '0');
                    cnt--;
                }
                pStr++;
                continue;
            case 'o':
                // ��������
                ArgIntVal = va_arg(pArgs, int);
                if (ArgIntVal < 0) // ���Ϊ������ӡ������
                {
                    ArgIntVal = -ArgIntVal; // ȡ�෴��

                    my_send_char('-');
                    ret_num++;
                }
                val_seg = ArgIntVal; // ��ֵ�� val_seg��������
                // ����ArgIntVal����
                if (ArgIntVal)
                {
                    while (val_seg)
                    {
                        cnt++;
                        val_seg /= 8;
                    }
                }
                else
                    cnt = 1; // ����0�ĳ���Ϊ1

                ret_num += cnt; // �ַ��������������ĳ���

                // ������תΪ�����ַ���ӡ
                while (cnt)
                {
                    val_seg = ArgIntVal / m_pow_n(8, cnt - 1);
                    ArgIntVal %= m_pow_n(8, cnt - 1);
                    my_send_char((char)val_seg + '0');
                    cnt--;
                }
                pStr++;
                continue;
            case 'x':
                // ����16����
                ArgHexVal = va_arg(pArgs, unsigned long);
                val_seg = ArgHexVal;
                // ����ArgIntVal����
                if (ArgHexVal)
                {
                    while (val_seg)
                    {
                        cnt++;
                        val_seg /= 16;
                    }
                }
                else
                    cnt = 1; // ����0�ĳ���Ϊ1

                ret_num += cnt; // �ַ��������������ĳ���
                // ������תΪ�����ַ���ӡ
                while (cnt)
                {
                    val_seg = ArgHexVal / m_pow_n(16, cnt - 1);
                    ArgHexVal %= m_pow_n(16, cnt - 1);
                    if (val_seg <= 9)
                    {
                        my_send_char((char)val_seg + '0');
                    }
                    else
                    {
                        // my_send_char((char)val_seg - 10 + 'a'); //Сд��ĸ
                        my_send_char((char)val_seg - 10 + 'A');
                    }
                    cnt--;
                }
                pStr++;
                continue;
            case 'b':
                // ��������
                ArgIntVal = va_arg(pArgs, int);
                val_seg = ArgIntVal;
                // ����ArgIntVal����
                if (ArgIntVal)
                {
                    while (val_seg)
                    {
                        cnt++;
                        val_seg /= 2;
                    }
                }
                else
                    cnt = 1; // ����0�ĳ���Ϊ1

                ret_num += cnt; // �ַ��������������ĳ���
                // ������תΪ�����ַ���ӡ
                while (cnt)
                {
                    val_seg = ArgIntVal / m_pow_n(2, cnt - 1);
                    ArgIntVal %= m_pow_n(2, cnt - 1);
                    my_send_char((char)val_seg + '0');
                    cnt--;
                }
                pStr++;
                continue;
            case 's':
                // �����ַ�
                ArgStrVal = va_arg(pArgs, char *);
                ret_num += (unsigned int)strlen(ArgStrVal);
                while (*ArgStrVal)
                {
                    my_send_char(*ArgStrVal);
                    ArgStrVal++;
                }

                pStr++;
                continue;

            case 'f':
                // ���ո����� ����6ΪС��������ȡ��������
                ArgFloVal = va_arg(pArgs, double);
                if (ArgFloVal < 0)
                {
                    my_send_char('-');
                    ArgFloVal = -ArgFloVal;
                }
                val_seg = (unsigned long)ArgFloVal; // ȡ��������
                val_temp = val_seg;                 // ��ʱ����������������
                ArgFloVal = ArgFloVal - val_seg;    // �ó����µ�С������
                // �����������ֳ���
                if (val_seg)
                {
                    while (val_seg)
                    {
                        cnt++;
                        val_seg /= 10;
                    }
                }
                else
                    cnt = 1;    // ����0�ĳ���Ϊ1
                ret_num += cnt; // �ַ��������������ĳ���
                // ������תΪ�����ַ���ӡ
                while (cnt)
                {
                    val_seg = val_temp / m_pow_n(10, cnt - 1);
                    val_temp %= m_pow_n(10, cnt - 1);
                    my_send_char((char)val_seg + '0');
                    cnt--;
                }
                // ��ӡС����
                my_send_char('.');
                ret_num++;
                // ��ʼ���С������
                ArgFloVal *= 1000000;
                // printf("\r\n %f\r\n", ArgFloVal);
                cnt = 6;
                val_temp = (int)ArgFloVal; // ȡ��������
                while (cnt)
                {
                    val_seg = val_temp / m_pow_n(10, cnt - 1);
                    val_temp %= m_pow_n(10, cnt - 1);
                    my_send_char((char)val_seg + '0');
                    cnt--;
                }
                ret_num += 6;
                pStr++;
                continue;
            default: // % ƥ�����������ո�
                my_send_char(' ');
                ret_num++;
                continue;
            }
            break;

        default:
            my_send_char(*pStr);
            ret_num++;
            break;
        }
        pStr++;
    }
    va_end(pArgs); // ����ȡ����

    return ret_num;
}

// int myfunc_str_to_int(char *str,int n){
//     zf_assert(str != NULL);
//     uint8 sign = 0;                                                             // ��Ƿ��� 0-���� 1-����
//     int32 temp = 0;   // ��ʱ�������
////    int i = 0;
//    if(n == 0)
//    {
//        return 0;
//    }
//    if('-' == str[0])                                                         // �����һ���ַ��Ǹ���
//    {
//        sign = 1;                                                           // ��Ǹ���
//    }
//
//    for (int j = 1;j < n;j ++){
//        temp = temp * 10 + ((uint8)(str[j]) - 0x30);                          // ������ֵ
//    }
//    if(sign)
//    {
//        temp = -temp;
//    }
//    return temp;
//}

#endif
