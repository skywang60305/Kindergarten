/*
 * spi_sd.c
 *
 *  Created on: 2021年8月4日
 *      Author: 95159
 */

#include "spi_sd.h"

#define SD_SPIN SPI_1
#define SD_SCL SPI1_SCLK_P11_6
#define SD_SDA SPI1_MOSI_P11_9
#define SD_SDA_IN SPI1_MISO_P11_3
#define SD_CS SPI1_CS3_P11_10
#define SD_CS_PIN P11_10


#define SD_CS_DISABLE gpio_set_level(SD_CS_PIN, 1);
#define SD_CS_ENABLE gpio_set_level(SD_CS_PIN, 0);
// SD卡类型定义
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06
// SD卡指令表
#define CMD0    0       //卡复位
#define CMD1    1
#define CMD8    8       //命令8 ，SEND_IF_COND
#define CMD9    9       //命令9 ，读CSD数据
#define CMD10   10      //命令10，读CID数据
#define CMD12   12      //命令12，停止数据传输
#define CMD16   16      //命令16，设置SectorSize 应返回0x00
#define CMD17   17      //命令17，读sector
#define CMD18   18      //命令18，读Multi sector
#define CMD23   23      //命令23，设置多sector写入前预先擦除N个block
#define CMD24   24      //命令24，写sector
#define CMD25   25      //命令25，写Multi sector
#define CMD41   41      //命令41，应返回0x00
#define CMD55   55      //命令55，应返回0x01
#define CMD58   58      //命令58，读OCR信息
#define CMD59   59      //命令59，使能/禁止CRC，应返回0x00
//数据写入回应字意义
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD卡回应标记字
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF

uint8 sd_type = 0;//sd卡类型

uint8 sd_spi_readWriteByte(uint8 data)
{
    uint8 modata = data;
    uint8 midata[1];
    spi_transfer_8bit(SD_SPIN, &modata, midata, 1);
    return midata[0];
}

uint8 sd_waitReady(void)
{
    uint32 t = 0;
    do
    {
        if(sd_spi_readWriteByte(0xFF) == 0xFF)
        {
            return 0;//OK
        }
    }while(t < 0xFFFFFF);//等待
    return 1;
}

void sd_disSelect(void)//取消选择，释放SPI总线
{
    SD_CS_DISABLE;
    sd_spi_readWriteByte(0xFF);
}

uint8 sd_select(void)
{
    SD_CS_ENABLE;
    if(sd_waitReady() == 0)
    {
        return 0;//等待成功
    }
    sd_disSelect();
    return 1;//等待失败
}

uint8 sd_sendCmd(uint8 cmd, uint32 arg, uint8 crc)
{
    uint8 rl;
    uint8 retry = 0;
    uint8 send_buf[8];
    sd_disSelect();//取消上次片选;
    sd_spi_readWriteByte(0xFF);
    if(sd_select())
    {
        return 0xFF;//片选失败
    }
    send_buf[0] = 0xff;
    send_buf[1] = ((uint8)(cmd | 0x40));//分别写入命令
    send_buf[2] = ((uint8)(arg >> 24));
    send_buf[3] = ((uint8)(arg >> 16));
    send_buf[4] = ((uint8)(arg >> 8));
    send_buf[5] = ((uint8)arg);
    send_buf[6] = (crc);
    send_buf[7] = 0xff;
//    //发送
//    sd_spi_readWritspi_write_8bit_arrayeByte((uint8)(cmd | 0x40));//分别写入命令
//    sd_spi_readWriteByte((uint8)(arg >> 24));
//    sd_spi_readWriteByte((uint8)(arg >> 16));
//    sd_spi_readWriteByte((uint8)(arg >> 8));
//    sd_spi_readWriteByte((uint8)arg);
//    sd_spi_readWriteByte(crc);
    spi_write_8bit_array(SD_SPIN,send_buf,8);
    if(cmd == CMD12)
    {
        sd_spi_readWriteByte(0xFF);//skip a stuff byte when stop reading
    }
    //一次返回10个数据 0~8位 0xff 第九位 0x01
    retry = 20;
    do
    {
        //20次 全部返回0xff
        rl = sd_spi_readWriteByte(0xFF);
        if((rl & 0x80) == 0 )
        {
            break;
        }
        if(rl != 0xff)
        {
            break;
        }

    } while(retry--);
    SD_CS_DISABLE;
    sd_spi_readWriteByte(0xFF);
    return rl;
}

uint8 sd_sendBlock(uint8 *buf, uint8 cmd)
{
    uint16 t;
    if(sd_waitReady())
    {
        return 1;//等待准备失败
    }
    sd_spi_readWriteByte(cmd);
    if(cmd != 0xFD)//不是结束指令
    {
        for(t = 0; t < 512; t++)
        {
            sd_spi_readWriteByte(buf[t]);
        }
        sd_spi_readWriteByte(0xFF);//忽略crc
        sd_spi_readWriteByte(0xFF);
        t = sd_spi_readWriteByte(0xFF);//接收响应
        if((t & 0x1F) != 0x05)
        {
            return 2;//响应错误
        }
    }
    return 0;//写入成功
}

//初始化时需要低速
void sd_spi_speedLow(void)
{
    spi_init(SD_SPIN, SPI_MODE3, 400 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

//正常工作的时候可以高速了
void sd_spi_speedHigh(void)
{
    spi_init(SD_SPIN, SPI_MODE3, 36 * 1000 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

void sd_spi_init(void)
{
    //硬件SPI初始化，初始化低速模式，正点原子为280k，模式应该是3
    spi_init(SD_SPIN, SPI_MODE3, 400 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
//    spi_init(SPI_2, SPI_MODE0, 1*1000*1000, SPI2_SCLK_P15_3, SPI2_MOSI_P15_5, SPI2_MISO_P15_4, SPI2_CS0_P15_2); // 硬件SPI初始化  模式0 波特率为1Mhz
    gpio_init(SD_CS_PIN, GPO, 1, GPO_PUSH_PULL);
//    gpio_init(SD_CS_PIN, GPO, 0, GPO_PUSH_PULL);
    SD_CS_DISABLE;
}

uint8 sd_getResponse(uint8 response)
{
    uint16 count = 0xFFFF;
    while((sd_spi_readWriteByte(0xFF) != response) && count)
    {
        count--;
    }
    if(count == 0)
    {
        return MSD_RESPONSE_FAILURE;//fail
    }
    else
    {
        return MSD_RESPONSE_NO_ERROR;
    }
}

uint8 sd_recvData(uint8 *buf, uint16 len)
{
    if(sd_getResponse(0xFE))
    {
        return 1;
    }
    while(len--)
    {
        *buf = sd_spi_readWriteByte(0xFF);
        buf++;
    }
    sd_spi_readWriteByte(0xFF);
    sd_spi_readWriteByte(0xFF);
    return 0;//success
}

//获取SD卡的CSD信息，包括容量和速度信息
//输入:u8 *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//       1：错误
uint8 sd_getCSD(uint8 *csd_data)
{
    uint8 rl;
    rl = sd_sendCmd(CMD9, 0, 0x01);
    if(rl == 0)
    {
        rl = sd_recvData(csd_data, 16);
    }
    sd_disSelect();
    if(rl)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//获取SD卡的总扇区数（扇区数）
//返回值:0： 取容量出错
//       其他:SD卡的容量(扇区数/512字节)
//每扇区的字节数必为512，因为如果不是512，则初始化不能通过.
uint32 sd_getSectorCount(void)
{
    uint8 csd[16];
    uint32 capacity;
    uint8 n;
    uint16 csize;

    if(sd_getCSD(csd) != 0)
    {
        return 0;
    }
    if((csd[0] & 0xC0) == 0x40)
    {
        csize = csd[9] + ((uint16)csd[8] << 8) + 1;
        capacity = (uint32)csize << 10;
    }
    else
    {
        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
        csize = (csd[8] >> 6) + ((uint16)csd[7] << 2) + ((uint16)(csd[6] & 3) << 10) + 1;
        capacity= (uint32)csize << (n - 9);//得到扇区数
    }
    return capacity;
}

uint8 sd_initialize(void)
{
    uint8 rl;//存放sd卡的返回值
    uint16 retry;//用来进行超时计数
    uint8 buf[4];
    uint16 i;

    sd_spi_init();//初始化IO

    for(i = 0; i < 10; ++i)
    {
        sd_spi_readWriteByte(0xFF);//发送最少74个脉冲
    }
    retry = 200;
    do
    {
        rl = sd_sendCmd(CMD0, 0, 0x95);//进入IDLE状态
    } while((rl != 0x01) && retry--);


    rl = sd_sendCmd(CMD8, 0x1aa,0x87);//版本


    sd_type = 0;//默认无卡
    if(rl == 0x01)
    {
        if(sd_sendCmd(CMD8, 0x1AA, 0x87) == 1)//SD v2.0
        {
            for(i = 0; i < 4; ++i)
            {
                buf[i] = sd_spi_readWriteByte(0xFF);//get trailing return value of R7 resp
            }
            if(buf[2] == 0x01 && buf[3] == 0xAA)//卡是否支持2.7~3.6v
            {
                retry = 0xFFFE;
                do
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0x40000000, 0x01);
                } while(rl && retry--);
                if(retry && sd_sendCmd(CMD58, 0, 0x01) == 0)//鉴别sd2.0卡版本开始
                {
                    for(i = 0; i < 4; ++i)
                    {
                        buf[i] = sd_spi_readWriteByte(0xFF);//得到OCR值
                        if(buf[0] & 0x40)
                        {
                            sd_type = SD_TYPE_V2HC;//检查CCS
                        }
                        else
                        {
                            sd_type = SD_TYPE_V2;
                        }
                    }
                }
            }
        }
        else//sd v1.x/ MMC v3
        {
            sd_sendCmd(CMD55, 0, 0x01);//发送CMD55
            rl = sd_sendCmd(CMD41, 0, 0x01);//发送CMD41
            if(rl <= 1)
            {
                sd_type = SD_TYPE_V1;
                retry = 0xFFFE;
                do//等待退出IDLE模式
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0, 0x01);
                } while(rl && retry--);
            }
            else//MMC卡不支持CMD55+CMD41识别
            {
                sd_type = SD_TYPE_MMC;//MMC v3
                retry = 0xFFFE;
                do//退出IDLE模式
                {
                    rl = sd_sendCmd(CMD1, 0, 0x01);
                } while(rl && retry--);
            }
            if(retry == 0 || sd_sendCmd(CMD16, 512, 0x01) != 0)
            {
                sd_type = SD_TYPE_ERR;//错误的卡
            }
        }
    }
    sd_disSelect();//取消片选
    sd_spi_speedHigh();//高速
    if(sd_type)
    {
        return 0;
    }
    else if(rl)
    {
        return rl;
    }
    return 0xaa;//其他错误
}

uint8 sd_writeDisk(uint8 *buf, uint32 sector, uint8 cnt)
{
    uint8 rl;
    if(sd_type != SD_TYPE_V2HC)
    {
        sector *= 512;
    }
    if(cnt == 1)
    {
        rl = sd_sendCmd(CMD24, sector, 0x01);//写数据
        if(rl == 0)//指令发送成功
        {
            rl = sd_sendBlock(buf, 0xFE);//写512个字节
        }
    }
    else
    {
        if(sd_type != SD_TYPE_MMC)
        {
            sd_sendCmd(CMD55, 0, 0x01);
            sd_sendCmd(CMD23, cnt, 0x01);
        }
        rl = sd_sendCmd(CMD25, sector, 0x01);//连续读命令
        if(rl == 0)
        {
            do
            {
                rl = sd_sendBlock(buf, 0xFC);//接收512字节
                buf += 512;
            } while(--cnt && rl == 0);
            rl = sd_sendBlock(0, 0xFD);//接收512字节
        }
    }
    sd_disSelect();
    return rl;
}

uint8 sd_readDisk(uint8 *buf, uint32 sector, uint8 cnt)
{
    uint8 rl;
    if(sd_type != SD_TYPE_V2HC)
    {
        sector <<= 9;
    }
    if(cnt == 1)
    {
        rl = sd_sendCmd(CMD17, sector, 0x01);
        if(rl == 0)
        {
            rl = sd_recvData(buf, 512);
        }
    }
    else
    {
        sd_sendCmd(CMD18, sector, 0x01);
        do
        {
            rl = sd_recvData(buf, 512);
            buf += 512;
        }while(--cnt && rl == 0);
        sd_sendCmd(CMD12, 0, 0x01);
    }
    sd_disSelect();
    return rl;
}




