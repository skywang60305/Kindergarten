/*
 * spi_sd.c
 *
 *  Created on: 2021��8��4��
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
// SD�����Ͷ���
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06
// SD��ָ���
#define CMD0    0       //����λ
#define CMD1    1
#define CMD8    8       //����8 ��SEND_IF_COND
#define CMD9    9       //����9 ����CSD����
#define CMD10   10      //����10����CID����
#define CMD12   12      //����12��ֹͣ���ݴ���
#define CMD16   16      //����16������SectorSize Ӧ����0x00
#define CMD17   17      //����17����sector
#define CMD18   18      //����18����Multi sector
#define CMD23   23      //����23�����ö�sectorд��ǰԤ�Ȳ���N��block
#define CMD24   24      //����24��дsector
#define CMD25   25      //����25��дMulti sector
#define CMD41   41      //����41��Ӧ����0x00
#define CMD55   55      //����55��Ӧ����0x01
#define CMD58   58      //����58����OCR��Ϣ
#define CMD59   59      //����59��ʹ��/��ֹCRC��Ӧ����0x00
//����д���Ӧ������
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD����Ӧ�����
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF

uint8 sd_type = 0;//sd������

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
    }while(t < 0xFFFFFF);//�ȴ�
    return 1;
}

void sd_disSelect(void)//ȡ��ѡ���ͷ�SPI����
{
    SD_CS_DISABLE;
    sd_spi_readWriteByte(0xFF);
}

uint8 sd_select(void)
{
    SD_CS_ENABLE;
    if(sd_waitReady() == 0)
    {
        return 0;//�ȴ��ɹ�
    }
    sd_disSelect();
    return 1;//�ȴ�ʧ��
}

uint8 sd_sendCmd(uint8 cmd, uint32 arg, uint8 crc)
{
    uint8 rl;
    uint8 retry = 0;
    uint8 send_buf[8];
    sd_disSelect();//ȡ���ϴ�Ƭѡ;
    sd_spi_readWriteByte(0xFF);
    if(sd_select())
    {
        return 0xFF;//Ƭѡʧ��
    }
    send_buf[0] = 0xff;
    send_buf[1] = ((uint8)(cmd | 0x40));//�ֱ�д������
    send_buf[2] = ((uint8)(arg >> 24));
    send_buf[3] = ((uint8)(arg >> 16));
    send_buf[4] = ((uint8)(arg >> 8));
    send_buf[5] = ((uint8)arg);
    send_buf[6] = (crc);
    send_buf[7] = 0xff;
//    //����
//    sd_spi_readWritspi_write_8bit_arrayeByte((uint8)(cmd | 0x40));//�ֱ�д������
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
    //һ�η���10������ 0~8λ 0xff �ھ�λ 0x01
    retry = 20;
    do
    {
        //20�� ȫ������0xff
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
        return 1;//�ȴ�׼��ʧ��
    }
    sd_spi_readWriteByte(cmd);
    if(cmd != 0xFD)//���ǽ���ָ��
    {
        for(t = 0; t < 512; t++)
        {
            sd_spi_readWriteByte(buf[t]);
        }
        sd_spi_readWriteByte(0xFF);//����crc
        sd_spi_readWriteByte(0xFF);
        t = sd_spi_readWriteByte(0xFF);//������Ӧ
        if((t & 0x1F) != 0x05)
        {
            return 2;//��Ӧ����
        }
    }
    return 0;//д��ɹ�
}

//��ʼ��ʱ��Ҫ����
void sd_spi_speedLow(void)
{
    spi_init(SD_SPIN, SPI_MODE3, 400 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

//����������ʱ����Ը�����
void sd_spi_speedHigh(void)
{
    spi_init(SD_SPIN, SPI_MODE3, 36 * 1000 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

void sd_spi_init(void)
{
    //Ӳ��SPI��ʼ������ʼ������ģʽ������ԭ��Ϊ280k��ģʽӦ����3
    spi_init(SD_SPIN, SPI_MODE3, 400 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
//    spi_init(SPI_2, SPI_MODE0, 1*1000*1000, SPI2_SCLK_P15_3, SPI2_MOSI_P15_5, SPI2_MISO_P15_4, SPI2_CS0_P15_2); // Ӳ��SPI��ʼ��  ģʽ0 ������Ϊ1Mhz
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

//��ȡSD����CSD��Ϣ�������������ٶ���Ϣ
//����:u8 *cid_data(���CID���ڴ棬����16Byte��
//����ֵ:0��NO_ERR
//       1������
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

//��ȡSD����������������������
//����ֵ:0�� ȡ��������
//       ����:SD��������(������/512�ֽ�)
//ÿ�������ֽ�����Ϊ512����Ϊ�������512�����ʼ������ͨ��.
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
        capacity= (uint32)csize << (n - 9);//�õ�������
    }
    return capacity;
}

uint8 sd_initialize(void)
{
    uint8 rl;//���sd���ķ���ֵ
    uint16 retry;//�������г�ʱ����
    uint8 buf[4];
    uint16 i;

    sd_spi_init();//��ʼ��IO

    for(i = 0; i < 10; ++i)
    {
        sd_spi_readWriteByte(0xFF);//��������74������
    }
    retry = 200;
    do
    {
        rl = sd_sendCmd(CMD0, 0, 0x95);//����IDLE״̬
    } while((rl != 0x01) && retry--);


    rl = sd_sendCmd(CMD8, 0x1aa,0x87);//�汾


    sd_type = 0;//Ĭ���޿�
    if(rl == 0x01)
    {
        if(sd_sendCmd(CMD8, 0x1AA, 0x87) == 1)//SD v2.0
        {
            for(i = 0; i < 4; ++i)
            {
                buf[i] = sd_spi_readWriteByte(0xFF);//get trailing return value of R7 resp
            }
            if(buf[2] == 0x01 && buf[3] == 0xAA)//���Ƿ�֧��2.7~3.6v
            {
                retry = 0xFFFE;
                do
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0x40000000, 0x01);
                } while(rl && retry--);
                if(retry && sd_sendCmd(CMD58, 0, 0x01) == 0)//����sd2.0���汾��ʼ
                {
                    for(i = 0; i < 4; ++i)
                    {
                        buf[i] = sd_spi_readWriteByte(0xFF);//�õ�OCRֵ
                        if(buf[0] & 0x40)
                        {
                            sd_type = SD_TYPE_V2HC;//���CCS
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
            sd_sendCmd(CMD55, 0, 0x01);//����CMD55
            rl = sd_sendCmd(CMD41, 0, 0x01);//����CMD41
            if(rl <= 1)
            {
                sd_type = SD_TYPE_V1;
                retry = 0xFFFE;
                do//�ȴ��˳�IDLEģʽ
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0, 0x01);
                } while(rl && retry--);
            }
            else//MMC����֧��CMD55+CMD41ʶ��
            {
                sd_type = SD_TYPE_MMC;//MMC v3
                retry = 0xFFFE;
                do//�˳�IDLEģʽ
                {
                    rl = sd_sendCmd(CMD1, 0, 0x01);
                } while(rl && retry--);
            }
            if(retry == 0 || sd_sendCmd(CMD16, 512, 0x01) != 0)
            {
                sd_type = SD_TYPE_ERR;//����Ŀ�
            }
        }
    }
    sd_disSelect();//ȡ��Ƭѡ
    sd_spi_speedHigh();//����
    if(sd_type)
    {
        return 0;
    }
    else if(rl)
    {
        return rl;
    }
    return 0xaa;//��������
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
        rl = sd_sendCmd(CMD24, sector, 0x01);//д����
        if(rl == 0)//ָ��ͳɹ�
        {
            rl = sd_sendBlock(buf, 0xFE);//д512���ֽ�
        }
    }
    else
    {
        if(sd_type != SD_TYPE_MMC)
        {
            sd_sendCmd(CMD55, 0, 0x01);
            sd_sendCmd(CMD23, cnt, 0x01);
        }
        rl = sd_sendCmd(CMD25, sector, 0x01);//����������
        if(rl == 0)
        {
            do
            {
                rl = sd_sendBlock(buf, 0xFC);//����512�ֽ�
                buf += 512;
            } while(--cnt && rl == 0);
            rl = sd_sendBlock(0, 0xFD);//����512�ֽ�
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




