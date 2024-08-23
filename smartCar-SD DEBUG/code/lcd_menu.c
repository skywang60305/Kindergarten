/*
 * lcd_menu.c
 *
 *  Created on: 2021年8月11日
 *      Author: 95159
 */

#define PAGE_DISP_NUM 7
#include "lcd_menu.h"

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (8)                                 // 存储数据用的页码 倒数第一个页码
uint32 beep_time = 0;
uint32 beep_time_tmp = 0;
uint16 page_order = 0;                         //当前页序号
uint16 data_order = 0;                         //当前页中数据序号
uint32 EEPROM_ORDER_W = 0;                     //EEPROM数据写入序号
uint32 beep_flag = 0;
uint32 camera_flag = 0;
extern uint32 lcd_flag;
extern int16 mt9v03x_set_confing_buffer[MT9V03X_CONFIG_FINISH][2];

extern uint8 OSTU_MIN;
extern uint8 OSTU_MAX;
//extern KEY_MSG_t           key_msg[KEY_MSG_FIFO_SIZE];             //按键消息FIFO
//extern volatile uint8      key_msg_front;    //接收FIFO的指针
void beep_on()
{
    gpio_set_level(BEEP_ENABLE,1);
}
void beep_off()
{
    gpio_set_level(BEEP_ENABLE,0);
}
void beep_init()
{
    gpio_init(BEEP_ENABLE,GPO,0,GPO_PUSH_PULL);
}
void beep()
{
   beep_on();
   beep_time_tmp = beep_time;
   beep_flag =1;
}
void BEEP_FLAG()
{
    if(beep_flag ==1 )
    {
       if(beep_time_tmp>=10)
           beep_time_tmp-=10;
       else
        {
           beep_off();
           beep_flag = 0;
        }
    }
}
volatile uint8 ExitMenu_flag = 0;
uint32 *EEPROM_DATA[] = {
        (uint32 *)&beep_time,
        (uint32 *)&mt9v03x_set_confing_buffer[1][1],
        (uint32 *)&mt9v03x_set_confing_buffer[2][1],
        (uint32 *)&mt9v03x_set_confing_buffer[3][1],
        (uint32 *)&mt9v03x_set_confing_buffer[4][1],
        (uint32 *)&mt9v03x_set_confing_buffer[5][1],
        (uint32 *)&mt9v03x_set_confing_buffer[6][1],
        (uint32 *)&mt9v03x_set_confing_buffer[7][1],
        (uint32 *)&mt9v03x_set_confing_buffer[8][1],
};

//----------------------------------   主菜单   -------------------------------
//Menu_Flag menu_flag;



MENU_PRMT MainMenu_Prmt;

MENU_TABLE MainMenu_Table[] =
{
        {(uint8 *)"1.NULL          ", Menu_Null, NULL},
        {(uint8 *)"2.Beep          ", Menu_exp7, NULL},
        {(uint8 *)"3.Beep Time          ", Menu_exp8, NULL},
        {(uint8 *)"4.Camera Setting", Menu_Camera, NULL},
        {(uint8 *)"5.Camera Image  ", Menu_Image, NULL},
};


MENU_PRMT exp7_Prmt;

MENU_TABLE exp7_MenuTable[] =
{
        {(uint8 *)"1.BEEP ON    ", beep_on, NULL},
        {(uint8 *)"2.BEEP OFF   ", beep_off, NULL},
        {(uint8 *)"3.BEEP TIME  ", beep, NULL},
};

void Menu_exp7(void)
{
    tft180_clear();
    uint8 menuNum;
    menuNum = sizeof(exp7_MenuTable) / sizeof(exp7_MenuTable[0]); // 菜单项数
    Menu_Process((uint8 *)" -=    exp7    =- ", &exp7_Prmt, exp7_MenuTable, menuNum);
}

MENU_PRMT exp8_Prmt;

MENU_TABLE exp8_MenuTable[] =
{
        {(uint8 *)"1.beep time", Menu_Null, (uint16 *)&beep_time},
};

void Menu_exp8(void)
{

    tft180_clear();
    uint8 menuNum;
    menuNum = sizeof(exp8_MenuTable) / sizeof(exp8_MenuTable[0]); // 菜单项数
    Menu_Process((uint8 *)" -=    exp8    =- ", &exp8_Prmt, exp8_MenuTable, menuNum);
}

MENU_PRMT Camera_Prmt;

MENU_TABLE Camera_MenuTable[] =
{
        {(uint8*)"AUTO_EXP",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[1][1]},
        {(uint8*)"EXP_TIME",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[2][1]},
        {(uint8*)"FPS",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[3][1]},
        {(uint8*)"SET_COL",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[4][1]},
        {(uint8*)"SET_ROW",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[5][1]},
        {(uint8*)"LR_OFFSET",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[6][1]},
        {(uint8*)"UD_OFFSET",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[7][1]},
        {(uint8*)"GAIN",Menu_Null,(uint16*)&mt9v03x_set_confing_buffer[8][1]},
};

void Menu_Camera(void)
{
    camera_flag = 1;
    tft180_clear();
    uint8 menuNum;
    menuNum = sizeof(Camera_MenuTable) / sizeof(Camera_MenuTable[0]); // 菜单项数
    Menu_Process((uint8 *)" -=    Camera    =- ", &Camera_Prmt, Camera_MenuTable, menuNum);
}


MENU_PRMT Image_Prmt;

void image_show_gray(void)
{
    tft180_clear();
    while(1)
    {
        tft180_show_gray_image(0,0,mt9v03x_image[0],MT9V03X_W,MT9V03X_H,160,128,0);

        if(keymsg.status == KEY_DOWN)
        {
            if(keymsg.key==KEY_L)
            {
                tft180_clear();
                break;
            }
        }
    }
    //最后一个参数可改为二值化阈值
}
void image_show_binary(void)
{
    tft180_clear();
    uint8 threshold = 0;
    while(1)
    {
        threshold = getOstuThreshold(mt9v03x_image[0],CAMERA_W,0,YM,(uint8)OSTU_MAX,(uint8)OSTU_MIN);
        tft180_show_gray_image(0,0,mt9v03x_image[0],MT9V03X_W,MT9V03X_H,160,128,threshold);
        if(keymsg.status == KEY_DOWN)
        {
            if(keymsg.key==KEY_L)
            {
                tft180_clear();
                break;
            }
        }
    }
}
void image_show_all(void)
{
    tft180_clear();
    while(1)
    {
        Get_Image();
        binaryAlgorithm();
        tft180_show_gray_image(0,0,allmap[0],MT9V03X_W,MT9V03X_H,160,128,0);
        if(keymsg.status == KEY_DOWN)
        {
            if(keymsg.key==KEY_L)
            {
                tft180_clear();
                break;
            }
        }
    }

}
void Ostu_Compress(uint8 start_rows, uint8 end_rows)
{
    for(uint8 j=start_rows; j<end_rows; j++)
    {
        for(uint8 i=0; i<XM; i++)
        {
            uint8 cnt_black = 0;
//            for(uint8 index=0; index<4; index++)
//            {
//                if(imgGray[j][i*4 + index] < OSTU_THRESHOLD)
//                {
//                    cnt_black ++;
//                    img_binary[j][i*4 + index] = PIXEL_BLACK;
//                }
//                else
//                    img_binary[j][i*4 + index] = PIXEL_WHITE;
//            }

            if(cnt_black >= 2)
                allmap[j][i] = PIXEL_BLACK;
            else
                allmap[j][i] = PIXEL_WHITE;
        }
    }
}

MENU_TABLE Image_MenuTable[] =
{
        {(uint8 *)"1.Gray Image",image_show_gray , NULL},
        {(uint8 *)"2.Binarilized Image",image_show_binary , NULL},
        {(uint8 *)"2.All Image",image_show_all , NULL},
};

void Menu_Image(void)
{
    tft180_clear();
    uint8 menuNum;
    menuNum = sizeof(Image_MenuTable) / sizeof(Image_MenuTable[0]); // 菜单项数
    Menu_Process((uint8 *)" -=    Image    =- ", &Image_Prmt, Image_MenuTable, menuNum);
}


/******************************************************************************
 * FunctionName   : KeySan()
 * Description    : 按键获取
 * EntryParameter : None
 * ReturnValue    : 按键值
 *******************************************************************************/
KEY_e KeySan(void)
{

    while (keymsg.status == KEY_UP&&!ExitMenu_flag)
    {
//        key_check_m(keymsg);
    }
    keymsg.status = KEY_UP;
    return keymsg.key;
}

/******************************************************************************
 * FunctionName   : Menu_Move()
 * Description    : 菜单移动
 * EntryParameter :  prmt - 菜单参数, key - 按键值
 * ReturnValue    : 有确认返回0，否则返回1
 ******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key)
{
    uint8 rValue = 1;

    switch (key)
    {
        case KEY_U: // 向上
        {
            if (prmt->Cursor != 0) // 光标不在顶端
            {
                prmt->Cursor--; // 光标上移
            }
            else // 光标在顶端
            {
                if (prmt->PageNo != 0) // 页面没有到最小
                {
                    prmt->PageNo--; // 向上翻
                }
                else
                {
                    prmt->Cursor = prmt->DispNum - 1; // 光标到底
                    prmt->PageNo = prmt->MaxPage - 1; // 最后页
                    tft180_clear();
                }
            }
            keymsg.status = KEY_UP;


            break;
        }

        case KEY_D: // 向下
        {
            if (prmt->Cursor < prmt->DispNum - 1) // 光标没有到底，移动光标
            {
                prmt->Cursor++; // 光标向下移动
            }
            else // 光标到底
            {
                if (prmt->PageNo < prmt->MaxPage - 1) // 页面没有到底，页面移动
                {
                    prmt->PageNo++; // 下翻一页
                    tft180_clear();
                }
                else // 页面和光标都到底，返回开始页
                {
                    prmt->Cursor = 0;
                    prmt->PageNo = 0;
                    tft180_clear();
                }
            }

            keymsg.status = KEY_UP;

            break;
        }
        case KEY_R: // 确认
        {
            prmt->Index = prmt->Cursor + prmt->PageNo; //计算执行项的索引
            rValue = 0;
            keymsg.status = KEY_UP;

            break;
        }
        case KEY_L: // 左键返回上级菜单
        {

                prmt->Cursor = 0;
                prmt->PageNo = 0;
                prmt->ExitMark = 1;

            keymsg.status = KEY_UP;
            if(camera_flag == 1)
            {
                tft180_clear();
                camera_flag = 0;
            }

            break;
        }
//        case KEY_R: // 右键跳到底部
//        {
//            prmt->Cursor = prmt->DispNum - 1; // 光标到底
//            prmt->PageNo = prmt->MaxPage - 1; // 最后页
//            keymsg.status = KEY_UP;
//            break;
//        }

        default:
            break;
    }

    return rValue; // 返回执行索引
}

/******************************************************************************
 * FunctionName   : MainMenu_Set()
 * Description    : 常规设置
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void MainMenu_Set(void)
{

    uint32 tmp = 30;
    tft180_show_string(0,0,"choose mode");
    while(tmp--)
    {
        system_delay_ms(100);
     if (GET_SWITCH1())
     {
         tft180_show_string(0,0,"Read from eeprom!");
         Read_EEPROM();
     }
    }

    tft180_clear();
    ExitMenu_flag = 0;
    uint8 menuNum = sizeof(MainMenu_Table) / sizeof(MainMenu_Table[0]); // 菜单项数
    Menu_Process((uint8 *)" -=   Setting   =- ", &MainMenu_Prmt, MainMenu_Table, menuNum);
    Write_EEPROM(); //将数据写入EEPROM保存
    tft180_clear();
    lcd_flag = 1;
}

/******************************************************************************
 * FunctionName   : Menu_Process()
 * Description    : 处理菜单项
 * EntryParameter : menuName - 菜单名称，prmt - 菜单参数，table - 菜单表项, num - 菜单项数
 * ReturnValue    : None
 ******************************************************************************/
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)
{
//    if(table == MainMenu_Table)
//        menu_flag = Main_Menu;
//    else if(table == exp7_MenuTable)
//        menu_flag = Menu_1st;
    KEY_e key;
    Site_t site;

    uint8 page; //显示菜单需要的页数

    if (num - PAGE_DISP_NUM <= 0)
        page = 1;
    else
    {
        page = num - PAGE_DISP_NUM + 1;
        num = PAGE_DISP_NUM;
    }

    // 显示项数和页数设置
    Menu_PrmtInit(prmt, num, page);

    do
    {
        tft180_show_string((uint16)0, (uint16)0, (const char *)menuName); //显示菜单标题
        // 显示菜单项
        Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
        key = KeySan(); //获取按键

        if (Menu_Move(prmt, key) == 0) //菜单移动 按下确认键
        {
            // 判断此菜单项有无需要调节的参数 有则进入参数调节
            if (table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
            {
                site.x = 120;
                site.y = (1 + prmt->Cursor) * 16;

                tft180_show_uint(site.x, site.y, *(table[prmt->Index].DebugParam),8);
                adjustParam(site, table[prmt->Index].DebugParam, 4, RGB565_WHITE, RGB565_RED);
            }
            // 不是参数调节的话就执行菜单函数
            else
            {
                table[prmt->Index].ItemHook(); // 执行相应项
            }

        }
    } while (prmt->ExitMark == 0 && ExitMenu_flag == 0);


    tft180_clear();
}

/******************************************************************************
 * FunctionName   : Menu_PrmtInit()
 * Description    : 初始化菜单参数
 * EntryParameter : prmt - 菜单参数, num - 每页显示项数, page - 最大显示页数
 * ReturnValue    : None
 *******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
    prmt->ExitMark = 0; //清除退出菜单标志

    prmt->Cursor = 0;    //光标清零
    prmt->PageNo = 0;    //页清零
    prmt->Index = 0;     //索引清零
    prmt->DispNum = num; //页最多显示项目数

    prmt->MaxPage = page; //最多页数
}

/******************************************************************************
 * FunctionName   : Menu_Display()
 * Description    : 显示菜单项
 * EntryParameter : page - 显示页，dispNum - 每一页的显示项，cursor - 光标位置
 * ReturnValue    : None
 *******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor)
{
    uint8 i;
    Site_t site;

    for (i = 0; i < dispNum; i++)
    {
        if (cursor == i)
        {
            /* 反白显示当前光标选中菜单项 */
            site.x = 0;
            site.y = (i + 1) * 16;
            tft180_set_color(RGB565_WHITE,RGB565_BLUE);//默认红笔 白背景
            tft180_show_string((uint16)site.x, (uint16)site.y, (const char *)menuTable[pageNo + i].MenuName);
            tft180_set_color(RGB565_RED,RGB565_WHITE);
            /* 若此菜单项有需要调的参数，则显示该参数 */
            if (menuTable[pageNo + i].DebugParam != NULL)
            {
                site.x = 120;

                uint16 num_t = (*(menuTable[pageNo + i].DebugParam));
                tft180_show_uint(site.x, site.y, num_t,8);
            }
        }
        else
        {
            /* 正常显示其余菜单项 */
            site.x = 0;
            site.y = (i + 1) * 16;
            tft180_show_string((uint16)site.x, (uint16)site.y, (const char *)menuTable[pageNo + i].MenuName);
            /* 若此菜单项有需要调的参数，则显示该参数 */
            if (menuTable[pageNo + i].DebugParam != NULL)
            {
                site.x = 120;

                uint16 num_t = (*(menuTable[pageNo + i].DebugParam));

                tft180_show_uint(site.x, site.y, num_t,8);
            }
            /*else if (showDirFlag && pageNo + i == 0)
                        {
                            showDir();
                        }*/
        }
    }
}



void adjustParam(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor)
{
    do
    {
        KeySan();

        switch (keymsg.key)
        {
            case KEY_U:
                if (*param <= 65534)
                    (*param)+=10;
                break;

            case KEY_D:
                if (*param >= 1)
                    (*param)-=10;
                break;

            case KEY_L:
                if (*param >= 10)
                    (*param)--;
                break;

//            case KEY_R:
//                if (*param <= 65525)
//                    (*param) += 10;
//                break;

            default:
                break;
        }
        tft180_show_uint(site.x, site.y, *param,8);
    } while (keymsg.key != KEY_R);
}

void Menu_Null()
{
    tft180_clear();
    tft180_show_string(50,50,"NOTHING");
    system_delay_ms(2000);
    tft180_clear();
}

void Write_EEPROM(void)
{
    const uint16 Flash_Save_Num = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    flash_erase_page(0,11);
    uint32 *temp_ptr=NULL;
    for (uint16 i =0; i < Flash_Save_Num; i++)
    {
        temp_ptr=(uint32 *)EEPROM_DATA[i];
        if(temp_ptr!=NULL)
        {
            flash_union_buffer[i].uint32_type=(uint16)*temp_ptr;
        }
    }
    flash_write_page_from_buffer(0, 11);
    flash_buffer_clear();
}
void Read_EEPROM(void) {

    const uint16 Flash_Save_Num = sizeof(EEPROM_DATA) / sizeof(EEPROM_DATA[0]);
    flash_buffer_clear();
    flash_read_page_to_buffer(0, 11);
    uint32 *temp_ptr=NULL;
    for (uint16 i = 0; i < Flash_Save_Num; i++) {
        uint32 temp_vaule=flash_union_buffer[i].uint32_type;
        if((uint16)temp_vaule<5000)
        {
            temp_ptr=(uint32 *)EEPROM_DATA[i];
            if (temp_ptr != NULL) {
                *temp_ptr = (uint16)temp_vaule;
            }
        }
        else
        {
            temp_ptr=(uint32 *)EEPROM_DATA[i];
            if (temp_ptr != NULL) {
                *temp_ptr = 0;
            }
        }
    }
}

