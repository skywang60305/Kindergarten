#include "deal_img.h"
#ifdef BOOM7_QT_DEBUG
extern int32 nowSpeedL_SD;
extern int32 nowSpeedR_SD;
#endif
/*************************************************
Copyright (C), 2020-2022, Tech. Co., Ltd.
File name: deal_img.c
Author: Samurai 王若愚 BOOM7
Version: unknown
Date: 2021/12/15
Description: 图像处理
// 或函数的接口，输出值、取值范围、含义及参数间的控
// 制、顺序、独立或依赖等关系
Others: 与BOOM之前的图像处理相比多加了扫线，相对的修改起来会比较困难些
Function List:
    略，直接看头文件
History: 于2021/12/15开始记录本文件的重要修改
1. Date: 2021/12/17
Author: Samurai_WRY
Modification: 增加真实角度计算函数，并利用于三叉判别
2. Date: 2021/12/18
Author: Samurai_WRY
Modification: 单边十字只判出口
3. Date: 2022/1/17
Author: Samurai_WRY
Modification: 将角度计算放在祖传的下角点检测之后，这个下角点判定是我唯一懂作用但是看不懂的函数了，十分准确，可以信任
              这些角度可用于区分十字三叉等
4. Date: 2022/2/10
Author: Samurai_WRY
Modification: 发现三叉内用中线控制偏差计算较为连续，直接取某个高度内的中线点来拟合kb
5. Date: 2022/2/12
Author: Samurai_WRY
Modification: 根据左或右侧特征判十字的1状态，继续对另一侧进行isCross_XXXX的判定，成功判定则将
              状态转化为正入十字
6. Date: 2022/2/24 第一次跑到3.2m/s
Author: Samurai_WRY
Modification: 三叉改回用正常控制方式，发现十字状态横向扫线会有bug
7. Date: 2022/2/25
Author: Samurai_WRY
Modification: 十字和单边十字合并，状态机大改

    左十字+右十字->正十字

    正入单边十字还没有见过图像，之后再说

    有行车记录仪的想法，记录下前范围内几帧偏差，并判定已经走了哪些弯，过了哪些元素什么的，然后可以写个决策树（笑嘻了）真的智能一手，用在速度规划吧，
    不用机器学习算法其实不太配得上智能这两个字的，如果我到最后还没有用机器学习的话，后人尽量填一下吧哈哈哈，杭电已经有轻度智能了
8. Date: 2022/3/5
Author: Samurai_WRY
Modification: 三叉判定大改，状态机重写
9. Date: 2022/3/12
Author: Samurai_WRY
Modification: 摄像头拉到30cm，开始一个一个凹元素
10. Date: 2022/4/15
Author: Samurai_WRY
Modification: 摄像头还原20cm，高摄像头图像是好，不过问题很多，抬轮，摄像头晃，圆环路径很差什么的，都要窒息了，
              不想纠结了，干脆直接拉回来，最好这是最后一次动摄像头了，现在已经应该花时间把图像写到没有任何bug，开始搞速度规划了
11. Date: 2022/4/27
Author: Samurai_WRY
Modification: 补线的距离改为可调，往返十字确定为自动选择方向加手写eeprom保障
12. Date: 2022/5/11
Author: Samurai_WRY
Modification: 加入灰度图判车库起跑线
13. Date: 2022/5/14
Author: Samurai_WRY
Modification: 图像处理基本写完，接下来就是稳定性，把图像这边的无用代码一一去除，然后容易判的元素，和不容易判的元素，分别做处理
              容易判的，多判几帧，不容易的，在状态机里加防误判
14. Date: 2022/5/14
Author: Samurai_WRY
Modification: 特殊元素内speedTop修正一下，对normalshift进行了修改，基于speedTop进行一些变速，所以speedTop要找对，variable的话就是在normalshift基础上放开了长直加速
15. Date: 2022/5/15
Author: Samurai_WRY
Modification: 根据TFMINI略微放开坡道的一些条件，更改圆环状态机进入条件，单判出某一边的十字也可进入，判下圆环预判，但不判圆环

16. Date: 2022/8/16（国赛前5天）
Author: Samurai_WRY
Modification: 5/15之后就没再记录了，因为没什么大改了，微调居多。建议不管大二还是大三，最晚在5月15号左右必须基本搞完图像了，后面还要弄速度规划，调电机，很麻烦。
              平时存的图不要删掉，省赛前一个礼拜的时候，一定要再过一遍
 *************************************************/
#ifdef BOOM7_QT_DEBUG
extern uint8 v_x_left;
extern uint8 v_y_left;
extern uint8 p1_x_left;
extern uint8 p1_y_left;
extern uint8 p2_x_left;
extern uint8 p2_y_left;
extern uint8 v_x_right;
extern uint8 v_y_right;
extern uint8 p1_x_right;
extern uint8 p1_y_right;
extern uint8 p2_x_right;
extern uint8 p2_y_right;
#endif
extern uint8 rampNum;
extern uint8 outGarageFlag;
extern EulerAngleTypedef angle;
/*图像信息结构体与赛道元素标志位*/
IMG_INFO II;
IMG_INFO LAST_II;
IMG_INFO II_INIT;
IMG_FLAGS IF;
IMG_FLAGS LAST_IF;

/*共有的图像操作*/
uint8 OSTU_THRESHOLD = 100;
uint8 OSTU_THRESHOLD_PRE = 0;
uint8 OSTU_MAX = 130;
uint8 OSTU_MIN = 0;
uint16 sobel_threshold = 30;
uint16 robert_threshold = 30;
uint8 highOstuGain = 110;
uint8 highOstuRow = 5; // 10;
uint16 histogram[256];
uint8 imgGray[CAMERA_H][CAMERA_W];    // 原图的拷贝
uint8 img_binary[CAMERA_H][CAMERA_W]; // 将拉伸后的图二值化 255是白色 0是黑色

/*祖传算法*/
uint8 basemap[YM][XM];
uint8 leftmap[YM][XM];
uint8 rightmap[YM][XM];
uint8 insidemap[YM][XM];
uint8 deletemap[YM][XM];
uint8 allmap[YM][XM];
uint8 midline[YM / 2];
PtStack noDown;
uint8 dbottomline[XM] = {0};
uint16 numCnt = 0;
uint8 deleteforleftflag = 0;
uint8 deleteforrightflag = 0;
/*扫线*/
lineInfo LI_INIT, RI_INIT, MI_INIT, HL_INIT, HR_INIT, BL_INIT, BR_INIT;
lineInfo LI, RI, MI, HL, HR, BL, BR;
keyPoints PL, PR, PL_INIT, PR_INIT;
/*控制方面*/
float leftline[YM];
float rightline[YM];
float bodyworkLine_left[YM];
float bodyworkLine_right[YM];
/*判到的元素个数*/
uint8 judgedAnnulusNum = 0;
uint8 judgedRampNum = 0;
uint8 judgedForkNum = 0;
/*开始补线的标志位*/
uint8 startAddLine_annulus = 0;
uint8 startAddLine_fork = 0;
uint8 startAddLine_enterGarage = 0;
/*清状态标志位*/
uint8 ramp_clearFlag = 0;
uint8 annulus_clearFlag = 0;
uint8 cross_clearFlag = 0;
uint8 garage_clearFlag = 0;
uint8 fork_clearFlag = 0;
uint8 ucross_clearFlag = 0;

uint8 dir_ucross_clearFlag = 0;
// 圆环
AnnulusDEV AD;
AnnulusDEV AD_INIT = {0, 0, 0, 0};

// 三叉
uint8 dir_fork = goRight;

// 十字
// 往返十字判定严谨度调节，越大越严，0到20
uint8 rigor_ucross_both = 8;
uint8 rigor_ucross_oneSide = 15;

uint8 dir_ucross = 0;
uint8 ucross_manual_setting = 0;
uint8 dir_ucross_manual_setting_1 = goLeft;
uint8 dir_ucross_manual_setting_2 = goLeft;
uint8 cnt_ucross = 0;
uint8 useUcrossServoOffset = 0;
uint8 UcrossServoOffset = 10;

uint8 lastUpLeft_x = 0;
uint8 lastUpLeft_y = YM;
uint8 lastUpRight_x = XM;
uint8 lastUpRight_y = YM;
uint8 lastDownLeft_x = XM;
uint8 lastDownLeft_y = YM;
uint8 lastDownRight_x = 0;
uint8 lastDownRight_y = YM;

// 车库
uint8 fistJudgeFlag = 0;
uint8 judgedCnt = 0;
uint8 garageDirectionFlag = goRight;
int16 stopCarCnt = 100;

// 开始补线或者说刹车的距离
uint8 drawLine_distance_fork = 20;
uint8 drawLine_distance_annulus = 70;
uint8 drawLine_distance_ucross = 20;
uint8 brake_distance_ucross = 45;
uint8 drawLine_distance_enterGarage = 35;
uint16 threshold_TFmini = 100;

// 逆透视
float k1[YM];
// 各种高度的逆透视表，不要直接用，自己量自己的，这里仅供参考
// 10cm
// float k2[YM] = {16, 16.5, 17, 17.5, 18, 18.5, 19.5, 20, 21, 22, 22.5, 23, 23, 24, 25,
//                26, 27, 27.5, 28.5, 29.5, 30.5, 31.5, 33, 34, 35, 36, 37, 38.5, 39, 41.5, 43.5,
//                45, 46, 47.5, 49.5, 52, 53.5, 55.5, 57, 59, 61.5, 63, 65.5, 68.5, 72, 77, 80,
//                84, 88, 92, 98.5, 104, 117, 125, 134, 145.5, 161, 165.5, 187.5, 200};
// 20cm中置
// float k2[YM] = { 30.00,30.09,30.26,30.50,30.82,31.20,31.65,32.16,32.72,33.34,
//                  34,34.71,35.46,36.25,37.07,37.92,38.80,39.70,40.62,41.56,42.5,
//                  43.45,44.43,45.43,46.46,47.54,48.68,49.89,51.17,52.54,54,55.57,
//                  57.24,59.01,60.88,62.84,64.90,67.04,69.2,71.597,74,76.494,79.132,
//                  81.977,85.092,88.541,92.386,96.691,101.52,106.93,113,119.83,127.76,
//                  137.18,148.46,162,178.18,197.38,220,246.41};
// };
// float k2[YM] = { 17.00,17.50,18.10,18.70,19.20,20.00,20.60,21.10,21.60,22.10,
//                  22.60,23.20,23.91,24.60,25.30,26.00,26.80,27.68,28.48,29.50,
//                  30.20,31.10,32.00,32.80,33.50,34.40,35.70,36.90,38.08,39.52,
//                  41.00,42.52,44.20,45.72,47.50,49.20,51.30,53.20,56.20,59.29,
//                  62.70,65.00,67.80,70.20,73.86,78.76,82.20,86.00,90.50,95.57,
//                  105.5,109.8,121.2,130.3,140.0,153.5,167.0,180.2,200.0,220.0};

// 江南k2
float k2[YM] = {21.30, 21.50, 22.60, 23.00, 23.50, 24.10, 24.70, 25.10, 25.60, 26.30,
                26.70, 27.50, 28.10, 28.90, 29.30, 30.00, 30.80, 31.50, 32.00, 32.70,
                33.40, 34.30, 35.30, 36.20, 37.00, 37.80, 39.10, 40.50, 41.50, 42.90,
                44.40, 45.50, 47.00, 48.70, 50.50, 52.00, 54.00, 55.80, 57.30, 59.50,
                61.70, 64.80, 67.80, 70.40, 72.90, 76.80, 80.20, 85.00, 89.00, 93.50,
                100.50, 112.00, 127.00, 136.80, 148.00, 161.50, 177.50, 194.00, 215.0};
// 30cm
// float k2[YM] = {7.160303183966766,8.510538141977758,9.359262489450225,9.890568040746544,10.245000059091923,10.5264231423627,10.808229419162254,11.138918758972888,11.54708070017197,12.045807799700686,
//                 12.636570108173697,13.31258047421802,14.061680381829495,14.868776024535094,15.717854320149419,16.593608569913787,17.482703406580607,18.37470914980966,19.26273402893006,20.143786049747135,
//                 21.018892136291527,21.893005495033428,22.7747304907724,23.675894797215655,24.610998526034592,25.596570038186357,26.650458141289505,27.791090376842433,29.036727101072124,30.404741063202295,
//                 31.910952184927694,33.569047244886406,35.390114171912806,37.38232065086648,39.55076674481768,41.897541237386896,44.42201139901418,47.12137588096523,49.99151044083561,53.028136203375304,
//                 56.228340160389315,59.592477613521915,63.126486263711826,66.84464165109861,70.77278364917441,74.95204371696025,79.44310261302522,84.33100827508254,89.73058356900637,95.79245361103308,
//                 102.70972236691759,110.7253282318692,120.14010829503948,131.32160099233806,144.71361685136245,160.8466070322936,180.348859368435,203.95855161027671,232.53669157684686,267.0809739180937,
//                };
float standardK = 1.1;
float standardB = 22;
float bodyworkK = 0.8;
float bodyworkB = 45;
/*************************************************
Function: standard()
Description: 设置标准线，图像信息大初始化
Input: null
Output: null
Return: null
Others: null
Author: zjut
 *************************************************/
void standard() // 标准线，根据摄像头高度角度调整
{
    /*标准线与扫线数组初始化*/
    for (uint8 i = 0; i < YM; ++i)
    {
        leftline[i] = (standardK * i + standardB) / 4;
        rightline[i] = XM - leftline[i];
        bodyworkLine_left[i] = (bodyworkK * i + bodyworkB) / 4;
        bodyworkLine_right[i] = XM - bodyworkLine_left[i];
        // 以后要把点都算出来
        k1[i] = 40.0 / (1.0 * (rightline[i] - leftline[i] + 1)); // float强制转换要放到里面去
        II_INIT.leftline[i] = 0;
        II_INIT.leftfindflag[i] = NOT_FOUND;
        II_INIT.leftdeleteflag[i] = 0;
        II_INIT.rightline[i] = XM;
        II_INIT.rightfindflag[i] = NOT_FOUND;
        II_INIT.rightdeleteflag[i] = 0;
        II_INIT.midline[i] = XM; // 这里置为XM就是说没算过
        II_INIT.midfindflag[i] = 0;
        II_INIT.middeleteflag[i] = 0;
    }
    /*扫线关键信息*/
    II_INIT.breakY = YY;
    II_INIT.searchLineMid = myCarMid; // 置为mycarmid为了保证即使找扫线中点出问题，扫出来的线也有一定参考价值
    II_INIT.line_forbid = NONELINE;
    II_INIT.speedTop = YM;
    II_INIT.annulusTop = YM;
    II_INIT.leftSearchRow = YM;
    II_INIT.rightSearchRow = YM;
    II_INIT.leftEndCol = 0;
    II_INIT.rightEndCol = XX;
    II_INIT.leftPortaitSearchLineFlag = 0;
    II_INIT.rightPortaitSearchLineFlag = 0;
    II_INIT.blnum = 0;
    II_INIT.brnum = 0;
    II_INIT.hlnum = 0;
    II_INIT.hrnum = 0;
    for (uint8 i = 0; i < XM; i++)
    {
        II_INIT.highlineLeft[i] = YM;
        II_INIT.bottomlineLeft[i] = YM;
        II_INIT.highlineRight[i] = YM;
        II_INIT.bottomlineRight[i] = YM;
    }
    for (uint8 i = 0; i < 10; i++)
    {
        LI_INIT.start[i] = 0;
        LI_INIT.end[i] = 0;
        LI_INIT.numCnt[i] = 0;
        LI_INIT.isUseful[i] = 1;

        RI_INIT.start[i] = 0;
        RI_INIT.end[i] = 0;
        RI_INIT.numCnt[i] = 0;
        RI_INIT.isUseful[i] = 1;

        MI_INIT.start[i] = 0;
        MI_INIT.end[i] = 0;
        MI_INIT.numCnt[i] = 0;
        MI_INIT.isUseful[i] = 1;

        HL_INIT.start[i] = 0;
        HL_INIT.end[i] = 0;
        HL_INIT.numCnt[i] = 0;
        HL_INIT.isUseful[i] = 0;

        HR_INIT.start[i] = 0;
        HR_INIT.end[i] = 0;
        HR_INIT.numCnt[i] = 0;
        HR_INIT.isUseful[i] = 0;

        BL_INIT.start[i] = 0;
        BL_INIT.end[i] = 0;
        BL_INIT.numCnt[i] = 0;
        BL_INIT.isUseful[i] = 0;

        BR_INIT.start[i] = 0;
        BR_INIT.end[i] = 0;
        BR_INIT.numCnt[i] = 0;
        BR_INIT.isUseful[i] = 0;

        PL_INIT.right_x[i] = 0;
        PL_INIT.left_x[i] = 0;
        PL_INIT.right_y[i] = 0;
        PL_INIT.left_y[i] = 0;

        PR_INIT.right_x[i] = 0;
        PR_INIT.left_x[i] = 0;
        PR_INIT.right_y[i] = 0;
        PR_INIT.left_y[i] = 0;
    }

    /*搜图关键信息与关键点*/
    II_INIT.num_lm = 0;
    II_INIT.num_rm = 0;

    memset(II_INIT.start_lm, 0, sizeof(II_INIT.start_lm));
    memset(II_INIT.start_rm, 0, sizeof(II_INIT.start_rm));

    II_INIT.d_bottom[0].x = 0;
    II_INIT.d_bottom[0].y = YY;
    II_INIT.d_bottom[1].x = XX;
    II_INIT.d_bottom[1].y = YY;

    II_INIT.repeatNum = 0;
    II_INIT.dnum_top = 0;
    II_INIT.bnum_all = 0;
    II_INIT.lnum_all = 0;
    II_INIT.rnum_all = 0;
    II_INIT.dnum_all = 0;
    II_INIT.inum_all = 0;
    II_INIT.leftnum = 0;
    II_INIT.rightnum = 0;
    II_INIT.lnum_control = 0;
    II_INIT.rnum_control = 0;
    II_INIT.midnum = 0;
    II_INIT.top = 0;

    II_INIT.left_x = XM;
    II_INIT.right_x = 0;
    II_INIT.left_y = YM;
    II_INIT.right_y = YM;
    II_INIT.upLeft_x = XM;
    II_INIT.upLeft_y = YM;
    II_INIT.upRight_x = 0;
    II_INIT.upRight_y = YM;
    II_INIT.angleL = 0;
    II_INIT.angleR = 0;

    II.d_bottom[0].x = 0;
    II.d_bottom[0].y = YY;
    II.d_bottom[1].x = XX;
    II.d_bottom[1].y = YY;

    for (uint8 i = 0; i < 5; ++i)
    {
        II_INIT.left_bottom[i].x = XX;
        II_INIT.right_bottom[i].x = 0;
        II_INIT.left_top[i].x = XX;
        II_INIT.right_top[i].x = 0;

        II_INIT.bottom_left[i].x = XX;
        II_INIT.bottom_right[i].x = 0;
        II_INIT.top_left[i].x = XX;
        II_INIT.top_right[i].x = 0;

        II_INIT.left_bottom[i].y = YY;
        II_INIT.right_bottom[i].y = YY;
        II_INIT.left_top[i].y = 0;
        II_INIT.right_top[i].y = 0;

        II_INIT.bottom_left[i].y = YY;
        II_INIT.bottom_right[i].y = YY;
        II_INIT.top_left[i].y = 0;
        II_INIT.top_right[i].y = 0;
    }
    II_INIT.annulusDealFlag = 0;
    for (uint8 i = 0; i < XM; i++)
    {
        II_INIT.highlineLeft[i] = YM;
        II_INIT.bottomlineLeft[i] = YM;
        II_INIT.highlineRight[i] = YM;
        II_INIT.bottomlineRight[i] = YM;
    }

    II.leftCrossDealFlag = 0;
    II.rightCrossDealFlag = 0;
    II.annulusDealFlag = 0;

    II_INIT.startRow = START_LINE;
    II_INIT.endRow = STOP_LINE;
    II.riskLevelLeft = 0;
    II.riskLevelRight = 0;
    II = II_INIT;
    IF.annulus = 0;
    IF.annulusDelay = 0;
    IF.crossroad = 0;
    IF.garage = 0;
    IF.ramp = 0;
    IF.rampDelay = 0;
    IF.startline = 0;
    IF.fork = 0;
    IF.crossAnticipation = 0;
    IF.annulusAnticipation = 0;
    BSI_INIT.areaLeft = 0;
    BSI_INIT.areaRight = 0;
    BSI_INIT.status = NORMAL;
    LAST_IF = IF;
}

/*************************************************
Function: map_init()
Description: 图像数组初始化
Input: null
Output: null
Return: null
Others: 注意初始化的值是什么
Author:zjut
 *************************************************/
void map_init()
{
    memset(basemap, 1, sizeof(basemap));
    memset(leftmap, 0, sizeof(leftmap));
    memset(rightmap, 0, sizeof(rightmap));
    memset(insidemap, 1, sizeof(insidemap));
    memset(deletemap, 0, sizeof(deletemap));
    noDown.num = 0;
    for (uint8 i = 0; i < XM; i++)
        dbottomline[i] = YY;
    deleteforleftflag = 0;
    deleteforrightflag = 0;
}

/*************************************************
Function: standard()
Description: 每一帧图像处理开始前，进行数组及结构体初始化
Input: null
Output: null
Return: null
Others: null
Author: Samurai
 *************************************************/
void statusReset()
{
    LAST_II = II;
    II = II_INIT;
    map_init();
    LI = LI_INIT;
    RI = RI_INIT;
    HL = HL_INIT;
    HR = HR_INIT;
    BL = BL_INIT;
    BR = BR_INIT;
    PL = PL_INIT;
    PR = PR_INIT;
    BSI = BSI_INIT;
    LAST_IF = IF;
}

/*************************************************
Function: binaryAlgorithm()
Description: BOOM5的二值化算法
Input: null
Output: null
Return: null
Others: 将灰度图二值化
Author: BOOM5 HLZ
 *************************************************/
void binaryAlgorithm()
{
    OSTU_THRESHOLD = myNewOstuThreshold(imgGray[0], CAMERA_W, 0, YM, (uint8)OSTU_MAX, (uint8)OSTU_MIN);
    Ostu_Robert(imgGray[0], img_binary[0], OSTU_THRESHOLD, sobel_threshold, 0, YM);
    horizonCompress(allmap[0], img_binary[0]);
    uint8 tmp;
    for (uint8 j = 0; j < YM / 2; j++) // 竖直方向的镜像
        for (uint8 i = 0; i < XM; i++)
        {
            tmp = allmap[j][i];
            allmap[j][i] = allmap[YY - j][i];
            allmap[YY - j][i] = tmp;
        }
}

/*************************************************
Function: myNewOstuThreshold(uint8 *image, uint32 col,
            uint32 startRows,uint32 endRows,
            uint8 GrayScale_Max,uint8 GrayScale_Min)
Description: BOOM5的大津法阈值获取
Input: 图像数组指针，图像宽度，起始行，终点行，阈值计算的始末
Output: null
Return: null
Others: 用于BOOM5的二值化算法，感觉GrayScale_Max和GrayScale_Min没啥必要
Author: BOOM5 HLZ
 *************************************************/
uint8 myNewOstuThreshold(uint8 *image, uint32 col, uint32 startRows, uint32 endRows, uint8 GrayScale_Max, uint8 GrayScale_Min)
{
#define GrayScale 256
    uint32 width = col;
    uint32 pixelCount[GrayScale];
    float pixelPro[GrayScale];
    uint32 i, j, pixelSum = width * (endRows - startRows);
    uint8 threshold = 0;
    uint8 *data = image; // 指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum = 0;
    // 统计灰度级中每个像素在整幅图像中的个数   //即获得直方图隔行获得
    for (i = startRows; i < endRows; i += 1)
        for (j = 0; j < width; j += 1)
        {
            pixelCount[(uint32)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (uint32)data[i * width + j];   // 灰度值总和
        }

    // 计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    // 遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = u0tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = GrayScale_Min; j < GrayScale_Max; j++)
    {
        w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j]; // 背景部分 每个灰度值的点的比例 *灰度值
        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;
        u0 = u0tmp / w0;                                                // 背景平均灰度
        u1 = u1tmp / w1;                                                // 前景平均灰度
        u = u0tmp + u1tmp;                                              // 全局平均灰度
        deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u); // 块一丢丢
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)j;
        }
        if (deltaTmp < deltaMax)
            break;
    }
    if (threshold < OSTU_MIN)
        threshold = OSTU_MIN;
    if (threshold > OSTU_MAX)
        threshold = OSTU_MAX;
    return threshold;
}

/*************************************************
Function: Ostu_Robert(unsigned char* org_in, unsigned char* ostu_out,unsigned char th_ostu
            , unsigned int th_edge, unsigned int start_rows, unsigned int end_rows)
Description: BOOM5的阳光算法
Input: 输入图像数组指针，输出图像数组指针，大津法阈值，
        罗伯特算子阈值，使用罗伯特算子的起始行和终点行
Output: null
Return: null
Others: 和BOOM4核心思路区别不大，所以也叫阳光算法2
        就是代码量少了些
Author: BOOM5 HLZ
 *************************************************/

#define MAX_COLS CAMERA_W                                                                                                                                     /*图像水平分辨率（188）*/
#define MAX_ROWS CAMERA_H                                                                                                                                     /*图像竖直分辨率（60）*/
void Ostu_Robert(unsigned char *org_in, unsigned char *ostu_out, unsigned char th_ostu, unsigned int th_edge, unsigned int start_rows, unsigned int end_rows) // 阳光算法2
{
    unsigned int i, col;
    unsigned int start_pixel, end_pixel;
    col = 0;
    start_pixel = start_rows * MAX_COLS; // 像素=实际长度*分辨率(此处实际长度被像素点个数替换)
    end_pixel = (end_rows + 1) * MAX_COLS;

    unsigned int dealMax = end_pixel - MAX_COLS;
    for (i = start_pixel; i < dealMax; i++) // 传进去的图像是反的所以底部是有条黑的
    {
        col++;
        if (org_in[i] >= th_ostu) /*等号是否需要根据小车情况进行处理*/ // 大津法
            ostu_out[i - start_pixel] = 255;                           /*置白*/
        else
            ostu_out[i - start_pixel] = 0; /*置黑*/
    }
}
#undef MAX_COLS
#undef MAX_ROWS

/*************************************************
Function: horizonCompress(uint8 tempimgbuff[], uint8 img[])
Description: BOOM5的图像压缩
Input: 输出图像数组指针，输入图像数组指针
Output: null
Return: null
Others: BOOM5的二值化是没有压缩的，所以还需要压缩处理
Author: BOOM5 HLZ
 *************************************************/
void horizonCompress(uint8 tempimgbuff[], uint8 img[])
{
    int dealMax = CAMERA_H * CAMERA_W / HORIZON_TIMES;
    for (int i = 0; i < dealMax; i++)
    {
        uint8 blackNumCnt = 0;
        for (int index = 0; index < HORIZON_TIMES; index++)
            if ((*(img + i * HORIZON_TIMES + index)) < 255)
                blackNumCnt++;

        if (blackNumCnt >= 2)
            tempimgbuff[i] = PIXEL_BLACK; // 2个黑点置黑色
        else
            tempimgbuff[i] = PIXEL_WHITE; // tempimgbuff为压缩完的图像数组
    }
}

/*---------------------------------------------------------------------------------------------------------------
 * ------------------------------------这里开始是赛道信息获取-------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------*/
/*************************************************
Function: searchLines(uint8 setMid)
Description: 基础扫线函数
Input: 扫线中心列
Output: null
Return: null
Others: 对每一行从中心列向两边扫，遇到黑点则记录为useful
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void searchLines(uint8 setMid)
{
    uint8 mid = setMid;

    for (uint8 j = 0; j < YM; j++) // 从下往上扫
    {
        if (basemap[j][mid] != 0)
        {
            II.breakY = j - 1;

            if (j <= 20) // 防止在一开始就break
                continue;
            break;
        }
        for (uint8 i = mid - 1; i < XX; i--)                  // 找左线
            if (basemap[j][i] != 0 && basemap[j][i + 1] != 0) // 连续遇到两个黑点
            {
                II.leftfindflag[j] = USEFUL; // 说明这个点扫线有扫到
                II.leftline[j] = i + 2;      // 边线赋值
                break;
            }
        for (uint8 i = mid + 1; i <= XX && i > 0; i++) // 找右线
            if (basemap[j][i] != 0 && basemap[j][i - 1] != 0)
            {
                II.rightfindflag[j] = USEFUL;
                II.rightline[j] = i - 2;
                break;
            }
        if (II.leftfindflag[j] || II.rightfindflag[j])
            II.midfindflag[j] = 1;
        II.midline[j] = (uint8)(0.5 * (II.leftline[j] + II.rightline[j]));
    }
}

/*************************************************
Function: getSearchLineColMid(uint8 setMid)
Description: 获取扫线中心列
Input: 上次的扫线中心列，中心列变化范围
Output: null
Return: 本次的扫线中心列
Others: 找范围内白点延续最高的一列
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getSearchLineColMid(uint8 lastMid, uint8 range)
{
    uint8 maxRow = 0;
    uint8 thisMid = XM;
    for (uint8 i = 0; i < range; i++)
    {
        // 向右找
        if (lastMid + i < XM)
            // 从下往上
            for (uint8 j = 0; j < YM; j++)
                if (allmap[j][lastMid + i] == PIXEL_BLACK || j == YY)
                {
                    if (j > maxRow)
                    {
                        maxRow = j;
                        thisMid = lastMid + i;
                    }
                    break;
                }
        // 向左找
        if (lastMid >= i)
            for (uint8 j = 0; j < YM; j++)
                if (allmap[j][lastMid - i] == PIXEL_BLACK || j == YY)
                {
                    if (j > maxRow)
                    {
                        maxRow = j;
                        thisMid = lastMid - i;
                    }
                    break;
                }
    }
    if (thisMid != XM)
        return thisMid;
    return lastMid;
}

/*************************************************
Function: getSearchLineColMid_oneSide(uint8 lastMid,uint8 range,uint8 dir)
Description: 单方向获取扫线中心列
Input: 上次的扫线中心列，中心列变化范围，寻找方向
Output: null
Return: 本次的扫线中心列
Others: 单方向找范围内白点延续最高的一列
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getSearchLineColMid_oneSide(uint8 lastMid, uint8 range, uint8 dir)
{
    uint8 maxRow = 0;
    uint8 thisMid = XM;
    if (dir == goLeft)
        for (uint8 i = 0; i < range; i++)
            if (lastMid >= i)
                for (uint8 j = 0; j < YM; j++)
                    if (allmap[j][lastMid - i] == 0 || j == YY)
                    {
                        if (j >= maxRow)
                        {
                            maxRow = j;
                            thisMid = lastMid - i;
                        }
                        break;
                    }
    if (dir == goRight)
        for (uint8 i = 0; i < range; i++)
            if (lastMid + i < XM)
                for (uint8 j = 0; j < YM; j++)
                    if (allmap[j][lastMid + i] == 0 || j == YY)
                    {
                        if (j >= maxRow)
                        {
                            maxRow = j;
                            thisMid = lastMid + i;
                        }
                        break;
                    }
    if (thisMid != XM)
        return thisMid;
    return lastMid;
}

/*************************************************
Function: searchLine_Portait(uint8 startRow,uint8 startCol,uint8 dir)
Description: 纵向扫线
Input: 扫线起始行，扫线起始列，扫线方向
Output: null
Return: null
Others: 解释不太清楚，看代码里注释吧
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void searchLines_Portait(uint8 startRow, uint8 startCol, uint8 dir)
{
    uint8 col = startCol;
    uint8 top = II.breakY < 58 ? II.breakY : 58;
    if (dir == goLeft)
    {
        /*
         * 向左进行
         */
        II.leftPortaitSearchLineFlag = 1;
        for (; col != 255; col--)
        {
            if (allmap[startRow][col] == PIXEL_BLACK)
            {
                II.leftEndCol = col + 1;
                break;
            }
            // 向上扫
            for (uint8 row = startRow; row < YY; row++)
                if (allmap[row][col] == PIXEL_BLACK)
                {
                    // 下面的判断很重要，不是所有纵向扫到的点都有用的
                    // 这些条件可以最大限度的排除无用点
                    // 这里row-1才是纵向扫线记录的点
                    if ((IF.crossAnticipation || !II.leftfindflag[row - 1]                        // 横向扫线没扫到                                                        //
                         || II.leftdeleteflag[row - 1]                                            // 被删了                                                            //这四条满足一个就行
                         || (distance(col, row - 1, II.leftline[row - 1], row - 1) > 2)           // 和左线该行扫到的线有距离                    //
                         || (distance(II.leftline[row], row, II.leftline[row - 1], row - 1) > 3)) // 左线中该行与相邻行扫线的点相差较大//
                        && row - 1 < top)                                                         // 这条必须满足
                    {
                        II.highlineLeft[col] = row - 1;
                        II.hlnum++;
                    }
                    break;
                }
            // 向下扫的时候没那么多事，扫到都要了
            for (uint8 row = startRow; row > 0; row--)
                if (allmap[row][col] == PIXEL_BLACK)
                {
                    II.bottomlineLeft[col] = row + 1;
                    II.blnum++;
                    break;
                }
        }
    }
    if (dir == goRight)
    {
        /*
         * 向右进行
         */
        II.rightPortaitSearchLineFlag = 1;
        for (; col < XM; col++)
        {
            if (allmap[startRow][col + 1] == PIXEL_BLACK)
            {
                II.rightEndCol = col - 1;
                break;
            }
            // 向上
            for (uint8 row = startRow; row < YY; row++)
                if (allmap[row][col] == PIXEL_BLACK)
                {
                    if ((IF.crossAnticipation || !II.rightfindflag[row - 1] || II.rightdeleteflag[row - 1] || (distance(col, row - 1, II.rightline[row - 1], row - 1) > 2) || (distance(II.rightline[row], row, II.rightline[row - 1], row - 1) > 3)) && row - 1 < top)
                    {
                        II.highlineRight[col] = row - 1;
                        II.hrnum++;
                    }
                    break;
                }
            // 向下
            for (uint8 row = startRow; row > 0; row--)
                if (allmap[row][col] == PIXEL_BLACK)
                {
                    II.bottomlineRight[col] = row + 1;
                    II.brnum++;
                    break;
                }
        }
    }
}

/*************************************************
Function: getSearchLineRowLeft(uint8 startRow,uint8 startCol,uint8 dir,uint8 range)
Description: 寻找左边纵向扫线起始行
Input: 扫线起始行，扫线起始列，寻找方向，寻找范围
Output: null
Return: 纵向扫线起始行
Others: 找范围内横向延伸最远的点作为纵向扫线起始行
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getSearchLineRowLeft(uint8 startRow, uint8 startCol, uint8 dir, uint8 range)
{
#ifdef BOOM7_QT_DEBUG_SEARCHLINES
    qout << "getSearchLineRowLeft" << "startCol" << startCol << "startRow" << startRow << "range" << range;
#endif
    uint8 minCol = XM;
    uint8 thisRow = YM;
    uint8 rowTemp;
    if (dir == goDown)
    {
        if (range > startRow)
            range = startRow;
        for (uint8 i = 0; i < range; i++)
        {
            uint8 j = startCol;
            rowTemp = startRow - i;
            for (; j > 0; j--)
                if (allmap[rowTemp][j] == PIXEL_BLACK)
                {
                    if (j <= minCol)
                    {
                        minCol = j;
                        thisRow = rowTemp;
                    }
                    break;
                }
            if (j == 0)
                return rowTemp;
        }
    }
    if (dir == goUp)
    {
        if (range + startRow > YY)
            range = YY - startRow;
        for (uint8 i = 0; i < range; i++)
        {
            rowTemp = startRow + i;
            uint8 j = startCol;
            for (; j > 0; j--)
                if (allmap[rowTemp][j] == PIXEL_BLACK)
                {
                    if (j <= minCol)
                    {
                        minCol = j;
                        thisRow = rowTemp;
                    }
                    break;
                }
            if (j == 0)
                return rowTemp;
        }
    }
    if (thisRow > II.breakY)
        thisRow = II.breakY;
    return thisRow;
}

/*************************************************
Function: getSearchLineRowLeft(uint8 startRow,uint8 startCol,uint8 dir,uint8 range)
Description: 寻找右边纵向扫线起始行
Input: 扫线起始行，扫线起始列，寻找方向，寻找范围
Output: null
Return: 纵向扫线起始行
Others: 找范围内横向延伸最远的点作为纵向扫线起始行
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getSearchLineRowRight(uint8 startRow, uint8 startCol, uint8 dir, uint8 range)
{
#ifdef BOOM7_QT_DEBUG_SEARCHLINES
    qout << "getSearchLineRowRight" << "startCol" << startCol << "startRow" << startRow << "range" << range;
#endif
    uint8 maxCol = 0;
    uint8 thisRow = YM;
    uint8 rowTemp;
    if (dir == goDown)
    {
        if (range > startRow)
            range = startRow;
        for (uint8 i = 0; i < range; i++)
        {
            rowTemp = startRow - i;
            uint8 j = startCol;
            for (; j < XX; j++)
                if (allmap[rowTemp][j] == PIXEL_BLACK)
                {
                    if (j >= maxCol)
                    {
                        maxCol = j;
                        thisRow = rowTemp;
                    }
                    break;
                }
            if (j == XX)
                return rowTemp;
        }
    }
    if (dir == goUp)
    {
        if (range + startRow > YY)
            range = YY - startRow;
        for (uint8 i = 0; i < range; i++)
        {
            rowTemp = startRow + i;
            uint8 j = startCol;
            for (; j < XX; j++)
                if (allmap[rowTemp][j] == PIXEL_BLACK)
                {
                    if (j >= maxCol)
                    {
                        maxCol = j;
                        thisRow = rowTemp;
                    }
                    break;
                }
            if (j == XX)
                return rowTemp;
        }
    }
    if (thisRow > II.breakY)
        thisRow = II.breakY;
    return thisRow;
}

/*************************************************
Function: getLineInfoLeft()，getLineInfoRight()
Description: 对基础扫线进行分析，分段，决定是否需要开纵向扫线
Input: null
Output: null
Return: null
Others: 扫线分析核心函数，这里处理好了后面判元素才舒服，仔细看注释
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void getLineInfoLeft()
{
    uint8 flag = 0; // 判断前一高度是否扫到线
    int now = 0;
    uint8 rowTmp1 = YM, rowTmp2 = YM; // 记录纵向扫线的起始行
    // 根据横纵坐标上的连续性分段
    for (uint8 j = 0; j <= II.breakY; j++)
    {
        if (!II.leftfindflag[j] || j == II.breakY) // j高度没扫到或者到顶了
        {
            if (flag == 1) // 前一高度是扫到线的
            {
                flag = 0; // 置0，说明这次没有扫到线
                if (j != II.breakY)
                    LI.end[LI.segCnt - 1] = j - 1; // 记录第segCnt个连续线段终点
                else
                    LI.end[LI.segCnt - 1] = j;
                LI.numCnt[LI.segCnt - 1] = LI.end[LI.segCnt - 1] - LI.start[LI.segCnt - 1] + 1; // 算下线段点数
            }
            else if (j == II.breakY && II.leftfindflag[j]) // 最高点看看是不是单独一个点，是的话就删掉
                II.leftdeleteflag[j] = 1;
        }
        else // j高度扫到线
        {
            if (flag == 0) // 如果上一高度没有扫到线
            {
                LI.segCnt++;                 // 线段个数加1
                LI.start[LI.segCnt - 1] = j; // 记录起点
                flag = 1;                    // 置1，说明这次找到了
            }
            else
            {
                now = (int)II.leftline[j + 1] - (int)II.leftline[j]; // 计算相邻高度两点距离
                if (abs(now) > 5)                                    // 相差过大，分一段（考虑是否要修改）
                    if (flag == 1)                                   // 前一高度是扫到线的
                    {
                        flag = 0;          // 置0，说明这次没有扫到线
                        if (LI.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                        {
                            LI.end[LI.segCnt - 1] = j;
                            LI.numCnt[LI.segCnt - 1] = LI.end[LI.segCnt - 1] - LI.start[LI.segCnt - 1] + 1; // 算下线段点数
                        }
                    }
            }
        }
    }
#define DELETE_RANGE 8
    for (uint8 k = 0; k < LI.segCnt; k++) // 依次遍历各线段
    {
        if (LI.numCnt[k] < 5 || (LI.numCnt[k] < 10 && L_START_Y(k) > 30 && L_START_X(k) < DELETE_RANGE && L_END_X(k) < DELETE_RANGE)) // 点数小于5，删掉
        {
            for (uint8 j = LI.start[k]; j <= LI.end[k]; j++)
            {
                II.leftdeleteflag[j] = 1; // 删线标志位置1
                II.leftfindflag[j] = NOT_FOUND;
            }
            for (uint8 j = k; j < LI.segCnt; j++) // 从当前开始依次从后往前移
            {
                LI.start[j] = LI.start[j + 1];
                LI.end[j] = LI.end[j + 1];
                LI.numCnt[j] = LI.numCnt[j + 1];
            }
            LI.segCnt--;
            k--;
        }
        else if (!IF.annulus) // （到时候不开试试）
        {
            uint8 cnt = 0; // 记录连续递减点数
            uint8 row = LI.start[k];
            for (uint8 i = LI.start[k]; i <= LI.end[k]; i++)
                if (II.leftline[i + 1] < II.leftline[i] || (cnt > 0 && II.leftline[i + 1] <= II.leftline[i])) // 这里两句没有多余，cnt>0时相等的点也算上
                {
                    cnt++;
                    if (cnt > 3 && row < 55) // 过高就不开纵向扫线了
                    {
                        // 开启纵向扫线条件1：一段线中出现连续3个以上递减点
                        rowTmp1 = getSearchLineRowLeft(row, II.searchLineMid, goUp, 55 - row);
                        break;
                    }
                }
                else if (II.leftline[i + 1] > II.leftline[i])
                {
                    cnt = 0;
                    row = i + 1;
                }
        }
    }
    /*
     * 对每一段线头尾两点进行判断
     */
    for (uint8 k = 0; k < LI.segCnt; k++)
    {
        int now = (int)II.leftline[LI.start[k]] - (int)II.leftline[LI.start[k] + 1];
        if (abs(now) > 5)
        {
            II.leftdeleteflag[LI.start[k]] = 1;
            LI.start[k]++;
            LI.numCnt[k]--;
        }
        now = (int)II.leftline[LI.end[k]] - (int)II.leftline[LI.end[k] - 1];
        if (abs(now) > 5)
        {
            II.leftdeleteflag[LI.end[k]] = 1;
            LI.end[k]--;
            LI.numCnt[k]--;
        }
    }
    getKeyPoints_L();
    // 特殊元素特殊选取纵向扫线行
    if (IF.annulus == AL1)
    {
        uint8 flag = 0;
        for (uint8 row = 0; row < 35; row++)
        {
            if (II.leftfindflag[row])
                flag = 1;
        }
        if (flag)
            II.leftSearchRow = getSearchLineRowLeft(II.breakY, II.searchLineMid, goDown, 30);
        else
            II.leftSearchRow = 0;
        searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
    }
    else if (IF.annulus == AL2)
    {
        II.leftSearchRow = 0;
        searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
    }
    else if (IF.crossroad == CL1 || IF.crossroad == CM1 || IF.crossroad == UL1 || IF.crossAnticipation == CPL)
    {
        if (LI.segCnt && L_START_Y(0) < 20)
            II.leftSearchRow = getSearchLineRowLeft(PL.right_y[0], II.searchLineMid, goUp, 48 - PL.right_y[0]);
        else if (II.num_lm && RT_Y(0) < 40)
            II.leftSearchRow = getSearchLineRowLeft(RT_Y(0), II.searchLineMid, goUp, 10);
        else
            II.leftSearchRow = getSearchLineRowLeft(0, II.searchLineMid, goUp, 30);
        searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
    }
    else if (IF.crossroad == CM2 || IF.crossroad == CL2 || IF.crossroad == UL2 || IF.crossAnticipation == CPR)
    {
        II.leftSearchRow = getSearchLineRowLeft(0, II.searchLineMid, goUp, 30);
        searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
    }
    else if (IF.garage == 1 || IF.garage == 3 /* || IF.garage == 5*/)
    {
        II.leftSearchRow = getSearchLineRowLeft(L_START_Y(LI.segCnt - 1), II.searchLineMid, goDown, 5);
        searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
    }
    // 一般的时候
    else
    {
        for (uint8 k = 0; k < LI.segCnt - 1; k++) // 依次遍历各线段
            if (LI.start[k + 1] - LI.end[k] > 3)
            {
                // 开启纵向扫线条件2：前一段线和后一段线之间丢线行数大于2
                uint8 range = LI.start[k + 1] - LI.end[k] > 5 ? LI.start[k + 1] - LI.end[k] : 5;
                rowTmp2 = getSearchLineRowLeft(LI.end[k], II.searchLineMid, goUp, range);
                break;
            }
            else if (abs((int)L_START_X(k + 1) - (int)L_END_X(k)) > 8)
            {
                // 开启纵向扫线条件3：前一段线终点和后一段线起点之间横坐标差距过大
                rowTmp2 = getSearchLineRowLeft(LI.end[k] - 1, II.searchLineMid, goUp, 5);
                //                rowTmp2 = L_END_Y(k);
                break;
            }
        // 太高的时候纵向扫线不开
        if (rowTmp1 > 56)
            rowTmp1 = YM;
        if (rowTmp2 > 56)
            rowTmp2 = YM;
        // 接下来选择纵向扫线的起始行
        // 1和2选低的
        if (rowTmp1 != YM || rowTmp2 != YM)
        {
            if (rowTmp1 != YM && rowTmp2 != YM)
                II.leftSearchRow = rowTmp1 < rowTmp2 ? rowTmp1 : rowTmp2;
            else if (rowTmp1 != YM)
                II.leftSearchRow = rowTmp1;
            else if (rowTmp2 != YM)
                II.leftSearchRow = rowTmp2;
            searchLines_Portait(II.leftSearchRow, II.searchLineMid, goLeft);
        }
    }

    /*
     * 如果有过纵向扫线，则也对纵向线段分段，以备使用，方法类似
     */
    if (II.leftPortaitSearchLineFlag)
    {
        uint8 flag = 0;
        int now = 0;
        for (uint8 col = II.searchLineMid; col >= II.leftEndCol && col != 255; col--)
        {
            if (II.highlineLeft[col] == YM || col == II.leftEndCol)
            {
                if (flag == 1) // 前一高度是扫到线的
                {
                    flag = 0;
                    if (col != II.leftEndCol)
                        HL.end[HL.segCnt - 1] = col + 1; // 记录第segCnt个连续线段终点
                    else
                        HL.end[HL.segCnt - 1] = col;
                    HL.numCnt[HL.segCnt - 1] = HL.start[HL.segCnt - 1] - HL.end[HL.segCnt - 1] + 1; // 算下线段点数
                }
            }
            else // 扫到线
            {
                if (flag == 0) // 如果上一高度没有扫到线
                {
                    HL.segCnt++;                   // 线段个数计数器自加
                    HL.start[HL.segCnt - 1] = col; // 记录起点
                    flag = 1;                      // 置1，说明这次找到了
                }
                else
                {
                    now = (int)II.highlineLeft[col - 1] - (int)II.highlineLeft[col];
                    if (abs(now) > 3)
                        if (flag == 1) // 前一高度是扫到线的
                        {
                            flag = 0;          // 置0，说明这次没有扫到线
                            if (HL.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                            {
                                HL.end[HL.segCnt - 1] = col;
                                HL.numCnt[HL.segCnt - 1] = HL.start[HL.segCnt - 1] - HL.end[HL.segCnt - 1] + 1; // 算下线段点数
                            }
                        }
                }
            }
        }
        for (uint8 k = 0; k < HL.segCnt; k++) // 依次遍历各线段
            if (HL.numCnt[k] <= 5)            // 点数小于5，删掉
            {
                for (uint8 j = HL.start[k]; j <= HL.end[k]; j++)
                    II.highlineLeft[j] = YM;          // 删线标志位置1
                for (uint8 j = k; j < HL.segCnt; j++) // 从当前开始依次从后往前移
                {
                    HL.start[j] = HL.start[j + 1];
                    HL.end[j] = HL.end[j + 1];
                    HL.numCnt[j] = HL.numCnt[j + 1];
                }
                HL.segCnt--;
                k--;
            }
        flag = 0;
        for (uint8 col = II.searchLineMid; col >= II.leftEndCol && col != 255; col--)
        {
            if (II.bottomlineLeft[col] == YM || col == II.leftEndCol)
            {
                if (flag == 1) // 前一高度是扫到线的
                {
                    flag = 0;
                    if (BL.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                    {
                        if (col != II.leftEndCol)
                            BL.end[BL.segCnt - 1] = col + 1; // 记录第segCnt个连续线段终点
                        else
                            BL.end[BL.segCnt - 1] = col;
                        BL.numCnt[BL.segCnt - 1] = BL.start[BL.segCnt - 1] - BL.end[BL.segCnt - 1] + 1; // 算下线段点数
                    }
                }
            }
            else // 扫到线
            {
                if (flag == 0) // 如果上一高度没有扫到线
                {
                    BL.segCnt++;                   // 线段个数计数器自加
                    BL.start[BL.segCnt - 1] = col; // 记录起点
                    flag = 1;                      // 置1，说明这次找到了
                }
                else
                {
                    now = (int)II.bottomlineLeft[col - 1] - (int)II.bottomlineLeft[col];
                    if (abs(now) > 5)
                    {
                        if (flag == 1) // 前一高度是扫到线的
                        {
                            flag = 0;          // 置0，说明这次没有扫到线
                            if (BL.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                            {
                                BL.end[BL.segCnt - 1] = col;
                                BL.numCnt[BL.segCnt - 1] = BL.start[BL.segCnt - 1] - BL.end[BL.segCnt - 1] + 1; // 算下线段点数
                            }
                        }
                    }
                }
            }
        }
        for (uint8 k = 0; k < HL.segCnt; k++)
        {
            int now = II.highlineLeft[HL.start[k]] - II.highlineLeft[HL.start[k] - 1];
            if (abs(now) > 3)
            {
                HL.start[k]++;
                BL.numCnt[k]--;
            }
            now = II.highlineLeft[HL.end[k]] - II.highlineLeft[HL.end[k] + 1];
            if (abs(now) > 3)
            {
                HL.end[k]--;
                HL.numCnt[k]--;
            }
        }
        for (uint8 k = 0; k < BL.segCnt; k++)
        {
            int now = II.bottomlineLeft[BL.start[k]] - II.bottomlineLeft[BL.start[k] - 1];
            if (abs(now) > 3)
            {
                BL.start[k]++;
                BL.numCnt[k]--;
            }
            now = II.bottomlineLeft[BL.end[k]] - II.bottomlineLeft[BL.end[k] + 1];
            if (abs(now) > 3)
            {
                BL.end[k]--;
                BL.numCnt[k]--;
            }
        }
    }
}

void getLineInfoRight()
{
    uint8 flag = 0; // 判断前一高度是否扫到线
    int now;
    uint8 rowTmp1 = YM, rowTmp2 = YM;
    // 根据横纵坐标上的连续性分段
    for (uint8 j = 0; j <= II.breakY; j++)
    {
        if (!II.rightfindflag[j] || j == II.breakY)
        {
            if (flag == 1)
            {
                flag = 0;
                if (j != II.breakY)
                    RI.end[RI.segCnt - 1] = j - 1;
                else
                    RI.end[RI.segCnt - 1] = j;
                RI.numCnt[RI.segCnt - 1] = RI.end[RI.segCnt - 1] - RI.start[RI.segCnt - 1] + 1; // 算下线段点数
            }
            else if (j == II.breakY && II.rightfindflag[j])
                II.rightdeleteflag[j] = 1;
        }
        else // 扫到线
        {
            if (flag == 0) // 如果上一高度没有扫到线
            {
                RI.segCnt++;                 // 线段个数计数器自加
                RI.start[RI.segCnt - 1] = j; // 记录起点
                flag = 1;                    // 置1，说明这次找到了
            }
            else
            {
                now = (int)II.rightline[j + 1] - (int)II.rightline[j];
                if (abs(now) > 5)
                    if (flag == 1) // 前一高度是扫到线的
                    {
                        flag = 0;          // 置0，说明这次没有扫到线
                        if (RI.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                        {
                            RI.end[RI.segCnt - 1] = j;
                            RI.numCnt[RI.segCnt - 1] = RI.end[RI.segCnt - 1] - RI.start[RI.segCnt - 1] + 1; // 算下线段点数
                        }
                    }
            }
        }
    }
    for (uint8 k = 0; k < RI.segCnt; k++) // 依次遍历各线段
    {
        if (RI.numCnt[k] < 5 || (RI.numCnt[k] < 10 && R_START_Y(k) > 30 && R_START_X(k) > XX - DELETE_RANGE && R_END_X(k) > XX - DELETE_RANGE)) // 点数小于5，删掉
        {
            for (uint8 j = RI.start[k]; j <= RI.end[k]; j++)
            {
                II.rightdeleteflag[j] = 1; // 删线标志位置1
                II.rightfindflag[j] = NOT_FOUND;
            }
            for (uint8 j = k; j < RI.segCnt; j++) // 从当前开始依次从后往前移
            {
                RI.start[j] = RI.start[j + 1];
                RI.end[j] = RI.end[j + 1];
                RI.numCnt[j] = RI.numCnt[j + 1];
            }
            RI.segCnt--;
            k--;
        }
        else if (!IF.annulus)
        {
            uint8 cnt = 0;
            uint8 row = RI.start[k];
            for (uint8 i = RI.start[k]; i <= RI.end[k]; i++)
                if (II.rightline[i + 1] > II.rightline[i] || (cnt > 0 && II.rightline[i + 1] >= II.rightline[i]))
                {
                    cnt++;
                    if (cnt >= 3 && row < 55)
                    {
                        rowTmp1 = getSearchLineRowRight(row, II.searchLineMid, goUp, 55 - row);
                        break;
                    }
                }
                else if (II.rightline[i + 1] < II.rightline[i])
                {
                    cnt = 0;
                    row = i + 1;
                }
        }
    }
    /*
     * 现在对头尾两点进行判断
     */
    for (uint8 k = 0; k < RI.segCnt; k++)
    {
        int now = (int)II.rightline[RI.start[k]] - (int)II.rightline[RI.start[k] + 1];
        if (abs(now) > 5)
        {
            II.rightdeleteflag[RI.start[k]] = 1;
            RI.start[k]++;
            RI.numCnt[k]--;
        }
        now = (int)II.rightline[RI.end[k]] - (int)II.rightline[RI.end[k] - 1];
        if (abs(now) > 5)
        {
            II.rightdeleteflag[RI.end[k]] = 1;
            RI.end[k]--;
            RI.numCnt[k]--;
        }
    }
    getKeyPoints_R();
    if (IF.annulus == AR1)
    {
        uint8 flag = 0;
        for (uint8 row = 0; row < 35; row++)
        {
            if (II.rightfindflag[row])
                flag = 1;
        }
        if (flag)
            II.rightSearchRow = getSearchLineRowRight(II.breakY, II.searchLineMid, goDown, 30);
        else
            II.rightSearchRow = 0;
        searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
    }
    else if (IF.annulus == AR2)
    {
        II.rightSearchRow = 0;
        searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
    }
    else if (IF.crossroad == CR1 || IF.crossroad == CM1 || IF.crossroad == UR1 || IF.crossAnticipation == CPR)
    {
        if (RI.segCnt && R_START_Y(0) < 20)
            II.rightSearchRow = getSearchLineRowRight(PR.left_y[0], II.searchLineMid, goUp, 48 - PR.left_y[0]);
        else if (II.num_rm && LT_Y(0) < 40)
            II.rightSearchRow = getSearchLineRowRight(LT_Y(0), II.searchLineMid, goUp, 10);
        else
            II.rightSearchRow = getSearchLineRowRight(0, II.searchLineMid, goUp, 30);
        searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
    }
    else if (IF.crossroad == CM2 || IF.crossroad == CR2 || IF.crossroad == UR2 || IF.crossAnticipation == CPL)
    {
        II.rightSearchRow = getSearchLineRowRight(0, II.searchLineMid, goUp, 30);
        searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
    }
    else if (IF.garage == 2 || IF.garage == 4 /* || IF.garage == 6*/)
    {
        II.rightSearchRow = getSearchLineRowRight(R_START_Y(RI.segCnt - 1), II.searchLineMid, goDown, 5);
        searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
    }
    else
    {
        for (uint8 k = 0; k < RI.segCnt - 1; k++) // 依次遍历各线段
            if (RI.start[k + 1] - RI.end[k] > 3)
            {
                uint8 range = RI.start[k + 1] - RI.end[k] > 5 ? RI.start[k + 1] - RI.end[k] : 5;
                rowTmp2 = getSearchLineRowRight(RI.end[k], II.searchLineMid, goUp, range);
                break;
            }
            else if (abs((int)R_START_X(k + 1) - (int)R_END_X(k)) > 8)
            {
                rowTmp2 = getSearchLineRowRight(RI.end[k] - 1, II.searchLineMid, goUp, 5);
                break;
            }
        // 2021/5/21新加：若线段数大于0，第一条线段起始点大于一定值，开启纵向扫线
        if (rowTmp1 > 56)
            rowTmp1 = YM;
        if (rowTmp2 > 56)
            rowTmp2 = YM;
        if (rowTmp1 != YM || rowTmp2 != YM)
        {
            if (rowTmp1 != YM && rowTmp2 != YM)
                II.rightSearchRow = rowTmp1 < rowTmp2 ? rowTmp1 : rowTmp2;
            else if (rowTmp1 != YM)
                II.rightSearchRow = rowTmp1;
            else if (rowTmp2 != YM)
                II.rightSearchRow = rowTmp2;
            searchLines_Portait(II.rightSearchRow, II.searchLineMid, goRight);
        }
    }

    /*
     * 如果有过纵向扫线，则也对纵向线段分段，以备使用
     */
    if (II.rightPortaitSearchLineFlag)
    {
        uint8 flag = 0; // 判断前一高度是否扫到线
        int now = 0;
        for (uint8 col = II.searchLineMid; col <= II.rightEndCol && col != 255; col++)
        {
            if (II.highlineRight[col] == YM || col == II.rightEndCol)
            {
                if (flag == 1) // 前一高度是扫到线的
                {
                    flag = 0;
                    if (col != II.rightEndCol)
                        HR.end[HR.segCnt - 1] = col - 1; // 记录第segCnt个连续线段终点
                    else
                        HR.end[HR.segCnt - 1] = col;
                    HR.numCnt[HR.segCnt - 1] = HR.end[HR.segCnt - 1] - HR.start[HR.segCnt - 1] + 1; // 算下线段点数
                }
            }
            else // 扫到线
            {
                if (flag == 0) // 如果上一高度没有扫到线
                {
                    HR.segCnt++;                   // 线段个数计数器自加
                    HR.start[HR.segCnt - 1] = col; // 记录起点
                    flag = 1;                      // 置1，说明这次找到了
                }
                else
                {
                    now = (int)II.highlineRight[col - 1] - (int)II.highlineRight[col];
                    if (abs(now) > 3)
                        if (flag == 1) // 前一高度是扫到线的
                        {
                            flag = 0;          // 置0，说明这次没有扫到线
                            if (HR.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                            {
                                HR.end[HR.segCnt - 1] = col;
                                HR.numCnt[HR.segCnt - 1] = HR.end[HR.segCnt - 1] - HR.start[HR.segCnt - 1] + 1; // 算下线段点数
                            }
                        }
                }
            }
        }
        for (uint8 k = 0; k < HR.segCnt; k++) // 依次遍历各线段
            if (HR.numCnt[k] <= 5)            // 点数小于5，删掉
            {
                for (uint8 j = HR.start[k]; j <= HR.end[k]; j++)
                    II.highlineRight[j] = YM;         // 删线标志位置1
                for (uint8 j = k; j < HR.segCnt; j++) // 从当前开始依次从后往前移
                {
                    HR.start[j] = HR.start[j + 1];
                    HR.end[j] = HR.end[j + 1];
                    HR.numCnt[j] = HR.numCnt[j + 1];
                }
                HR.segCnt--;
                k--;
            }
        flag = 0;
        for (uint8 col = II.searchLineMid; col <= II.rightEndCol && col != 255; col++)
        {
            if (II.bottomlineRight[col] == YM || col == II.rightEndCol)
            {
                if (flag == 1) // 前一高度是扫到线的
                {
                    flag = 0;
                    if (col != II.rightEndCol)
                        BR.end[BR.segCnt - 1] = col - 1; // 记录第segCnt个连续线段终点
                    else
                        BR.end[BR.segCnt - 1] = col;
                    BR.numCnt[BR.segCnt - 1] = BR.end[BR.segCnt - 1] - BR.start[BR.segCnt - 1] + 1; // 算下线段点数
                }
            }
            else // 扫到线
            {
                if (flag == 0) // 如果上一高度没有扫到线
                {
                    BR.segCnt++;                   // 线段个数计数器自加
                    BR.start[BR.segCnt - 1] = col; // 记录起点
                    flag = 1;                      // 置1，说明这次找到了
                }
                else
                {
                    now = (int)II.bottomlineRight[col - 1] - (int)II.bottomlineRight[col];
                    if (abs(now) > 5)
                    {
                        if (flag == 1) // 前一高度是扫到线的
                        {
                            flag = 0;          // 置0，说明这次没有扫到线
                            if (BR.segCnt > 0) // 如果不是第一次断线（我觉得这个判定去掉好像没啥事，先放着吧）
                            {
                                BR.end[BR.segCnt - 1] = col;
                                BR.numCnt[BR.segCnt - 1] = BR.end[BR.segCnt - 1] - BR.start[BR.segCnt - 1] + 1; // 算下线段点数
                            }
                        }
                    }
                }
            }
        }
        for (uint8 k = 0; k < HR.segCnt; k++)
        {
            int now = II.highlineRight[HR.start[k]] - II.highlineRight[HR.start[k] + 1];
            if (abs(now) > 3)
            {
                HR.start[k]++;
                HR.numCnt[k]--;
            }
            now = II.highlineRight[HR.end[k]] - II.highlineRight[HR.end[k] - 1];
            if (abs(now) > 3)
            {
                HR.end[k]--;
                HR.numCnt[k]--;
            }
        }
        for (uint8 k = 0; k < BR.segCnt; k++)
        {
            int now = II.bottomlineRight[BR.start[k]] - II.bottomlineRight[BR.start[k] + 1];
            if (abs(now) > 3)
            {
                BR.start[k]++;
                BR.numCnt[k]--;
            }
            now = II.bottomlineRight[BR.end[k]] - II.bottomlineRight[BR.end[k] - 1];
            if (abs(now) > 3)
            {
                BR.end[k]--;
                BR.numCnt[k]--;
            }
        }
    }
}

/*************************************************
Function: getKeyPoints()
Description: 获取每段线的关键点
Input: null
Output: null
Return: null
Others: 实用价值不很高，但有时还是比较好用的
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void getKeyPoints_L()
{
    for (uint8 i = 0; i < LI.segCnt; i++)
    {
        uint8 left = XX, right = 0;
        PL.right_y[i] = PL.left_y[i] = LI.start[i];
        for (uint8 j = LI.start[i]; j <= LI.end[i]; j++)
        {
            if (II.leftline[j] <= left && j < PL.left_y[i] + 5)
            {
                left = II.leftline[j];
                PL.left_x[i] = left;
                PL.left_y[i] = j;
            }
            if (II.leftline[j] >= right && j < PL.right_y[i] + 5)
            {
                right = II.leftline[j];
                PL.right_x[i] = right;
                PL.right_y[i] = j;
            }
        }
    }
}

void getKeyPoints_R()
{
    for (uint8 i = 0; i < RI.segCnt; i++)
    {
        uint8 left = XX, right = 0;
        PR.right_y[i] = PR.left_y[i] = RI.start[i];
        for (uint8 j = RI.start[i]; j <= RI.end[i]; j++)
        {
            if (II.rightline[j] <= left && j < PR.left_y[i] + 5)
            {
                left = II.rightline[j];
                PR.left_x[i] = left;
                PR.left_y[i] = j;
            }
            if (II.rightline[j] >= right && j < PR.right_y[i] + 5)
            {
                right = II.rightline[j];
                PR.right_x[i] = right;
                PR.right_y[i] = j;
            }
        }
    }
}

/*
 * 搜图部分，算法部分大体用BOOM4的为主，BOOM5的基本跑不动，不过也有所借鉴
 */
/*搜索basemap*/
void searchimg(uint8 x, uint8 y)
{
    Stack s;
    Point data[Stack_Size];
    InitStack(&s, Stack_Size, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (allmap[a.y][a.x] == PIXEL_WHITE)
        {
            if (basemap[a.y][a.x] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y][a.x] = 0;
                basemap[a.y][a.x] = 0;
                if (II.top < a.y)
                    II.top = a.y;
            }
            if (a.x > 0 && basemap[a.y][a.x - 1] == 1)
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && basemap[a.y][a.x + 1] == 1)
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && basemap[a.y - 1][a.x] == 1)
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && basemap[a.y + 1][a.x] == 1)
                PushStack(&s, a.x, a.y + 1);

            else
                PopStack(&s);
        }
        else
        {
            basemap[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

/*
 * 加速搜basemap的函数，但是会导致basemap中边界出问题
 * 不过对搜索leftmap和rightmap的搜索无影响，今年不打算使用，后续可以修改修改用
 */
void searchimg_accelerated(uint8 x, uint8 y)
{
    Stack s;
    Point data[Stack_Size];
    InitStack(&s, Stack_Size, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (allmap[a.y][a.x] == PIXEL_WHITE)
        {
            //(X,Y)
            if (basemap[a.y][a.x] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y][a.x] = 0;
                basemap[a.y][a.x] = 0;
                if (II.top < a.y)
                    II.top = a.y;
            }
            //(X+1,Y)
            if (a.x < XX && allmap[a.y][a.x + 1] == PIXEL_WHITE && basemap[a.y][a.x + 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y][a.x + 1] = 0;
                basemap[a.y][a.x + 1] = 0;
            }
            //(X+1,Y+1)
            if (a.x < XX && a.y < YY && allmap[a.y + 1][a.x + 1] == PIXEL_WHITE && basemap[a.y + 1][a.x + 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y + 1][a.x + 1] = 0;
                basemap[a.y + 1][a.x + 1] = 0;
                if (II.top < a.y + 1)
                    II.top = a.y + 1;
            }
            //(X,Y+1)
            if (a.y < YY && allmap[a.y + 1][a.x] == PIXEL_WHITE && basemap[a.y + 1][a.x] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y + 1][a.x] = 0;
                basemap[a.y + 1][a.x] = 0;
                if (II.top < a.y + 1)
                    II.top = a.y + 1;
            }
            //(X-1,Y+1)
            if (a.x > 0 && a.y < YY && allmap[a.y + 1][a.x - 1] == PIXEL_WHITE && basemap[a.y + 1][a.x - 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y + 1][a.x - 1] = 0;
                basemap[a.y + 1][a.x - 1] = 0;
                if (II.top < a.y + 1)
                    II.top = a.y + 1;
            }
            //(X-1,Y)
            if (a.x > 0 && allmap[a.y][a.x - 1] == PIXEL_WHITE && basemap[a.y][a.x - 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y][a.x - 1] = 0;
                basemap[a.y][a.x - 1] = 0;
            }
            //(X-1,Y-1)
            if (a.x > 0 && a.y > 0 && allmap[a.y - 1][a.x - 1] == PIXEL_WHITE && basemap[a.y - 1][a.x - 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y - 1][a.x - 1] = 0;
                basemap[a.y - 1][a.x - 1] = 0;
            }
            //(X,Y-1)
            if (a.y > 0 && allmap[a.y - 1][a.x] == PIXEL_WHITE && basemap[a.y - 1][a.x] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y - 1][a.x] = 0;
                basemap[a.y - 1][a.x] = 0;
            }
            //(X+1,Y-1)
            if (a.x < XX && a.y > 0 && allmap[a.y - 1][a.x + 1] == PIXEL_WHITE && basemap[a.y - 1][a.x + 1] == 1)
            {
                ++II.bnum_all;
                insidemap[a.y - 1][a.x + 1] = 0;
                basemap[a.y - 1][a.x + 1] = 0;
            }

            if (a.x > 1 && basemap[a.y][a.x - 2] == 1)
                PushStack(&s, a.x - 2, a.y);
            else if (a.x < XX - 1 && basemap[a.y][a.x + 2] == 1)
                PushStack(&s, a.x + 2, a.y);
            else if (a.y > 1 && basemap[a.y - 2][a.x] == 1)
                PushStack(&s, a.x, a.y - 2);
            else if (a.y < YY - 1 && basemap[a.y + 2][a.x] == 1)
                PushStack(&s, a.x, a.y + 2);
            else
                PopStack(&s);
        }
        else
        {
            basemap[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

/*
 * uint8 getDown()
 * 环岛状态中修改一次搜图的起始纵坐标最高高度限制
 */
uint8 getDown()
{
    if (!outGarageFlag)
        return 30;
    if (IF.annulus)
    {
        if (IF.annulus == AL1 || IF.annulus == AR1)
            return 30;
        if (IF.annulus == AR2)
        {
            uint8 min = getMapYMin_Col2(XX, 0, basemap);
            if (min > 15)
                for (uint8 i = XX; i > XX - 20; --i)
                    if (basemap[min - 1][i] == 2)
                        return min - 3;
            return min - 1;
        }
        else if (IF.annulus == AL2)
        {
            uint8 min = getMapYMin_Col2(0, 0, basemap);
            if (min > 15)
                for (uint8 i = 0; i < 20; ++i)
                    if (basemap[min - 1][i] == 2)
                        return min - 3;
            return min - 1;
        }
        else if (IF.annulus != AR1 && IF.annulus != AL1 && IF.annulus != AR3 && IF.annulus != AL3)
            return 30;
        return YM;
    }
    if (IF.garage == 1 || IF.garage == 3)
        return getMapYMax_Col(0, basemap, 0) - 1;
    else if (IF.garage == 2 || IF.garage == 4)
        return getMapYMax_Col(XX, basemap, 0) - 1;
    return STEP1;
}

/*搜索左右图*/
void searchLeftAndRightMap()
{
    // 这里开始一次搜图，起始点的高度由STEP1限制住，一次搜图左右各图最多搜1个连通域
    for (int i = 0; i < II.step; ++i)
    {
        if (basemap[i][0] && leftmap[i][0] != 1 && rightmap[i][0] != 1 && II.num_lm < 2 && IF.garage != 3 && IF.garage != 5)
        {
            searchleftmap(0, (uint8)i);
            II.start_lm[II.num_lm] = (uint8)i; // 递归入口
            II.lnum_all += II.lnum[II.num_lm];
            ++II.num_lm;
        }
        if (basemap[i][XX] && leftmap[i][XX] != 1 && rightmap[i][XX] != 1 && II.num_rm < 2 && IF.garage != 4 && IF.garage != 6)
        {
            searchrightmap(XX, (uint8)i);
            II.start_rm[II.num_rm] = (uint8)i; // 递归入口
            II.rnum_all += II.rnum[II.num_rm];
            ++II.num_rm;
        }
    }
}

#define searchleftmapPoint(x, y)                              \
    {                                                         \
        if (x > RB_X(II.num_lm))                              \
        {                                                     \
            RB_X(II.num_lm) = x;                              \
            RB_Y(II.num_lm) = y;                              \
            RT_X(II.num_lm) = x;                              \
            RT_Y(II.num_lm) = y;                              \
        }                                                     \
        else if (x == RB_X(II.num_lm))                        \
        {                                                     \
            if (y < RB_Y(II.num_lm))                          \
            {                                                 \
                RB_Y(II.num_lm) = y;                          \
            }                                                 \
            else if (y > RT_Y(II.num_lm))                     \
            {                                                 \
                RT_Y(II.num_lm) = y;                          \
            }                                                 \
        }                                                     \
        if (y < BR_Y(II.num_lm))                              \
        {                                                     \
            BR_X(II.num_lm) = x;                              \
            BR_Y(II.num_lm) = y;                              \
        }                                                     \
        else if (y == BR_Y(II.num_lm) && x > BR_X(II.num_lm)) \
        {                                                     \
            BR_X(II.num_lm) = x;                              \
        }                                                     \
        if (y > TR_Y(II.num_lm))                              \
        {                                                     \
            TR_X(II.num_lm) = x;                              \
            TR_Y(II.num_lm) = y;                              \
        }                                                     \
        else if (y == TR_Y(II.num_lm) && x > TR_X(II.num_lm)) \
        {                                                     \
            TR_X(II.num_lm) = x;                              \
        }                                                     \
    }

#define searchleftPoint(x, y, r)                                  \
    {                                                             \
        if (x > r->right_bottom.x)                                \
        {                                                         \
            r->right_bottom.x = x;                                \
            r->right_bottom.y = y;                                \
            r->right_top.x = x;                                   \
            r->right_top.y = y;                                   \
        }                                                         \
        else if (x == r->right_bottom.x)                          \
        {                                                         \
            if (y < r->right_bottom.y)                            \
            {                                                     \
                r->right_bottom.y = y;                            \
            }                                                     \
            else if (y > r->right_top.y)                          \
            {                                                     \
                r->right_top.y = y;                               \
            }                                                     \
        }                                                         \
        if (y < r->bottom_right.y)                                \
        {                                                         \
            r->bottom_right.x = x;                                \
            r->bottom_right.y = y;                                \
        }                                                         \
        else if (y == r->bottom_right.y && x > r->bottom_right.x) \
        {                                                         \
            r->bottom_right.x = x;                                \
        }                                                         \
        if (y > r->top_right.y)                                   \
        {                                                         \
            r->top_right.x = x;                                   \
            r->top_right.y = y;                                   \
        }                                                         \
        else if (y == r->top_right.y && x > r->top_right.x)       \
        {                                                         \
            r->top_right.x = x;                                   \
        }                                                         \
    }

#define searchrightPoint(x, y, r)                               \
    {                                                           \
        if (x < r->left_bottom.x)                               \
        {                                                       \
            r->left_bottom.x = x;                               \
            r->left_bottom.y = y;                               \
            r->left_top.x = x;                                  \
            r->left_top.y = y;                                  \
        }                                                       \
        else if (x == r->left_bottom.x)                         \
        {                                                       \
            if (y < r->left_bottom.y)                           \
            {                                                   \
                r->left_bottom.y = y;                           \
            }                                                   \
            else if (y > r->left_top.y)                         \
            {                                                   \
                r->left_top.y = y;                              \
            }                                                   \
        }                                                       \
        if (y < r->bottom_left.y)                               \
        {                                                       \
            r->bottom_left.x = x;                               \
            r->bottom_left.y = y;                               \
        }                                                       \
        else if (y == r->bottom_left.y && x < r->bottom_left.x) \
        {                                                       \
            r->bottom_left.x = x;                               \
        }                                                       \
        if (y > r->top_left.y)                                  \
        {                                                       \
            r->top_left.x = x;                                  \
            r->top_left.y = y;                                  \
        }                                                       \
        else if (y == r->top_left.y && x < r->top_left.x)       \
        {                                                       \
            r->top_left.x = x;                                  \
        }                                                       \
    }

#define searchrightmapPoint(x, y)                             \
    {                                                         \
        if (x < LB_X(II.num_rm))                              \
        {                                                     \
            LB_X(II.num_rm) = x;                              \
            LB_Y(II.num_rm) = y;                              \
            LT_X(II.num_rm) = x;                              \
            LT_Y(II.num_rm) = y;                              \
        }                                                     \
        else if (x == LB_X(II.num_rm))                        \
        {                                                     \
            if (y < LB_Y(II.num_rm))                          \
            {                                                 \
                LB_Y(II.num_rm) = y;                          \
            }                                                 \
            else if (y > LT_Y(II.num_rm))                     \
            {                                                 \
                LT_Y(II.num_rm) = y;                          \
            }                                                 \
        }                                                     \
        if (y < BL_Y(II.num_rm))                              \
        {                                                     \
            BL_X(II.num_rm) = x;                              \
            BL_Y(II.num_rm) = y;                              \
        }                                                     \
        else if (y == BL_Y(II.num_rm) && x < BL_X(II.num_rm)) \
        {                                                     \
            BL_X(II.num_rm) = x;                              \
        }                                                     \
        if (y > TL_Y(II.num_rm))                              \
        {                                                     \
            TL_X(II.num_rm) = x;                              \
            TL_Y(II.num_rm) = y;                              \
        }                                                     \
        else if (y == TL_Y(II.num_rm) && x < TL_X(II.num_rm)) \
        {                                                     \
            TL_X(II.num_rm) = x;                              \
        }                                                     \
    }

void getRegionInfoLeft(uint8 x, uint8 y, uint8 src[][XM],
                       uint8 dst[][XM], RegionInfoLeft *r)
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        x = a.x;
        y = a.y;
        if (src[a.y][a.x])
        {
            if (dst[a.y][a.x] == 0)
            {
                ++r->numCnt;
                searchleftPoint(x, y, r);
                dst[a.y][a.x] = 1;
            }
            if (a.x > 0 && !dst[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !dst[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && !dst[a.y - 1][a.x])
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && !dst[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);

            else
                PopStack(&s);
        }
        else
        {
            dst[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

void getRegionInfoRight(uint8 x, uint8 y, uint8 src[][XM],
                        uint8 dst[][XM], RegionInfoRight *r)
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        x = a.x;
        y = a.y;
        if (src[a.y][a.x])
        {
            if (dst[a.y][a.x] == 0)
            {
                ++r->numCnt;
                searchrightPoint(x, y, r);
                dst[a.y][a.x] = 1;
            }
            if (a.x > 0 && !dst[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !dst[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && !dst[a.y - 1][a.x])
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && !dst[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);

            else
                PopStack(&s);
        }
        else
        {
            dst[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

/*防噪点搜左图，今年没有使用，后续如果图像阴影严重，可以考虑使用，不过阴影严重，首先调摄像头增肌，就是gain，我们写到菜单里可以调的*/
void searchleftmapRemoveNoise(uint8 x, uint8 y, uint16 cntMin)
{
    if (basemap[y][x])
    {
        ++II.lnum[0];
        if (II.lnum[0] >= cntMin + 3) // 排除噪点和加快算法效率用的     +3为了避免提前出来
            return;
        leftmap[y][x] = 1;
        if (x > 0 && 0 == leftmap[y][x - 1])
            searchleftmapRemoveNoise(x - 1, y, cntMin);
        if (x < XX && 0 == leftmap[y][x + 1])
            searchleftmapRemoveNoise(x + 1, y, cntMin);
        if (y < YY && 0 == leftmap[y + 1][x])
            searchleftmapRemoveNoise(x, y + 1, cntMin);
        if (y > 0 && 0 == leftmap[y - 1][x])
            if (y <= II.top)
                searchleftmapRemoveNoise(x, y - 1, cntMin);
    }
    else
        leftmap[y][x] = 2; // 白色在边界数组中记为2，且退出迭代
}

void searchrightmapRemoveNoise(uint8 x, uint8 y, uint16 cntMin)
{
    if (basemap[y][x])
    {
        ++II.rnum[0];

        if (II.rnum[0] >= cntMin + 3) // 排除噪点和加快算法效率用的   +3为了避免提前出来
            return;
        rightmap[y][x] = 1;

        if (x < XX && 0 == rightmap[y][x + 1])
            searchrightmapRemoveNoise(x + 1, y, cntMin);
        if (x > 0 && 0 == rightmap[y][x - 1])
            searchrightmapRemoveNoise(x - 1, y, cntMin);
        if (y < YY && 0 == rightmap[y + 1][x])
            searchrightmapRemoveNoise(x, y + 1, cntMin);
        if (y > 0 && 0 == rightmap[y - 1][x])
            if (y <= II.top)
                searchrightmapRemoveNoise(x, y - 1, cntMin);
    }
    else
        rightmap[y][x] = 2; // 白色在边界数组中记为2，且退出迭代
}

void searchleftmap(uint8 x, uint8 y)
{
    Point a;
    a.x = x;
    a.y = y;
    Stack s;
    Point data[Stack_Size];
    InitStack(&s, Stack_Size, data);
    if (basemap[a.y][a.x])
        PushStack(&s, x, y);
    else
        leftmap[y][x] = 2;
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (leftmap[a.y][a.x] == 0)
            {
                searchleftmapPoint(a.x, a.y);
                leftmap[a.y][a.x] = 1;
                insidemap[a.y][a.x] = 0;
                if (rightmap[a.y][a.x])
                {
                    ++II.repeatNum;
                }
                ++II.lnum[II.num_lm];
                if (a.y < STOP_LINE)
                    ++II.lnum_control;
            }

            if (a.x > 0 && !leftmap[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !leftmap[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y < YY && !leftmap[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);
            else if (a.y > 0 && !leftmap[a.y - 1][a.x])
            {
                if (a.y <= II.top - 3)
                    PushStack(&s, a.x, a.y - 1);
                else if (basemap[a.y - 1][a.x] && noDown.num < XM)
                {
                    //                    if(a.y-1 == II.top && basemap[a.y-1][a.x]==2) leftmap[a.y - 1][a.x] = 2;
                    noDown.point[noDown.num].x = a.x;
                    noDown.point[noDown.num].y = a.y - 1;
                    ++noDown.num;
                    PopStack(&s);
                }
                else if (!basemap[a.y - 1][a.x])
                    leftmap[a.y - 1][a.x] = 2;
                else
                    PopStack(&s);
            }
            else
                PopStack(&s);
        }
        else
        {
            leftmap[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

void searchrightmap(uint8 x, uint8 y)
{
    Point a;
    a.x = x;
    a.y = y;
    Stack s;
    Point data[Stack_Size];
    InitStack(&s, Stack_Size, data);
    if (basemap[a.y][a.x])
        PushStack(&s, x, y);
    else
        rightmap[y][x] = 2;
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (rightmap[a.y][a.x] == 0)
            {
                searchrightmapPoint(a.x, a.y);
                rightmap[a.y][a.x] = 1;
                insidemap[a.y][a.x] = 0;
                if (leftmap[a.y][a.x])
                {
                    ++II.repeatNum;
                }
                ++II.rnum[II.num_rm];
                if (a.y < STOP_LINE)
                    ++II.rnum_control;
            }
            if (a.x > 0 && !rightmap[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !rightmap[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y < YY && !rightmap[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);
            else if (a.y > 0 && !rightmap[a.y - 1][a.x])
            {
                if (a.y <= II.top)
                    PushStack(&s, a.x, a.y - 1);
                else if (basemap[a.y - 1][a.x] && noDown.num < XM)
                {
                    //                    if(a.y-1 == II.top && basemap[a.y-1][a.x]==2) rightmap[a.y - 1][a.x] = 2;
                    noDown.point[noDown.num].x = a.x;
                    noDown.point[noDown.num].y = a.y - 1;
                    ++noDown.num;
                    PopStack(&s);
                }
                else if (!basemap[a.y - 1][a.x])
                    rightmap[a.y - 1][a.x] = 2;
                else
                    PopStack(&s);
            }
            else
                PopStack(&s);
        }
        else
        {
            rightmap[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

uint16 cntMap(uint8 x, uint8 y)
{
    numCnt = 0;
    uint8 map[YM][XM];
    memset(map, 0, sizeof(map));
    searchCountmap(x, y, map);
    return numCnt;
}

void searchCountmap(uint8 x, uint8 y, uint8 src[][XM])
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (src[a.y][a.x] == 0)
            {
                ++numCnt;
                src[a.y][a.x] = 1;
            }
            if (a.x > 0 && !src[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !src[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && !src[a.y - 1][a.x])
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && !src[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);

            else
                PopStack(&s);
        }
        else
        {
            src[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

void searchmap(uint8 x, uint8 y, uint8 src[][XM])
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (src[a.y][a.x] == 0)
                src[a.y][a.x] = 1;
            if (a.x > 0 && !src[a.y][a.x - 1])
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !src[a.y][a.x + 1])
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && !src[a.y - 1][a.x])
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && !src[a.y + 1][a.x])
                PushStack(&s, a.x, a.y + 1);
            else
                PopStack(&s);
        }
        else
        {
            src[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

void Get_insideMap()
{
    if (II.bnum_all + II.lnum_all + II.rnum_all == XM * YM + II.repeatNum)
        return;

    for (uint8 i = 0; i < noDown.num; ++i)
    {
        if (leftmap[noDown.point[i].y][noDown.point[i].x] != 1 &&
            rightmap[noDown.point[i].y][noDown.point[i].x] != 1 &&
            !deletemap[noDown.point[i].y][noDown.point[i].x])
            searchdeletemap2(noDown.point[i].x, noDown.point[i].y);
    }
    for (uint8 i = 0; i < YM; ++i)
    {
        if (basemap[i][0] && !leftmap[i][0] && !rightmap[i][0] &&
            !deletemap[i][0])
            searchdeletemap(0, i);
        if (basemap[i][XX] && !leftmap[i][XX] && !rightmap[i][XX] &&
            !deletemap[i][XX])
            searchdeletemap(XX, i);
    }
    for (uint8 i = 0; i < XM; ++i)
        if (basemap[YY][i] && !leftmap[YY][i] && !rightmap[YY][i] &&
            !deletemap[YY][i])
            searchdeletemap(i, YY);
    II.inum_all = XM * YM + II.repeatNum - II.bnum_all - II.lnum_all - II.rnum_all - II.dnum_all;
}

void searchdeletemap(uint8 x, uint8 y)
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (deletemap[a.y][a.x] == 0)
            {
                ++II.dnum_all;
                deletemap[a.y][a.x] = 1;
                insidemap[a.y][a.x] = 0;
            }
            if (a.x > 0 && deletemap[a.y][a.x - 1] == 0)
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && deletemap[a.y][a.x + 1] == 0)
                PushStack(&s, a.x + 1, a.y);
            else if (a.y > 0 && deletemap[a.y - 1][a.x] == 0)
                PushStack(&s, a.x, a.y - 1);
            else if (a.y < YY && deletemap[a.y + 1][a.x] == 0)
                PushStack(&s, a.x, a.y + 1);
            else
                PopStack(&s);
        }
        else
        {
            if ((a.y <= II.d_bottom[0].y && a.x < II.d_bottom[0].x) || a.y < II.d_bottom[0].y)
            {
                II.d_bottom[0].y = a.y;
                II.d_bottom[0].x = a.x;
            }
            if ((a.y <= II.d_bottom[1].y && a.x > II.d_bottom[1].x) || a.y < II.d_bottom[1].y)
            {
                II.d_bottom[1].y = a.y;
                II.d_bottom[1].x = a.x;
            }
            if (IF.fork)
            {
                if (a.y < dbottomline[a.x])
                    dbottomline[a.x] = a.y;
            }
            deletemap[a.y][a.x] = 2;
            PopStack(&s);
        }
    }
}

void searchdeletemap2(uint8 x, uint8 y)
{
    Stack s;
    Point data[3000];
    InitStack(&s, 3000, data);
    PushStack(&s, x, y);
    while (!EmptyStack(&s))
    {
        Point a;
        a = s.data[s.top];
        if (basemap[a.y][a.x])
        {
            if (deletemap[a.y][a.x] == 0)
            {
                ++II.dnum_all;
                ++II.dnum_top;
                deletemap[a.y][a.x] = 254;
                insidemap[a.y][a.x] = 0;
            }
            if (a.x > 0 && !deletemap[a.y][a.x - 1] && leftmap[a.y][a.x - 1] != 1 &&
                rightmap[a.y][a.x - 1] != 1)
                PushStack(&s, a.x - 1, a.y);
            else if (a.x < XX && !deletemap[a.y][a.x + 1] && leftmap[a.y][a.x + 1] != 1 &&
                     rightmap[a.y][a.x + 1] != 1)
                PushStack(&s, a.x + 1, a.y);
            else if (a.y < YY && !deletemap[a.y + 1][a.x] && leftmap[a.y + 1][a.x] != 1 &&
                     rightmap[a.y + 1][a.x] != 1)
                PushStack(&s, a.x, a.y + 1);
            else if (a.y > 0 && !deletemap[a.y - 1][a.x] && leftmap[a.y - 1][a.x] != 1 &&
                     rightmap[a.y - 1][a.x] != 1)
                PushStack(&s, a.x, a.y - 1);

            else
                PopStack(&s);
        }
        else
        {
            if ((a.y <= II.d_bottom[0].y && a.x < II.d_bottom[0].x) || a.y < II.d_bottom[0].y)
            {
                II.d_bottom[0].y = a.y;
                II.d_bottom[0].x = a.x;
            }
            if ((a.y <= II.d_bottom[1].y && a.x > II.d_bottom[1].x) || a.y < II.d_bottom[1].y)
            {
                II.d_bottom[1].y = a.y;
                II.d_bottom[1].x = a.x;
            }
            if (IF.fork)
            {
                if (a.y < dbottomline[a.x])
                    dbottomline[a.x] = a.y;
            }
            deletemap[a.y][a.x] = 253;
            PopStack(&s);
        }
    }
}

/*************************************************
Function: getSpeedTop()
Description: 获取速度参数speedtop
Input: null
Output: null
Return: null
Others: 用于速度规划
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void getSpeedTop()
{
    for (uint8 i = 0; i < YM; ++i)
        if (basemap[i][myCarMid] && !insidemap[i][myCarMid]) // 可以无视掉起跑线的
        {
            II.speedTop = i;
            break;
        }
}

void regetSpeedTop()
{
    for (uint8 i = 0; i < YM; ++i)
        if ((leftmap[i][myCarMid] || rightmap[i][myCarMid]) && !insidemap[i][myCarMid])
        {
            II.speedTop = i;
            break;
        }
}

/*************************************************
Function: uint8 getLeftDownCorner() uint8 getRightDownCorner()
Description: 祖传左右下角点检测
Input: null
Output: 左右下角点以及角点处的角度
Return: null
Others: 新加余弦定理角度计算
Author: ZJUT    BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getLeftDownCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "getLeftDownCorner";
#endif
    if (!IF.ramp && !(IF.annulusDelay || IF.annulus == AL5 || IF.annulus == AR5))
    {
        for (uint8 n = 0; n < II.num_lm; ++n)
        {
            if (BOTTOM_LM(n) == 0)
                continue;
            Point bottom_right;
            bottom_right.x = BR_X(n);
            bottom_right.y = BR_Y(n) - 1;
            Point last[100];
            uint8 num = 0;
            if (leftmap[bottom_right.y][0] == 2)
            {
                leftmap[bottom_right.y][0] = 0;
                last[num].x = 0;
                last[num].y = bottom_right.y;
                ++num;
            }
            for (uint8 j = bottom_right.y; j < bottom_right.y + 8; ++j)
            {
                for (uint8 i = 0, flagLeft = 0, flagRight = 0; i < XM; ++i)
                {
                    if (!flagLeft && bottom_right.x >= i)
                        if (leftmap[j][bottom_right.x - i] == 2)
                            leftmap[j][bottom_right.x - i] = 0;
                    if (!flagRight && bottom_right.x + i <= XX &&
                        leftmap[j][bottom_right.x + i])
                        if (leftmap[j][bottom_right.x + i] == 2)
                        {
                            leftmap[j][bottom_right.x + i] = 0;
                            if (leftmap[j][bottom_right.x + i + 1] == 0)
                            {
                                last[num].x = bottom_right.x + i;
                                last[num].y = j;
                                ++num;
                            }
                        }
                }
            }
            for (int i = num - 1; i >= 0; --i)
                if ((last[i].y + 1 < YM &&
                     leftmap[last[i].y + 1][last[i].x] == 2) ||
                    (last[i].y + 1 < YM && last[i].x + 1 < XM &&
                     leftmap[last[i].y + 1][last[i].x + 1] == 2) ||
                    (last[i].x + 1 < XM &&
                     leftmap[last[i].y][last[i].x + 1] == 2) ||
                    (last[i].y + 1 < YM && last[i].x >= 1 &&
                     leftmap[last[i].y + 1][last[i].x - 1] == 2) ||
                    (last[i].x >= 1 &&
                     leftmap[last[i].y][last[i].x - 1] == 2) ||
                    (last[i].y >= 1 &&
                     leftmap[last[i].y - 1][last[i].x] == 2) ||
                    (last[i].y >= 1 && last[i].x + 1 < XM &&
                     leftmap[last[i].y - 1][last[i].x + 1] == 2) ||
                    (last[i].y >= 1 && last[i].x >= 1 &&
                     leftmap[last[i].y - 1][last[i].x - 1] == 2))
                    leftmap[last[i].y][last[i].x] = 2;
        }
    }
    if (!II.num_lm || II.lnum_all < 100 || II.start_lm[0] > 15)
        return 0;

    uint8 width = 5;
    uint8 rightX = 255;      // 对上去之后的点的横坐标
    uint8 rightTop = 255;    // 对上去之后的点的纵坐标
    uint8 rightBottom = 255; // 左图最下面最靠右的紧贴边界点的点的y值
    uint8 leftX = 255;
    uint8 leftTop = 255;
    uint8 leftBottom = 255;
    for (uint8 i = 0, flag = 0; i < XM; ++i)
    {
        for (uint8 j = 0; j <= II.top; ++j)
            if (leftmap[j][XX - i] == 1)
            {
                rightBottom = j;
                for (uint8 k = j + 1; k <= II.top; ++k) // 找到之后向上对过去
                    if (leftmap[k][XX - i] != 1 && (k + width < YM && leftmap[k + width][XX - i] != 1))
                    {
                        // width用于规避对到最高点太高的情况
                        rightX = XX - i;
                        rightTop = k - 1;
                        flag = 1;
                        break;
                    }
                break;
            }
        if (flag == 1)
            break;
    }
    if (leftmap[rightTop + 1][rightX] == 2 && leftmap[rightTop - 1][rightX] == 2)
        for (uint8 row = rightTop; row < YM; row++)
            if (leftmap[row + 1][rightX - 1] == 2)
            {
                rightX = rightX - 1;
                rightTop = row;
                break;
            }
    if (rightTop >= 50)
        return 0;

#ifdef BOOM7_QT_DEBUG_CORNER
    qout << "enter 1";
#endif
    uint16 pointnum = 0;
    uint16 mynum = 0;
    if (rightX == 255 || rightTop == 255 || rightBottom == 255 ||
        rightTop > rightBottom + 20)
        /*这种情况多半就是对到图像顶端都没找到一个边界点*/
        return 0;
    for (uint8 i = rightTop; i < YM; ++i)
    {
        // 如果对过去已经是右图了
        if (rightmap[i][rightX] == 1)
            return 0;
        if (i > rightTop + (YM - rightTop) / 1.2)
            break;
    }
    if (IF.annulus == AL1 || IF.garage)
    {
        return 0;
    }
#ifdef BOOM7_QT_DEBUG_CORNER
    qout << "enter 2";
#endif
    do
    {
        if (rightX < width)
        {
            break;
        }
        leftX = rightX - width;
        for (uint8 j = 0; j <= II.top; ++j)
            if (leftmap[j][leftX] == 1)
            {
                leftBottom = j;
                for (uint8 k = j + 1; k <= II.top; ++k)
                    if (leftmap[k][leftX] != 1)
                    {
                        leftTop = k - 1;
                        break;
                    }
                break;
            }
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "leftX" << leftX << "leftBottom" << leftBottom << "leftTop" << leftTop;
        qout << "rightX" << rightX << "rightTop" << rightTop << "rightBottom" << rightBottom;
#endif
        if (leftBottom == 255 || leftTop == 255 || leftTop > leftBottom + 40 || rightTop < 10)
            return 0;
        // 面积判定,我稍微加一点
        // 祖传整个矩形直接判，我是把大矩形分成上下两半分别判,
        //         祖传的
        for (uint8 j = leftBottom; j <= leftTop; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (leftmap[j][i] == 1)
                    ++pointnum;
        mynum = abs((int)leftTop - (int)leftBottom + 1) * (rightX - leftX + 1) / 2;
        mynum += abs((int)leftTop - (int)leftBottom + 1) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "all" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        if (mynum <= pointnum)
            return 0;

        // 我的增加
        // 数上半矩形区域内实际的点数
        pointnum = 0;
        for (uint8 j = rightTop; j <= leftTop; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (leftmap[j][i] == 1)
                    ++pointnum;
        // 我们这里希望图像中在角点处呈现三角形特征，设定一个预期点数，就是矩形区域面积的一半
        mynum = abs((int)leftTop - (int)rightTop + 1) * (rightX - leftX + 1) / 2;
        // 加点裕量，不然太严了，裕量设置为矩形区域对角线长×一个系数，自己调，可以调角点判定的松紧的
        mynum += distance(leftX, leftTop, rightX, rightTop) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "up" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        // 预期点数小于实际点数，说明不对，不像三角形
        if (mynum <= pointnum)
            return 0;

        // 下半同理
        pointnum = 0;
        for (uint8 j = leftBottom; j <= rightBottom; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (leftmap[j][i] == 1)
                    ++pointnum;
        mynum = abs((int)rightBottom - (int)leftBottom + 1) * (rightX - leftX + 1) / 2;
        mynum += distance(leftX, leftBottom, rightX, rightBottom) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "down" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        if (mynum <= pointnum)
            return 0;

#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "enter 3";
#endif

        II.right_x = rightX; // 十字下角点
        II.right_y = rightTop;
        // 点数过少不算角度
        uint8 threshold = 100;
        if (IF.crossroad)
            threshold = 50;
        if (II.lnum_all < threshold)
            return 0;
        // 计算角点处角度
        Point v, p1, p2;
        // 顶点就是角点
        v.x = II.right_x;
        v.y = II.right_y;
        // 从顶点往左数最多8行
        uint8 limit = v.x > 8 ? 8 : v.x;
        uint8 limit_up = (YY - v.y - 10) > 15 ? 15 : (YY - v.y - 10);
        Point last_p2 = v;
        uint8 start = v.x > 4 ? 4 : v.x;
        last_p2.y = v.y - 5;
        for (uint8 i = start; i < limit; i++)
        {
            for (uint8 row = v.y - 5; row < v.y + limit_up + 5 && row < YM; row++)
            {
                if (leftmap[row][v.x - i] && !leftmap[row + 1][v.x - i] && row >= last_p2.y)
                {
                    p2.y = row - 1;
                    p2.x = v.x - i;
                    last_p2 = p2;
                }
            }
        }
        p1.x = BR_X(0);
        p1.y = BR_Y(0);
        II.angleL = getRealAngle(v, p1, p2);
#ifdef BOOM7_QT_DEBUG
        v_x_left = v.x;
        v_y_left = v.y;
        p1_x_left = p1.x;
        p1_y_left = p1.y;
        p2_x_left = p2.x;
        p2_y_left = p2.y;
#endif
    } while (0);

    return 0;
}

uint8 getRightDownCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "getLeftDownCorner";
#endif
    if (!IF.ramp && !((IF.annulusDelay || IF.annulus == AL5 || IF.annulus == AR5) &&
                      II.num_lm + II.num_rm == 1))
    {
        for (uint8 n = 0; n < II.num_rm; ++n)
        {
            if (BOTTOM_RM(n) == 0)
                continue;
            Point bottom_left;
            bottom_left.x = BL_X(n);
            bottom_left.y = BL_Y(n) - 1;
            Point last[100];
            uint8 num = 0;
            if (rightmap[bottom_left.y][XX] == 2)
            {
                rightmap[bottom_left.y][XX] = 0;
                last[num].x = XX;
                last[num].y = bottom_left.y;
                ++num;
            }
            for (uint8 j = bottom_left.y; j < bottom_left.y + 8; ++j)
            {
                for (uint8 i = 0, flagRight = 0, flagLeft = 0; i < XM; ++i)
                {
                    if (!flagRight && bottom_left.x + i <= XX)
                        if (rightmap[j][bottom_left.x + i] == 2)
                            rightmap[j][bottom_left.x + i] = 0;
                    if (!flagLeft && bottom_left.x >= i &&
                        rightmap[j][bottom_left.x - i])
                        if (rightmap[j][bottom_left.x - i] == 2)
                        {
                            rightmap[j][bottom_left.x - i] = 0;
                            if (rightmap[j][bottom_left.x - i - 1] == 0)
                            {
                                last[num].x = bottom_left.x - i;
                                last[num].y = j;
                                ++num;
                            }
                        }
                }
            }
            for (int i = num - 1; i >= 0; --i)
            {
                if ((last[i].y + 1 < YM &&
                     rightmap[last[i].y + 1][last[i].x] == 2) ||
                    (last[i].y + 1 < YM && last[i].x >= 1 &&
                     rightmap[last[i].y + 1][last[i].x - 1] == 2) ||
                    (last[i].x >= 1 &&
                     rightmap[last[i].y][last[i].x - 1] == 2) ||
                    (last[i].y + 1 < YM && last[i].x + 1 < XM &&
                     rightmap[last[i].y + 1][last[i].x + 1] == 2) ||
                    (last[i].x + 1 < XM &&
                     rightmap[last[i].y][last[i].x + 1] == 2) ||
                    (last[i].y >= 1 &&
                     rightmap[last[i].y - 1][last[i].x] == 2) ||
                    (last[i].y >= 1 && last[i].x + 1 < XM &&
                     rightmap[last[i].y - 1][last[i].x + 1] == 2) ||
                    (last[i].y >= 1 && last[i].x >= 1 &&
                     rightmap[last[i].y - 1][last[i].x - 1] == 2))
                    rightmap[last[i].y][last[i].x] = 2;
            }
        }
    }
    if (!II.num_rm || II.rnum_all < 100 || II.start_rm[0] > 15)
        return 0;

    uint8 width = 5;
    uint8 leftX = 255;
    uint8 leftTop = 255;
    uint8 leftBottom = 255;
    uint8 rightX = 255;
    uint8 rightTop = 255;
    uint8 rightBottom = 255;
    for (uint8 i = 0, flag = 0; i < XM; ++i) // 找到右图最左边的最下面和最上面的点
    {
        for (uint8 j = 0; j <= II.top; ++j)
        {
            if (rightmap[j][i] == 1)
            {
                leftBottom = j;
                for (uint8 k = j + 1; k <= II.top; ++k)
                {
                    if (rightmap[k][i] != 1 && (k + width < YM && rightmap[k + width][i] != 1))
                    {
                        leftX = i;
                        leftTop = k - 1;
                        flag = 1;
                        break;
                    }
                }
                break;
            }
        }
        if (flag == 1)
            break;
    }
    if (rightmap[leftTop + 1][leftX] == 2 && rightmap[leftTop - 1][leftX] == 2)
        for (uint8 row = leftTop; row < YM; row++)
            if (rightmap[row + 1][leftX + 1] == 2)
            {
                leftX = leftX + 1;
                leftTop = row;
                break;
            }
    if (leftTop >= 50)
        return 0;

    uint16 pointnum = 0;
    uint16 mynum = 0;
    if (leftX == 255 || leftTop == 255 || leftBottom == 255 ||
        leftTop > leftBottom + 15)
        return 0;
    for (uint8 i = leftTop; i < YM; ++i)
    {
        if (leftmap[i][leftX] == 1)
            return 0;
        if (i > leftTop + (YM - leftTop) / 1.2)
            break;
    }
    if (IF.annulus == AR1 || IF.garage)
    {
        return 0;
    }

    do
    {
        if (leftX + width > XX)
        {
            break;
        }
        rightX = leftX + width;
        for (uint8 j = 0; j <= II.top; ++j)
            if (rightmap[j][rightX] == 1)
            {
                rightBottom = j;
                for (uint8 k = j + 1; k <= II.top; ++k)
                    if (rightmap[k][rightX] != 1)
                    {
                        rightTop = k - 1;
                        break;
                    }
                break;
            }
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "leftX" << leftX << "leftTop" << leftTop << "leftBottom" << leftBottom;
        qout << "rightX" << rightX << "rightTop" << rightTop << "rightBottom" << rightBottom;
#endif
        if (rightBottom == 255 || rightTop == 255 || rightTop > rightBottom + 40 || leftTop < 10)
            return 0;

        for (uint8 j = rightBottom; j <= rightTop; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (rightmap[j][i] == 1)
                    ++pointnum;
        mynum = abs((int)rightTop - (int)rightBottom + 1) * (rightX - leftX + 1) / 2;
        mynum += abs((int)rightTop - (int)rightBottom + 1) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "all" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        if (mynum <= pointnum)
            return 0;

        // 数矩形区域内实际的点数
        pointnum = 0;
        for (uint8 j = leftTop; j <= rightTop; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (rightmap[j][i] == 1)
                    ++pointnum;
        // 我们这里希望图像中在角点处呈现三角形特征，设定一个预期点数，就是矩形区域面积的一半
        mynum = abs((int)rightTop - (int)leftTop + 1) * (rightX - leftX + 1) / 2;
        // 加点裕量，不然太严了，裕量设置为矩形区域对角线长×一个系数，自己调，可以调角点判定的松紧的
        mynum += distance(leftX, leftTop, rightX, rightTop) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "up" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        if (mynum <= pointnum)
            return 0;
        // 下半同理
        pointnum = 0;
        for (uint8 j = rightBottom; j <= leftBottom; ++j)
            for (uint8 i = leftX; i <= rightX; ++i)
                if (rightmap[j][i] == 1)
                    ++pointnum;
        mynum = abs((int)rightBottom - (int)leftBottom + 1) * (rightX - leftX + 1) / 2;
        mynum += distance(leftX, leftBottom, rightX, rightBottom) * 1.2 + 5;
#ifdef BOOM7_QT_DEBUG_CORNER
        qout << "down" << "mynum" << mynum << "pointnum" << pointnum;
#endif
        if (mynum <= pointnum)
            return 0;
        II.left_x = leftX;
        II.left_y = leftTop;
        // 点数过少不算角度
        uint8 threshold = 100;
        if (IF.crossroad)
            threshold = 50;
        if (II.rnum_all < threshold)
            return 0;
        Point v, p1, p2;
        // 顶点就是角点
        v.x = II.left_x;
        v.y = II.left_y;
        // 从顶点往右数最多8行
        uint8 limit = (XX - v.x) > 8 ? 8 : (XX - v.x);
        // 找点的时候设限，往上15以内
        uint8 limit_up = (YY - v.y - 10) > 15 ? 15 : (YY - v.y - 10);
        Point last_p2 = v;
        last_p2.y = v.y - 5;
        uint8 start = v.x > 4 ? 4 : v.x;
        for (uint8 i = start; i < limit; i++)
            for (uint8 row = v.y - 5; row < v.y + limit_up + 5 && row < YM; row++)
                if (rightmap[row][v.x + i] && !rightmap[row + 1][v.x + i] && row >= last_p2.y)
                {
                    p2.y = row - 1;
                    p2.x = v.x + i;
                    last_p2 = p2;
                }
        p1.x = BL_X(0);
        p1.y = BL_Y(0);
        II.angleR = getRealAngle(v, p1, p2);
#ifdef BOOM7_QT_DEBUG
        v_x_right = v.x;
        v_y_right = v.y;
        p1_x_right = p1.x;
        p1_y_right = p1.y;
        p2_x_right = p2.x;
        p2_y_right = p2.y;
#endif
    } while (0);

    return 0;
}

/*
 *                          工具函数
 */
/**************************General***************************/
/*
 * float get_k(int x1,int x2,int y1,int y2)
 * 工具函数，获得两点间k值
 */
float get_k(int x1, int x2, int y1, int y2)
{
    if (x1 == x2)
        return 10000;
    float k = 0;
    k = (float)(y1 - y2) / (float)(x1 - x2);
    if (k == 0)
        return 0;
    return k;
}

float distance(float x1, float y1, float x2, float y2)
{
    return sqrt((float)((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}

PointF getRealPoint(Point p)
{
    PointF ret;
    ret.x = ((float)p.x - leftline[p.y]) * k1[p.y];
    ret.y = k2[p.y];
#ifdef BOOM7_QT_DEBUG
    qout << "p x y:" << p.x << p.y << "p_real x y:" << ret.x << ret.y;
#endif
    return ret;
}

float getRealK(Point p1, Point p2)
{
    float k;
    PointF p1_real = getRealPoint(p1);
    PointF p2_real = getRealPoint(p2);
    k = (float)(p1_real.y - p2_real.y) / (float)(p1_real.x - p2_real.x);
    return k;
}

/*************************************************
Function: float getRealAngle(Point vertex, Point point1, Point point2)
Description: 求真实夹角，三叉和`十字这样比较好区分
Input: 顶点和两条边上任意一点
Output: null
Return: 角度大小（0到180）
Others: 主要用到余弦定理,想要精确的话要保证逆透视系数k2准确
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 zoom = 92;
float getRealAngle(Point vertex, Point point1, Point point2)
{
    PointF vertex_real = getRealPoint(vertex);
    PointF p1_real = getRealPoint(point1);
    PointF p2_real = getRealPoint(point2);
    float dist1 = distance(vertex_real.x, vertex_real.y, p1_real.x, p1_real.y);
    float dist2 = distance(vertex_real.x, vertex_real.y, p2_real.x, p2_real.y);
    float dist3 = distance(p2_real.x, p2_real.y, p1_real.x, p1_real.y);
    float cos = (dist1 * dist1 + dist2 * dist2 - dist3 * dist3) / (2 * dist1 * dist2);
    uint8 ret = acos(cos) / PI * 180 * (float)zoom / 100;
#ifdef BOOM7_QT_DEBUG
    qout << cos;
    qout << "vertex:(" << vertex.x << vertex.y << ") point1:(" << point1.x << point1.y << ") point2:(" << point2.x << point2.y << ")" << "angle:(" << ret;
#endif
    if (isnan(ret))
        return 0;
    return ret;
}
/*************************************************
Function: getSegOfPoint(uint8 y,uint8 dir)
Description: 找y这个点在哪个线段上，找不到返回segCnt
Input: 纵坐标，在左线还是右线找
Output: null
Return: 第几段
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getSegOfPoint(uint8 y, uint8 dir)
{
    if (dir == goLeft)
    {
        for (uint8 i = 0; i < LI.segCnt; i++)
            if (y >= LI.start[i] && y <= LI.end[i])
                return i;
    }
    else if (dir == goRight)
    {
        for (uint8 i = 0; i < RI.segCnt; i++)
            if (y >= RI.start[i] && y <= RI.end[i])
                return i;
    }
#ifdef BOOM7_QT_DEBUG
    qout << "can not find the right seg, please check your code";
#endif
    return dir == goLeft ? LI.segCnt : RI.segCnt;
}

/*********************************************************
uint8 isSearch(uint8 x, uint8 y, uint8 map[][XM])
返回1情况：
1.basemap从该点往上有白点；
2.basemap从该点往上无白，且map中从该点往上无黑点（(x,y)上方的区域没被搜到map中就返回1）
也就是说有搜图必要就返回1
 **********************************************************/
uint8 isSearch(uint8 x, uint8 y, uint8 map[][XM])
{
    /*从（x,y）开始向上循环搜索，若搜到赛道内部点就返回1，
     若在basemap中属于赛道外部且map中也属于赛道外部则返回0*/
    for (uint8 k = y + 1; k < YM; ++k)
    {
        if (!basemap[k][x])
            break;
        if (map[k][x])
            return 0;
    }
    return 1;
}

/*************************************************
Function: Point getFirstBlackPoint(Point src,uint8 dir, uint8 range)
          Point getFirstWhitePoint(Point src,uint8 dir, uint8 range)
Description: 找src这个点在某方向上在某范围内遇到的第一个黑（白）点
Input: 起始点、方向、范围
Output: null
Return: 第一个黑点
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
Point getFirstBlackPoint(Point src, uint8 dir, uint8 range)
{
    Point ret = src;
    for (uint8 i = 0; i < range; i++)
    {
        if (dir == goUp)
        {
            if (basemap[src.y + i][src.x])
            {
                ret.x = src.x;
                ret.y = src.y + i;
                return ret;
            }
        }
        else if (dir == goDown)
        {
            if (basemap[src.y - i][src.x])
            {
                ret.x = src.x;
                ret.y = src.y - i;
                return ret;
            }
        }
        else if (dir == goLeft)
        {
            if (basemap[src.y][src.x - i])
            {
                ret.x = src.x - i;
                ret.y = src.y;
                return ret;
            }
        }
        else if (dir == goRight)
        {
            if (basemap[src.y][src.x + i])
            {
                ret.x = src.x + i;
                ret.y = src.y;
                return ret;
            }
        }
    }
    return ret;
}

Point getFirstWhitePoint(Point src, uint8 dir, uint8 range)
{
    Point ret = src;
    for (uint8 i = 0; i < range; i++)
    {
        if (dir == goUp)
        {
            if (!basemap[src.y + i][src.x])
            {
                ret.x = src.x;
                ret.y = src.y + i;
                return ret;
            }
        }
        else if (dir == goDown)
        {
            if (!basemap[src.y - i][src.x])
            {
                ret.x = src.x;
                ret.y = src.y - i;
                return ret;
            }
        }
        else if (dir == goLeft)
        {
            if (!basemap[src.y][src.x - i])
            {
                ret.x = src.x - i;
                ret.y = src.y;
                return ret;
            }
        }
        else if (dir == goRight)
        {
            if (!basemap[src.y][src.x + i])
            {
                ret.x = src.x + i;
                ret.y = src.y;
                return ret;
            }
        }
    }
    return ret;
}

uint8 getMapYMin_Col(uint8 x, uint8 map[][XM], uint8 value)
{
    for (uint8 i = 0; i < YM; ++i)
        if (map[i][x] == value)
            return i;
    return YM;
}

uint8 getMapYMax_Col(uint8 x, uint8 map[][XM], uint8 value)
{
    for (uint8 i = 0; i < YM; ++i)
        if (map[YY - i][x] == value)
            return YY - i;
    return YM;
}

/****************************************************************
uint8 getMapYMin_Col2(uint8 x, uint8 y, uint8 map[][XM])
作用：找到该图中，(x,y)上方第一个边界点纵坐标
 *****************************************************************/
uint8 getMapYMin_Col2(uint8 x, uint8 y, uint8 map[][XM])
{
    // 从（x，y）该点循环向上搜索，搜到0继续，搜到1返回YM，搜到2返回坐标
    for (uint8 i = y; i < YM; ++i)
    {
        if (map[i][x] == 1)
            break;
        if (map[i][x] == 2)
            return i;
    }
    return YM;
}

uint8 getMapXMax_Row(uint8 y, uint8 map[][XM], uint8 value)
{
    for (uint8 i = 0; i < XM; ++i)
        if (map[y][XX - i] == value)
            return XX - i;
    return XM;
}

uint8 getMapXMin_Row(uint8 y, uint8 map[][XM], uint8 value)
{
    for (uint8 i = 0; i < XM; ++i)
        if (map[y][i] == value)
            return i;
    return XM;
}

uint8 X_WBW_Detect(int8 x1, int8 x2, uint8 y, uint8 map[][XM], uint8 flag)
{
    if (flag == goRight)
        for (uint8 j = x1; j <= x2; ++j)
            if (map[y][j] != 1 && map[y][j] != 254)
            {
                for (uint8 k = j + 1; k <= x2 && k < XM; ++k)
                    if (map[y][k] == 1 || map[y][k] == 254)
                    {
                        for (uint8 m = k + 1; m <= x2; ++m)
                            if (map[y][m] != 1 && map[y][m] != 254)
                                return 1;
                        break;
                    }
                break;
            }
    if (flag == goLeft)
        for (uint8 j = x1; j >= x2 && j < 255; --j)
            if (map[y][j] != 1 && map[y][j] != 254)
            {
                for (uint8 k = j - 1; k >= x2 && k < 255; --k)
                    if (map[y][k] == 1 || map[y][k] == 254)
                    {
                        for (uint8 m = k - 1; m >= x2 && m < 255; --m)
                            if (map[y][m] != 1 && map[y][m] != 254)
                                return 1;
                        break;
                    }
                break;
            }
    return 0;
}

uint8 X_WBW_Detect2(int8 x1, int8 x2, uint8 y, uint8 map[][XM], uint8 flag)
{
    if (flag == goRight)
        for (uint8 j = x1; j <= x2; ++j)
            if (map[y][j] == 2 || map[y][j] == 253)
            {
                for (uint8 k = j + 1; k <= x2 && k < XM; ++k)
                    if (map[y][k] == 1 || map[y][k] == 254)
                    {
                        for (uint8 m = k + 1; m <= x2; ++m)
                            if (map[y][m] == 2 || map[y][m] == 253)
                                return 1;
                        break;
                    }
                break;
            }
    if (flag == goLeft)
        for (uint8 j = x1; j >= x2 && j < 255; --j)
            if (map[y][j] != 1)
            {
                for (uint8 k = j - 1; k >= x2 && k < 255; --k)
                    if (map[y][k] == 1)
                    {
                        for (uint8 m = k - 1; m >= x2 && m < 255; --m)
                            if (map[y][m] != 1)
                                return 1;
                        break;
                    }
                break;
            }
    return 0;
}

uint8 Y_BWB_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM])
{
    for (uint8 i = y1; i <= y2; ++i)
        if (map[i][x] == 1 || map[i][x] == 2)
        {
            for (uint8 k = i + 1; k <= y2; ++k)
                if (map[k][x] == 0)
                {
                    for (uint8 m = k + 1; m <= y2; ++m)
                        if (map[m][x] == 1 || map[m][x] == 2)
                            return 1;
                    break;
                }
            break;
        }
    return 0;
}

uint8 Y_WBW_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM])
{
    for (uint8 i = y1; i <= y2; ++i)
        if (map[i][x] == 0)
        {
            for (uint8 k = i + 1; k <= y2; ++k)
                if (map[k][x] == 1)
                {
                    for (uint8 m = k + 1; m <= y2; ++m)
                        if (map[m][x] == 0)
                            return 1;
                    break;
                }
            break;
        }
    return 0;
}

uint8 Y_RBR_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM])
{
    for (uint8 i = y1; i <= y2; ++i)
        if (map[i][x] == 2)
        {
            for (uint8 k = i + 1; k <= y2; ++k)
                if (map[k][x] == 1)
                {
                    for (uint8 m = k + 1; m <= y2; ++m)
                        if (map[m][x] == 2)
                            return 1;
                    break;
                }
            break;
        }
    return 0;
}

// 直线判定 （x1,y1） (x2,y2)确定直线的斜率，sy1 sy2为判断区间 limit是拿来调节判断的松紧的，越小判的越严
uint8 strJudge(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
               uint8 map[][XM], uint8 sy1, uint8 sy2, int8 limit,
               uint8 errorNum)
{
#ifdef BOOM7_QT_DEBUG
    qout << "strjudge";
#endif
    uint8 x;
    uint8 num = 0;
    // 输入坐标限制
    if (y1 >= y2 || sy1 < y1 || sy2 > y2)
        return 0;

#ifdef BOOM7_QT_DEBUG
    qout << "p1:(" << x1 << y1 << ") p2:(" << x2 << y2 << ")" << "limit:" << (int)limit;
#endif
    // 算出两点间斜率，注意是x/y
    float k = (float)(x2 - x1) / (y2 - y1);
    // 在判断区间循环
    for (uint8 i = sy1; i < sy2; ++i)
    {
        // 算出按照12两点构成的直线下，该纵坐标下横坐标应该在哪里
        x = (uint8)(x1 + k * (i - y1) + 0.5);
        // 如果就是边界点，则进行下个点的判定
        if (map[i][x] == 2 || map[i][x] == 253)
            continue;
        else
        {
            // 否则在该点左右3个单位找边界点
            uint8 flag = 0;
            for (int j = -3; j <= 3; ++j)
            {
                if (x + j < 0 || x + j > XX)
                    continue;
                if (map[i][x + j] == 2 || map[i][x + j] == 253) // 找到边界点，看是否越过设置的界限
                {
                    flag = 1;
                    if (j < -limit || j > limit)
                        ++num;
                    break;
                }
            }
            if (flag == 0)
            {
#ifdef BOOM7_QT_DEBUG
                for (uint8 i = y1; i < y2; ++i)
                    if (map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 2 && map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 3 && map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 4)
                        map[i][(uint8)(x1 + k * (i - y1) + 0.5)] = 7;
                qout << "error:" << num << "edge" << errorNum;
#endif
                return 0;
            }
        }
    }
#ifdef BOOM7_QT_DEBUG
    int color = num > errorNum ? 7 : 6;
    for (uint8 i = y1; i < y2; ++i)
        if (map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 2 && map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 3 && map[i][(uint8)(x1 + k * (i - y1) + 0.5)] != 4)
            map[i][(uint8)(x1 + k * (i - y1) + 0.5)] = color;
    qout << "error:" << num << "edge:" << errorNum;
#endif
    if (num > errorNum)
        return 0;
    return 1;
}

float strJudgeK(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
                uint8 map[][XM], uint8 sy1, uint8 sy2, int8 limit,
                uint8 errorNum)
{
#ifdef BOOM7_QT_DEBUG
    qout << "strjudgeK";
#endif
    uint8 x;
    uint8 num = 0;
    if (y1 >= y2 || sy1 < y1 || sy2 > y2 || y1 + 1 == y2)
        return 0;

#ifdef BOOM7_QT_DEBUG
    qout << "p1:(" << x1 << y1 << ") p2:(" << x2 << y2 << ")" << "limit:" << (int)limit;
#endif
    float k = (float)(x2 - x1) / (y2 - y1);

    for (uint8 i = sy1; i < sy2; ++i)
    {
        x = (uint8)(x1 + k * (i - y1) + 0.5);
        if (map[i][x] == 2 || map[i][x] == 253)
            continue;
        else
        {
            uint8 flag = 0;
            for (int8 j = -3; j <= 3; ++j)
            {
                if (x + j < 0 || x + j > XX)
                {
                    continue;
                }
                if (map[i][x + j] == 2 || map[i][x + j] == 253)
                {
                    flag = 1;
                    if (j < -limit || j > limit)
                    {
                        ++num;
                    }
                    break;
                }
            }
            if (flag == 0)
            {
#ifdef BOOM7_QT_DEBUG
                for (uint8 i = y1; i < y2; ++i)
                {
                    if (map[i][(uint8)(x1 + k * (i - y1))] != 2 && map[i][(uint8)(x1 + k * (i - y1))] != 3 && map[i][(uint8)(x1 + k * (i - y1))] != 4)
                    {
                        map[i][(uint8)(x1 + k * (i - y1))] = 7;
                    }
                }
#endif
                return 0;
            }
        }
    }
#ifdef BOOM7_QT_DEBUG
    int color = num > errorNum ? 7 : 6;
    for (uint8 i = y1; i < y2; ++i)
        if (map[i][(uint8)(x1 + k * (i - y1))] != 2 && map[i][(uint8)(x1 + k * (i - y1))] != 3 && map[i][(uint8)(x1 + k * (i - y1))] != 4)
            map[i][(uint8)(x1 + k * (i - y1))] = color;
    qout << "error:" << num << "edge" << errorNum;
#endif
    if (num > errorNum)
        return 0;
    return k;
}

uint8 strJudge_X(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
                 uint8 map[][XM], uint8 sx1, uint8 sx2, int8 limit,
                 uint8 errorNum)
{
#ifdef BOOM7_QT_DEBUG
    qout << "strjudge_X";
#endif
    uint8 y;
    uint8 num = 0;
    if (x1 >= x2 || sx1 < x1 || sx2 > x2)
        return 0;
#ifdef BOOM7_QT_DEBUG
    qout << "p1:(" << x1 << y1 << ") p2:(" << x2 << y2 << ")" << "limit:" << (int)limit;
#endif
    float k = (float)(y2 - y1) / (x2 - x1);

    for (uint8 i = sx1; i < sx2; ++i)
    {
        y = (uint8)(y1 + k * (i - x1) + 0.5);
        if (map[y][i] == 2 || map[y][i] == 253)
            continue;
        else
        {
            uint8 flag = 0;
            for (int8 j = -3; j <= 3; ++j)
            {
                if (y + j < 0 || y + j > YY)
                    continue;
                if (map[y + j][i] == 2 || map[y + j][i] == 253)
                {
                    flag = 1;
                    if (j < -limit || j > limit)
                        ++num;
                    break;
                }
            }
            if (flag == 0)
            {
#ifdef BOOM7_QT_DEBUG
                for (uint8 i = x1; i < x2; ++i)
                    map[(uint8)(y1 + k * (i - x1) + 0.5)][i] = 7;
#endif
                return 0;
            }
        }
    }
#ifdef BOOM7_QT_DEBUG
    int color = num > errorNum ? 7 : 6;
    for (uint8 i = x1; i < x2; ++i)
        map[(uint8)(y1 + k * (i - x1) + 0.5)][i] = color;
    qout << "error:" << num << "edge" << errorNum;
#endif
    if (num > errorNum)
        return 0;
    return 1;
}

/*************************************************
Function: countTripPoints(Point p1, Point p2, uint8 map[][XM])
Description: 数两点间的黑白跳变次数
Input: 两个点、在哪张图里数
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 countTripPoints(Point p1, Point p2, uint8 map[][XM])
{
    Point left, right;
    if (p1.x == p2.x)
        return 0;
    else if (p1.x < p2.x)
    {
        left = p1;
        right = p2;
    }
    else
    {
        left = p2;
        right = p1;
    }
    float k = (float)(p2.y - p1.y) / (float)(p2.x - p1.x) * 1.0;
    uint8 p_array[XM];
    for (uint8 i = left.x; i <= right.x; i++)
        p_array[i] = left.y + (i - left.x) * k + 0.5;

    uint8 jumpCnt = 0;
    for (uint8 i = left.x; i <= right.x; i++)
        if ((1 == map[p_array[i]][i] && 0 == map[p_array[i + 1]][i + 1]) || (0 == map[p_array[i]][i] && 1 == map[p_array[i + 1]][i + 1]))
            jumpCnt++;

#ifdef BOOM7_QT_DEBUG
    if (jumpCnt > 8)
        for (uint8 i = left.x; i <= right.x; i++)
            map[p_array[i]][i] = 4;
#endif
    return jumpCnt;
}

/*************************************************
Function: uint8 countImgBinaryTripPoints(uint8 col_min, uint8 col_max, uint8 row)
Description: 数二值化图内两点间的黑白跳变次数
Input: 两个点、在哪张图里数
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 countImgBinaryTripPoints(uint8 col_min, uint8 col_max, uint8 row)
{
#ifdef BOOM7_QT_DEBUG
    qout << "col_min:" << col_min << "col_max:" << col_max << "row" << row;
#endif
    uint8 left, right;
    if (col_min == col_max)
        return 0;
    else if (col_min < col_max)
    {
        left = col_min;
        right = col_max;
    }
    else
    {
        left = col_max;
        right = col_min;
    }
    uint8 jumpCnt = 0;
    for (uint8 i = left; i <= right; i++)
    {
        if ((PIXEL_BLACK == img_binary[row][i] && PIXEL_WHITE == img_binary[row][i + 1]) || (PIXEL_WHITE == img_binary[row][i] && PIXEL_BLACK == img_binary[row][i + 1]))
            jumpCnt++;
    }

#ifdef BOOM7_QT_DEBUG
    if (jumpCnt > 12)
        for (uint8 i = left; i <= right; i++)
            img_binary[row][i] = PIXEL_BLACK;
#endif
    return jumpCnt;
}

void drawline(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
              uint8 map[][XM]) // y1>y2
{
    if (y1 <= y2)
    {
        uint8 tmp;
        tmp = x1;
        x1 = x2;
        x2 = tmp;
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }
#ifdef BOOM7_QT_DEBUG
    qout << "p1:(" << x1 << y1 << ") p2:(" << x2 << y2 << ")";
#endif
    float k = (float)(x1 - x2) / (y1 - y2);
    for (uint8 i = y2; i <= y1; ++i)
    {
        if (map == leftmap)
            II.leftdrawflag[i] = 1;
        else if (map == rightmap)
            II.rightdrawflag[i] = 1;
        map[i][(uint8)(x2 + k * (i - y2))] = 3;
    }
}

void drawline2(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
               uint8 map[][XM], uint8 n) // x1>x2
{
    uint8 lasty = YM;
    if (x1 <= x2)
    {
        uint8 tmp;
        tmp = x1;
        x1 = x2;
        x2 = tmp;
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }
#ifdef BOOM7_QT_DEBUG
    qout << "p1:(" << x1 << y1 << ") p2:(" << x2 << y2 << ")";
#endif
    float k = (float)(y1 - y2) / (x1 - x2);
    float mid = (float)(x1 + x2) / 2;
    uint8 climax = (uint8)n * (x1 - x2) / XM; // 12.0
    for (uint8 j = x2; j <= x1; ++j)
    {
        uint8 temp = (uint8)((j - x2) * k + y2 + climax -
                             climax * (2 * fabs(j - mid) / (x1 - x2)) *
                                 (2 * fabs(j - mid) / (x1 - x2)));
        if (temp > YY)
        {
            continue;
        }
        if (map == leftmap)
            II.leftdrawflag[temp] = 1;
        else if (map == rightmap)
            II.rightdrawflag[temp] = 1;
        map[temp][j] = 3;
        int8 gap = (int8)temp - lasty;
        if (gap > 1 && lasty != YM)
        {
            while (--gap)
            {

                if (map == leftmap)
                    II.leftdrawflag[temp - gap] = 1;
                else if (map == rightmap)
                    II.rightdrawflag[temp - gap] = 1;
                map[temp - gap][j] = 3;
            }
        }
        if (gap < -1 && lasty != YM)
        {
            while (++gap)
            {
                if (map == leftmap)
                    II.leftdrawflag[temp - gap] = 1;
                else if (map == rightmap)
                    II.rightdrawflag[temp - gap] = 1;
                map[temp - gap][j - 1] = 3;
            }
        }
        lasty = temp;
    }
}

void drawline3(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
               uint8 map[][XM]) // x1>x2
{
    uint8 lasty = YM;
    if (x1 == x2)
        return;
    float k = (float)(y1 - y2) / (x1 - x2);
    float mid = (float)(x1 + x2) / 2;
    uint8 climax = (uint8)12 * (x1 - x2) / XM; // 12.0
    for (uint8 j = x2; j <= x1; ++j)
    {
        uint8 temp = (uint8)((j - x2) * k + y2 - climax +
                             climax * (2 * fabs(j - mid) / (x1 - x2)) *
                                 (2 * fabs(j - mid) / (x1 - x2)));
        if (temp > YY)
        {
            continue;
        }
        map[temp][j] = 3;
        int8 gap = (int8)temp - lasty;
        if (gap > 1 && lasty != YM)
        {
            while (--gap)
            {
                map[temp - gap][j] = 3;
            }
        }
        if (gap < -1 && lasty != YM)
        {
            while (++gap)
            {
                map[temp - gap][j - 1] = 3;
            }
        }
        lasty = temp;
    }
}

uint8 twentyCmOutGarageStateMachine(void)
{
    static uint8 status = 253;
    if (garage_clearFlag)
    {
        outGarageFlag = 1;
        status = 0;
        garage_clearFlag = 0;
    }
    if (status == 253)
    {
        static uint8 cntL = 0, cntR = 0;
        if (II.num_lm && !II.num_rm)
            cntL++;
        else if (II.num_rm && !II.num_lm)
            cntR++;
        else if (II.num_lm && II.num_rm)
        {
            if (II.lnum_all > II.rnum_all)
                cntL++;
            else
                cntR++;
        }
        if (cntL > 10)
        {
            garageDirectionFlag = goRight;
            status = 254;
        }
        else if (cntR > 10)
        {
            garageDirectionFlag = goLeft;
            status = 254;
        }
#ifndef BOOM7_QT_DEBUG
        BEEP(1000);
#endif
    }
    if (status == 254)
    {
        do
        {
            if (garageDirectionFlag == goLeft)
            {
                if (!II.num_rm)
                {
                    status = 255;
                    break;
                }
                uint8 up_x = 10;
                uint8 up_y = getMapYMax_Col(up_x, basemap, 0);
                uint8 down_x = LT_X(0);
                uint8 down_y = LT_Y(0);

                if (II.num_lm)
                {
                    up_x = 3 * RT_X(0);
                    up_y = getMapYMin_Col(up_x, deletemap, 1);
                }
                if (down_x < XM / 2)
                {
                    down_x = getMapXMax_Row(0, basemap, 0);
                    down_y = 0;
                }
                if (up_y > 40)
                    up_y = 40;
                drawline2(down_x, down_y, up_x, up_y, rightmap, 12);
            }
            else
            {
                if (!II.num_lm)
                {
                    status = 255;
                    break;
                }
                uint8 up_x = XX - 10;
                uint8 up_y = getMapYMax_Col(up_x, basemap, 0);
                uint8 down_x = RT_X(0);
                uint8 down_y = RT_Y(0);

                if (II.num_rm)
                {
                    up_x = XX - 3 * (XX - LT_X(0));
                    up_y = getMapYMin_Col(up_x, deletemap, 1);
                }
                if (down_x > XM / 2)
                {
                    down_x = getMapXMin_Row(0, basemap, 0);
                    down_y = 0;
                }
                if (up_y > 40)
                    up_y = 40;
                drawline2(up_x, up_y, down_x, down_y, leftmap, 12);
            }
        } while (0);
    }
    if (status == 255)
    {
        static uint8 cnt = 0;
        do
        {
            if (garageDirectionFlag == goLeft)
            {
                if (RI.numCnt[RI.segCnt - 1] > 30)
                {
                    cnt++;
                    if (cnt > 2)
                    {
                        status = 0;
                        outGarageFlag = 1;
                        break;
                    }
#ifndef BOOM7_QT_DEBUG
                    BEEP(1000);
#endif
                }
                uint8 up_x = 10;
                uint8 up_y = getMapYMin_Col(up_x, deletemap, 1);
                if (up_y == YM)
                    up_y = getMapYMin_Col(up_x, rightmap, 1);
                if (up_y == YM)
                    up_y = getMapYMin_Col(up_x, basemap, 1);
                if (up_y > 40 || up_y == 0)
                    up_y = 40;
                uint8 down_x = XX;
                uint8 down_y = 0;
                drawline(up_x, up_y, down_x, down_y, rightmap);
            }
            else
            {
                if (LI.numCnt[LI.segCnt - 1] > 30)
                {
                    cnt++;
                    if (cnt > 2)
                    {
                        status = 0;
                        outGarageFlag = 1;
                        break;
                    }
#ifndef BOOM7_QT_DEBUG
                    BEEP(1000);
#endif
                }
                uint8 up_x = XX - 10;
                uint8 up_y = getMapYMin_Col(up_x, deletemap, 1);
                if (up_y == YM)
                    up_y = getMapYMin_Col(up_x, leftmap, 1);
                if (up_y == YM)
                    up_y = getMapYMin_Col(up_x, basemap, 1);
                if (up_y > 40 || up_y == 0)
                    up_y = 40;
                uint8 down_x = 0;
                uint8 down_y = 0;
                drawline(up_x, up_y, down_x, down_y, leftmap);
            }
        } while (0);
    }
    return status;
}

uint8 twentyCmEnterGarageStateMachine()
{
    static uint8 enterGarageStatus = 0;
    static uint8 delay = 0;
    if (delay > 0)
    {
        delay--;
        return 0;
    }
    else
        fistJudgeFlag = 0;
    if (garage_clearFlag)
        goto GARAGE_CLEAR;
    if (0 == enterGarageStatus && angle.Pitch > -7 && angle.Pitch < 7)
    {
        uint8 isGarage = twentyCmGarageJudge(judgedCnt);
        if (!delay && isGarage)
        {
#ifdef BOOM7_QT_DEBUG
            qout << "startLineDetected";
#endif
            if (judgedCnt == 0)
            {
                judgedCnt++;
                //                dir_fork = 3-dir_fork;//改三叉标志位（第二圈往另一个方向走）
                delay = 80;
                fistJudgeFlag = 1;
            }
            else
            {
                IF.rampDelay = 0;
                IF.annulusDelay = 0;
                enterGarageStatus = isGarage;
            }
        }
    }
    if (GL1 == enterGarageStatus)
    {
        do
        {
            if (readyEnterGarage())
            {
                enterGarageStatus = GL2;
                break;
            }
            if (!startAddLine_enterGarage && timeToTurn_enterGarage())
            {
                IF.crossroad = 0;
                cross_clearFlag = 1;
                startAddLine_enterGarage = 1;
            }
            if (startAddLine_enterGarage)
                garageAddLine(enterGarageStatus);
            else
            {
                // 单纯删线
                if (II.right_y != YM)
                    for (uint8 row = II.right_y; row < YM; row++)
                        II.leftdeleteflag[row] = 1;
                else
                    for (uint8 row = PL.right_y[0]; row < YM; row++)
                        II.leftdeleteflag[row] = 1;
            }
        } while (0);
    }
    if (GL2 == enterGarageStatus)
    {
        do
        {
            if ((II.num_rm && II.rnum[0] > 500) /*||stopCarCnt==0*/)
            {
                enterGarageStatus = GL3;
                break;
            }
            garageAddLine(enterGarageStatus);
        } while (0);
    }
    if (GL3 == enterGarageStatus)
    {
        if (stop())
        {
#ifndef BOOM7_QT_DEBUG
            runState = BRAKING;
            stopReason = EnterGarage;
#else
            qout << "end";
#endif
        }
    }
    if (GR1 == enterGarageStatus)
    {
        do
        {
            if (readyEnterGarage())
            {
                enterGarageStatus = GR2;
                break;
            }
            if (!startAddLine_enterGarage && timeToTurn_enterGarage())
                startAddLine_enterGarage = 1;
            if (startAddLine_enterGarage)
                garageAddLine(enterGarageStatus);
            else
            {
                // 单纯删线
                if (II.left_y != YM)
                    for (uint8 row = II.left_y; row < YM; row++)
                        II.rightdeleteflag[row] = 1;
                else
                    for (uint8 row = PR.left_y[0]; row < YM; row++)
                        II.rightdeleteflag[row] = 1;
            }
        } while (0);
    }
    if (GR2 == enterGarageStatus)
    {
        do
        {
            if ((II.num_lm && II.lnum[0] > 500))
            {
                enterGarageStatus = GR3;
                break;
            }
            garageAddLine(enterGarageStatus);
        } while (0);
    }
    if (GR3 == enterGarageStatus)
    {
        if (stop())
        {
#ifndef BOOM7_QT_DEBUG
            runState = BRAKING;
            stopReason = EnterGarage;
#else
            qout << "end";
#endif
        }
    }
    return enterGarageStatus;
GARAGE_CLEAR:
    IF.garage = 0;
    enterGarageStatus = 0;
    garage_clearFlag = 0;
    startAddLine_enterGarage = 0;
    return 0;
}

/*************************************************
Function: twentyCmGarageJudge()
Description: 入库判定，判起跑线
Input: null
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmGarageJudge(uint8 judgedCnt)
{
    if (!twentyCmStartLineJudge(judgedCnt))
        return 0;
    //    uint8 sumL = 0,sumR = 0;
    //    for(uint8 i=0;i<LI.segCnt;i++)
    //        sumL+=LI.numCnt[i];
    //    for(uint8 i=0;i<RI.segCnt;i++)
    //        sumR+=RI.numCnt[i];
    //    if(sumL>35 && II.num_lm==1 && II.start_lm[0]<=15 && II.lnum[0]>500) return goRight;
    //    if(sumR>35 && II.num_rm==1 && II.start_rm[0]<=15 && II.rnum[0]>500) return goLeft;
    if (garageDirectionFlag == goRight)
        return goRight;
    if (garageDirectionFlag == goLeft)
        return goLeft;
    return 0;
}

/*************************************************
Function: twentyCmStartLineJudge()
Description: 判起跑线
Input: null
Output: null
Return: 是否有起跑线
Others: 思路：把insidemap里面黑点的范围的矩形框找到，对两个对角线，和水平中线数跳变，满足一个就行
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmStartLineJudge(uint8 judgedCnt)
{
    if (II.inum_all <= 20)
        return 0;
    uint8 cnt = 0;
    uint8 col_min = XX, col_max = 0;
    uint8 row_min = YY, row_max = 0;
    uint8 cnt_chopping = 0;
    for (uint8 row = 0; row < YM; row++)
    {
        uint8 min = getMapXMin_Row(row, insidemap, 1),
              max = getMapXMax_Row(row, insidemap, 1);
        if (min == XM && max == XM)
        {
            if (cnt == 0)
                continue;
            else
            {
                cnt_chopping++;
                if (cnt_chopping > 2)
                    break;
            }
        }
        else
        {
            if (max > col_max)
                col_max = max;
            if (min < col_min)
                col_min = min;
            if (cnt == 0)
            {
                row_min = row;
            }
            cnt++;
            row_max = row;
        }
    }
#ifdef BOOM7_QT_DEBUG
    qout << "col_min:" << col_min << "col_max:" << col_max << "row_min:" << row_min << "row_max:" << row_max;
    for (uint8 row = row_min - 1; row <= row_max + 1; row++)
    {
        if (row != YM && row != 255 && col_max != XX)
            insidemap[row][col_max + 1] = 2;
        if (row != YM && row != 255 && col_min != 0)
            insidemap[row][col_min - 1] = 2;
    }
    for (uint8 col = col_min - 1; col <= col_max + 1; col++)
    {
        if (col != XM && col != 255 && row_max != YY)
            insidemap[row_max + 1][col] = 2;
        if (col != XM && col != 255 && row_min != 0)
            insidemap[row_min - 1][col] = 2;
    }
#endif

    Point LB, LT, RB, RT, LM, RM;
    LB.x = col_min;
    LB.y = row_min;
    LT.x = col_min;
    LT.y = row_max;
    RB.x = col_max;
    RB.y = row_min;
    RT.x = col_max;
    RT.y = row_max;
    LM.x = col_min;
    LM.y = (row_max + row_min) / 2;
    RM.x = col_max;
    RM.y = (row_max + row_min) / 2;
    uint8 LT2RB = countTripPoints(LT, RB, insidemap);
    uint8 LB2RT = countTripPoints(LB, RT, insidemap);
    uint8 LM2RM = countTripPoints(LM, RM, insidemap);
    if (row_min > 55)
        return 0;
#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
    qout << "LT2RB" << LT2RB << "LT2RB" << LB2RT << "LM2RM" << LM2RM;
#endif
    if (LT2RB > 8 || LB2RT > 8 || LM2RM > 8)
    {
        return 1;
    }
    // 过第一次起跑线前不用灰度图判
    if (judgedCnt == 0)
        return 0;
        // 到此如果不行，进行灰度图的判定，灰度图只判长直的入库，转弯入库不用那么早判出来
#ifdef BOOM7_QT_DEBUG
    qout << "DETECT_BY_IMGBINARY";
    if (imgSizeInfo == _188_60)
    {
#endif
        // 速度不快，就不用判灰度图了
#ifndef BOOM7_QT_DEBUG
        int maxSpeed = SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0]; // 当前两个轮子中速度大的那个
        if (maxSpeed < 230)
            return 0;
#endif

        uint8 keySegL = 255, keySegR = 255;
        for (uint8 seg = 0; seg < LI.segCnt; seg++)
            if (L_END_Y(seg) > 40 || LI.numCnt[seg] > 40)
            {
                keySegL = seg;
                break;
            }
        for (uint8 seg = 0; seg < RI.segCnt; seg++)
            if (R_END_Y(seg) > 40 || RI.numCnt[seg] > 40)
            {
                keySegR = seg;
                break;
            }
#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
        qout << "keySegL" << keySegL << "keySegR" << keySegR;
#endif
        if (keySegL == 255 || keySegR == 255)
            return 0;
        if (L_START_Y(keySegL) < 30 && R_START_Y(keySegR) < 30)
        {
            uint8 lower = L_END_Y(keySegL) < R_END_Y(keySegR) ? goLeft : goRight;
            row_min = lower == goLeft ? L_END_Y(keySegL) : R_END_Y(keySegR);
            row_min -= 4;
            row_max = row_min + 7 > YY ? YY : row_min + 7;

            if (lower == goLeft)
            {
                col_min = II.leftline[row_min];
                col_max = getMapXMax_Row(row_min, rightmap, 0) + 1;
                if (LI.segCnt == 2)
                    row_max = L_START_Y(1);
                else
                    row_max += 2;
            }
            else
            {
                col_max = II.rightline[row_min];
                col_min = getMapXMin_Row(row_min, leftmap, 0) - 1;
                if (RI.segCnt == 2)
                    row_max = R_START_Y(1);
                else
                    row_max += 2;
            }
            row_min += 2;
        }
#ifdef BOOM7_QT_DEBUG
        for (uint8 row = row_min - 1; row <= row_max + 1; row++)
        {
            if (row != YM && row != 255 && col_max != XX)
                insidemap[row][col_max + 1] = 2;
            if (row != YM && row != 255 && col_min != 0)
                insidemap[row][col_min - 1] = 2;
        }
        for (uint8 col = col_min - 1; col <= col_max + 1; col++)
        {
            if (col != XM && col != 255 && row_max != YY)
                insidemap[row_max + 1][col] = 2;
            if (col != XM && col != 255 && row_min != 0)
                insidemap[row_min - 1][col] = 2;
        }
#endif
        // 横纵坐标转换到灰度图的横坐标
        col_max *= HORIZON_TIMES;
        col_min *= HORIZON_TIMES;
        uint8 tmp = row_max;
        row_max = YY - row_min;
        row_min = YY - tmp;

#ifdef BOOM7_QT_DEBUG
        for (uint8 row = row_min - 1; row <= row_max + 1; row++)
        {
            if (row != YM && row != 255 && col_max != CAMERA_W - 1)
                imgGray[row][col_max + 1] = 2;
            if (row != YM && row != 255 && col_min != 0)
                imgGray[row][col_min - 1] = 2;
        }
        for (uint8 col = col_min - 1; col <= col_max + 1; col++)
        {
            if (col != XM && col != 255 && row_max != CAMERA_H - 1)
                imgGray[row_max + 1][col] = 0;
            if (col != XM && col != 255 && row_min != 0)
                imgGray[row_min - 1][col] = 0;
        }
#endif

        for (uint8 row = row_min; row <= row_max; row++)
        {
            int cnt = countImgBinaryTripPoints(col_min, col_max, row);
#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
            qout << row << cnt;
#endif
            if (cnt > 11)
                return 1;
        }
#ifdef BOOM7_QT_DEBUG
    }
#endif
    return 0;
}

uint8 timeToTurn_enterGarage()
{
    if (!II.inum_all)
        return 0;
    float dist;
    uint8 top_x = XM, top_y = YM;
    uint8 bottom_x = XM, bottom_y = 0;

    uint8 cnt = 0;
    uint8 col_min = XX, col_max = 0;
    uint8 row_min = YY, row_max = 0;
    uint8 cnt_chopping = 0;
    for (uint8 row = 0; row < YM; row++)
    {
        uint8 min = getMapXMin_Row(row, insidemap, 1),
              max = getMapXMax_Row(row, insidemap, 1);
        if (min == XM && max == XM)
        {
            if (cnt == 0)
                continue;
            else
            {
                cnt_chopping++;
                if (cnt_chopping > 2)
                    break;
            }
        }
        else
        {
            if (max > col_max)
                col_max = max;
            if (min < col_min)
                col_min = min;
            if (cnt == 0)
            {
                row_min = row;
            }
            cnt++;
            row_max = row;
        }
    }

    top_x = (col_max + col_min) / 2;
    top_y = (row_max + row_min) / 2;
    bottom_x = II.midline[0];
    int p1_x_real = ((int)bottom_x - leftline[bottom_y]) * k1[bottom_y];
    int p1_y_real = k2[bottom_y];
    int p2_x_real = ((int)top_x - leftline[top_y]) * k1[top_y];
    int p2_y_real = k2[top_y];

    dist = distance(p1_x_real, p1_y_real, p2_x_real, p2_y_real);
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "top:" << top_x << top_y;
    qout << "bottom:" << bottom_x << bottom_y;
    qout << "dist:" << dist;
#endif
    if (dist < drawLine_distance_enterGarage)
        return 1;
    return 0;
}

uint8 readyEnterGarage()
{
    uint8 realRowCnt = 0;
    for (uint8 row = 0; row < 30; row++)
        if (X_WBW_Detect(0, XX, row, deletemap, goRight))
            realRowCnt++;
    if (realRowCnt >= 5)
        return 1;
    if (garageDirectionFlag == goLeft && II.searchLineMid < 5)
        if ((II.dnum_all > 300 && II.d_bottom[0].y < 30 && II.d_bottom[0].x < XX - 5) || (II.lnum_all > 300 && II.start_lm[0] > 0 && BR_Y(0) < 30))
            return 1;
    if (garageDirectionFlag == goRight && II.searchLineMid > XM - 5)
        if ((II.dnum_all > 300 && II.d_bottom[0].y < 30 && II.d_bottom[0].x > 5) || (II.rnum_all > 300 && II.start_rm[0] > 0 && BL_Y(0) < 30))
            return 1;
    return 0;
}

/*************************************************
Function: uint8 stop()
Description: 停车判定
Input: null
Output: null
Return: 是否停车
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 stop()
{
    if (II.top < 20)
        return 1;
    return 0;
}

/*************************************************
Function: garageAddLine(uint8 status)
Description: 入库补线
Input: 目前入库状态
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void garageAddLine(uint8 status)
{
    uint8 up_x = XM, up_y = YM;
    uint8 down_x = XM, down_y = YM;
    II.inum_all = 0;
    for (uint8 i = 0; i < 50; i++)
        for (uint8 j = 0; j < XM; j++)
            if (insidemap[i][j])
                II.inum_all++;

    if (status == GL1)
    {
        if (II.inum_all)
        {
            uint8 cnt = 0;
            uint8 col_min = XX, col_max = 0;
            uint8 row_min = YY, row_max = 0;
            uint8 cnt_chopping = 0;
            for (uint8 row = 0; row < YM; row++)
            {
                uint8 min = getMapXMin_Row(row, insidemap, 1),
                      max = getMapXMax_Row(row, insidemap, 1);
                if (min == XM && max == XM)
                {
                    if (cnt == 0)
                        continue;
                    else
                    {
                        cnt_chopping++;
                        if (cnt_chopping > 2)
                            break;
                    }
                }
                else
                {
                    if (max > col_max)
                        col_max = max;
                    if (min < col_min)
                        col_min = min;
                    if (cnt == 0)
                        row_min = row;
                    cnt++;
                    row_max = row;
                }
            }
            if (row_min == YY)
                down_y = 0;
            else
            {
                // 找下方点横坐标，有点半开环的意思，固定实际距离，往下找点
                row_min = (row_max + row_min) / 2;
                if (k2[row_min] < 10 + k2[0])
                    down_y = 0;
                else
                    for (uint8 row = 0; row < row_min; row++)
                    {
                        if (k2[row] + 10 < k2[row_min])
                            down_y = row;
                        else
                            break;
                    }
            }
            if (II.rnum_all < 50)
                down_x = XX;
            else
                down_x = getMapXMin_Row(down_y, rightmap, 1) - 1;
            if (down_x == XM - 1)
                down_x = XX;

            // 上方点,在insidemap里找最靠左的点
            col_max = XX;
            row_max = XX;
            for (uint8 row = row_min; row < row_min + 10; row++)
            {
                uint8 max = getMapXMin_Row(row, insidemap, 1);
                if (max == XM)
                    continue;
                if (max < col_max)
                {
                    col_max = max;
                    row_max = row;
                }
            }
#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
            qout << col_max << row_max;
#endif
            // 这个不可能找不到的，找到之后，把坐标根据实际距离向左平移10cm
            if (col_max * k1[row_max] < 10)
                up_x = 0;
            else
                up_x = col_max - 10 / k1[row_max];

            for (uint8 row = row_max; row < YM; row++)
                if (basemap[row][up_x])
                {
                    up_y = row;
                    break;
                }

#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
            qout << down_x << down_y << up_x << up_y;
#endif
            if (up_y != YM)
            {
                drawline2(down_x, down_y, up_x, up_y, rightmap, 12);
                for (uint8 row = down_y; row < up_y; row++)
                    II.leftdeleteflag[row] = 1;
            }
            for (uint8 i = up_y; i < YM; i++)
            {
                II.leftdeleteflag[i] = 1;
                II.rightdeleteflag[i] = 1;
            }
        }
        else
            II.line_forbid = BOTHLINE;
    }
    else if (status == GL2)
    {
        if (II.searchLineMid >= myCarMid - 5)
        {
            uint8 tmp = YM;
            for (uint8 i = 0; i < 10; i++)
            {
                tmp = getMapYMin_Col(i, deletemap, 1);
                if (tmp != YM && (up_y <= tmp || up_y == YM))
                {
                    up_y = tmp;
                    up_x = i;
                }
            }
            down_x = XX;
            down_y = 0;
            if (up_y != YM)
                drawline2(down_x, down_y, up_x, up_y, rightmap, 12);
            else
                II.line_forbid = BOTHLINE;
            for (uint8 i = up_y; i < YM; i++)
            {
                II.leftdeleteflag[i] = 1;
                II.rightdeleteflag[i] = 1;
            }
        }
        else
        {
            drawline2(XX, 0, R_START_X(RI.segCnt - 1), R_START_Y(RI.segCnt - 1), rightmap, 10);
            for (uint8 row = R_START_Y(RI.segCnt - 1); row < R_END_Y(RI.segCnt - 1); row++)
                rightmap[row][II.rightline[row]] = 2;
        }
    }
    else if (status == GR1) // 右库状态1
    {
        // 根据insidemap找，insidemap没东西还在这个1状态，差不多就无了
        if (II.inum_all)
        {
            uint8 cnt = 0;
            uint8 col_min = XX, col_max = 0;
            uint8 row_min = YY, row_max = 0;
            uint8 cnt_chopping = 0;
            for (uint8 row = 0; row < YM; row++)
            {
                uint8 min = getMapXMin_Row(row, insidemap, 1),
                      max = getMapXMax_Row(row, insidemap, 1);
                if (min == XM && max == XM)
                {
                    if (cnt == 0)
                        continue;
                    else
                    {
                        cnt_chopping++;
                        if (cnt_chopping > 2)
                            break;
                    }
                }
                else
                {
                    if (max > col_max)
                        col_max = max;
                    if (min < col_min)
                        col_min = min;
                    if (cnt == 0)
                        row_min = row;
                    cnt++;
                    row_max = row;
                }
            }
            if (row_min == YY)
                down_y = 0;
            else
            {
                // 找下方点横坐标，有点半开环的意思，固定实际距离，往下找点
                row_min = (row_max + row_min) / 2;
                if (k2[row_min] < 10 + k2[0])
                    down_y = 0;
                else
                    for (uint8 row = 0; row < row_min; row++)
                    {
                        if (k2[row] + 10 < k2[row_min])
                            down_y = row;
                        else
                            break;
                    }
            }
            // 在左图找对应横坐标
            if (II.lnum_all < 50)
                down_x = 0;
            else
                down_x = getMapXMax_Row(down_y, leftmap, 1) + 1;
            if (down_x == XM + 1)
                down_x = 0;

            // 上方点,在insidemap里找最靠右的点
            col_max = 0;
            row_max = XX;
            for (uint8 row = row_min; row < row_min + 10; row++)
            {
                uint8 max = getMapXMax_Row(row, insidemap, 1);
                if (max == XM)
                    continue;
                if (max > col_max)
                {
                    col_max = max;
                    row_max = row;
                }
            }
#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
            qout << col_max << row_max;
#endif
            // 这个不可能找不到的，找到之后，把坐标根据实际距离向右平移10cm
            if ((XM - col_max) * k1[row_max] < 10)
                up_x = XX;
            else
                up_x = col_max + 10 / k1[row_max];

            for (uint8 row = row_max; row < YM; row++)
                if (basemap[row][up_x])
                {
                    up_y = row;
                    break;
                }

#ifdef BOOM7_QT_DEBUG_ENTERGARAGE
            qout << down_x << down_y << up_x << up_y;
#endif
            if (up_y != YM)
            {
                drawline2(up_x, up_y, down_x, down_y, leftmap, 12);
                for (uint8 row = down_y; row < up_y; row++)
                    II.rightdeleteflag[row] = 1;
            }
            for (uint8 i = up_y; i < YM; i++)
            {
                II.leftdeleteflag[i] = 1;
                II.rightdeleteflag[i] = 1;
            }
        }
        else
            II.line_forbid = BOTHLINE;
    }
    else if (status == GR2) // 右库状态2
    {
        if (II.searchLineMid <= myCarMid + 5)
        {
            uint8 tmp = YM;
            for (uint8 i = XX - 10; i < XM; i++)
            {
                tmp = getMapYMin_Col(i, deletemap, 1);
                if (tmp != YM && (up_y <= tmp || up_y == YM))
                {
                    up_y = tmp;
                    up_x = i;
                }
            }
            down_x = 0;
            down_y = 0;
            if (up_y != YM)
                drawline2(up_x, up_y, down_x, down_y, leftmap, 12);
            else
                II.line_forbid = BOTHLINE;
            for (uint8 i = up_y; i < YM; i++)
            {
                II.leftdeleteflag[i] = 1;
                II.rightdeleteflag[i] = 1;
            }
        }
        else
        {
            drawline2(0, 0, L_START_X(LI.segCnt - 1), L_START_Y(LI.segCnt - 1), leftmap, 10);
            for (uint8 row = L_START_Y(LI.segCnt - 1); row < L_END_Y(LI.segCnt - 1); row++)
                leftmap[row][II.leftline[row]] = 2;
        }
    }
}

/*十字状态机，BOOM6十字有时会不稳，还是写一个*/
void crossLastKeyPointInit()
{
    lastUpLeft_x = 0;
    lastUpLeft_y = YM;
    lastUpRight_x = XM;
    lastUpRight_y = YM;
    lastDownLeft_x = XM;
    lastDownLeft_y = YM;
    lastDownRight_x = 0;
    lastDownRight_y = YM;
}

/*************************************************
Function: twentyCmGoCrossroadStateMachine()
Description: 十字状态机
Input: null
Output: null
Return: 十字状态
Others:

Author: BOOM5 HLZ BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmGoCrossroadStateMachine()
{
    static uint8 status = 0;
    static uint8 num_map[5] = {255, 255, 255, 255, 255};
    if (cross_clearFlag)
        goto CROSS_CLEAR;
    if (status == 0)
    {
        status = twentyCmCrossroadJudge();
        if (twentyCmUcrossJudge())
        {
            if (ucross_manual_setting)
            {
                if (cnt_ucross == 0)
                    dir_ucross = dir_ucross_manual_setting_1;
                if (cnt_ucross == 1)
                    dir_ucross = dir_ucross_manual_setting_2;
            }
            else if (dir_ucross == 0)
                dir_ucross = getUcrossDirByDrivingRecorder();
            cnt_ucross++;
            if (cnt_ucross == 2)
                cnt_ucross = 0;
            if (dir_ucross == goLeft)
                status = UL1;
            else
                status = UR1;
        }
    }

    if (status == CL1)
    {
        if (II.right_y == YM)
            twentyCmCarRefindLeftDownCorner();
        do
        {
            if (II.rightPortaitSearchLineFlag && TOP_RM(0) < R_END_Y(0) + 10 && isCross_right())
            {
                status = CM1;
                break;
            }
            if (L_START_Y(0) > 30 || II.num_lm == 0 || II.start_lm[0] > 20)
            {
                status = CL2;
                break;
            }
            twentyCmCarFindLeftUpCorner(); // 找上角点
            if (II.upRight_y != YM)
                II.leftCrossDealFlag = 1;
        } while (0);
    }
    if (status == CL2) // 左  十字中
    {
        do
        {
            if ((LI.segCnt && L_START_Y(0) < 20 && II.lnum_all > 50) || (RI.segCnt && R_START_Y(0) < 20 && II.rnum_all > 50))
                goto CROSS_CLEAR;
            twentyCmCarFindLeftUpCorner(); // 找上角点
            if (II.upRight_y != YM)
                II.leftCrossDealFlag = 1;
        } while (0);
    }
    // 右侧特征判
    if (status == CR1)
    {
        if (II.left_y == YM)
            twentyCmCarRefindRightDownCorner();
        do
        {
            if (II.leftPortaitSearchLineFlag && TOP_LM(0) < L_END_Y(0) + 10 && isCross_left())
            {
                status = CM1;
                break;
            }
            if (R_START_Y(0) > 30 || II.num_rm == 0 || II.start_rm[0] > 20)
            {
                status = CR2;
                break;
            }
            twentyCmCarFindRightUpCorner(); // 找上角点
            if (II.upLeft_y != YM)
                II.rightCrossDealFlag = 1;
        } while (0);
    }
    if (status == CR2) // 右  十字中
    {
        do
        {
            if ((LI.segCnt && L_START_Y(0) < 20 && II.lnum_all > 200) || (RI.segCnt && R_START_Y(0) < 20 && II.rnum_all > 200))
                goto CROSS_CLEAR;
            twentyCmCarFindRightUpCorner(); // 找上角点
            if (II.upLeft_y != YM)
                II.rightCrossDealFlag = 1;
        } while (0);
    }
    if (status == CM1) // 直入
    {
        if (II.right_y == YM)
            twentyCmCarRefindLeftDownCorner(); // 重新找下角点
        if (II.left_y == YM)
            twentyCmCarRefindRightDownCorner(); // 重新找下角点
        do
        {
            if (R_START_Y(0) > 20 && L_START_Y(0) > 20)
            {
                status = 6;
                break;
            }
            twentyCmCarFindLeftUpCorner();  // 找上角点
            twentyCmCarFindRightUpCorner(); // 找上角点
            if (II.upRight_y != YM)
                II.leftCrossDealFlag = 1;
            if (II.upLeft_y != YM)
                II.rightCrossDealFlag = 1;
        } while (0);
    }
    if (status == CM2) // 直  十字中
    {
        do
        {
            if ((LI.segCnt && L_START_Y(0) < 20 && II.lnum_all > 200) || (RI.segCnt && R_START_Y(0) < 20 && II.rnum_all > 200))
                goto CROSS_CLEAR;
            twentyCmCarFindLeftUpCorner();  // 找上角点
            twentyCmCarFindRightUpCorner(); // 找上角点
            if (II.upLeft_y != YM)
                II.rightCrossDealFlag = 1;
            if (II.upRight_y != YM)
                II.leftCrossDealFlag = 1;
        } while (0);
    }
    if (status == UR1)
    {
        do
        {
            if (isCross_right())
            {
                status = CR1;
                break;
            }
            if (timeToBrake_ucross())
            {
                status = UR2;
                break;
            }
            if (isNotUcross_Right())
                goto CROSS_CLEAR;
            twentyCmUnilateralCrossDeal(1, goRight);
        } while (0);
    }
    if (status == UR2)
    {
        do
        {
            if (isCross_right())
            {
                status = CR1;
                break;
            }
            if (timeToTurn_ucross())
            {
                status = UR3;
                break;
            }
            if (isNotUcross_Right())
                goto CROSS_CLEAR;
            twentyCmUnilateralCrossDeal(2, goRight);
        } while (0);
    }
    if (status == UR3)
    {
        for (uint8 i = 0; i < 4; i++)
            num_map[i] = num_map[i + 1];
        if (II.lnum[0] > 500 && LI.segCnt && (LI.numCnt[0] + LI.numCnt[1]) > 20 && II.start_lm[0] < 15)
            num_map[4] = II.num_lm;
        else
            num_map[4] = 0;
        if (num_map[0] == 0 && num_map[1] == 0 && num_map[2] == 0 && num_map[3] == 0 && num_map[4])
        {
            dir_ucross = 0;
            goto CROSS_CLEAR;
        }
        twentyCmUnilateralCrossDeal(3, goRight);
    }
    if (status == UL1)
    {
        do
        {
            if (isCross_left())
            {
                status = CL1;
                break;
            }
            if (timeToBrake_ucross())
            {
                status = UL2;
                break;
            }
            if (isNotUcross_Left())
                goto CROSS_CLEAR;
            twentyCmUnilateralCrossDeal(1, goLeft);
        } while (0);
    }
    if (status == UL2)
    {
        do
        {
            if (isCross_left())
            {
                status = CL1;
                break;
            }
            if (timeToTurn_ucross())
            {
                status = UL3;
                break;
            }
            if (isNotUcross_Left())
                goto CROSS_CLEAR;
            twentyCmUnilateralCrossDeal(2, goLeft);
        } while (0);
    }
    if (status == UL3)
    {
        for (uint8 i = 0; i < 4; i++)
            num_map[i] = num_map[i + 1];
        if (II.rnum[0] > 500 && RI.segCnt && (RI.numCnt[0] + RI.numCnt[1] > 20) && II.start_rm[0] < 15)
            num_map[4] = II.num_rm;
        else
            num_map[4] = 0;
        if (num_map[0] == 0 && num_map[1] == 0 && num_map[2] == 0 && num_map[3] == 0 && num_map[4])
        {
            dir_ucross = 0;
            goto CROSS_CLEAR;
        }
        twentyCmUnilateralCrossDeal(3, goLeft);
    }
#ifdef BOOM7_QT_DEBUG
    if (status)
        qout << "dealFlag" << "l:" << II.leftCrossDealFlag << "r:" << II.rightCrossDealFlag;
#endif

    if (fistJudgeFlag)
    {
        II.leftCrossDealFlag = 0;
        II.rightCrossDealFlag = 0;
    }
    if (II.leftCrossDealFlag)
        drawLine_cross_left();
    if (II.rightCrossDealFlag)
        drawLine_cross_right();
    // 记录上一次的下角点，用于2状态补线点
    if (II.left_y != YM)
    {
        lastDownLeft_x = II.left_x;
        lastDownLeft_y = II.left_y;
    }
    if (II.right_y != YM)
    {
        lastDownRight_x = II.right_x;
        lastDownRight_y = II.right_y;
    }
    return status;
CROSS_CLEAR:
    // 解除十字
    status = 0;
    IF.crossroad = 0;
    crossLastKeyPointInit(); // 对可能存在的“上一帧关键点”进行初始化
    II.leftCrossDealFlag = 0;
    II.rightCrossDealFlag = 0;
    IF.crossAnticipation = 0;
    for (uint8 i = 0; i < 5; i++)
        num_map[i] = 255;
    cross_clearFlag = 0; // 防止重复进入
    return 0;
}

/*************************************************
Function: uint8 twentyCmCrossroadJudge()
Description: 十字判定
Input: null
Output: null
Return: 是否为十字
Others: return: 1:根据左边特征判斜入十字 2:右边特征判斜入十字 3：正入十字
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmCrossroadJudge()
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCrossroadJudge";
#endif
    if (II.top < 50 || (!II.leftPortaitSearchLineFlag && !II.rightPortaitSearchLineFlag) || (LI.segCnt < 2 && RI.segCnt < 2))
        return 0;
    uint8 flag_l = 0;
    uint8 flag_r = 0;

    if (II.leftPortaitSearchLineFlag && LI.segCnt >= 2 && II.angleL > 75 && II.angleL < 105 && L_START_Y(0) < 15 && II.right_x < XX - 10 && II.right_y < 45)
        flag_l = isCross_left();
    if (II.rightPortaitSearchLineFlag && RI.segCnt >= 2 && II.angleR > 75 && II.angleR < 105 && R_START_Y(0) < 15 && II.left_x > 10 && II.left_y < 45)
        flag_r = isCross_right();
#ifdef BOOM7_QT_DEBUG
    qout << "l" << flag_l << "r" << flag_r;
#endif
    if (flag_l == 1 && flag_r == 1)
        return 3;
    if (flag_l == 1)
        return 1;
    if (flag_r == 1)
        return 2;
    return 0;
}

// 十字的左边线判定
uint8 isCross_left()
{
#ifdef BOOM7_QT_DEBUG
    qout << "left";
#endif
    if (II.right_y > STOP_LINE)
        return 0;
    // 找上下关键线段
    uint8 seg_up_l = 255;
    uint8 seg_down_l = 255;
    float k1_l;
    float k2_l;
    float k3_l;
    for (uint8 i = 0; i < LI.segCnt; i++)
        if (seg_down_l == 255 && LI.start[i] < 30)
            seg_down_l = i;
        else if (seg_down_l != 255 && seg_up_l == 255 && LI.start[i] > 30 && L_START_X(i) > L_END_X(seg_down_l))
            seg_up_l = i;
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "seg_down_l" << seg_down_l << "seg_up_l" << seg_up_l;
#endif
    // 找不到或者说不满足要求，退出
    if (seg_down_l == 255 || seg_up_l == 255 || (RT_Y(0) < 55 && L_START_Y(seg_up_l) < RT_Y(0) + 5) || II.leftEndCol > PL.right_x[seg_down_l] + 10)
        return 0;
    /*********************************************************************
                                核心思路
         下方段起点到下方段终点，下方段终点到上方段起点，上方段起点到上方段终点
                 十字关键就是这三段，斜率要相近，起码符号要一样
     *********************************************************************/
    // 找各个线段起始点和终点，不是之前求得的起点终点，是满足单调增或减的部分的起点终点
    uint8 end_down = L_END_Y(seg_down_l);
    uint8 start_down = L_START_Y(seg_down_l);
    uint8 end_up = L_END_Y(seg_up_l);
    uint8 start_up = L_START_Y(seg_up_l);

    for (uint8 row = L_END_Y(seg_down_l) - 1; row >= L_START_Y(seg_down_l) && row != 255; row--)
        if (II.leftline[row + 1] <= II.leftline[row])
            end_down = row;
        else
            break;

    for (uint8 row = L_START_Y(seg_up_l); row <= end_up - 3; row++)
        if (II.leftline[row + 1] == II.leftline[row] || II.leftline[row + 1] == II.leftline[row] + 1)
        {
            start_up = row;
            end_up = start_up + 5;
            break;
        }

    for (uint8 row = start_up + 1; row < start_up + 5 && row < L_END_Y(seg_up_l); row++)
    {
        end_up = row;
        if (II.leftline[row + 1] < II.leftline[row])
            break;
    }
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "start_down" << start_down << "end_down" << end_down;
    qout << "start_up" << start_up << "end_up" << end_up;
#endif
    if (end_down - start_down < 6 || end_up - start_up < 2)
        return 0;
    // 计算斜率
    k1_l = get_k(II.leftline[start_down], II.leftline[end_down], start_down, end_down);
    k2_l = get_k(II.leftline[end_down], II.leftline[start_up], end_down, start_up);
    k3_l = get_k(II.leftline[start_up], II.leftline[end_up], start_up, end_up);
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "k1" << k1_l << "k2" << k2_l << "k3" << k3_l;
#endif
    if (k1_l > 0 && k2_l > 0 && fabs(k1_l - k2_l) < 0.6 && (k3_l > 0 || k3_l == 10000))
        return 1;
    return 0;
}

uint8 isCross_right()
{
#ifdef BOOM7_QT_DEBUG
    qout << "right";
#endif
    if (II.left_y > STOP_LINE)
        return 0;
    uint8 seg_up_r = 255;
    uint8 seg_down_r = 255;
    float k1_r;
    float k2_r;
    float k3_r;
    for (uint8 i = 0; i < RI.segCnt; i++)
        if (seg_down_r == 255 && RI.start[i] < 30)
            seg_down_r = i;
        else if (seg_down_r != 255 && seg_up_r == 255 && RI.start[i] > 30 && R_START_X(i) < R_END_X(seg_down_r))
            seg_up_r = i;
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "seg_down_r" << seg_down_r << "seg_up_r" << seg_up_r;
#endif
    if (seg_down_r == 255 || seg_up_r == 255 || (LT_Y(0) < 55 && R_START_Y(seg_up_r) < LT_Y(0) + 5) || II.rightEndCol < PR.left_x[seg_down_r] + 10)
        return 0;
    uint8 end_down = R_END_Y(seg_down_r);
    uint8 start_down = R_START_Y(seg_down_r);
    uint8 end_up = R_END_Y(seg_up_r);
    uint8 start_up = R_START_Y(seg_up_r);

    for (uint8 row = R_END_Y(seg_down_r) - 1; row >= R_START_Y(seg_down_r) && row != 255; row--)
        if (II.rightline[row + 1] >= II.rightline[row])
            end_down = row;
        else
            break;

    for (uint8 row = R_START_Y(seg_up_r); row <= end_up - 3; row++)
        if (II.rightline[row + 1] == II.rightline[row] || II.rightline[row + 1] == II.rightline[row] + 1)
        {
            start_up = row;
            end_up = start_up + 5;
            break;
        }

    for (uint8 row = start_up + 1; row < start_up + 5 && row < R_END_Y(seg_up_r); row++)
    {
        end_up = row;
        if (II.rightline[row + 1] > II.rightline[row])
            break;
    }
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "start_down" << start_down << "end_down" << end_down;
    qout << "start_up" << start_up << "end_up" << end_up;
#endif
    if (end_down - start_down < 6 || end_up - start_up < 2)
        return 0;
    k1_r = get_k(II.rightline[start_down], II.rightline[end_down], start_down, end_down);
    k2_r = get_k(II.rightline[end_down], II.rightline[start_up], end_down, start_up);
    k3_r = get_k(II.rightline[start_up], II.rightline[end_up], start_up, end_up);
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "k1" << k1_r << "k2" << k2_r << "k3" << k3_r;
#endif
    if (k1_r < 0 && k2_r < 0 && fabs(k1_r - k2_r) < 0.6 && (k3_r < 0 || k3_r == 10000))
        return 1;
    return 0;
}

uint8 twentyCmCarFindLeftUpCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCarFindLeftUpCorner";
#endif
    //
    if (!II.leftPortaitSearchLineFlag || HL.segCnt == 0)
    {
        II.upRight_x = XM;
        II.upRight_y = YM;
        return 0;
    }
    uint8 LI_keySeg = 0;
    if (LI.segCnt == 1 || L_START_Y(0) > 20)
        LI_keySeg = 0;
    else
        for (uint8 seg = 1; seg < LI.segCnt; seg++)
            if (L_START_Y(seg) > 20 && PL.right_x[seg] > PL.right_x[0])
            {
                LI_keySeg = seg;
                break;
            }
    uint8 keySeg = 0;
    float tmp = 100;
    for (uint8 seg = 0; seg < HL.segCnt; seg++)
    {
        uint8 dist = distance(HL_START_X(seg), HL_START_Y(seg), L_START_X(LI_keySeg), L_START_Y(LI_keySeg));
        if (dist < tmp)
        {
            tmp = dist;
            keySeg = seg;
        }
    }

    II.upRight_x = II.leftline[HL_START_Y(keySeg) + 2];
    II.upRight_y = HL_START_Y(keySeg) + 2;
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "LI_keySeg:" << LI_keySeg << "HL_keyseg:" << keySeg;
    qout << "x dist:" << abs((int)II.upRight_x - (int)HL_START_X(keySeg));
    qout << "HL_START_Y" << HL_START_Y(keySeg) << "L_START_Y" << L_START_Y(LI_keySeg);
#endif
    if (II.upRight_y < YM && abs((int)II.upRight_x - (int)HL_START_X(keySeg)) < 10 && HL_START_Y(keySeg) - 2 <= L_START_Y(LI_keySeg))
        return 1;
    II.upRight_x = XM;
    II.upRight_y = YM;
    return 0;
}

uint8 twentyCmCarFindRightUpCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCarFindRightUpCorner()";
#endif
    if (!II.rightPortaitSearchLineFlag || HR.segCnt == 0)
    {
        II.upLeft_x = XM;
        II.upLeft_y = YM;
        return 0;
    }
    uint8 RI_keySeg = 0;
    if (RI.segCnt == 1 || R_START_Y(0) > 20)
        RI_keySeg = 0;
    else
        for (uint8 seg = 1; seg < RI.segCnt; seg++)
            if (R_START_Y(seg) > 20 && PR.left_x[seg] < PR.left_x[0])
            {
                RI_keySeg = seg;
                break;
            }

    uint8 keySeg = 0;
    float tmp = 100;
    for (uint8 seg = 0; seg < HR.segCnt; seg++)
    {
        uint8 dist = distance(HR_START_X(seg), HR_START_Y(seg), R_START_X(RI_keySeg), R_START_Y(RI_keySeg));
        if (dist < tmp)
        {
            tmp = dist;
            keySeg = seg;
        }
    }
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "RI_keySeg:" << RI_keySeg << "HR_keyseg:" << keySeg;
#endif
    II.upLeft_x = II.rightline[HR_START_Y(keySeg) + 2];
    II.upLeft_y = HR_START_Y(keySeg) + 2;
    if (II.upLeft_y < YM && abs((int)II.upLeft_x - (int)HR_START_X(keySeg)) < 10 && HR_START_Y(keySeg) - 2 <= R_START_Y(RI_keySeg))
        return 1;
    II.upLeft_x = XM;
    II.upLeft_y = YM;
    return 0;
}

void twentyCmCarRefindLeftDownCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCarRefindLeftDownCorner";
#endif
    if (II.lnum_all < 30)
        return;
    if (II.num_lm > 0 && II.start_lm[0] < 40 && RB_Y(0) < 40)
    {
        II.right_x = RB_X(0) + 1;
        II.right_y = RB_Y(0);
    }
    else if (BL.segCnt != 0)
    {
        II.right_x = II.leftline[BL_START_Y(0) - 1];
        II.right_y = BL_START_Y(0) - 1;
    }
}

void twentyCmCarRefindRightDownCorner()
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCarRefindRightDownCorner";
#endif
    if (II.rnum_all < 30)
        return;
    if (II.num_rm > 0 && II.start_rm[0] < 40 && LB_Y(0) < 40)
    {
        II.left_x = LB_X(0) - 1;
        II.left_y = LB_Y(0);
    }
    else if (BR.segCnt != 0)
    {
        II.left_x = II.rightline[BR_START_Y(0) - 1];
        II.left_y = BR_START_Y(0) - 1;
    }
}

void drawLine_cross_left()
{
    if (II.right_y <= 3 || II.right_y == YM)
    {
        II.right_y = lastDownRight_y;
        II.right_x = lastDownRight_x;
    }
    else if (II.right_y >= 3 && II.leftfindflag[II.right_y - 3])
    {
        II.right_y -= 3;
        II.right_x = II.leftline[II.right_y];
    }
    drawline(II.upRight_x, II.upRight_y, II.right_x, II.right_y, leftmap);
}

void drawLine_cross_right()
{
    if (II.left_y <= 3 || II.left_y == YM)
    {
        II.left_y = lastDownLeft_y;
        II.left_x = lastDownLeft_x;
    }
    else if (II.left_y >= 3 && II.rightfindflag[II.left_y - 3])
    {
        II.left_y -= 3;
        II.left_x = II.rightline[II.left_y];
    }
    drawline(II.upLeft_x, II.upLeft_y, II.left_x, II.left_y, rightmap);
}

uint8 twentyCmUcrossJudge()
{
    // 大排除
    if (II.dnum_all < 200 || II.breakY > 57)
        return 0;
    // 根据左右图的点数区分用哪个判据
    if (II.num_lm && II.lnum_all > 150 && II.rnum_all > 150 && II.num_rm)
        return isUcross_Mid();
    else if (II.num_lm && II.lnum_all > 150 && II.rnum_all < 150)
        return isUCross_left();
    else if (II.num_rm && II.rnum_all > 150 && II.lnum_all < 150)
        return isUCross_right();
    return 0;
}

uint8 isUcross_Mid()
{
#ifdef BOOM7_QT_DEBUG
    qout << "isUCross_Mid";
#endif
    // 角点和角度限制
    if (II.left_y == YM || II.right_y == YM)
        return 0;
    uint8 flagL = 0, flagR = 0;
    if (II.angleL && II.angleL < 105)
        flagL = 1;
    if (II.angleR && II.angleR < 105)
        flagR = 1;
    if (!flagL && !flagR)
        return 0;
    // 直线判定
    // 左边
    Point v_l, p1_l;
    uint8 up = II.right_y, down = II.right_y;
    for (uint8 i = II.right_y; i < YM; ++i)
        if (!leftmap[i][II.right_x + 1])
        {
            up = i;
            break;
        }
    for (uint8 i = II.right_y - 1; i < YM; --i)
        if (!leftmap[i][II.right_x + 1])
        {
            down = i;
            break;
        }
    v_l.x = II.right_x + 1;
    v_l.y = (up + down) / 2;
    // 看在哪一段线里面
    uint8 seg = getSegOfPoint(v_l.y, goLeft);
    // 第一个点设为该段起点
    p1_l.x = L_START_X(seg);
    p1_l.y = L_START_Y(seg);
    if (!strJudge(p1_l.x, p1_l.y, v_l.x, v_l.y - 1, leftmap, p1_l.y, v_l.y - 1, 2, (v_l.y - p1_l.y) / 5))
        return 0;
    // 右边
    Point v, p1_r;
    // 顶点
    up = II.left_y, down = II.left_y;
    for (uint8 i = II.left_y; i < YM; ++i)
        if (!leftmap[i][II.left_x - 1])
        {
            up = i;
            break;
        }
    for (uint8 i = II.left_y - 1; i < YM; --i)
        if (!leftmap[i][II.left_x - 1])
        {
            down = i;
            break;
        }
    v.x = II.left_x - 1;
    v.y = (up + down) / 2;
    // 看在哪一段线里面
    seg = getSegOfPoint(v.y, goRight);
    // 第一个点设为该段起点
    p1_r.x = R_START_X(seg);
    p1_r.y = R_START_Y(seg);
    if (!strJudge(p1_r.x, p1_r.y, v.x, v.y - 1, rightmap, p1_r.y, v.y - 1, 2, (v.y - p1_r.y) / 5))
        return 0;

    // 判deletemap里的直线
    // 找deletemap左边的点
    // 求左图角点以下线段的斜率截距

    uint8 basecol_l, basecol_r;
    Point deletemap_point_left, deletemap_point_right;
    uint8 corner_left_x = RB_X(0), corner_left_y = RB_Y(0);
    if (II.right_y != YM)
    {
        corner_left_x = II.right_x;
        corner_left_y = II.right_y;
    }
    else if (RB_Y(0) > 50 && LI.segCnt)
    {
        corner_left_x = PL.right_x[0];
        corner_left_y = PL.right_y[0];
    }
    if (corner_left_x < rigor_ucross_both / k1[corner_left_y])
        basecol_l = 0;
    else
        basecol_l = corner_left_x - rigor_ucross_both / k1[corner_left_y];
    float k_l = ((float)corner_left_x - (float)BR_X(0)) / ((float)corner_left_x - (float)BR_Y(0));
    float b_l = basecol_l - k_l * corner_left_y;
    for (uint8 row = corner_left_y; row < YM; row++)
    {
        uint8 col = k_l * row + b_l + 0.5;
        if (deletemap[row][col])
        {
            deletemap_point_left.x = col;
            deletemap_point_left.y = row;
            break;
        }
    }
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "corner_left_x" << corner_left_x << "corner_left_y" << corner_left_y;
    qout << "deletemap_point_left" << deletemap_point_left.x << deletemap_point_left.y;
#endif
    // 右边的点
    uint8 corner_right_x = LB_X(0), corner_right_y = LB_Y(0);
    if (II.left_y != YM)
    {
        corner_right_x = II.left_x;
        corner_right_y = II.left_y;
    }
    else if (LB_Y(0) > 50 && RI.segCnt)
    {
        corner_right_x = PR.left_x[0];
        corner_right_y = PR.left_y[0];
    }
    if (XX - corner_right_x < rigor_ucross_both / k1[corner_right_y])
        basecol_r = XX;
    else
        basecol_r = corner_right_x + rigor_ucross_both / k1[corner_right_y];
    float k_r = ((float)corner_right_x - (float)BL_X(0)) / ((float)corner_right_y - (float)BL_Y(0));
    float b_r = basecol_r - k_r * LB_Y(0);
    for (uint8 row = corner_right_y; row < YM; row++)
    {
        uint8 col = k_r * row + b_r + 0.5;
        if (deletemap[row][col])
        {
            deletemap_point_right.x = col;
            deletemap_point_right.y = row;
            break;
        }
    }
#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "corner_right_x" << corner_right_x << "corner_right_y" << corner_right_y;
    qout << "deletemap_point_right" << deletemap_point_right.x << deletemap_point_right.y;
#endif
    if (distance(deletemap_point_right.x, deletemap_point_right.y, deletemap_point_left.x, deletemap_point_left.y) < 3)
        return 0;
    return strJudge_X(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y,
                      deletemap, deletemap_point_left.x, deletemap_point_right.x, 1, (deletemap_point_right.x - deletemap_point_left.x) / 10);
}

uint8 isUCross_left()
{
#ifdef BOOM7_QT_DEBUG
    qout << "isUCross_left";
#endif
    // 角度限制
    if (II.right_y == YM)
        return 0;
    if (II.angleL > 105)
        return 0;

    uint8 wbwCnt = 0;
    for (uint8 i = BOTTOM_LM(0); i <= TOP_LM(0); i++)
        if (X_WBW_Detect(RIGHT(0), 0, i, leftmap, goLeft))
            wbwCnt++;
    if (wbwCnt > 15)
        return 0;

    Point v, p1;
    uint8 up = II.right_y, down = II.right_y;
    for (uint8 i = II.right_y; i < YM; ++i)
        if (!leftmap[i][II.right_x + 1])
        {
            up = i;
            break;
        }
    for (uint8 i = II.right_y - 1; i < YM; --i)
        if (!leftmap[i][II.right_x + 1])
        {
            down = i;
            break;
        }
    v.x = II.right_x + 1;
    v.y = (up + down) / 2;

    // 看在哪一段线里面
    uint8 seg = getSegOfPoint(v.y, goLeft);
    // 第一个点设为该段起点
    p1.x = L_START_X(seg);
    p1.y = L_START_Y(seg);
    if (distance(p1.x, p1.y, v.x, v.y) < 10)
        return 0;
    if (!strJudge(p1.x, p1.y, v.x, v.y - 1, leftmap, p1.y, v.y - 1, 1, (v.y - p1.y) / 15))
        return 0;

    uint8 basecol_l;
    Point deletemap_point_left, deletemap_point_right;
    // 先右边的，这种主要靠左图来判的话，右侧选点就定为deletemap中右图最左点横坐标下的最低点

    deletemap_point_right.x = LT_X(0);
    deletemap_point_right.y = getMapYMin_Col(deletemap_point_right.x, deletemap, 2);
    if (deletemap_point_right.y == YM)
        deletemap_point_right.y = getMapYMin_Col(deletemap_point_right.x, deletemap, 253);
#ifdef BOOM7_QT_DEBUG
    qout << "deletemap_point_right" << deletemap_point_right.x << deletemap_point_right.y;
#endif
    if (deletemap_point_right.y == YM)
        return 0;

    uint8 corner_left_x = RB_X(0), corner_left_y = RB_Y(0);
    if (II.right_y != YM)
    {
        corner_left_x = II.right_x;
        corner_left_y = II.right_y;
    }
    else if (RB_Y(0) > 50 && LI.segCnt)
    {
        corner_left_x = PL.right_x[0];
        corner_left_y = PL.right_y[0];
    }

    if (corner_left_x < rigor_ucross_oneSide / k1[corner_left_y])
        basecol_l = 0;
    else
        basecol_l = corner_left_x - rigor_ucross_oneSide / k1[corner_left_y];

    float k_l = ((float)corner_left_x - (float)BR_X(0)) / ((float)corner_left_x - (float)BR_Y(0));
    float b_l = basecol_l - k_l * corner_left_y;

    for (uint8 row = corner_left_y; row < YM; row++)
    {
        uint8 col = k_l * row + b_l + 0.5;
        if (deletemap[row][col])
        {
            deletemap_point_left.x = col;
            deletemap_point_left.y = row;
            break;
        }
    }

    v.x = II.right_x + 1;
    v.y = getMapYMin_Col(v.x, leftmap, 2);
    float k = get_k(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y);
    float k_down = getRealK(v, p1);
    float k_up = getRealK(deletemap_point_left, deletemap_point_right);

#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "corner_left_x" << corner_left_x << "corner_left_y" << corner_left_y;
    qout << "deletemap_point_left" << deletemap_point_left.x << deletemap_point_left.y;
    qout << "k_up" << k_up << "k_down" << k_down << "mul" << k_up * k_down;
#endif

    if (fabs(k) > 0.8)
        return 0;
    if (k_up * k_down + 1 > 1.5 || k_up * k_down + 1 < -1.5)
        return 0;
    if (distance(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y) < 10)
        return 0;
    return strJudge_X(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y,
                      deletemap, deletemap_point_left.x, deletemap_point_right.x, 1, (deletemap_point_right.x - deletemap_point_left.x) / 15);
}

uint8 isUCross_right()
{
#ifdef BOOM7_QT_DEBUG
    qout << "isUCross_right";
#endif
    if (II.left_y == YM)
        return 0;
    if (II.angleR > 105)
        return 0;

    uint8 wbwCnt = 0;
    for (uint8 i = BOTTOM_RM(0); i <= TOP_RM(0); i++)
        if (X_WBW_Detect(LEFT(0), XX, i, rightmap, goRight))
            wbwCnt++;
    if (wbwCnt > 15)
        return 0;

    Point v, p1;
    // 顶点
    uint8 up = II.left_y, down = II.left_y;
    for (uint8 i = II.left_y; i < YM; ++i)
        if (!leftmap[i][II.left_x - 1])
        {
            up = i;
            break;
        }
    for (uint8 i = II.left_y - 1; i < YM; --i)
        if (!leftmap[i][II.left_x - 1])
        {
            down = i;
            break;
        }
    v.x = II.left_x - 1;
    v.y = (up + down) / 2;

    // 看在哪一段线里面
    uint8 seg = getSegOfPoint(v.y, goRight);
    // 第一个点设为该段起点
    p1.x = R_START_X(seg);
    p1.y = R_START_Y(seg);
    if (distance(p1.x, p1.y, v.x, v.y) < 10)
        return 0;
    if (!strJudge(p1.x, p1.y, v.x, v.y - 1, rightmap, p1.y, v.y - 1, 1, (v.y - p1.y) / 15))
        return 0;

    uint8 basecol_r;
    Point deletemap_point_left, deletemap_point_right;

    deletemap_point_left.x = RT_X(0);
    deletemap_point_left.y = getMapYMin_Col(deletemap_point_left.x, deletemap, 2);
    if (deletemap_point_left.y == YM)
        deletemap_point_left.y = getMapYMin_Col(deletemap_point_left.x, deletemap, 253);
#ifdef BOOM7_QT_DEBUG
    qout << "deletemap_point_left" << deletemap_point_left.x << deletemap_point_left.y;
#endif
    if (deletemap_point_left.y == YM)
        return 0;

    uint8 corner_right_x = LB_X(0), corner_right_y = LB_Y(0);
    if (II.left_y != YM)
    {
        corner_right_x = II.left_x;
        corner_right_y = II.left_y;
    }
    else if (LB_Y(0) > 50 && RI.segCnt)
    {
        corner_right_x = PR.left_x[0];
        corner_right_y = PR.left_y[0];
    }

    if (XX - corner_right_x < rigor_ucross_oneSide / k1[corner_right_y])
        basecol_r = XX;
    else
        basecol_r = corner_right_x + rigor_ucross_oneSide / k1[corner_right_y];

    float k_r = ((float)corner_right_x - (float)BL_X(0)) / ((float)corner_right_y - (float)BL_Y(0));
    float b_r = basecol_r - k_r * LB_Y(0);

    for (uint8 row = corner_right_y; row < YM; row++)
    {
        uint8 col = k_r * row + b_r + 0.5;
        if (deletemap[row][col])
        {
            deletemap_point_right.x = col;
            deletemap_point_right.y = row;
            break;
        }
    }

    v.x = II.left_x - 1;
    v.y = getMapYMin_Col(v.x, rightmap, 2);

    float k = get_k(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y);
    float k_down = getRealK(v, p1);
    float k_up = getRealK(deletemap_point_left, deletemap_point_right);

#ifdef BOOM7_QT_DEBUG_CROSS
    qout << "corner_right_x" << corner_right_x << "corner_right_y" << corner_right_y;
    qout << "deletemap_point_right" << deletemap_point_right.x << deletemap_point_right.y;
    qout << "k" << k << "k_up" << k_up << "k_down" << k_down << "mul" << k_up * k_down;
#endif

    if (fabs(k) > 0.8)
        return 0;
    if (k_up * k_down + 1 > 1.5 || k_up * k_down + 1 < -1.5)
        return 0;
    if (distance(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y) < 10)
        return 0;
    return strJudge_X(deletemap_point_left.x, deletemap_point_left.y, deletemap_point_right.x, deletemap_point_right.y,
                      deletemap, deletemap_point_left.x, deletemap_point_right.x, 1, (deletemap_point_right.x - deletemap_point_left.x) / 15);
}

/*************************************************
Function: timeToTurn_ucross()
Description: 单边十字出口补线时机
Input: 方向
Output: null
Return: 单边十字状态是否转换
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 timeToTurn_ucross()
{
#ifdef BOOM7_QT_DEBUG
    qout << "timeToTurn_ucross";
#endif
    Point left, right;
    left.x = 0;
    left.y = 0;
    right.x = XX;
    right.y = 0;
    // 找点
    if (II.num_lm && II.lnum[0] > 50 && II.start_lm[0] < 10)
    {
        if (II.right_y != YM)
        {
            left.x = II.right_x;
            left.y = II.right_y;
        }
        else
            left = RT(0);
    }

    if (II.num_rm && II.rnum[0] > 50 && II.start_rm[0] < 10)
    {
        if (II.left_y != YM)
        {
            right.x = II.left_x;
            right.y = II.left_y;
        }
        else
            right = LT(0);
    }
    //    top_x = (left.x+right.x)/2;
    //    top_y = getMapYMin_Col(top_x,deletemap,1);
    //    if(top_y == YM) top_y = getMapYMin_Col(top_x,deletemap,254);
    //    if(top_y == YM) top_y = getMapYMin_Col(top_x,leftmap,1);
    //    if(top_y == YM) top_y = getMapYMin_Col(top_x,rightmap,1);
    //    if(top_y == YM) return 0;
    Point mid, bottom;
    if (left.y > 0 && right.y > 0)
    {
        mid.x = (left.x + right.x) / 2;
        mid.y = (left.y + right.y) / 2;
    }
    else if (left.y > 0)
    {
        mid.x = (XM - left.x) / 2 + left.x;
        mid.y = left.y * 0.85;
    }
    else if (right.y > 0)
    {
        mid.x = right.x / 2;
        mid.y = right.y * 0.85;
    }
    else
    {
        mid.x = myCarMid;
        mid.y = YY;
    }

    bottom.x = II.midline[0];
    bottom.y = 0;

    PointF bottom_real = getRealPoint(bottom);
    PointF mid_real = getRealPoint(mid);
    float dist = distance(bottom_real.x, bottom_real.y, mid_real.x, mid_real.y);
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "left:" << left.x << left.y << "right:" << right.x << right.y;
    qout << "mid:" << mid.x << mid.y;
    qout << "bottom:" << bottom.x << bottom.y;
    qout << "dist:" << dist;
#endif
    if (dist < drawLine_distance_ucross)
        return 1;
    return 0;
}

/*************************************************
Function: timeToBrake_ucross(uint8 dir)
Description: 单边十字出口刹车时机
Input: 方向
Output: null
Return: 单边十字状态是否转换
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 timeToBrake_ucross()
{
    float dist = 0;
    Point left, right;
    left.x = XM;
    left.y = YM;
    right.x = XM;
    right.y = YM;
    // 找点
    if (II.num_lm && II.lnum[0] > 50 && II.start_lm[0] < 20)
    {
        left.x = RB_X(0);
        left.y = RB_Y(0);
    }

    if (II.num_rm && II.rnum[0] > 50 && II.start_rm[0] < 20)
    {
        right.x = LB_X(0);
        right.y = LB_Y(0);
    }

    // 算车头到路口的距离
    if (left.x != XM && right.x != XM) // 两个点都有
    {
        int x = (left.x + right.x) / 2;
        int y = (left.y + right.y) / 2;
        dist = sqrt(k2[y] * k2[y] + (x - myCarMid) * (x - myCarMid));
    }
    else if (left.x != XM)
    {
        int p1_x_real = ((int)left.x - leftline[left.y]) * k1[left.y];
        int p1_y_real = k2[left.y];
        int p2_x_real = ((int)BR(0).x - leftline[BR(0).y]) * k1[BR(0).y];
        int p2_y_real = k2[BR(0).y];
        float tan;
        if (p1_x_real != p2_x_real)
            tan = (p1_y_real - p2_y_real) / (p1_x_real - p2_x_real);
        else
            tan = 999;
        dist = distance(p1_x_real, p1_y_real, p2_x_real, p2_y_real) - 20 / tan;
        //        qout<<tan<<dist;
    }
    else if (right.x != XM)
    {
        int p1_x_real = ((int)right.x - leftline[left.y]) * k1[right.y];
        int p1_y_real = k2[right.y];
        int p2_x_real = ((int)BL(0).x - leftline[BL(0).y]) * k1[BL(0).y];
        int p2_y_real = k2[BL(0).y];
        float tan;
        if (p1_x_real != p2_x_real)
            tan = (p1_y_real - p2_y_real) / (p2_x_real - p1_x_real);
        else
            tan = 999;
        dist = distance(p1_x_real, p1_y_real, p2_x_real, p2_y_real) - 20 / tan;
    }
    else
        return 0;
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "left:" << left.x << left.y << "right:" << right.x << right.y;
    qout << "dist:" << dist;
#endif
    if (dist && dist < brake_distance_ucross)
        return 1;
    return 0;
}

/*************************************************
Function: isOut_ucross(uint8 dir)
Description: 单边十字状态结束判据
Input: 方向
Output: null
Return: 单边十字是否结束
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 isOut_ucross(uint8 dir)
{
    if (dir == goRight)
    {
        if (II.num_lm && II.start_lm[0] == 0 && II.lnum_all > 250)
        {
            return 1;
        }
    }
    if (dir == goLeft)
    {
        if (II.num_rm && II.start_rm[0] == 0 && II.rnum_all > 250)
        {
            return 1;
        }
    }
    return 0;
}

/*************************************************
Function: uint8 isNotUcross_Left()
Description: 单边十字状态解除判据，针对12状态看是不是该解除
Input: 更新的方向，是否可以被改变
Output: null
Return: null
Others: 判定比较松，靠这个排下误判
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 isNotUcross_Left()
{
    if (II.d_bottom[0].y < II.right_y - 10 && II.right_y != YM)
        return 1;
    //    if(II.dnum_all<100 && II.num_rm && II.start_rm[0]>20 && II.rnum[0]>50) return 1;
    return 0;
}

uint8 isNotUcross_Right()
{
    if (II.d_bottom[0].y < II.left_y - 10 && II.left_y != YM)
        return 1;
    //    if(II.dnum_all<100 && II.num_lm && II.start_lm[0]>20 && II.lnum[0]>50) return 1;
    return 0;
}

void check_dir_ucross()
{
    if (IF.annulus || IF.fork || IF.garage || IF.ramp)
        dir_ucross_clearFlag = 1;
    if (IF.crossroad == CL2 || IF.crossroad == CR2 || IF.crossroad == CM1 || IF.crossroad == CM2)
        dir_ucross_clearFlag = 1;
    if (dir_ucross_clearFlag)
    {
        dir_ucross_Update(0, 2);
        dir_ucross_clearFlag = 0;
        return;
    }

    if (!IF.crossroad && twentyCmCrossAnticipation_left())
        dir_ucross_Update(goLeft, 0);
    if (!IF.crossroad && twentyCmCrossAnticipation_right())
        dir_ucross_Update(goRight, 0);
    if (twentyCmEnterLeftUcrossJudge())
        dir_ucross_Update(goRight, 1);
    if (twentyCmEnterRightUcrossJudge())
        dir_ucross_Update(goLeft, 1);
}

/*************************************************
Function: void dir_ucross_Update(uint8 dir, uint8 isUnchanged)
Description: 更新单边十字的方向
Input: 更新的方向，是否可以被改变
Output: null
Return: null
Others: 怕这个单边十字，里面有很多东西，虽然卓晴已经说不会有了，不过还是做好万全准备
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void dir_ucross_Update(uint8 dir, uint8 state_lock)
{
    // 第二个参数的值为上锁或解锁，规定上锁为1，解锁为2，对锁不做操作为0
    // 解锁时赋值方向为0，也就是未知,上锁可覆盖
    static uint8 lock = 0;

    if (state_lock == 2)
    {
        lock = 0;
        dir_ucross = 0;
        return;
    }
    if (!lock || state_lock == 1)
        dir_ucross = dir;

    if (state_lock == 1)
        lock = 1;
}

/*************************************************
Function: uint8 getUcrossDirByDrivingRecorder()
Description: 倒数第二手，根据行车记录仪判断方向
Input: null
Output: null
Return: 方向
Others: 怕这个单边十字，里面有很多东西，虽然卓晴已经说不会有了，不过还是做好完全准备
        最后一手就是直接人为eeprom里设定了
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 getUcrossDirByDrivingRecorder()
{
    for (int i = roadRecorder.top; i > 0; i--)
        if (roadRecorder.all[i] == enterLeftCorner || roadRecorder.all[i] == inLeftCorner || roadRecorder.all[i] == outLeftCorner)
            return goRight;
        else if (roadRecorder.all[i] == enterRightCorner || roadRecorder.all[i] == inRightCorner || roadRecorder.all[i] == outRightCorner)
            return goLeft;
    return goLeft;
}

/*************************************************
Function: uint8 twentyCmCrossAnticipation_left(void)
Description: 单边十字出口预判
0Input: null
Output: null
Return: 单边十字的状态
Others: 只对出口进行判定
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmCrossAnticipation_left(void)
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCrossAnticipation_left";
#endif
    if (II.angleL > 100 || II.angleL < 80)
        return 0;

    if (II.num_lm == 1 && II.start_lm[0] < 10 && II.leftPortaitSearchLineFlag && II.angleL && II.lnum_all > 200)
    {
        Point v, p1;
        v.x = II.right_x;
        v.y = II.right_y;
        // 看在哪一段线里面
        uint8 seg = getSegOfPoint(v.y, goLeft);

        // 第一个点设为该段起点
        p1.x = L_START_X(seg);
        p1.y = L_START_Y(seg);

        // 直线的判定
        if (strJudge(p1.x, p1.y, v.x, v.y - 1, leftmap, p1.y, v.y - 1, 2, (v.y - p1.y) / 5))
            return 1;
    }
    return 0;
}
uint8 twentyCmCrossAnticipation_right(void)
{
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmCrossAnticipation_right";
#endif
    if (II.angleR > 100 || II.angleR < 80)
        return 0;
    if (II.num_rm == 1 && II.start_rm[0] < 10 && II.rightPortaitSearchLineFlag && II.angleR && II.rnum_all > 200)
    {
        Point v, p1;
        // 顶点
        v.x = II.left_x;
        v.y = II.left_y;
        // 看在哪一段线里面
        uint8 seg = getSegOfPoint(v.y, goRight);

        // 第一个点设为该段起点
        p1.x = R_START_X(seg);
        p1.y = R_START_Y(seg);

        // 直线的判定
        if (strJudge(p1.x, p1.y, v.x, v.y - 1, rightmap, p1.y, v.y - 1, 2, (v.y - p1.y) / 5))
            return 1;
    }
    return 0;
}

/*************************************************
Function: uint8 twentyCmEnterLeftUcrossJudge()
Description: 判单边十字的入口，判的准一点，别把入环前面和普通的斜入十字判了
Input: 更新的方向，是否可以被改变
Output: null
Return: null
Others: 怕这个单边十字，里面有很多东西，虽然卓晴已经说不会有了，不过还是做好万全准备
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmEnterLeftUcrossJudge()
{
    if (IF.crossroad != CL1)
        return 0;

    // 只判中间或者偏不多的吧，真遇到急转弯入这玩意，就靠预判或者弯内的方向吧，真不行就eeprom
    // 为防止把圆环出口误判成这个，只能判出几帧后，如果有入环就清除了
    if (II.num_lm && II.num_rm)
    {
        for (uint8 i = L_END_Y(0) - 3; i < L_END_Y(LI.segCnt - 1); i++)
            if (!II.rightfindflag[i])
                return 0;
        return 1;
    }
    return 0;
}

uint8 twentyCmEnterRightUcrossJudge()
{
    if (IF.crossroad != CR1)
        return 0;

    if (II.num_lm && II.num_rm)
    {
        for (uint8 i = R_END_Y(0) - 3; i < R_END_Y(RI.segCnt - 1); i++)
            if (!II.leftfindflag[i])
                return 0;
        return 1;
    }
    return 0;
}

/*************************************************
Function: twentyCmUnilateralCrossDeal(uint8 status, uint8 dir)
Description: 单边十字补线
Input: 状态，方向
Output: null
Return: null
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void twentyCmUnilateralCrossDeal(uint8 status, uint8 dir)
{
    for (uint8 row = RT_Y(0); row < YM; row++)
        II.leftdeleteflag[row] = 1;
    for (uint8 row = LT_Y(0); row < YM; row++)
        II.rightdeleteflag[row] = 1;
    if (status == 1)
    {
        if (dir == goRight)
            II.line_forbid = LEFTLINE;
        else
            II.line_forbid = RIGHTLINE;
    }
    if (status == 3)
    {
        if (dir == goRight)
        {
            for (uint8 row = 0; row < YM; row++)
                II.rightdeleteflag[row] = 1;

            uint8 up_x = 0, up_y = 40;
            uint8 down_x = 0, down_y = 0;

            // 上方点，实在麻烦，情况有点多，想分清也比较难

            up_x = II.searchLineMid;
            up_y = II.breakY;
            // 下方点，和三叉找点基本类似
            // 左图
            if (II.num_lm && II.start_lm[0] < 10)
            {
                // 检测到角点直接用
                if (II.right_y != YM)
                {
                    down_x = II.right_x;
                    down_y = getMapYMin_Col(down_x, leftmap, 1);
                }
                else if (II.lnum[0] > 50)
                {
                    // 左边左图的关键点可以直接用
                    if (RB_Y(0) < up_y - 5)
                    {
                        down_x = RB_X(0);
                        down_y = RB_Y(0);
                    }
                    // 左图区域可能连通到顶上，这时用扫线方式找
                    else if (LI.segCnt)
                    {
                        for (uint8 i = 0; i < 40; i++)
                            if (II.leftline[i + 1] >= II.leftline[i])
                            {
                                down_y = i;
                                down_x = II.leftline[down_y];
                            }
                            else
                                break;
                    }
                }
                // 点数很少，不怕找错，直接用
                else if (II.num_lm)
                {
                    down_y = RB_Y(0);
                    down_x = RB_X(0);
                }
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(down_x, down_y, up_x, up_y, leftmap, 12);
            }
            // 左图没有可靠信息的情况，靠右图找
            else if (II.num_rm && II.start_rm[0] <= 20 && II.rnum_all > 50)
            {
                down_x = 0;
                if (LB_Y(0) < 50)
                    down_y = LB_Y(0);
                else
                    for (uint8 i = 0; i < 40; i++)
                        if (II.rightline[i + 1] <= II.rightline[i])
                            down_y = i;
                        else
                            break;

                down_y *= 0.66;
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(down_x, down_y, up_x, up_y, leftmap, 12);
            }
            // 怎么都找不到，用默认值
            else
            {
                down_x = 0;
                down_y = 0;
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(up_x, up_y, down_x, down_y, leftmap, 6);
                //                drawline(up_x,up_y,down_x,down_y,leftmap);
            }
        }
        if (dir == goLeft)
        {
            for (uint8 row = 0; row < YM; row++)
                II.leftdeleteflag[row] = 1;

            uint8 up_x = XX, up_y = 40;
            uint8 down_x = XX, down_y = 0;
            //            if()
            up_x = II.searchLineMid;
            up_y = II.breakY;

            if (II.num_rm && II.start_rm[0] < 10)
            {
                if (II.left_y != YM)
                {
                    down_x = II.left_x;
                    down_y = getMapYMin_Col(down_x, rightmap, 1);
                }
                else if (II.rnum[0] > 50)
                {
                    if (LT_Y(0) < up_y - 5)
                    {
                        down_x = LB_X(0);
                        down_y = LB_Y(0);
                    }
                    else if (RI.segCnt)
                    {
                        for (uint8 i = 0; i < 40; i++)
                            if (II.rightline[i + 1] <= II.rightline[i])
                            {
                                down_y = i;
                                down_x = II.rightline[down_y];
                            }
                            else
                                break;
                    }
                    else
                    {
                        down_y = LB_Y(0);
                        down_x = LB_X(0);
                    }
                }
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(down_x, down_y, up_x, up_y, rightmap, 12);
            }
            else if (II.num_lm && II.start_lm[0] <= 20 && II.lnum_all > 50)
            {
                down_x = XX;
                if (RB_Y(0) < 50)
                    down_y = RB_Y(0);
                else
                    for (uint8 i = 0; i < 40; i++)
                        if (II.leftline[i + 1] >= II.leftline[i])
                            down_y = i;
                        else
                            break;
                down_y *= 0.66;
                down_x = XX;
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(down_x, down_y, up_x, up_y, rightmap, 12);
            }
            else
            {
                down_x = XX;
                down_y = 0;
#ifdef BOOM7_QT_DEBUG
                qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
                drawline2(up_x, up_y, down_x, down_y, rightmap, 6);
                //                drawline(up_x,up_y,down_x,down_y,rightmap);
            }
        }
    }
}

uint8 twentyCmGoRampStateMachine()
{
    //    按boom5的坡道流程来改的，黄林志大哥的坡道流程不错的其实
    //    具体来说有这几点：
    //    1.图像判坡之后一段时间没有俯仰角上升解除状态
    //    2.各阶段中有持续太久的话，解除状态
    //    3.下坡后一段延时，判元素，不长直道加速
#define CAMERA_CHANGE_TIME 5
#define CAMERA_CHANGE_ROWS 45
    static uint8 rampFlag = 0;
    static uint8 debounce = 0;
    static int camera_change_time = 0;
    // 暂停键消除状态
    if (ramp_clearFlag)
        goto RAMP_CLEAR;

    // 上坡前图像判坡判据
    if (0 == rampFlag && !IF.rampDelay)
    {
        if (twentyCmRampJudge() && angle.Pitch > -6.0)
        {
            if (IF.crossroad == CM2)
            {
                cross_clearFlag = 1;
                IF.crossroad = 0;
            }
            rampFlag = 1;
            debounce = 0;
            judgedRampNum++;
            if (judgedRampNum > 5)
            {
                judgedRampNum = 1; // 置1而不是置0,因为上面有++
            }
#ifndef BOOM7_QT_DEBUG
            BEEP(1000); // 图像判坡
#endif
        }
        //    //用陀螺仪判正在坡上
        if (angle.Pitch > 15.0) // 直接用陀螺仪判断坡道，防止图像和激光没有判到
        {
            //            cameraLookDown(CAMERA_CHANGE_ROWS);
            rampFlag = 2;
            judgedRampNum++; // 这个必须放在最后才 ++  不然就用的是下一个坡道的参数了
            debounce = 0;
#ifndef BOOM7_QT_DEBUG
            BEEP(1000); // 图像判坡
#endif
        }
    }
    // 正在坡上判据
    if (rampFlag == 1)
    {

        ++debounce;
        if (angle.Pitch > 6.0) // 陀螺仪的值以后通过SD卡传进来
        {
            //            cameraLookDown(CAMERA_CHANGE_ROWS);
            rampFlag = 2;
            debounce = 0;
        }
        else if (debounce > 40)
        {
            goto RAMP_CLEAR;
        }
    }

    // 在坡上
    if (2 == rampFlag)
    {
        //        if(camera_change_time < CAMERA_CHANGE_TIME)
        //        {
        //            ++camera_change_time;
        //            cameraLookDown(CAMERA_CHANGE_ROWS/CAMERA_CHANGE_TIME);
        //        }

        ++debounce;
        if ((angle.Pitch < 0 && rampDown()) || angle.Pitch < -15)
        {
            rampFlag = 3;
            debounce = 0;
#ifndef BOOM7_QT_DEBUG
            BEEP(1000); // 图像判坡
#endif
            //            cameraLookUp(CAMERA_CHANGE_ROWS);
        }
        else if (debounce > 100)
        {
            goto RAMP_CLEAR;
        }
    }
    // 下坡
    if (3 == rampFlag || rampFlag == 4)
    {
        ++debounce;
        if (debounce > 10 && !twentyCmRampJudge() && rampFlag == 3 && angle.Pitch > -5)
        {
            rampFlag = 4;
            IF.rampDelay = 1; // 用来防止下坡还未完全下去时误判长直道
#ifndef BOOM7_QT_DEBUG
            BEEP(1000);
#endif
        }
        if (debounce > 40)
        {
            goto RAMP_CLEAR;
        }
    }

    // 防止下坡还未完全下去时误判长直道
    static int avoidJudgeLongStrightDelay = 0;
    if (1 == IF.rampDelay)
    {
#ifdef BOOM7_QT_DEBUG
        qout << "avoidJudgeLongStrightDelay:" << avoidJudgeLongStrightDelay;
#endif
        avoidJudgeLongStrightDelay++;
        if (avoidJudgeLongStrightDelay > 10) // 大概200ms  也就是60cm    一般来说坡道是不会接长直的
        {
            avoidJudgeLongStrightDelay = 0;
            goto RAMP_CLEAR;
        }
    }
#ifdef BOOM7_QT_DEBUG_RAMP
    if (rampFlag)
        qout << "debounce:" << debounce;
#endif
    if (1 == rampFlag || 2 == rampFlag || 3 == rampFlag) // 状态4是延时再判坡，这个时候可以判其他元素，比如车库
    {
        deleterampline();
        return rampFlag;
    }
    else
    {
        return 0;
    }
RAMP_CLEAR:
    rampFlag = 0;
    IF.ramp = 0;
    IF.rampDelay = 0;
    ramp_clearFlag = 0;
    return 0;
}

uint8 twentyCmRampJudge()
{
    uint8 high = 15;
    uint8 rampTop = 255;
    uint8 max = 20;
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmRampJudge";
#endif
    if (II.top < YY - 5)
        return 0;

    /*
     * 根据左图单边判坡
     */
    if (II.num_lm == 1 && II.num_rm == 0)
    {
        if (TOP_LM(0) != YY)
            goto BOTH_SIDES; // 说明单边判坡不太现实了，进入双线判坡
        if (YM != getMapYMin_Col(XX, deletemap, 1) && II.dnum_all > 80)
            goto BOTH_SIDES;

        // 排大曲线
        if (LI.segCnt == 1)
        {
            do
            {
                Point L_start, L_end;
                L_start.y = L_START_Y(0);
                L_start.x = getMapXMax_Row(L_start.y, leftmap, 1);
                L_end.y = L_END_Y(LI.segCnt - 1);
                L_end.x = getMapXMax_Row(L_end.y, leftmap, 1);
                if (L_start.x > L_end.x)
                    break;
                float k = (float)(L_start.y - L_end.y) / (float)(L_start.x - L_end.x);
                float b = (float)L_start.y - k * (float)L_start.x;
                uint8 cnt = 0;
                for (uint8 col = L_start.x + 1; col < L_end.x - 1; col++)
                {
                    uint8 row = k * col + b + 0.5;
                    if (leftmap[row][col - 1] == 1)
                        cnt++;
                }
#ifdef BOOM7_QT_DEBUG_RAMP
                qout << "start" << L_start.x << L_start.y << "end" << L_end.x << L_end.y << "cnt" << cnt;
#endif
                if (cnt <= 5)
                    return 0;
            } while (0);
        }
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp left" << "start count";
#endif
        uint8 num_large = 0;
        for (uint8 j = 10; j < YM; ++j)
        {
            uint8 l = getMapXMax_Row(j, leftmap, 1);
            if (l == XM)
                goto BOTH_SIDES;

            uint8 r = getMapXMin_Row(j, deletemap, 1);
            if (r != XM && r <= l + rightline[YY] - leftline[YY] + 2)
                goto BOTH_SIDES;
            if (r == XM)
                r = getMapXMin_Row(j, deletemap, 254);
            if (r == XM)
                r = getMapXMin_Row(j, insidemap, 1);
            if (r == XM)
                r = XX;

#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp left" << "l" << l << "r" << r << "aim r:" << l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 4;
#endif
            for (uint8 i = 0; i < 4; ++i)
                if (Y_WBW_Detect(0, 50, l + i, basemap) ||
                    Y_WBW_Detect(0, 50, r - i, basemap))
                    goto BOTH_SIDES;
            if (j >= YY - high)
                if (r >= l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 4)
                    num_large++;
#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp left" << "num_large" << num_large;
#endif
            if (num_large >= 8)
                return 1;
        }

        goto BOTH_SIDES;
    }
    else if (II.num_lm == 0 && II.num_rm == 1)
    {
        if (TOP_RM(0) != YY)
            goto BOTH_SIDES;
        // 左边边线太高导致没被leftmap找到，判定deletemap有没有东西，有东西就去双线判定
        if (YM != getMapYMin_Col(0, deletemap, 1) && II.dnum_all > 80)
            goto BOTH_SIDES;
        if (RI.segCnt == 1)
        {
            do
            {
                Point R_start, R_end;
                R_start.y = R_START_Y(0);
                R_start.x = getMapXMin_Row(R_start.y, rightmap, 1);
                R_end.y = R_END_Y(RI.segCnt - 1);
                R_end.x = getMapXMin_Row(R_end.y, rightmap, 1);
                if (R_start.x < R_end.x)
                    break;
                float k = (float)(R_start.y - R_end.y) / (float)(R_start.x - R_end.x);
                float b = (float)R_start.y - k * (float)R_start.x;
                uint8 cnt = 0;
                for (uint8 col = R_end.x + 1; col < R_start.x - 1; col++)
                {
                    uint8 row = k * col + b + 0.5;
                    if (rightmap[row][col + 1] == 1)
                        cnt++;
                }
#ifdef BOOM7_QT_DEBUG_RAMP
                qout << "start" << R_start.x << R_start.y << "end" << R_end.x << R_end.y << "cnt" << cnt;
#endif
                if (cnt <= 5)
                    return 0;
            } while (0);
        }
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp right" << "start count";
#endif
        uint8 num_large = 0;
        for (uint8 j = 10; j < YM; ++j)
        {
            uint8 r = getMapXMin_Row(j, rightmap, 1);
            if (r == XM)
                goto BOTH_SIDES;

            uint8 l = getMapXMax_Row(j, deletemap, 1);
            if (l != XM && r <= l + rightline[YY] - leftline[YY] + 2)
                goto BOTH_SIDES;
            if (l == XM)
                l = getMapXMax_Row(j, deletemap, 254);
            if (l == XM)
                l = getMapXMax_Row(j, insidemap, 1);
            if (l == XM)
                l = 0;

#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp right" << "l" << l << "r" << r << "aim r:" << l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 4;
            ;
#endif
            // 向外4列白黑白
            for (uint8 i = 0; i < 4; ++i)
                if (Y_WBW_Detect(0, 50, r - i, basemap) ||
                    Y_WBW_Detect(0, 50, l + i, basemap))
                    goto BOTH_SIDES;
            if (j >= YY - high)
                if (r >= l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 4)
                    num_large++;
#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp right" << "num_large" << num_large;
#endif
            if (num_large >= 8)
                return 1;
        }

        goto BOTH_SIDES;
    }

BOTH_SIDES:
    // 进入双线判坡
    // 首先左右图区域数不能大于1
#ifdef BOOM7_QT_DEBUG_RAMP
    qout << "ramp both";
#endif
    if (!II.num_lm && !II.num_rm)
        return 0;
    if (II.start_lm[0] > 20 && II.start_rm[0] > 20 && IF.crossroad != CM2)
        return 0;
    if (TFMINI_PLUS_DISTANCE > threshold_TFmini || TFMINI_PLUS_DISTANCE == 0)
        return 0;

    if (II.num_lm <= 1 && II.num_rm <= 1)
    {
        // 左右图有搜到区域时，区域最高点要比较高
        if ((II.num_lm == 1 && TOP_LM(0) < 55) || (II.num_rm == 1 && TOP_RM(0) < 55))
            return 0;
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "check left and right";
#endif
        uint8(*mapLeft)[XM] = leftmap;
        uint8(*mapRight)[XM] = rightmap;
        // 左边没搜到图
        if (II.num_lm == 0)
        {
            // 在deletemap中找横坐标为0的最高黑点
            uint8 topL = getMapYMax_Col(0, deletemap, 1);
#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp both" << "topL" << topL;
#endif
            // 找不到或者比较低就退出
            if (topL == YM || topL < 50)
                return 0;
            // 左边没有搜到图说明车身偏右，则右边最低黑点纵坐标要求需要比较低
            if (getMapYMin_Col(XX, rightmap, 1) >= 20)
                return 0;
            // deletemap中不能出现纵向白黑白
            for (uint8 j = 0; j < YY; ++j)
            {
                uint8 l = getMapXMax_Row(j, deletemap, 1);
                if (l > 0 && Y_WBW_Detect(0, YY, l - 1, basemap))
                    return 0;
            }
            // 通过到这里则可将deletemap做左图处理
            mapLeft = deletemap;
        }
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "left ok";
#endif
        if (II.num_rm == 0)
        {
            uint8 topR = getMapYMax_Col(XX, deletemap, 1);
            if (topR == YM || topR < 50)
                return 0;
            if (getMapYMin_Col(0, leftmap, 1) >= 20)
                return 0;
            for (uint8 j = 0; j < YY; ++j)
            {
                uint8 r = getMapXMin_Row(j, deletemap, 1);
                if (r < XX && Y_WBW_Detect(0, YY, r + 1, basemap))
                    return 0;
            }
            mapRight = deletemap;
        }
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "right ok";
#endif

        uint8 n = 0;
        uint8 cnt_error = 0;
        // 从上往下判max行
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "max" << max;
#endif
        for (uint8 i = 0; i < max; ++i)
        {
            for (uint8 j = 0; j < XM; ++j)
            {
                if (mapLeft[YY - i][XX - j] == 1)
                {
                    for (uint8 k = XX - j + 1; k < XM; ++k)
                    {
                        if (mapRight[YY - i][k] == 1)
                        {
                            // 找到第一个左右（deletemap）图中都有黑点的一行作为ramptop
                            if (rampTop == 255)
                                rampTop = YY - i;
                                // k右边，(XX - j)左边，相减为宽度，后两个或者为坐标限制
                                // 主要判据：
                                // 该行赛道宽度<=该行标准线中的赛道宽度（应较窄）+一定补偿
                                // 补偿：
#ifdef BOOM7_QT_DEBUG_RAMP
                            qout << "row" << YY - i << "k" << k << "XX-j" << XX - j;
                            qout << "k - (XX - j)" << k - (XX - j) << "aim" << rightline[YY - i] - leftline[YY - i] + (float)(high - (rampTop - (YY - i))) * 4 / high + 1;
#endif
                            if (k - (XX - j) <=
                                    rightline[YY - i] - leftline[YY - i] +
                                        (float)(high - (rampTop - (YY - i))) * 4 / high + 1 ||
                                k < 15 || XX - j > 31)
                                cnt_error++;
                            else
                                ++n;
#ifdef BOOM7_QT_DEBUG_RAMP
                            qout << "cnt_error" << cnt_error;
#endif
                            break;
                        }
                        if (cnt_error > 5)
                            return 0;
                        if (k == XX && i > max - high)
                            return 0;
                    }
                    break;
                }
                if (j == XX && i > max - high)
                    return 0;
            }
            if (n >= 12) // 根据实际情况调整要求的行数
                break;
        }
        // n最上方max行中是左右图（deletemap）中都有黑点的行数
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "n" << n;
#endif
        if (n < 12)
            return 0;

        uint8 recX = XM, lastRecX = XM;
        // 判最上方20行横坐标无递减
        for (uint8 j = YY; j > YM - 20; --j)
        {
            for (uint8 i = 0; i < XM; ++i)
                if (mapLeft[j][XX - i])
                {
                    recX = XX - i;
                    break;
                }
            if (recX > lastRecX)
                return 0;
            lastRecX = recX;
        }
        recX = 0;
        lastRecX = 0;
        for (uint8 j = YY; j > YM - 20; --j)
        {
            for (uint8 i = 0; i < XM; ++i)
                if (mapRight[j][i])
                {
                    recX = i;
                    break;
                }
            if (recX < lastRecX)
                return 0;
            lastRecX = recX;
        }
        return 1;
    }
    return 0;
}

// 删了很多防误判的坡判定，用于下坡
uint8 rampDown()
{
    uint8 high = 15;
    uint8 rampTop = 255;
    uint8 max = 20;
#ifdef BOOM7_QT_DEBUG
    qout << "twentyCmRampJudge";
#endif
    if (II.top < YY - 5)
        return 0;

    /*
     * 根据左图单边判坡
     */
    if (II.num_lm == 1 && II.num_rm == 0)
    {
        if (TOP_LM(0) != YY)
            goto BOTH_SIDES; // 说明单边判坡不太现实了，进入双线判坡
        if (YM != getMapYMin_Col(XX, deletemap, 1) && II.dnum_all > 80)
            goto BOTH_SIDES;

#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp left" << "start count";
#endif
        uint8 num_large = 0;
        for (uint8 j = 10; j < YM; ++j)
        {
            uint8 l = getMapXMax_Row(j, leftmap, 1);
            if (l == XM)
                goto BOTH_SIDES;

            uint8 r = getMapXMin_Row(j, deletemap, 1);
            if (r != XM)
                if (r <= l + rightline[YY] - leftline[YY] + 2)
                    goto BOTH_SIDES;
            if (r == XM)
                r = getMapXMin_Row(j, deletemap, 254);
            if (r == XM)
                r = getMapXMin_Row(j, insidemap, 1);
            if (r == XM)
                r = XX;

#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp left" << "l" << l << "r" << r;
            qout << "aim r:" << l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 3;
#endif

            for (uint8 i = 0; i < 4; ++i)
                if (Y_WBW_Detect(0, 50, l + i, basemap) ||
                    Y_WBW_Detect(0, 50, r - i, basemap))
                    goto BOTH_SIDES;
            if (j >= YY - high)
                if (r >= l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 3)
                    num_large++;
#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp left" << "num_large" << num_large;
#endif
            if (num_large >= 8)
                return 1;
        }

        goto BOTH_SIDES;
    }
    else if (II.num_lm == 0 && II.num_rm == 1)
    {
        if (TOP_RM(0) != YY)
            goto BOTH_SIDES;

        // 左边边线太高导致没被leftmap找到，判定deletemap有没有东西，有东西就去双线判定
        if (YM != getMapYMin_Col(0, deletemap, 1) && II.dnum_all > 80)
            goto BOTH_SIDES;

#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp right" << "start count";
#endif
        uint8 num_large = 0;
        for (uint8 j = 10; j < YM; ++j)
        {
            uint8 r = getMapXMin_Row(j, rightmap, 1);
            if (r == XM)
                goto BOTH_SIDES;

            uint8 l = getMapXMax_Row(j, deletemap, 1);
            if (l != XM)
                if (r <= l + rightline[YY] - leftline[YY] + 2)
                    goto BOTH_SIDES;
            if (l == XM)
                l = getMapXMax_Row(j, deletemap, 254);
            if (l == XM)
                l = getMapXMax_Row(j, insidemap, 1);
            if (l == XM)
                l = 0;

#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp right" << "l" << l << "r" << r;
            qout << "aim r:" << l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 3;
#endif
            // 向外4列白黑白
            for (uint8 i = 0; i < 4; ++i)
                if (Y_WBW_Detect(0, 50, r - i, basemap) ||
                    Y_WBW_Detect(0, 50, l + i, basemap))
                    goto BOTH_SIDES;
            if (j >= YY - high)
                if (r >= l + rightline[j] - leftline[j] + (float)(j - (YY - high)) / high * 5 + 3)
                    num_large++;
#ifdef BOOM7_QT_DEBUG_RAMP
            qout << "ramp right" << "num_large" << num_large;
#endif
            if (num_large >= 8)
                return 1;
        }
        goto BOTH_SIDES;
    }

BOTH_SIDES:
    // 进入双线判坡
    // 首先左右图区域数不能大于1
#ifdef BOOM7_QT_DEBUG_RAMP
    qout << "ramp both";
#endif
    if (II.num_lm <= 1 && II.num_rm <= 1)
    {
        uint8(*mapLeft)[XM] = leftmap;
        uint8(*mapRight)[XM] = rightmap;
        uint8 n = 0;
        uint8 cnt_error = 0;
        // 从上往下判max行
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "max" << max;
#endif
        for (uint8 i = 0; i < max; ++i)
        {
            for (uint8 j = 0; j < XM; ++j)
            {
                if (mapLeft[YY - i][XX - j] == 1)
                {
                    for (uint8 k = XX - j + 1; k < XM; ++k)
                    {
                        if (mapRight[YY - i][k] == 1)
                        {
                            // 找到第一个左右（deletemap）图中都有黑点的一行作为ramptop
                            if (rampTop == 255)
                            {
                                rampTop = YY - i;
                            }
                            // k右边，(XX - j)左边，相减为宽度，后两个或者为坐标限制
                            // 主要判据：
                            // 该行赛道宽度<=该行标准线中的赛道宽度（应较窄）+一定补偿
                            // 补偿：
#ifdef BOOM7_QT_DEBUG_RAMP
                            qout << "row" << i << "k" << k << "XX-j" << XX - j;
                            qout << "k - (XX - j)" << k - (XX - j) << "aim" << rightline[YY - i] - leftline[YY - i] + (float)(high - (rampTop - (YY - i))) * 4 / high + 2;
#endif
                            if (k - (XX - j) <=
                                    rightline[YY - i] - leftline[YY - i] +
                                        (float)(high - (rampTop - (YY - i))) * 4 / high + 2 ||
                                k < 15 || XX - j > 31)
                                cnt_error++;
                            else
                                ++n;
#ifdef BOOM7_QT_DEBUG_RAMP
                            qout << "cnt_error" << cnt_error;
#endif
                            break;
                        }
                        if (cnt_error > 5)
                            return 0;
                        if (k == XX && i > max - high)
                            return 0;
                    }
                    break;
                }
                if (j == XX && i > max - high)
                    return 0;
            }
            if (n >= 12) // 根据实际情况调整要求的行数
                break;
        }
        // n最上方max行中是左右图（deletemap）中都有黑点的行数
#ifdef BOOM7_QT_DEBUG_RAMP
        qout << "ramp both" << "n" << n;
#endif
        if (n < 12)
            return 0;
        return 1;
    }
    return 0;
}

void deleterampline()
{
    deleteforleft(1);
    deleteforright(1);
}

/*************************************************
Function: uint8 twentyCmGoAnnulusStateMachine()
Description: 圆环状态机
Input: null
Output: null
Return: 圆环状态
Others: null
Author: ZJUT && BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmGoAnnulusStateMachine()
{
    static uint8 status = 0;
    static uint8 dir;
    static uint16 time = 0;
    static uint8 anticipationTime = 0;
    if (annulus_clearFlag)
    {
        status = 0;
        startAddLine_annulus = 0;
        AD = AD_INIT;
        annulus_clearFlag = 0;
        II.annulusDealFlag = 0;
        anticipationTime = 0;
    }
    if (!anticipationTime)
        IF.annulusAnticipation = 0;
    if (status == 0)
    {
        if (leftAnnulusDetect() && !IF.crossroad)
        {
#ifndef BOOM7_QT_DEBUG
            BEEP(1000);
#endif
            status = 1;
            dir = goLeft;
            time = 0;
            anticipationTime = 0;
            AD = AD_INIT;
            ++judgedAnnulusNum;
        }
        else if (rightAnnulusDetect() && !IF.crossroad)
        {
#ifndef BOOM7_QT_DEBUG
            BEEP(1000);
#endif
            status = 1;
            dir = goRight;
            time = 0;
            anticipationTime = 0;
            AD = AD_INIT;
            ++judgedAnnulusNum;
        }
        else if (annulusAnticipation())
        {
            anticipationTime = 30;
            IF.annulusAnticipation = AA;
            return 0;
        }
        else if (anticipationTime)
        {
            anticipationTime--;
            IF.annulusAnticipation = AA;
            return 0;
        }
    }
    if (status == 1)
    {
        do
        {
            if (isEnter(dir))
            {
                status = 2;
                break;
            }
            if (timeToTurn_annulus(dir))
            {
                startAddLine_annulus = 1;
            }
            if (startAddLine_annulus)
                II.annulusDealFlag = AnnulusDeal(dir, status);
        } while (0);
    }
    if (status == 2)
    {
        do
        {
            if ((dir == goLeft && getMapYMin_Col(0, deletemap, 1) == YM &&
                 LT_X(0) == 0 && II.top <= 55) ||
                (dir == goRight && getMapYMin_Col(XX, deletemap, 1) == YM &&
                 RT_X(0) == XX && II.top <= 55))
            {
                status = 3;
                AD.flag = 1;
                break;
            }
            II.annulusDealFlag = AnnulusDeal(dir, status);
        } while (0);
    }
    if (status == 3)
    {
        if ((dir == goRight && II.right_y <= 50 && II.right_x < XX - 5 && II.right_x != 0 && II.angleL > 40 && II.angleL < 100) || // && annulus_yaw > annulus_yaw_limit) ||
            (dir == goLeft && II.left_y <= 50 && II.left_x > 5 && II.left_x != XM && II.angleR > 40 && II.angleR < 100))           // && annulus_yaw > annulus_yaw_limit))
        {
            status = 4;
            AD.flag = 2;
        }
    }
    if (status == 4)
    {
        do
        {
            if ((dir == goRight && (II.num_lm == 0 || (II.start_lm[0] > 10 && II.start_lm[1] > 10) || II.lnum_all < 50)) || (dir == goLeft && (II.num_rm == 0 || (II.start_rm[0] > 10 && II.start_rm[1] > 10) || II.rnum_all < 50)))
            {
                status = 5;
                AD.flag = 3;
                break;
            }
            AD.flag = 2;
            leave(dir);
        } while (0);
    }
    if (status == 5)
    {
#ifndef BOOM7_QT_DEBUG
        BEEP(1000);
#endif
#define MIN_DOWN 15
        ++time;
        II.line_forbid = BOTHLINE;
        if (dir == goRight)
        {
            if ((II.num_lm == 1 && II.lnum[0] > 100 && II.start_lm[0] <= MIN_DOWN) || time > 20)
            {
                time = 0;
                AD = AD_INIT;
                startAddLine_annulus = 0;
                status = 6;
            }
        }
        else if (dir == goLeft)
        {
            if ((II.num_rm == 1 && II.rnum[0] > 100 && II.start_rm[0] <= MIN_DOWN) || time > 20)
            {
                time = 0;
                AD = AD_INIT;
                startAddLine_annulus = 0;
                status = 6;
            }
        }
    }
    if (status == 6)
    {
        if (dir == goRight)
            II.line_forbid = RIGHTLINE;
        else
            II.line_forbid = LEFTLINE;
        ++time;
        if (time > 15)
        {
            status = 0;
        }
    }
    if (status == 6)
    {
        IF.annulusDelay = 1;
    }
    else
    {
        IF.annulusDelay = 0;
    }
    if (status && status != 6)
    {
        return (dir - 1) * 6 + status;
    }
    else
    {
        return 0;
    }
}

/*************************************************
Function: uint8 annulusAnticipation()
Description: 圆环预判
Input: null
Output: null
Return: 前方是否会有圆环
Others: 第一次听这个概念是在黄林志大哥那里，没入环时候找
        圆环出口轻微刹车，避免入环冲出去
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 annulusAnticipation()
{
    // 大排除
    if (II.dnum_all < 100 || II.inum_all)
        return 0;
#ifdef BOOM7_QT_DEBUG
    qout << "annulusAnticipation";
#endif
    if (LI.segCnt == 1 && LI.numCnt[0] >= 45 && II.left_y < 45 && L_END_Y(0) > 55 && II.rightPortaitSearchLineFlag)
    {
#ifdef BOOM7_QT_DEBUG_ANNULUS
        qout << "right";
#endif
        // 1行白黑白
        if (X_WBW_Detect(II.left_x - 5, XX, II.left_y, rightmap, goRight))
        {
            uint8 cnt = 0;
            // 角点往右在deletemap中找白黑白的列数

            for (uint8 col = II.searchLineMid; col < II.d_bottom[0].x; col++)
            {
                if (Y_WBW_Detect(II.rightSearchRow, YY, col, basemap))
                    cnt++;
            }
#ifdef BOOM7_QT_DEBUG_ANNULUS
            qout << "WBW cnt:" << cnt;
#endif
            // 达到一定列就认为是圆环预判
            if (cnt >= 4)
                return 1;
        }
    }
    if (RI.segCnt == 1 && RI.numCnt[0] >= 45 && II.right_y < 45 && R_END_Y(0) > 55 && II.leftPortaitSearchLineFlag)
    {
#ifdef BOOM7_QT_DEBUG_ANNULUS
        qout << "left";
#endif
        // 1行白黑白
        if (X_WBW_Detect(II.right_x + 5, 0, II.right_y, leftmap, goLeft))
        {
            uint8 cnt = 0;
            // 角点往右在deletemap中找白黑白的列数

            for (uint8 col = II.searchLineMid; col > II.d_bottom[1].x && col < 255; col--)
            {
                if (Y_WBW_Detect(II.leftSearchRow, YY, col, basemap))
                    cnt++;
            }
#ifdef BOOM7_QT_DEBUG_ANNULUS
            qout << "WBW cnt:" << cnt;
#endif
            // 达到一定列就认为是圆环预判
            if (cnt >= 4)
                return 1;
        }
    }
    return 0;
}

uint8 leftAnnulusDetect()
{

    if (II.num_lm != 1                         // 左图1张
        || II.inum_all > 10 || II.lnum[0] < 25 // 左图点数大于25
        || II.num_rm != 1                      // 右图1张
        || II.lnum[0] + II.rnum[0] < 250       // 点数要够250
        || TOP_RM(0) < II.start_rm[0] + 25     // 右图纵向长度要大于25
        || TOP_LM(0) < 20                      // 左图最高点要大于20
        || II.top < 55                         // 图像top点要大于50
        || II.speedTop < 47 || PL.right_y[0] + 2 > L_END_Y(0) || II.leftSearchRow > 55 || !II.leftPortaitSearchLineFlag || (II.rightPortaitSearchLineFlag && II.rightSearchRow < 45))
        return 0;

#ifdef BOOM7_QT_DEBUG
    qout << "annL in";
#endif
    for (uint8 row = II.breakY; row > II.breakY - 3; row--)
        if (II.leftfindflag[row] && II.rightfindflag[row] && II.rightline[row] - II.leftline[row] < 3)
            return 0;
    // 要有几行赛道比正常的宽
    uint8 cnt = 0;
    if (L_START_Y(0) > R_START_Y(0) && L_START_Y(0) >= 22 && II.lnum[0] > 150)
    {
        for (uint8 row = L_START_Y(0); row < L_START_Y(0) + 5; row++)
        {
            if (II.rightline[row] - II.leftline[row] < (rightline[row] - leftline[row]))
                cnt++;
#ifdef BOOM7_QT_DEBUG_ANNULUS
            qout << "REAL" << II.rightline[row] - II.leftline[row];
            qout << "STARDARD" << rightline[row] - leftline[row];
            qout << "cnt" << cnt;
#endif
        }
        if (cnt > 3)
            return 0;
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annL in1";
#endif

    // 圆环判据：
    // 左边：若无连通域，则左图1区域，deletemap1区域
    //      有连通域，则左图1区域通到顶 deletemap0区域
    // 右边：右图1区域 1直线
    // 扫线方面左边有纵向扫线，右边没有，纵向扫线的上方线是平着的，下方的线递减

    // 1.对左边纵向扫线上方线进行分析
    // 根据连续性找到疑似角点
    uint8 keySeg = HL.segCnt - 1;
    uint8 start_x = HL_START_X(keySeg), start_y = HL_START_Y(keySeg);
    for (uint8 i = HL_START_X(keySeg) - 1; i > HL_END_X(keySeg); i--)
    {
        if (abs((int)II.highlineLeft[i] - (int)II.highlineLeft[i + 1]) >= 2)
        {
            start_x = i;
            start_y = II.highlineLeft[i];
            break;
        }
    }
    //    qout<<HL_END_X(keySeg)<<HL_END_Y(keySeg)<<start_x<<start_y;
    // 根据坐标限制、该线之上有2行以上左右都扫到线，确认是否是真实角点
    if (HL.segCnt == 0 || HL_END_Y(keySeg) < 45 || start_x > 30 || start_y < 45)
        return 0;
    if (HL_END_X(keySeg) > 8)
    {
        uint8 row = II.rightSearchRow + 1;
        uint8 col = getMapXMin_Row(row, basemap, 0);
        if (col > 8)
            return 0;
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annL in1.5";
#endif
    // 左上线要平
    uint8 max = start_y;
    uint8 min = start_y;
    for (uint8 col = HL_END_X(0); col < start_x - 1; col++)
    {
        if (II.highlineLeft[col] > max)
            max = II.highlineLeft[col];
        if (II.highlineLeft[col] < min)
            min = II.highlineLeft[col];
    }
    if (max - min > 5)
        return 0;

#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annL in2" << start_x << start_y;
#endif
    // 2.对左边纵向扫线下方线进行分析
    if (BL.segCnt > 2 || BL_END_X(0) > 15 || BL_END_Y(0) < 30 || BL_START_X(0) > 30)
        return 0;
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annL in3";
#endif
    // 3.多线综合分析
    // 左上线终点和左下线终点横坐标相差不大，纵坐标相差不大
    if (abs((int)HL_END_X(keySeg) - (int)BL_END_X(0)) > 7)
        return 0;

    // 4.左边线曲线判定
    uint8 x1, y1, x2, y2;
    x1 = BR_X(0) + 1;
    y1 = BR_Y(0);
    if (PL.right_y[0] + 10 < RB_Y(0) && RB_Y(0) < II.leftSearchRow)
    {
        x2 = RB_X(0) + 1;
        y2 = RB_Y(0);
    }
    else
    {
        x2 = PL.right_x[0];
        y2 = PL.right_y[0];
    }
    for (uint8 row = PL.right_y[0]; row < YM; row++)
    {
        if (leftmap[row][x2] == 2 || leftmap[row][x2] == 1) // 可能点已经被删了
        {
            y2 = row;
            // 注意这里没有break
        }
        if (leftmap[row][x2] == 0)
            break;
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annL in4";
#endif
    if (strJudge(x1, y1, x2, y2, leftmap, y1, y2, 1, 5))
        return 0;
    // 5.右边线直线判定
    x1 = BL_X(0) - 1;
    y1 = BL_Y(0);
    y2 = start_y;
    x2 = II.rightline[y2];
    if (distance(start_x, start_y, x1, y1) < 5)
        return 0;
    return strJudge(x1, y1, x2, y2, rightmap, y1, y2, 1, (y2 - y1) / 10);
}

uint8 rightAnnulusDetect()
{
    if (II.num_rm != 1 || II.inum_all > 10 || II.rnum[0] < 25 || II.num_lm != 1 || II.rnum[0] + II.lnum[0] < 250 || TOP_LM(0) < II.start_lm[0] + 25 || TOP_RM(0) < 20 || II.top < 55 || II.speedTop < 47 || PR.left_y[0] + 2 > R_END_Y(0) || II.rightSearchRow > 55 || !II.rightPortaitSearchLineFlag || (II.leftPortaitSearchLineFlag && II.leftSearchRow < 45))
        return 0;

#ifdef BOOM7_QT_DEBUG
    qout << "annR in";
#endif
    for (uint8 row = II.breakY; row > II.breakY - 3; row--)
        if (II.leftfindflag[row] && II.rightfindflag[row] && II.rightline[row] - II.leftline[row] < 3)
            return 0;
    uint8 cnt = 0;
    if (L_START_Y(0) < R_START_Y(0) && R_START_Y(0) >= 22 && II.rnum[0] > 150)
    {

        for (uint8 row = R_START_Y(0); row < R_START_Y(0) + 5; row++)
        {
            if (II.rightline[row] - II.leftline[row] < (rightline[row] - leftline[row]))
                cnt++;
#ifdef BOOM7_QT_DEBUG_ANNULUS
            qout << "REAL" << II.rightline[row] - II.leftline[row];
            qout << "STARDARD" << rightline[row] - leftline[row];
            qout << "cnt" << cnt;
#endif
        }
        if (cnt > 3)
            return 0;
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annR in1";
#endif

    // 1.对右边纵向扫线上方线进行分析
    // 根据连续性找到疑似角点
    uint8 keySeg = HR.segCnt - 1;
    uint8 start_x = HR_START_X(keySeg), start_y = HR_START_Y(keySeg);
    for (uint8 i = HR_END_X(keySeg) - 1; i > HR_START_X(keySeg); i--)
    {
        if (abs((int)II.highlineRight[i] - (int)II.highlineRight[i - 1]) >= 2)
        {
            start_x = i + 1;
            start_y = II.highlineRight[i + 1];
            break;
        }
    }
    // 根据坐标限制、该线之上有2行以上左右都扫到线，确认是否是真实角点
    if (HR.segCnt == 0 || HR_END_Y(keySeg) < 45 || start_x < 17 || start_y < 45)
        return 0;
    if (HR_END_X(keySeg) < XM - 8)
    {
        // 再多判一行
        uint8 row = II.rightSearchRow + 1;
        uint8 col = getMapXMax_Row(row, basemap, 0);
        if (col < XM - 8)
            return 0;
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annR in1.5" << "keySeg" << keySeg << "start" << start_x << start_y;
#endif
    // 右上线要平
    uint8 max = start_y;
    uint8 min = start_y;
    for (uint8 col = start_x + 1; col < HR_END_X(0); col++)
    {
        if (II.highlineRight[col] > max)
            max = II.highlineRight[col];
        if (II.highlineRight[col] < min)
            min = II.highlineRight[col];
    }
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "max" << max << "min" << min;
#endif
    if (max - min > 5)
        return 0;

#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annR in2";
#endif
    // 2.对右边纵向扫线下方线进行分析
    if (BR.segCnt > 2 || BR_END_X(0) < XM - 15 || BR_END_Y(0) < 30 || BR_START_X(0) < 17)
        return 0;
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annR in3";
#endif

    // 3.多线综合分析
    // 右上线终点和右下线终点横坐标相差不大，纵坐标相差不大
    if (abs((int)HR_END_X(keySeg) - (int)BR_END_X(0)) > 7)
        return 0;
#ifdef BOOM7_QT_DEBUG_ANNULUS
    qout << "annR in4";
#endif
    // 4.右边线曲线判定,要防手连通域,x2，y2在扫线的关键点基础上，向右一格后，向上找边界
    uint8 x1, y1, x2, y2;
    x1 = BL_X(0) - 1;
    y1 = BL_Y(0);
    if (PL.left_y[0] + 10 < LB_Y(0) && LB_Y(0) < II.rightSearchRow)
    {
        x2 = LB_X(0) - 1;
        y2 = LB_Y(0);
    }
    else
    {
        x2 = PR.left_x[0];
        y2 = PR.left_y[0];
    }
    for (uint8 row = PR.left_y[0]; row < YM; row++)
    {
        if (rightmap[row][x2] == 2 || rightmap[row][x2] == 1) // 可能点已经被删了
        {
            y2 = row;
            // 注意这里没有break
        }
        if (rightmap[row][x2] == 0)
            break;
    }
    if (strJudge(x1, y1, x2, y2, rightmap, y1, y2, 1, 5))
        return 0;
    // 5.左边线直线判定
    x1 = BR_X(0) + 1;
    y1 = BR_Y(0);
    y2 = start_y;
    x2 = II.leftline[y2];
    if (distance(start_x, start_y, x1, y1) < 5)
        return 0;
    return strJudge(x1, y1, x2, y2, leftmap, y1, y2, 1, (y2 - y1) / 10);
}

// 判断开始补线时机，这套圆环判断判的很早，所以要晚点开始补线，具体什么时候自己改，随摄像头高度有变化
uint8 timeToTurn_annulus(uint8 dir)
{
    if (dir == goLeft && II.num_lm > 0)
    {
        if (k2[RT_Y(0)] < drawLine_distance_annulus && II.start_lm[0] < 15)
            return 1;
    }
    else if (dir == goRight && II.num_rm > 0)
    {
        if (k2[LT_Y(0)] < drawLine_distance_annulus && II.start_rm[0] < 15)
            return 1;
    }
    return 0;
}

uint8 isEnter(uint8 dir)
{
    if (dir == goLeft && II.rnum[0] > 50)
        return 0;
    if (dir == goRight && II.lnum[0] > 50)
        return 0;

    uint8 num = 0;
    for (uint8 i = 0; i < 40; ++i) // deletemap3行白黑白
    {
        if (num >= 3)
            return 1;

        for (uint8 j = 0; j < XM; ++j)
            if (deletemap[i][j] == 0)
            {
                for (uint8 k = j + 1; k < XM; ++k)
                {
                    if (deletemap[i][k])
                    {
                        for (uint8 m = k + 1; m < XM; ++m)
                            if (deletemap[i][m] == 0)
                            {
                                ++num;
                                break;
                            }
                        break;
                    }
                }
                break;
            }
    }
    return 0;
}

uint8 leave(uint8 dir)
{
    // 找点
    Point up, down;
    if (dir == goLeft)
    {
        // 上点
        uint8 min = getMapYMin_Col(0, deletemap, 1);
        if (min == YM)
            min = getMapYMin_Col(0, deletemap, 254);
        if (min == YM)
            min = getMapYMin_Col(0, leftmap, 1);
        if (min == YM)
            goto RETURN;
        up.y = min;
        up.x = 0;

        // 下点，有角点就基于角点来
        if (II.left_y != YM)
        {
            if (II.left_x < 5)
                return 0;
            else if ((II.left_x == XM && TOP_RM(0) == YY && LEFT(0) < XM / 2) ||
                     II.left_x >= XX - 5)
                goto RETURN;
            // 基于角点处理
            else
            {
                down.y = getMapYMin_Col(II.left_x, rightmap, 1);
                down.x = II.left_x;
            }
        }
        // 角点没检测出来
        else
        {
            // LB可以用，就用LB
            if (LB_Y(0) < YY - 10)
            {
                down.y = getMapYMin_Col(LB_X(0), rightmap, 1);
                down.x = LB_X(0);
            }
            // 不然用扫线关键点
            else if (PR.left_y[0] > PR.right_y[0] && RI.numCnt[0] > 10 && RI.start[0] < 20)
            {
                down.y = getMapYMin_Col(PR.left_x[0], rightmap, 2);
                down.x = PR.left_x[0];
            }
            // 不然不补了
            else
                goto RETURN;
        }
#ifdef BOOM7_QT_DEBUG_ANNULUS
        qout << "up" << up.x << up.y << "down" << down.x << down.y;
#endif
        // 上点在角点之上，对上点修正
        if (up.y < down.y)
            up.y = down.y + 1;
        // 删线
        for (uint8 i = down.y; i < YM; i++)
            II.rightdeleteflag[i] = 1;
        for (uint8 i = 0; i < YM; i++)
            II.leftdeleteflag[i] = 1;

        if (get_k(down.x, up.x, down.y, up.y) > get_k(BL_X(0), down.x, BL_Y(0), down.y))
        {
            drawline2(up.x, up.y, down.x, down.y, rightmap, 10);
            II.annulusDealFlag = 1;
            AD.flag = 4;
        }
    }
    else if (dir == goRight)
    {
        // 上点
        uint8 min = getMapYMin_Col(XX, deletemap, 1);
        if (min == YM)
            min = getMapYMin_Col(XX, deletemap, 254);
        if (min == YM)
            min = getMapYMin_Col(XX, rightmap, 1);
        if (min == YM)
            goto RETURN;
        up.y = min;
        up.x = XX;

        // 下点，有角点就基于角点来
        if (II.right_y != YM)
        {
            if (II.right_x > XX - 5)
                return 0;
            // 角点有问题或者很靠边就把线禁了
            else if ((II.right_x == XM && TOP_LM(0) == YY && RIGHT(0) > XM / 2) || II.right_x <= 5)
                goto RETURN;
            // 基于角点处理
            else
            {
                down.y = getMapYMin_Col(II.right_x, leftmap, 1);
                down.x = II.right_x;
            }
        }
        // 角点没检测出来
        else
        {
            // RB可以用，就用RB
            if (RB_Y(0) < YY - 10)
            {
                down.y = getMapYMin_Col(RB_X(0), leftmap, 1);
                down.x = RB_X(0);
            }
            // 不然用扫线关键点
            else if (PL.left_y[0] < PL.right_y[0] && LI.numCnt[0] > 10 && LI.start[0] < 20)
            {
                down.y = getMapYMin_Col(PL.right_x[0], leftmap, 2);
                down.x = PL.right_x[0];
            }
            // 不然不补了
            else
                goto RETURN;
        }
#ifdef BOOM7_QT_DEBUG_ANNULUS
        qout << "up" << up.x << up.y << "down" << down.x << down.y;
#endif
        // 上点在角点之上，对上点修正
        if (up.y < down.y)
            up.y = down.y + 1;
        // 删线
        for (uint8 i = down.y; i < YM; i++)
            II.leftdeleteflag[i] = 1;
        for (uint8 i = 0; i < YM; i++)
            II.rightdeleteflag[i] = 1;
        // 补线
        if (get_k(down.x, up.x, down.y, up.y) < get_k(BR_X(0), down.x, BR_Y(0), down.y))
        {
            drawline2(up.x, up.y, down.x, down.y, leftmap, 10);
            II.annulusDealFlag = 1;
            AD.flag = 4;
        }
    }
    return 1;
RETURN:
    II.line_forbid = BOTHLINE;
    return 0;
}

uint8 AnnulusDeal(uint8 ADir, uint8 status)
{
#define yhmap deletemap
    if (ADir == 0 || status > 2)
        return 0;

    uint8 downY = YM, downX = XM;
    uint8 ret = 0;

    if (ADir == goLeft)
    {
        uint8 value_1 = 1;
        uint8 value_2 = 2;
        uint8 min = getMapYMin_Col(0, deletemap, 1);
        if (min == YM)
        {
            min = getMapYMin_Col(0, deletemap, 254);
            value_1 = 254;
            value_2 = 253;
            if (min == YM)
                return 0;
        }

        if (II.top > min - 1)
            II.top = min - 1;

        if (cntMap(0, min) <= 30)
        {
            return 0;
        }

        uint8 myflag = 0;
        if (value_1 == 1)
        {
            memset(deletemap, 0, sizeof(deletemap));
            searchmap(0, min, yhmap);
        }
        for (uint8 i = 0; i < XM - 1; ++i)
        {
            for (uint8 j = 0; j < YY; ++j)
            {
                if (yhmap[j][i] == value_2 &&
                    (yhmap[j + 1][i] != value_2 || i <= 3))
                {
                    rightmap[j][i] = 3;
                    downY = j;
                    downX = i;
                    break;
                }
                else if (yhmap[j][i] == value_2 && yhmap[j + 1][i] == value_2)
                {
                    if (i < XX && getMapYMin_Col(i + 1, yhmap, value_1) > j + 1 && myflag == 0)
                    {
                        myflag = 1;
                        break;
                    }
                    else
                    {
                        rightmap[j][i] = 3;
                        rightmap[j + 1][i] = 3;
                        downY = j;
                        downX = i;
                        break;
                    }
                }
            }
            if (myflag)
                break;
        }

        if (downY < II.speedTop)
            II.speedTop = downY;

        if (downY > 55 || downX < 5)
        {
            for (uint8 i = min - 10; i < downY + 5 && i < YM; ++i)
                for (uint8 j = 0; j <= downX; ++j)
                    if (rightmap[i][j] == 3)
                        rightmap[i][j] = 0;
            return 0;
        }

        uint8 flag2 = 0;
        int x, y;
        float kk = 1.5;

        for (uint8 i = 1; i < XM; ++i)
        {
            x = downX + i;
            y = (int)(downY - i * kk);
            if (x > XX || y < 0)
                break;
            if (rightmap[y][x] || x == XX || y == 0)
            {
                flag2 = 1;
                uint8 bo = y;
                if (rightmap[y][x] == 2)
                    bo = y + 1;
                for (uint8 k = bo; k < YM; ++k)
                    for (uint8 m = 0; m < XM; ++m)
                        if (rightmap[k][m] == 2)
                            rightmap[k][m] = 0;
                break;
            }
        }

        if (flag2)
        {
            drawline2(x, y, downX, downY, rightmap, 12);
            ret = 1;
        }
    }
    else if (ADir == goRight)
    {
        uint8 value_1 = 1;
        uint8 value_2 = 2;
        uint8 min = getMapYMin_Col(XX, deletemap, 1);
        if (min == YM)
        {
            min = getMapYMin_Col(XX, deletemap, 254);
            value_1 = 254;
            value_2 = 253;
            if (min == YM)
                return 0;
        }

        if (II.top > min - 1)
            II.top = min - 1;

        if (cntMap(XX, min) <= 30)
        {
            return 0;
        }
        uint8 myflag = 0;
        if (value_1 == 1)
        {
            memset(deletemap, 0, sizeof(deletemap));
            searchmap(XX, min, yhmap);
        }
        for (uint8 i = 0; i < XM - 1; ++i)
        {
            for (uint8 j = 0; j < YY; ++j)
            {
                if (yhmap[j][XX - i] == value_2 &&
                    (yhmap[j + 1][XX - i] != value_2 || i <= 3))
                {
                    leftmap[j][XX - i] = 3;
                    downY = j;
                    downX = XX - i;
                    break;
                }
                else if (yhmap[j][XX - i] == value_2 && yhmap[j + 1][XX - i] == value_2)
                {
                    if (i < XX && getMapYMin_Col(XX - i - 1, yhmap, value_1) > j + 1 && myflag == 0)
                    {
                        myflag = 1;
                        break;
                    }
                    else
                    {
                        leftmap[j][XX - i] = 3;
                        leftmap[j + 1][XX - i] = 3;
                        downY = j;
                        downX = XX - i;
                        break;
                    }
                }
            }
            if (myflag)
                break;
        }
        if (downY < II.speedTop)
            II.speedTop = downY;

        if (downY > 55 || downX > XX - 5)
        {
            for (uint8 i = min - 10; i < downY + 5 && i < YM; ++i)
                for (int j = XX; j >= downX; --j)
                    if (leftmap[i][j] == 3)
                        leftmap[i][j] = 0;
            return 0;
        }

        uint8 flag2 = 0;
        int x, y;
        float kk = 1.5;

        for (uint8 i = 1; i < XM; ++i)
        {
            x = downX - i;
            y = (int)(downY - i * kk);
            if (x < 0 || y < 0)
                break;
            if (leftmap[y][x] || x == 0 || y == 0)
            {
                flag2 = 1;
                uint8 bo = y;
                if (leftmap[y][x] == 2)
                    bo = y + 1;
                for (uint8 k = bo; k < YM; ++k)
                    for (uint8 m = 0; m < XM; ++m)
                        if (leftmap[k][m] == 2)
                            leftmap[k][m] = 0;
                break;
            }
        }

        if (flag2)
        {
            drawline2(downX, downY, x, y, leftmap, 12);
            ret = 1;
        }
    }
    return ret;
}

/*************************************************
Function: uint8 twentyCmGoForkStateMachine()
Description: 三叉状态机
Input: null
Output: null
Return: 三叉状态
Others: null
Author: BOOM7 SAMURAI_WRY
 *************************************************/
uint8 twentyCmGoForkStateMachine()
{
    static uint8 status = 0;
    static uint8 num_map[5] = {255, 255, 255, 255, 255};
    if (fork_clearFlag)
        goto FORK_CLEAR;

    if (0 == status && forkDetect())
    {
#ifndef BOOM7_QT_DEBUG
        if (status)
            BEEP(1000);
#endif
        if (dir_fork == goLeft)
            status = 1;
        else
            status = 1;
    }
    if (1 == status)
    {
        do
        {
            if (!startAddLine_fork)
                startAddLine_fork = timeToTurn_fork();
            for (uint8 i = 0; i < 4; i++)
                num_map[i] = num_map[i + 1];
            if (dir_fork == goLeft)
            {
                if (II.rnum[0] > 500)
                    num_map[4] = II.num_rm;
                else
                    num_map[4] = 0;
            }
            if (dir_fork == goRight)
            {
                if (II.lnum[0] > 500)
                    num_map[4] = II.num_lm;
                else
                    num_map[4] = 0;
            }
            if (num_map[0] == 0 && num_map[1] == 0 && num_map[2] == 0 && num_map[3] == 0 && num_map[4])
                goto FORK_CLEAR;
            if (startAddLine_fork)
                forkDeal(dir_fork);
        } while (0);
    }
    if (status)
        return dir_fork;
    else
        return 0;
FORK_CLEAR:
#ifndef BOOM7_QT_DEBUG
    BEEP(1000);
#else
    qout << "FORK_CLEAR";
#endif
    status = 0;
    fork_clearFlag = 0;
    startAddLine_fork = 0;
    for (uint8 i = 0; i < 5; i++)
        num_map[i] = 255;
    return 0;
}

/*************************************************
Function: forkDetect()
Description: 计算偏差
Input: null
Output: null
Return: 该图是否为三叉
Others: 按照注释一点点把流程理清楚
Author: BOOM6 WJC BOOM7 SAMURAI_WRY
 *************************************************/
uint8 forkDetect()
{
    if (II.inum_all > 100)
    {
        uint8 y = getMapYMin_Col(XX / 2, insidemap, 1);
        if (y != YM)
            searchdeletemap(XX / 2, y);
    }
    // 一些很简单的排除
    if (II.d_bottom[0].y > 53 || II.d_bottom[1].y > 53 || II.dnum_all < 100 || (!II.leftPortaitSearchLineFlag && !II.rightPortaitSearchLineFlag) || (!II.angleL && !II.angleR) || (II.d_bottom[0].x < II.right_x && II.right_y != YM) || (II.d_bottom[1].x > II.left_x && II.left_y != YM))
        return 0;
    if (II.left_y != YM && II.d_bottom[0].y < II.left_y)
        return 0;
    if (II.right_y != YM && II.d_bottom[0].y < II.right_y)
        return 0;
#ifdef BOOM7_QT_DEBUG
    qout << "forkDetect";
#endif
    uint8 temp = 0;
    uint8 downLeft_x = XX;
    uint8 downLeft_y = 0;
    uint8 downRight_x = 0;
    uint8 downRight_y = 0;
    // 找到判断角点,这里wjc找的挺准的，不改了
    if (II.num_lm != 0)
    {
        if (RT_Y(0) >= YY - 10 && II.right_y < YY)
            for (uint8 i = II.right_y; i < YM; --i)
            {
                if (leftmap[i][II.right_x] != 1)
                {
                    downLeft_x = II.right_x;
                    downLeft_y = i + 1;
                    break;
                }
            }
        else
        {
            downLeft_x = II.right_bottom[0].x;
            downLeft_y = II.right_bottom[0].y;
        }
    }
    if (II.num_rm != 0)
    {
        if (LT_Y(0) >= YY - 10 && II.left_y < YY)
            for (uint8 i = II.left_y; i < YM; --i)
            {
                if (rightmap[i][II.left_x] != 1)
                {
                    downRight_x = II.left_x;
                    downRight_y = i + 1;
                    break;
                }
            }
        else
        {
            downRight_x = II.left_bottom[0].x;
            downRight_y = II.left_bottom[0].y;
        }
    }
    float k1 = strJudgeK(BR_X(0) + 1, BR_Y(0), downLeft_x, downLeft_y, leftmap, BR_Y(0), downLeft_y, 1, (downLeft_y - BR_Y(0)) / 15);
    float k2 = strJudgeK(BL_X(0) - 1, BL_Y(0), downRight_x, downRight_y, rightmap, BL_Y(0), downRight_y, 1, (downRight_y - BL_Y(0)) / 15);
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "downLeft:" << downLeft_x << downLeft_y << "k1:" << k1;
    qout << "downRight:" << downRight_x << downRight_y << "k2:" << k2;
#endif

    // v型判断,可能左入和右入看不到v，直接跳转到后面判断
    uint8 v_y = YM;
    for (uint8 i = 30; i < YM; ++i)
    {
        if (X_WBW_Detect(0, XX, i, deletemap, goRight))
        {
            if (v_y == YM)
                v_y = i - 1;
            temp++;
        }
        if (getMapXMax_Row(i, deletemap, 1) == XX || getMapXMin_Row(i, deletemap, 1) == 0)
            break;
    }
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "v_y" << v_y << "temp" << temp;
#endif
    if ((v_y > II.d_bottom[0].y + 2 || v_y > II.d_bottom[1].y + 2) && v_y != YM)
        return 0;

    // 定义只能放在跳转前面，不然就跳不了了
    uint8 leftX = 0;
    uint8 leftY = 0;
    uint8 rightX = XX;
    uint8 rightY = 0;
    uint8 flagl = 1, flagr = 1;
    if (temp < 4)
    {
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "goto LEFTORRIGHT";
#endif
        goto LEFTORRIGHT;
    }

    if (II.angleL > 135 || II.angleL < 105)
        flagl = 0;
    if (II.angleR > 135 || II.angleR < 105)
        flagr = 0;

    // 两边必须有一边角度满足
    if (flagl + flagr == 0)
        return 0;
    // 只有一边角度满足，则要两个条件里满足一个。要不放开要求到100°往上，要不是因为点数过少没有算角度
    if (!flagr)
        if (!((II.lnum_all > 150 && II.angleL > 100) || II.lnum_all < 150))
            return 0;
    if (!flagl)
        if (!((II.rnum_all > 150 && II.angleR > 100) || II.rnum_all < 150))
            return 0;
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "mid";
#endif

    if (0 != k1 && II.lnum[0] > 30)
        for (uint8 i = downLeft_y + 1; i < YM; ++i)
        {
            temp = (uint8)(k1 * (i - downLeft_y) + downLeft_x + 0.5);
            if (1 == basemap[i][temp])
            {
                leftY = i;
                leftX = temp;
                break;
            }
        }
    if (0 != k2 && II.rnum[0] > 30)
        for (uint8 i = downRight_y + 1; i < YM; ++i)
        {
            temp = (uint8)(k2 * (i - downRight_y) + downRight_x + 0.5);
            if (1 == basemap[i][temp])
            {
                rightY = i;
                rightX = temp;
                break;
            }
        }
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "leftX:" << leftX << "leftY:" << leftY;
    qout << "rightX:" << rightX << "rightY:" << rightY;
#endif
    if (II.d_bottom[0].x < leftX || II.d_bottom[1].x > rightX)
        return 0;
    if (0 != k1 && 0 != k2 && leftX != 0 && rightX != XX && II.num_lm != 0 && II.num_rm != 0)
    {
        uint8 num = 0;
        uint8 maxY = leftY >= rightY ? leftY : rightY;
        uint8 minY = leftY < rightY ? leftY : rightY;
        int16 zero = (int16)((float)(maxY - minY) * (float)(rightY - leftY) * 0.8 + 0.5);
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "maxY:" << maxY << "minY:" << minY << "zero:" << zero;
#endif
        for (uint8 i = leftX; i <= rightX; ++i)
            for (uint8 j = minY; j <= maxY; ++j)
                if (1 == basemap[j][i] || 2 == basemap[j][i])
                {
                    num++;
                    if (num >= zero)
                    {
#ifdef BOOM7_QT_DEBUG_FORK
                        qout << "both";
#endif
                        return 1;
                    }
                }
        return 0;
    }

LEFTORRIGHT:
    if (II.num_lm != 0 && 0 != k1 && II.rnum[0] < 250 && downLeft_y < II.d_bottom[0].y && II.lnum_all > 100 && II.right_x < XX - 12)
    {
        Point v, p1, p2;
        v.x = II.right_x;
        v.y = II.right_y;
        for (uint8 i = v.y; i < YM; i++)
            if (leftmap[i][v.x] == 2)
                v.y = i;
            else if (leftmap[i][v.x] == 0)
                break;

        p1 = BR(0);
        p2.y = II.d_bottom[1].y;
        p2.x = (II.d_bottom[0].x + II.d_bottom[1].x) / 2;

        float angle = getRealAngle(v, p1, p2);
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "left" << "angle" << angle;
#endif
        if (II.angleL > 130 || II.angleL < 105 || angle > 130 || angle < 105)
            return 0;

        if (II.d_bottom[0].x == II.d_bottom[1].x && II.d_bottom[0].x == XX)
        {
            if (II.d_bottom[0].y > 45)
                return 0; // 很高的话，还来得及之后再判，这种角点在边上的，其实是不该判的，怕元素接的太急，稍微放开下罢了
            uint8 range = II.right_x < 8 ? II.right_x : 8;
            uint8 up_x = XM, up_y = YM, find = 0;
            for (uint8 col = II.right_x - range; col < II.right_x && !find; col++)
                for (uint8 row = II.right_y; row < YY - 1 && !find; row++)
                    if (leftmap[row][col] && !leftmap[row + 1][col])
                    {
                        up_x = col;
                        up_y = row;
                        find = 1;
                    }
            if (up_x != XM)
                if (!strJudge(II.right_x, II.right_y, up_x, up_y, leftmap, II.right_y, up_y, 1, 4))
                    return 0;
        }

        float k3 = ((float)(II.d_bottom[0].x - downLeft_x) / (II.d_bottom[0].y + 1 - downLeft_y));
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "k3" << k3;
#endif
        if (fabs(k3 - k1) < 0.15 || (II.num_lm == 1 && downLeft_x > XX / 2 + 7))
            return 0;
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "left";
#endif
        return 1;
    }
    else if (II.num_rm != 0 && 0 != k2 && II.lnum[0] < 250 && downRight_y < II.d_bottom[1].y && II.rnum_all > 100 && II.left_x > 12)
    {
        Point v, p1, p2;
        v.x = II.left_x;
        v.y = II.left_y;
        for (uint8 i = v.y; i < YM; i++)
            if (rightmap[i][v.x] == 2)
                v.y = i;
            else if (rightmap[i][v.x] == 0)
                break;

        p1 = BL(0);
        p2.y = II.d_bottom[1].y;
        p2.x = (II.d_bottom[0].x + II.d_bottom[1].x) / 2;

        float angle = getRealAngle(v, p1, p2);
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "right" << "angle" << angle;
#endif
        if (II.angleR > 130 || II.angleR < 105 || angle > 130 || angle < 105)
            return 0;

        if (II.d_bottom[0].x == II.d_bottom[1].x && II.d_bottom[1].x == 0)
        {
            if (II.d_bottom[0].y > 45)
                return 0;
            uint8 wbwCnt = 0;
            for (uint8 i = II.d_bottom[0].y; i < YM; i++)
                if (X_WBW_Detect(0, XX, i, deletemap, goRight))
                    wbwCnt++;
            if (wbwCnt > 5)
                return 0;
            uint8 range = II.left_x < XX - 8 ? 8 : XX - II.left_x;
            uint8 up_x = XM, up_y = YM, find = 0;
            for (uint8 col = II.left_x + range; col > II.left_x && !find; col--)
                for (uint8 row = II.left_y; row < YY - 1 && !find; row++)
                    if (rightmap[row][col] && !rightmap[row + 1][col])
                    {
                        up_x = col;
                        up_y = row;
                        find = 1;
                    }

            if (up_x != XM)
                if (!strJudge(II.left_x, II.left_y, up_x, up_y, rightmap, II.left_y, up_y, 1, 4))
                    return 0;
        }
        float k3 = ((float)(II.d_bottom[1].x - downRight_x) / (II.d_bottom[1].y + 1 - downRight_y));
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "k3" << k3;
#endif
        if (fabs(k3 - k1) < 0.15 || (II.num_rm == 1 && downRight_x < XX / 2 - 7))
            return 0;
#ifdef BOOM7_QT_DEBUG_FORK
        qout << "right";
#endif
        return 1;
    }
    return 0;
}

uint8 forkDeal(uint8 Tdir)
{
    // insidemap完善下deletemap
    if (II.inum_all > 100)
    {
        uint8 y = getMapYMin_Col(XX / 2, insidemap, 1);
        if (y != YM)
            searchdeletemap(XX / 2, y);
    }

    if (goRight == Tdir)
    {
        // 左下到右上补
        // deletemap没图，在左图找
        if (II.num_lm && II.lnum_all > 100 && II.dnum_all < 50)
            for (uint8 row = YY; row > 40; row--)
                if (basemap[row][XX] && !deletemap[row][XX] && !leftmap[row][XX])
                    searchdeletemap(XX, row);
        // 开始找点
        uint8 down_y = II.startRow;
        uint8 down_x = leftline[down_y];
        uint8 up_x = II.d_bottom[0].x;
        uint8 up_y = II.d_bottom[0].y;
        // 先在左图找
        // 检测到角点直接用
        if (II.right_y != YM)
        {
            down_x = II.right_x;
            down_y = getMapYMin_Col(down_x, leftmap, 1);
        }
        // 左边左图的关键点可以直接用
        else if (II.num_lm && II.start_lm[0] <= 20 && RT_Y(0) < 50 && II.lnum_all > 50)
        {
            down_x = RB_X(0);
            down_y = RB_Y(0);
        }
        // 左图区域可能连通到顶上，这时用扫线方式找
        else if (II.num_lm && II.start_lm[0] <= 20 && II.lnum_all > 50 && LI.segCnt)
        {
            for (uint8 i = 0; i < 40; i++)
                if (II.leftline[i + 1] >= II.leftline[i])
                {
                    down_y = i;
                    down_x = II.leftline[down_y];
                }
                else
                    break;
        }
        // 点数很少，不怕找错，直接用
        else if (II.num_lm)
        {
            down_y = RB_Y(0);
            down_x = RB_X(0);
        }
        // 左图没有可靠信息的情况，靠右图找
        else if (II.num_rm && II.start_rm[0] <= 20 && II.rnum_all > 50)
        {
            // 先找纵坐标，找不到的话补到最低
            down_y = 0;
            // 右图关键点可用，乘一定系数
            if (LB_Y(0) < 50)
                down_y = LB_Y(0);
            else
                for (uint8 i = 0; i < 40; i++)
                    if (II.rightline[i + 1] <= II.rightline[i])
                        down_y = i;
                    else
                        break;
            down_y *= 0.85;
            down_x = 0;
        }
        // 怎么都找不到，用默认值
        else
        {
            down_y = II.startRow;
            down_x = leftline[down_y];
        }

        //        if(deletemap[II.d_bottom[0].y][II.d_bottom[0].x] == 2)
        //        {
        //            up_y = II.d_bottom[0].y+3;
        //            for(uint8 col=II.d_bottom[1].x;col<XX;col++)
        //                if(!deletemap[up_y][col])
        //                {
        //                    up_x = col;break;
        //                }

        //            up_x = getMapXMax_Row(up_y,deletemap,1);
        //        }
        //        else if(deletemap[II.d_bottom[0].y][II.d_bottom[0].x] == 253)
        //        {
        //            up_y = II.d_bottom[0].y+3;

        //            up_x = getMapXMax_Row(up_y,deletemap,254);
        //        }

        up_y = II.d_bottom[0].y + 3;
        for (uint8 col = II.d_bottom[1].x; col < XX; col++)
            if (!deletemap[up_y][col])
            {
                up_x = col;
                break;
            }

#ifdef BOOM7_QT_DEBUG
        qout << "down_x" << down_x << "down_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif
        // 删线
        for (uint8 i = down_y + 1; i <= YM; ++i)
        {
            II.leftdeleteflag[i] = 1;
            II.rightdeleteflag[i] = 1;
        }
        if (up_x > down_x + 1 && down_y < up_y)
        {
            if (up_y > down_y + 18)
                drawline2(up_x, up_y, down_x + 1, down_y, leftmap, 12);
            else
                drawline2(up_x, up_y, down_x + 1, down_y, leftmap, 8);
            if (up_x < XX - 3)
                for (uint8 i = up_x; i < (XX - 3); ++i)
                    leftmap[dbottomline[i]][i] = 3;
        }
    }
    else if (goLeft == Tdir)
    {
        if (II.num_rm && II.rnum_all > 100)
            for (uint8 row = YY; row > 40; row--)
                if (basemap[row][XX] && !deletemap[row][XX] && !rightmap[row][XX])
                    searchdeletemap(XX, row);
        uint8 down_y = II.startRow;
        uint8 down_x = rightline[down_y];
        uint8 up_x = II.d_bottom[1].x;
        uint8 up_y = II.d_bottom[1].y;
        if (II.left_y != YM)
        {
            down_x = II.left_x;
            down_y = getMapYMin_Col(down_x, rightmap, 1);
        }
        else if (II.num_rm && II.start_rm[0] <= 20 && LT_Y(0) < 50 && II.rnum_all > 50)
        {
            down_x = LB_X(0);
            down_y = LB_Y(0);
        }
        else if (II.num_rm && II.start_rm[0] <= 20 && II.rnum_all > 50 && RI.segCnt)
        {
            for (uint8 i = 0; i < 40; i++)
                if (II.rightline[i + 1] <= II.rightline[i])
                {
                    down_y = i;
                    down_x = II.rightline[down_y];
                }
                else
                    break;
        }
        else if (II.num_rm)
        {
            down_y = LB_Y(0);
            down_x = LB_X(0);
        }
        else if (II.num_lm && II.start_lm[0] <= 20 && II.lnum[0] > 50)
        {
            down_y = 0;
            if (RB_Y(0) < 50)
                down_y = RB_Y(0);
            else
                for (uint8 i = 0; i < 40; i++)
                    if (II.leftline[i + 1] >= II.leftline[i])
                        down_y = i;
                    else
                        break;
            down_y *= 0.85;
            down_x = XX;
        }
        else
        {
            down_y = II.startRow;
            down_x = rightline[down_y];
        }
        //        if(deletemap[II.d_bottom[1].y][II.d_bottom[1].x] == 2)
        //        {
        //            up_y = II.d_bottom[1].y+3;
        //            up_x = getMapXMin_Row(up_y,deletemap,1);
        //        }
        //        else if(deletemap[II.d_bottom[1].y][II.d_bottom[1].x] == 253)
        //        {
        //            up_y = II.d_bottom[1].y+3;
        //            up_x = getMapXMin_Row(up_y,deletemap,254);
        //        }

        up_y = II.d_bottom[0].y + 3;
        for (uint8 col = II.d_bottom[1].x; col > 0; col--)
            if (!deletemap[up_y][col])
            {
                up_x = col;
                break;
            }

#ifdef BOOM7_QT_DEBUG
        qout << "down_x" << down_x << "upleft_y" << down_y << "up_x" << up_x << "up_y" << up_y;
#endif

        for (uint8 i = down_y + 1; i <= YM; ++i)
        {
            II.leftdeleteflag[i] = 1;
            II.rightdeleteflag[i] = 1;
        }
        if (up_x < down_x - 1 && down_y < up_y)
        {
            if (up_y > down_y + 18)
                drawline2(up_x, up_y, down_x + 1, down_y, rightmap, 12);
            else
                drawline2(up_x, up_y, down_x + 1, down_y, rightmap, 8);
            if (3 < up_x)
                for (uint8 i = 3; i < up_x; ++i)
                    rightmap[dbottomline[i]][i] = 3;
        }
    }
    return 1;
}

uint8 timeToTurn_fork()
{
    uint8 downLeft_x = XX;
    uint8 downLeft_y = 0;
    uint8 downRight_x = 0;
    uint8 downRight_y = 0;
    if (II.num_lm != 0)
    {
        if (RT_Y(0) >= YY - 10 && II.right_y < YY)
            for (uint8 i = II.right_y; i < YM; --i)
            {
                if (leftmap[i][II.right_x] != 1)
                {
                    downLeft_x = II.right_x;
                    downLeft_y = i + 1;
                    break;
                }
            }
        else
        {
            downLeft_x = II.right_bottom[0].x;
            downLeft_y = II.right_bottom[0].y;
        }
    }
    if (II.num_rm != 0)
    {
        if (LT_Y(0) >= YY - 10 && II.left_y < YY)
            for (uint8 i = II.left_y; i < YM; --i)
            {
                if (rightmap[i][II.left_x] != 1)
                {
                    downRight_x = II.left_x;
                    downRight_y = i + 1;
                    break;
                }
            }
        else
        {
            downRight_x = II.left_bottom[0].x;
            downRight_y = II.left_bottom[0].y;
        }
    }
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "downLeft:" << downLeft_x << downLeft_y << "k1:" << k1;
    qout << "downRight:" << downRight_x << downRight_y << "k2:" << k2;
#endif
    Point mid, bottom;
    if (downLeft_y > 0 && downRight_y > 0)
    {
        mid.x = (downLeft_x + downRight_x) / 2;
        mid.y = (downLeft_y + downRight_y) / 2;
    }
    else if (downLeft_y > 0)
    {
        mid.x = (XM - downLeft_x) / 2 + downLeft_x;
        mid.y = downLeft_y * 0.85;
    }
    else if (downRight_y > 0)
    {
        mid.x = downRight_x / 2;
        mid.y = downRight_y * 0.85;
    }
    else
    {
        mid.x = myCarMid;
        mid.y = YY;
    }
    if (mid.y != YY)
        II.speedTop = mid.y;
    else
        II.speedTop = II.d_bottom[0].y;
    bottom.x = II.midline[0];
    bottom.y = 0;

    PointF bottom_real = getRealPoint(bottom);
    PointF mid_real = getRealPoint(mid);
    float dist = distance(bottom_real.x, bottom_real.y, mid_real.x, mid_real.y);
#ifdef BOOM7_QT_DEBUG_FORK
    qout << "mid:" << mid.x << mid.y;
    qout << "bottom:" << bottom.x << bottom.y;
    qout << "dist:" << dist;
#endif
    if (dist < drawLine_distance_fork)
        return 1;
    return 0;
}

uint8 isOut()
{
    // 首先陀螺仪判出界
    static uint8 array[5] = {0, 0, 0, 0, 0};
    for (uint8 i = 0; i < 4; i++)
        array[i] = array[i + 1];
    array[4] = 0;
    if (fabs(angle.Roll) > 30 || fabs(angle.Pitch > 45))
        array[4] = 1;
    // 然后图像
    if (!IF.ramp)
    {
        if (II.bnum_all < 100)
            return 1;
        uint16 num = 0;
        for (uint8 row = 0; row < 20; row++)
            for (uint8 col = 0; col < XM; col++)
                if (basemap[row][col] == 0)
                    num++;
        if (num < 300)
            array[4] = 1;
    }
    uint8 cnt = 0;
    for (uint8 i = 0; i < 5; i++)
        cnt += array[4];
    if (cnt > 3)
        return 1;
    return 0;
}

/***********************删线*****************************/
/*************************************************
Function: mergeAndFix()
Description: 扫线与搜图融合修正
Input: null
Output: null
Return: null
Others: 一点点看代码吧
Author: BOOM7 SAMURAI_WRY
 *************************************************/
void mergeAndFix()
{
    // 删水平线
    // 删掉左图最右点左边的右线，和右图最左点右边的左线，这才是最好的删水平线方法，入弯时删，弯内因为有个图没了就不删，不会删错
    if (II.num_rm && II.rnum[0] > 200 && TOP_RM(II.num_rm - 1) != YY)
    {
        uint8 left_x = XX, left_y = YY;
        if (LB_Y(0) < 50 && LB_X(0) != 0 && !IF.crossroad)
        {
            left_x = LB_X(0);
            left_y = LB_Y(0);
        }
        else
        {
            left_x = II.searchLineMid + 1;
            left_y = II.breakY;
        }
        for (uint8 col = left_x; col < XM; col++)
            for (uint8 row = 30; row < YM; row++)
                if (leftmap[row][col] == 3 || leftmap[row][col] == 2)
                    leftmap[row][col] = 0;
    }
    //    同
    if (II.num_lm && II.lnum[0] > 200 && TOP_LM(II.num_lm - 1) != YY)
    {
        uint8 right_x = XX, right_y = YY;
        if (RB_Y(0) < 50 && RB_X(0) != XX && !IF.crossroad)
        {
            right_x = RB_X(0);
            right_y = RB_Y(0);
        }
        else
        {
            right_x = II.searchLineMid - 1;
            right_y = II.breakY;
        }
        for (uint8 col = 0; col < right_x; col++)
            for (uint8 row = 30; row < YM; row++)
                if (rightmap[row][col] == 3 || rightmap[row][col] == 2)
                    rightmap[row][col] = 0;
    }

    if (TOP_LM(0) >= YY - 5 && TOP_RM(0) >= YY - 5 && PL.right_y[0] > 37 && PL.right_y[0] > 37 && II.dnum_all >= 50)
    {
        // 删直入三叉
        do
        {
            Point up, down;
            // 左边
            if (RT_Y(0) < YM - 5)
                up = RT(0);
            else
            {
                up.x = PL.right_x[0];
                up.y = PL.right_y[0];
            }
            down = BR(0);
            if (!strJudge(down.x + 1, down.y, up.x + 1, up.y, leftmap, down.y, up.y, 1, 5))
                break;

            // 右边
            if (LT_Y(0) < YM - 5)
                up = LT(0);
            else
            {
                up.x = PR.left_x[0];
                up.y = PR.left_y[0];
            }
            down = BL(0);
            if (!strJudge(down.x - 1, down.y, up.x - 1, up.y, rightmap, down.y, up.y, 1, 5))
                break;
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "delete";
#endif
            for (uint8 row = PL.right_y[0] < PR.left_y[0] ? PL.right_y[0] : PR.left_y[0]; row < YM; row++)
            {
                II.leftdeleteflag[row] = 1;
                II.rightdeleteflag[row] = 1;
            }
        } while (0);
    }

    /*
            十字画线筛选
    */
    // 将角点之上的点设为UNDETERMINED
    // 出现在insidemap里的几行设为undetermined
    for (uint8 i = 0; i < LI.segCnt; i++)
    {
        // 关键点到某段线结束，方向不对
        if (PL.right_y[i] != LI.end[i])
            for (uint8 j = PL.right_y[i]; j <= LI.end[i]; j++)
                //                if(II.num_rm)
                II.leftfindflag[j] = UNDETERMINED;

        // insidemap有东西，判白黑白看有没有可能是起跑线，是的话，不往上画
        if (II.inum_all > 0 && getMapXMin_Row((L_START_Y(i) + L_END_Y(i)) / 2, insidemap, 1) != XM)
            for (uint8 row = L_START_Y(i); row <= L_END_Y(i); row++)
            {
                uint8 left = getMapXMin_Row(row, insidemap, 1);
                uint8 right = getMapXMax_Row(row, insidemap, 1);
                if (X_WBW_Detect(left, right, row, insidemap, goRight))
                    II.leftfindflag[row] = UNDETERMINED;
            }

        // 下一段起始横坐标比上一段最靠中间的点靠外，不往上画
        if (i > 0 && L_START_X(i) < PL.right_x[i - 1])
            for (uint8 row = L_START_Y(i); row <= L_END_Y(i); row++)
                II.leftfindflag[row] = UNDETERMINED;
    }
    for (uint8 i = 0; i < RI.segCnt; i++)
    {
        if (PR.left_y[i] != RI.end[i])
            for (uint8 j = PR.left_y[i]; j <= RI.end[i]; j++)
                //                if(II.num_lm)
                II.rightfindflag[j] = UNDETERMINED;

        if (II.inum_all > 0 && getMapXMin_Row((R_START_Y(i) + R_END_Y(i)) / 2, insidemap, 1) != XM)
            for (uint8 row = R_START_Y(i); row <= R_END_Y(i); row++)
            {
                uint8 left = getMapXMin_Row(row, insidemap, 1);
                uint8 right = getMapXMax_Row(row, insidemap, 1);
                if (X_WBW_Detect(left, right, row, insidemap, goRight))
                    II.rightfindflag[row] = UNDETERMINED;
            }

        if (i > 0 && R_START_X(i) > PR.left_x[i - 1])
            for (uint8 row = R_START_Y(i); row <= R_END_Y(i); row++)
                II.rightfindflag[row] = UNDETERMINED;
    }

    // 暂时没用上，一个衡量危险程度的指标
    II.riskLevelLeft = getRiskLevel(goLeft);
    II.riskLevelRight = getRiskLevel(goRight);

    // 判断是否要删线的标志位
    uint8 deleteFlagL = 0, deleteFlagR = 0;

    if (II.num_lm)
        deleteFlagL = 1;
    if (II.num_rm)
        deleteFlagR = 1;

    if (IF.crossroad == CL2 || IF.crossroad == CR2 || IF.crossroad == CM2)
    {
        deleteFlagL = 0;
        deleteFlagR = 0;
    }
#ifdef BOOM7_QT_DEBUG
    qout << "deleteFlagL" << deleteFlagL << "deleteFlagR" << deleteFlagR;
#endif
    // 保证需要删的边线表征正确的行驶方向
    if (deleteFlagL)
    {
        uint8 corner_x = 0, corner_y = 0;
        uint8 end = L_END_Y(LI.segCnt - 1);
        // 1.有角点的情况
        if (II.right_y != YM && II.right_y > 25)
        {
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Left" << "corner exists";
#endif
            corner_x = II.right_x;
            corner_y = getMapYMin_Col(II.right_x, leftmap, 1) + 1;
        }
        // 关键点RT能直接用的情况
        else if (RT_Y(0) < 50)
        {
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Left" << "RT exists";
#endif
            corner_x = RT_X(0);
            corner_y = RT_Y(0);
        }
        // 什么都没法用
        else if (LI.segCnt)
        {
            for (uint8 i = 0; i < end; i++)
                if (II.leftline[i + 1] >= II.leftline[i])
                {
                    corner_x = II.leftline[i];
                    corner_y = i;
                }
                else
                    break;
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Left" << "refind corner";
#endif
        }
#ifdef BOOM7_QT_DEBUG_DELETELINE
        qout << "corner:" << corner_x << corner_y;
#endif
        if (IF.fork)
        {
            for (uint8 i = corner_y; i <= end + 1 && i < YM; i++)
                II.leftdeleteflag[i] = 1;
        }
        else
            do
            {
                if (corner_y < 10)
                {
                    uint8 seg = getSegOfPoint(corner_y, goLeft);
#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "seg:" << seg;
#endif
                    if (corner_y + 15 > L_END_Y(seg))
                        for (uint8 i = corner_y; i <= L_END_Y(seg); i++)
                            II.leftdeleteflag[i] = 1;
                }
                else if (corner_y > YY - 12)
                {
                    uint8 seg = getSegOfPoint(corner_y, goLeft);
#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "seg:" << seg;
#endif
                    if (corner_x < myCarMid || (II.leftPortaitSearchLineFlag && II.leftSearchRow > corner_y) || (seg < LI.segCnt && corner_x > L_END_X(seg)))
                        for (uint8 i = corner_y; i <= YY; i++)
                            II.leftdeleteflag[i] = 1;
                }
                else
                {
                    if (corner_x < 10)
                        break;
                    uint8 cnt = 0;
                    uint8 up = corner_y + 12;
                    if (II.leftPortaitSearchLineFlag && II.leftSearchRow > up)
                        up = II.leftSearchRow;
                    for (uint8 col = corner_x - 10; col <= corner_x; col++)
                        if (Y_WBW_Detect(0, up, col, leftmap))
                            cnt++;
#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "col:" << corner_x - 10 << "-" << corner_x << "row:" << 0 << "-" << up << "wbw cnt:" << cnt;
#endif
                    if (cnt >= 5)
                        for (uint8 i = corner_y; i <= end + 1 && i < YM; i++)
                            II.leftdeleteflag[i] = 1;
                }
            } while (0);
    }

    if (deleteFlagR)
    {
        uint8 corner_x = 0, corner_y = 0;
        uint8 end = R_END_Y(RI.segCnt - 1);

        if (II.left_y != YM && II.left_y > 25)
        {
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Right" << "corner exists";
#endif
            corner_x = II.left_x;
            corner_y = getMapYMin_Col(II.left_x, rightmap, 1) + 1;
        }
        else if (LT_Y(0) < 50)
        {
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Right" << "LT exists";
#endif
            corner_x = LT_X(0);
            corner_y = LT_Y(0);
        }
        else if (RI.segCnt)
        {
            for (uint8 i = 0; i < end; i++)
                if (II.rightline[i + 1] <= II.rightline[i])
                {
                    corner_x = II.rightline[i];
                    corner_y = i;
                }
                else
                    break;
#ifdef BOOM7_QT_DEBUG_DELETELINE
            qout << "Right" << "refind corner";
#endif
        }
#ifdef BOOM7_QT_DEBUG_DELETELINE
        qout << "corner:" << corner_x << corner_y;
#endif
        if (IF.fork)
        {
            for (uint8 i = corner_y; i <= end + 1 && i < YM; i++)
                II.rightdeleteflag[i] = 1;
        }
        else
            do
            {
                if (corner_y < 10)
                {
                    // 删一段
                    uint8 seg = getSegOfPoint(corner_y, goRight);
#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "seg:" << seg;
#endif
                    if (corner_y + 15 > R_END_Y(seg))
                        for (uint8 i = corner_y; i <= R_END_Y(seg); i++)
                            II.rightdeleteflag[i] = 1;
                }
                else if (corner_y > YY - 12)
                {
                    uint8 seg = getSegOfPoint(corner_y, goRight);
#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "seg:" << seg;
#endif
                    if (corner_x > myCarMid || (II.rightPortaitSearchLineFlag && II.rightSearchRow > corner_y) || (seg < RI.segCnt && corner_x < R_END_X(seg)))
                        for (uint8 i = corner_y; i < YY; i++)
                            II.rightdeleteflag[i] = 1;
                }
                else
                {
                    if (corner_x > XX - 10)
                        break;
                    uint8 cnt = 0;
                    uint8 up = corner_y + 12;
                    if (II.rightPortaitSearchLineFlag && II.rightSearchRow > up)
                        up = II.rightSearchRow;
                    for (uint8 col = corner_x; col <= corner_x + 10; col++)
                        if (Y_WBW_Detect(0, up, col, rightmap))
                            cnt++;

#ifdef BOOM7_QT_DEBUG_DELETELINE
                    qout << "col:" << corner_x << "-" << corner_x + 10 << "row:" << 0 << "-" << up << "wbw cnt:" << cnt;
#endif
                    if (cnt >= 5)
                        for (uint8 i = corner_y; i <= end + 1 && i < YM; i++)
                            II.rightdeleteflag[i] = 1;
                }
            } while (0);
    }

    // 开始融合修正
    if (IF.crossroad || IF.ramp)
    {
        if (fistJudgeFlag && IF.crossroad && IF.crossroad != CM1 && IF.crossroad != CM2)
            for (uint8 row = 0; row < YM; row++)
                if (getMapXMin_Row(row, insidemap, 1) != XM)
                {
                    II.leftdeleteflag[row] = 1;
                    II.rightdeleteflag[row] = 1;
                }

        for (uint8 j = 0; j < YM; j++)
        {
            // 融合 把可以用于控制的左右扫到的线以补线形式画入左右图防止被删除
            if (II.leftfindflag[j] == USEFUL && !II.leftdeleteflag[j] && !II.leftdrawflag[j])
                leftmap[j][II.leftline[j]] = 3;
            if (II.rightfindflag[j] == USEFUL && !II.rightdeleteflag[j] && !II.rightdrawflag[j])
                rightmap[j][II.rightline[j]] = 3;
        }
        if (IF.crossroad == CL1)
        {
            if (II.leftPortaitSearchLineFlag && II.right_y < YM)
                for (uint8 i = II.right_y; i <= II.leftSearchRow; i++)
                    II.leftdeleteflag[i] = 1;
        }
        if (IF.crossroad == CR1)
        {
            if (II.rightPortaitSearchLineFlag && II.left_y < YM)
                for (uint8 i = II.left_y; i <= II.rightSearchRow; i++)
                    II.rightdeleteflag[i] = 1;
        }
        // 正入十字时，删水平线
        if (IF.crossroad == CM2)
        {
            if (!II.num_lm)
            {
#ifdef BOOM7_QT_DEBUG_DELETELINE
                qout << "delete";
#endif
                uint8 right = 0;
                for (uint8 i = 0; i < LI.segCnt; i++)
                    if (right < PL.right_x[i])
                        right = PL.right_x[i];
                for (uint8 i = 0; i < right; i++)
                    for (uint8 j = 0; j < YM; j++)
                        if (rightmap[j][i] == 2 || rightmap[j][i] == 3)
                            rightmap[j][i] = 0;
            }
            if (!II.num_rm)
            {
#ifdef BOOM7_QT_DEBUG_DELETELINE
                qout << "delete";
#endif
                uint8 left = XX;
                for (uint8 i = 0; i < RI.segCnt; i++)
                    if (left < PR.left_x[i])
                        left = PR.left_x[i];
                for (uint8 i = left; i < XX; i++)
                    for (uint8 j = 0; j < YM; j++)
                        if (leftmap[j][i] == 2 || leftmap[j][i] == 3)
                            leftmap[j][i] = 0;
            }
        }
    }
    for (uint8 i = 0; i < YM; i++)
        if (II.leftdrawflag[i] || II.rightdrawflag[i])
        {
            uint8 left = 0, right = XX;
            if (II.leftfindflag[i])
                left = II.leftline[i];
            if (II.rightfindflag[i])
                right = II.rightline[i];
            for (uint8 col = XX; col < 255; col--)
                if (leftmap[i][col])
                {
                    left = col;
                    break;
                }
            for (uint8 col = 0; col < XM; col++)
                if (rightmap[i][col])
                {
                    right = col;
                    break;
                }
            II.midfindflag[i] = 1;
            II.midline[i] = (left + right) * 0.5;
        }

    for (uint8 i = 1; i < YY; i++)
    {
        if (II.midfindflag[i])
        {
            if (II.midfindflag[i - 1])
            {
                if (abs((int)II.midline[i] - (int)II.midline[i - 1]) > 2)
                {
                    if (!II.midfindflag[i + 1])
                        II.midfindflag[i] = 0;
                    else if (II.midfindflag[i + 1] && abs((int)II.midline[i] - (int)II.midline[i + 1]) > 2)
                        II.midfindflag[i] = 0;
                }
            }
            else if (II.midfindflag[i + 1])
            {
                if (abs((int)II.midline[i] - (int)II.midline[i + 1]) > 2)
                    II.midfindflag[i] = 0;
            }
            else
                II.midfindflag[i] = 0;
        }
    }
    // 该行有补过线，或者没有扫到，或者说被删了就把一整行删了
    for (uint8 j = 0; j <= II.top /* && j<STOP_LINE*/; j++)
    {
        if (!II.leftfindflag[j] || II.leftdeleteflag[j] || II.leftdrawflag[j])
            for (uint8 i = 0; i < XM; i++)
            {
                if (leftmap[j][i] == 2)
                    leftmap[j][i] = 0;
            }
        else if (II.leftfindflag[j])
        {
            if (II.leftline[j] > 5)
                for (uint8 i = 0; i < II.leftline[j] - 2; i++)
                    if (leftmap[j][i] == 2)
                        leftmap[j][i] = 0;
            if (II.leftline[j] < XX - 5)
                for (uint8 i = II.leftline[j] + 5; i < XM; i++)
                    if (leftmap[j][i] == 2)
                        leftmap[j][i] = 0;
        }
        if (!II.rightfindflag[j] || II.rightdeleteflag[j] || II.rightdrawflag[j])
            for (uint8 i = 0; i < XM; i++)
            {
                if (rightmap[j][i] == 2)
                    rightmap[j][i] = 0;
            }
        else if (II.rightfindflag[j])
        {
            if (II.rightline[j] < XM - 5)
                for (uint8 i = II.rightline[j] + 2; i < XM; i++)
                    if (rightmap[j][i] == 2)
                        rightmap[j][i] = 0;
            if (II.rightline[j] > 5)
                for (uint8 i = 0; i < II.rightline[j] - 5; i++)
                    if (rightmap[j][i] == 2)
                        rightmap[j][i] = 0;
        }
    }
    // 修补边线
    if (!IF.garage && II.lnum_all > 1000 && (II.rnum_all <= 100 || (II.start_rm[0] > 20 && LB_X(0) == 0 && II.rnum[0] > 900)) && II.speedTop < 50 && roadRecorder.now == inRightCorner)
        for (uint8 col = XM - 15; col < XM; col++)
        {
            uint8 row = getMapYMin_Col(col, leftmap, 1);
            if (abs((int)row - (int)II.speedTop) < 15 && !leftmap[row - 1][col])
                leftmap[row - 1][col] = 2;
        }
    if (!IF.garage && II.rnum_all > 1000 && (II.lnum_all <= 100 || (II.start_lm[0] > 20 && RB_X(0) == XX && II.lnum[0] > 900)) && II.speedTop < 50 && roadRecorder.now == inLeftCorner)
        for (uint8 col = 0; col < 15; col++)
        {
            uint8 row = getMapYMin_Col(col, rightmap, 1);
            if (abs((int)row - (int)II.speedTop) < 15 && !rightmap[row - 1][col])
                rightmap[row - 1][col] = 2;
        }
}

void deleteline()
{
    if (IF.annulus == AL1 || IF.annulus == AR1)
    {
        return;
    }
#define DELETE_NUM 0
    uint8 down = YM;
    for (uint8 i = TOP_LM(0); i > 1; --i)
    {
        if (X_WBW_Detect(0, XX, i, leftmap, goRight))
            down = i;
        else
            break;
    }
    uint8 minDown;
    if (down > RT_Y(0) + 1)
    {
        minDown = down;
    }
    else
    {
        minDown = RT_Y(0) + 1;
        for (uint8 j = down; j < RT_Y(0) + 1 && j < YM; ++j)
        {
            for (uint8 i = 0; i < getMapXMax_Row(j, leftmap, 1) && i != XM; ++i)
            {
                if (leftmap[j][i] == 2)
                    leftmap[j][i] = 0;
            }
        }
    }
    uint8 up = TOP_LM(0) + 1 < YY ? TOP_LM(0) + 1 : YY;
    if (minDown != YM && up >= DELETE_NUM + minDown)
    {
        for (uint8 j = minDown; j + DELETE_NUM <= up; ++j)
        {
            for (uint8 i = 0; i <= getMapXMax_Row(j, leftmap, 2) && i < XM; ++i)
            {
                if (leftmap[j][i] == 2)
                    leftmap[j][i] = 0;
            }
        }
    }
    down = YM;
    for (uint8 i = TOP_RM(0); i > 1; --i)
    {
        if (X_WBW_Detect(XX, 0, i, rightmap, goLeft))
            down = i;
        else
            break;
    }
    if (down > LT_Y(0) + 1)
    {
        minDown = down;
    }
    else
    {
        minDown = LT_Y(0) + 1;
        for (uint8 j = down; j < LT_Y(0) + 1 && j < YM; ++j)
        {
            for (int8 i = XX; i > getMapXMin_Row(j, rightmap, 1) && i != XM; --i)
            {
                if (rightmap[j][i] == 2)
                    rightmap[j][i] = 0;
            }
        }
    }
    up = TOP_RM(0) + 1 < YY ? TOP_RM(0) + 1 : YY;
    if (minDown != YM && up >= DELETE_NUM + minDown)
    {
        for (uint8 j = minDown; j + DELETE_NUM <= up; ++j)
        {
            for (uint8 i = getMapXMin_Row(j, rightmap, 2); i < XM; ++i)
            {
                if (rightmap[j][i] == 2)
                    rightmap[j][i] = 0;
            }
        }
    }
    if (II.num_lm == 0 && II.num_rm)
        deleteforright(1);
    else if (II.num_rm == 0 && II.num_lm)
        deleteforleft(1);
}

uint8 deleteforleft(uint8 undelete_flag)
{
    if (deleteforleftflag)
        return 0;
    deleteforleftflag = 1;

    uint8 right = 0;
    for (uint8 i = 0; i < II.num_lm; ++i)
        if (RIGHT(i) > right)
            right = RIGHT(i);

    for (uint8 i = 0; i <= right; ++i)
        for (uint8 j = 0; j <= II.top; ++j)
            if (leftmap[j][i] == 1)
            {
                for (uint8 k = j + 1; k <= II.top; ++k)
                    if (leftmap[k][i] == 2)
                    {
                        if (!undelete_flag && II.right_y > k + 5)
                            break;
                        for (uint8 m = k; m <= II.top; ++m)
                            if (leftmap[m][i] == 0)
                            {
                                for (uint8 n = m + 1; n <= II.top; ++n)
                                    if (leftmap[n][i] == 1)
                                    {
                                        for (uint8 a = n + 1; a <= II.top; ++a)
                                            if (leftmap[a][i] == 2)
                                                leftmap[a][i] = 0;
                                        break;
                                    }
                            }
                        break;
                    }
                break;
            }
    return 1;
}

// 把从下往上第一条边界线之后的边界删掉
uint8 deleteforright(uint8 undelete_flag)
{
    // 这个函数可能进来两次，每一帧图像处理开始会初始化为0，进来一次后就变1了
    if (deleteforrightflag)
        return 0;
    deleteforrightflag = 1;

    uint8 left = XX;
    for (uint8 i = 0; i < II.num_rm; ++i)
        if (left > LEFT(i))
            left = LEFT(i);

    for (uint8 i = XX; i >= left && i < 255; --i)
        for (uint8 j = 0; j <= II.top; ++j)
            if (rightmap[j][i] == 1)
            {
                for (uint8 k = j + 1; k <= II.top; ++k)
                    if (rightmap[k][i] == 2)
                    {
                        if (!undelete_flag && II.left_y > k + 5)
                            break;
                        for (uint8 m = k; m <= II.top; ++m)
                            if (rightmap[m][i] == 0)
                            {
                                for (uint8 n = m + 1; n <= II.top; ++n)
                                    if (rightmap[n][i] == 1)
                                    {
                                        for (uint8 a = n + 1; a <= II.top; ++a)
                                            if (rightmap[a][i] == 2)
                                                rightmap[a][i] = 0;
                                        break;
                                    }
                                break;
                            }
                        break;
                    }
                break;
            }
    return 1;
}
