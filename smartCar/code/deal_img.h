#ifndef DEAL_IMG_H
#define DEAL_IMG_H
#include "zf_common_headfile.h"
#define CAMERA_W 188 // 定义摄像头图像宽度
#define CAMERA_H 60  // 定义摄像头图像高度
#define CAMERA_SIZE CAMERA_W *CAMERA_H
#define HORIZON_TIMES 4 // 横向压缩比例
#ifdef BOOM7_QT_DEBUG
#define qout qDebug() << __FILE__ << __LINE__ << __PRETTY_FUNCTION__
#endif
/*随图像大小和摄像头高度调整*/
#define XM 47
#define XX 46
#define YM 60 // 各个map最大纵坐标,以后用来图像处理的所有图纵坐标统一为60或80
#define YY 59 // 最大纵坐标-1
#define myCarMid 23
#define RANGE_GETMID 20 // 找扫线中点的范围
#define STEP1 33        // 一次搜图起始点的最高高度（仔细调整）
#define STOP_LINE 55
#define START_LINE 0

/*
 * 便于写代码的东西
 */
#define PIXEL_WHITE 255
#define PIXEL_BLACK 0
#define goLeft 1
#define goRight 2
#define goUp 3
#define goDown 4

/*搜图关键点*/
#define LB(i) II.left_bottom[i]
#define RB(i) II.right_bottom[i]
#define LT(i) II.left_top[i]
#define RT(i) II.right_top[i]

#define BL(i) II.bottom_left[i]
#define BR(i) II.bottom_right[i]
#define TL(i) II.top_left[i]
#define TR(i) II.top_right[i]

#define LB_X(i) II.left_bottom[i].x
#define RB_X(i) II.right_bottom[i].x
#define LT_X(i) II.left_top[i].x
#define RT_X(i) II.right_top[i].x

#define BL_X(i) II.bottom_left[i].x
#define BR_X(i) II.bottom_right[i].x
#define TL_X(i) II.top_left[i].x
#define TR_X(i) II.top_right[i].x

#define LB_Y(i) II.left_bottom[i].y
#define RB_Y(i) II.right_bottom[i].y
#define LT_Y(i) II.left_top[i].y
#define RT_Y(i) II.right_top[i].y

#define BL_Y(i) II.bottom_left[i].y
#define BR_Y(i) II.bottom_right[i].y
#define TL_Y(i) II.top_left[i].y
#define TR_Y(i) II.top_right[i].y

#define TOP_LM(i) TR_Y(i)
#define BOTTOM_LM(i) BR_Y(i)
#define TOP_RM(i) TL_Y(i)
#define BOTTOM_RM(i) BL_Y(i)
#define RIGHT(i) RT_X(i)
#define LEFT(i) LT_X(i)

// 扫线关键点
#define L_START_Y(i) LI.start[i]
#define L_START_X(i) II.leftline[L_START_Y(i)]
#define L_END_Y(i) LI.end[i]
#define L_END_X(i) II.leftline[L_END_Y(i)]

#define R_START_Y(i) RI.start[i]
#define R_START_X(i) II.rightline[R_START_Y(i)]
#define R_END_Y(i) RI.end[i]
#define R_END_X(i) II.rightline[R_END_Y(i)]

#define HL_START_Y(i) II.highlineLeft[HL_START_X(i)]
#define HL_START_X(i) HL.start[i]
#define HL_END_Y(i) II.highlineLeft[HL_END_X(i)]
#define HL_END_X(i) HL.end[i]

#define HR_START_Y(i) II.highlineRight[HR_START_X(i)]
#define HR_START_X(i) HR.start[i]
#define HR_END_Y(i) II.highlineRight[HR_END_X(i)]
#define HR_END_X(i) HR.end[i]

#define BL_START_Y(i) II.bottomlineLeft[BL_START_X(i)]
#define BL_START_X(i) BL.start[i]
#define BL_END_Y(i) II.bottomlineLeft[BL_END_X(i)]
#define BL_END_X(i) BL.end[i]

#define BR_START_Y(i) II.bottomlineRight[BR_START_X(i)]
#define BR_START_X(i) BR.start[i]
#define BR_END_Y(i) II.bottomlineRight[BR_END_X(i)]
#define BR_END_X(i) BR.end[i]
/*元素标志位*/
/*环岛*/
#define AL1 1
#define AL2 2
#define AL3 3
#define AL4 4
#define AL5 5
#define AL6 6

#define AR1 7
#define AR2 8
#define AR3 9
#define AR4 10
#define AR5 11
#define AR6 12
// 圆环预判
#define AA 13

// 十字预判
#define CPL 1
#define CPR 2
#define CPM 3

#define CL1 1
#define CL2 4
#define CR1 2
#define CR2 5
#define CM1 3
#define CM2 6

#define UL1 7
#define UL2 8
#define UL3 9
#define UR1 10
#define UR2 11
#define UR3 12

// 三叉
#define FL 1
#define FR 2

#define GL1 1
#define GL2 3
#define GL3 5
#define GR1 2
#define GR2 4
#define GR3 6

#ifdef BOOM7_QT_DEBUG
#define PI 3.14159265358979f
#endif

// 用于搜图和扫线的融合修正，扫线里面删除的行在搜图里面也删除，扫线里面是USEFUL的就画在左右图里
typedef enum
{
    NOT_FOUND = 0,
    USEFUL = 1,
    UNDETERMINED = 2
} PointState;

typedef struct
{
    /*扫线部分*/
    uint8 breakY;                // 结束横向扫线的纵坐标
    uint8 searchLineMid;         // 记录扫线中线的横坐标
    uint8 leftline[YM];          // 左线数组
    PointState leftfindflag[YM]; // 是否有扫到线
    uint8 leftdeleteflag[YM];    // 是否被删除
    uint8 leftdrawline[YM];      // 补线数组
    uint8 leftdrawflag[YM];      // 补线标志

    uint8 rightline[YM];          // 右线数组
    PointState rightfindflag[YM]; // 是否有扫到线
    uint8 rightdeleteflag[YM];    // 是否被删除
    uint8 rightdrawline[YM];      // 补线数组
    uint8 rightdrawflag[YM];      // 补线标志

    uint8 midline[YM];       // 中线数组
    uint8 midfindflag[YM];   // 左右有一边扫到算中线也扫到（该定义可修改，看具体干什么用的了）
    uint8 middeleteflag[YM]; // 是否被删除

    uint8 leftPortaitSearchLineFlag; // 左边是否开启纵向扫线（条件仍需修改，是否开启是个很好的判据）
    uint8 leftSearchRow;             // 左边纵向扫线起始纵坐标
    uint8 leftEndCol;                // 左边纵向扫线起始纵坐标向左横向延申最远处
    uint8 highlineLeft[XM];          // 左上线数组
    uint8 hlnum;                     // 左上线总点数（未减去删除的点）
    uint8 bottomlineLeft[XM];        // 左下线数组
    uint8 blnum;                     // 左下线总点数（未减去删除的点）

    uint8 rightPortaitSearchLineFlag; // 右边是否开启纵向扫线（条件仍需修改，是否开启是个很好的判据）
    uint8 rightSearchRow;             // 右边纵向扫线起始纵坐标
    uint8 rightEndCol;                // 右边纵向扫线起始纵坐标向右横向延申最远处
    uint8 highlineRight[XM];          // 右上线数组
    uint8 hrnum;                      // 右上线总点数（未减去删除的点）
    uint8 bottomlineRight[XM];        // 右上线数组
    uint8 brnum;                      // 右下线总点数（未减去删除的点）

    /*祖传搜图部分*/
    uint8 step;   // 一次搜图起始纵坐标最高高度限制
    uint8 num_lm; // 多次搜图左图区域总个数
    uint8 num_rm; // 多次搜图右图区域总个数

    uint8 start_lm[5]; // 左图每区域搜图起始点的纵坐标
    uint8 start_rm[5]; // 右图每区域搜图起始点的纵坐标

    // 之下的8个点都是黑点
    // 在左右图中值为1
    Point left_bottom[5];  // 右图，区域最左边边界的底部
    Point right_bottom[5]; // 左图，区域最右边边界的底部
    Point left_top[5];     // 右图，区域最左边边界的顶部
    Point right_top[5];    // 左图，区域最右边边界的顶部

    Point bottom_left[5];  // 右图，区域底部边界的最左边
    Point bottom_right[5]; // 左图，区域底部边界的最右边
    Point top_left[5];     // 右图，区域顶部边界的最左边
    Point top_right[5];    // 左图，区域顶部边界的最右边

    // 角点 (不一定是十字的)
    uint8 left_x;
    uint8 left_y; // 右下角点
    int angleR;   // 右下角点处真实角度
    uint8 right_x;
    uint8 right_y; // 左下角点
    int angleL;    // 左下角点处真实角度
    uint8 upLeft_x;
    uint8 upLeft_y;
    uint8 upRight_x;
    uint8 upRight_y;

    uint8 top; // 赛道部分最高点

    uint16 repeatNum; // 左右图重复点数
    uint16 bnum_all;  // basemap总白点数量
    uint16 dnum_top;  // 254,253的点数
    uint16 lnum_all;  // leftmap总黑点数量
    uint16 rnum_all;  // rightmap总黑点数量
    uint16 dnum_all;  // deletemap总黑点数量
    uint16 inum_all;
    uint16 lnum[5];      // leftmap各区域黑点数量
    uint16 rnum[5];      // rightmap各区域黑点数量
    uint16 lnum_control; // 拿来选线的点数，是在stopline之下的点数
    uint16 rnum_control;
    uint16 leftnum;
    uint16 rightnum;
    uint16 midnum;

    // 补线标志
    uint8 leftCrossDealFlag;
    uint8 rightCrossDealFlag;
    // 判三叉时的关键点
    Point d_bottom[2];

    /*共有的*/
    uint8 startRow;
    uint8 endRow;
    uint8 line_forbid;     // 控制的时候规避的线
    uint8 speedTop;        // 整张图中间横坐标对上去的最高点
    uint8 annulusTop;      // 预判圆环减速
    uint8 annulusDealFlag; // 圆环是否补线过

    float riskLevelLeft;
    float riskLevelRight;
} IMG_INFO;

typedef struct
{
    uint8 ramp;                // 坡道
    uint8 annulus;             // 圆环
    uint8 annulusAnticipation; // 圆环预判
    uint8 annulusDelay;        // 出环延时
    uint8 startline;           // 起跑线
    uint8 crossAnticipation;
    uint8 crossroad; // 十字及单边十字
    uint8 garage;    // 车库
    uint8 rampDelay; // 下坡延时
    uint8 fork;      // 三叉
} IMG_FLAGS;

typedef struct
{
    uint8 segCnt;       // 总共有几段
    uint8 numCnt[10];   // 每一段线的点数
    uint8 start[10];    // 每一段线的起点
    uint8 end[10];      // 每一段线的终点
    uint8 isUseful[10]; // 每一段线在最终计算偏差时是否纳入计算
} lineInfo;

typedef struct
{
    uint8 left_x[10];
    uint8 left_y[10];
    uint8 right_x[10];
    uint8 right_y[10];
} keyPoints;

typedef struct
{
    uint8 leftline[YM];
    uint8 rightline[YM];
    uint8 midline[YM];
    uint8 stopLine;
    uint8 lnum;
    uint8 rnum;
} ControlLines;

/*
 * 祖传图像的结构体与函数（有修改）
 */
typedef struct
{
    uint16 numCnt;
    Point right_bottom;
    Point right_top;
    Point bottom_right;
    Point top_right;
} RegionInfoLeft;

typedef struct
{
    uint16 numCnt;
    Point left_bottom;
    Point left_top;
    Point bottom_left;
    Point top_left;
} RegionInfoRight;

typedef struct
{
    uint16 cnt;
    float sumDev;
    float sumDK;
    uint8 flag;
} AnnulusDEV;

typedef struct PointNode
{
    uint8 num;
    Point point[XX];
} PtStack;

/*
 * 图像部分初始化
 */
void standard(void);
void map_init(void);
/*
 * 图像截取与二值化
 */
void statusReset(void);
void binaryAlgorithm(void);
uint8 myNewOstuThreshold(uint8 *image, uint32 col, uint32 startRows, uint32 endRows, uint8 GrayScale_Max, uint8 GrayScale_Min);
void Ostu_Robert(unsigned char *org_in, unsigned char *ostu_out, unsigned char th_ostu, unsigned int th_edge, unsigned int start_rows, unsigned int end_rows); // 阳光算法2
/*
 * 图像压缩（搜图用）
 */
void horizonCompress(uint8 tempimgbuff[], uint8 img[]); // 水平压缩
/*
 * 图像信息获取
 */
/*扫线*/
void searchLines(uint8 setMid);
uint8 getSearchLineColMid(uint8 lastMid, uint8 range);
uint8 getSearchLineColMid_oneSide(uint8 lastMid, uint8 range, uint8 dir);
void searchLines_Portait(uint8 startRow, uint8 startCol, uint8 dir);
uint8 getSearchLineRowLeft(uint8 startRow, uint8 startCol, uint8 dir, uint8 range);
uint8 getSearchLineRowRight(uint8 startRow, uint8 startCol, uint8 dir, uint8 range);

void getLineInfoLeft(void);
void getLineInfoRight(void);

void getKeyPoints_L();
void getKeyPoints_R();

/*搜图*/
void searchimg(uint8 x, uint8 y);
void searchimg_accelerated(uint8 x, uint8 y);

uint8 getDown(void);
void searchLeftAndRightMap(void);
void getRegionInfoLeft(uint8 x, uint8 y, uint8 src[][XM],
                       uint8 dst[][XM], RegionInfoLeft *r);
void getRegionInfoRight(uint8 x, uint8 y, uint8 src[][XM],
                        uint8 dst[][XM], RegionInfoRight *r);
void searchleftmapRemoveNoise(uint8 x, uint8 y, uint16 cntMin);
void searchrightmapRemoveNoise(uint8 x, uint8 y, uint16 cntMin);
void searchleftmap(uint8 x, uint8 y);
void searchrightmap(uint8 x, uint8 y);
uint16 cntMap(uint8 x, uint8 y);
void searchCountmap(uint8 x, uint8 y, uint8 src[][XM]);
void searchmap(uint8 x, uint8 y, uint8 src[][XM]);
void Get_insideMap(void);
void searchdeletemap(uint8 x, uint8 y);
void searchdeletemap2(uint8 x, uint8 y);
void getSpeedTop(void);
void regetSpeedTop(void);
uint8 getLeftDownCorner(void); // 找下角点
uint8 getRightDownCorner(void);

/*
 * 工具函数
 */
/*通用*/
float get_k(int x1, int x2, int y1, int y2);
float distance(float x1, float y1, float x2, float y2);
PointF getRealPoint(Point p);
float getRealK(Point p1, Point p2);
float getRealAngle(Point vertex, Point point1, Point point2);
/*扫线*/
uint8 getSegOfPoint(uint8 y, uint8 dir);
/*搜图*/
uint8 isSearch(uint8 x, uint8 y, uint8 map[][XM]);
Point getFirstBlackPoint(Point src, uint8 dir, uint8 range);
Point getFirstWhitePoint(Point src, uint8 dir, uint8 range);
uint8 getMapYMin_Col(uint8 x, uint8 map[][XM], uint8 value);
uint8 getMapYMax_Col(uint8 x, uint8 map[][XM], uint8 value);
uint8 getMapYMin_Col2(uint8 x, uint8 y, uint8 map[][XM]);
uint8 getMapXMax_Row(uint8 y, uint8 map[][XM], uint8 value);
uint8 getMapXMin_Row(uint8 y, uint8 map[][XM], uint8 value);
uint8 X_WBW_Detect(int8 x1, int8 x2, uint8 y, uint8 map[][XM],
                   uint8 flag);
uint8 X_WBW_Detect2(int8 x1, int8 x2, uint8 y, uint8 map[][XM],
                    uint8 flag);
uint8 Y_BWB_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM]);
uint8 Y_WBW_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM]);
uint8 Y_RBR_Detect(uint8 y1, uint8 y2, uint8 x, uint8 map[][XM]);
uint8 strJudge(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
               uint8 map[][XM], uint8 sy1, uint8 sy2, int8 limit,
               uint8 errorNum);
float strJudgeK(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
                uint8 map[][XM], uint8 sy1, uint8 sy2, int8 limit,
                uint8 errorNum);
uint8 strJudge_X(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
                 uint8 map[][XM], uint8 sx1, uint8 sx2, int8 limit,
                 uint8 errorNum);
uint8 countTripPoints(Point p1, Point p2, uint8 map[][XM]);
uint8 countImgBinaryTripPoints(uint8 col_min, uint8 col_max, uint8 row);

/*
 * 补线
 */
void drawline(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);           // y1>y2
void drawline2(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM], uint8 n); // x1>x2
void drawline3(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);          // x1>x2

/*
 * 元素处理
 */
/*出库*/
uint8 twentyCmOutGarageStateMachine(void);
/*入库*/
uint8 twentyCmEnterGarageStateMachine(void);
// 判定
uint8 twentyCmGarageJudge(uint8 judgedCnt);
uint8 twentyCmStartLineJudge(uint8 judgedCnt);
// 状态转换
uint8 timeToTurn_enterGarage();
uint8 readyEnterGarage();
uint8 stop();
// 补线
void garageAddLine(uint8 status);

/*十字和单边十字用一个状态机*/
void crossLastKeyPointInit(void);
uint8 twentyCmGoCrossroadStateMachine(void);
/*十字部分*/
// 判定
uint8 twentyCmCrossroadJudge(void);
uint8 isCross_left(void);
uint8 isCross_right(void);
// 找补线点
uint8 twentyCmCarFindLeftUpCorner(void);
void twentyCmCarRefindLeftDownCorner(void);
uint8 twentyCmCarFindRightUpCorner(void);
void twentyCmCarRefindRightDownCorner(void);
// 补线
void drawLine_cross_left();
void drawLine_cross_right();

/*单边十字部分*/
// 判定
uint8 twentyCmUcrossJudge();
uint8 isUcross_Mid();
uint8 isUCross_left();
uint8 isUCross_right();
// 状态转换
uint8 timeToTurn_ucross();
uint8 timeToBrake_ucross();
uint8 isOut_ucross(uint8 dir);
// 防误判
uint8 isNotUcross_Left();
uint8 isNotUcross_Right();
// 单边十字方向确定
void check_dir_ucross();
void dir_ucross_Update(uint8 dir, uint8 state_lock);
uint8 getUcrossDirByDrivingRecorder();
uint8 twentyCmCrossAnticipation_left(void);
uint8 twentyCmCrossAnticipation_right(void);
uint8 twentyCmEnterLeftUcrossJudge();
uint8 twentyCmEnterRightUcrossJudge();
// 补线
void twentyCmUnilateralCrossDeal(uint8 status, uint8 dir);

/*坡道*/
uint8 twentyCmGoRampStateMachine(void);
uint8 twentyCmRampJudge(void);
uint8 rampDown(void);
void deleterampline(void);

/*圆环*/
uint8 twentyCmGoAnnulusStateMachine(void);
// 预判
uint8 annulusAnticipation(void);
// 判定
uint8 leftAnnulusDetect(void);
uint8 rightAnnulusDetect(void);
// 状态转换
uint8 timeToTurn_annulus(uint8 dir);
uint8 isEnter(uint8 dir);
uint8 leave(uint8 dir);
uint8 AnnulusDeal(uint8 dir, uint8 status);

/*三叉*/
uint8 twentyCmGoForkStateMachine(void);
// 判定
uint8 forkDetect();
// 补线
uint8 forkDeal(uint8 dir);
// 状态转换
uint8 timeToTurn_fork();
/*出界*/
uint8 isOut(void);

/*
 * 删线
 */
// 融合修正
void mergeAndFix(void);
/*扫线*/
void deleteUseless(void);
/*搜图*/
void deleteline();
uint8 deleteforleft(uint8 undelete_flag);
uint8 deleteforright(uint8 undelete_flag);

extern IMG_INFO II;
extern IMG_INFO LAST_II;
extern IMG_INFO II_INIT;
extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern AnnulusDEV AD;
extern uint8 basemap[YM][XM];
extern uint8 leftmap[YM][XM];
extern uint8 rightmap[YM][XM];
extern uint8 insidemap[YM][XM];
extern uint8 deletemap[YM][XM];
extern uint8 allmap[YM][XM];
extern uint8 midline[YM / 2];
extern uint8 imgGray[CAMERA_H][CAMERA_W];    // 原图的拷贝
extern uint8 img_binary[CAMERA_H][CAMERA_W]; // 将拉伸后的图二值化 255是白色 0是黑色
extern int16 stopCarCnt;
extern float leftline[YM];
extern float rightline[YM];
extern float bodyworkLine_left[YM];
extern float bodyworkLine_right[YM];
extern float k1[YM], k2[YM];
extern lineInfo LI, RI, MI, HL, HR, BL, BR;
extern keyPoints PL, PR, PL_INIT, PR_INIT;
extern uint8 OSTU_THRESHOLD;
extern uint8 OSTU_THRESHOLD_PRE;
extern uint8 OSTU_MAX;
extern uint8 OSTU_MIN;
extern uint16 sobel_threshold;
extern uint16 robert_threshold;
extern uint8 highOstuGain;
extern uint8 highOstuRow;
extern uint8 dir_fork;
extern uint8 dir_ucross;
extern uint8 cnt_ucross;
extern uint8 ucross_manual_setting;
extern uint8 dir_ucross_manual_setting_1;
extern uint8 dir_ucross_manual_setting_2;
extern uint8 judgedAnnulusNum;
extern uint8 judgedRampNum;
extern uint8 judgedForkNum;
extern AnnulusDEV AD;
extern uint8 startAddLine_annulus;
extern uint8 startAddLine_fork;
extern uint8 garageDirectionFlag;
extern uint8 needToOutGarage;
extern uint8 outGarageFlag;

extern uint8 drawLine_distance_fork;
extern uint8 drawLine_distance_annulus;
extern uint8 drawLine_distance_ucross;
extern uint8 drawLine_distance_enterGarage;
extern uint8 brake_distance_ucross;
extern uint16 threshold_TFmini;
extern uint8 startAddLine_enterGarage;
extern uint8 zoom;
extern uint8 rigor_ucross_both;
extern uint8 rigor_ucross_oneSide;
extern uint8 judgedCnt;
#endif // DEAL_IMG_H
