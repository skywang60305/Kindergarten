#ifndef DEAL_IMG_H
#define DEAL_IMG_H
#include "zf_common_headfile.h"
#define CAMERA_W 188 // ��������ͷͼ����
#define CAMERA_H 60  // ��������ͷͼ��߶�
#define CAMERA_SIZE CAMERA_W *CAMERA_H
#define HORIZON_TIMES 4 // ����ѹ������
#ifdef BOOM7_QT_DEBUG
#define qout qDebug() << __FILE__ << __LINE__ << __PRETTY_FUNCTION__
#endif
/*��ͼ���С������ͷ�߶ȵ���*/
#define XM 47
#define XX 46
#define YM 60 // ����map���������,�Ժ�����ͼ���������ͼ������ͳһΪ60��80
#define YY 59 // ���������-1
#define myCarMid 23
#define RANGE_GETMID 20 // ��ɨ���е�ķ�Χ
#define STEP1 33        // һ����ͼ��ʼ�����߸߶ȣ���ϸ������
#define STOP_LINE 55
#define START_LINE 0

/*
 * ����д����Ķ���
 */
#define PIXEL_WHITE 255
#define PIXEL_BLACK 0
#define goLeft 1
#define goRight 2
#define goUp 3
#define goDown 4

/*��ͼ�ؼ���*/
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

// ɨ�߹ؼ���
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
/*Ԫ�ر�־λ*/
/*����*/
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
// Բ��Ԥ��
#define AA 13

// ʮ��Ԥ��
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

// ����
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

// ������ͼ��ɨ�ߵ��ں�������ɨ������ɾ����������ͼ����Ҳɾ����ɨ��������USEFUL�ľͻ�������ͼ��
typedef enum
{
    NOT_FOUND = 0,
    USEFUL = 1,
    UNDETERMINED = 2
} PointState;

typedef struct
{
    /*ɨ�߲���*/
    uint8 breakY;                // ��������ɨ�ߵ�������
    uint8 searchLineMid;         // ��¼ɨ�����ߵĺ�����
    uint8 leftline[YM];          // ��������
    PointState leftfindflag[YM]; // �Ƿ���ɨ����
    uint8 leftdeleteflag[YM];    // �Ƿ�ɾ��
    uint8 leftdrawline[YM];      // ��������
    uint8 leftdrawflag[YM];      // ���߱�־

    uint8 rightline[YM];          // ��������
    PointState rightfindflag[YM]; // �Ƿ���ɨ����
    uint8 rightdeleteflag[YM];    // �Ƿ�ɾ��
    uint8 rightdrawline[YM];      // ��������
    uint8 rightdrawflag[YM];      // ���߱�־

    uint8 midline[YM];       // ��������
    uint8 midfindflag[YM];   // ������һ��ɨ��������Ҳɨ�����ö�����޸ģ��������ʲô�õ��ˣ�
    uint8 middeleteflag[YM]; // �Ƿ�ɾ��

    uint8 leftPortaitSearchLineFlag; // ����Ƿ�������ɨ�ߣ����������޸ģ��Ƿ����Ǹ��ܺõ��оݣ�
    uint8 leftSearchRow;             // �������ɨ����ʼ������
    uint8 leftEndCol;                // �������ɨ����ʼ�������������������Զ��
    uint8 highlineLeft[XM];          // ����������
    uint8 hlnum;                     // �������ܵ�����δ��ȥɾ���ĵ㣩
    uint8 bottomlineLeft[XM];        // ����������
    uint8 blnum;                     // �������ܵ�����δ��ȥɾ���ĵ㣩

    uint8 rightPortaitSearchLineFlag; // �ұ��Ƿ�������ɨ�ߣ����������޸ģ��Ƿ����Ǹ��ܺõ��оݣ�
    uint8 rightSearchRow;             // �ұ�����ɨ����ʼ������
    uint8 rightEndCol;                // �ұ�����ɨ����ʼ���������Һ���������Զ��
    uint8 highlineRight[XM];          // ����������
    uint8 hrnum;                      // �������ܵ�����δ��ȥɾ���ĵ㣩
    uint8 bottomlineRight[XM];        // ����������
    uint8 brnum;                      // �������ܵ�����δ��ȥɾ���ĵ㣩

    /*�洫��ͼ����*/
    uint8 step;   // һ����ͼ��ʼ��������߸߶�����
    uint8 num_lm; // �����ͼ��ͼ�����ܸ���
    uint8 num_rm; // �����ͼ��ͼ�����ܸ���

    uint8 start_lm[5]; // ��ͼÿ������ͼ��ʼ���������
    uint8 start_rm[5]; // ��ͼÿ������ͼ��ʼ���������

    // ֮�µ�8���㶼�Ǻڵ�
    // ������ͼ��ֵΪ1
    Point left_bottom[5];  // ��ͼ����������߽߱�ĵײ�
    Point right_bottom[5]; // ��ͼ���������ұ߽߱�ĵײ�
    Point left_top[5];     // ��ͼ����������߽߱�Ķ���
    Point right_top[5];    // ��ͼ���������ұ߽߱�Ķ���

    Point bottom_left[5];  // ��ͼ������ײ��߽�������
    Point bottom_right[5]; // ��ͼ������ײ��߽�����ұ�
    Point top_left[5];     // ��ͼ�����򶥲��߽�������
    Point top_right[5];    // ��ͼ�����򶥲��߽�����ұ�

    // �ǵ� (��һ����ʮ�ֵ�)
    uint8 left_x;
    uint8 left_y; // ���½ǵ�
    int angleR;   // ���½ǵ㴦��ʵ�Ƕ�
    uint8 right_x;
    uint8 right_y; // ���½ǵ�
    int angleL;    // ���½ǵ㴦��ʵ�Ƕ�
    uint8 upLeft_x;
    uint8 upLeft_y;
    uint8 upRight_x;
    uint8 upRight_y;

    uint8 top; // ����������ߵ�

    uint16 repeatNum; // ����ͼ�ظ�����
    uint16 bnum_all;  // basemap�ܰ׵�����
    uint16 dnum_top;  // 254,253�ĵ���
    uint16 lnum_all;  // leftmap�ܺڵ�����
    uint16 rnum_all;  // rightmap�ܺڵ�����
    uint16 dnum_all;  // deletemap�ܺڵ�����
    uint16 inum_all;
    uint16 lnum[5];      // leftmap������ڵ�����
    uint16 rnum[5];      // rightmap������ڵ�����
    uint16 lnum_control; // ����ѡ�ߵĵ���������stopline֮�µĵ���
    uint16 rnum_control;
    uint16 leftnum;
    uint16 rightnum;
    uint16 midnum;

    // ���߱�־
    uint8 leftCrossDealFlag;
    uint8 rightCrossDealFlag;
    // ������ʱ�Ĺؼ���
    Point d_bottom[2];

    /*���е�*/
    uint8 startRow;
    uint8 endRow;
    uint8 line_forbid;     // ���Ƶ�ʱ���ܵ���
    uint8 speedTop;        // ����ͼ�м���������ȥ����ߵ�
    uint8 annulusTop;      // Ԥ��Բ������
    uint8 annulusDealFlag; // Բ���Ƿ��߹�

    float riskLevelLeft;
    float riskLevelRight;
} IMG_INFO;

typedef struct
{
    uint8 ramp;                // �µ�
    uint8 annulus;             // Բ��
    uint8 annulusAnticipation; // Բ��Ԥ��
    uint8 annulusDelay;        // ������ʱ
    uint8 startline;           // ������
    uint8 crossAnticipation;
    uint8 crossroad; // ʮ�ּ�����ʮ��
    uint8 garage;    // ����
    uint8 rampDelay; // ������ʱ
    uint8 fork;      // ����
} IMG_FLAGS;

typedef struct
{
    uint8 segCnt;       // �ܹ��м���
    uint8 numCnt[10];   // ÿһ���ߵĵ���
    uint8 start[10];    // ÿһ���ߵ����
    uint8 end[10];      // ÿһ���ߵ��յ�
    uint8 isUseful[10]; // ÿһ���������ռ���ƫ��ʱ�Ƿ��������
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
 * �洫ͼ��Ľṹ���뺯�������޸ģ�
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
 * ͼ�񲿷ֳ�ʼ��
 */
void standard(void);
void map_init(void);
/*
 * ͼ���ȡ���ֵ��
 */
void statusReset(void);
void binaryAlgorithm(void);
uint8 myNewOstuThreshold(uint8 *image, uint32 col, uint32 startRows, uint32 endRows, uint8 GrayScale_Max, uint8 GrayScale_Min);
void Ostu_Robert(unsigned char *org_in, unsigned char *ostu_out, unsigned char th_ostu, unsigned int th_edge, unsigned int start_rows, unsigned int end_rows); // �����㷨2
/*
 * ͼ��ѹ������ͼ�ã�
 */
void horizonCompress(uint8 tempimgbuff[], uint8 img[]); // ˮƽѹ��
/*
 * ͼ����Ϣ��ȡ
 */
/*ɨ��*/
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

/*��ͼ*/
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
uint8 getLeftDownCorner(void); // ���½ǵ�
uint8 getRightDownCorner(void);

/*
 * ���ߺ���
 */
/*ͨ��*/
float get_k(int x1, int x2, int y1, int y2);
float distance(float x1, float y1, float x2, float y2);
PointF getRealPoint(Point p);
float getRealK(Point p1, Point p2);
float getRealAngle(Point vertex, Point point1, Point point2);
/*ɨ��*/
uint8 getSegOfPoint(uint8 y, uint8 dir);
/*��ͼ*/
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
 * ����
 */
void drawline(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);           // y1>y2
void drawline2(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM], uint8 n); // x1>x2
void drawline3(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);          // x1>x2

/*
 * Ԫ�ش���
 */
/*����*/
uint8 twentyCmOutGarageStateMachine(void);
/*���*/
uint8 twentyCmEnterGarageStateMachine(void);
// �ж�
uint8 twentyCmGarageJudge(uint8 judgedCnt);
uint8 twentyCmStartLineJudge(uint8 judgedCnt);
// ״̬ת��
uint8 timeToTurn_enterGarage();
uint8 readyEnterGarage();
uint8 stop();
// ����
void garageAddLine(uint8 status);

/*ʮ�ֺ͵���ʮ����һ��״̬��*/
void crossLastKeyPointInit(void);
uint8 twentyCmGoCrossroadStateMachine(void);
/*ʮ�ֲ���*/
// �ж�
uint8 twentyCmCrossroadJudge(void);
uint8 isCross_left(void);
uint8 isCross_right(void);
// �Ҳ��ߵ�
uint8 twentyCmCarFindLeftUpCorner(void);
void twentyCmCarRefindLeftDownCorner(void);
uint8 twentyCmCarFindRightUpCorner(void);
void twentyCmCarRefindRightDownCorner(void);
// ����
void drawLine_cross_left();
void drawLine_cross_right();

/*����ʮ�ֲ���*/
// �ж�
uint8 twentyCmUcrossJudge();
uint8 isUcross_Mid();
uint8 isUCross_left();
uint8 isUCross_right();
// ״̬ת��
uint8 timeToTurn_ucross();
uint8 timeToBrake_ucross();
uint8 isOut_ucross(uint8 dir);
// ������
uint8 isNotUcross_Left();
uint8 isNotUcross_Right();
// ����ʮ�ַ���ȷ��
void check_dir_ucross();
void dir_ucross_Update(uint8 dir, uint8 state_lock);
uint8 getUcrossDirByDrivingRecorder();
uint8 twentyCmCrossAnticipation_left(void);
uint8 twentyCmCrossAnticipation_right(void);
uint8 twentyCmEnterLeftUcrossJudge();
uint8 twentyCmEnterRightUcrossJudge();
// ����
void twentyCmUnilateralCrossDeal(uint8 status, uint8 dir);

/*�µ�*/
uint8 twentyCmGoRampStateMachine(void);
uint8 twentyCmRampJudge(void);
uint8 rampDown(void);
void deleterampline(void);

/*Բ��*/
uint8 twentyCmGoAnnulusStateMachine(void);
// Ԥ��
uint8 annulusAnticipation(void);
// �ж�
uint8 leftAnnulusDetect(void);
uint8 rightAnnulusDetect(void);
// ״̬ת��
uint8 timeToTurn_annulus(uint8 dir);
uint8 isEnter(uint8 dir);
uint8 leave(uint8 dir);
uint8 AnnulusDeal(uint8 dir, uint8 status);

/*����*/
uint8 twentyCmGoForkStateMachine(void);
// �ж�
uint8 forkDetect();
// ����
uint8 forkDeal(uint8 dir);
// ״̬ת��
uint8 timeToTurn_fork();
/*����*/
uint8 isOut(void);

/*
 * ɾ��
 */
// �ں�����
void mergeAndFix(void);
/*ɨ��*/
void deleteUseless(void);
/*��ͼ*/
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
extern uint8 imgGray[CAMERA_H][CAMERA_W];    // ԭͼ�Ŀ���
extern uint8 img_binary[CAMERA_H][CAMERA_W]; // ��������ͼ��ֵ�� 255�ǰ�ɫ 0�Ǻ�ɫ
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
