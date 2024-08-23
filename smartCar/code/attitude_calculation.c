/*********************************************************************************************************************
 *
 * @file            Attitude_Calculation.c
 * @author          Alex
 * @version         v1.0
 * @Software        IAR 8.1
 * @date            2016-11-9
 ********************************************************************************************************************/

#include "attitude_calculation.h"

AttitudeDatatypedef Acc;
AttitudeDatatypedef Gyro;

float dongtaoge = 0;

#define XA (-Acc.Xdata)
#define YA (Acc.Zdata)
#define ZA (-Acc.Ydata)

#define XG (Gyro.Xdata)
#define YG (-Gyro.Zdata)
#define ZG (Gyro.Ydata)

#define ATTITUDE_COMPENSATE_LIMIT ((float)1 / 180 * PI / PERIODHZ)

QuaternionTypedef Quaternion;
EulerAngleTypedef EulerAngle;
QuaternionTypedef AxisAngle;
EulerAngleTypedef EulerAngleRate;

QuaternionTypedef MeaQuaternion;
EulerAngleTypedef MeaEulerAngle;
QuaternionTypedef MeaAxisAngle;

QuaternionTypedef ErrQuaternion;
EulerAngleTypedef ErrEulerAngle;
QuaternionTypedef ErrAxisAngle;
QuaternionTypedef lastErrQuaternion;
QuaternionTypedef lastErrAxisAngle;

float XN, XE, XD;
float YN, YE, YD;
float ZN, ZE, ZD;

float FastSqrtI(float x)
{
    //////////////////////////////////////////////////////////////////////////
    // less accuracy, more faster
    /*
    L2F l2f;
    float xhalf = 0.5f * x;
    l2f.f = x;

    l2f.i = 0x5f3759df - (l2f.i >> 1);
    x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
    return x;
    */
    //////////////////////////////////////////////////////////////////////////
    union
    {
        unsigned int i;
        float f;
    } l2f;
    l2f.f = x;
    l2f.i = 0x5F1F1412 - (l2f.i >> 1);
    return l2f.f * (1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
}

float FastSqrt(float x)
{
    return x * FastSqrtI(x);
}

/* 四元数归一化 */
void Quaternion_Normalize(QuaternionTypedef *Qu)
{
    float Normal = 0;
    Normal = sqrtf(Qu->W * Qu->W + Qu->X * Qu->X + Qu->Y * Qu->Y + Qu->Z * Qu->Z);
    if (isnan(Normal) || Normal <= 0)
    {
        Qu->W = 1;
        Qu->X = 0;
        Qu->Y = 0;
        Qu->Z = 0;
    }
    else
    {
        Qu->W /= Normal;
        Qu->X /= Normal;
        Qu->Y /= Normal;
        Qu->Z /= Normal;
    }
}

// 四元数求逆, p=q^-1=q*/(|q|^2), 原始四元数需为单位四元数
void Quaternion_Invert(QuaternionTypedef *p, const QuaternionTypedef *q)
{
    p->W = q->W;
    p->X = -q->X;
    p->Y = -q->Y;
    p->Z = -q->Z;
}

/* 四元数乘法, result=pq */
void Quaternion_Multi(QuaternionTypedef *result, const QuaternionTypedef *p, const QuaternionTypedef *q)
{
    result->W = p->W * q->W - p->X * q->X - p->Y * q->Y - p->Z * q->Z;
    result->X = p->W * q->X + p->X * q->W - p->Y * q->Z + p->Z * q->Y;
    result->Y = p->W * q->Y + p->X * q->Z + p->Y * q->W - p->Z * q->X;
    result->Z = p->W * q->Z - p->X * q->Y + p->Y * q->X + p->Z * q->W;
}

/* 四元数转欧拉角 */
void Quaternion_ToEulerAngle(const QuaternionTypedef *q, EulerAngleTypedef *e)
{
    e->Roll = atan2f(2 * (q->W * q->X + q->Y * q->Z), 1 - 2 * (q->X * q->X + q->Y * q->Y));

    float k = 2 * (q->W * q->Y - q->Z * q->X);
    if (k > 1)
        k = 1;
    else if (k < -1)
        k = -1;

    if (k >= 1.0 && k < 1.1)
    {
        e->Pitch = PI / 2;
        e->Yaw = atan2f(2 * (q->W * q->Z + q->X * q->Y), 1 - 2 * (q->Y * q->Y + q->Z * q->Z));
        return;
    }
    else if (k > -1.1 && k <= -1.0)
    {
        e->Pitch = -PI / 2;
        e->Yaw = atan2f(2 * (q->W * q->Z + q->X * q->Y), 1 - 2 * (q->Y * q->Y + q->Z * q->Z));
        return;
    }

    e->Pitch = asinf(k);
    e->Yaw = atan2f(2 * (q->W * q->Z + q->X * q->Y), 1 - 2 * (q->Y * q->Y + q->Z * q->Z));
}

/* 欧拉角转四元数 */
void Quaternion_FromEulerAngle(QuaternionTypedef *q, const EulerAngleTypedef *e)
{
    float cosx = cosf(e->Roll / 2);
    float sinx = sinf(e->Roll / 2);
    float cosy = cosf(e->Pitch / 2);
    float siny = sinf(e->Pitch / 2);
    float cosz = cosf(e->Yaw / 2);
    float sinz = sinf(e->Yaw / 2);

    q->W = cosx * cosy * cosz + sinx * siny * sinz;
    q->X = sinx * cosy * cosz - cosx * siny * sinz;
    q->Y = cosx * siny * cosz + sinx * cosy * sinz;
    q->Z = cosx * cosy * sinz - sinx * siny * cosz;
}

/* 四元数转轴角 */
void Quaternion_ToAxisAngle(const QuaternionTypedef *q, QuaternionTypedef *a)
{

    a->W = 0;
    a->X = q->X;
    a->Y = q->Y;
    a->Z = q->Z;

    Quaternion_Normalize(a);

    if ((q->W) >= 1.0 && (q->W) < 1.1)
    {
        a->W = 0;
        return;
    }
    else if ((q->W) > -1.1 && (q->W) <= -1.0)
    {
        a->W = PI * 2;
        return;
    }
    a->W = acosf(q->W) * 2;
}

/* 轴角转换为四元数 */
void Quaternion_FromAxisAngle(QuaternionTypedef *q, const QuaternionTypedef *a)
{
    float c = cosf(a->W / 2);

    float s = sinf(a->W / 2);

    q->W = 0;
    q->X = a->X;
    q->Y = a->Y;
    q->Z = a->Z;

    Quaternion_Normalize(q);
    q->W = c;
    q->X *= s;
    q->Y *= s;
    q->Z *= s;
}

/* 角速度周期转化四元数增量 */
void Quaternion_FromGyro(QuaternionTypedef *q, float wx, float wy, float wz, float dt)
{
    q->W = 1;
    q->X = wx * dt / 2;
    q->Y = wy * dt / 2;
    q->Z = wz * dt / 2;
    Quaternion_Normalize(q);
}

/* 使用角速度更新四元数(一阶龙哥库塔法Runge-Kunta, 等价于使用角度增量) */
void Quaternion_UpdateFromGyro(QuaternionTypedef *q, float x, float y, float z, float dt)
{
    float dW = 0.5 * (-q->X * x - q->Y * y - q->Z * z) * dt;
    float dX = 0.5 * (q->W * x + q->Y * z - q->Z * y) * dt;
    float dY = 0.5 * (q->W * y - q->X * z + q->Z * x) * dt;
    float dZ = 0.5 * (q->W * z + q->X * y - q->Y * x) * dt;
    q->W += dW;
    q->X += dX;
    q->Y += dY;
    q->Z += dZ;
    Quaternion_Normalize(q);
}

/* 采集加速度计获得四元数 */
void QuaternionFromAcc(QuaternionTypedef *Qu, float ax, float ay, float az, float mx, float my, float mz)
{
    float Normal = 0;
    XD = ax;
    YD = ay;
    ZD = az; // 取重力方向
    Normal = sqrtf(XD * XD + YD * YD + ZD * ZD);
    XD /= Normal;
    YD /= Normal;
    ZD /= Normal; // 归一化

    XN = -mx;
    YN = -my;
    ZN = -mz;                             // 取磁力反方向为北
    Normal = XD * XN + YD * YN + ZD * ZN; // 北
    XN -= Normal * XD;
    YN -= Normal * YD;
    ZN -= Normal * ZD; // 正交化
    Normal = sqrtf(XN * XN + YN * YN + ZN * ZN);
    XN /= Normal;
    YN /= Normal;
    ZN /= Normal; // 归一化

    XE = YD * ZN - YN * ZD; // 东 正交化
    YE = ZD * XN - ZN * XD;
    ZE = XD * YN - XN * YD;
    Normal = sqrtf(XE * XE + YE * YE + ZE * ZE);
    XE /= Normal;
    XE /= Normal;
    XE /= Normal; // 归一化

    Qu->W = 0.5 * sqrtf(XN + YE + ZD + 1); // 旋转矩阵转换四元数
    Qu->X = (YD - ZE) / (4 * Qu->W);
    Qu->Y = (ZN - XD) / (4 * Qu->W);
    Qu->Z = (XE - YN) / (4 * Qu->W);
    Quaternion_Normalize(Qu);
    return;
}

/* 四元数初始化 */
void Quaternion_init(void)
{
    if (XA != 0 || YA != 0 || ZA != 0)
    {
        QuaternionFromAcc(&Quaternion, XA, YA, ZA, -1, 0, 0);
    }
    else
    {
        Quaternion.W = 1;
        Quaternion.X = 0;
        Quaternion.Y = 0;
        Quaternion.Z = 0;
    }
    Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
    Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);
    EulerAngleRate.Pitch = 0;
    EulerAngleRate.Roll = 0;
    EulerAngleRate.Yaw = 0;
}

/* 深度融合更新 */
void Attitude_UpdateAcc(void)
{
    QuaternionTypedef EstQuaternion;
    EulerAngleTypedef EstEulerAngle;
    QuaternionTypedef DivQuaternion;
    QuaternionTypedef ComAxisangle;
    QuaternionTypedef Compensate;
    QuaternionTypedef Last;

    QuaternionFromAcc(&MeaQuaternion, 0, YA, ZA, -1, 0, 0);
    Quaternion_ToEulerAngle(&MeaQuaternion, &MeaEulerAngle);
    Quaternion_ToAxisAngle(&MeaQuaternion, &MeaAxisAngle); // 计算当前加速度计姿态

    EstEulerAngle.Roll = EulerAngle.Roll;
    EstEulerAngle.Pitch = EulerAngle.Pitch;
    EstEulerAngle.Yaw = 0;

    Quaternion_FromEulerAngle(&EstQuaternion, &EstEulerAngle); // 估计欧拉角转四元数

    // 计算估计与测得四元数偏差
    Quaternion_Invert(&DivQuaternion, &EstQuaternion);
    Quaternion_Multi(&ErrQuaternion, &DivQuaternion, &MeaQuaternion);
    Quaternion_Normalize(&ErrQuaternion);
    Quaternion_ToEulerAngle(&ErrQuaternion, &ErrEulerAngle);

    lastErrQuaternion = ErrQuaternion;
    lastErrAxisAngle = ErrAxisAngle;
    Quaternion_ToAxisAngle(&ErrQuaternion, &ErrAxisAngle);

    // 轴角校正限幅
    memcpy(&ComAxisangle, &ErrAxisAngle, sizeof(QuaternionTypedef));
    if (ComAxisangle.W > ATTITUDE_COMPENSATE_LIMIT)
    {
        ComAxisangle.W = ATTITUDE_COMPENSATE_LIMIT;
    }
    Quaternion_FromAxisAngle(&Compensate, &ComAxisangle);

    // 执行校正
    memcpy(&Last, &EstQuaternion, sizeof(QuaternionTypedef));
    Quaternion_Multi(&EstQuaternion, &Last, &Compensate);

    Quaternion_ToEulerAngle(&EstQuaternion, &EstEulerAngle);
    EstEulerAngle.Yaw = EulerAngle.Yaw; // 不使用加速度计测偏航角
    Quaternion_FromEulerAngle(&Quaternion, &EstEulerAngle);
    Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
    Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);
}

/* 快速更新 */
void Attitude_UpdateGyro()
{

    QuaternionTypedef g1, tmp;
    EulerAngleTypedef LastEulerAngle;
    QuaternionTypedef LastQuanternion;

    // 保留上一次的欧拉角和四元数
    memcpy(&LastEulerAngle, &EulerAngle, sizeof(EulerAngleTypedef));
    memcpy(&LastQuanternion, &Quaternion, sizeof(QuaternionTypedef));

    // 进行姿态更新
    float gx = XG / 180 * PI;
    float gy = YG / 180 * PI;
    float gz = ZG / 180 * PI;

    Quaternion_UpdateFromGyro(&Quaternion, gx, gy, gz, PERIODS);
    Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
    Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);

    // 计算欧拉角速度
    // Yaw为偏航角速度,为绕NED中的D轴(Z轴)旋转的角速度,使用四元数计算

    g1.W = 0;
    g1.X = gx;
    g1.Y = gy;
    g1.Z = gz;

    Quaternion_Invert(&LastQuanternion, &LastQuanternion);
    Quaternion_Multi(&tmp, &LastQuanternion, &g1);
    Quaternion_Invert(&LastQuanternion, &LastQuanternion);
    Quaternion_Multi(&g1, &tmp, &LastQuanternion);

    EulerAngleRate.Yaw = g1.Z;
    // Pitch为俯仰角速度, 为绕Y轴旋转的角速度, 使用???计算
    if (fabs(LastEulerAngle.Pitch - EulerAngle.Pitch) < PI / 2)
        EulerAngleRate.Pitch = EulerAngle.Pitch - LastEulerAngle.Pitch;
    else if (EulerAngle.Pitch - LastEulerAngle.Pitch > PI / 2)
        EulerAngleRate.Pitch = -PI + (EulerAngle.Pitch - LastEulerAngle.Pitch);
    else if (EulerAngle.Pitch - LastEulerAngle.Pitch < -PI / 2)
        EulerAngleRate.Pitch = PI + (EulerAngle.Pitch - LastEulerAngle.Pitch);

    EulerAngleRate.Pitch /= PERIODS;
    // Roll为横滚角速度, 绕X''轴旋转的角速度, 直接使用陀螺仪数据
    EulerAngleRate.Roll = gx;
}

// static const float Q_angle=0.005,Q_gyro=0.001,R_angle=40,dt=0.005;
// static float P[2][2] = {
//     { 1, 0 },
//     { 0, 1 }
// };
// static float Pdot[4] ={0,0,0,0};
// static const char C_0 = 1;
// static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
////******************************************************************************
////      卡尔曼Filter
////******************************************************************************
// void Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
//{
//     attitudeTemp.angle+=(gyro_m-q_bias) * dt;
//
//     Pdot[0]=Q_angle - P[0][1] - P[1][0];
//     Pdot[1]=- P[1][1];
//     Pdot[2]=- P[1][1];
//     Pdot[3]=Q_gyro;
//
//     P[0][0] += Pdot[0] * dt;
//     P[0][1] += Pdot[1] * dt;
//     P[1][0] += Pdot[2] * dt;
//     P[1][1] += Pdot[3] * dt;
//
//     angle_err = angle_m - attitudeTemp.angle;
//
//     PCt_0 = C_0 * P[0][0];
//     PCt_1 = C_0 * P[1][0];
//
//     E = R_angle + C_0 * PCt_0;
//
//     K_0 = PCt_0 / E;
//     K_1 = PCt_1 / E;
//
//     t_0 = PCt_0;
//     t_1 = C_0 * P[0][1];
//
//     P[0][0] -= K_0 * t_0;
//     P[0][1] -= K_0 * t_1;
//     P[1][0] -= K_1 * t_0;
//     P[1][1] -= K_1 * t_1;
//
//
//     attitudeTemp.angle   += K_0 * angle_err;
//     q_bias  += K_1 * angle_err;
//     attitudeTemp.angle_dot = gyro_m-q_bias;
// }

/*******************************************************************************
        卡尔曼滤波的五个核心公式解释
NO.1    预测现在的状态,更新系统结果
    x(k|k-1)=A*x(k-1|k-1)+B*U(k)
    x(k|k-1):利用上一状态预测的结果
    x(k-1|k-1):上一时刻的最优预测值
    U(k):现在状态的控制量，如果没有，可以为0

NO.2    更新x(k|k-1)的covariance，p表示covariance
    p(k|k-1)=A*p(k-1|k-1)*AT+Q
    p_mid=p(k|k-1),p_last=p(k-1|k-1)

NO.1 NO.2是对系统的预测

NO.3    参考测量值进行估计
    x(k|k)=x(k|k-1)+kg*(Z(k)-H*x(k|k-1))
    为了实现递归，每次的kg都是实时更新的

NO.4    卡尔曼增益 Kalman Gain
    kg=p(k|k-1)*HT/(H*p(k|k-1)*HT+R)

NO.5    p(k|k)=(1-kg*H)*p(k|k-1)
    这样每次p(k|k)和kg都需要前一时刻的值来更新，递归的估计下去
*******************************************************************************/

/*******************************************************************************
* Function Name  : Kalman_Filter
* Description    : 卡尔曼滤波
* Input          : None
* Output         : None
* Return         : None

Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏，系统噪声
R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
ResrcData:  ADC采集的数据

注：  1   A=1；    控制量：U(k)=0;
        2   个人感觉float足够用了，double太浪费了！
            如果用户感觉精度不够，可以改为double

*******************************************************************************/

float Kalman_Filter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R)
{
    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;

    static float x_last;

    float x_mid;
    float x_now;

    static float p_last;

    float p_mid;
    float p_now;

    float kg;

    x_mid = x_last;                           // x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q;                       // p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=过程噪声
    kg = p_mid / (p_mid + R);                 // kg为Kalman Gain，R为测量噪声
    x_now = x_mid + kg * (ResrcData - x_mid); // 估计出的最优值
    p_now = (1 - kg) * p_mid;                 // 最优值对应的covariance

    p_last = p_now; // 更新covariance值
    x_last = x_now; // 更新系统状态值

    return x_now;
}
