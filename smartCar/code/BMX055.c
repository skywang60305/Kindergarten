/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * All rights reserved.
 *濞戞搩鍘煎畷鈩冨緞瑜嶉鐔割渶閸曨厾褰介弶鍫嫹
 *
 * @file            BMX055
 * @author          Alex
 * @version         v1.0
 * @Software        IAR 8.1
 * @date            2017-11-9
 ********************************************************************************************************************/

#include "BMX055.h"
vuint8 IsGyroOffsetReset = 1; // 濠碘�冲�归悘澶愭閿熺晫鎲版担鐣岀閻炴稑鐭傚褔鎽舵潪鏉垮磿闂傚棗鐖奸ˉ婵嬫儗椤愶綆鍔�闁告帗鐟ラ惃銏ゅ绩閻熸澘缍侀梺鎻掔箳閻ゅ棙绋夐敓锟� 1
BMX055Datatypedef BMX055_data;
EulerAngleTypedef SystemAttitude;     // 濠殿喗瀵ч敓鎴掓祰椤拷
EulerAngleTypedef SystemAttitudeRate; // 濠殿喗瀵ч敓鎴掓祰椤鏌呴悢宄邦唺
AttitudeDatatypedef GyroOffset;

EulerAngleTypedef Tar_Ang_Vel[2];
EulerAngleTypedef Target_Attitude;

EulerAngleTypedef firstAngle;
EulerAngleTypedef angle;

// extern RunState runState;
// uint16 GyroOffset_init_flag = 0;

float AccZAngle = 0;
EulerAngleTypedef previousSystemAttitude;
float relativeYaw = 0;

// void getBMXData(void)
//{
//     static uint8 IsAttitudeinit = 0;
//     static uint8 first = 0;
//     static uint16 num = 0;
//
//     /*****************************************************************/
//     /*闁瑰瓨鍨跺Σ鎼佸礆閸℃瑦娅曠紒鎯у皡缁辨繃绋夋繝鍥ㄦ〃闁哄嫷鍨跺ù鍌炲磻韫囨洜鍙撴慨婵撴嫹*/
//     /******************************************************************/
//
//     BMX055_DataRead(&BMX055_data, 0);
//     BMX055_data.GYROXdata = (BMX055_data.GYROXdata - GyroOffset.Xdata) * 0.030517578;
//     BMX055_data.GYROYdata = (BMX055_data.GYROYdata - GyroOffset.Ydata) * 0.030517578;
//     BMX055_data.GYROZdata = (BMX055_data.GYROZdata - GyroOffset.Zdata) * 0.030517578;
//
//     //1000 / 32768     //BMX055闁哄牜鍓濋棅鈺呮⒖閸洦妫熼柛鎴犲С缁狀噣宕ｉ娆庣鞍闊洨鏅弳鎰▔瀹ュ牜鍚�闁挎稑濂旂徊楣冨及椤栨氨鏆旈柛蹇嬪姀閹癸絿鎲存担鐣岀闁哄嫷鍨抽悡顐㈩潰閿濆嫮顏卞☉鎿勬嫹
//     BMX055_data.ACCXdata *= 0.001953125; //4 / 2048
//     BMX055_data.ACCYdata *= 0.001953125;
//     BMX055_data.ACCZdata *= 0.001953125;
//
//     Acc.Xdata = BMX055_data.ACCXdata;
//     Acc.Ydata = BMX055_data.ACCYdata;
//     Acc.Zdata = BMX055_data.ACCZdata;
//     Gyro.Xdata = BMX055_data.GYROXdata;
//     Gyro.Ydata = BMX055_data.GYROYdata;
//     Gyro.Zdata = BMX055_data.GYROZdata;
//
//     if (IsAttitudeinit == 0)
//     {
//         Quaternion_init(); // 濠殿喗瀵ч敓鎴掓祰琚欑紒鐘愁殔閸ㄥ灚鎱ㄧ�ｎ亜顕�
//         IsAttitudeinit = 1;
//     }
//     else
//     {
//         Attitude_UpdateGyro(); // 闊浂鍋婇敓鐣屽枑濞插潡寮敓锟�
//         Attitude_UpdateAcc();  // 婵烇絽宕�规娊鎽跺鍛�ら柡鍥х摠閺岋拷
//
//         SystemAttitude.Pitch = EulerAngle.Roll / PI * 180;         // 濞ｅ浂鍨拠婵堟喆閿燂拷
//         SystemAttitude.Roll = -EulerAngle.Pitch / PI * 180;        // 缂傚牏绮划瀵告喆閿燂拷
//         SystemAttitude.Yaw = EulerAngle.Yaw / PI * 180;            // 闁稿绻楅崺鍛喆閿燂拷
//         SystemAttitudeRate.Pitch = EulerAngleRate.Roll / PI * 180; // 濞ｅ浂鍨拠婵堟喆閹烘鎷烽悢宄邦唺
//         SystemAttitudeRate.Yaw = EulerAngleRate.Yaw / PI * 180;    // 闁稿绻楅崺鍛喆閹烘鎷烽悢宄邦唺
//
//         //        if (annulus_Meet_flag)
//         //        {
//         //            annulus_yaw = annulus_yaw_limit;
//         //        }
//         //        else if (IF.annulus == AL1 || IF.annulus == AL2 || IF.annulus == AL3 ||
//         //                 IF.annulus == AR1 || IF.annulus == AR2 || IF.annulus == AR3)
//         //        {
//         //            annulus_yaw = annulus_yaw + PERIODS * (SystemAttitudeRate.Yaw > 0 ? SystemAttitudeRate.Yaw : -1 * SystemAttitudeRate.Yaw);
//         //        }
//         //        else if (IF.annulus == AL4 || IF.annulus == AR4)
//         //            annulus_yaw = 0;
//
//         relativeYaw += subAngle(SystemAttitude.Yaw, previousSystemAttitude.Yaw);
//
//         float AccZ, AccZAdjust;
//         AccZ = -Acc.Zdata;
//         if (AccZ >= 1.0)
//         {
//             //            AccZ = 1;
//             AccZAngle = PI / 2;
//         }
//         else if (AccZ <= -1.0)
//         {
//             //            AccZ = -1;
//             AccZAngle = -PI / 2;
//         }
//         else
//         {
//             AccZAngle = asinf(AccZ) * 180 / PI;
//         }
//         AccZAdjust = (AccZAngle - SystemAttitude.Pitch);
//         SystemAttitude.Pitch += (-Gyro.Xdata + AccZAdjust) * PERIODS;
//
//         if (first == 0)
//         {
//             if (num < 1000)
//             {
//                 ++num;
//                 firstAngle.Pitch += SystemAttitude.Pitch;
//                 firstAngle.Roll += SystemAttitude.Roll;
//                 firstAngle.Yaw += SystemAttitude.Yaw;
//             }
//             else
//             {
//                 first = 1;
//                 firstAngle.Pitch /= 1000;
//                 firstAngle.Roll /= 1000;
//                 firstAngle.Yaw /= 1000;
//             }
//         }
//         else
//         {
//             angle.Pitch = -subAngle(SystemAttitude.Pitch, firstAngle.Pitch);
//             angle.Roll = -subAngle(SystemAttitude.Roll, firstAngle.Roll);
//             angle.Yaw = -subAngle(SystemAttitude.Yaw, firstAngle.Yaw);
//         }
//
//         previousSystemAttitude.Pitch = SystemAttitude.Pitch;
//         previousSystemAttitude.Roll = SystemAttitude.Roll;
//         previousSystemAttitude.Yaw = SystemAttitude.Yaw;
//     }
// }
//
//
///* 闂傚嫸鎷烽柧鏄忔〃閸楀酣姊块崼鏇ㄦ闁告帗绻傞～鎰板礌閿燂拷 */
// void GyroOffset_init(void)
//{
//     GyroOffset.Xdata = 0;
//     GyroOffset.Ydata = 0;
//     GyroOffset.Zdata = 0;
//     for (uint16 i = 0; i < 1000; ++i)
//     {
//         BMX055_DataRead(&BMX055_data, 0);
//         GyroOffset.Xdata += BMX055_data.GYROXdata;
//         GyroOffset.Ydata += BMX055_data.GYROYdata;
//         GyroOffset.Zdata += BMX055_data.GYROZdata;
//     }
//
//     GyroOffset.Xdata /= 1000;
//     GyroOffset.Ydata /= 1000;
//     GyroOffset.Zdata /= 1000;
//     IsGyroOffsetReset = 0;
// }
//
// uint8 BMX055_init(void)
//{
//
//     /************* 闁告梻濞�閿熺晫鍠庣�规娊鏌婂鍥╂瀭 *************/
//     uint8 ErrCount = 0;
//     while (simiic_read_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_ID, SIMIIC) != 0xFA) // 缁绢収鍠涢濠氭嚍椤栨粌顣籌D
//     {
//         ErrCount++;
//         if (ErrCount > 5)
//             return 0;
//     }
//
//     simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMURANGE, 0x05); // 4G  3:2G  5:4G  8:8G
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMUBW, 0x0F); // 1000HZ
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMULPM, 0x00); // Normal Mode
//     Common_delay(10);
//
//     /****** 闂傚嫸鎷烽柧鏄忔〃閸楀酣鏌婂鍥╂瀭 ******/
//     ErrCount = 0;
//     while (simiic_read_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_ID, SIMIIC) != 0x0F) // 缁绢収鍠涢濠氭嚍椤栨粌顣籌D
//     {
//         ErrCount++;
//         if (ErrCount > 5)
//             return 0;
//     }
//     simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RANGE, 0x01); // +-1000
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_BW, 0x02); // 1000HZ
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_LPM, 0x00); // Normal MODE
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RATEHBW, 0x08); // 濡ゅ倹锕㈤敓鑺ョ閹躲倕鈻旈敓锟� 闁挎稑鑻ぐ鍙夌閵夈倗鐟濋悷鏇嫹
//     Common_delay(10);
//
//     /********* 缁惧彞绀佹慨蹇曟媼閿熺姴甯崇紓鍐挎嫹 *********/
//     ErrCount = 0;
//     simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_POM, 0x81);
//     Common_delay(10);
//     while (simiic_read_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_ID, SIMIIC) != 0x32) // 缁绢収鍠涢濠氭嚍椤栨粌顣籌D
//     {
//         ErrCount++;
//         if (ErrCount > 5)
//             return 0;
//     }
//     simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_DATARATE, 0x38); // 閺夊牊鎸搁崵顓㈡焻閻斿搫鑺�30HZ
//     Common_delay(10);
//     simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_INTEN, 0x00); // 濞戞挸绉虫繛鍥嚄閹存帟鍘柡鍌︽嫹
//     Common_delay(10);
//
//     return 1;
// }
//
// uint8 BMX055_DataRead(BMX055Datatypedef *Q, uint8 type)
//{
//     uint8 datatemp[6] = {0};
//
//     /* 闂傚嫸鎷烽柧鏄忔〃閸楀酣寮悧鍫濈ウ閻犲洩顕цぐ锟� */
//     simiic_read_regs(IIC_BMX055_GYRO_ADR, BMX055_GYRO_XDATALSB, datatemp, 6, SIMIIC);
//     Q->GYROXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]));
//     Q->GYROYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]));
//     Q->GYROZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]));
//
//     /* 闁告梻濞�閿熺晫鍠庣�规娊寮悧鍫濈ウ閻犲洩顕цぐ锟� */
//     simiic_read_regs(IIC_BMX055_ACC_ADR, BMX055_ACC_XDATALSB, datatemp, 6, SIMIIC);
//     Q->ACCXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]) >> 4);
//     Q->ACCYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]) >> 4);
//     Q->ACCZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]) >> 4);
//
//     /* 缁惧彞绀佹慨蹇曟媼閳╁啯娈堕柟璇″枦椤曚即宕ｉ敓锟� */
//     if (type)
//     {
//         simiic_read_regs(IIC_BMX055_MAG_ADR, BMX055_MAG_XDATALSB, datatemp, 6, SIMIIC);
//         Q->MAGXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]) >> 3);
//         Q->MAGYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]) >> 3);
//         Q->MAGZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]) >> 1);
//     }
//
//     return 1;
// }
