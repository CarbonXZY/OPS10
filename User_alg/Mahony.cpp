#include "Mahony.h"

// Definitions
#define sampleFreq 100.0f      // sample frequency in Hz
#define twoKpDef (2.0f * 0.1f) // 2 * proportional gain (降低以抑制加速度计高频噪声)
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

// Variable definitions
volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;

static float invSqrt(float x);

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
   float recipNorm;
   float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
   float hx, hy, hz, bx, bz;
   float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
   float halfex, halfey, halfez;
   float qa, qb, qc;

   // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
   // 在磁力计数据无效时使用六轴融合算法
   if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
   {
       MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
       return;
   }

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   // 只在加速度计数据有效时才进行运算
   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
   {

       // Normalise accelerometer measurement
       // 将加速度计得到的实际重力加速度向量v单位化
       recipNorm = invSqrt(ax * ax + ay * ay + az * az);
       ax *= recipNorm;
       ay *= recipNorm;
       az *= recipNorm;

       // Normalise magnetometer measurement
       // 将磁力计得到的实际磁场向量m单位化
       recipNorm = invSqrt(mx * mx + my * my + mz * mz);
       mx *= recipNorm;
       my *= recipNorm;
       mz *= recipNorm;

       // Auxiliary variables to avoid repeated arithmetic
       // 辅助变量，以避免重复运算
       q0q0 = q0 * q0;
       q0q1 = q0 * q1;
       q0q2 = q0 * q2;
       q0q3 = q0 * q3;
       q1q1 = q1 * q1;
       q1q2 = q1 * q2;
       q1q3 = q1 * q3;
       q2q2 = q2 * q2;
       q2q3 = q2 * q3;
       q3q3 = q3 * q3;

       // Reference direction of Earth's magnetic field
       // 通过磁力计测量值与坐标转换矩阵得到大地坐标系下的理论地磁向量
       hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
       hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
       hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
       bx = sqrt(hx * hx + hy * hy);
       bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

       // Estimated direction of gravity and magnetic field
       // 将理论重力加速度向量与理论地磁向量变换至机体坐标系
       halfvx = q1q3 - q0q2;
       halfvy = q0q1 + q2q3;
       halfvz = q0q0 - 0.5f + q3q3;
       halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
       halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
       halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

       // Error is sum of cross product between estimated direction and measured direction of field vectors
       // 通过向量外积得到重力加速度向量和地磁向量的实际值与测量值之间误差
       halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
       halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
       halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

       // Compute and apply integral feedback if enabled
       // 在PI补偿器中积分项使能情况下计算并应用积分项
       if (twoKi > 0.0f)
       {
           // integral error scaled by Ki
           // 积分过程
           integralFBx += twoKi * halfex * (1.0f / sampleFreq);
           integralFBy += twoKi * halfey * (1.0f / sampleFreq);
           integralFBz += twoKi * halfez * (1.0f / sampleFreq);

           // apply integral feedback
           // 应用误差补偿中的积分项
           gx += integralFBx;
           gy += integralFBy;
           gz += integralFBz;
       }
       else
       {
           // prevent integral windup
           // 避免为负值的Ki时积分异常饱和
           integralFBx = 0.0f;
           integralFBy = 0.0f;
           integralFBz = 0.0f;
       }

       // Apply proportional feedback
       // 应用误差补偿中的比例项
       gx += twoKp * halfex;
       gy += twoKp * halfey;
       gz += twoKp * halfez;
   }

   // Integrate rate of change of quaternion
   // 微分方程迭代求解
   gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
   gy *= (0.5f * (1.0f / sampleFreq));
   gz *= (0.5f * (1.0f / sampleFreq));
   qa = q0;
   qb = q1;
   qc = q2;
   q0 += (-qb * gx - qc * gy - q3 * gz);
   q1 += (qa * gx + qc * gz - q3 * gy);
   q2 += (qa * gy - qb * gz + q3 * gx);
   q3 += (qa * gz + qb * gy - qc * gx);

   // Normalise quaternion
   // 单位化四元数 保证四元数在迭代过程中保持单位性质
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;

   // Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
   roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
   pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
   yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
   float recipNorm;
   float halfvx, halfvy, halfvz;
   float halfex, halfey, halfez;
   float qa, qb, qc;

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   // 只在加速度计数据有效时才进行运算
   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
   {

       // Normalise accelerometer measurement
       // 将加速度计得到的实际重力加速度向量v单位化
       recipNorm = invSqrt(ax * ax + ay * ay + az * az); // 该函数返回平方根的倒数
       ax *= recipNorm;
       ay *= recipNorm;
       az *= recipNorm;

       // Estimated direction of gravity
       // 通过四元数得到理论重力加速度向量g
       // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
       // 这样处理目的是减少乘法运算量
       halfvx = q1 * q3 - q0 * q2;
       halfvy = q0 * q1 + q2 * q3;
       halfvz = q0 * q0 - 0.5f + q3 * q3;

       // Error is sum of cross product between estimated and measured direction of gravity
       // 对实际重力加速度向量v与理论重力加速度向量g做外积
       halfex = (ay * halfvz - az * halfvy);
       halfey = (az * halfvx - ax * halfvz);
       halfez = (ax * halfvy - ay * halfvx);

       // Compute and apply integral feedback if enabled
       // 在PI补偿器中积分项使能情况下计算并应用积分项
       if (twoKi > 0.0f)
       {
           // integral error scaled by Ki
           // 积分过程
           integralFBx += twoKi * halfex * (1.0f / sampleFreq);
           integralFBy += twoKi * halfey * (1.0f / sampleFreq);
           integralFBz += twoKi * halfez * (1.0f / sampleFreq);

           // apply integral feedback
           // 应用误差补偿中的积分项
           gx += integralFBx;
           gy += integralFBy;
           gz += integralFBz;
       }
       else
       {
           // prevent integral windup
           // 避免为负值的Ki时积分异常饱和
           integralFBx = 0.0f;
           integralFBy = 0.0f;
           integralFBz = 0.0f;
       }

       // Apply proportional feedback
       // 应用误差补偿中的比例项
       gx += twoKp * halfex;
       gy += twoKp * halfey;
       gz += twoKp * halfez;
   }

   // Integrate rate of change of quaternion
   // 微分方程迭代求解
   gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
   gy *= (0.5f * (1.0f / sampleFreq));
   gz *= (0.5f * (1.0f / sampleFreq));
   qa = q0;
   qb = q1;
   qc = q2;
   q0 += (-qb * gx - qc * gy - q3 * gz);
   q1 += (qa * gx + qc * gz - q3 * gy);
   q2 += (qa * gy - qb * gz + q3 * gx);
   q3 += (qa * gz + qb * gy - qc * gx);

   // Normalise quaternion
   // 单位化四元数 保证四元数在迭代过程中保持单位性质
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;

   // Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
   roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
   pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
   yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

/**
* @brief 自定义1/sqrt(x),速度更快
*
* @param x x
* @return float
*/
static float invSqrt(float x)
{
   float halfx = 0.5f * x;
   float y = x;
   long i = *(long *)&y;
   i = 0x5f375a86 - (i >> 1);
   y = *(float *)&i;
   y = y * (1.5f - (halfx * y * y));
   return y;
}
