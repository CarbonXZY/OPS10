#ifndef ITA_CHARIOT_H
#define ITA_CHARIOT_H

#include "Mahony.h"
#include "QuaternionEKF.h"
#include "Quaternion_EKF.hpp"
#include "arm_math.h"
#include "drv_can.h"
#include "dvc_BMP388.h"
#include "dvc_ICM42688.h"
#include "dvc_MMC5983.h"
#include "dvc_MT6816.h"
#include "dvc_dwt.h"
class Class_Chariot
{
public:
    //QuaternionEKF<7, 6> EKF; // 四元数EKF，状态维度7（4四元数+3陀螺仪零偏），观测维度6（3加速度计+3磁力计）
    // 传感器对象
    // BMP388气压计
    Class_BMP388 bmp388;

    // ICM42688陀螺仪和加速度计
    Class_ICM42688 icm42688;

    // MMC5983磁力计
    Class_MMC5983 mmc5983;

    // MT6816角度传感器
    Class_MT6816 mt6816_x;
    Class_MT6816 mt6816_y;

    
    bool is_magnetometer_valid = false; // 磁力计数据有效标志

    float yaw_mag = 0.0f;
    float yaw_mag_offset = 0.0f; // 磁力计航向偏移值，需根据实际情况标定
    float yaw_kalman = 0.0f;
    float gyro_z_kalman = 0.0f;

    float bias_lpf_alpha = 0.8f; // 零偏低通滤波系数，范围0-1，值越小滤波越平滑但响应越慢
    float gyro_bias = 0.0f; // 陀螺仪零偏估计值
    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Communicate_PeriodElapsedCallback();

private:
    float yaw_kalman_P = 1.0f;
    float yaw_kalman_Q = 100.0f;
    float yaw_kalman_R = 1.0f;

    float NormalizeAngle(float angle);
    void UpdateYawKalman(float gyro_z, float yaw_meas, float dt);

private:
    float Position_X = 0.0f;
    float Position_Y = 0.0f;
    // float Position_Z; // 未来需要高度可以添加

    float last_x = 0.0f, last_y = 0.0f; // 上一次的位置
    float now_x = 0.0f, now_y = 0.0f;   // 当前的位置

    uint32_t dwt_cnt = 0; // DWT计数器，用于计算时间增量


    
    void Update_SensorData();
    void Calculate_Position();
};
#endif