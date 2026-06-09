#ifndef ITA_CHARIOT_H
#define ITA_CHARIOT_H

#include "Mahony.h"
#include "QuaternionEKF.h"
#include "arm_math.h"
#include "drv_can.h"
#include "dvc_BMP388.h"
#include "dvc_ICM42688.h"
#include "dvc_MMC5983.h"
#include "dvc_MT6816.h"
#include "dvc_dwt.h"
#include "dvc_LSM6DSV16X.h"
class Class_Chariot
{
public:
    // QuaternionEKF<7, 6> EKF; // 四元数EKF，状态维度7（4四元数+3陀螺仪零偏），观测维度6（3加速度计+3磁力计）
    //  传感器对象
    //  BMP388气压计
    Class_BMP388 bmp388;

    // // ICM42688陀螺仪和加速度计
    // Class_ICM42688 icm42688;

    // LSM6DSV16X陀螺仪和加速度计
    Class_LSM6DSV16X lsm6dsv16x;

    // MMC5983磁力计
    Class_MMC5983 mmc5983;

    // MT6816角度传感器
    Class_MT6816 mt6816_x;
    Class_MT6816 mt6816_y;

    bool is_magnetometer_valid = false; // 磁力计数据有效标志

    float yaw_mag = 0.0f;
    float yaw_mag_offset = 0.0f; // 磁力计航向偏移值，需根据实际情况标定
    float yaw_kalman = 0.0f;

    float mag_x_max = 0.0f;
    float mag_x_min = 0.0f;
    float mag_y_max = 0.0f;
    float mag_y_min = 0.0f;
    float mag_y_x_scale = 1.0f;

    float mag_x_offset = 0.0f; // 磁力计X轴偏置
    float mag_y_offset = 0.0f; // 磁力计Y轴偏置
    // float mag_z_offset = 0.0f; // 磁力计Z轴偏置

    float base_pitch = 0.0f;                    // 基准俯仰角
    float base_roll = 0.0f;                     // 基准横滚角
    bool is_roll_pitch_base_calibrated = false; // 是否完成俯仰角和横滚角基准校准

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Communicate_PeriodElapsedCallback();

private:
    float yaw_kalman_P = 1.0f;
    float yaw_kalman_Q = 100.0f;
    float yaw_kalman_R = 1.0f;

    float NormalizeAngle(float angle);
    void UpdateYawKalman(float gyro_z, float yaw_meas, float dt);

    float Position_X = 0.0f;
    float Position_Y = 0.0f;
    // float Position_Z; // 未来需要高度可以添加

    float last_x = 0.0f, last_y = 0.0f; // 上一次的位置
    float now_x = 0.0f, now_y = 0.0f;   // 当前的位置

    uint32_t dwt_cnt = 0; // DWT计数器，用于计算时间增量

    bool mag_is_calibrated = false; // 磁力计是否完成校准的标志
    bool start_calibration = false; // 是否开始校准的标志

    void Update_SensorData();
    void Calculate_Position();
    void Calibrate_Magnetometer();
};
#endif