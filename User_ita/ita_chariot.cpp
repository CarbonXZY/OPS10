#include "ita_chariot.h"
#include <algorithm>
#include <array>
#include <numeric>

// static std::array<float, 100> mag_x_buffer = {0};
// static std::array<float, 100> mag_y_buffer = {0};
uint8_t mag_buffer_index = 0;
void Class_Chariot::Init()
{
    // 传感器初始化
    // icm42688.Init(&hspi2);

    // bmp388.Init(&hi2c2);
    // mmc5983.Init(&hi2c1);

    lsm6dsv16x.Init(&hspi2,GPIOB, GPIO_PIN_12);
    mt6816_x.Init(&hspi1, GPIOC, GPIO_PIN_1);
    mt6816_y.Init(&hspi3, GPIOA, GPIO_PIN_0);

    // IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    //  EKF.Init(10.0f, 0.001f, 1000000.0f, 1000.0f, 0.9996f, 0.0f, 0.0f, true);

    //Calibrate_Magnetometer();
}

float dt = 0.0f;
//int stable_samples = 0;
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    dt = DWT_GetDeltaT(&dwt_cnt);
    Update_SensorData();

    IMU_QuaternionEKF_Update(lsm6dsv16x.Get_GyrX() / 180.0f * PI, lsm6dsv16x.Get_GyrY() / 180.0f * PI, lsm6dsv16x.Get_GyrZ() / 180.0f * PI,
                             lsm6dsv16x.Get_AccX(), lsm6dsv16x.Get_AccY(), lsm6dsv16x.Get_AccZ(), dt);

    // if (is_roll_pitch_base_calibrated)
    // {
    //     UpdateYawKalman(gyro_z, yaw_mag, dt);
    // }
    // else
    // {
    //     // 使用 EKF 内部的收敛标志
    //     if (QEKF_INS.ConvergeFlag == 1)
    //     {
    //         // EKF 已经收敛且载体静止
    //         stable_samples++;

    //         if (stable_samples > 1000) // 连续稳定1000帧
    //         {
    //             base_pitch = QEKF_INS.Pitch;
    //             base_roll = QEKF_INS.Roll;
    //             is_roll_pitch_base_calibrated = true;
    //         }
    //     }
    //     else
    //     {
    //         stable_samples = 0;
    //     }
    // }
}
// /**
//  * @brief 磁力计校准函数，使用旋转方法采集数据并计算偏置和缩放因子
//  */
// void Class_Chariot::Calibrate_Magnetometer()
// {
//     mmc5983.ReadMagnetOffset();

//     while (!start_calibration)
//     {
//         // 等待校准开始的标志
//         HAL_Delay(100);
//     }

//     while (start_calibration)
//     {
//         if (mmc5983.ReadMagnet() == MMC5983MA_OK)
//         {
//             // 将新的磁力计数据加入缓冲区
//             std::rotate(mag_x_buffer.begin(), mag_x_buffer.begin() + 1, mag_x_buffer.end());
//             std::rotate(mag_y_buffer.begin(), mag_y_buffer.begin() + 1, mag_y_buffer.end());
//             mag_x_buffer.back() = mmc5983.magnet_data.x_gauss;
//             mag_y_buffer.back() = mmc5983.magnet_data.y_gauss;
//             mag_buffer_index++;
//         }

//         mag_x_max = *std::max_element(mag_x_buffer.begin(), mag_x_buffer.end());
//         mag_x_min = *std::min_element(mag_x_buffer.begin(), mag_x_buffer.end());
//         mag_y_max = *std::max_element(mag_y_buffer.begin(), mag_y_buffer.end());
//         mag_y_min = *std::min_element(mag_y_buffer.begin(), mag_y_buffer.end());

//         HAL_Delay(100);
//     }

//     mag_x_offset = (mag_x_max + mag_x_min) / 2.0f;
//     mag_y_offset = (mag_y_max + mag_y_min) / 2.0f;
//     mag_y_x_scale = (mag_y_max - mag_y_min) / (mag_x_max - mag_x_min);
// }
uint8_t result;
void Class_Chariot::TIM_Communicate_PeriodElapsedCallback()
{
		result = 0;
    uint8_t buffer[] = {0,1,2,3,4,5,6,7};
    result = FDCAN_Send_Data(&hfdcan1,0x200,buffer,FDCAN_ID_Standard,8);
}

void Class_Chariot::Update_SensorData()
{
    // 更新传感器数据，注意要先保存上一次的数据以计算增量
    //    last_x = now_x;
    //    last_y = now_y;

    // icm42688.Get_Data();
    lsm6dsv16x.Get_Data();
    // bmp388.TIM_Calculate_PeriodElapsedCallback();
    // mmc5983.ReadMagnet();
    // mmc5983.ReadTemperature();
    mt6816_x.ReadAngle();
    mt6816_y.ReadAngle();

    // now_x = mt6816_x.GetAngleRad();
    // now_y = mt6816_y.GetAngleRad();
}

void Class_Chariot::Calculate_Position()
{
    //    // 自身坐标系下的位置增量
    //    float dx_self = now_x - last_x;
    //    float dy_self = now_y - last_y;

    //    // 转换到全局坐标系
    //    float dx = dx_self * arm_cos_f32(yaw) - dy_self * arm_sin_f32(yaw);
    //    float dy = dx_self * arm_sin_f32(yaw) + dy_self * arm_cos_f32(yaw);

    //    Position_X += dx;
    //    Position_Y += dy;
}

// float Class_Chariot::NormalizeAngle(float angle)
// {
//     while (angle > 180.0f)
//         angle -= 360.0f;
//     while (angle < -180.0f)
//         angle += 360.0f;
//     return angle;
// }

// void Class_Chariot::UpdateYawKalman(float gyro_z, float yaw_meas, float dt)
// {
//     float gyro_z_deg = gyro_z * 180.0f / PI;
//     float yaw_predict = yaw_kalman + gyro_z_deg * dt;
//     yaw_predict = NormalizeAngle(yaw_predict);

//     float residual = NormalizeAngle(yaw_meas - yaw_predict);

//     yaw_kalman_P += yaw_kalman_Q * dt;

//     float K = yaw_kalman_P / (yaw_kalman_P + yaw_kalman_R);

//     yaw_kalman = yaw_predict + K * residual;
//     yaw_kalman = NormalizeAngle(yaw_kalman);

//     yaw_kalman_P = (1.0f - K) * yaw_kalman_P;
// }