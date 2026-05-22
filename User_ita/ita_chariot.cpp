#include "ita_chariot.h"

void Class_Chariot::Init()
{
    // 传感器初始化
    icm42688.Init(&hspi2);

    bmp388.Init(&hi2c2);
    mmc5983.Init(&hi2c1);
    mt6816_x.Init(&hspi1, GPIOC, GPIO_PIN_1);
    mt6816_y.Init(&hspi3, GPIOA, GPIO_PIN_0);

    // IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996, 0);
    // EKF.Init(10.0f, 0.001f, 1000000.0f, 1000.0f, 0.9996f, 0.0f, 0.0f, true);

    mmc5983.ReadMagnetOffset();

    mmc5983.ReadMagnet();
    float mag_x = mmc5983.magnet_data.x_gauss - 0.05f;
    float mag_y = mmc5983.magnet_data.y_gauss - 0.05f;

   // yaw_mag_offset = atan2f(mag_y, mag_x) * 180.0f / PI;
}

float dt = 0.0f;
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    dt = DWT_GetDeltaT(&dwt_cnt);
    Update_SensorData();

    //    // 动态裁剪: 1ms周期用6轴, 每10ms周期用9轴
    // if (is_magnetometer_valid)
    // {
    //     EKF.Update(icm42688.Get_GyrX() / 180.0f * PI, icm42688.Get_GyrY() / 180.0f * PI, icm42688.Get_GyrZ() / 180.0f * PI,
    //                 icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(),
    //                 mmc5983.magnet_data.x_gauss, mmc5983.magnet_data.y_gauss, mmc5983.magnet_data.z_gauss, dt);
    // 		is_magnetometer_valid = false;
    // }
    // else
    // {
    // EKF.UpdateAccelOnly(icm42688.Get_GyrX() / 180.0f * PI, icm42688.Get_GyrY() / 180.0f * PI, icm42688.Get_GyrZ() / 180.0f * PI,
    //                      icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(), dt);
    // }
    float gyro_z = icm42688.Get_GyrZ() / 180.0f * PI;
    IMU_QuaternionEKF_Update(icm42688.Get_GyrX() / 180.0f * PI, icm42688.Get_GyrY() / 180.0f * PI, gyro_z,
                             icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(), dt);
    UpdateYawKalman(gyro_z, yaw_mag, dt);
}

void Class_Chariot::TIM_Communicate_PeriodElapsedCallback()
{
    //    uint8_t buffer[] = {0,1,2,3,4,5,6,7};
    //    FDCAN_Send_Data(&hfdcan1,0x200,buffer,FDCAN_ID_Standard,8);
}

void Class_Chariot::Update_SensorData()
{
    // 更新传感器数据，注意要先保存上一次的数据以计算增量
    //    last_x = now_x;
    //    last_y = now_y;

    icm42688.Get_Data();
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

float Class_Chariot::NormalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// void Class_Chariot::UpdateYawKalman(float gyro_z, float yaw_meas, float dt)
// {
//     float gyro_z_deg = gyro_z * 180.0f / PI;
    
//     // 【修改 1】预测时，直接扣除上一次计算出来的零偏
//     float yaw_predict = yaw_kalman + gyro_z_deg  * dt;
//     yaw_predict = NormalizeAngle(yaw_predict);

//     float residual = NormalizeAngle(yaw_meas - yaw_predict);

//     yaw_kalman_P += yaw_kalman_Q * dt;

//     float K = yaw_kalman_P / (yaw_kalman_P + yaw_kalman_R);

//     // 【修改 2】核心：在更新 yaw 之前，计算由零偏导致的步长误差，并通过低通滤波累加给 gyro_bias
//     // K * residual 是卡尔曼给角度的修正量，除以 dt 变换回角速度维度 (deg/s)
//     float instant_bias = (K * residual) / dt;
    
//     // 一阶低通滤波，慢慢把这个零偏“吸”出来
//     gyro_bias += bias_lpf_alpha * instant_bias; 

//     // 后续逻辑完全保持你原来的样子不变
//     yaw_kalman = yaw_predict + K * residual;
//     yaw_kalman = NormalizeAngle(yaw_kalman);

//     yaw_kalman_P = (1.0f - K) * yaw_kalman_P;
    
// }

void Class_Chariot::UpdateYawKalman(float gyro_z, float yaw_meas, float dt)
{
    float gyro_z_deg = gyro_z * 180.0f / PI;
    float yaw_predict = yaw_kalman + gyro_z_deg * dt;
    yaw_predict = NormalizeAngle(yaw_predict);

    float residual = NormalizeAngle(yaw_meas - yaw_predict);

    yaw_kalman_P += yaw_kalman_Q * dt;

    float K = yaw_kalman_P / (yaw_kalman_P + yaw_kalman_R);

    yaw_kalman = yaw_predict + K * residual;
    yaw_kalman = NormalizeAngle(yaw_kalman);

    yaw_kalman_P = (1.0f - K) * yaw_kalman_P;
}