#include "ita_chariot.h"

void Class_Chariot::Init()
{
    // 传感器初始化
    icm42688.Init(&hspi2);

    bmp388.Init(&hi2c2);
    mmc5983.Init(&hi2c1);
    // mt6816_x.Init(&hspi1, GPIOC, GPIO_PIN_1);
    // mt6816_y.Init(&hspi3, GPIOA, GPIO_PIN_0);

    //IMU_QuaternionEKF_Init(0.01f, 0.01f, 0.1f, 0.5f, 0.1f);
    mmc5983.ReadMagnetOffset();
}

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    Update_SensorData();

//    MahonyAHRSupdate(
//         icm42688.Get_GyrX(), icm42688.Get_GyrY(), icm42688.Get_GyrZ(),
//         icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(),
//         mmc5983.magnet_data.x, mmc5983.magnet_data.y, mmc5983.magnet_data.z);

    MahonyAHRSupdateIMU(icm42688.Get_GyrX(), icm42688.Get_GyrY(), icm42688.Get_GyrZ(),icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ());
        //Calculate_Position();
}

void Class_Chariot::TIM_Communicate_PeriodElapsedCallback()
{
    // 这里可以添加发送CAN数据的代码
}

void Class_Chariot::Update_SensorData()
{
    // 更新传感器数据，注意要先保存上一次的数据以计算增量
//    last_x = now_x;
//    last_y = now_y;

    icm42688.Get_Data();
    bmp388.TIM_Calculate_PeriodElapsedCallback();
    mmc5983.ReadMagnet();
    mmc5983.ReadTemperature();
    // mt6816_x.ReadAngle();
    // mt6816_y.ReadAngle();

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