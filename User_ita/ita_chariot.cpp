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
    EKF.Init(10.0f,0.001f,1000000.0f,100000.0f,0.9996f,0.0f,0.0f,true);

    mmc5983.ReadMagnetOffset();
}

float dt = 0.0f;
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    dt = DWT_GetDeltaT(&dwt_cnt);
    Update_SensorData();

    //    MahonyAHRSupdate(
    //         icm42688.Get_GyrX(), icm42688.Get_GyrY(), icm42688.Get_GyrZ(),
    //         icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(),
    //         mmc5983.magnet_data.x, mmc5983.magnet_data.y, mmc5983.magnet_data.z);

    //    IMU_QuaternionEKF_Update(
    //        icm42688.Get_GyrX() / 180.0f * PI, icm42688.Get_GyrY() / 180.0f * PI, icm42688.Get_GyrZ() / 180.0f * PI,
    //        icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(), dt);

        EKF.Update(icm42688.Get_GyrX() / 180.0f * PI, icm42688.Get_GyrY() / 180.0f * PI, icm42688.Get_GyrZ() / 180.0f * PI,
                    icm42688.Get_AccX(), icm42688.Get_AccY(), icm42688.Get_AccZ(),
                    mmc5983.magnet_data.x_gauss, mmc5983.magnet_data.y_gauss, mmc5983.magnet_data.z_gauss, dt);
    

    
        //    Update_Position();
        //    Update_Servo();
        //    Update_CAN();
        //    Update_Bluetooth();
        //    Update_Display();
    
        //    if (dt > 0.02f) dt = 0.02f; // 限制最大dt以防止异常值
    
        //    printf("dt: %.4f s\n", dt);
    // Calculate_Position();
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