#include "tsk_config_and_callback.h"
#include "drv_spi.h"
#include "drv_tim.h"
#include "dvc_dwt.h"
#include "ita_chariot.h"
#include "stm32h5xx_hal.h"

bool init_finished = false;

Class_Chariot chariot;

void Task_Init(void)
{
    DWT_Init();
    FDCAN_Init(&hfdcan1, nullptr);
    SPI_Init(&hspi1, nullptr);
    SPI_Init(&hspi2, nullptr);
    SPI_Init(&hspi3, nullptr);
    TIM_Init(&htim2, Task1ms_TIM2_Callback);
    TIM_Init(&htim3, Task10ms_TIM3_Callback);
//    chariot.Init();

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    // 标记初始化完成
    init_finished = true;
}

void Task1ms_TIM2_Callback()
{
//    chariot.TIM_Calculate_PeriodElapsedCallback();
    chariot.TIM_Communicate_PeriodElapsedCallback();
}

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

void Task10ms_TIM3_Callback()
{

//    chariot.mmc5983.ReadMagnet();

//    float mag_x = chariot.mmc5983.magnet_data.x_gauss - chariot.mag_x_offset;
//    float mag_y = chariot.mmc5983.magnet_data.y_gauss - chariot.mag_y_offset;
//    float mag_z = chariot.mmc5983.magnet_data.z_gauss;

//    if (chariot.is_magnetometer_valid)
//    {
////        if (Math_Abs(QEKF_INS.Pitch - chariot.base_pitch) > 1.0f || Math_Abs(QEKF_INS.Roll - chariot.base_roll) > 1.0f)
////        {
////            // 如果当前俯仰角或横滚角与基准值相差较大，说明姿态不稳定，暂不更新磁力计航向
////            return;
////        }
//        chariot.yaw_mag = atan2f(mag_y, mag_x * chariot.mag_y_x_scale) * 180.0f / PI - chariot.yaw_mag_offset;
//    }
//    else
//    {
//        chariot.yaw_mag_offset = atan2f(mag_y, mag_x * chariot.mag_y_x_scale) * 180.0f / PI;
//        chariot.is_magnetometer_valid = true;
//    }
}
