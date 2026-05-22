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
    chariot.Init();

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    // 标记初始化完成
    init_finished = true;
}

void Task1ms_TIM2_Callback()
{
    chariot.TIM_Calculate_PeriodElapsedCallback();
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
    if(chariot.mmc5983.ReadMagnet() == MMC5983MA_OK)
    {
        chariot.is_magnetometer_valid = true;
    }
    else
    {
        chariot.is_magnetometer_valid = false;
    }
    float mag_x = chariot.mmc5983.magnet_data.x_gauss - 0.05f;
    float mag_y = chariot.mmc5983.magnet_data.y_gauss - 0.05f;


    chariot.yaw_mag = atan2f(mag_y, mag_x) * 180.0f / PI - chariot.yaw_mag_offset;
}

