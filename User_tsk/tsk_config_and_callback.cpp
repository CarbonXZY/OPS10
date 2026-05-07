#include "tsk_config_and_callback.h"
#include "drv_spi.h"
#include "drv_tim.h"
#include "dvc_ICM42688.h"
#include "dvc_dwt.h"
#include "stm32h5xx_hal.h"
#include "dvc_BMP388.h"
#include "dvc_MCC5983.h"

bool init_finished = false;
Class_ICM42688 ICM42688;
Class_BMP388 BMP388;
Class_MCC5983 MCC5983;

void Task_Init(void)
{
    DWT_Init();
    SPI_Init(&hspi2, nullptr);
    TIM_Init(&htim2, Task1ms_TIM2_Callback);

    ICM42688.Init(&hspi2);
    BMP388.Init(&hi2c1);
    MCC5983.Init(&hi2c2);

    HAL_TIM_Base_Start_IT(&htim2);

    // 标记初始化完成
    init_finished = true;
}

void Task1ms_TIM2_Callback()
{
    ICM42688.Get_Data();
    BMP388.TIM_Calculate_PeriodElapsedCallback();
    MCC5983.TIM_Calculate_PeriodElapsedCallback();
}