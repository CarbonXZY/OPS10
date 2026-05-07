#include "tsk_config_and_callback.h"
#include "drv_spi.h"
#include "drv_tim.h"
#include "dvc_ICM42688.h"
#include "dvc_dwt.h"
#include "stm32h5xx_hal.h"

bool init_finished = false;
Class_ICM42688 ICM42688;

void Task_Init(void)
{
    DWT_Init();
    SPI_Init(&hspi2, nullptr);
    SPI2_Manage_Object.Now_GPIOx = GPIOB;
    SPI2_Manage_Object.Now_GPIO_Pin = GPIO_PIN_12;
    TIM_Init(&htim2, Task1ms_TIM2_Callback);

    ICM42688.Init(&hspi2);

    HAL_TIM_Base_Start_IT(&htim2);

    // 标记初始化完成
    init_finished = true;
}

void Task1ms_TIM2_Callback()
{
    ICM42688.getAGT();
}