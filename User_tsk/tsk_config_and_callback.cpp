#include "tsk_config_and_callback.h"
#include "drv_spi.h"
#include "drv_tim.h"
#include "ita_chariot.h"
#include "dvc_dwt.h"
#include "stm32h5xx_hal.h"

bool init_finished = false;
Class_Chariot chaiot;

void Task_Init(void)
{
    DWT_Init();
    SPI_Init(&hspi2, nullptr);
    TIM_Init(&htim2, Task1ms_TIM2_Callback);

    chaiot.Init();

    HAL_TIM_Base_Start_IT(&htim2);

    // 标记初始化完成
    init_finished = true;
}

static uint32_t time1 ;
static uint32_t time2 ;
void Task1ms_TIM2_Callback()
{
		time1 = DWT_GetCurrentTimeUs();
    chaiot.TIM_Calculate_PeriodElapsedCallback();
    time2 = DWT_GetCurrentTimeUs();
    chaiot.TIM_Communicate_PeriodElapsedCallback();
}