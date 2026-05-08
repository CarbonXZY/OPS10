#include "ita_chaiot.h"

void Class_Chariot::Init()
{
    icm42688.Init(&hspi2);
    bmp388.Init(&hi2c2);
    mmc5983.Init(&hi2c1);
}

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    icm42688.Get_Data();
    bmp388.TIM_Calculate_PeriodElapsedCallback();
    mmc5983.ReadMagnet();
    mmc5983.ReadTemperature();
}