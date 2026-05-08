#ifndef ITA_CHARIOT_H
#define ITA_CHARIOT_H

#include "dvc_MMC5983.h"
#include "dvc_BMP388.h"
#include "dvc_ICM42688.h"
#include "dvc_MT6816.h"

class Class_Chariot
{
public:
    Class_BMP388 bmp388;
    Class_ICM42688 icm42688;
    Class_MMC5983 mmc5983;
    //Class_MT6816 mt6816;

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();

private:

};
#endif