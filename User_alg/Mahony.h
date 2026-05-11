#ifndef MAHONY_H
#define MAHONY_H

#include "drv_math.h"

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

extern float yaw, pitch, roll;
#endif