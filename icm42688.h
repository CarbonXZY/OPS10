#ifndef SKYFCS_ICM42688P_H
#define SKYFCS_ICM42688P_H

#include "stm32h5xx_hal.h"

void ICM42688_Init(void);
void ICM42688_WriteRegister(uint8_t reg, uint8_t data);
uint8_t ICM42688_ReadRegister(uint8_t reg);
void ReadSensorData(int16_t* accel, int16_t* gyro);
float ICM42688_ReadTemperature(void);
void ConvertRawData(int16_t raw_accel[3], int16_t raw_gyro[3], float* accel_g, float* gyro_dps);
#endif //SKYFCS_ICM42688P_H

