#ifndef DVC_BMP388_H
#define DVC_BMP388_H

#include "stm32h5xx_hal.h"
#include "i2c.h"

#define BMP388_I2C_ADDR 0xEC

#define REG_CHIP_ID 0x00
#define REG_ERR_REG 0x02
#define REG_STATUS 0x03
#define REG_PRESS_XLSB 0x04
#define REG_TEMP_XLSB 0x07
#define REG_INT_STATUS 0x39
#define REG_FIFO_TIME 0x3C
#define REG_CMD 0x7E
#define REG_PWR_CTRL 0x7D
#define REG_OSR 0x75
#define REG_CFG 0x73

#define CMD_SOFTRESET 0xB6
#define CMD_FIFO_FLUSH 0xB0

#define PWR_PRESS_EN 0x01
#define PWR_TEMP_EN 0x02
#define PWR_NORMAL 0x03
#define PWR_SLEEP 0x00

#define OSR_1X 0x00
#define OSR_2X 0x01
#define OSR_4X 0x02
#define OSR_8X 0x03
#define OSR_16X 0x04
#define OSR_32X 0x05

#define IIR_FILTER_DISABLE 0x00
#define IIR_FILTER_2 0x01
#define IIR_FILTER_4 0x02
#define IIR_FILTER_8 0x03
#define IIR_FILTER_16 0x04
#define IIR_FILTER_32 0x05
#define IIR_FILTER_64 0x06
#define IIR_FILTER_128 0x07

#define INT_STATUS_DRDY 0x01

struct BMP388_Calib_t
{
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

class Class_BMP388
{
public:
    void Init(I2C_HandleTypeDef *hi2c);
    void TIM_Calculate_PeriodElapsedCallback();

    inline float GetPressure() { return Pressure; }
    inline float GetTemperature() { return Temperature; }

protected:
    HAL_StatusTypeDef Write_Register(uint8_t reg, uint8_t data);
    HAL_StatusTypeDef Read_Register(uint8_t reg, uint8_t *buf, uint16_t len);

    void Read_Calibration();
    void Compensate_Temperature();
    void Compensate_Pressure();

    I2C_HandleTypeDef *hi2c;

    BMP388_Calib_t Calibration_Data;

    uint32_t raw_pressure;
    uint32_t raw_temperature;

    float Pressure;
    float Temperature;
};

#endif // DVC_BMP388_H