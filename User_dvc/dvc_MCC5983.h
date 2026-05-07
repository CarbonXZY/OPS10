#ifndef DVC_MCC5983_H
#define DVC_MCC5983_H

#include "stm32h5xx_hal.h"
#include "i2c.h"

#define MCC5983_I2C_ADDR 0x60

#define REG_XOUT0     0x00
#define REG_XOUT1     0x01
#define REG_YOUT0     0x02
#define REG_YOUT1     0x03
#define REG_ZOUT0     0x04
#define REG_ZOUT1     0x05
#define REG_XYZOUT2   0x06
#define REG_TOUT      0x07
#define REG_STATUS    0x08
#define REG_CTRL0     0x09
#define REG_CTRL1     0x0A
#define REG_CTRL2     0x0B
#define REG_CTRL3     0x0C
#define REG_PRODUCT_ID 0x2F

#define STATUS_MEAS_M_DONE 0x01
#define STATUS_MEAS_T_DONE 0x02

#define CTRL0_TM_M     0x01
#define CTRL0_TM_T     0x02
#define CTRL0_SET      0x08
#define CTRL0_RESET    0x10

#define CTRL1_BW_100HZ 0x00
#define CTRL1_BW_200HZ 0x04
#define CTRL1_BW_400HZ 0x08
#define CTRL1_BW_800HZ 0x0C
#define CTRL1_SW_RST   0x80

#define PRODUCT_ID_VALUE 0x30

class Class_MCC5983
{
public:
    void Init(I2C_HandleTypeDef *hi2c);
    void TIM_Calculate_PeriodElapsedCallback();

    inline float GetFieldX_mG() { return FieldX_mG; }
    inline float GetFieldY_mG() { return FieldY_mG; }
    inline float GetFieldZ_mG() { return FieldZ_mG; }
    inline float GetTemperature_C() { return Temperature_C; }

protected:
    HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t data);
    HAL_StatusTypeDef ReadReg(uint8_t reg, uint8_t *buf, uint16_t len);

    void ReadMeasurement();

    I2C_HandleTypeDef *hi2c;

    uint8_t state;

    uint32_t RawX, RawY, RawZ;
    uint32_t RawX_Set, RawY_Set, RawZ_Set;
    uint32_t RawX_Rst, RawY_Rst, RawZ_Rst;
    uint8_t RawT;

    float FieldX_mG;
    float FieldY_mG;
    float FieldZ_mG;
    float Temperature_C;
};

#endif // DVC_MCC5983_H
