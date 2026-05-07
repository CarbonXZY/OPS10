#include "dvc_MCC5983.h"

void Class_MCC5983::Init(I2C_HandleTypeDef *hi2c)
{
    this->hi2c = hi2c;
    state = 0;

    uint8_t id;
    if (ReadReg(REG_PRODUCT_ID, &id, 1) != HAL_OK)
    {
        return;
    }
    if (id != PRODUCT_ID_VALUE)
    {
        return;
    }

    WriteReg(REG_CTRL0, CTRL0_SET);
    HAL_Delay(1);
    WriteReg(REG_CTRL0, CTRL0_RESET);
    HAL_Delay(1);

    WriteReg(REG_CTRL1, CTRL1_BW_200HZ);
}

void Class_MCC5983::TIM_Calculate_PeriodElapsedCallback()
{
    switch (state)
    {
    case 0:
        WriteReg(REG_CTRL0, CTRL0_SET);
        state = 1;
        break;
    case 1:
        WriteReg(REG_CTRL0, CTRL0_TM_M);
        state = 2;
        break;
    case 2:
        ReadMeasurement();
        RawX_Set = RawX;
        RawY_Set = RawY;
        RawZ_Set = RawZ;
        state = 3;
        break;
    case 3:
        WriteReg(REG_CTRL0, CTRL0_RESET);
        state = 4;
        break;
    case 4:
        WriteReg(REG_CTRL0, CTRL0_TM_M);
        state = 5;
        break;
    case 5:
        ReadMeasurement();
        RawX_Rst = RawX;
        RawY_Rst = RawY;
        RawZ_Rst = RawZ;
        FieldX_mG = (float)((int32_t)RawX_Set - (int32_t)RawX_Rst) / 32.768f;
        FieldY_mG = (float)((int32_t)RawY_Set - (int32_t)RawY_Rst) / 32.768f;
        FieldZ_mG = (float)((int32_t)RawZ_Set - (int32_t)RawZ_Rst) / 32.768f;
        state = 0;
        break;
    }
}

void Class_MCC5983::ReadMeasurement()
{
    uint8_t status;
    for (uint8_t i = 0; i < 20; i++)
    {
        if (ReadReg(REG_STATUS, &status, 1) != HAL_OK)
        {
            return;
        }
        if (status & STATUS_MEAS_M_DONE)
        {
            break;
        }
        HAL_Delay(1);
    }

    uint8_t buf[8];
    if (ReadReg(REG_XOUT0, buf, 8) != HAL_OK)
    {
        return;
    }

    RawX = (uint32_t)buf[0] << 10 | (uint32_t)buf[1] << 2 | (buf[6] >> 6);
    RawY = (uint32_t)buf[2] << 10 | (uint32_t)buf[3] << 2 | ((buf[6] >> 4) & 0x03);
    RawZ = (uint32_t)buf[4] << 10 | (uint32_t)buf[5] << 2 | ((buf[6] >> 2) & 0x03);

    RawT = buf[7];
    Temperature_C = (float)buf[7] * 0.8f - 75.0f;
}

HAL_StatusTypeDef Class_MCC5983::WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, MCC5983_I2C_ADDR, reg, 1, &data, 1, 10);
}

HAL_StatusTypeDef Class_MCC5983::ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MCC5983_I2C_ADDR, reg, 1, buf, len, 10);
}
