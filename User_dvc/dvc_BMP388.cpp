#include "dvc_BMP388.h"

void Class_BMP388::Init(I2C_HandleTypeDef *hi2c)
{
    this->hi2c = hi2c;
    Read_Calibration();

    uint8_t id;

    if (HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR, 0x00, 1, &id, 1, 10) != HAL_OK)
    {
        return;
    }
    if (id != 0x50)
    {
        return;
    }

    Write_Register(0x7E, 0xB6); // 复位
    HAL_Delay(20);

    Read_Calibration();

    Write_Register(0x1C, 0x12);      // OSR: Px4, Tx1
    Write_Register(0x1F, 0x02 << 1); // IIR: 3
    Write_Register(0x1D, 0x02);      // ODR: 50Hz
    Write_Register(0x1B, 0x33);      // Normal Mode, Enable P & T
}

void Class_BMP388::TIM_Calculate_PeriodElapsedCallback()
{
    uint8_t raw[6];
    if (Read_Register(0x04, raw, 6) != HAL_OK)
    {
        return;
    }

    raw_pressure = (uint32_t)raw[2] << 16 | (uint32_t)raw[1] << 8 | raw[0];
    raw_temperature = (uint32_t)raw[5] << 16 | (uint32_t)raw[4] << 8 | raw[3];

    Compensate_Temperature();
    Compensate_Pressure();
}

HAL_StatusTypeDef Class_BMP388::Write_Register(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, BMP388_I2C_ADDR, reg, 1, &data, 1, 10);
}

HAL_StatusTypeDef Class_BMP388::Read_Register(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR, reg, 1, buf, len, 10);
}

void Class_BMP388::Read_Calibration()
{
    uint8_t cali_data[21];
    Read_Register(0x31, cali_data, 21);

    // 温度系数
    Calibration_Data.t1 = (double)((uint16_t)cali_data[1] << 8 | cali_data[0]) / 0.00390625;
    Calibration_Data.t2 = (double)((uint16_t)cali_data[3] << 8 | cali_data[2]) / 1073741824.0;
    Calibration_Data.t3 = (double)((int8_t)cali_data[4]) / 281474976710656.0;

    // 压力系数 (注意 p1, p2 的偏移量 2^14)
    Calibration_Data.p1 = ((double)((int16_t)((uint16_t)cali_data[6] << 8 | cali_data[5])) - 16384.0) / 1048576.0;
    Calibration_Data.p2 = ((double)((int16_t)((uint16_t)cali_data[8] << 8 | cali_data[7])) - 16384.0) / 536870912.0;
    Calibration_Data.p3 = (double)((int8_t)cali_data[9]) / 4294967296.0;
    Calibration_Data.p4 = (double)((int8_t)cali_data[10]) / 137438953472.0;
    Calibration_Data.p5 = (double)((uint16_t)cali_data[12] << 8 | cali_data[11]) / 0.125;
    Calibration_Data.p6 = (double)((uint16_t)cali_data[14] << 8 | cali_data[13]) / 65536.0;
    Calibration_Data.p7 = (double)((int8_t)cali_data[15]) / 256.0;
    Calibration_Data.p8 = (double)((int8_t)cali_data[16]) / 32768.0;
    Calibration_Data.p9 = (double)((int16_t)((uint16_t)cali_data[18] << 8 | cali_data[17])) / 281474976710656.0;
    Calibration_Data.p10 = (double)((int8_t)cali_data[19]) / 281474976710656.0;
    Calibration_Data.p11 = (double)((int8_t)cali_data[20]) / 36893488147419103232.0;
}

void Class_BMP388::Compensate_Temperature()
{
    double partial_data1 = (double)raw_temperature - Calibration_Data.t1;
    double partial_data2 = partial_data1 * Calibration_Data.t2;

    Temperature = partial_data2 + (partial_data1 * partial_data1) * Calibration_Data.t3;
}

void Class_BMP388::Compensate_Pressure()
{
    double partial_data1, partial_data2, partial_data3, partial_data4, out1, out2;

    partial_data1 = Calibration_Data.p6 * Temperature;
    partial_data2 = Calibration_Data.p7 * (Temperature * Temperature);
    partial_data3 = Calibration_Data.p8 * (Temperature * Temperature * Temperature);
    out1 = Calibration_Data.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = Calibration_Data.p2 * Temperature;
    partial_data2 = Calibration_Data.p3 * (Temperature * Temperature);
    partial_data3 = Calibration_Data.p4 * (Temperature * Temperature * Temperature);
    out2 = (double)raw_pressure * (Calibration_Data.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (double)raw_pressure * (double)raw_pressure;
    partial_data2 = Calibration_Data.p9 + Calibration_Data.p10 * Temperature;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((double)raw_pressure * (double)raw_pressure * (double)raw_pressure) * Calibration_Data.p11;

    Pressure = out1 + out2 + partial_data4;
}
