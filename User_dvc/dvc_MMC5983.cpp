/**
 ******************************************************************************
 * @file    MMC5983MA.c
 * @brief   MMC5983MA 3-axis Magnetic Sensor Driver - Implementation
 * @version 1.0
 * @date    2026-05-07
 ******************************************************************************
 */

#include "dvc_MMC5983.h"
#include "dvc_dwt.h"

/* 私有变量 -----------------------------------------------------------------*/

/* 私有函数声明 -------------------------------------------------------------*/

/* 私有函数实现 -------------------------------------------------------------*/

/**
 * @brief I2C 写单字节到寄存器
 */
HAL_StatusTypeDef Class_MMC5983::I2C_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, MMC5983MA_I2C_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief I2C 读单字节从寄存器
 */
HAL_StatusTypeDef Class_MMC5983::I2C_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(hi2c, MMC5983MA_I2C_ADDR_WRITE, reg,
                            I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
 * @brief I2C 连续读多个字节
 */
HAL_StatusTypeDef Class_MMC5983::I2C_ReadBytes(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MMC5983MA_I2C_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, HAL_MAX_DELAY);
}

/* 公共函数实现 -------------------------------------------------------------*/

/**
 * @brief 初始化 MMC5983MA 传感器
 * @param hi2c I2C 句柄指针
 * @return MMC5983MA_Status_t 初始化状态
 */
MMC5983MA_Status_t Class_MMC5983::Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t product_id = 0;
    uint32_t start_tick;

    if (hi2c == NULL)
    {
        return MMC5983MA_ERROR;
    }

    this->hi2c = hi2c;

    // 读取产品 ID 验证通信
    if (ReadProductID(&product_id) != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 检查产品 ID 是否为 0x30
    if (product_id != 0x30)
    {
        return MMC5983MA_ERROR;
    }

    // 执行 SET 操作，恢复传感器特性
    if (DoSet() != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 等待 SET 完成
    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    // 设置默认带宽 BW=00（50Hz ODR，最低噪声）
    if (SetBandwidth(MMC5983MA_BW_00) != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }

    return MMC5983MA_OK;
}

/**
 * @brief 执行 SET 操作
 */
MMC5983MA_Status_t Class_MMC5983::DoSet(void)
{
    HAL_StatusTypeDef ret;

    ret = I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_SET);
    if (ret != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    return MMC5983MA_OK;
}

/**
 * @brief 执行 RESET 操作
 */
MMC5983MA_Status_t Class_MMC5983::DoReset(void)
{
    HAL_StatusTypeDef ret;

    ret = I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_RESET);
    if (ret != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    return MMC5983MA_OK;
}

/**
 * @brief 触发磁场测量并读取数据（16位模式）
 */
MMC5983MA_Status_t Class_MMC5983::ReadMagnet()
{
    uint8_t status = 0;
    uint8_t raw[6] = {0};
    uint32_t start_tick;
    int16_t x_raw, y_raw, z_raw;

    // 1. 触发磁场测量（TM_M = 1）
    if (I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_TM_M) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 2. 用 DWT 轮询等待测量完成（不依赖 systick，在定时中断回调里安全）
    uint32_t tick_start = DWT_GetCurrentTimeUs();
    do {
        if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
        {
            return MMC5983MA_ERROR;
        }
        if (status & MMC5983MA_STATUS_MEAS_M_DONE)
            break;
    } while ((DWT_GetCurrentTimeUs() - tick_start) < MMC5983MA_MEASURE_TIMEOUT);

    // 3. 读取 X, Y, Z 数据（从 0x00 开始连续读 6 字节）
    if (I2C_ReadBytes(MMC5983MA_REG_XOUT0, raw, 6) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 4. 组合成 16 位整数
    x_raw = (int16_t)((raw[0] << 8) | raw[1]);
    y_raw = (int16_t)((raw[2] << 8) | raw[3]);
    z_raw = (int16_t)((raw[4] << 8) | raw[5]);

    // 5. 填充结构体
    magnet_data.x = x_raw;
    magnet_data.y = y_raw;
    magnet_data.z = z_raw;
    magnet_data.x_gauss = (float)x_raw * MMC5983MA_G_PER_LSB_16BIT;
    magnet_data.y_gauss = (float)y_raw * MMC5983MA_G_PER_LSB_16BIT;
    magnet_data.z_gauss = (float)z_raw * MMC5983MA_G_PER_LSB_16BIT;

    return MMC5983MA_OK;
}

/**
 * @brief 触发磁场测量并读取数据（18位模式）
 */
MMC5983MA_Status_t Class_MMC5983::ReadMagnet18()
{
    uint8_t status = 0;
    uint8_t raw[6] = {0};
    uint8_t low = 0;
    uint32_t start_tick;

    // 1. 触发磁场测量
    if (I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_TM_M) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    uint32_t tick_start = DWT_GetCurrentTimeUs();
    do {
        if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
        {
            return MMC5983MA_ERROR;
        }
        if (status & MMC5983MA_STATUS_MEAS_M_DONE)
            break;
    } while ((DWT_GetCurrentTimeUs() - tick_start) < MMC5983MA_MEASURE_TIMEOUT);

    // 3. 读取 X, Y, Z 数据（6 + 1 字节）
    if (I2C_ReadBytes(MMC5983MA_REG_XOUT0, raw, 6) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    if (I2C_ReadReg(MMC5983MA_REG_XYZOUT2, &low) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 4. 组合成 18 位整数
    magnet_data.x = ((int32_t)raw[0] << 10) | ((int32_t)raw[1] << 2) | ((low >> 0) & 0x03);
    magnet_data.y = ((int32_t)raw[2] << 10) | ((int32_t)raw[3] << 2) | ((low >> 2) & 0x03);
    magnet_data.z = ((int32_t)raw[4] << 10) | ((int32_t)raw[5] << 2) | ((low >> 4) & 0x03);

    return MMC5983MA_OK;
}

/**
 * @brief 读取温度数据（触发新测量）
 */
MMC5983MA_Status_t Class_MMC5983::ReadTemperature()
{
    uint8_t status = 0;
    uint8_t temp_raw = 0;
    uint32_t start_tick;

    // 1. 触发温度测量（TM_T = 1）
    if (I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_TM_T) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 2. 用 DWT 轮询等待测量完成（不依赖 systick，在定时中断回调里安全）
    uint32_t tick_start = DWT_GetCurrentTimeUs();
    do {
        if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
        {
            return MMC5983MA_ERROR;
        }
        if (status & MMC5983MA_STATUS_MEAS_T_DONE)
            break;
    } while ((DWT_GetCurrentTimeUs() - tick_start) < MMC5983MA_MEASURE_TIMEOUT);

    // 3. 读取温度数据
    if (I2C_ReadReg(MMC5983MA_REG_TOUT, &temp_raw) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 4. 填充结构体
    temp_data.raw = temp_raw;
    temp_data.celsius = (float)temp_raw * MMC5983MA_TEMP_LSB_PER_C + MMC5983MA_TEMP_OFFSET;

    return MMC5983MA_OK;
}

/**
 * @brief 读取状态寄存器
 */
MMC5983MA_Status_t Class_MMC5983::ReadStatus()
{
    if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    return MMC5983MA_OK;
}

/**
 * @brief 检查磁场测量是否完成
 */
uint8_t Class_MMC5983::IsMeasMDone()
{
    uint8_t status = 0;

    if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
    {
        return 0;
    }

    return (status & MMC5983MA_STATUS_MEAS_M_DONE) ? 1 : 0;
}

/**
 * @brief 检查温度测量是否完成
 */
uint8_t Class_MMC5983::IsMeasTDone()
{
    uint8_t status = 0;

    if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != HAL_OK)
    {
        return 0;
    }

    return (status & MMC5983MA_STATUS_MEAS_T_DONE) ? 1 : 0;
}

/**
 * @brief 清除测量完成标志（写入1清除）
 */
void Class_MMC5983::ClearMeasMDone(void)
{
    uint8_t status = 0;

    // 读取当前状态
    if (I2C_ReadReg(MMC5983MA_REG_STATUS, &status) == HAL_OK)
    {
        // 写入1清除对应位
        if (status & MMC5983MA_STATUS_MEAS_M_DONE)
        {
            I2C_WriteReg(MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_M_DONE);
        }
    }
}

/**
 * @brief 配置连续测量模式
 */
MMC5983MA_Status_t Class_MMC5983::ConfigContinuousMode(uint8_t en, uint8_t freq)
{
    uint8_t reg_val = 0;

    if (en)
    {
        reg_val = MMC5983MA_CMM_EN | (freq & 0xE0);
    }
    else
    {
        reg_val = 0x00;
    }

    if (I2C_WriteReg(MMC5983MA_REG_CONTROL2, reg_val) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    return MMC5983MA_OK;
}

/**
 * @brief 配置带宽
 */
MMC5983MA_Status_t Class_MMC5983::SetBandwidth(uint8_t bw)
{
    // 带宽配置在 CONTROL1 寄存器的 bit0 和 bit4
    // BW[1:0] -> bit4 (BW1), bit0 (BW0)
    uint8_t reg_val = 0;

    if (bw & 0x01)
        reg_val |= 0x01; // BW0
    if (bw & 0x10)
        reg_val |= 0x10; // BW1

    if (I2C_WriteReg(MMC5983MA_REG_CONTROL1, reg_val) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    return MMC5983MA_OK;
}

/**
 * @brief 读取产品ID
 */
MMC5983MA_Status_t Class_MMC5983::ReadProductID(uint8_t *id)
{
    if (id == NULL)
    {
        return MMC5983MA_ERROR;
    }

    if (I2C_ReadReg(MMC5983MA_REG_PRODUCT_ID, id) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    return MMC5983MA_OK;
}

/**
 * @brief 使用 SET/RESET 方法读取磁场（消除 Offset 误差）
 *
 * 原理：
 *   输出1 (SET后)
= +H + Offset
 *   输出2 (RESET后) = -H + Offset
 *   真实 H = (输出1 - 输出2) / 2
 *   Offset = (输出1 + 输出2) / 2
 */
MMC5983MA_Status_t Class_MMC5983::ReadMagnetWithOffsetCancellation()
{
    MMC5983MA_MagnetData_t data_set, data_reset;
    int16_t x_sum, y_sum, z_sum;
    int16_t x_diff, y_diff, z_diff;

    // 1. 执行 SET 操作
    if (DoSet() != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }
    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    // 2. 测量（SET后）
    if (ReadMagnet() != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 3. 执行 RESET 操作
    if (DoReset() != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }
    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    // 4. 测量（RESET后）
    if (ReadMagnet() != MMC5983MA_OK)
    {
        return MMC5983MA_ERROR;
    }

    // 5. 计算真实磁场值（消除 Offset）
    // 注意：RESET 后磁场极性相反，所以用 SET - RESET 再除以 2
    x_diff = (int16_t)(data_set.x - data_reset.x);
    y_diff = (int16_t)(data_set.y - data_reset.y);
    z_diff = (int16_t)(data_set.z - data_reset.z);

    magnet_data.x = x_diff / 2;
    magnet_data.y = y_diff / 2;
    magnet_data.z = z_diff / 2;

    magnet_data.x_gauss = (float)magnet_data.x * MMC5983MA_G_PER_LSB_16BIT;
    magnet_data.y_gauss = (float)magnet_data.y * MMC5983MA_G_PER_LSB_16BIT;
    magnet_data.z_gauss = (float)magnet_data.z * MMC5983MA_G_PER_LSB_16BIT;

    // 可选：计算并存储 Offset（可用于后续快速补偿）
    // x_sum = (data_set.x + data_reset.x) / 2;
    // y_sum = (data_set.y + data_reset.y) / 2;
    // z_sum = (data_set.z + data_reset.z) / 2;

    return MMC5983MA_OK;
}

/**
 * @brief 软复位传感器（通过 OTP_READ 位触发）
 */
MMC5983MA_Status_t Class_MMC5983::SoftReset()
{
    // 向 CONTROL0 的 OTP_READ 位写 1 会触发重新加载校准数据
    if (I2C_WriteReg(MMC5983MA_REG_CONTROL0, MMC5983MA_OTP_READ) != HAL_OK)
    {
        return MMC5983MA_ERROR;
    }

    HAL_Delay(MMC5983MA_SET_RESET_DELAY);

    return MMC5983MA_OK;
}