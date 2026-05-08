/**
 ******************************************************************************
 * @file    MMC5983MA.h
 * @brief   MMC5983MA 3-axis Magnetic Sensor Driver
 * @version 1.0
 * @date    2026-05-07
 ******************************************************************************
 */

#ifndef __MMC5983MA_H
#define __MMC5983MA_H

/* 包含头文件 ---------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"

/* 宏定义 -------------------------------------------------------------------*/
// I2C 设备地址（7位地址 0x30，左移1位变为8位写地址）
#define MMC5983MA_I2C_ADDR_WRITE (0x30 << 1)         // 0x60
#define MMC5983MA_I2C_ADDR_READ ((0x30 << 1) | 0x01) // 0x61

// 寄存器地址定义
#define MMC5983MA_REG_XOUT0 0x00      // X轴数据高位 [17:10]
#define MMC5983MA_REG_XOUT1 0x01      // X轴数据中位 [9:2]
#define MMC5983MA_REG_YOUT0 0x02      // Y轴数据高位 [17:10]
#define MMC5983MA_REG_YOUT1 0x03      // Y轴数据中位 [9:2]
#define MMC5983MA_REG_ZOUT0 0x04      // Z轴数据高位 [17:10]
#define MMC5983MA_REG_ZOUT1 0x05      // Z轴数据中位 [9:2]
#define MMC5983MA_REG_XYZOUT2 0x06    // X/Y/Z 低2位（18位模式）
#define MMC5983MA_REG_TOUT 0x07       // 温度输出
#define MMC5983MA_REG_STATUS 0x08     // 状态寄存器
#define MMC5983MA_REG_CONTROL0 0x09   // 控制寄存器0
#define MMC5983MA_REG_CONTROL1 0x0A   // 控制寄存器1
#define MMC5983MA_REG_CONTROL2 0x0B   // 控制寄存器2（连续模式配置）
#define MMC5983MA_REG_CONTROL3 0x0C   // 控制寄存器3（SPI 3线模式等）
#define MMC5983MA_REG_PRODUCT_ID 0x2F // 产品ID（固定 0x30）

// CONTROL0 寄存器位定义
#define MMC5983MA_TM_M 0x01       // 触发磁场测量
#define MMC5983MA_TM_T 0x02       // 触发温度测量
#define MMC5983MA_SET 0x08        // SET 操作
#define MMC5983MA_RESET 0x10      // RESET 操作
#define MMC5983MA_AUTO_SR_EN 0x40 // 自动 SET/RESET 使能
#define MMC5983MA_OTP_READ 0x80   // OTP 读取

// STATUS 寄存器位定义
#define MMC5983MA_STATUS_MEAS_M_DONE 0x01   // 磁场测量完成
#define MMC5983MA_STATUS_MEAS_T_DONE 0x02   // 温度测量完成
#define MMC5983MA_STATUS_OTP_READ_DONE 0x04 // OTP 读取完成

// CONTROL2 寄存器位定义（连续模式）
#define MMC5983MA_CMM_EN 0x80     // 连续模式使能
#define MMC5983MA_EN_PRD_SET 0x40 // 周期性 SET 使能
// CM_Freq[2:0] - 连续模式测量频率（需要配合 CMM_EN）
#define MMC5983MA_CM_FREQ_OFF 0x00    // 关闭连续模式
#define MMC5983MA_CM_FREQ_1HZ 0x20    // 1 Hz
#define MMC5983MA_CM_FREQ_10HZ 0x40   // 10 Hz
#define MMC5983MA_CM_FREQ_20HZ 0x60   // 20 Hz
#define MMC5983MA_CM_FREQ_50HZ 0x80   // 50 Hz
#define MMC5983MA_CM_FREQ_100HZ 0xA0  // 100 Hz
#define MMC5983MA_CM_FREQ_200HZ 0xC0  // 200 Hz（需 BW=01）
#define MMC5983MA_CM_FREQ_1000HZ 0xE0 // 1000 Hz（需 BW=11）
// Prd_set[2:0] - 周期性 SET 间隔（需配合 EN_PRD_SET）
#define MMC5983MA_PRD_SET_1 0x00    // 每 1 次测量后 SET
#define MMC5983MA_PRD_SET_25 0x01   // 每 25 次
#define MMC5983MA_PRD_SET_75 0x02   // 每 75 次
#define MMC5983MA_PRD_SET_100 0x03  // 每 100 次
#define MMC5983MA_PRD_SET_250 0x04  // 每 250 次
#define MMC5983MA_PRD_SET_500 0x05  // 每 500 次
#define MMC5983MA_PRD_SET_1000 0x06 // 每 1000 次
#define MMC5983MA_PRD_SET_2000 0x07 // 每 2000 次

// 带宽配置（CONTROL1 寄存器）
#define MMC5983MA_BW_00 0x00 // 50Hz ODR, 0.4mG 噪声
#define MMC5983MA_BW_01 0x01 // 100Hz ODR, 0.6mG 噪声
#define MMC5983MA_BW_10 0x10 // 225Hz ODR, 0.8mG 噪声
#define MMC5983MA_BW_11 0x11 // 580Hz ODR, 1.2mG 噪声

// 转换系数
#define MMC5983MA_LSB_PER_G_16BIT 4096.0f    // 16位模式 4096 counts/G
#define MMC5983MA_LSB_PER_G_18BIT 16384.0f   // 18位模式 16384 counts/G
#define MMC5983MA_MG_PER_LSB_16BIT 0.25f     // 0.25 mG/LSB
#define MMC5983MA_G_PER_LSB_16BIT 0.00025f   // 0.00025 G/LSB
#define MMC5983MA_G_PER_LSB_18BIT 0.0000625f // 0.0000625 G/LSB

// 温度转换系数
#define MMC5983MA_TEMP_LSB_PER_C 0.8f // 0.8°C/LSB
#define MMC5983MA_TEMP_OFFSET -75.0f  // 0x00 = -75°C

// 超时定义（毫秒）
#define MMC5983MA_MEASURE_TIMEOUT 1000
#define MMC5983MA_SET_RESET_DELAY 10

/* 枚举类型定义 -------------------------------------------------------------*/
typedef enum
{
    MMC5983MA_OK = 0,
    MMC5983MA_ERROR = 1
} MMC5983MA_Status_t;

typedef enum
{
    MMC5983MA_MODE_16BIT = 0,
    MMC5983MA_MODE_18BIT = 1
} MMC5983MA_Resolution_t;

/* 数据结构定义 -------------------------------------------------------------*/
typedef struct
{
    int16_t x;     // X轴原始数据（16位模式）
    int16_t y;     // Y轴原始数据（16位模式）
    int16_t z;     // Z轴原始数据（16位模式）
    float x_gauss; // X轴磁场强度（Gauss）
    float y_gauss; // Y轴磁场强度（Gauss）
    float z_gauss; // Z轴磁场强度（Gauss）
} MMC5983MA_MagnetData_t;

typedef struct
{
    uint8_t raw;   // 原始温度数据
    float celsius; // 温度（摄氏度）
} MMC5983MA_TempData_t;

/* 函数声明 -----------------------------------------------------------------*/

class Class_MMC5983
{
public:
    /**
     * @brief 初始化 MMC5983MA 传感器
     * @param hi2c I2C 句柄指针
     * @return MMC5983MA_Status_t 初始化状态
     */
    MMC5983MA_Status_t Init(I2C_HandleTypeDef *hi2c);

    /**
     * @brief 执行 SET 操作（恢复传感器磁特性）
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t DoSet();

    /**
     * @brief 执行 RESET 操作
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t DoReset();

    /**
     * @brief 触发磁场测量并读取数据（16位模式，轮询方式）
     * @param data 存储磁场数据的结构体指针
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadMagnet();

    /**
     * @brief 触发磁场测量并读取数据（18位模式）
     * @param data 存储磁场数据的结构体指针（x,y,z 为 18位有符号值）
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadMagnet18();

    /**
     * @brief 读取温度数据（触发新测量）
     * @param temp 存储温度数据的结构体指针
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadTemperature();

    /**
     * @brief 读取状态寄存器
     * @param status 存储状态值的指针
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadStatus();

    /**
     * @brief 检查磁场测量是否完成
     * @return 1=完成, 0=未完成
     */
    uint8_t IsMeasMDone(void);

    /**
     * @brief 检查温度测量是否完成
     * @return 1=完成, 0=未完成
     */
    uint8_t IsMeasTDone(void);

    /**
     * @brief 清除测量完成标志
     */
    void ClearMeasMDone(void);

    /**
     * @brief 配置连续测量模式
     * @param en 1=使能, 0=关闭
     * @param freq 频率值（使用 MMC5983MA_CM_FREQ_xxx 宏）
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ConfigContinuousMode(uint8_t en, uint8_t freq);

    /**
     * @brief 配置带宽
     * @param bw 带宽值（使用 MMC5983MA_BW_xx 宏）
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t SetBandwidth(uint8_t bw);

    /**
     * @brief 读取产品ID
     * @param id 存储ID的指针（应为 0x30）
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadProductID(uint8_t *id);

    /**
     * @brief 使用 SET/RESET 方法读取磁场（消除了 Offset 误差）
     * @param data 存储磁场数据的结构体指针
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t ReadMagnetWithOffsetCancellation();

    /**
     * @brief 软复位传感器
     * @return MMC5983MA_Status_t
     */
    MMC5983MA_Status_t SoftReset(void);

private:
    I2C_HandleTypeDef *hi2c; // I2C 句柄

    MMC5983MA_MagnetData_t magnet_data; // 存储磁场数据的结构体
    MMC5983MA_TempData_t temp_data;     // 存储温度数据的结构体

    uint8_t status; // 存储状态寄存器值

    HAL_StatusTypeDef I2C_WriteReg(uint8_t reg, uint8_t data);
    HAL_StatusTypeDef I2C_ReadReg(uint8_t reg, uint8_t *data);
    HAL_StatusTypeDef I2C_ReadBytes(uint8_t reg, uint8_t *buffer, uint8_t len);
};

#endif /* __MMC5983MA_H */