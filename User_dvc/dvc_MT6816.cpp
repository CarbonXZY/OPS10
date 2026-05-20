/**
 * @file dvc_MT6816.cpp
 * @brief MT6816 磁性角度编码器驱动实现
 *
 * 基于 MT6816 数据手册 v2.0 实现的基本读写函数
 * 支持 4 线 SPI 通信，模式 3 (CPOL=1, CPHA=1)
 */

#include "dvc_MT6816.h"
#include "tsk_config_and_callback.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846f

// 跨平台用
#define Write_Pin HAL_GPIO_WritePin

static HAL_StatusTypeDef SPI_Transmit_Receive(SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    if (hspi == nullptr || tx_data == nullptr || rx_data == nullptr || size == 0)
    {
        return HAL_ERROR; // 参数检查
    }

    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(SPI1_Manage_Object.Now_GPIOx, SPI1_Manage_Object.Now_GPIO_Pin, GPIO_PIN_RESET); // 片选拉低
        HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size, 100);
        HAL_GPIO_WritePin(SPI1_Manage_Object.Now_GPIOx, SPI1_Manage_Object.Now_GPIO_Pin, GPIO_PIN_SET); // 片选拉高
    }
    else if (hspi->Instance == SPI3)
    {
        HAL_GPIO_WritePin(SPI3_Manage_Object.Now_GPIOx, SPI3_Manage_Object.Now_GPIO_Pin, GPIO_PIN_RESET); // 片选拉低
        HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size, 100);
        HAL_GPIO_WritePin(SPI3_Manage_Object.Now_GPIOx, SPI3_Manage_Object.Now_GPIO_Pin, GPIO_PIN_SET); // 片选拉高
    }
    return HAL_OK;
}

/**
 * @brief 初始化 MT6816 传感器
 *
 * @param hspi SPI句柄
 * @param cs_gpio_port 片选GPIO端口
 * @param cs_gpio_pin 片选GPIO引脚
 */
void Class_MT6816::Init(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_gpio_port, uint16_t _cs_gpio_pin)
{
    if (_hspi->Instance == SPI1)
    {
        SPI_Manage_Object = &SPI1_Manage_Object;
    }
    else if (_hspi->Instance == SPI3)
    {
        SPI_Manage_Object = &SPI3_Manage_Object;
    }

    SPI_Manage_Object->SPI_Handler = _hspi;
    SPI_Manage_Object->Now_GPIOx = _cs_gpio_port;
    SPI_Manage_Object->Now_GPIO_Pin = _cs_gpio_pin;

    Write_Pin(_cs_gpio_port, _cs_gpio_pin, GPIO_PIN_SET);

    // 等待传感器上电稳定（数据手册要求至少16ms）
    HAL_Delay(20);
}

/**
 * @brief SPI数据传输函数
 *
 * @param tx_data 发送数据缓冲区
 * @param rx_data 接收数据缓冲区
 * @param size 数据大小
 * @return uint8_t HAL状态
 */
uint8_t Class_MT6816::SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    return SPI_Transmit_Receive(SPI_Manage_Object->SPI_Handler, tx_data, rx_data, size);
}

/**
 * @brief 读取MT6816寄存器
 *
 * @param reg_addr 寄存器地址 (0x00-0x7F)
 * @param data 读取到的数据
 * @return uint8_t 0:成功, 1:失败
 */
uint8_t Class_MT6816::ReadRegister(uint8_t reg_addr, uint8_t *data)
{
    uint8_t status = 1;

    // 构造读取命令：bit0=1(读), bit1-7=地址
    uint8_t read_cmd = 0x80 | (reg_addr & 0x7F);

    // 准备发送数据
    SPI_Manage_Object->Tx_Buffer[0] = read_cmd; // 发送读取命令
    SPI_Manage_Object->Tx_Buffer[1] = 0x00;     // 用于接收数据

    // 发送读取命令并接收数据
    if (SPI_Transfer(SPI_Manage_Object->Tx_Buffer, SPI_Manage_Object->Rx_Buffer, 2) == HAL_OK)
    {
        *data = SPI_Manage_Object->Rx_Buffer[1]; // 第二个字节是读取的数据
        status = 0;
    }

    return status;
}

/**
 * @brief 写入MT6816寄存器
 *
 * @param reg_addr 寄存器地址 (0x00-0x7F)
 * @param data 要写入的数据
 * @return uint8_t 0:成功, 1:失败
 */
uint8_t Class_MT6816::WriteRegister(uint8_t reg_addr, uint8_t data)
{
    uint8_t status = 1;

    // 构造写入命令：bit0=0(写), bit1-7=地址
    uint8_t write_cmd = reg_addr & 0x7F;

    // 准备发送数据
    SPI_Manage_Object->Tx_Buffer[0] = write_cmd; // 发送写入命令
    SPI_Manage_Object->Tx_Buffer[1] = data;      // 发送要写入的数据

    // 发送写入命令和数据
    if (SPI_Transfer(SPI_Manage_Object->Tx_Buffer, SPI_Manage_Object->Rx_Buffer, 2) == HAL_OK)
    {
        status = 0;
    }
    return status;
}

/**
 * @brief 读取角度数据（完整14位）
 *
 * @return uint8_t 0:成功, 1:失败
 */
uint8_t Class_MT6816::ReadAngle(void)
{
    uint8_t status = 1;
    uint8_t angle_high = 0;
    uint8_t angle_low = 0;
    uint8_t status_reg = 0;

    // 读取角度高8位寄存器 (0x03)
    if (ReadRegister(MT6816_REG_ANGLE_HIGH, &angle_high) != 0)
    {
        return 1;
    }

    // 读取角度低6位和状态寄存器 (0x04)
    if (ReadRegister(MT6816_REG_ANGLE_LOW, &angle_low) != 0)
    {
        return 1;
    }

    // 读取状态寄存器 (0x05)
    if (ReadRegister(MT6816_REG_STATUS, &status_reg) != 0)
    {
        return 1;
    }

    // 解析14位角度值
    // 角度高8位: angle_high[7:0] = Angle<13:6>
    // 角度低6位: angle_low[7:2] = Angle<5:0>
    uint16_t raw_angle = ((uint16_t)angle_high << 6) | ((angle_low >> 2) & 0x3F);

    // 解析状态标志
    uint8_t no_mag_warning = (angle_low >> 1) & 0x01; // 弱磁报警位
    uint8_t parity_bit = angle_low & 0x01;            // 奇偶校验位
    uint8_t over_speed = (status_reg >> 3) & 0x01;    // 超速报警位

    // 计算奇偶校验（偶校验）
    uint8_t data_for_parity = angle_high ^ ((angle_low >> 2) & 0x3F);
    uint8_t parity_calc = 0;

    // 计算数据中的1的个数
    for (int i = 0; i < 8; i++)
    {
        if (data_for_parity & (1 << i))
        {
            parity_calc ^= 1;
        }
    }

    // 检查奇偶校验
    uint8_t parity_error = (parity_calc != parity_bit) ? 1 : 0;

    // 填充角度数据结构
    angle_data.raw_angle = raw_angle;
    angle_data.angle_deg = ConvertRawToDeg(raw_angle);
    angle_data.angle_rad = ConvertRawToRad(raw_angle);
    angle_data.status = (no_mag_warning << 1) | (over_speed << 3);
    angle_data.no_mag_warning = no_mag_warning;
    angle_data.over_speed = over_speed;
    angle_data.parity_error = parity_error;

    status = 0;

    return status;
}

/**
 * @brief 获取角度值（度）
 *
 * @return float 角度值(度), 0-360
 */
float Class_MT6816::GetAngleDeg(void)
{
    if (ReadAngle() == 0)
    {
        return angle_data.angle_deg;
    }

    return -1.0f; // 读取失败
}

/**
 * @brief 获取角度值（弧度）
 *
 * @return float 角度值(弧度), 0-2π
 */
float Class_MT6816::GetAngleRad(void)
{
    if (ReadAngle() == 0)
    {
        return angle_data.angle_rad;
    }

    return -1.0f; // 读取失败
}

/**
 * @brief 检查传感器状态
 *
 * @return uint8_t 状态字节
 */
uint8_t Class_MT6816::CheckStatus(void)
{
    if (ReadAngle() == 0)
    {
        return angle_data.status;
    }

    return 0xFF; // 读取失败
}

/**
 * @brief 检查磁场是否正常
 *
 * @return uint8_t 0:磁场正常, 1:弱磁报警
 */
uint8_t Class_MT6816::IsMagneticFieldOK(void)
{
    if (ReadAngle() == 0)
    {
        return (angle_data.no_mag_warning == 0) ? 0 : 1;
    }

    return 1; // 读取失败，假设磁场异常
}

/**
 * @brief 检查速度是否正常
 *
 * @return uint8_t 0:速度正常, 1:超速报警
 */
uint8_t Class_MT6816::IsSpeedOK(void)
{
    if (ReadAngle() == 0)
    {
        return (angle_data.over_speed == 0) ? 0 : 1;
    }

    return 1; // 读取失败，假设速度异常
}

/**
 * @brief 将原始角度值转换为度
 *
 * @param raw_angle 原始14位角度值
 * @return float 角度值(度)
 */
float Class_MT6816::ConvertRawToDeg(uint16_t raw_angle)
{
    return (raw_angle * 360.0f) / MT6816_ANGLE_RESOLUTION;
}

/**
 * @brief 将原始角度值转换为弧度
 *
 * @param raw_angle 原始14位角度值
 * @return float 角度值(弧度)
 */
float Class_MT6816::ConvertRawToRad(uint16_t raw_angle)
{
    return (raw_angle * 2.0f * M_PI) / MT6816_ANGLE_RESOLUTION;
}
