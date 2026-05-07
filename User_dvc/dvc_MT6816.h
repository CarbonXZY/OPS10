#ifndef __DVC_MT6816_H
#define __DVC_MT6816_H

#include "stm32h5xx_hal.h"
#include "drv_spi.h"

// MT6816 寄存器地址定义
#define MT6816_REG_ANGLE_HIGH 0x03 // 角度数据高8位寄存器
#define MT6816_REG_ANGLE_LOW 0x04  // 角度数据低6位寄存器
#define MT6816_REG_STATUS 0x05     // 状态寄存器

// MT6816 状态标志位
#define MT6816_STATUS_NO_MAG_WARNING (1 << 1) // 弱磁报警位 (寄存器0x04 bit1)
#define MT6816_STATUS_OVER_SPEED (1 << 3)     // 超速报警位 (寄存器0x05 bit3)

// SPI 通信相关定义
#define MT6816_SPI_TIMEOUT_MS 1000       // SPI通信超时时间(ms)
#define MT6816_ANGLE_RESOLUTION 16384.0f // 14位角度分辨率

// 角度数据结构
typedef struct
{
    float angle_deg;        // 角度值(度)
    float angle_rad;        // 角度值(弧度)
    uint16_t raw_angle;     // 原始14位角度值
    uint8_t status;         // 状态标志
    uint8_t no_mag_warning; // 弱磁报警标志
    uint8_t over_speed;     // 超速报警标志
    uint8_t parity_error;   // 奇偶校验错误标志
} MT6816_AngleData;

class Class_MT6816
{
public:
    enum
    {
        IDLE,
        BUSY,
        ERROR
    } status;   // 状态机让dma不会影响时序

    // 初始化函数
    void Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_gpio_port, uint16_t cs_gpio_pin);

    // 基本读写函数
    uint8_t ReadRegister(uint8_t reg_addr, uint8_t *data);
    uint8_t WriteRegister(uint8_t reg_addr, uint8_t data);

    // 角度读取函数
    uint8_t ReadAngle(void);
    float GetAngleDeg(void);
    float GetAngleRad(void);

    // 状态检查函数
    uint8_t CheckStatus(void);
    uint8_t IsMagneticFieldOK(void);
    uint8_t IsSpeedOK(void);

    // 工具函数
    float ConvertRawToDeg(uint16_t raw_angle);
    float ConvertRawToRad(uint16_t raw_angle);

    inline MT6816_AngleData *Get_AngleDataPtr(void) { return &angle_data; }

private:
    Struct_SPI_Manage_Object *SPI_Manage_Object;

    // SPI_HandleTypeDef *hspi;    // SPI句柄
    // GPIO_TypeDef *cs_gpio_port; // 片选GPIO端口
    // uint16_t cs_gpio_pin;       // 片选GPIO引脚

    MT6816_AngleData angle_data;

    // SPI通信缓冲区
    // uint8_t tx_buffer_[3];
    // uint8_t rx_buffer_[3];

    // 内部函数
    uint8_t SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
};

#endif