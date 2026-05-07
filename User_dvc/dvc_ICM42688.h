/**
 * @file ICM42688.h
 * @author finani
 * @version 1.0
 * @date 2026-03-03
 *
 * @see https://github.com/finani/ICM42688
 *
 * 原版是Arduino驱动，作者将接口层改成STM32可用的
 * 目前尚未支持INT引脚中断功能
 * 未测试过，编译可通过
 *
 */

#ifndef ICM42688_H
#define ICM42688_H

#include "stm32h5xx_hal.h"
#include "drv_spi.h"

#define delay 		HAL_Delay

/**
 * @brief ICM-42688-P惯性测量单元驱动类
 *
 *         支持加速度计、陀螺仪和温度数据的读取与校准
 */
class Class_ICM42688
{
public:
    // 状态机定义，与MT6816风格保持一致
    enum
    {
        IDLE,   ///< 空闲状态
        BUSY,   ///< 忙状态
        ERROR   ///< 错误状态
    } status;   ///< 状态机确保DMA不会影响时序

    /// 陀螺仪量程选择
    enum GyroFS : uint8_t
    {
        dps2000 = 0x00,   ///< ±2000 dps
        dps1000 = 0x01,   ///< ±1000 dps
        dps500 = 0x02,    ///< ±500 dps
        dps250 = 0x03,    ///< ±250 dps
        dps125 = 0x04,    ///< ±125 dps
        dps62_5 = 0x05,   ///< ±62.5 dps
        dps31_25 = 0x06,  ///< ±31.25 dps
        dps15_625 = 0x07  ///< ±15.625 dps
    };

    /// 加速度计量程选择
    enum AccelFS : uint8_t
    {
        gpm16 = 0x00,     ///< ±16g
        gpm8 = 0x01,      ///< ±8g
        gpm4 = 0x02,      ///< ±4g
        gpm2 = 0x03       ///< ±2g
    };

    /// 输出数据速率
    enum ODR : uint8_t
    {
        odr32k = 0x01,    ///< 32kHz (仅LN模式)
        odr16k = 0x02,    ///< 16kHz (仅LN模式)
        odr8k = 0x03,     ///< 8kHz (仅LN模式)
        odr4k = 0x04,     ///< 4kHz (仅LN模式)
        odr2k = 0x05,     ///< 2kHz (仅LN模式)
        odr1k = 0x06,     ///< 1kHz (仅LN模式)
        odr200 = 0x07,    ///< 200Hz
        odr100 = 0x08,    ///< 100Hz
        odr50 = 0x09,     ///< 50Hz
        odr25 = 0x0A,     ///< 25Hz
        odr12_5 = 0x0B,   ///< 12.5Hz
        odr6a25 = 0x0C,   ///< 6.25Hz (仅LP模式，仅加速度计)
        odr3a125 = 0x0D,  ///< 3.125Hz (仅LP模式，仅加速度计)
        odr1a5625 = 0x0E, ///< 1.5625Hz (仅LP模式，仅加速度计)
        odr500 = 0x0F,    ///< 500Hz
    };

    /// 陀螺仪陷波滤波器带宽选择
    enum GyroNFBWsel : uint8_t
    {
        nfBW1449Hz = 0x00,///< 1449Hz带宽
        nfBW680z = 0x01,  ///< 680Hz带宽
        nfBW329Hz = 0x02, ///< 329Hz带宽
        nfBW162Hz = 0x03, ///< 162Hz带宽
        nfBW80Hz = 0x04,  ///< 80Hz带宽
        nfBW40Hz = 0x05,  ///< 40Hz带宽
        nfBW20Hz = 0x06,  ///< 20Hz带宽
        nfBW10Hz = 0x07,  ///< 10Hz带宽
    };

    /// UI滤波器阶数
    enum UIFiltOrd : uint8_t
    {
        first_order = 0x00,   ///< 一阶
        second_order = 0x01,  ///< 二阶
        third_order = 0x02,   ///< 三阶
    };

    /**
     * @brief 初始化设备
     *
     * @param hspi SPI句柄指针
     * @return int 成功返回1，失败返回负数错误码
     */
    int Init(SPI_HandleTypeDef *hspi);

    /**
     * @brief 设置加速度计量程
     *
     * @param fssel 满量程选择
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAccelFS(AccelFS fssel);

    /**
     * @brief 获取加速度计量程
     *
     * @return int 当前量程选择值
     */
    int getAccelFS();

    /**
     * @brief 设置陀螺仪量程
     *
     * @param fssel 满量程选择
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyroFS(GyroFS fssel);

    /**
     * @brief 获取陀螺仪量程
     *
     * @return int 当前量程选择值
     */
    int getGyroFS();

    /**
     * @brief 设置加速度计ODR
     *
     * @param odr 输出数据速率
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAccelODR(ODR odr);

    /**
     * @brief 获取加速度计ODR
     *
     * @return int 当前ODR值
     */
    int getAccelODR();

    /**
     * @brief 设置陀螺仪ODR
     *
     * @param odr 输出数据速率
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyroODR(ODR odr);

    /**
     * @brief 获取陀螺仪ODR
     *
     * @return int 当前ODR值
     */
    int getGyroODR();

    /**
     * @brief 设置滤波器使能
     *
     * @param gyroFilters 陀螺仪滤波器使能
     * @param accFilters 加速度计滤波器使能
     * @return int 成功返回1，失败返回负数错误码
     */
    int setFilters(bool gyroFilters, bool accFilters);

    /**
     * @brief 使能数据就绪中断
     *
     *         - 将UI数据就绪中断路由到INT1
     *         - 推挽、脉冲、高电平有效中断
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int enableDataReadyInterrupt();

    /**
     * @brief 禁用数据就绪中断
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int disableDataReadyInterrupt();

    /**
     * @brief 读取ICM-42688-P最新数据并转换为物理量
     *
     *         必须先调用此函数才能访问新的测量数据
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int getAGT();

    /**
     * @brief 读取ICM-42688-P原始数据
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int getRawAGT();

    /**
     * @brief 获取X轴加速度数据
     *
     * @return float 加速度值(g)
     */
    float accX() const { return _acc[0]; }

    /**
     * @brief 获取Y轴加速度数据
     *
     * @return float 加速度值(g)
     */
    float accY() const { return _acc[1]; }

    /**
     * @brief 获取Z轴加速度数据
     *
     * @return float 加速度值(g)
     */
    float accZ() const { return _acc[2]; }

    /**
     * @brief 获取X轴陀螺仪数据
     *
     * @return float 角速度值(dps)
     */
    float gyrX() const { return _gyr[0]; }

    /**
     * @brief 获取Y轴陀螺仪数据
     *
     * @return float 角速度值(dps)
     */
    float gyrY() const { return _gyr[1]; }

    /**
     * @brief 获取Z轴陀螺仪数据
     *
     * @return float 角速度值(dps)
     */
    float gyrZ() const { return _gyr[2]; }

    /**
     * @brief 获取芯片温度
     *
     * @return float 温度值(°C)
     */
    float temp() const { return Temperature; }

    /**
     * @brief 获取X轴加速度原始数据
     *
     * @return int16_t 原始加速度数据
     */
    int16_t rawAccX() const { return _rawAcc[0]; }

    /**
     * @brief 获取Y轴加速度原始数据
     *
     * @return int16_t 原始加速度数据
     */
    int16_t rawAccY() const { return _rawAcc[1]; }

    /**
     * @brief 获取Z轴加速度原始数据
     *
     * @return int16_t 原始加速度数据
     */
    int16_t rawAccZ() const { return _rawAcc[2]; }

    /**
     * @brief 获取X轴陀螺仪原始数据
     *
     * @return int16_t 原始角速度数据
     */
    int16_t rawGyrX() const { return _rawGyr[0]; }

    /**
     * @brief 获取Y轴陀螺仪原始数据
     *
     * @return int16_t 原始角速度数据
     */
    int16_t rawGyrY() const { return _rawGyr[1]; }

    /**
     * @brief 获取Z轴陀螺仪原始数据
     *
     * @return int16_t 原始角速度数据
     */
    int16_t rawGyrZ() const { return _rawGyr[2]; }

    /**
     * @brief 获取温度原始数据
     *
     * @return int16_t 原始温度数据
     */
    int16_t rawTemp() const { return _rawT; }

    /**
     * @brief 获取X轴加速度原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasAccX() const { return _rawAccBias[0]; }

    /**
     * @brief 获取Y轴加速度原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasAccY() const { return _rawAccBias[1]; }

    /**
     * @brief 获取Z轴加速度原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasAccZ() const { return _rawAccBias[2]; }

    /**
     * @brief 获取X轴陀螺仪原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasGyrX() const { return _rawGyrBias[0]; }

    /**
     * @brief 获取Y轴陀螺仪原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasGyrY() const { return _rawGyrBias[1]; }

    /**
     * @brief 获取Z轴陀螺仪原始零偏
     *
     * @return int32_t 原始零偏值
     */
    int32_t rawBiasGyrZ() const { return _rawGyrBias[2]; }

    /**
     * @brief 计算零偏值
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int computeOffsets();

    /**
     * @brief 设置所有计算出的零偏Offset
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAllOffsets();

    /**
     * @brief 单独设置X轴陀螺仪Offset
     *
     * @param gyrXoffset X轴陀螺仪偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyrXOffset(int16_t gyrXoffset);

    /**
     * @brief 单独设置Y轴陀螺仪Offset
     *
     * @param gyrYoffset Y轴陀螺仪偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyrYOffset(int16_t gyrYoffset);

    /**
     * @brief 单独设置Z轴陀螺仪Offset
     *
     * @param gyrZoffset Z轴陀螺仪偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyrZOffset(int16_t gyrZoffset);

    /**
     * @brief 单独设置X轴加速度计Offset
     *
     * @param accXoffset X轴加速度计偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAccXOffset(int16_t accXoffset);

    /**
     * @brief 单独设置Y轴加速度计Offset
     *
     * @param accYoffset Y轴加速度计偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAccYOffset(int16_t accYoffset);

    /**
     * @brief 单独设置Z轴加速度计Offset
     *
     * @param accZoffset Z轴加速度计偏移量
     * @return int 成功返回1，失败返回负数错误码
     */
    int setAccZOffset(int16_t accZoffset);

    /**
     * @brief 获取加速度计分辨率
     *
     * @return float 加速度计量程分辨率
     */
    float getAccelRes();

    /**
     * @brief 获取陀螺仪分辨率
     *
     * @return float 陀螺仪量程分辨率
     */
    float getGyroRes();

    /**
     * @brief 设置UI滤波器组
     *
     * @param gyroUIFiltOrder 陀螺仪UI滤波器阶数
     * @param accelUIFiltOrder 加速度计UI滤波器阶数
     * @return int 成功返回1
     */
    int setUIFilterBlock(UIFiltOrd gyroUIFiltOrder, UIFiltOrd accelUIFiltOrder);

    /**
     * @brief 设置陀螺仪陷波滤波器
     *
     * @param gyroNFfreq_x X轴陷波频率
     * @param gyroNFfreq_y Y轴陷波频率
     * @param gyroNFfreq_z Z轴陷波频率
     * @param gyro_nf_bw 陷波带宽选择
     * @return int 成功返回1，失败返回负数错误码
     */
    int setGyroNotchFilter(float gyroNFfreq_x, float gyroNFfreq_y, float gyroNFfreq_z, GyroNFBWsel gyro_nf_bw);

    /**
     * @brief 自检功能
     *
     * @return int 当前仅返回1
     */
    int selfTest();

    /**
     * @brief 测试函数
     *
     * @return int 始终返回1
     */
    int testingFunction();

    /**
     * @brief 陀螺仪零偏校准
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int calibrateGyro();

    /**
     * @brief 获取X轴陀螺仪零偏
     *
     * @return float X轴零偏值(dps)
     */
    float getGyroBiasX();

    /**
     * @brief 获取Y轴陀螺仪零偏
     *
     * @return float Y轴零偏值(dps)
     */
    float getGyroBiasY();

    /**
     * @brief 获取Z轴陀螺仪零偏
     *
     * @return float Z轴零偏值(dps)
     */
    float getGyroBiasZ();

    /**
     * @brief 设置X轴陀螺仪零偏
     *
     * @param bias 零偏值(dps)
     */
    void setGyroBiasX(float bias);

    /**
     * @brief 设置Y轴陀螺仪零偏
     *
     * @param bias 零偏值(dps)
     */
    void setGyroBiasY(float bias);

    /**
     * @brief 设置Z轴陀螺仪零偏
     *
     * @param bias 零偏值(dps)
     */
    void setGyroBiasZ(float bias);

    /**
     * @brief 加速度计校准
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int calibrateAccel();

    /**
     * @brief 获取X轴加速度计零偏(m/s²)
     *
     * @return float X轴零偏值
     */
    float getAccelBiasX_mss();

    /**
     * @brief 获取X轴加速度计比例因子
     *
     * @return float X轴比例因子
     */
    float getAccelScaleFactorX();

    /**
     * @brief 获取Y轴加速度计零偏(m/s²)
     *
     * @return float Y轴零偏值
     */
    float getAccelBiasY_mss();

    /**
     * @brief 获取Y轴加速度计比例因子
     *
     * @return float Y轴比例因子
     */
    float getAccelScaleFactorY();

    /**
     * @brief 获取Z轴加速度计零偏(m/s²)
     *
     * @return float Z轴零偏值
     */
    float getAccelBiasZ_mss();

    /**
     * @brief 获取Z轴加速度计比例因子
     *
     * @return float Z轴比例因子
     */
    float getAccelScaleFactorZ();

    /**
     * @brief 设置X轴加速度计校准参数
     *
     * @param bias 零偏值(m/s²)
     * @param scaleFactor 比例因子
     */
    void setAccelCalX(float bias, float scaleFactor);

    /**
     * @brief 设置Y轴加速度计校准参数
     *
     * @param bias 零偏值(m/s²)
     * @param scaleFactor 比例因子
     */
    void setAccelCalY(float bias, float scaleFactor);

    /**
     * @brief 设置Z轴加速度计校准参数
     *
     * @param bias 零偏值(m/s²)
     * @param scaleFactor 比例因子
     */
    void setAccelCalZ(float bias, float scaleFactor);

protected:
    /// SPI数据传输函数
    uint8_t SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

    /// SPI句柄
    SPI_HandleTypeDef *_hspi = NULL;

    /// 传感器读取缓冲区
    uint8_t _buffer[15] = {};

    /// 数据缓冲区
    float Temperature = 0.0f;
    float _acc[3] = {};
    float _gyr[3] = {};

    int16_t _rawT = 0;
    int16_t _rawAcc[3] = {};
    int16_t _rawGyr[3] = {};

    /// 加速度计和陀螺仪原始零偏
    int32_t _rawAccBias[3] = {0, 0, 0};
    int32_t _rawGyrBias[3] = {0, 0, 0};

    /// 加速度计和陀螺仪Offset
    int16_t _AccOffset[3] = {0, 0, 0};
    int16_t _GyrOffset[3] = {0, 0, 0};

    /// 满量程分辨率因子
    float _accelScale = 0.0f;
    float _gyroScale = 0.0f;

    /// 满量程选择
    AccelFS _accelFS = gpm16;
    GyroFS _gyroFS = dps2000;

    /// 加速度计校准
    float _accBD[3] = {};
    float _accB[3] = {};
    float _accS[3] = {1.0f, 1.0f, 1.0f};
    float _accMax[3] = {};
    float _accMin[3] = {};

    /// 陀螺仪校准
    float _gyroBD[3] = {};
    float _gyrB[3] = {};

    /// 常量
    static constexpr uint8_t WHO_AM_I = 0x47;       ///< UB0_REG_WHO_AM_I寄存器期望值
    static constexpr int NUM_CALIB_SAMPLES = 1000;   ///< 陀螺仪/加速度计零偏校准采样数

    /// 温度转换公式(见数据手册Sec 4.13)
    static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
    static constexpr float TEMP_OFFSET = 25.0f;

    uint8_t _bank = 0; ///< 当前用户Bank

    const uint8_t FIFO_EN = 0x5F;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;

    // BANK 1
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

protected:
    /**
     * @brief 写入寄存器
     *
     * @param subAddress 寄存器地址
     * @param data 写入数据
     * @return int 成功返回1，失败返回负数
     */
    int writeRegister(uint8_t subAddress, uint8_t data);

    /**
     * @brief 读取寄存器
     *
     * @param subAddress 起始寄存器地址
     * @param count 读取字节数
     * @param dest 存储数据的指针
     * @return int 成功返回1，失败返回负数
     */
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);

    /**
     * @brief 切换寄存器Bank
     *
     * @param bank Bank号(0-4)
     * @return int 切换结果
     */
    int setBank(uint8_t bank);

    /**
     * @brief 软件复位设备
     */
    void reset();

    /**
     * @brief 读取WHO_AM_I寄存器
     *
     * @return int8_t WHO_AM_I寄存器值
     */
    int8_t whoAmI();

    /**
     * @brief 自检内部实现
     *
     * @param accelDiff 加速度计差分值输出
     * @param gyroDiff 陀螺仪差分值输出
     * @param ratio 比率输出
     */
    void selfTest(int16_t *accelDiff, int16_t *gyroDiff, float *ratio);
};

/**
 * @brief ICM-42688 FIFO扩展驱动类
 *
 *         继承自Class_ICM42688，增加FIFO读写功能
 */
class Class_ICM42688_FIFO : public Class_ICM42688
{
public:
    /**
     * @brief 使能FIFO缓冲区
     *
     * @param accel 使能加速度计FIFO
     * @param gyro 使能陀螺仪FIFO
     * @param temp 使能温度FIFO
     * @return int 成功返回1，失败返回负数错误码
     */
    int enableFifo(bool accel, bool gyro, bool temp);

    /**
     * @brief 启动FIFO数据流
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int streamToFifo();

    /**
     * @brief 读取FIFO数据
     *
     * @return int 成功返回1，失败返回负数错误码
     */
    int readFifo();

    /**
     * @brief 获取FIFO中X轴加速度数据(m/s²)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoAccelX_mss(size_t *size, float *data);

    /**
     * @brief 获取FIFO中Y轴加速度数据(m/s²)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoAccelY_mss(size_t *size, float *data);

    /**
     * @brief 获取FIFO中Z轴加速度数据(m/s²)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoAccelZ_mss(size_t *size, float *data);

    /**
     * @brief 获取FIFO中X轴陀螺仪数据(dps)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoGyroX(size_t *size, float *data);

    /**
     * @brief 获取FIFO中Y轴陀螺仪数据(dps)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoGyroY(size_t *size, float *data);

    /**
     * @brief 获取FIFO中Z轴陀螺仪数据(dps)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoGyroZ(size_t *size, float *data);

    /**
     * @brief 获取FIFO中芯片温度数据(°C)
     *
     * @param size 输出：数据点数
     * @param data 输出：数据缓冲区
     */
    void getFifoTemperature_C(size_t *size, float *data);

protected:
    /// FIFO配置
    bool _enFifoAccel = false;
    bool _enFifoGyro = false;
    bool _enFifoTemp = false;
    bool _enFifoTimestamp = false;
    bool _enFifoHeader = false;
    size_t _fifoSize = 0;
    size_t _fifoFrameSize = 0;

    /// 加速度计FIFO数据
    float _axFifo[85] = {};
    float _ayFifo[85] = {};
    float _azFifo[85] = {};
    size_t _aSize = 0;

    /// 陀螺仪FIFO数据
    float _gxFifo[85] = {};
    float _gyFifo[85] = {};
    float _gzFifo[85] = {};
    size_t _gSize = 0;

    /// 温度FIFO数据
    float _tFifo[256] = {};
    size_t _tSize = 0;
};

#endif // Class_ICM42688_H
