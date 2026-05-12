#include "dvc_ICM42688.h"
#include "dvc_ICM42688_reg.h"
#include "math.h"
#include "string.h"
#include "tsk_config_and_callback.h"
#define PI 3.14159265f

/**
 * @brief SPI发送接收底层封装
 *
 * @param hspi SPI句柄指针
 * @param tx_data 发送数据缓冲区
 * @param rx_data 接收数据缓冲区
 * @param size 数据长度
 * @return HAL状态
 */
static HAL_StatusTypeDef SPI_Transmit_Receive(SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    if (hspi == nullptr || tx_data == nullptr || rx_data == nullptr || size == 0)
    {
        return HAL_ERROR; // 参数检查
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 片选拉低
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 片选拉高

    return HAL_OK;
}

/**
 * @brief 初始化ICM42688设备
 *
 * @param hspi SPI句柄指针
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::Init(SPI_HandleTypeDef *hspi)
{
    _hspi = hspi;

    // 配置大端序（必须在reset之前设置，与C版本一致）
    writeRegister(UB0_REG_INTF_CONFIG0, 0x90);

    // 配置电源管理：6轴低噪声模式
    writeRegister(UB0_REG_PWR_MGMT0, 0x0F);

    // 检查WHO AM I寄存器
    if (whoAmI() != WHO_AM_I)
    {
        return -3;
    }

    // 配置加速度计：±8g, 1kHz ODR
    int ret = setAccelFS(gpm8);
    if (ret < 0)
    {
        return ret;
    }
    ret = setAccelODR(odr1k);
    if (ret < 0)
    {
        return ret;
    }

    // 配置陀螺仪：±500dps, 1kHz ODR
    ret = setGyroFS(dps500);
    if (ret < 0)
    {
        return ret;
    }
    ret = setGyroODR(odr1k);
    if (ret < 0)
    {
        return ret;
    }

    HAL_Delay(100);

    // 估计陀螺仪零偏（不切换量程，避免_scale被重新计算）
    {
        float sum[3] = {0, 0, 0};
        for (int i = 0; i < NUM_CALIB_SAMPLES; i++)
        {
            Get_Data();
            sum[0] += Gyro[0];
            sum[1] += Gyro[1];
            sum[2] += Gyro[2];
            delay(1);
        }
        _gyrB[0] = sum[0] / NUM_CALIB_SAMPLES;
        _gyrB[1] = sum[1] / NUM_CALIB_SAMPLES;
        _gyrB[2] = sum[2] / NUM_CALIB_SAMPLES;
    }

    // 初始化成功，返回1
    return 1;
}

/**
 * @brief SPI数据传输函数
 *
 * @param tx_data 发送数据缓冲区
 * @param rx_data 接收数据缓冲区
 * @param size 数据长度
 * @return uint8_t HAL状态
 */
uint8_t Class_ICM42688::SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    uint8_t result = SPI_Transmit_Receive(_hspi, tx_data, rx_data, size);
    return result;
}

/**
 * @brief 设置加速度计满量程范围
 *
 * @param fssel 满量程选择
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAccelFS(AccelFS fssel)
{
    setBank(0);

    // 读取当前寄存器值
    uint8_t reg;
    if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }

    // 仅修改FS_SEL位
    reg = (fssel << 5) | (reg & 0x1F);

    if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0)
    {
        return -2;
    }

    _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
    _accelFS = fssel;

    return 1;
}

/**
 * @brief 获取加速度计满量程范围
 *
 * @return int ACCEL_FS_SEL值，失败返回-1
 */
int Class_ICM42688::getAccelFS()
{
    setBank(0);
    uint8_t reg;
    if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }
    return (reg & 0xE0) >> 5;
}

/**
 * @brief 设置陀螺仪满量程范围
 *
 * @param fssel 满量程选择
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyroFS(GyroFS fssel)
{
    setBank(0);

    uint8_t reg;
    if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }

    reg = (fssel << 5) | (reg & 0x1F);

    if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0)
    {
        return -2;
    }

    _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
    _gyroFS = fssel;

    return 1;
}

/**
 * @brief 获取陀螺仪满量程范围
 *
 * @return int GYRO_FS_SEL值，失败返回-1
 */
int Class_ICM42688::getGyroFS()
{
    setBank(0);
    uint8_t reg;
    if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }
    return (reg & 0xE0) >> 5;
}

/**
 * @brief 设置加速度计输出数据速率(ODR)
 *
 * @param odr 输出数据速率
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAccelODR(ODR odr)
{
    setBank(0);

    uint8_t reg;
    if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }

    // 仅修改ODR位
    reg = odr | (reg & 0xF0);

    if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0)
    {
        return -2;
    }

    return 1;
}

/**
 * @brief 设置陀螺仪输出数据速率(ODR)
 *
 * @param odr 输出数据速率
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyroODR(ODR odr)
{
    setBank(0);

    uint8_t reg;
    if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }

    reg = odr | (reg & 0xF0);

    if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0)
    {
        return -2;
    }

    return 1;
}

/**
 * @brief 设置滤波器使能
 *
 * @param gyroFilters 陀螺仪滤波器使能
 * @param accFilters 加速度计滤波器使能
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setFilters(bool gyroFilters, bool accFilters)
{
    if (setBank(1) < 0)
    {
        return -1;
    }

    if (gyroFilters == true)
    {
        if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0)
        {
            return -2;
        }
    }
    else
    {
        if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0)
        {
            return -3;
        }
    }

    if (setBank(2) < 0)
    {
        return -4;
    }

    if (accFilters == true)
    {
        if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0)
        {
            return -5;
        }
    }
    else
    {
        if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0)
        {
            return -6;
        }
    }
    if (setBank(0) < 0)
    {
        return -7;
    }
    return 1;
}

/**
 * @brief 使能数据就绪中断
 *
 *         - 将UI数据就绪中断路由到INT1
 *         - 推挽、脉冲、高电平有效中断
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::enableDataReadyInterrupt()
{
    // 推挽、脉冲、高电平有效中断
    if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0)
    {
        return -1;
    }

    // 需要清除bit4以允许INT1和INT2正常工作
    uint8_t reg;
    if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    {
        return -2;
    }
    reg &= ~0x10;
    if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0)
    {
        return -3;
    }

    // 将UI数据就绪中断路由到INT1
    if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0)
    {
        return -4;
    }

    return 1;
}

/**
 * @brief 禁用数据就绪中断
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::disableDataReadyInterrupt()
{
    uint8_t reg;
    if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    {
        return -1;
    }
    reg |= 0x10;
    if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0)
    {
        return -2;
    }

    // 恢复中断源寄存器到复位值
    if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0)
    {
        return -3;
    }

    return 1;
}

/**
 * @brief 读取ICM42688最新数据并转换为物理量
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::Get_Data()
{
    if (Get_Raw_Data() < 0)
    {
        return -1;
    }

    Temperature = (static_cast<float>(_rawT) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

    Accel[0] = ((_rawAcc[0] * _accelScale) - _accB[0]) * _accS[0];
    Accel[1] = ((_rawAcc[1] * _accelScale) - _accB[1]) * _accS[1];
    Accel[2] = ((_rawAcc[2] * _accelScale) - _accB[2]) * _accS[2];

    Gyro[0] = (_rawGyr[0] * _gyroScale) - _gyrB[0];
    Gyro[1] = (_rawGyr[1] * _gyroScale) - _gyrB[1];
    Gyro[2] = (_rawGyr[2] * _gyroScale) - _gyrB[2];

    return 1;
}

/**
 * @brief 读取ICM42688原始数据
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::Get_Raw_Data()
{
    if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0)
    {
        return -1;
    }

    // 组合字节为16位值
    int16_t rawMeas[7]; // 温度、加速度XYZ、陀螺仪XYZ
    for (size_t i = 0; i < 7; i++)
    {
        rawMeas[i] = ((int16_t)_buffer[i * 2] << 8) | _buffer[i * 2 + 1];
    }

    _rawT = rawMeas[0];
    _rawAcc[0] = rawMeas[1];
    _rawAcc[1] = rawMeas[2];
    _rawAcc[2] = rawMeas[3];
    _rawGyr[0] = rawMeas[4];
    _rawGyr[1] = rawMeas[5];
    _rawGyr[2] = rawMeas[6];

    return 1;
}

/**
 * @brief 使能FIFO缓冲区
 *
 *         配置FIFO结构，强制使用最佳匹配的结构(1、2或3)
 *
 * @param accel 使能加速度计FIFO
 * @param gyro 使能陀螺仪FIFO
 * @param temp 使能温度FIFO
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688_FIFO::enableFifo(bool accel, bool gyro, bool temp)
{
    _enFifoAccel = accel;
    _enFifoGyro = gyro;
    _enFifoTemp = true; // 所有结构都包含1字节温度，为保持向后兼容性不返回错误
    _enFifoTimestamp =
        accel && gyro;             // 只能在结构3或4中同时读取加速度计和陀螺仪，两者都有2字节时间戳
    _enFifoHeader = accel || gyro; // 如果都不请求传感器，FIFO将不再发送数据包
    _fifoFrameSize = _enFifoHeader * 1 + _enFifoAccel * 6 + _enFifoGyro * 6 + _enFifoTemp + _enFifoTimestamp * 2;

    if (writeRegister(FIFO_EN, (_enFifoAccel * FIFO_ACCEL) | (_enFifoGyro * FIFO_GYRO) | (_enFifoTemp * FIFO_TEMP_EN)) < 0)
    {
        return -2;
    }
    return 1;
}

/**
 * @brief 启动FIFO数据流
 *
 *         在大多数传感器配置下，enableFifo()后必须调用此函数
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688_FIFO::streamToFifo()
{
    if (writeRegister(UB0_REG_FIFO_CONFIG, 1 << 6) < 0)
    {
        return -2;
    }
    return 1;
}

/**
 * @brief 读取ICM42688 FIFO数据并解析
 *
 *         尚不支持高分辨率模式
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688_FIFO::readFifo()
{
    // 获取FIFO数据量
    readRegisters(UB0_REG_FIFO_COUNTH, 2, _buffer);
    _fifoSize = (((uint16_t)(_buffer[0] & 0x0F)) << 8) + ((uint16_t)_buffer[1]);

    // 预计算数据包结构，因为逐包根据头部重新计算不可靠
    // 头部不确认数据包是否为高分辨率(20位)数据
    size_t numFrames = _fifoSize / _fifoFrameSize;
    size_t accIndex = 1;
    size_t gyroIndex = accIndex + _enFifoAccel * 6;
    size_t tempIndex = gyroIndex + _enFifoGyro * 6;

    // 读取并解析缓冲区
    for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++)
    {
        if (readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer) < 0)
        {
            return -1;
        }
        if (_enFifoAccel)
        {
            // 组合为16位值
            int16_t rawMeas[3];
            rawMeas[0] = (((int16_t)_buffer[0 + accIndex]) << 8) | _buffer[1 + accIndex];
            rawMeas[1] = (((int16_t)_buffer[2 + accIndex]) << 8) | _buffer[3 + accIndex];
            rawMeas[2] = (((int16_t)_buffer[4 + accIndex]) << 8) | _buffer[5 + accIndex];
            // 转换并转为浮点值
            _axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
            _ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
            _azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
            _aSize = numFrames;
        }
        if (_enFifoTemp)
        {
            int8_t rawMeas = _buffer[tempIndex + 0];
            _tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
            _tSize = numFrames;
        }
        if (_enFifoGyro)
        {
            int16_t rawMeas[3];
            rawMeas[0] = (((int16_t)_buffer[0 + gyroIndex]) << 8) | _buffer[1 + gyroIndex];
            rawMeas[1] = (((int16_t)_buffer[2 + gyroIndex]) << 8) | _buffer[3 + gyroIndex];
            rawMeas[2] = (((int16_t)_buffer[4 + gyroIndex]) << 8) | _buffer[5 + gyroIndex];
            _gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
            _gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
            _gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
            _gSize = numFrames;
        }
    }
    return 1;
}

/**
 * @brief 获取FIFO中X轴加速度数据(m/s²)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoAccelX_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _axFifo, _aSize * sizeof(float));
}

/**
 * @brief 获取FIFO中Y轴加速度数据(m/s²)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoAccelY_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _ayFifo, _aSize * sizeof(float));
}

/**
 * @brief 获取FIFO中Z轴加速度数据(m/s²)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoAccelZ_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _azFifo, _aSize * sizeof(float));
}

/**
 * @brief 获取FIFO中X轴陀螺仪数据(dps)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoGyroX(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gxFifo, _gSize * sizeof(float));
}

/**
 * @brief 获取FIFO中Y轴陀螺仪数据(dps)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoGyroY(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gyFifo, _gSize * sizeof(float));
}

/**
 * @brief 获取FIFO中Z轴陀螺仪数据(dps)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoGyroZ(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gzFifo, _gSize * sizeof(float));
}

/**
 * @brief 获取FIFO中芯片温度数据(°C)
 *
 * @param size 输出：数据点数
 * @param data 输出：数据缓冲区
 */
void Class_ICM42688_FIFO::getFifoTemperature_C(size_t *size, float *data)
{
    *size = _tSize;
    memcpy(data, _tFifo, _tSize * sizeof(float));
}

/**
 * @brief 估计陀螺仪零偏
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::calibrateGyro()
{
    // 使用与运行相同的量程(更准确)，因为IMU静止不动
    const GyroFS current_fssel = _gyroFS;
    if (setGyroFS(current_fssel) < 0)
    {
        return -1;
    }

    // 采集样本并找出零偏
    _gyroBD[0] = 0;
    _gyroBD[1] = 0;
    _gyroBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++)
    {
        Get_Data();
        _gyroBD[0] += (Get_GyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
        _gyroBD[1] += (Get_GyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
        _gyroBD[2] += (Get_GyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;
        delay(1);
    }
    _gyrB[0] = _gyroBD[0];
    _gyrB[1] = _gyroBD[1];
    _gyrB[2] = _gyroBD[2];

    // 恢复满量程设置
    if (setGyroFS(current_fssel) < 0)
    {
        return -4;
    }
    return 1;
}

/**
 * @brief 获取X轴陀螺仪零偏(dps)
 *
 * @return float X轴零偏值
 */
float Class_ICM42688::getGyroBiasX()
{
    return _gyrB[0];
}

/**
 * @brief 获取Y轴陀螺仪零偏(dps)
 *
 * @return float Y轴零偏值
 */
float Class_ICM42688::getGyroBiasY()
{
    return _gyrB[1];
}

/**
 * @brief 获取Z轴陀螺仪零偏(dps)
 *
 * @return float Z轴零偏值
 */
float Class_ICM42688::getGyroBiasZ()
{
    return _gyrB[2];
}

/**
 * @brief 设置X轴陀螺仪零偏(dps)
 *
 * @param bias 零偏值
 */
void Class_ICM42688::setGyroBiasX(float bias)
{
    _gyrB[0] = bias;
}

/**
 * @brief 设置Y轴陀螺仪零偏(dps)
 *
 * @param bias 零偏值
 */
void Class_ICM42688::setGyroBiasY(float bias)
{
    _gyrB[1] = bias;
}

/**
 * @brief 设置Z轴陀螺仪零偏(dps)
 *
 * @param bias 零偏值
 */
void Class_ICM42688::setGyroBiasZ(float bias)
{
    _gyrB[2] = bias;
}

/**
 * @brief 加速度计零偏和比例因子校准
 *
 *         应对每个轴的每个方向(共6个方向)运行此函数，
 *         以找出每个方向的最小值和最大值
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::calibrateAccel()
{
    // 使用较低量程(更高分辨率)，因为IMU静止不动
    const AccelFS current_fssel = _accelFS;
    if (setAccelFS(gpm2) < 0)
    {
        return -1;
    }

    // 采集样本并找出最小/最大值
    _accBD[0] = 0;
    _accBD[1] = 0;
    _accBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++)
    {
        Get_Data();
        _accBD[0] += (Get_AccX() / _accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
        _accBD[1] += (Get_AccY() / _accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
        _accBD[2] += (Get_AccZ() / _accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
        delay(1);
    }
    if (_accBD[0] > 0.9f)
    {
        _accMax[0] = _accBD[0];
    }
    if (_accBD[1] > 0.9f)
    {
        _accMax[1] = _accBD[1];
    }
    if (_accBD[2] > 0.9f)
    {
        _accMax[2] = _accBD[2];
    }
    if (_accBD[0] < -0.9f)
    {
        _accMin[0] = _accBD[0];
    }
    if (_accBD[1] < -0.9f)
    {
        _accMin[1] = _accBD[1];
    }
    if (_accBD[2] < -0.9f)
    {
        _accMin[2] = _accBD[2];
    }

    // 计算零偏和比例因子
    if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f))
    {
        _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
        _accS[0] = 1 / ((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
    }
    if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f))
    {
        _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
        _accS[1] = 1 / ((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
    }
    if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f))
    {
        _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
        _accS[2] = 1 / ((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
    }

    // 恢复满量程设置
    if (setAccelFS(current_fssel) < 0)
    {
        return -4;
    }
    return 1;
}

/**
 * @brief 获取X轴加速度计零偏(m/s²)
 *
 * @return float X轴零偏值
 */
float Class_ICM42688::getAccelBiasX_mss()
{
    return _accB[0];
}

/**
 * @brief 获取X轴加速度计比例因子
 *
 * @return float X轴比例因子
 */
float Class_ICM42688::getAccelScaleFactorX()
{
    return _accS[0];
}

/**
 * @brief 获取Y轴加速度计零偏(m/s²)
 *
 * @return float Y轴零偏值
 */
float Class_ICM42688::getAccelBiasY_mss()
{
    return _accB[1];
}

/**
 * @brief 获取Y轴加速度计比例因子
 *
 * @return float Y轴比例因子
 */
float Class_ICM42688::getAccelScaleFactorY()
{
    return _accS[1];
}

/**
 * @brief 获取Z轴加速度计零偏(m/s²)
 *
 * @return float Z轴零偏值
 */
float Class_ICM42688::getAccelBiasZ_mss()
{
    return _accB[2];
}

/**
 * @brief 获取Z轴加速度计比例因子
 *
 * @return float Z轴比例因子
 */
float Class_ICM42688::getAccelScaleFactorZ()
{
    return _accS[2];
}

/**
 * @brief 设置X轴加速度计零偏(m/s²)和比例因子
 *
 * @param bias 零偏值
 * @param scaleFactor 比例因子
 */
void Class_ICM42688::setAccelCalX(float bias, float scaleFactor)
{
    _accB[0] = bias;
    _accS[0] = scaleFactor;
}

/**
 * @brief 设置Y轴加速度计零偏(m/s²)和比例因子
 *
 * @param bias 零偏值
 * @param scaleFactor 比例因子
 */
void Class_ICM42688::setAccelCalY(float bias, float scaleFactor)
{
    _accB[1] = bias;
    _accS[1] = scaleFactor;
}

/**
 * @brief 设置Z轴加速度计零偏(m/s²)和比例因子
 *
 * @param bias 零偏值
 * @param scaleFactor 比例因子
 */
void Class_ICM42688::setAccelCalZ(float bias, float scaleFactor)
{
    _accB[2] = bias;
    _accS[2] = scaleFactor;
}

/**
 * @brief 向ICM42688寄存器写入一个字节
 *
 * @param subAddress 寄存器地址
 * @param data 写入的数据
 * @return int 成功返回1，失败返回-1
 */
int Class_ICM42688::writeRegister(uint8_t subAddress, uint8_t data)
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    // 构造发送数据：寄存器地址（写命令）和数据
    tx_buffer[0] = subAddress & 0x7F; // 写命令，bit7=0
    tx_buffer[1] = data;

    // 使用SPI_Transfer函数进行传输
    if (SPI_Transfer(tx_buffer, rx_buffer, 2) != HAL_OK)
    {
        return -1;
    }

    return 1;
}

/**
 * @brief 从ICM42688寄存器读取多个字节
 *
 * @param subAddress 起始寄存器地址
 * @param count 读取字节数
 * @param dest 存储数据的指针
 * @return int 成功返回1，失败返回-1
 */
int Class_ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    uint8_t tx_buffer[16] = {0}; // 最大16字节传输
    uint8_t rx_buffer[16] = {0};

    // 构造发送数据：寄存器地址（读命令）
    tx_buffer[0] = subAddress | 0x80;

    // 使用SPI_Transfer函数进行传输
    if (SPI_Transfer(tx_buffer, rx_buffer, count + 1) != HAL_OK)
    {
        return -1;
    }

    // 复制接收到的数据（跳过第一个字节，它是发送的寄存器地址）
    memcpy(dest, &rx_buffer[1], count);

    return 1;
}

/**
 * @brief 切换到指定寄存器Bank
 *
 * @param bank Bank号(0-4)
 * @return int 切换结果
 */
int Class_ICM42688::setBank(uint8_t bank)
{
    // 如果已经在此Bank则直接返回
    if (_bank == bank)
    {
        return 1;
    }

    _bank = bank;

    return writeRegister(REG_BANK_SEL, bank);
}

/**
 * @brief 软件复位设备
 */
void Class_ICM42688::reset()
{
    setBank(0);

    writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

    // 等待ICM42688重新启动
    delay(1);
}

/**
 * @brief 读取WHO_AM_I寄存器
 *
 * @return int8_t WHO_AM_I寄存器值，失败返回-1
 */
int8_t Class_ICM42688::whoAmI()
{
    setBank(0);

    if (readRegisters(UB0_REG_WHO_AM_I, 1, _buffer) < 0)
    {
        return -1;
    }
    return _buffer[0];
}

/**
 * @brief 计算原始零偏(Offset)
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::computeOffsets()
{
    const AccelFS current_Accelfssel = _accelFS;
    const GyroFS current_Gyrofssel = _gyroFS;

    // 设置IMU到正确的分辨率
    setAccelFS(Class_ICM42688::AccelFS::gpm2);
    setGyroFS(Class_ICM42688::GyroFS::dps250);
    int16_t FullScale_Acc = 2;
    int16_t FullScale_Gyr = 250;

    // 清零Offset_user寄存器
    setBank(4);
    if (writeRegister(UB4_REG_OFFSET_USER5, 0) < 0)
    {
        return -2; // 低Ax字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER6, 0) < 0)
    {
        return -2; // 低Ay字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER8, 0) < 0)
    {
        return -2; // 低Az字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER2, 0) < 0)
    {
        return -2; // 低Gy字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER3, 0) < 0)
    {
        return -2; // 低Gz字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER0, 0) < 0)
    {
        return -2; // 低Gx字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER4, 0) < 0)
    {
        return -2; // 高Ax和Gz字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER7, 0) < 0)
    {
        return -2; // 高Az和Ay字节
    }
    if (writeRegister(UB4_REG_OFFSET_USER1, 0) < 0)
    {
        return -2; // 高Gy和Gx字节
    }
    setBank(0);
    // 重新初始化_rawAccBias和_rawGyrBias
    for (size_t ii = 1; ii < 3; ii++)
    {
        _rawAccBias[ii] = 0;
        _rawGyrBias[ii] = 0;
    }
    // 记录原始值并累加样本
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++)
    {
        Get_Raw_Data();
        _rawAccBias[0] += _rawAcc[0];
        _rawAccBias[1] += _rawAcc[1];
        _rawAccBias[2] += _rawAcc[2];
        _rawGyrBias[0] += _rawGyr[0];
        _rawGyrBias[1] += _rawGyr[1];
        _rawGyrBias[2] += _rawGyr[2];
        delay(1);
    }

    // 求平均
    _rawAccBias[0] = (int32_t)((double)_rawAccBias[0] / (double)NUM_CALIB_SAMPLES);
    _rawAccBias[1] = (int32_t)((double)_rawAccBias[1] / (double)NUM_CALIB_SAMPLES);
    _rawAccBias[2] = (int32_t)((double)_rawAccBias[2] / (double)NUM_CALIB_SAMPLES);
    _rawGyrBias[0] = (int32_t)((double)_rawGyrBias[0] / (double)NUM_CALIB_SAMPLES);
    _rawGyrBias[1] = (int32_t)((double)_rawGyrBias[1] / (double)NUM_CALIB_SAMPLES);
    _rawGyrBias[2] = (int32_t)((double)_rawGyrBias[2] / (double)NUM_CALIB_SAMPLES);

    // 补偿重力并计算_AccOffset和_GyrOffset
    for (size_t ii = 0; ii < 3; ii++)
    {
        _AccOffset[ii] =
            (int16_t)(-(_rawAccBias[ii]) * (FullScale_Acc / 32768.0f * 2048));
        if (_rawAccBias[ii] * FullScale_Acc > 26000)
        {
            _AccOffset[ii] = (int16_t)(-(_rawAccBias[ii] - 32768 / FullScale_Acc) * (FullScale_Acc / 32768.0f * 2048));
        } // 26000 ~80% of 32768
        if (_rawAccBias[ii] * FullScale_Acc < -26000)
        {
            _AccOffset[ii] = (int16_t)(-(_rawAccBias[ii] + 32768 / FullScale_Acc) * (FullScale_Acc / 32768.0f * 2048));
        }
        _GyrOffset[ii] = (int16_t)((-_rawGyrBias[ii]) * (FullScale_Gyr / 32768.0f * 32)); // 1/32 dps分辨率
    }

    if (setAccelFS(current_Accelfssel) < 0)
    {
        return -4;
    }
    if (setGyroFS(current_Gyrofssel) < 0)
    {
        return -4;
    }
    return 1;
}

/**
 * @brief 设置所有计算出的Offset
 *
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAllOffsets()
{
    setBank(4);
    uint8_t reg;

    // 清零所有offset寄存器
    if (writeRegister(UB4_REG_OFFSET_USER0, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER1, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER2, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER3, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER4, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER5, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER6, 0) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER7, 0) < 0)
    {
        return -2;
    }

    reg = _AccOffset[0] & 0x00FF; // 低Ax字节
    if (writeRegister(UB4_REG_OFFSET_USER5, reg) < 0)
    {
        return -2;
    }
    reg = _AccOffset[1] & 0x00FF; // 低Ay字节
    if (writeRegister(UB4_REG_OFFSET_USER6, reg) < 0)
    {
        return -2;
    }
    reg = _AccOffset[2] & 0x00FF; // 低Az字节
    if (writeRegister(UB4_REG_OFFSET_USER8, reg) < 0)
    {
        return -2;
    }

    reg = _GyrOffset[1] & 0x00FF; // 低Gy字节
    if (writeRegister(UB4_REG_OFFSET_USER2, reg) < 0)
    {
        return -2;
    }
    reg = _GyrOffset[2] & 0x00FF; // 低Gz字节
    if (writeRegister(UB4_REG_OFFSET_USER3, reg) < 0)
    {
        return -2;
    }
    reg = _GyrOffset[0] & 0x00FF; // 低Gx字节
    if (writeRegister(UB4_REG_OFFSET_USER0, reg) < 0)
    {
        return -2;
    }

    reg = (_AccOffset[0] & 0x0F00) >> 4 | (_GyrOffset[2] & 0x0F00) >> 8; // 高Ax和Gz字节
    if (writeRegister(UB4_REG_OFFSET_USER4, reg) < 0)
    {
        return -2;
    }
    reg = (_AccOffset[2] & 0x0F00) >> 4 | (_AccOffset[1] & 0x0F00) >> 8; // 高Az和Ay字节
    if (writeRegister(UB4_REG_OFFSET_USER7, reg) < 0)
    {
        return -2;
    }
    reg = (_GyrOffset[1] & 0x0F00) >> 4 | (_GyrOffset[0] & 0x0F00) >> 8; // 高Gy和Gx字节
    if (writeRegister(UB4_REG_OFFSET_USER1, reg) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置X轴加速度计Offset
 *
 * @param accXoffset X轴加速度计偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAccXOffset(int16_t accXoffset)
{
    setBank(4);
    uint8_t reg1 = (accXoffset & 0x00FF);
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER4, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0x0F) | ((accXoffset & 0x0F00) >> 4);
    if (writeRegister(UB4_REG_OFFSET_USER5, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER4, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置Y轴加速度计Offset
 *
 * @param accYoffset Y轴加速度计偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAccYOffset(int16_t accYoffset)
{
    setBank(4);
    uint8_t reg1 = (accYoffset & 0x00FF);
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER7, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0xF0) | ((accYoffset & 0x0F00) >> 8);
    if (writeRegister(UB4_REG_OFFSET_USER6, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER7, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置Z轴加速度计Offset
 *
 * @param accZoffset Z轴加速度计偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setAccZOffset(int16_t accZoffset)
{
    setBank(4);
    uint8_t reg1 = accZoffset & 0x00FF;
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER7, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0x0F) | ((accZoffset & 0x0F00) >> 4);
    if (writeRegister(UB4_REG_OFFSET_USER8, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER7, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置X轴陀螺仪Offset
 *
 * @param gyrXoffset X轴陀螺仪偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyrXOffset(int16_t gyrXoffset)
{
    setBank(4);
    uint8_t reg1 = gyrXoffset & 0x00FF;
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER1, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0xF0) | ((gyrXoffset & 0x0F00) >> 8);
    if (writeRegister(UB4_REG_OFFSET_USER0, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER1, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置Y轴陀螺仪Offset
 *
 * @param gyrYoffset Y轴陀螺仪偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyrYOffset(int16_t gyrYoffset)
{
    setBank(4);
    uint8_t reg1 = gyrYoffset & 0x00FF;
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER1, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0x0F) | ((gyrYoffset & 0x0F00) >> 4);
    reg2 = (gyrYoffset & 0x0F00) >> 4 | (reg2 & 0x0F00) >> 4;
    if (writeRegister(UB4_REG_OFFSET_USER2, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER1, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 单独设置Z轴陀螺仪Offset
 *
 * @param gyrZoffset Z轴陀螺仪偏移量
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyrZOffset(int16_t gyrZoffset)
{
    setBank(4);
    uint8_t reg1 = gyrZoffset & 0x00FF;
    uint8_t reg2;
    if (readRegisters(UB4_REG_OFFSET_USER4, 1, &reg2) < 0)
    {
        return -1;
    }
    reg2 = (reg2 & 0xF0) | ((gyrZoffset & 0x0F00) >> 8);
    if (writeRegister(UB4_REG_OFFSET_USER3, reg1) < 0)
    {
        return -2;
    }
    if (writeRegister(UB4_REG_OFFSET_USER4, reg2) < 0)
    {
        return -2;
    }
    setBank(0);
    return 1;
}

/**
 * @brief 设置UI滤波器组
 *
 * @param gyroUIFiltOrder 陀螺仪UI滤波器阶数
 * @param accelUIFiltOrder 加速度计UI滤波器阶数
 * @return int 当前仅返回1
 */
int Class_ICM42688::setUIFilterBlock(UIFiltOrd gyroUIFiltOrder, UIFiltOrd accelUIFiltOrder)
{
    return 1;
}

/**
 * @brief 设置陀螺仪陷波滤波器
 *
 * @param gyroNFfreq_x X轴陷波频率
 * @param gyroNFfreq_y Y轴陷波频率
 * @param gyroNFfreq_z Z轴陷波频率
 * @param gyro_nf_bw 陷波带宽选择
 * @return int 成功返回1，失败返回负数错误码
 */
int Class_ICM42688::setGyroNotchFilter(float gyroNFfreq_x, float gyroNFfreq_y, float gyroNFfreq_z, GyroNFBWsel gyro_nf_bw)
{
    setBank(3);
    uint8_t reg;
    if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
    {
        return -1;
    }
    uint8_t clkdiv = reg & 0x3F;
    setBank(1);
    uint16_t nf_coswz;
    uint8_t gyro_nf_coswz_low[3] = {0};
    uint8_t buff = 0;
    float Fdrv = 19200 / (clkdiv * 10.0f);                                // 单位kHz (19.2MHz = 19200 kHz)
    const float fdesired[3] = {gyroNFfreq_x, gyroNFfreq_y, gyroNFfreq_z}; // 单位kHz - fdesired介于1kHz到3kHz
    for (size_t ii = 0; ii < 3; ii++)
    {
        float coswz = cos(2 * PI * fdesired[ii] / Fdrv);
        if (coswz <= 0.875)
        {
            nf_coswz = (uint16_t)round(coswz * 256);
            gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF); // 取低位
            buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);     // 取高位并拼接到缓冲区
        }
        else
        {
            buff = buff & (1 << (3 + ii));
            if (coswz > 0.875)
            {
                nf_coswz = (uint16_t)round(8 * (1 - coswz) * 256);
                gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF); // 取低位
                buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);     // 取高位并拼接到缓冲区
            }
            else if (coswz < -0.875)
            {
                nf_coswz = (uint16_t)round(-8 * (1 - coswz) * 256);
                gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF); // 取低位
                buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);     // 取高位并拼接到缓冲区
            }
        }
    }
    // 写入寄存器
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC6, gyro_nf_coswz_low[0]) < 0)
    {
        return -2;
    }
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC7, gyro_nf_coswz_low[1]) < 0)
    {
        return -2;
    }
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC8, gyro_nf_coswz_low[2]) < 0)
    {
        return -2;
    }
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC9, buff) < 0)
    {
        return -2;
    }
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC10, gyro_nf_bw) < 0)
    {
        return -2;
    }
    // 切回Bank 0以允许数据测量
    setBank(0);
    return 1;
}

/**
 * @brief 测试函数
 *
 * @return int 始终返回1
 */
int Class_ICM42688::testingFunction()
{
    return 1;
}

/**
 * @brief 获取加速度计量程分辨率
 *
 * @return float 加速度计分辨率
 */
float Class_ICM42688::getAccelRes()
{
    int currentAccFS = getAccelFS();
    float accRes;
    switch (currentAccFS)
    {
        case Class_ICM42688::AccelFS::gpm2:
            accRes = 16.0f / (32768.0f);
            break;
        case Class_ICM42688::AccelFS::gpm4:
            accRes = 4.0f / (32768.0f);
            break;
        case Class_ICM42688::AccelFS::gpm8:
            accRes = 8.0f / (32768.0f);
            break;
        case Class_ICM42688::AccelFS::gpm16:
            accRes = 16.0f / (32768.0f);
            break;
    }
    return accRes;
}

/**
 * @brief 获取陀螺仪量程分辨率
 *
 * @return float 陀螺仪分辨率
 */
float Class_ICM42688::getGyroRes()
{
    int currentGyroFS = getGyroFS();
    float gyroRes;
    switch (currentGyroFS)
    {
        case Class_ICM42688::GyroFS::dps2000:
            gyroRes = 2000.0f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps1000:
            gyroRes = 1000.0f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps500:
            gyroRes = 500.0f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps250:
            gyroRes = 250.0f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps125:
            gyroRes = 125.0f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps62_5:
            gyroRes = 62.5f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps31_25:
            gyroRes = 31.25f / 32768.0f;
            break;
        case Class_ICM42688::GyroFS::dps15_625:
            gyroRes = 15.625f / 32768.0f;
            break;
    }
    return gyroRes;
}

/**
 * @brief 自检功能
 *
 * @return int 当前仅返回1
 */
int Class_ICM42688::selfTest()
{
    return 1;
}

/*
// TODO清单
// 高优先级
原始数据读取          <-- 已验证
零偏计算              <-- 已验证
零偏设置              <-- 已验证
获取满量程分辨率      <-- 待测试
陷波滤波器            <-- 待测试
AAF抗混叠滤波器       <-- 待开发
UI滤波器组            <-- 待开发
自检功能              <-- 待开发

// 低优先级
读取INT_STATUS        <-- 获取数据可用信息

ApexStatus => INT_STATUS2 和 INT_STATUS3
8.1 APEX ODR支持
8.2 DMP节能模式
8.3 计步器编程
8.4 倾斜检测编程
8.5 抬手唤醒/睡眠编程
8.6 轻敲检测编程
8.7 运动唤醒编程
8.8 显著运动检测编程 p47
*/
