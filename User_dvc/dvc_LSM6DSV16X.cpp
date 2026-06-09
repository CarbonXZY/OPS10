#include "dvc_LSM6DSV16X.h"
#include "dvc_LSM6DSV16X_reg.h"
#include "math.h"
#include "string.h"
#include "tsk_config_and_callback.h"
#define PI 3.14159265f

/**
 * @brief SPI 发送+接收底层封装
 */
static HAL_StatusTypeDef SPI_Transmit_Receive(SPI_HandleTypeDef *hspi,
                                               uint8_t *tx_data,
                                               uint8_t *rx_data,
                                               uint16_t size)
{
    if (hspi == nullptr || tx_data == nullptr || rx_data == nullptr || size == 0)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    return HAL_OK;
}

// ======================== Init ========================

int Class_LSM6DSV16X::Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    _hspi = hspi;
    _cs_port = cs_port;
    _cs_pin = cs_pin;

    // 使能 SPI 地址自增 (CTRL3 bit 2)
    writeRegister(LSM6DSV16X_CTRL3, 0x04);

    // 使能 BDU (CTRL3 bit 6) — 防止读取时高低字节来自不同采样
    writeRegister(LSM6DSV16X_CTRL3, 0x44);

    // 检查 WHO_AM_I
    if (whoAmI() != WHO_AM_I)
    {
        return -3;
    }

    // 加速度计: ±8g, 1kHz ODR (high-performance mode)
    int ret = setAccelFS(gpm8);
    if (ret < 0) return ret;

    ret = setAccelODR(odr960Hz);
    if (ret < 0) return ret;

    // 陀螺仪: ±2000dps, 960Hz ODR (high-performance mode)
    ret = setGyroFS(dps2000);
    if (ret < 0) return ret;

    ret = setGyroODR(odr960Hz);
    if (ret < 0) return ret;

    HAL_Delay(100);

    // 估计陀螺仪零偏
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

    return 1;
}

// ======================== SPI Transfer ========================

uint8_t Class_LSM6DSV16X::SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    uint8_t result = SPI_Transmit_Receive(_hspi, tx_data, rx_data, size);
    return result;
}

// ======================== Register R/W ========================

int Class_LSM6DSV16X::writeRegister(uint8_t subAddress, uint8_t data)
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    tx_buffer[0] = subAddress;       // bit7=0 -> write
    tx_buffer[1] = data;

    if (SPI_Transfer(tx_buffer, rx_buffer, 2) != HAL_OK)
    {
        return -1;
    }
    return 1;
}

int Class_LSM6DSV16X::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    uint8_t tx_buffer[17] = {0};
    uint8_t rx_buffer[17] = {0};

    // LSM6DSV16X SPI 读: bit7=1
    tx_buffer[0] = subAddress | 0x80;

    if (SPI_Transfer(tx_buffer, rx_buffer, count + 1) != HAL_OK)
    {
        return -1;
    }

    memcpy(dest, &rx_buffer[1], count);
    return 1;
}

// ======================== WHO_AM_I / Reset ========================

void Class_LSM6DSV16X::reset()
{
    // CTRL3 bit 0: sw_reset
    writeRegister(LSM6DSV16X_CTRL3, 0x41);
    delay(1);
}

int8_t Class_LSM6DSV16X::whoAmI()
{
    if (readRegisters(LSM6DSV16X_WHO_AM_I_REG, 1, _buffer) < 0)
    {
        return -1;
    }
    return _buffer[0];
}

// ======================== Accel FS ========================

int Class_LSM6DSV16X::setAccelFS(AccelFS fssel)
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL8, 1, &reg) < 0)
    {
        return -1;
    }

    // 修改 fs_xl 位 [1:0]
    reg = (reg & 0xFC) | (fssel & 0x03);

    if (writeRegister(LSM6DSV16X_CTRL8, reg) < 0)
    {
        return -2;
    }

    _accelFS = fssel;

    // LSM6DSV16X accel 灵敏度: 2g/4g/8g/16g → 0.061/0.122/0.244/0.488 mg/LSB
    // 转 g/LSB:
    switch (fssel)
    {
        case gpm2:   _accelScale = 0.000061f; break;
        case gpm4:   _accelScale = 0.000122f; break;
        case gpm8:   _accelScale = 0.000244f; break;
        case gpm16:  _accelScale = 0.000488f; break;
        default:     _accelScale = 0.000244f; break;
    }

    return 1;
}

int Class_LSM6DSV16X::getAccelFS()
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL8, 1, &reg) < 0)
    {
        return -1;
    }
    return reg & 0x03;
}

// ======================== Gyro FS ========================

int Class_LSM6DSV16X::setGyroFS(GyroFS fssel)
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL6, 1, &reg) < 0)
    {
        return -1;
    }

    // 修改 fs_g 位 [3:0]
    reg = (reg & 0xF0) | (fssel & 0x0F);

    if (writeRegister(LSM6DSV16X_CTRL6, reg) < 0)
    {
        return -2;
    }

    _gyroFS = fssel;

    // LSM6DSV16X gyro 灵敏度: 换算为 dps/LSB
    switch (fssel)
    {
        case dps125:   _gyroScale = 0.003828125f; break;
        case dps250:   _gyroScale = 0.00765625f;  break;
        case dps500:   _gyroScale = 0.0153125f;   break;
        case dps1000:  _gyroScale = 0.030625f;    break;
        case dps2000:  _gyroScale = 0.061f;       break;
        case dps4000:  _gyroScale = 0.122f;       break;
        default:       _gyroScale = 0.061f;       break;
    }

    return 1;
}

int Class_LSM6DSV16X::getGyroFS()
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL6, 1, &reg) < 0)
    {
        return -1;
    }
    return reg & 0x0F;
}

// ======================== Accel ODR ========================

int Class_LSM6DSV16X::setAccelODR(ODR odr)
{
    // LSM6DSV16X CTRL1: odr_xl[3:0] + op_mode_xl[6:4]
    // 始终使用 high-performance mode (op_mode_xl = 000)
    if (writeRegister(LSM6DSV16X_CTRL1, odr & 0x0F) < 0)
    {
        return -1;
    }
    return 1;
}

int Class_LSM6DSV16X::getAccelODR()
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL1, 1, &reg) < 0)
    {
        return -1;
    }
    return reg & 0x0F;
}

// ======================== Gyro ODR ========================

int Class_LSM6DSV16X::setGyroODR(ODR odr)
{
    // LSM6DSV16X CTRL2: odr_g[3:0] + op_mode_g[6:4]
    // 始终使用 high-performance mode (op_mode_g = 000)
    if (writeRegister(LSM6DSV16X_CTRL2, odr & 0x0F) < 0)
    {
        return -1;
    }
    return 1;
}

int Class_LSM6DSV16X::getGyroODR()
{
    uint8_t reg;
    if (readRegisters(LSM6DSV16X_CTRL2, 1, &reg) < 0)
    {
        return -1;
    }
    return reg & 0x0F;
}

// ======================== Data Read ========================

int Class_LSM6DSV16X::Get_Data()
{
    if (Get_Raw_Data() < 0)
    {
        return -1;
    }

    Temperature = (static_cast<float>(_rawT) / TEMP_SCALE) + TEMP_OFFSET;

    Accel[0] = (_rawAcc[0] * _accelScale) - _accB[0];
    Accel[1] = (_rawAcc[1] * _accelScale) - _accB[1];
    Accel[2] = (_rawAcc[2] * _accelScale) - _accB[2];

    Gyro[0] = (_rawGyr[0] * _gyroScale) - _gyrB[0];
    Gyro[1] = (_rawGyr[1] * _gyroScale) - _gyrB[1];
    Gyro[2] = (_rawGyr[2] * _gyroScale) - _gyrB[2];

    return 1;
}

int Class_LSM6DSV16X::Get_Raw_Data()
{
    // 从 0x20 开始连续读 14 字节: TEMP(2) + GYRO(6) + ACCEL(6)
    if (readRegisters(LSM6DSV16X_OUT_TEMP_L, 14, _buffer) < 0)
    {
        return -1;
    }

    // LSM6DSV16X 数据寄存器为小端序 (L 在前, H 在后)
    _rawT   = (int16_t)((_buffer[1] << 8) | _buffer[0]);
    _rawGyr[0] = (int16_t)((_buffer[3] << 8) | _buffer[2]);
    _rawGyr[1] = (int16_t)((_buffer[5] << 8) | _buffer[4]);
    _rawGyr[2] = (int16_t)((_buffer[7] << 8) | _buffer[6]);
    _rawAcc[0] = (int16_t)((_buffer[9] << 8) | _buffer[8]);
    _rawAcc[1] = (int16_t)((_buffer[11] << 8) | _buffer[10]);
    _rawAcc[2] = (int16_t)((_buffer[13] << 8) | _buffer[12]);

    return 1;
}

// ======================== Gyro Calibration ========================

int Class_LSM6DSV16X::calibrateGyro()
{
    const GyroFS current_fssel = _gyroFS;
    if (setGyroFS(current_fssel) < 0)
    {
        return -1;
    }

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

    if (setGyroFS(current_fssel) < 0)
    {
        return -4;
    }
    return 1;
}

float Class_LSM6DSV16X::getGyroBiasX() { return _gyrB[0]; }
float Class_LSM6DSV16X::getGyroBiasY() { return _gyrB[1]; }
float Class_LSM6DSV16X::getGyroBiasZ() { return _gyrB[2]; }

void Class_LSM6DSV16X::setGyroBiasX(float bias) { _gyrB[0] = bias; }
void Class_LSM6DSV16X::setGyroBiasY(float bias) { _gyrB[1] = bias; }
void Class_LSM6DSV16X::setGyroBiasZ(float bias) { _gyrB[2] = bias; }

// ======================== Accel Calibration ========================

int Class_LSM6DSV16X::calibrateAccel()
{
    const AccelFS current_fssel = _accelFS;
    if (setAccelFS(gpm2) < 0)
    {
        return -1;
    }

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
    if (_accBD[0] > 0.9f)  _accMax[0] = _accBD[0];
    if (_accBD[1] > 0.9f)  _accMax[1] = _accBD[1];
    if (_accBD[2] > 0.9f)  _accMax[2] = _accBD[2];
    if (_accBD[0] < -0.9f) _accMin[0] = _accBD[0];
    if (_accBD[1] < -0.9f) _accMin[1] = _accBD[1];
    if (_accBD[2] < -0.9f) _accMin[2] = _accBD[2];

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

    if (setAccelFS(current_fssel) < 0)
    {
        return -4;
    }
    return 1;
}

float Class_LSM6DSV16X::getAccelBiasX_mss() { return _accB[0]; }
float Class_LSM6DSV16X::getAccelScaleFactorX() { return _accS[0]; }
float Class_LSM6DSV16X::getAccelBiasY_mss() { return _accB[1]; }
float Class_LSM6DSV16X::getAccelScaleFactorY() { return _accS[1]; }
float Class_LSM6DSV16X::getAccelBiasZ_mss() { return _accB[2]; }
float Class_LSM6DSV16X::getAccelScaleFactorZ() { return _accS[2]; }

void Class_LSM6DSV16X::setAccelCalX(float bias, float scaleFactor) { _accB[0] = bias; _accS[0] = scaleFactor; }
void Class_LSM6DSV16X::setAccelCalY(float bias, float scaleFactor) { _accB[1] = bias; _accS[1] = scaleFactor; }
void Class_LSM6DSV16X::setAccelCalZ(float bias, float scaleFactor) { _accB[2] = bias; _accS[2] = scaleFactor; }

// ======================== Resolution ========================

float Class_LSM6DSV16X::getAccelRes()
{
    int currentAccFS = getAccelFS();
    float accRes;
    switch (currentAccFS)
    {
        case gpm2:  accRes = 2.0f / 32768.0f;  break;
        case gpm4:  accRes = 4.0f / 32768.0f;  break;
        case gpm8:  accRes = 8.0f / 32768.0f;  break;
        case gpm16: accRes = 16.0f / 32768.0f; break;
        default:    accRes = 8.0f / 32768.0f;  break;
    }
    return accRes;
}

float Class_LSM6DSV16X::getGyroRes()
{
    int currentGyroFS = getGyroFS();
    float gyroRes;
    switch (currentGyroFS)
    {
        case dps125:   gyroRes = 125.0f / 32768.0f;   break;
        case dps250:   gyroRes = 250.0f / 32768.0f;   break;
        case dps500:   gyroRes = 500.0f / 32768.0f;   break;
        case dps1000:  gyroRes = 1000.0f / 32768.0f;  break;
        case dps2000:  gyroRes = 2000.0f / 32768.0f;  break;
        case dps4000:  gyroRes = 4000.0f / 32768.0f;  break;
        default:       gyroRes = 2000.0f / 32768.0f;  break;
    }
    return gyroRes;
}

// ======================== Offset (保留框架, LSM6DSV16X 无硬件 offset 寄存器) ========================

int Class_LSM6DSV16X::computeOffsets()
{
    // LSM6DSV16X 没有与 ICM42688 相同的 OFFSET_USER 硬件寄存器
    // 此处的零偏通过软件 _gyrB[] / _accB[] 完成, 返回 1 保持 API 兼容
    return 1;
}

int Class_LSM6DSV16X::setAllOffsets() { return 1; }
int Class_LSM6DSV16X::setGyrXOffset(int16_t) { return 1; }
int Class_LSM6DSV16X::setGyrYOffset(int16_t) { return 1; }
int Class_LSM6DSV16X::setGyrZOffset(int16_t) { return 1; }
int Class_LSM6DSV16X::setAccXOffset(int16_t) { return 1; }
int Class_LSM6DSV16X::setAccYOffset(int16_t) { return 1; }
int Class_LSM6DSV16X::setAccZOffset(int16_t) { return 1; }
