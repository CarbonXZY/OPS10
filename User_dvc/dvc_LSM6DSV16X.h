#ifndef DVC_LSM6DSV16X_H
#define DVC_LSM6DSV16X_H

#include "drv_spi.h"
#include "stm32h5xx_hal.h"
#include "dvc_LSM6DSV16X_reg.h"

#define delay HAL_Delay

/**
 * @brief LSM6DSV16X 6轴IMU驱动类
 *
 *        支持加速度计、陀螺仪和温度数据读取与校准
 *        
 */
class Class_LSM6DSV16X
{
public:
    /// 陀螺仪量程选择
    enum GyroFS : uint8_t
    {
        dps125 = 0x00,
        dps250 = 0x01,
        dps500 = 0x02,
        dps1000 = 0x03,
        dps2000 = 0x04,
        dps4000 = 0x0C
    };

    /// 加速度计量程选择
    enum AccelFS : uint8_t
    {
        gpm2 = 0x00,
        gpm4 = 0x01,
        gpm8 = 0x02,
        gpm16 = 0x03
    };

    /// 输出数据速率 (ODR 编码值 0x0-0xC, 对应 CTRL1/CTRL2 的 odr 字段)
    enum ODR : uint8_t
    {
        odrOff = 0x00,
        odr1Hz875 = 0x01,
        odr7Hz5 = 0x02,
        odr15Hz = 0x03,
        odr30Hz = 0x04,
        odr60Hz = 0x05,
        odr120Hz = 0x06,
        odr240Hz = 0x07,
        odr480Hz = 0x08,
        odr960Hz = 0x09,
        odr1920Hz = 0x0A,
        odr3840Hz = 0x0B,
        odr7680Hz = 0x0C
    };

    int Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

    int setAccelFS(AccelFS fssel);
    int getAccelFS();
    int setGyroFS(GyroFS fssel);
    int getGyroFS();
    int setAccelODR(ODR odr);
    int getAccelODR();
    int setGyroODR(ODR odr);
    int getGyroODR();

    int Get_Data();
    int Get_Raw_Data();

    inline float Get_AccX() const { return Accel[0]; }
    inline float Get_AccY() const { return Accel[1]; }
    inline float Get_AccZ() const { return Accel[2]; }
    inline float Get_GyrX() const { return Gyro[0]; }
    inline float Get_GyrY() const { return Gyro[1]; }
    inline float Get_GyrZ() const { return Gyro[2]; }
    inline float Get_Temperature() const { return Temperature; }

    inline int16_t Get_RawAccX() const { return _rawAcc[0]; }
    inline int16_t Get_RawAccY() const { return _rawAcc[1]; }
    inline int16_t Get_RawAccZ() const { return _rawAcc[2]; }
    inline int16_t Get_RawGyrX() const { return _rawGyr[0]; }
    inline int16_t Get_RawGyrY() const { return _rawGyr[1]; }
    inline int16_t Get_RawGyrZ() const { return _rawGyr[2]; }
    inline int16_t Get_RawTemp() const { return _rawT; }

    inline int32_t Get_RawAccBiasX() const { return _rawAccBias[0]; }
    inline int32_t Get_RawAccBiasY() const { return _rawAccBias[1]; }
    inline int32_t Get_RawAccBiasZ() const { return _rawAccBias[2]; }
    inline int32_t Get_RawGyrBiasX() const { return _rawGyrBias[0]; }
    inline int32_t Get_RawGyrBiasY() const { return _rawGyrBias[1]; }
    inline int32_t Get_RawGyrBiasZ() const { return _rawGyrBias[2]; }

    int computeOffsets();
    int setAllOffsets();

    int setGyrXOffset(int16_t gyrXoffset);
    int setGyrYOffset(int16_t gyrYoffset);
    int setGyrZOffset(int16_t gyrZoffset);
    int setAccXOffset(int16_t accXoffset);
    int setAccYOffset(int16_t accYoffset);
    int setAccZOffset(int16_t accZoffset);

    float getAccelRes();
    float getGyroRes();

    int calibrateGyro();
    float getGyroBiasX();
    float getGyroBiasY();
    float getGyroBiasZ();
    void setGyroBiasX(float bias);
    void setGyroBiasY(float bias);
    void setGyroBiasZ(float bias);

    int calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias, float scaleFactor);
    void setAccelCalY(float bias, float scaleFactor);
    void setAccelCalZ(float bias, float scaleFactor);

protected:
    uint8_t SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

    SPI_HandleTypeDef *_hspi = NULL;
    GPIO_TypeDef *_cs_port = NULL;
    uint16_t _cs_pin = 0;

    uint8_t _buffer[16] = {};

    float Temperature = 0.0f;
    float Accel[3] = {};
    float Gyro[3] = {};

    int16_t _rawT = 0;
    int16_t _rawAcc[3] = {};
    int16_t _rawGyr[3] = {};

    int32_t _rawAccBias[3] = {0, 0, 0};
    int32_t _rawGyrBias[3] = {0, 0, 0};

    int16_t _AccOffset[3] = {0, 0, 0};
    int16_t _GyrOffset[3] = {0, 0, 0};

    float _accelScale = 0.0f;
    float _gyroScale = 0.0f;

    AccelFS _accelFS = gpm16;
    GyroFS _gyroFS = dps2000;

    float _accBD[3] = {};
    float _accB[3] = {};
    float _accS[3] = {1.0f, 1.0f, 1.0f};
    float _accMax[3] = {};
    float _accMin[3] = {};

    float _gyroBD[3] = {};
    float _gyrB[3] = {};

    static constexpr uint8_t WHO_AM_I = LSM6DSV16X_WHO_AM_I_VAL;
    static constexpr int NUM_CALIB_SAMPLES = 1000;

    /// LSM6DSV16X 温度: 原始值 / 256.0 + 25.0 degC
    static constexpr float TEMP_SCALE = 256.0f;
    static constexpr float TEMP_OFFSET = 25.0f;

protected:
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
    void reset();
    int8_t whoAmI();
};

#endif // DVC_LSM6DSV16X_H
