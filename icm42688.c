#include "icm42688.h"
#include "spi.h"

#define ICM42688_CS_PIN GPIO_PIN_12
#define ICM42688_CS_PORT GPIOB

// ICM-42688-P寄存器定义 (Bank 0)
#define WHO_AM_I_REG       0x75
#define PWR_MGMT0_REG      0x4E
#define GYRO_CONFIG0_REG   0x4F
#define ACCEL_CONFIG0_REG  0x50
#define INTF_CONFIG0_REG   0x4C
#define DEVICE_CONFIG_REG  0x11


void ICM42688_Init(void) {
    uint8_t whoami;

    // 重新配置CS引脚为GPIO输出（覆盖SpiMspInit的AF模式）
   // ICM42688_CS_Init();
		HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);

    // 配置为大端序（与读取代码一致，保留复位值bit7）
    ICM42688_WriteRegister(INTF_CONFIG0_REG, 0x90);
    HAL_Delay(10);

    // 配置电源管理：6轴低噪声模式
    ICM42688_WriteRegister(PWR_MGMT0_REG, 0x0F);

    // 检查WHO_AM_I
    whoami = ICM42688_ReadRegister(WHO_AM_I_REG);
    if(whoami != 0x47) {
        while(1);
    }

    // 配置陀螺仪：±500dps, 1kHz ODR
    ICM42688_WriteRegister(GYRO_CONFIG0_REG, (0x02 << 5) | 0x06);

    // 配置加速度计：±8g, 1kHz ODR
    ICM42688_WriteRegister(ACCEL_CONFIG0_REG, (0x01 << 5) | 0x06);

    HAL_Delay(100);
}

void ICM42688_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t tx_buffer[2] = {reg & 0x7F, data};

    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
}

uint8_t ICM42688_ReadRegister(uint8_t reg) {
    uint8_t tx_buffer[2] = {reg | 0x80, 0x00};
    uint8_t rx_buffer[2] = {0};

    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);

    return rx_buffer[1];
}

void ReadSensorData(int16_t* accel, int16_t* gyro) {
    accel[0] = (int16_t)((ICM42688_ReadRegister(0x1F) << 8) | ICM42688_ReadRegister(0x20));
    accel[1] = (int16_t)((ICM42688_ReadRegister(0x21) << 8) | ICM42688_ReadRegister(0x22));
    accel[2] = (int16_t)((ICM42688_ReadRegister(0x23) << 8) | ICM42688_ReadRegister(0x24));

    gyro[0] = (int16_t)((ICM42688_ReadRegister(0x25) << 8) | ICM42688_ReadRegister(0x26));
    gyro[1] = (int16_t)((ICM42688_ReadRegister(0x27) << 8) | ICM42688_ReadRegister(0x28));
    gyro[2] = (int16_t)((ICM42688_ReadRegister(0x29) << 8) | ICM42688_ReadRegister(0x2A));
}

void ConvertRawData(int16_t raw_accel[3], int16_t raw_gyro[3], float* accel_g, float* gyro_dps) {
    for(int i = 0; i < 3; i++) {
        accel_g[i] = (float)raw_accel[i] / 4096.0f;
        gyro_dps[i] = (float)raw_gyro[i] / 65.5f;
    }
}

#define TEMP_SENSITIVITY 132.48f
#define TEMP_OFFSET      25.0f

float ICM42688_ReadTemperature(void) {
    int16_t temp_raw = (int16_t)((ICM42688_ReadRegister(0x1D) << 8) | ICM42688_ReadRegister(0x1E));
    return ((float)temp_raw / TEMP_SENSITIVITY) + TEMP_OFFSET;
}
