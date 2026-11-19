/**
 * @file drv_spi.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief 仿照SCUT-Robotlab改写的SPI通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 24赛季定稿
 *
 * @copyright RoboPioneer (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_spi.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_SPI_Manage_Object SPI1_Manage_Object = {0};
Struct_SPI_Manage_Object SPI2_Manage_Object = {0};
Struct_SPI_Manage_Object SPI3_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化SPI
 *
 * @param hspi SPI编号
 * @param Callback_Function 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back Callback_Function)
{
    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.SPI_Handler = hspi;
        SPI1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.SPI_Handler = hspi;
        SPI2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.SPI_Handler = hspi;
        SPI3_Manage_Object.Callback_Function = Callback_Function;
    }
}

/**
 * @brief 交互数据帧
 * 
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Send_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t Tx_Length, uint16_t Rx_Length)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);

    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.Now_GPIOx = GPIOx;
        SPI1_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI1_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI1_Manage_Object.Now_Rx_Length = Rx_Length;

        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI1_Manage_Object.Tx_Buffer, SPI1_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.Now_GPIOx = GPIOx;
        SPI2_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI2_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI2_Manage_Object.Now_Rx_Length = Rx_Length;

        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI2_Manage_Object.Tx_Buffer, SPI2_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.Now_GPIOx = GPIOx;
        SPI3_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI3_Manage_Object.Now_Tx_Length = Tx_Length;
        SPI3_Manage_Object.Now_Rx_Length = Rx_Length;

        return (HAL_SPI_TransmitReceive_DMA(hspi, SPI3_Manage_Object.Tx_Buffer, SPI3_Manage_Object.Rx_Buffer, Tx_Length + Rx_Length));
    }
		return 1;
}

/**
 * @brief SPI的TIM定时器中断交互回调函数
 * 
 */
void TIM_100us_SPI_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库SPI交互DMA中断
 * 
 * @param hspi SPI编号
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 判断程序初始化完成
    if (init_finished == 0)
    {
        return;
    }

    // 选择回调函数
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_TogglePin(SPI1_Manage_Object.Now_GPIOx, SPI1_Manage_Object.Now_GPIO_Pin);
        if(SPI1_Manage_Object.Callback_Function != nullptr)
        {
            SPI1_Manage_Object.Callback_Function(SPI1_Manage_Object.Tx_Buffer, SPI1_Manage_Object.Rx_Buffer, SPI1_Manage_Object.Now_Tx_Length);
        }
    }
    else if (hspi->Instance == SPI2)
    {
        HAL_GPIO_TogglePin(SPI2_Manage_Object.Now_GPIOx, SPI2_Manage_Object.Now_GPIO_Pin);
        if(SPI2_Manage_Object.Callback_Function != nullptr)
        {
            SPI2_Manage_Object.Callback_Function(SPI2_Manage_Object.Tx_Buffer, SPI2_Manage_Object.Rx_Buffer, SPI2_Manage_Object.Now_Tx_Length);
        }
    }
    else if (hspi->Instance == SPI3)
    {
        HAL_GPIO_TogglePin(SPI3_Manage_Object.Now_GPIOx, SPI3_Manage_Object.Now_GPIO_Pin);
        if(SPI3_Manage_Object.Callback_Function != nullptr)
        {
            SPI3_Manage_Object.Callback_Function(SPI3_Manage_Object.Tx_Buffer, SPI3_Manage_Object.Rx_Buffer, SPI3_Manage_Object.Now_Tx_Length);
        }
    }
}

/************************ COPYRIGHT(C) ROBOPIONEER **************************/

HAL_StatusTypeDef SPI_TransferWithCS(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin,
                                      const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    if (!hspi || !port || !tx || !rx || size == 0U)
        return HAL_ERROR;
    if (hspi->State != HAL_SPI_STATE_READY)
        return HAL_BUSY;
    __HAL_LOCK(hspi);
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

    SPI_TypeDef *spi = hspi->Instance;
    MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, size);
    __HAL_SPI_ENABLE(hspi);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    volatile uint8_t *tx_fifo = reinterpret_cast<volatile uint8_t *>(&spi->TXDR);
    volatile uint8_t *rx_fifo = reinterpret_cast<volatile uint8_t *>(&spi->RXDR);
    *tx_fifo = tx[0];
    SET_BIT(spi->CR1, SPI_CR1_CSTART);
    uint16_t sent = 1U;
    uint16_t received = 0U;
    // DWT keeps counting while TIM2 preempts SysTick.
    const uint32_t started = DWT->CYCCNT;
    const uint32_t timeout_cycles = SystemCoreClock / 1000U;
    HAL_StatusTypeDef result = HAL_OK;
    for (;;)
    {
        const uint32_t flags = spi->SR;
        if (flags & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_TIFRE | SPI_SR_UDR))
        {
            result = HAL_ERROR;
            break;
        }
        if (received < size && (flags & SPI_SR_RXP))
            rx[received++] = *rx_fifo;
        if (sent < size && (flags & SPI_SR_TXP))
            *tx_fifo = tx[sent++];
        if (received == size && (flags & SPI_SR_EOT))
            break;
        if (DWT->CYCCNT - started >= timeout_cycles)
        {
            result = HAL_TIMEOUT;
            break;
        }
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    __HAL_SPI_DISABLE(hspi);
    spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC |
                SPI_IFCR_MODFC | SPI_IFCR_TIFREC | SPI_IFCR_UDRC;
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);
    return result;
}
