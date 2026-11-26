/**
 * @file    icm42688_hal.c
 * @brief   ICM-42688-P 驱动与 STM32 HAL 的适配层 —— 实现
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 *
 * @details
 * SPI 时序（4-wire 模式）：
 *   读：拉低 CS → 发送 (reg | 0x80) → 连续接收 len 字节 → 拉高 CS
 *   写：拉低 CS → 发送 (reg & 0x7F) → 连续发送 len 字节 → 拉高 CS
 *
 * @warning 整帧期间 CS 必须保持低电平，中途不要插入其他 SPI 事务或调用 HAL_SPI_Abort。
 */
#include "icm42688_hal.h"

/**
 * @brief  绑定 SPI 句柄与片选引脚
 * @param  bus        适配实例
 * @param  hspi       SPI 句柄
 * @param  cs_port    片选端口
 * @param  cs_pin     片选引脚
 * @param  timeout_ms 超时（ms），0 表示使用默认值 100
 */
void icm_hal_spi_bind(icm_hal_spi_t *bus, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t timeout_ms)
{
    if (bus == NULL) {
        return;
    }
    bus->hspi       = hspi;
    bus->cs_port    = cs_port;
    bus->cs_pin     = cs_pin;
    bus->timeout_ms = (timeout_ms == 0u) ? 100u : timeout_ms;
}

/**
 * @brief  拉低片选
 * @param  bus 适配实例
 */
static inline void cs_low(icm_hal_spi_t *bus)
{
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief  拉高片选
 * @param  bus 适配实例
 */
static inline void cs_high(icm_hal_spi_t *bus)
{
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET);
}

int icm_hal_spi_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    icm_hal_spi_t *bus = (icm_hal_spi_t *)ctx;
    if (bus == NULL || bus->hspi == NULL || bus->cs_port == NULL || buf == NULL || len == 0u) {
        return -1;
    }

    /* 最高位置 1 表示读操作 */
    uint8_t addr = (uint8_t)(reg | 0x80u);
    int rc = 0;

    cs_low(bus);
    if (HAL_SPI_Transmit(bus->hspi, &addr, 1, bus->timeout_ms) != HAL_OK) {
        rc = -1;
    } else if (HAL_SPI_Receive(bus->hspi, buf, len, bus->timeout_ms) != HAL_OK) {
        rc = -1;
    }
    cs_high(bus);   /* 无论成功失败都要释放片选，避免总线被占死 */

    return rc;
}

int icm_hal_spi_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    icm_hal_spi_t *bus = (icm_hal_spi_t *)ctx;
    if (bus == NULL || bus->hspi == NULL || bus->cs_port == NULL || buf == NULL || len == 0u) {
        return -1;
    }

    /* 最高位清 0 表示写操作 */
    uint8_t addr = (uint8_t)(reg & 0x7Fu);
    int rc = 0;

    cs_low(bus);
    if (HAL_SPI_Transmit(bus->hspi, &addr, 1, bus->timeout_ms) != HAL_OK) {
        rc = -1;
    } else if (HAL_SPI_Transmit(bus->hspi, (uint8_t *)buf, len, bus->timeout_ms) != HAL_OK) {
        rc = -1;
    }
    cs_high(bus);

    return rc;
}
