/**
 * @file    icm42688_hal.h
 * @brief   ICM-42688-P 驱动与 STM32 HAL 的适配层 —— 接口
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 *
 * @details
 * 只有使用 STM32 HAL 时才需要编译本文件；纯 C 环境可以只用 src/ 下的驱动。
 *
 * 用法：
 * @code
 *   icm_hal_spi_t bus;
 *   icm_hal_spi_bind(&bus, &hspi1, GPIOA, GPIO_PIN_4, 100);
 *   icm_init(&imu, icm_hal_spi_read, icm_hal_spi_write, delay_us, &bus);
 * @endcode
 */
#ifndef ICM42688_HAL_H
#define ICM42688_HAL_H

#include "icm42688.h"
#include "main.h"   /* 提供 GPIO_TypeDef / SPI_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPI 适配实例，作为 icm_init() 的 ctx 传入
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /**< 已初始化的 SPI 句柄 */
    GPIO_TypeDef      *cs_port;     /**< 片选所在端口，如 GPIOA */
    uint16_t           cs_pin;      /**< 片选引脚，如 GPIO_PIN_4 */
    uint32_t           timeout_ms;  /**< HAL 收发超时，单位 ms；为 0 时取 100 */
} icm_hal_spi_t;

/**
 * @brief  绑定 SPI 句柄与片选引脚
 *
 * @param bus        适配实例，由调用方分配
 * @param hspi       已初始化的 SPI 句柄
 * @param cs_port    片选端口
 * @param cs_pin     片选引脚
 * @param timeout_ms 收发超时（ms），传 0 使用默认 100
 *
 * @note 绑定后把 bus 作为 ctx、把下面两个函数作为总线回调传给 icm_init()，
 *       另提供一个平台相关的延时回调。
 */
void icm_hal_spi_bind(icm_hal_spi_t *bus, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t timeout_ms);

/**
 * @brief  SPI 读回调，供 icm_bus_read_fn 使用
 *
 * @param ctx 指向 icm_hal_spi_t
 * @param reg 起始寄存器
 * @param buf 输出缓冲区
 * @param len 字节数
 * @return 0 成功，-1 失败
 */
int icm_hal_spi_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  SPI 写回调，供 icm_bus_write_fn 使用
 *
 * @param ctx 指向 icm_hal_spi_t
 * @param reg 起始寄存器
 * @param buf 待写数据
 * @param len 字节数
 * @return 0 成功，-1 失败
 */
int icm_hal_spi_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_HAL_H */
