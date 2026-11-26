/**
 * @file    stm32g474_basic.c
 * @brief   STM32G474 使用示例：初始化 ICM-42688-P 并通过串口输出数据
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 */

#include <stdio.h>
#include <stdbool.h>

#include "icm42688.h"
#include "icm42688_hal.h"

/*
 * STM32G474 最小示例：每 10 ms 读一次，串口打印加速度/角速度。
 *
 * 接线（按你的板子改）：
 *   SPI1_SCK  -> PA5
 *   SPI1_MISO -> PA6
 *   SPI1_MOSI -> PA7
 *   CS        -> PA4（软件片选）
 *   INT1/INT2 -> 可选，用于数据就绪中断
 */

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

static icm_t        g_icm;
static icm_hal_spi_t g_bus;

static void app_delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    /* HAL_Delay() 只能按毫秒延时，向上取整以满足最小等待时间。 */
    HAL_Delay((us + 999u) / 1000u);
}

/* printf 重定向到 USART1 */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, 100);
    return ch;
}

/**
 * @brief  初始化 SPI 适配层与 ICM-42688-P，并设置采样参数
 * @note   初始化内部处理软复位和寄存器写入间隔，陀螺仪启动仍需额外等待 50 ms。
 */
void app_imu_start(void)
{
    icm_hal_spi_bind(&g_bus, &hspi1, GPIOA, GPIO_PIN_4, 100);

    icm_status_t st = icm_init(&g_icm, icm_hal_spi_read, icm_hal_spi_write,
                               app_delay_us, &g_bus);
    if (st != ICM_OK) {
        printf("icm_init failed: %d\r\n", (int)st);
        return;
    }

    /* 陀螺仪上电后需要约 50 ms 才能输出有效数据；本库不做阻塞延时 */
    HAL_Delay(50);

    (void)icm_set_accel_fs(&g_icm, ICM_ACCEL_FS_8G);
    (void)icm_set_gyro_fs(&g_icm, ICM_GYRO_FS_1000DPS);
    (void)icm_set_odr(&g_icm, ICM_ODR_100HZ);
    printf("icm42688 ready\r\n");
}

/**
 * @brief  轮询一次数据：有新数据则读取并打印加速度、角速度与温度
 * @note   在主循环中以 10 ms 周期调用即可
 */
void app_imu_poll(void)
{
    bool ready = false;
    if (icm_data_ready(&g_icm, &ready) != ICM_OK || !ready) {
        return;
    }

    icm_scaled_t s;
    float temp = 0.0f;
    if (icm_read_scaled(&g_icm, &s) != ICM_OK) {
        return;
    }
    (void)icm_read_temperature(&g_icm, &temp);

    printf("A[g] %+6.3f %+6.3f %+6.3f | G[dps] %+7.2f %+7.2f %+7.2f | T %.1f C\r\n",
           s.accel_g[0], s.accel_g[1], s.accel_g[2],
           s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2],
           temp);
}

/* 在 main() 的 while(1) 里调用：
 *
 *   app_imu_start();
 *   while (1) {
 *       app_imu_poll();
 *       HAL_Delay(10);
 *   }
 */
