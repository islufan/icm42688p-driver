# ICM-42688-P 驱动

[English](README.md) | **简体中文**

[![ci](https://github.com/islufan/icm42688p-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/islufan/icm42688p-driver/actions/workflows/ci.yml)

面向嵌入式 C99 的 ICM-42688-P 六轴 IMU 驱动。驱动本体不依赖 STM32 HAL、ESP-IDF 或其他平台 SDK，总线和延时通过回调提供。

## 特性

- SPI 或 I2C 总线适配
- 加速度计和陀螺仪量程、ODR 配置
- 原始数据换算为 g、dps 和摄氏温度
- 六轴 12 字节连续读取
- 状态码错误处理
- 不需要硬件的 mock 单元测试

## 项目结构

```text
src/       平台无关驱动
port/      STM32 HAL 适配层
examples/  STM32G474 示例
tests/     mock 单元测试
```

## 快速开始

总线需要提供读、写和延时三个回调。SPI 读事务保持 CS 有效，先发送 `reg | 0x80`，再连续读取数据。SPI 写事务发送 `reg & 0x7F`，再连续写入数据。

`delay_us` 用于软复位后的 1 ms 等待和传感器使能后的 200 μs 保护时间。陀螺仪使能后还需要等待约 45 ms 才能读取有效数据。

```c
static void delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    platform_delay_us(us);
}

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* SPI 示例。I2C 适配只需遵循相同的回调约定。 */
    (void)ctx;
    uint8_t address = (uint8_t)(reg | 0x80u);
    cs_low();
    spi_transfer(&address, 1, NULL, 0);
    spi_transfer(NULL, 0, buf, len);
    cs_high();
    return 0;
}

static int bus_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)ctx;
    uint8_t address = (uint8_t)(reg & 0x7Fu);
    cs_low();
    spi_transfer(&address, 1, NULL, 0);
    spi_transfer(buf, len, NULL, 0);
    cs_high();
    return 0;
}

icm_t imu;
if (icm_init(&imu, bus_read, bus_write, delay_us, NULL) == ICM_OK) {
    platform_delay_ms(50);

    icm_scaled_t data;
    if (icm_read_scaled(&imu, &data) == ICM_OK) {
        /* data.accel_g[0..2]，data.gyro_dps[0..2] */
    }
}
```

初始化默认使用 ±16 g、±2000 dps、1 kHz 和低噪声模式。配置可以在初始化后调整：

```c
icm_set_accel_fs(&imu, ICM_ACCEL_FS_4G);
icm_set_gyro_fs(&imu, ICM_GYRO_FS_500DPS);
icm_set_odr(&imu, ICM_ODR_200HZ);
```

STM32 HAL 适配层位于 `port/`，使用时将 `icm_hal_spi_bind()` 提供的总线实例作为 `ctx` 传入，并确保 STM32Cube 工程的 `main.h` 和 HAL 头文件在包含路径中。

## API

| 函数 | 作用 |
| :--- | :--- |
| `icm_init()` | 复位、检查 WHO_AM_I、写入默认配置并使能传感器 |
| `icm_soft_reset()` | 软复位并清除初始化状态 |
| `icm_set_accel_fs()` | 设置加速度计量程 |
| `icm_set_gyro_fs()` | 设置陀螺仪量程 |
| `icm_set_odr()` | 同时设置加速度计和陀螺仪 ODR |
| `icm_enable()` | 以低噪声模式使能两颗传感器 |
| `icm_read_raw()` | 读取六轴原始码值 |
| `icm_read_scaled()` | 读取并换算六轴数据 |
| `icm_read_temperature()` | 读取芯片温度 |
| `icm_data_ready()` | 查询数据就绪状态 |
| `icm_read_reg()` / `icm_write_reg()` | 直接访问寄存器 |
| `icm_set_bank()` | 切换寄存器 Bank，范围为 0~4 |

状态码包括 `ICM_OK`、`ICM_ERR_PARAM`、`ICM_ERR_BUS`、`ICM_ERR_ID` 和 `ICM_ERR_STATE`。

## 配置

加速度计量程和灵敏度如下：

| 枚举 | 量程 | 灵敏度 |
| :--- | :--- | :--- |
| `ICM_ACCEL_FS_16G` | ±16 g | 2048 LSB/g |
| `ICM_ACCEL_FS_8G` | ±8 g | 4096 LSB/g |
| `ICM_ACCEL_FS_4G` | ±4 g | 8192 LSB/g |
| `ICM_ACCEL_FS_2G` | ±2 g | 16384 LSB/g |

陀螺仪量程和灵敏度如下：

| 枚举 | 量程 | 灵敏度 |
| :--- | :--- | :--- |
| `ICM_GYRO_FS_2000DPS` | ±2000 dps | 16.4 LSB/dps |
| `ICM_GYRO_FS_1000DPS` | ±1000 dps | 32.8 LSB/dps |
| `ICM_GYRO_FS_500DPS` | ±500 dps | 65.5 LSB/dps |
| `ICM_GYRO_FS_250DPS` | ±250 dps | 131 LSB/dps |

`icm_set_odr()` 同时配置两颗传感器，可用 ODR 如下：

| 枚举 | ODR | 寄存器值 |
| :--- | :--- | :--- |
| `ICM_ODR_32KHZ` | 32 kHz | `0x01` |
| `ICM_ODR_16KHZ` | 16 kHz | `0x02` |
| `ICM_ODR_8KHZ` | 8 kHz | `0x03` |
| `ICM_ODR_4KHZ` | 4 kHz | `0x04` |
| `ICM_ODR_2KHZ` | 2 kHz | `0x05` |
| `ICM_ODR_1KHZ` | 1 kHz | `0x06` |
| `ICM_ODR_200HZ` | 200 Hz | `0x07` |
| `ICM_ODR_100HZ` | 100 Hz | `0x08` |
| `ICM_ODR_50HZ` | 50 Hz | `0x09` |
| `ICM_ODR_25HZ` | 25 Hz | `0x0A` |
| `ICM_ODR_12_5HZ` | 12.5 Hz | `0x0B` |
| `ICM_ODR_500HZ` | 500 Hz | `0x0F` |

`0x0C`、`0x0D` 和 `0x0E` 是陀螺仪保留值，因此驱动会拒绝这些编码。加速度计在低功耗模式下支持更低的 ODR，但本接口要求两颗传感器使用同一 ODR。

数据换算公式如下：

```text
accel_g  = accel_raw / accel_lsb_per_g
gyro_dps = gyro_raw  / gyro_lsb_per_dps
temp_c   = temp_raw  / 132.48 + 25
```

## 构建和测试

```bash
cmake -S . -B build -DICM_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

作为子目录集成时，测试默认关闭：

```cmake
add_subdirectory(icm42688)
target_link_libraries(my_app PRIVATE icm42688)
```

## 常见问题

- `WHO_AM_I` 应读到 `0x47`。读不到时检查 CS、SPI Mode 0 或 Mode 3、MISO 和供电。
- 使能后立即读取可能得到无效数据。等待约 45 ms，并使用 `icm_data_ready()` 判断是否有新数据。
- 温度寄存器使用 `132.48` 的灵敏度。`2.07` 只适用于 FIFO 中的 8 位温度字段。

## 许可

MIT
