# ICM-42688-P Driver

[![ci](https://github.com/islufan/icm42688p-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/islufan/icm42688p-driver/actions/workflows/ci.yml)

**English** | [简体中文](README.zh-CN.md)

A C99 driver for the ICM-42688-P 6-axis IMU, written for bare-metal embedded targets. The driver does not depend on STM32 HAL, ESP-IDF or any other platform SDK: the bus and the delay are injected as callbacks.

## Features

- SPI or I2C bus adapter
- Accel and gyro full-scale / ODR configuration
- Raw data converted to g, dps and degrees Celsius
- 12-byte burst read of all six axes
- Status-code based error handling
- Mock-bus unit tests that need no hardware

## Layout

```text
src/       platform-independent driver
port/      STM32 HAL adapter
examples/  STM32G474 example
tests/     mock-bus unit tests
```

## Quick Start

The bus needs three callbacks: read, write and delay. An SPI read keeps CS asserted, sends `reg | 0x80`, then reads the payload. An SPI write sends `reg & 0x7F`, then the payload.

`delay_us` covers the 1 ms wait after a soft reset and the 200 µs guard time after enabling the sensors. The gyroscope additionally needs about 45 ms before its data is valid.

```c
static void delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    platform_delay_us(us);
}

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* SPI example. An I2C adapter just follows the same callback contract. */
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
        /* data.accel_g[0..2], data.gyro_dps[0..2] */
    }
}
```

Initialization defaults to ±16 g, ±2000 dps, 1 kHz and low-noise mode. Settings can be changed afterwards:

```c
icm_set_accel_fs(&imu, ICM_ACCEL_FS_4G);
icm_set_gyro_fs(&imu, ICM_GYRO_FS_500DPS);
icm_set_odr(&imu, ICM_ODR_200HZ);
```

The STM32 HAL adapter lives in `port/`. Pass the bus instance filled in by `icm_hal_spi_bind()` as `ctx`, and make sure `main.h` and the HAL headers of your STM32Cube project are on the include path.

## API

| Function | Purpose |
| :--- | :--- |
| `icm_init()` | Reset, verify WHO_AM_I, apply defaults and enable the sensors |
| `icm_soft_reset()` | Soft reset and clear the initialized state |
| `icm_set_accel_fs()` | Set the accelerometer full scale |
| `icm_set_gyro_fs()` | Set the gyroscope full scale |
| `icm_set_odr()` | Set the ODR of both sensors |
| `icm_enable()` | Enable both sensors in low-noise mode |
| `icm_read_raw()` | Read the raw 6-axis counts |
| `icm_read_scaled()` | Read and convert the 6-axis data |
| `icm_read_temperature()` | Read the die temperature |
| `icm_data_ready()` | Poll the data-ready flag |
| `icm_read_reg()` / `icm_write_reg()` | Direct register access |
| `icm_set_bank()` | Select a register bank, 0 to 4 |

Status codes: `ICM_OK`, `ICM_ERR_PARAM`, `ICM_ERR_BUS`, `ICM_ERR_ID`, `ICM_ERR_STATE`.

## Configuration

Accelerometer full scale and sensitivity:

| Enum | Range | Sensitivity |
| :--- | :--- | :--- |
| `ICM_ACCEL_FS_16G` | ±16 g | 2048 LSB/g |
| `ICM_ACCEL_FS_8G` | ±8 g | 4096 LSB/g |
| `ICM_ACCEL_FS_4G` | ±4 g | 8192 LSB/g |
| `ICM_ACCEL_FS_2G` | ±2 g | 16384 LSB/g |

Gyroscope full scale and sensitivity:

| Enum | Range | Sensitivity |
| :--- | :--- | :--- |
| `ICM_GYRO_FS_2000DPS` | ±2000 dps | 16.4 LSB/dps |
| `ICM_GYRO_FS_1000DPS` | ±1000 dps | 32.8 LSB/dps |
| `ICM_GYRO_FS_500DPS` | ±500 dps | 65.5 LSB/dps |
| `ICM_GYRO_FS_250DPS` | ±250 dps | 131 LSB/dps |

`icm_set_odr()` configures both sensors at once. Available ODR settings:

| Enum | ODR | Register value |
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

`0x0C`, `0x0D` and `0x0E` are reserved on the gyroscope, so the driver rejects them. The accelerometer supports lower rates in low-power mode, but this API requires a single ODR shared by both sensors.

Conversion formulas:

```text
accel_g  = accel_raw / accel_lsb_per_g
gyro_dps = gyro_raw  / gyro_lsb_per_dps
temp_c   = temp_raw  / 132.48 + 25
```

## Build and Test

```bash
cmake -S . -B build -DICM_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

When the library is added as a subdirectory, tests are off by default:

```cmake
add_subdirectory(icm42688p-driver)
target_link_libraries(my_app PRIVATE icm42688)
```

## Troubleshooting

- `WHO_AM_I` should read `0x47`. If it does not, check CS, SPI mode 0 or 3, MISO and the supply.
- Reading immediately after enabling can return invalid data. Wait about 45 ms and use `icm_data_ready()` to tell whether new data is available.
- The temperature register uses a sensitivity of `132.48`. The `2.07` figure only applies to the 8-bit temperature field in the FIFO.

## License

MIT
