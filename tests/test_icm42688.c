/**
 * @file    test_icm42688.c
 * @brief   驱动单元测试：用 mock 总线验证寄存器操作与数据换算，不需要真实硬件
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "icm42688.h"

/* ------------------------------------------------------------------ */
/* mock 总线                                                           */
/* ------------------------------------------------------------------ */
static uint8_t g_regs[256];
static int     g_bus_fail = 0;
static uint32_t g_delay_total_us = 0;

static void regs_reset(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    g_bus_fail = 0;
    g_delay_total_us = 0;
    g_regs[ICM_REG_WHO_AM_I] = ICM_WHO_AM_I_VALUE;
}

static void mock_delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    g_delay_total_us += us;
}

static int mock_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)ctx;
    if (g_bus_fail) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = g_regs[(uint8_t)(reg + i)];
    }
    return 0;
}

static int mock_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)ctx;
    if (g_bus_fail) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        g_regs[(uint8_t)(reg + i)] = buf[i];
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 极简测试框架                                                        */
/* ------------------------------------------------------------------ */
static int g_failures = 0;

#define CHECK(cond, name)                                     \
    do {                                                      \
        if (cond) {                                           \
            printf("  ok   : %s\n", name);                    \
        } else {                                              \
            printf("  FAIL : %s\n", name);                    \
            g_failures++;                                     \
        }                                                     \
    } while (0)

static icm_t make_dev(void)
{
    icm_t dev;
    memset(&dev, 0, sizeof(dev));
    (void)icm_init(&dev, mock_read, mock_write, mock_delay_us, NULL);
    return dev;
}

/* ------------------------------------------------------------------ */
/* 用例                                                                */
/* ------------------------------------------------------------------ */
/**
 * @brief 初始化成功路径：软复位、WHO_AM_I 校验、默认配置写入
 */
static void test_init_ok(void)
{
    printf("[test] init / WHO_AM_I\n");
    regs_reset();
    icm_t dev;
    memset(&dev, 0, sizeof(dev));
    icm_status_t st = icm_init(&dev, mock_read, mock_write, mock_delay_us, NULL);
    CHECK(st == ICM_OK, "init returns ICM_OK");
    CHECK(dev.ready, "dev.ready is true");
    CHECK(g_regs[ICM_REG_PWR_MGMT0] == ICM_PWR_MGMT0_LN_BOTH, "power mode = accel+gyro low noise");
    CHECK(g_delay_total_us >= 1200u, "reset and power-up guard delays were applied");
}

/**
 * @brief WHO_AM_I 不匹配时应返回 ICM_ERR_ID
 */
static void test_init_bad_id(void)
{
    printf("[test] init with wrong WHO_AM_I\n");
    regs_reset();
    g_regs[ICM_REG_WHO_AM_I] = 0x00;
    icm_t dev;
    memset(&dev, 0, sizeof(dev));
    icm_status_t st = icm_init(&dev, mock_read, mock_write, mock_delay_us, NULL);
    CHECK(st == ICM_ERR_ID, "returns ICM_ERR_ID");
    CHECK(!dev.ready, "dev.ready stays false");
}

/**
 * @brief 总线读写失败时应返回 ICM_ERR_BUS
 */
static void test_init_bus_error(void)
{
    printf("[test] init with failing bus\n");
    regs_reset();
    g_bus_fail = 1;
    icm_t dev;
    memset(&dev, 0, sizeof(dev));
    icm_status_t st = icm_init(&dev, mock_read, mock_write, mock_delay_us, NULL);
    CHECK(st == ICM_ERR_BUS, "returns ICM_ERR_BUS");
}

/**
 * @brief 加速度量程写入 bit[7:5]，且不破坏 ODR 位（回归用例）
 */
static void test_accel_fs_bits(void)
{
    printf("[test] accel full-scale lands on bit[7:5], ODR untouched\n");
    regs_reset();
    icm_t dev = make_dev();
    uint8_t odr_before = (uint8_t)(g_regs[ICM_REG_ACCEL_CONFIG0] & ICM_CONFIG0_ODR_MASK);

    icm_status_t st = icm_set_accel_fs(&dev, ICM_ACCEL_FS_2G);
    uint8_t v = g_regs[ICM_REG_ACCEL_CONFIG0];

    CHECK(st == ICM_OK, "set_accel_fs returns ICM_OK");
    CHECK(((v & ICM_CONFIG0_FS_MASK) >> ICM_CONFIG0_FS_SHIFT) == (uint8_t)ICM_ACCEL_FS_2G,
          "FS_SEL == 3 (±2g)");
    CHECK((v & ICM_CONFIG0_ODR_MASK) == odr_before, "ODR bits unchanged");
    CHECK(dev.accel_lsb_per_g == 16384.0f, "scale updated to 16384 LSB/g");

    /* 回归：旧实现改的是 bit[1:0]，会把 ODR 写坏 */
    CHECK((v & 0x03u) == (odr_before & 0x03u), "regression: low 2 bits (ODR) not clobbered");
}

/**
 * @brief 陀螺仪量程写入与灵敏度同步更新
 */
static void test_gyro_fs_bits(void)
{
    printf("[test] gyro full-scale\n");
    regs_reset();
    icm_t dev = make_dev();
    icm_status_t st = icm_set_gyro_fs(&dev, ICM_GYRO_FS_250DPS);
    uint8_t v = g_regs[ICM_REG_GYRO_CONFIG0];
    CHECK(st == ICM_OK, "set_gyro_fs returns ICM_OK");
    CHECK(((v & ICM_CONFIG0_FS_MASK) >> ICM_CONFIG0_FS_SHIFT) == (uint8_t)ICM_GYRO_FS_250DPS,
          "FS_SEL == 3 (±250 dps)");
    CHECK(dev.gyro_lsb_per_dps == 131.0f, "scale updated to 131 LSB/dps");
}

/**
 * @brief ODR 同时作用于两颗传感器，且保留各自量程
 */
static void test_odr(void)
{
    printf("[test] ODR applies to both sensors, FS preserved\n");
    regs_reset();
    icm_t dev = make_dev();
    (void)icm_set_accel_fs(&dev, ICM_ACCEL_FS_4G);
    (void)icm_set_gyro_fs(&dev, ICM_GYRO_FS_500DPS);

    icm_status_t st = icm_set_odr(&dev, ICM_ODR_100HZ);
    uint8_t a = g_regs[ICM_REG_ACCEL_CONFIG0];
    uint8_t g = g_regs[ICM_REG_GYRO_CONFIG0];

    CHECK(st == ICM_OK, "set_odr returns ICM_OK");
    CHECK((a & ICM_CONFIG0_ODR_MASK) == 0x08, "accel ODR == 100 Hz");
    CHECK((g & ICM_CONFIG0_ODR_MASK) == 0x08, "gyro ODR == 100 Hz");
    CHECK(((a & ICM_CONFIG0_FS_MASK) >> ICM_CONFIG0_FS_SHIFT) == (uint8_t)ICM_ACCEL_FS_4G,
          "accel FS preserved");
    CHECK(((g & ICM_CONFIG0_FS_MASK) >> ICM_CONFIG0_FS_SHIFT) == (uint8_t)ICM_GYRO_FS_500DPS,
          "gyro FS preserved");
}

/**
 * @brief 原始码值换算为 g / dps，含负号处理
 */
static void test_read_scaled(void)
{
    printf("[test] raw -> physical conversion\n");
    regs_reset();
    icm_t dev = make_dev();
    (void)icm_set_accel_fs(&dev, ICM_ACCEL_FS_16G);   /* 2048 LSB/g */
    (void)icm_set_gyro_fs(&dev, ICM_GYRO_FS_2000DPS); /* 16.4 LSB/dps */

    /* accel: 2048 -> 1 g, 1024 -> 0.5 g, -2048 -> -1 g */
    int16_t ax = 2048, ay = 1024, az = -2048;
    g_regs[0x1F] = (uint8_t)(ax >> 8); g_regs[0x20] = (uint8_t)(ax & 0xFF);
    g_regs[0x21] = (uint8_t)(ay >> 8); g_regs[0x22] = (uint8_t)(ay & 0xFF);
    g_regs[0x23] = (uint8_t)(az >> 8); g_regs[0x24] = (uint8_t)(az & 0xFF);
    /* gyro: 164 -> 10 dps */
    int16_t gx = 164, gy = 0, gz = -164;
    g_regs[0x25] = (uint8_t)(gx >> 8); g_regs[0x26] = (uint8_t)(gx & 0xFF);
    g_regs[0x27] = (uint8_t)(gy >> 8); g_regs[0x28] = (uint8_t)(gy & 0xFF);
    g_regs[0x29] = (uint8_t)(gz >> 8); g_regs[0x2A] = (uint8_t)(gz & 0xFF);

    icm_scaled_t s;
    icm_status_t st = icm_read_scaled(&dev, &s);
    CHECK(st == ICM_OK, "read_scaled returns ICM_OK");
    CHECK(s.accel_g[0] > 0.99f && s.accel_g[0] < 1.01f, "accel X == 1 g");
    CHECK(s.accel_g[1] > 0.49f && s.accel_g[1] < 0.51f, "accel Y == 0.5 g");
    CHECK(s.accel_g[2] < -0.99f && s.accel_g[2] > -1.01f, "accel Z == -1 g (sign preserved)");
    CHECK(s.gyro_dps[0] > 9.9f && s.gyro_dps[0] < 10.1f, "gyro X == 10 dps");
    CHECK(s.gyro_dps[2] < -9.9f && s.gyro_dps[2] > -10.1f, "gyro Z == -10 dps");
}

/**
 * @brief 温度换算公式 T = raw / 132.48 + 25
 */
static void test_temperature(void)
{
    printf("[test] temperature formula\n");
    regs_reset();
    icm_t dev = make_dev();
    g_regs[ICM_REG_TEMP_DATA1] = 0x00;
    g_regs[ICM_REG_TEMP_DATA0] = 0x00;
    float t = 0.0f;
    icm_status_t st = icm_read_temperature(&dev, &t);
    CHECK(st == ICM_OK, "read_temperature returns ICM_OK");
    CHECK(t > 24.9f && t < 25.1f, "raw 0 -> 25.0 C");

    int16_t raw = 13248; /* 100 * 132.48 */
    g_regs[ICM_REG_TEMP_DATA1] = (uint8_t)(raw >> 8);
    g_regs[ICM_REG_TEMP_DATA0] = (uint8_t)(raw & 0xFF);
    st = icm_read_temperature(&dev, &t);
    CHECK(st == ICM_OK, "second read ok");
    CHECK(t > 124.9f && t < 125.1f, "raw 13248 -> 125.0 C");
}

/**
 * @brief INT_STATUS 数据就绪位解析
 */
static void test_data_ready(void)
{
    printf("[test] data ready flag\n");
    regs_reset();
    icm_t dev = make_dev();
    bool rdy = false;

    g_regs[ICM_REG_INT_STATUS] = 0x00;
    (void)icm_data_ready(&dev, &rdy);
    CHECK(!rdy, "no data -> false");

    g_regs[ICM_REG_INT_STATUS] = ICM_INT_STATUS_DATA_RDY;
    (void)icm_data_ready(&dev, &rdy);
    CHECK(rdy, "DATA_RDY set -> true");
}

/**
 * @brief 空指针与越界枚举的参数保护
 */
static void test_param_guards(void)
{
    printf("[test] parameter guards\n");
    regs_reset();
    icm_t dev = make_dev();
    CHECK(icm_init(NULL, mock_read, mock_write, mock_delay_us, NULL) == ICM_ERR_PARAM, "init(NULL) rejected");
    CHECK(icm_init(&dev, mock_read, mock_write, NULL, NULL) == ICM_ERR_PARAM, "init without delay rejected");
    CHECK(icm_read_scaled(&dev, NULL) == ICM_ERR_PARAM, "read_scaled(NULL) rejected");
    CHECK(icm_set_accel_fs(&dev, (icm_accel_fs_t)9) == ICM_ERR_PARAM, "accel fs out of range rejected");
    CHECK(icm_set_odr(&dev, (icm_odr_t)0) == ICM_ERR_PARAM, "odr 0 rejected");
    CHECK(icm_set_odr(&dev, (icm_odr_t)0x0C) == ICM_ERR_PARAM, "reserved odr rejected");
    CHECK(icm_set_bank(&dev, 3u) == ICM_OK, "bank 3 accepted");
    CHECK(icm_set_bank(&dev, 4u) == ICM_OK, "bank 4 accepted");
    CHECK(icm_set_bank(&dev, 5u) == ICM_ERR_PARAM, "invalid bank rejected");
}

int main(void)
{
    printf("=== icm42688 unit tests ===\n");
    test_init_ok();
    test_init_bad_id();
    test_init_bus_error();
    test_accel_fs_bits();
    test_gyro_fs_bits();
    test_odr();
    test_read_scaled();
    test_temperature();
    test_data_ready();
    test_param_guards();

    printf("=== %s (%d failure%s) ===\n",
           g_failures == 0 ? "PASS" : "FAIL",
           g_failures, g_failures == 1 ? "" : "s");
    return (g_failures == 0) ? 0 : 1;
}
