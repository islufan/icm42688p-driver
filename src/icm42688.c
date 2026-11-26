/**
 * @file    icm42688.c
 * @brief   ICM-42688-P 六轴 IMU 驱动 —— 实现
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 *
 * @details
 * 实现约定：
 *   - 量程字段位于 CONFIG0 的 bit[7:5]，采样率字段位于 bit[3:0]，两者互不影响；
 *   - 所有寄存器访问都经 bus_read / bus_write，失败立即返回错误码；
 *   - 短时序等待通过调用方提供的 delay_us 回调完成，驱动不打印；
 *     陀螺仪启动所需的长时间等待仍由调用方保证。
 *
 * 详细接口说明见 icm42688.h。
 */
#include "icm42688.h"

/* ================================================================== */
/* 内部实现                                                            */
/* ================================================================== */

/**
 * @brief  封装读回调，把非 0 返回值统一转换为 ICM_ERR_BUS
 * @param  dev 设备实例
 * @param  reg 起始寄存器
 * @param  buf 输出缓冲
 * @param  len 字节数
 * @return ICM_OK 或 ICM_ERR_*
 */
static icm_status_t bus_read(icm_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (dev == NULL || dev->read == NULL) {
        return ICM_ERR_PARAM;
    }
    return (dev->read(dev->ctx, reg, buf, len) == 0) ? ICM_OK : ICM_ERR_BUS;
}

/**
 * @brief  封装写回调，把非 0 返回值统一转换为 ICM_ERR_BUS
 * @param  dev 设备实例
 * @param  reg 起始寄存器
 * @param  buf 待写数据
 * @param  len 字节数
 * @return ICM_OK 或 ICM_ERR_*
 */
static icm_status_t bus_write(icm_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (dev == NULL || dev->write == NULL) {
        return ICM_ERR_PARAM;
    }
    return (dev->write(dev->ctx, reg, buf, len) == 0) ? ICM_OK : ICM_ERR_BUS;
}

/**
 * @brief  读-改-写 CONFIG0 类寄存器，只修改 mask 指定的位域
 *
 * @param  dev   设备实例
 * @param  reg   目标寄存器（GYRO_CONFIG0 或 ACCEL_CONFIG0）
 * @param  mask  需要修改的位域掩码
 * @param  value 新值（自动与 mask 对齐）
 * @return ICM_OK 或 ICM_ERR_*
 *
 * @note   旧值等于新值时直接返回，避免无意义的总线写操作。
 */
static icm_status_t update_config0(icm_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old_val = 0;
    icm_status_t st = bus_read(dev, reg, &old_val, 1);
    if (st != ICM_OK) {
        return st;
    }

    uint8_t new_val = (uint8_t)((old_val & (uint8_t)~mask) | (value & mask));
    if (new_val == old_val) {
        return ICM_OK;
    }
    return bus_write(dev, reg, &new_val, 1);
}

/**
 * @brief  查表：加速度量程对应的 LSb/g
 * @param  fs 量程枚举
 * @return 灵敏度，非法值回退到 ±16 g 的 2048
 */
static float accel_lsb_per_g(icm_accel_fs_t fs)
{
    switch (fs) {
    case ICM_ACCEL_FS_16G: return 2048.0f;
    case ICM_ACCEL_FS_8G:  return 4096.0f;
    case ICM_ACCEL_FS_4G:  return 8192.0f;
    case ICM_ACCEL_FS_2G:  return 16384.0f;
    default:               return 2048.0f;
    }
}

/**
 * @brief  查表：陀螺仪量程对应的 LSb/dps
 * @param  fs 量程枚举
 * @return 灵敏度，非法值回退到 ±2000 dps 的 16.4
 */
static float gyro_lsb_per_dps(icm_gyro_fs_t fs)
{
    switch (fs) {
    case ICM_GYRO_FS_2000DPS: return 16.4f;
    case ICM_GYRO_FS_1000DPS: return 32.8f;
    case ICM_GYRO_FS_500DPS:  return 65.5f;
    case ICM_GYRO_FS_250DPS:  return 131.0f;
    default:                  return 16.4f;
    }
}

static bool valid_odr(icm_odr_t odr)
{
    switch (odr) {
    case ICM_ODR_32KHZ:
    case ICM_ODR_16KHZ:
    case ICM_ODR_8KHZ:
    case ICM_ODR_4KHZ:
    case ICM_ODR_2KHZ:
    case ICM_ODR_1KHZ:
    case ICM_ODR_200HZ:
    case ICM_ODR_100HZ:
    case ICM_ODR_50HZ:
    case ICM_ODR_25HZ:
    case ICM_ODR_12_5HZ:
    case ICM_ODR_500HZ:
        return true;
    default:
        return false;
    }
}

/* ================================================================== */
/* 公共接口实现                                                        */
/* ================================================================== */

icm_status_t icm_set_bank(icm_t *dev, uint8_t bank)
{
    if (dev == NULL) {
        return ICM_ERR_PARAM;
    }
    if ((unsigned)bank >= ICM_REG_BANK_MAX) {
        return ICM_ERR_PARAM;
    }
    if (dev->bank == bank) {
        return ICM_OK;   /* 已在该 Bank，省一次总线写 */
    }

    uint8_t v = bank;
    icm_status_t st = bus_write(dev, ICM_REG_REG_BANK_SEL, &v, 1);
    if (st == ICM_OK) {
        dev->bank = bank;
    }
    return st;
}

icm_status_t icm_read_reg(icm_t *dev, uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ICM_ERR_PARAM;
    }
    return bus_read(dev, reg, value, 1);
}

icm_status_t icm_write_reg(icm_t *dev, uint8_t reg, uint8_t value)
{
    icm_status_t st = bus_write(dev, reg, &value, 1);
    if (st == ICM_OK && reg == ICM_REG_REG_BANK_SEL) {
        dev->bank = ((unsigned)value < ICM_REG_BANK_MAX) ? value : 0xFFu;
    }
    return st;
}

icm_status_t icm_soft_reset(icm_t *dev)
{
    if (dev == NULL || dev->delay_us == NULL) {
        return ICM_ERR_PARAM;
    }

    icm_status_t st = icm_set_bank(dev, 0);
    if (st != ICM_OK) {
        return st;
    }

    st = icm_write_reg(dev, ICM_REG_DEVICE_CONFIG, ICM_DEVICE_CONFIG_SOFT_RESET);
    if (st != ICM_OK) {
        return st;
    }

    dev->delay_us(dev->ctx, 1000u);
    dev->ready = false;   /* 复位后必须重新初始化 */
    return ICM_OK;
}

icm_status_t icm_set_accel_fs(icm_t *dev, icm_accel_fs_t fs)
{
    if (dev == NULL) {
        return ICM_ERR_PARAM;
    }
    if ((unsigned)fs > (unsigned)ICM_ACCEL_FS_2G) {
        return ICM_ERR_PARAM;
    }

    icm_status_t st = icm_set_bank(dev, 0);
    if (st != ICM_OK) {
        return st;
    }

    /* 量程在 bit[7:5]；直接写 bit[1:0] 会误改采样率 */
    st = update_config0(dev, ICM_REG_ACCEL_CONFIG0, ICM_CONFIG0_FS_MASK,
                        (uint8_t)((uint8_t)fs << ICM_CONFIG0_FS_SHIFT));
    if (st == ICM_OK) {
        dev->accel_lsb_per_g = accel_lsb_per_g(fs);
    }
    return st;
}

icm_status_t icm_set_gyro_fs(icm_t *dev, icm_gyro_fs_t fs)
{
    if (dev == NULL) {
        return ICM_ERR_PARAM;
    }
    if ((unsigned)fs > (unsigned)ICM_GYRO_FS_250DPS) {
        return ICM_ERR_PARAM;
    }

    icm_status_t st = icm_set_bank(dev, 0);
    if (st != ICM_OK) {
        return st;
    }

    st = update_config0(dev, ICM_REG_GYRO_CONFIG0, ICM_CONFIG0_FS_MASK,
                        (uint8_t)((uint8_t)fs << ICM_CONFIG0_FS_SHIFT));
    if (st == ICM_OK) {
        dev->gyro_lsb_per_dps = gyro_lsb_per_dps(fs);
    }
    return st;
}

icm_status_t icm_set_odr(icm_t *dev, icm_odr_t odr)
{
    if (dev == NULL) {
        return ICM_ERR_PARAM;
    }
    if (!valid_odr(odr)) {
        return ICM_ERR_PARAM;
    }

    icm_status_t st = icm_set_bank(dev, 0);
    if (st != ICM_OK) {
        return st;
    }

    /* 两个 CONFIG0 地址连续，一次读写可避免总线故障时只改到其中一颗传感器。 */
    uint8_t config[2] = {0};
    st = bus_read(dev, ICM_REG_GYRO_CONFIG0, config, sizeof(config));
    if (st != ICM_OK) {
        return st;
    }

    config[0] = (uint8_t)((config[0] & 0xF0u) | (uint8_t)odr);
    config[1] = (uint8_t)((config[1] & 0xF0u) | (uint8_t)odr);
    return bus_write(dev, ICM_REG_GYRO_CONFIG0, config, sizeof(config));
}

icm_status_t icm_enable(icm_t *dev)
{
    if (dev == NULL || dev->delay_us == NULL) {
        return ICM_ERR_PARAM;
    }

    icm_status_t st = icm_set_bank(dev, 0);
    if (st != ICM_OK) {
        return st;
    }
    st = icm_write_reg(dev, ICM_REG_PWR_MGMT0, ICM_PWR_MGMT0_LN_BOTH);
    if (st == ICM_OK) {
        dev->delay_us(dev->ctx, 200u);
    }
    return st;
}

icm_status_t icm_read_raw(icm_t *dev, icm_raw_t *out)
{
    if (dev == NULL || out == NULL) {
        return ICM_ERR_PARAM;
    }
    if (!dev->ready) {
        return ICM_ERR_STATE;
    }

    /* 一次读 12 字节：ACCEL X/Y/Z 与 GYRO X/Y/Z 在同一帧内采样 */
    uint8_t buf[12] = {0};
    icm_status_t st = bus_read(dev, ICM_REG_ACCEL_DATA_X1, buf, sizeof(buf));
    if (st != ICM_OK) {
        return st;
    }

    /* 寄存器为大端，先拼成 uint16_t 再转有符号，避免符号扩展问题 */
    out->accel[0] = (int16_t)((uint16_t)(buf[0]  << 8) | buf[1]);
    out->accel[1] = (int16_t)((uint16_t)(buf[2]  << 8) | buf[3]);
    out->accel[2] = (int16_t)((uint16_t)(buf[4]  << 8) | buf[5]);
    out->gyro[0]  = (int16_t)((uint16_t)(buf[6]  << 8) | buf[7]);
    out->gyro[1]  = (int16_t)((uint16_t)(buf[8]  << 8) | buf[9]);
    out->gyro[2]  = (int16_t)((uint16_t)(buf[10] << 8) | buf[11]);
    return ICM_OK;
}

icm_status_t icm_read_scaled(icm_t *dev, icm_scaled_t *out)
{
    if (dev == NULL || out == NULL) {
        return ICM_ERR_PARAM;
    }

    icm_raw_t raw;
    icm_status_t st = icm_read_raw(dev, &raw);
    if (st != ICM_OK) {
        return st;
    }

    /* 灵敏度已随量程更新，这里只做除法 */
    for (int i = 0; i < 3; i++) {
        out->accel_g[i]  = (float)raw.accel[i] / dev->accel_lsb_per_g;
        out->gyro_dps[i] = (float)raw.gyro[i]  / dev->gyro_lsb_per_dps;
    }
    return ICM_OK;
}

icm_status_t icm_read_temperature(icm_t *dev, float *deg_c)
{
    if (dev == NULL || deg_c == NULL) {
        return ICM_ERR_PARAM;
    }

    uint8_t buf[2] = {0};
    icm_status_t st = bus_read(dev, ICM_REG_TEMP_DATA1, buf, sizeof(buf));
    if (st != ICM_OK) {
        return st;
    }

    int16_t raw = (int16_t)((uint16_t)(buf[0] << 8) | buf[1]);
    *deg_c = (float)raw / 132.48f + 25.0f;   /* 传感器寄存器的换算公式 */
    return ICM_OK;
}

icm_status_t icm_data_ready(icm_t *dev, bool *ready)
{
    if (dev == NULL || ready == NULL) {
        return ICM_ERR_PARAM;
    }

    uint8_t status = 0;
    icm_status_t st = bus_read(dev, ICM_REG_INT_STATUS, &status, 1);
    if (st != ICM_OK) {
        return st;
    }

    *ready = (status & ICM_INT_STATUS_DATA_RDY) != 0;
    return ICM_OK;
}

icm_status_t icm_init(icm_t *dev, icm_bus_read_fn rd, icm_bus_write_fn wr,
                      icm_delay_us_fn delay_us, void *ctx)
{
    if (dev == NULL || rd == NULL || wr == NULL || delay_us == NULL) {
        return ICM_ERR_PARAM;
    }

    /* 先绑定总线，后续所有寄存器操作都依赖这两个回调 */
    dev->read  = rd;
    dev->write = wr;
    dev->delay_us = delay_us;
    dev->ctx   = ctx;
    dev->bank  = 0xFF;                /* 不可能的值，强制首次 icm_set_bank() 真正下发 */
    dev->accel_lsb_per_g  = 2048.0f;  /* 与默认量程 ±16 g 对应 */
    dev->gyro_lsb_per_dps = 16.4f;    /* 与默认量程 ±2000 dps 对应 */
    dev->ready = false;

    icm_status_t st = icm_soft_reset(dev);
    if (st != ICM_OK) {
        return st;
    }

    uint8_t id = 0;
    st = icm_read_reg(dev, ICM_REG_WHO_AM_I, &id);
    if (st != ICM_OK) {
        return st;
    }
    if (id != ICM_WHO_AM_I_VALUE) {
        return ICM_ERR_ID;
    }

    st = icm_set_accel_fs(dev, ICM_ACCEL_FS_16G);
    if (st != ICM_OK) {
        return st;
    }
    st = icm_set_gyro_fs(dev, ICM_GYRO_FS_2000DPS);
    if (st != ICM_OK) {
        return st;
    }
    st = icm_set_odr(dev, ICM_ODR_1KHZ);
    if (st != ICM_OK) {
        return st;
    }
    st = icm_enable(dev);
    if (st != ICM_OK) {
        return st;
    }

    dev->ready = true;   /* 只有全部配置成功才标记可用 */
    return ICM_OK;
}
