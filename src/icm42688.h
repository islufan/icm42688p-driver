/**
 * @file    icm42688.h
 * @brief   ICM-42688-P 六轴 IMU 驱动 —— 公共接口
 * @author  LuFan
 * @date    2026-11-26
 * @license MIT
 *
 * @details
 * 本驱动面向嵌入式裸机环境，只依赖 C99 标准库，不绑定任何 HAL 或 SDK。
 * 总线（SPI / I2C）通过 icm_bus_read_fn / icm_bus_write_fn 回调注入，
 * 因此同一份源码可以运行在 STM32、ESP32 或 PC 上，也便于用 mock 总线做单元测试。
 *
 * 典型用法：
 * @code
 *   icm_t imu;
 *   if (icm_init(&imu, my_read, my_write, my_delay_us, &my_bus) == ICM_OK) {
 *       delay_ms(50);
 *       icm_scaled_t s;
 *       icm_read_scaled(&imu, &s);   // s.accel_g[] 单位 g，s.gyro_dps[] 单位 dps
 *   }
 * @endcode
 *
 * @warning 本驱动不是线程安全的。若多个任务共用同一个 icm_t 实例，
 *          调用方需自行加锁，否则寄存器读写可能交错。
 */
#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* NULL */

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* 寄存器地址（Bank 0）                                                */
/* ================================================================== */
#define ICM_REG_DEVICE_CONFIG    0x11u  /**< 设备配置：bit0 软复位 */
#define ICM_REG_TEMP_DATA1       0x1Du  /**< 温度数据高字节 */
#define ICM_REG_TEMP_DATA0       0x1Eu  /**< 温度数据低字节 */
#define ICM_REG_ACCEL_DATA_X1    0x1Fu  /**< 加速度 X 高字节，其后连续 11 字节为 XY Z + GYRO XYZ */
#define ICM_REG_INT_STATUS       0x2Du  /**< 中断状态：bit3 为数据就绪 */
#define ICM_REG_PWR_MGMT0        0x4Eu  /**< 电源管理：bit[3:2] 陀螺模式，bit[1:0] 加速度模式 */
#define ICM_REG_GYRO_CONFIG0     0x4Fu  /**< 陀螺配置：bit[7:5] 量程，bit[3:0] 采样率 */
#define ICM_REG_ACCEL_CONFIG0    0x50u  /**< 加速度配置：bit[7:5] 量程，bit[3:0] 采样率 */
#define ICM_REG_WHO_AM_I         0x75u  /**< 器件 ID，固定为 0x47 */
#define ICM_REG_REG_BANK_SEL     0x76u  /**< 寄存器 Bank 选择 */

#define ICM_WHO_AM_I_VALUE       0x47u  /**< WHO_AM_I 的期望值 */

/* CONFIG0 类寄存器的位定义（GYRO_CONFIG0 与 ACCEL_CONFIG0 相同） */
#define ICM_CONFIG0_FS_SHIFT     5u                        /**< 量程字段起始位 */
#define ICM_CONFIG0_FS_MASK      (0x07u << ICM_CONFIG0_FS_SHIFT) /**< 量程字段掩码 bit[7:5] */
#define ICM_CONFIG0_ODR_MASK     0x0Fu                     /**< 采样率字段掩码 bit[3:0] */
#define ICM_REG_BANK_MAX         5u                        /**< 可用寄存器 Bank 为 0~4 */

#define ICM_DEVICE_CONFIG_SOFT_RESET 0x01u  /**< 写入 bit0 触发软复位 */
#define ICM_PWR_MGMT0_LN_BOTH    0x0Fu      /**< 陀螺与加速度均设为低噪声模式 */
#define ICM_INT_STATUS_DATA_RDY  0x08u      /**< INT_STATUS 中的数据就绪位 */

/* ================================================================== */
/* 返回状态                                                            */
/* ================================================================== */
/**
 * @brief 所有 API 的返回状态
 */
typedef enum {
    ICM_OK = 0,      /**< 操作成功 */
    ICM_ERR_PARAM,   /**< 参数非法：空指针或枚举越界 */
    ICM_ERR_BUS,     /**< 总线读写失败（回调返回非 0） */
    ICM_ERR_ID,      /**< WHO_AM_I 与 ICM_WHO_AM_I_VALUE 不符 */
    ICM_ERR_STATE    /**< 状态错误，例如未初始化就读取数据 */
} icm_status_t;

/* ================================================================== */
/* 总线回调                                                            */
/* ================================================================== */
/**
 * @brief 总线读回调
 *
 * @param ctx 调用方上下文指针，由 icm_init() 原样透传
 * @param reg 起始寄存器地址
 * @param buf 输出缓冲区，由驱动提供
 * @param len 需要读取的字节数
 *
 * @return 0 表示成功，非 0 表示失败（驱动会转换为 ICM_ERR_BUS）
 *
 * @note 实现时需保证整帧期间片选保持有效；SPI 读操作应发送 reg|0x80。
 */
typedef int (*icm_bus_read_fn)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief 总线写回调
 *
 * @param ctx 调用方上下文指针，由 icm_init() 原样透传
 * @param reg 起始寄存器地址
 * @param buf 待写入数据
 * @param len 需要写入的字节数
 *
 * @return 0 表示成功，非 0 表示失败（驱动会转换为 ICM_ERR_BUS）
 *
 * @note SPI 写操作应发送 reg&0x7F。
 */
typedef int (*icm_bus_write_fn)(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);

/**
 * @brief 延时回调
 *
 * @param ctx 调用方上下文指针，由 icm_init() 原样透传
 * @param us 需要延时的微秒数
 *
 * @note 驱动使用此回调满足软复位后的 1 ms 等待，以及传感器上电后的
 *       200 us 无寄存器写入窗口。实现可以使用系统的微秒延时或更长的
 *       保守延时。
 */
typedef void (*icm_delay_us_fn)(void *ctx, uint32_t us);

/* ================================================================== */
/* 配置枚举                                                            */
/* ================================================================== */
/**
 * @brief 加速度计量程
 */
typedef enum {
    ICM_ACCEL_FS_16G = 0,  /**< ±16 g，灵敏度 2048 LSB/g */
    ICM_ACCEL_FS_8G  = 1,  /**< ±8 g，灵敏度 4096 LSB/g */
    ICM_ACCEL_FS_4G  = 2,  /**< ±4 g，灵敏度 8192 LSB/g */
    ICM_ACCEL_FS_2G  = 3   /**< ±2 g，灵敏度 16384 LSB/g */
} icm_accel_fs_t;

/**
 * @brief 陀螺仪量程
 */
typedef enum {
    ICM_GYRO_FS_2000DPS = 0,  /**< ±2000 dps，灵敏度 16.4 LSB/dps */
    ICM_GYRO_FS_1000DPS = 1,  /**< ±1000 dps，灵敏度 32.8 LSB/dps */
    ICM_GYRO_FS_500DPS  = 2,  /**< ±500 dps，灵敏度 65.5 LSB/dps */
    ICM_GYRO_FS_250DPS  = 3   /**< ±250 dps，灵敏度 131 LSB/dps */
} icm_gyro_fs_t;

/**
 * @brief 陀螺仪与加速度计共同支持的输出数据率（枚举值即寄存器编码）
 */
typedef enum {
    ICM_ODR_32KHZ     = 0x01,  /**< 32 kHz */
    ICM_ODR_16KHZ     = 0x02,  /**< 16 kHz */
    ICM_ODR_8KHZ      = 0x03,  /**< 8 kHz */
    ICM_ODR_4KHZ      = 0x04,  /**< 4 kHz */
    ICM_ODR_2KHZ      = 0x05,  /**< 2 kHz */
    ICM_ODR_1KHZ      = 0x06,  /**< 1 kHz（默认） */
    ICM_ODR_200HZ     = 0x07,  /**< 200 Hz */
    ICM_ODR_100HZ     = 0x08,  /**< 100 Hz */
    ICM_ODR_50HZ      = 0x09,  /**< 50 Hz */
    ICM_ODR_25HZ      = 0x0A,  /**< 25 Hz */
    ICM_ODR_12_5HZ    = 0x0B,  /**< 12.5 Hz */
    ICM_ODR_500HZ     = 0x0F   /**< 500 Hz */
} icm_odr_t;

/* ================================================================== */
/* 数据结构                                                            */
/* ================================================================== */
/**
 * @brief 原始码值（未经换算）
 */
typedef struct {
    int16_t accel[3];  /**< 加速度原始码值，顺序为 X、Y、Z */
    int16_t gyro[3];   /**< 角速度原始码值，顺序为 X、Y、Z */
} icm_raw_t;

/**
 * @brief 换算后的物理量
 */
typedef struct {
    float accel_g[3];    /**< 加速度，单位 g，顺序为 X、Y、Z */
    float gyro_dps[3];   /**< 角速度，单位 dps（度/秒），顺序为 X、Y、Z */
} icm_scaled_t;

/**
 * @brief 设备实例
 *
 * @note 除 read / write / delay_us / ctx 由 icm_init() 写入外，其余字段由驱动维护，
 *       调用方不应直接修改，否则会导致换算系数或状态判断错误。
 */
typedef struct {
    icm_bus_read_fn  read;              /**< 读回调，由 icm_init() 写入 */
    icm_bus_write_fn write;             /**< 写回调，由 icm_init() 写入 */
    icm_delay_us_fn  delay_us;          /**< 延时回调，由 icm_init() 写入 */
    void    *ctx;                       /**< 回调上下文，由 icm_init() 写入 */
    uint8_t  bank;                      /**< 当前寄存器 Bank，驱动内部缓存 */
    float    accel_lsb_per_g;           /**< 当前加速度灵敏度，随量程更新 */
    float    gyro_lsb_per_dps;          /**< 当前角速度灵敏度，随量程更新 */
    bool     ready;                     /**< 是否已成功初始化，icm_read_raw() 依赖此标志 */
} icm_t;

/* ================================================================== */
/* API                                                                 */
/* ================================================================== */

/**
 * @brief  初始化传感器：软复位 → 校验 WHO_AM_I → 写入默认配置并启用
 *
 * @param  dev  设备实例，由调用方分配；初始化后可重复使用
 * @param  rd   总线读回调，不可为 NULL
 * @param  wr   总线写回调，不可为 NULL
 * @param  delay_us 延时回调，不可为 NULL
 * @param  ctx  透传给读写与延时回调的上下文指针，可为 NULL
 *
 * @return ICM_OK        初始化成功
 * @return ICM_ERR_PARAM 任一指针为空
 * @return ICM_ERR_BUS   总线读写失败
 * @return ICM_ERR_ID    WHO_AM_I 不等于 0x47
 *
 * @note   默认配置为 ±16 g / ±2000 dps / 1 kHz / 低噪声模式。
 * @note   返回后需等待约 45 ms 再读取陀螺仪数据。函数只通过 delay_us
 *         回调执行芯片规定的短延时，不实现平台相关的长时间等待。
 * @warning 本函数会覆盖寄存器中已有的量程与采样率配置。
 */
icm_status_t icm_init(icm_t *dev, icm_bus_read_fn rd, icm_bus_write_fn wr,
                      icm_delay_us_fn delay_us, void *ctx);

/**
 * @brief  软复位传感器
 *
 * @param  dev 设备实例
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   复位后寄存器恢复默认值，需重新调用 icm_init() 或手动配置；
 *         函数会通过初始化时提供的延时回调等待复位完成，复位完成后
 *         dev->ready 被置为 false。
 */
icm_status_t icm_soft_reset(icm_t *dev);

/**
 * @brief  设置加速度计量程，保留当前采样率配置
 *
 * @param  dev 设备实例
 * @param  fs  目标量程，取值见 icm_accel_fs_t
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   成功后同步更新 dev->accel_lsb_per_g，后续 icm_read_scaled() 无需重算系数。
 */
icm_status_t icm_set_accel_fs(icm_t *dev, icm_accel_fs_t fs);

/**
 * @brief  设置陀螺仪量程，保留当前采样率配置
 *
 * @param  dev 设备实例
 * @param  fs  目标量程，取值见 icm_gyro_fs_t
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 */
icm_status_t icm_set_gyro_fs(icm_t *dev, icm_gyro_fs_t fs);

/**
 * @brief  同时设置加速度计与陀螺仪的采样率，保留量程配置
 *
 * @param  dev 设备实例
 * @param  odr 目标采样率，取值见 icm_odr_t
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   两颗传感器共用同一采样率，ICM-42688-P 不支持分别设置。
 */
icm_status_t icm_set_odr(icm_t *dev, icm_odr_t odr);

/**
 * @brief  使能加速度计与陀螺仪（低噪声模式）
 *
 * @param  dev 设备实例
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   函数会等待至少 200 us 再返回，调用方仍需等待约 45 ms 后才可
 *         依赖陀螺仪的有效数据。
 */
icm_status_t icm_enable(icm_t *dev);

/**
 * @brief  读取加速度与角速度的原始码值
 *
 * @param  dev 设备实例
 * @param  out 输出结构体，不可为 NULL
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS / ICM_ERR_STATE（未初始化）
 *
 * @note   一次 burst 读取 12 字节，保证六轴数据来自同一采样时刻。
 */
icm_status_t icm_read_raw(icm_t *dev, icm_raw_t *out);

/**
 * @brief  读取并按当前量程换算为物理量
 *
 * @param  dev 设备实例
 * @param  out 输出结构体，不可为 NULL
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS / ICM_ERR_STATE
 *
 * @note   换算公式：accel_g = raw / accel_lsb_per_g，gyro_dps = raw / gyro_lsb_per_dps。
 */
icm_status_t icm_read_scaled(icm_t *dev, icm_scaled_t *out);

/**
 * @brief  读取芯片温度
 *
 * @param  dev   设备实例
 * @param  deg_c 输出温度，单位 ℃，不可为 NULL
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   换算公式：T = raw / 132.48 + 25，精度约 ±0.5 ℃。
 */
icm_status_t icm_read_temperature(icm_t *dev, float *deg_c);

/**
 * @brief  查询是否有新数据
 *
 * @param  dev   设备实例
 * @param  ready 输出标志，true 表示可读取，不可为 NULL
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   读取 INT_STATUS 的数据就绪位；未使用中断时可用于轮询。
 */
icm_status_t icm_data_ready(icm_t *dev, bool *ready);

/**
 * @brief  直接写寄存器（用于 datasheet 中未封装的配置项）
 *
 * @param  dev   设备实例
 * @param  reg   寄存器地址
 * @param  value 写入值
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @warning 绕过驱动的配置校验，调用方需自行确认寄存器位定义。
 */
icm_status_t icm_write_reg(icm_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief  直接读寄存器
 *
 * @param  dev   设备实例
 * @param  reg   寄存器地址
 * @param  value 输出值，不可为 NULL
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 */
icm_status_t icm_read_reg(icm_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief  切换寄存器 Bank
 *
 * @param  dev  设备实例
 * @param  bank 目标 Bank，0~4
 * @return ICM_OK / ICM_ERR_PARAM / ICM_ERR_BUS
 *
 * @note   驱动内部会缓存当前 Bank，重复设置同一值不会产生总线访问。
 */
icm_status_t icm_set_bank(icm_t *dev, uint8_t bank);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
