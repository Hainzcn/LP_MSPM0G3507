/**
 * @file    mpu6050.h
 * @brief   MPU6050 (InvenSense 6 轴 IMU) 寄存器图 + 简单驱动。
 *
 * 量程 / 灵敏度（与 init 中配置一致，**改动 init 必须同步改 sensitivity 宏**）：
 *   - 加速度：±8 g  → 4096 LSB / g
 *   - 陀螺仪：±500 dps → 65.5 LSB / (°/s)
 *   - DLPF   : 44 Hz   （滤掉 PWM 20 kHz 旁瓣 + 大部分电机机械噪声）
 *   - SMPLRT : 1 kHz   （SMPLRT_DIV = 0，对 DLPF≠0/7 时基准 1 kHz）
 *
 * I²C：
 *   - 地址 0x68 (AD0=GND)；如硬件焊 AD0=VCC 改 0x69，需把 #define 调整。
 *   - 走 bsp_imu_i2c 阻塞接口，每次 burst read 14 字节耗时 ≈ 0.4 ms。
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR        0x68u   /* AD0 = GND；若拉高改 0x69 */
#define MPU6050_WHOAMI_VALUE    0x68u   /* WHO_AM_I 寄存器固定回读值 */

/* 灵敏度（int16 raw → 物理单位）；与 init 中 GYRO/ACCEL_CONFIG 强绑定 */
#define MPU6050_ACCEL_LSB_PER_G     4096.0f      /* ±8 g */
#define MPU6050_GYRO_LSB_PER_DPS    65.5f        /* ±500 °/s */
#define MPU6050_TEMP_OFFSET         36.53f
#define MPU6050_TEMP_LSB_PER_C      340.0f

/**
 * @brief  原始 14 字节 burst 解析后的整型读数。
 *         索引顺序 = 寄存器顺序 = ACCEL_X/Y/Z, TEMP, GYRO_X/Y/Z。
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_t;

/**
 * @brief  初始化 MPU6050：复位 → 解除 sleep → 设采样 / DLPF / 量程。
 * @return 0 成功；-1 = WHO_AM_I 校验失败；-2/-3 = I²C 通信错误。
 */
int32_t mpu6050_init(void);

/**
 * @brief  burst read 14 字节并按大端拼成 7 个 int16。
 * @return 0 成功，<0 = I²C 错误码。
 */
int32_t mpu6050_read_raw(mpu6050_raw_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
