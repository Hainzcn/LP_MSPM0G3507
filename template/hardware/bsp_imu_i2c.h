/**
 * @file    bsp_imu_i2c.h
 * @brief   I2C1 控制器模式寄存器读 / 写原语，仅用于 MPU6050 等 IMU。
 *
 * - 阻塞接口，每次操作内置软件超时（毫秒级），防 SDA 卡死把整车锁死。
 * - 不开 DMA。MPU6050 burst read 14 字节，按 400 kHz 折算 ≈ 0.4 ms，
 *   1 kHz 采样下占用 < 40 % I2C 总线时间，CPU 阻塞可接受。
 * - 多设备共线时，地址区分由上层传入；本驱动不维护地址表。
 */

#ifndef BSP_IMU_I2C_H
#define BSP_IMU_I2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_imu_i2c_init(void);

/**
 * @brief  写入连续寄存器（先发 reg 地址，再发 N 字节数据）。
 * @return 0 成功，<0 失败（-1 = 仲裁丢失，-2 = NACK，-3 = 超时）。
 */
int32_t bsp_imu_i2c_write_regs(uint8_t dev_addr, uint8_t reg,
                               const uint8_t *data, size_t len);

/** 单字节快捷封装。 */
int32_t bsp_imu_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val);

/**
 * @brief  顺序读取连续寄存器（先 W reg，再 RS 读 N 字节）。
 * @return 0 成功，<0 失败码同 write。
 */
int32_t bsp_imu_i2c_read_regs(uint8_t dev_addr, uint8_t reg,
                              uint8_t *out, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_IMU_I2C_H */
