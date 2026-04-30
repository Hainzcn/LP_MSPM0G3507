/**
 * @file    bsp_imu_i2c.c
 * @brief   I2C1 控制器模式阻塞读写实现，详见 bsp_imu_i2c.h。
 *
 * 关键说明（避坑）：
 *   - MSPM0 DriverLib 的 I2C 接口要求把"待发字节"先 fill 进 TX FIFO，
 *     再调用 startTransfer() 触发；否则 TX 全为 0xFF。
 *   - 7-bit 地址在 startTransfer 里直接传，SDK 会自动左移并附 R/W 位。
 *   - "失败"包含三种：仲裁丢失（多主机才会出现，本工程 MCU 唯一主机，
 *     可视为硬件异常）、目标 NACK（设备不在线 / 寄存器越界）、软件超时。
 */

#include "bsp_imu_i2c.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"

/* I2C 单次传输最大软件超时（ms）。MPU6050 14 B burst @ 400 kHz ≈ 0.4 ms，
 * 留 100 倍裕度，足以覆盖电平爬坡 + ISR 抢占等抖动。 */
#define I2C_TIMEOUT_MS  20u

/**
 * @brief  内部：等待 I2C 控制器空闲（IDLE）或超时。
 * @return 0 成功（IDLE 且无错），<0 失败。
 */
static int32_t i2c_wait_idle(uint32_t timeout_ms)
{
    uint32_t start = bsp_systick_get_ms();
    while ((DL_I2C_getControllerStatus(I2C_IMU_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0u) {
        if ((bsp_systick_get_ms() - start) >= timeout_ms) {
            return -3;
        }
    }
    return 0;
}

/**
 * @brief  检查最近一次传输是否出错（NACK / 仲裁丢失）。
 */
static int32_t i2c_check_error(void)
{
    uint32_t st = DL_I2C_getControllerStatus(I2C_IMU_INST);
    if ((st & DL_I2C_CONTROLLER_STATUS_ERROR) != 0u) {
        if ((st & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0u) {
            return -1;
        }
        return -2; /* NACK 类 */
    }
    return 0;
}

void bsp_imu_i2c_init(void)
{
    /* SysConfig 已配好引脚 / 速率 / 启用控制器；这里只清错误状态 */
    (void)DL_I2C_getControllerStatus(I2C_IMU_INST);
}

int32_t bsp_imu_i2c_write_regs(uint8_t dev_addr, uint8_t reg,
                               const uint8_t *data, size_t len)
{
    if (data == NULL && len > 0u) {
        return -2;
    }
    if (len > 15u) {
        /* 单次 burst 上限 = TX FIFO 深度 (16) - 1 (reg 地址)。
         * MPU6050 配置写最多一次 1~2 字节，远低于该限。 */
        return -2;
    }

    int32_t rc = i2c_wait_idle(I2C_TIMEOUT_MS);
    if (rc != 0) { return rc; }

    /* 先 fill TX FIFO：reg 地址 + N 字节数据 */
    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);
    for (size_t i = 0u; i < len; ++i) {
        DL_I2C_transmitControllerData(I2C_IMU_INST, data[i]);
    }

    DL_I2C_startControllerTransfer(I2C_IMU_INST,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        (uint16_t)(len + 1u));

    rc = i2c_wait_idle(I2C_TIMEOUT_MS);
    if (rc != 0) { return rc; }

    return i2c_check_error();
}

int32_t bsp_imu_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    return bsp_imu_i2c_write_regs(dev_addr, reg, &val, 1u);
}

int32_t bsp_imu_i2c_read_regs(uint8_t dev_addr, uint8_t reg,
                              uint8_t *out, size_t len)
{
    if (out == NULL || len == 0u) {
        return -2;
    }

    int32_t rc = i2c_wait_idle(I2C_TIMEOUT_MS);
    if (rc != 0) { return rc; }

    /* Phase 1: 写寄存器地址（不发 STOP，由下面的 RS 接续） */
    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);
    DL_I2C_startControllerTransfer(I2C_IMU_INST,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1u);
    rc = i2c_wait_idle(I2C_TIMEOUT_MS);
    if (rc != 0) { return rc; }
    rc = i2c_check_error();
    if (rc != 0) { return rc; }

    /* Phase 2: Repeated-Start + RX N 字节 */
    DL_I2C_startControllerTransfer(I2C_IMU_INST,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        (uint16_t)len);

    uint32_t start = bsp_systick_get_ms();
    for (size_t i = 0u; i < len; ++i) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
            if ((bsp_systick_get_ms() - start) >= I2C_TIMEOUT_MS) {
                return -3;
            }
        }
        out[i] = DL_I2C_receiveControllerData(I2C_IMU_INST);
    }

    rc = i2c_wait_idle(I2C_TIMEOUT_MS);
    if (rc != 0) { return rc; }

    return i2c_check_error();
}
