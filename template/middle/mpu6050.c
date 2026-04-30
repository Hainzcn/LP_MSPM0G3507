/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动实现，详见 mpu6050.h。
 */

#include "mpu6050.h"
#include "bsp_imu_i2c.h"
#include "bsp_systick.h"

/* ---------------------------------------------------------------------------
 *  寄存器地址（取自 InvenSense MPU-6000/6050 Register Map v4.2）
 * ------------------------------------------------------------------------- */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_INT_ENABLE     0x38
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

/* PWR_MGMT_1 字段 */
#define PWR_RESET_BIT      0x80
#define PWR_CLKSEL_PLL_X   0x01

/* CONFIG 字段：DLPF_CFG = 3 → Accel 44 Hz / Gyro 42 Hz */
#define CONFIG_DLPF_44HZ   0x03

/* GYRO_CONFIG: FS_SEL = 1 (±500 dps) */
#define GYRO_FS_500DPS     (1 << 3)

/* ACCEL_CONFIG: AFS_SEL = 2 (±8 g) */
#define ACCEL_FS_8G        (2 << 3)

int32_t mpu6050_init(void)
{
    /* 1) 复位整片，等 100 ms 让内部稳定。datasheet 要 ≥ 100 ms */
    int32_t rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_PWR_MGMT_1, PWR_RESET_BIT);
    if (rc != 0) { return rc; }
    bsp_systick_delay_ms(100u);

    /* 2) 解除 sleep，时钟选 PLL with X gyro（比内部振荡器更稳） */
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_PWR_MGMT_1, PWR_CLKSEL_PLL_X);
    if (rc != 0) { return rc; }
    bsp_systick_delay_ms(10u);

    /* 3) 校验 WHO_AM_I */
    uint8_t who = 0u;
    rc = bsp_imu_i2c_read_regs(MPU6050_I2C_ADDR,
        REG_WHO_AM_I, &who, 1u);
    if (rc != 0) { return rc; }
    if (who != MPU6050_WHOAMI_VALUE) { return -1; }

    /* 4) DLPF 44 Hz；MPU6050 在 DLPF != 0/7 时基准频率 = 1 kHz */
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_CONFIG, CONFIG_DLPF_44HZ);
    if (rc != 0) { return rc; }

    /* 5) SMPLRT_DIV = 0 → 1 kHz / (1+0) = 1 kHz */
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_SMPLRT_DIV, 0x00);
    if (rc != 0) { return rc; }

    /* 6) 量程：±500 dps + ±8 g（与 mpu6050.h 中灵敏度宏强绑定） */
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_GYRO_CONFIG, GYRO_FS_500DPS);
    if (rc != 0) { return rc; }
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_ACCEL_CONFIG, ACCEL_FS_8G);
    if (rc != 0) { return rc; }

    /* 7) 关闭 INT（本阶段走 SysTick 软轮询，不依赖 DataReady 中断） */
    rc = bsp_imu_i2c_write_reg(MPU6050_I2C_ADDR,
        REG_INT_ENABLE, 0x00);
    if (rc != 0) { return rc; }

    return 0;
}

int32_t mpu6050_read_raw(mpu6050_raw_t *out)
{
    if (out == NULL) { return -2; }

    uint8_t buf[14];
    int32_t rc = bsp_imu_i2c_read_regs(MPU6050_I2C_ADDR,
        REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (rc != 0) { return rc; }

    /* MPU6050 内部寄存器是大端：高字节在前 */
    out->accel_x = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
    out->accel_y = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
    out->accel_z = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
    out->temp    = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
    out->gyro_x  = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
    out->gyro_y  = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    out->gyro_z  = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);
    return 0;
}
