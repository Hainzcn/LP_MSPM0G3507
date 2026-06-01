/**
 * @file    bsp_laser.h
 * @brief   激光使能 GPIO（PA1 / LASER_EN）。
 *
 * 默认 BSP_LASER_ACTIVE_LOW=0（高电平开），与 Stage0 引脚表「OUT 0 = 关」一致。
 * 若模块为低有效，编译前改 BSP_LASER_ACTIVE_LOW 为 1。
 */

#ifndef BSP_LASER_H
#define BSP_LASER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BSP_LASER_ACTIVE_LOW
#define BSP_LASER_ACTIVE_LOW    (0)
#endif

/** 配置 PA1 为输出并关断激光（上电/初始化安全态）。 */
void bsp_laser_init(void);

/** @param on true=激光开，false=关。 */
void bsp_laser_set_enable(bool on);

/** @return 最近一次 set_enable 请求的状态。 */
bool bsp_laser_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LASER_H */
