/**
 * @file    bsp_systick.h
 * @brief   1 kHz SysTick 节拍 + ms 级延时 / 时间戳。
 *
 * 设计要点：
 *   - 使用 Cortex-M0+ 内置 SysTick（CMSIS `SysTick_Config`），不占任何 TIMG/TIMA。
 *   - SysTick_Handler 中只做 ① 计数器自增 ② 置 1 kHz "tick pending" 标志，
 *     业务在主循环里拉取标志位执行，避免重活塞进 ISR。
 *   - 其它模块需要 1 kHz 周期任务时，用 `bsp_systick_consume_tick()` 取/清标志，
 *     该函数对 SysTick_Handler 的写入做了原子保护（PRIMASK 关中断）。
 *   - 互补滤波等数值积分使用本模块提供的 `bsp_systick_get_ms()` 计算 dt。
 */

#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 1 kHz SysTick 中断。
 * @param  hz  期望中断频率，本工程固定传入 1000。
 * @return 0 = 成功；非 0 = SysTick_Config 失败（reload 超过 24-bit）。
 */
int32_t bsp_systick_init(uint32_t hz);

/**
 * @brief  返回上电至今的毫秒数（32-bit，约 49 天溢出）。
 */
uint32_t bsp_systick_get_ms(void);

/**
 * @brief  阻塞延时 ms 毫秒（粗粒度，依赖 SysTick）。
 */
void bsp_systick_delay_ms(uint32_t ms);

/**
 * @brief  消费 1 kHz tick pending 标志。
 * @return true  → 上一次消费后至少发生过一次 SysTick；标志已清零。
 *         false → 还未到下一个 1 ms。
 *
 *  典型用法：在主循环里 `if (bsp_systick_consume_tick()) { do_1khz_job(); }`。
 *  原子性由 PRIMASK 短暂关闭实现，避免和 SysTick_Handler 竞争。
 */
bool bsp_systick_consume_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
