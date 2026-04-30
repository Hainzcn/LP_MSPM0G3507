/**
 * @file    bsp_flash_pad.c
 * @brief   PT_LOAD 段 8 字节对齐尾填充 —— MSPM0G3507 flash 64-bit 写硬约束
 *
 * MSPM0G3507 flash controller 一次写一个 64-bit 字，dslite 烧录前会校验
 * PT_LOAD 段大小必须是 8 的倍数，否则抛
 *   "Length of block is N, but it should be divisible by 8".
 *
 * 链接器在 ER_IROM1 末尾把本文件整个 .o 放到 +Last（见
 * `template/keil/mspm0g3507.sct` 的 `bsp_flash_pad.o (+Last)`），
 * 配合 ALIGNALL 8 对 section 起始的对齐，整个 PT_LOAD 段大小一定
 * 落在 8 字节边界上，每次构建多写 0~15 字节填充。
 *
 * **设计要点**：
 *   · 单独成 `.c` 文件 —— scatter 用模块级 `bsp_flash_pad.o (+Last)`
 *     选择器，**优先级高于任何 `.ANY` 通配选择器**（包括 `.ANY1` /
 *     `.ANY3`），从而稳定地"先抢、再 +Last 排末尾"。曾尝试过把同名
 *     变量塞进 main.c 用 `.ANY3 (.flash_pad, +Last)` 抢，最终实测仍
 *     被 `.ANY1 (+RO)` 通配规则拦截 —— 失败现象：12588 → 12604，仍
 *     `mod 8 = 4`。模块级选择器是 ARMCLANG armlink 上唯一稳定保证。
 *   · `extern` 链接 + `used` —— 避免 ARMCLANG -O2 以"无引用"理由
 *     dead-strip 整个 .o（即使 scatter 引用了 section，链接器有时仍
 *     需要符号至少在某个 .c 文件可见）。`used` 属性强制 KEEP。
 *   · 全 `0xFF` —— flash 擦除态就是 `0xFF`，写 `0xFF` 等同未写入，
 *     避免占用一个真实 word。
 *
 * 详见 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.6。
 */
#include <stdint.h>

__attribute__((used, section(".flash_pad"), aligned(8)))
const uint64_t _flash_image_pad = 0xFFFFFFFFFFFFFFFFULL;
