/**
 * @file    bsp_battery.c
 * @brief   电池电压采样实现 —— ADC 轮询 + EMA 滤波 + 阈值状态机
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 bsp_battery.h 中我们定义了"电池监测有哪些功能"。
 * 这个 bsp_battery.c 是"具体怎么实现这些功能"。
 *
 * 电池监测的完整流程：
 *
 *   1. 触发 ADC 转换（启动一次模数转换）
 *   2. 等待转换完成（约 5 µs 的轮询等待）
 *   3. 读取 ADC 原始值（0~4095 的 12 位数字）
 *   4. 换算为电池电压（mV）——包含分压电阻的反算
 *   5. EMA 低通滤波——消除 ADC 噪声
 *   6. 阈值状态机判断——当前电压属于哪个等级
 *   7. 触发下一次转换——为下一拍做准备
 *
 * 所有运算都是整数运算（无浮点），
 * 因为 Cortex-M0+ 没有硬件浮点单元（FPU），
 * 浮点运算需要用软件模拟，很慢。
 *
 * ============================================================
 * 设计要点
 * ============================================================
 *   1. 软件触发 + 轮询完成（不用中断）——简单可靠
 *   2. EMA 用整数实现（alpha × 256 / 256）——避免浮点
 *   3. 状态机有回滞——防止阈值附近反复跳变
 *   4. 断线检测——未接电池时不被误判为 LOW_STOP
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "bsp_battery.h"
#include "ti_msp_dl_config.h"   /* SysConfig 生成：
                                  *   ADC_BAT_INST —— ADC0 实例
                                  *   DL_ADC12_xxx 函数声明 */
#include <stdint.h>              /* uint32_t、uint16_t 等类型 */

/* ================================================================
 * 模块内部状态
 * ================================================================
 * batt_state_t 结构体保存了电池监测模块的所有运行时状态。
 *
 * static 修饰：只在当前文件中可见，外部不能直接访问。
 * 外部代码通过公开函数（bsp_battery_get_mv() 等）间接读取。
 */

typedef struct {
    uint32_t            ema_mv;     /* EMA 滤波后的电池电压（毫伏） */
    uint16_t            last_raw;   /* 最近一次 ADC 原始值（0~4095） */
    uint8_t             primed;     /* 首拍标志：0=还没采到第一次有效值，1=已就绪 */
    bsp_battery_state_t state;      /* 当前阈值状态 */
} batt_state_t;

/** 全局唯一的电池状态实例 */
static batt_state_t s_batt;

/* ================================================================
 * 内部辅助函数
 * ================================================================
 * 以下函数用 static 修饰，只在 bsp_battery.c 内部使用。
 */

/**
 * trigger_conversion —— 触发一次 ADC 软件转换
 *
 * DL_ADC12_startConversion() 告诉 ADC 硬件：
 * "开始一次新的转换！"
 *
 * SysConfig 中把 ADC 配置为 AUTO_NEXT 模式。
 * 这个模式的含义是：每次触发后，只转换一次就停下来
 * （不是连续转换模式），等待下一次触发。
 *
 * 所以我们每次 update() 读取完结果后，
 * 都需要手动调用这个函数触发下一次转换。
 */
static void trigger_conversion(void)
{
    DL_ADC12_startConversion(ADC_BAT_INST);
}

/**
 * wait_conversion_done —— 阻塞等待 ADC 转换完成
 *
 * 功能：轮询等待上一次触发的 ADC 转换完成。
 *
 * 实现方式（为什么不用中断？）：
 *   这里通过读取 ADC 的"原始中断状态"（Raw Interrupt Status）
 * 来判断转换是否完成。虽然它叫"中断状态"，但我们不使能 NVIC 中断，
 * 只是当作一个"完成标志"来轮询。
 *
 * 等待策略：
 *   循环最多 1000 次。在 32 MHz 下，每次循环约 30 ns，
 *   1000 次 ≈ 30 µs。而正常转换只需要约 5 µs（约 50 次循环）。
 *   所以 1000 次的上限绰绰有余。
 *
 * @return true  = 转换完成
 *         false = 超时（硬件异常），调用方应跳过本拍
 */
static bool wait_conversion_done(void)
{
    /* 循环等待 MEM0 结果加载完成
     * DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED 表示
     * "转换结果已写入 MEM0 寄存器" */
    for (uint32_t i = 0u; i < 1000u; i++) {
        /* getRawInterruptStatus 读的是"原始"中断状态，
         * 不需要 NVIC 使能也能读到。
         * 这就是"轮询中断标志"——把中断标志当作状态位来查询。 */
        if (DL_ADC12_getRawInterruptStatus(ADC_BAT_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) != 0u) {
            /* 转换完成：清除中断标志（写 1 清除），返回 true */
            DL_ADC12_clearInterruptStatus(ADC_BAT_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
            return true;
        }
    }
    /* 超时：1000 次循环后仍未完成
     * 可能是 ADC 硬件异常（概率极低） */
    return false;
}

/**
 * classify —— 阈值 + 回滞状态机
 *
 * 这是电池监测的"决策大脑"——根据当前滤波后的电压（mv）
 * 和上一拍的状态（prev），决定当前应该是什么状态。
 *
 * 状态转换图：
 *
 *   ┌──────────┐
 *   │ UNKNOWN  │ ← 上电初始 / 断线检测
 *   └────┬─────┘
 *        │ mv > DISCONNECTED_MV（跨越断线阈值）
 *        ▼
 *   ┌──────────┐
 *   │ NORMAL   │ ← mv > WARN_MV
 *   └────┬─────┘
 *        │ mv ≤ WARN_MV
 *        ▼
 *   ┌──────────┐         ┌──────────┐
 *   │ LOW_WARN │ ←─────→ │ LOW_STOP │
 *   └──────────┘  mv ≤    └──────────┘
 *        │        STOP_MV
 *        │ mv ≥ WARN_MV + HYS（回滞！不是回到 WARN_MV 就直接回）
 *        ▼
 *   ┌──────────┐
 *   │ NORMAL   │
 *   └──────────┘
 *
 * 回滞（Hysteresis）的重要性：
 *   如果没有回滞，当电压在 9.5V 附近波动时（ADC 有噪声），
 *   状态会在 NORMAL 和 LOW_WARN 之间来回跳变。
 *
 *   有了 200mV 的回滞：
 *   从 NORMAL → LOW_WARN：9.5V（阈值）
 *   从 LOW_WARN → NORMAL：9.7V（阈值 + 回滞）
 *   这 200mV 的"死区"避免了频繁切换。
 *
 * 断线检测：
 *   如果 mv < DISCONNECTED_MV（默认 1V），认为电池没接，
 *   返回 UNKNOWN。不会触发 LOW_STOP。
 *
 * @param mv   当前 EMA 滤波后的电压（毫伏）
 * @param prev 上一拍的状态
 * @return 当前应处的状态
 */
static bsp_battery_state_t classify(uint32_t mv, bsp_battery_state_t prev)
{
    /* ---- 断线检测 ---- */
    if (mv < (uint32_t)BSP_BATTERY_DISCONNECTED_MV) {
        return BSP_BATT_STATE_UNKNOWN;
    }

    /* ---- 根据上一拍的状态做转换 ---- */
    switch (prev) {

    /* 当前处于 LOW_STOP（电压过低停车）
     * 唯一能离开的条件：电压回升到 STOP_MV + HYS 以上 */
    case BSP_BATT_STATE_LOW_STOP:
        if (mv >= (BSP_BATTERY_STOP_MV + BSP_BATTERY_HYSTERESIS_MV)) {
            return BSP_BATT_STATE_LOW_WARN;  /* 回到告警状态（不会直接回 NORMAL） */
        }
        return BSP_BATT_STATE_LOW_STOP;       /* 仍然过低 */

    /* 当前处于 LOW_WARN（低压告警）
     * 两种可能：电压继续下跌 → LOW_STOP；电压回升 → NORMAL */
    case BSP_BATT_STATE_LOW_WARN:
        if (mv <= BSP_BATTERY_STOP_MV) {
            return BSP_BATT_STATE_LOW_STOP;   /* 跌到停车线 */
        }
        if (mv >= (BSP_BATTERY_WARN_MV + BSP_BATTERY_HYSTERESIS_MV)) {
            return BSP_BATT_STATE_NORMAL;      /* 回升到正常（含回滞） */
        }
        return BSP_BATT_STATE_LOW_WARN;        /* 仍然在告警区间 */

    /* 当前处于 NORMAL（正常）
     * 电压下跌可能直接进入 LOW_STOP（跳过低 WARN）
     * 或先进入 LOW_WARN */
    case BSP_BATT_STATE_NORMAL:
        if (mv <= BSP_BATTERY_STOP_MV) {
            return BSP_BATT_STATE_LOW_STOP;   /* 跌得太厉害，直接停车 */
        }
        if (mv <= BSP_BATTERY_WARN_MV) {
            return BSP_BATT_STATE_LOW_WARN;   /* 进入告警 */
        }
        return BSP_BATT_STATE_NORMAL;          /* 仍然正常 */

    /* 当前处于 UNKNOWN（未知）或未定义状态
     * 首次跨越断线阈值后，按当前电压直接进入对应分类 */
    case BSP_BATT_STATE_UNKNOWN:
    default:
        if (mv <= BSP_BATTERY_STOP_MV)  return BSP_BATT_STATE_LOW_STOP;
        if (mv <= BSP_BATTERY_WARN_MV)  return BSP_BATT_STATE_LOW_WARN;
        return BSP_BATT_STATE_NORMAL;
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/* ----------------------------------------------------------------
 * bsp_battery_init() —— 初始化
 * ----------------------------------------------------------------
 * 清空状态、清除中断标志、触发第一次转换。
 *
 * 为什么要在 init 中就触发第一次转换？
 *   因为 ADC 转换需要时间（约 5 µs）。
 *   如果在 init 中触发，等第一次 update() 调用时（10 ms 后），
 *   转换早就完成了，可以直接读结果。
 *   否则第一次 update() 要先触发再等待——多浪费 5 µs。
 */
void bsp_battery_init(void)
{
    /* 清空所有状态 */
    s_batt.ema_mv   = 0u;       /* EMA 值 = 0（第一次 update 会覆盖） */
    s_batt.last_raw = 0u;       /* 上次 RAW = 0 */
    s_batt.primed   = 0u;       /* 首拍标志 = 0（还没采到第一次数据） */
    s_batt.state    = BSP_BATT_STATE_UNKNOWN;  /* 初始状态 = 未知 */

    /* 清除可能残留的 ADC 中断标志 */
    DL_ADC12_clearInterruptStatus(ADC_BAT_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    /* 触发第一次 ADC 转换
     * 这样下次 update() 调用时就能直接读到结果 */
    trigger_conversion();
}

/* ----------------------------------------------------------------
 * bsp_battery_raw_to_mv() —— 原始值转毫伏（纯函数）
 * ----------------------------------------------------------------
 * 这是整个模块最核心的换算函数。
 *
 * 计算路径（两步走）：
 *
 *   第 1 步：ADC 原始值 → ADC 引脚电压（mV）
 *     mv_pin = raw × REF_MV / FULL_SCALE
 *            = raw × 3300 / 4095
 *     例如 raw = 2000 → mv_pin = 2000 × 3300 / 4095 ≈ 1612 mV
 *
 *   第 2 步：ADC 引脚电压 → 电池电压（mV）
 *     v_bat = mv_pin / K（K = 分压比）
 *            = mv_pin × 10000 / DIVIDER_RATIO_X10000
 *            = mv_pin × 10000 / 1803
 *     例如 mv_pin = 1612 → v_bat = 1612 × 10000 / 1803 ≈ 8940 mV
 *
 * 为什么不用浮点数？
 *   全程 uint32_t 整数运算。
 *   mv_pin ≤ 3300 → 乘 10000 = 33,000,000 < 2^32（约 42.9 亿）
 *   所以 uint32_t 不会溢出。
 *
 * @param raw  ADC 原始值（0~4095）
 * @return 电池电压（毫伏）
 */
uint32_t bsp_battery_raw_to_mv(uint16_t raw)
{
    /* 输入保护：raw 不能超过 ADC 满量程 */
    if (raw > BSP_BATTERY_ADC_FULL_SCALE) {
        raw = BSP_BATTERY_ADC_FULL_SCALE;
    }

    /* 第 1 步：raw → 引脚毫伏
     * 先乘后除，保留精度。
     * raw ≤ 4095, REF_MV = 3300
     * 中间乘积 ≤ 4095 × 3300 ≈ 13.5e6 < 2^24，uint32_t 安全 */
    uint32_t mv_pin = ((uint32_t)raw * (uint32_t)BSP_BATTERY_ADC_REF_MV)
                       / (uint32_t)BSP_BATTERY_ADC_FULL_SCALE;

    /* 第 2 步：引脚毫伏 → 电池毫伏
     * mv_pin ≤ 3300 → 乘 10000 = 33e6 < 2^32，uint32_t 安全
     * 除 1803 得到最终电池电压 */
    return (mv_pin * 10000u) / (uint32_t)BSP_BATTERY_DIVIDER_RATIO_X10000;
}

/* ----------------------------------------------------------------
 * bsp_battery_update() —— 周期采样（建议 100 Hz）
 * ----------------------------------------------------------------
 * 每次调用执行一次完整的"采样 → 换算 → 滤波 → 判断"流程。
 *
 * 详细步骤：
 *   1. 等待上一次转换完成（轮询 < 5 µs）
 *   2. 如果超时（异常），跳过本拍，重新触发，等下一拍
 *   3. 读取 MEM0 结果寄存器（ADC 原始值）
 *   4. 调用 bsp_battery_raw_to_mv() 换算为电池电压
 *   5. 如果是第一次采样，直接使用（避免 EMA 从 0 开始慢爬）
 *   6. 否则更新 EMA 滤波值
 *   7. 调用 classify() 判断当前状态
 *   8. 触发下一次 ADC 转换
 */
void bsp_battery_update(void)
{
    /* ---- 步骤 1：等待转换完成 ---- */
    if (!wait_conversion_done()) {
        /* 上一次转换未完成（异常情况）：
         * 触发新的转换，本拍跳过。
         * 等下一拍再来取结果 */
        trigger_conversion();
        return;
    }

    /* ---- 步骤 2：读取 ADC 原始值 ---- */
    /* DL_ADC12_getMemResult() 从 ADC 的结果寄存器 MEM0 中
     * 读取转换完毕的数字值（0~4095 的 12 位结果）。
     * DL_ADC12_MEM_IDX_0 表示 MEM0 通道。 */
    uint16_t raw = (uint16_t)DL_ADC12_getMemResult(ADC_BAT_INST,
                       DL_ADC12_MEM_IDX_0);
    s_batt.last_raw = raw;  /* 保存最近一次原始值（供调试读取） */

    /* ---- 步骤 3：换算为电池电压 ---- */
    uint32_t mv_now = bsp_battery_raw_to_mv(raw);

    /* ---- 步骤 4：EMA 低通滤波 ----
     *
     * 首拍特殊处理：
     *   如果 primed == 0（首拍），直接把 mv_now 设为 EMA 值。
     *   为什么要这样？
     *   因为 EMA 的旧值初始是 0。如果从 0 开始滤波，
     *   需要很多拍才能"爬升"到真实电压（比如 12,000 mV）。
     *   在这期间，classify() 会误判为 LOW_STOP（电压太低）。
     *
     *   所以首拍直接"跳"到测量值，不走滤波。
     *   第二拍开始才正常滤波。
     *
     * EMA 公式（整数实现）：
     *   ema = (α × new + (256-α) × old + 128) >> 8
     *                                    ↑ 四舍五入
     *   >> 8 等价于除以 256。
     *   加 128 实现四舍五入（而不是直接截断）。
     *
     * 示例：α=32, old=10000, new=10100
     *   ema = (32×10100 + 224×10000 + 128) >> 8
     *        = (323200 + 2240000 + 128) >> 8
     *        = 2563328 >> 8
     *        = 10013（新值占 12.5%，旧值占 87.5%） */
    if (s_batt.primed == 0u) {
        s_batt.ema_mv = mv_now;       /* 首拍：直接使用 */
        s_batt.primed = 1u;           /* 标记已就绪 */
    } else {
        uint32_t a = (uint32_t)BSP_BATTERY_EMA_ALPHA_256;  /* α = 32 */
        s_batt.ema_mv = ((a * mv_now) + ((256u - a) * s_batt.ema_mv) + 128u) >> 8;
    }

    /* ---- 步骤 5：阈值状态机判断 ---- */
    s_batt.state = classify(s_batt.ema_mv, s_batt.state);

    /* ---- 步骤 6：触发下一次转换 ---- */
    trigger_conversion();
}

/* ----------------------------------------------------------------
 * 状态查询函数
 * ---------------------------------------------------------------- */

uint32_t bsp_battery_get_mv(void)
{
    return s_batt.ema_mv;
}

uint16_t bsp_battery_get_raw(void)
{
    return s_batt.last_raw;
}

bsp_battery_state_t bsp_battery_get_state(void)
{
    return s_batt.state;
}
