/**
 * @file    ms901m.h
 * @brief   ATK-MS901M 串口姿态传感器 —— 流式二进制协议解析器（C语言移植版）
 *
 * ============================================================
 * 文件作用一句话概括
 * ============================================================
 * 这个文件是 ATK-MS901M 姿态传感器的"翻译官"。
 * 传感器通过串口（UART）不断发送原始二进制数据，
 * 这个模块负责把这些看不懂的字节流翻译成"俯仰角、横滚角、偏航角……"等
 * 有意义的物理量，供平衡控制算法使用。
 *
 * ============================================================
 * 背景知识：什么是 MS901M？为什么需要这个模块？
 * ============================================================
 * MS901M 是一款"串口姿态传感器"（也叫 IMU，惯性测量单元），
 * 内部集成了陀螺仪、加速度计、磁力计、气压计，还自带一颗 MCU 做姿态解算。
 * 它通过 UART 串口以 200 Hz 的频率主动往外发送数据帧，
 * 我们只需要接上串口 RX 引脚，就能不断收到数据。
 *
 * 传感器发送的是"二进制协议"（不是人类可读的文本），
 * 数据格式为：0x55 0x55 <数据ID> <数据长度> <数据内容> <校验和>
 * 每一帧数据都按这个结构组织，本模块就是解析这个结构的。
 *
 * ============================================================
 * 移植说明（从 C++ 移植到 C）
 * ============================================================
 * 原始解析器是用 C++/Qt 写的（Ms901mStreamParser），移植到 C 时做了以下改动：
 *   1. 去掉 Qt 依赖（QByteArray / QList / QString），全部改用静态数组 + 标志位
 *      —— 因为 MSPM0G3507 单片机资源有限，跑不了 Qt
 *   2. 用 float（32位浮点数）替代 double（64位浮点数）
 *      —— Cortex-M0+ 没有 FPU（浮点运算单元），float 运算比 double 快 2~3 倍
 *   3. 解析方式改为"字节级状态机"
 *      —— 来一个字节处理一个字节，不需要先把整帧收完再解析，节省内存
 *   4. 输出结构体化为 ms901m_snapshot_t
 *      —— 直接提供字段化的角度/角速度/温度，而不是原始数组
 *   5. 校验和算法保持一致
 *      —— sum(0x55, 0x55, ID, LEN, DATA[0..N]) & 0xFF，结果与帧尾校验字节比对
 *
 * ============================================================
 * 帧结构详解（非常重要！）
 * ============================================================
 * 每一帧数据严格按照以下格式排列：
 * 
 *   字节索引    内容       说明
 *   ───────    ─────────  ──────────────────────────────────────
 *   0          0x55       帧头同步字节 1（固定值 0x55）
 *   1          0x55       帧头同步字节 2（固定值 0x55）
 *   2          ID         数据帧类型标识（0x01=姿态, 0x03=陀螺+加速度, 0x04=磁力+温度）
 *   3          LEN        数据段长度（表示 DATA 区域有多少个字节）
 *   4..4+LEN-1 DATA       数据内容（int16 小端格式，需要转换）
 *   4+LEN      CHECKSUM   校验和（前面所有字节的和 & 0xFF）
 *
 * 支持的帧类型（ID 值）：
 *   ID 0x01: 姿态角数据     LEN=6   → roll/pitch/yaw (int16 LE / 32768 * 180°)
 *   ID 0x02: 四元数数据     LEN=8   → q0/q1/q2/q3 (int16 LE / 32768)
 *   ID 0x03: 陀螺+加速度    LEN=12  → ax/ay/az/gx/gy/gz (int16 LE)
 *   ID 0x04: 磁力+温度      LEN=8   → mx/my/mz / temp (int16 LE, temp/100 °C)
 *   ID 0x05: 气压+高度      LEN=10  → pressure(int32 Pa) / altitude(int32 / 100 m)
 *
 * 本工程平衡控制只用 0x01（获取俯仰角 pitch）和 0x03（获取角速度 gy），
 * 其他帧类型虽然解析但不进业务逻辑，保留以备将来扩展。
 */

#ifndef MS901M_H                     /* 防止头文件重复包含的保护宏 */
#define MS901M_H                     /* 第一次 #include 时定义此宏，再次包含时跳过 */

#include <stdint.h>                  /* 引入 uint8_t, int16_t, uint32_t 等定宽整型 */
#include <stddef.h>                  /* 引入 size_t（sizeof 返回的类型） */
#include <stdbool.h>                 /* 引入 bool / true / false（C99 标准） */

#ifdef __cplusplus                   /* 如果被 C++ 编译器引用（__cplusplus 宏表示 C++ 环境） */
extern "C" {                         /* 用 C 链接方式导出符号，防止 C++ 名字修饰（name mangling） */
#endif

/**
 * @brief 最近一次解析完成的姿态数据快照（字段化结构体）
 *
 * 这个结构体是"一锤子买卖"的产物：ms901m_feed_bytes() 每解析完一帧数据，
 * 就会更新这个结构体里对应字段的值。上层应用（如平衡控制算法）随时调用
 * ms901m_get_snapshot() 就能拿到最新数据。
 *
 * 量纲选择说明（与 VOFA+ 上位机 / 自平衡控制习惯对齐）：
 *   - 角度：度（°）—— 平衡环控制用的是角度，不是弧度，直接喂 PD 更直观
 *   - 角速度：度/秒（°/s）—— 直接喂 PD 控制器的速率项，无需弧度转换
 *   - 加速度：g（重力加速度倍数）—— 静态时 ax² + ay² + az² ≈ 1.0
 *   - 温度：°C（摄氏度）
 *
 * ⚠️ 注意：这个结构体是"单线程"使用的
 *   本项目的设计里，只有主循环线程会读写这个结构体，
 *   UART 接收中断（ISR）只负责把字节写入环形缓冲区，
 *   不直接操作这个结构体，因此无需加锁。
 */
typedef struct {
    /* ---- 姿态角（来自 ID=0x01 帧） ---- */
    float pitch_deg;        /* 俯仰角 Pitch (°)：绕 Y 轴旋转，前后倾斜。
                             *   抬头为正，低头为负。
                             *   这是自平衡控制最核心的输入量！
                             *   平衡环就是要让 pitch 保持在 0°（即车身直立）。 */

    float roll_deg;         /* 横滚角 Roll (°)：绕 X 轴旋转，左右倾斜。
                             *   本项目的平衡环只用 pitch，roll 暂不参与控制。 */

    float yaw_deg;          /* 偏航角 Yaw (°)：绕 Z 轴旋转，左右转向。
                             *   注意：磁力计容易被电机 PWM 的电磁干扰，
                             *   所以 yaw 数据在电机运转时可能不准。 */

    /* ---- 陀螺仪角速度（来自 ID=0x03 帧） ---- */
    float gx_dps;           /* X 轴角速度 (°/s)：绕 X 轴旋转的速度。
                             *   数值为正 → 逆时针旋转（从 X 轴正方向看）。 */

    float gy_dps;           /* Y 轴角速度 (°/s)：绕 Y 轴旋转的速度。
                             *   这就是"俯仰角速度 pitch_rate"！
                             *   平衡控制的 PD 控制器中，D 项用的就是它。
                             *   有了角速度，PD 就能提前"预判"车体要倒，
                             *   不需要等角度偏差变大才反应。 */

    float gz_dps;           /* Z 轴角速度 (°/s)：绕 Z 轴旋转的速度。
                             *   可用于转向控制。 */

    /* ---- 加速度计（来自 ID=0x03 帧） ---- */
    float ax_g;             /* X 轴加速度 (g)：1g ≈ 9.8 m/s²
                             *   静态放置时，若 Z 轴朝上，ax ≈ 0, ay ≈ 0, az ≈ 1 */

    float ay_g;             /* Y 轴加速度 (g) */

    float az_g;             /* Z 轴加速度 (g) */

    /* ---- 温度（来自 ID=0x04 帧） ---- */
    float temp_c;           /* 传感器内部温度 (°C)
                             *   陀螺仪有温漂，温度变化会影响角速度精度。
                             *   高阶应用中可以用温度做补偿。 */

    /* ---- 有效标志位 ---- */
    bool  has_attitude;     /* 是否至少收到过一帧 0x01 姿态数据？
                             *   false = 还没收到过，此时 pitch/roll/yaw 都是 0
                             *   主控启动时通过检查这个标志判断 IMU 是否在线 */

    bool  has_gyro_acc;     /* 是否至少收到过一帧 0x03 陀螺+加速度数据？ */

    bool  has_mag_temp;     /* 是否至少收到过一帧 0x04 磁力+温度数据？ */

} ms901m_snapshot_t;        /* typedef 定义结束，以后可以直接用 ms901m_snapshot_t 定义变量 */

/* ============================================================
 * 上位机（电脑）向 MS901M 发送命令时的帧头常量
 * ============================================================
 * 传感器数据上报是"主动"的（上电就自动往外发），
 * 但我们也需要"被动"地给它发命令，比如：改变量程、改波特率、校准传感器。
 * 发给传感器的命令帧，帧头不是 0x55 0x55，而是 0x55 0xAF。
 * 下面这两个宏就是命令帧的同步头。
 */
#define MS901M_CMD_SYNC1          0x55u  /* 命令帧同步字节 1：固定 0x55 */
#define MS901M_CMD_SYNC2          0xAFu  /* 命令帧同步字节 2：固定 0xAF（注意！不是 0x55） */

/**
 * 命令帧最大长度估算：
 *   2 字节（帧头 0x55 0xAF）
 * + 1 字节（命令 ID）
 * + 1 字节（数据长度 LEN）
 * + 32 字节（数据体 DATA，留足扩展余量）
 * + 1 字节（校验和 CHECKSUM）
 * = 37 字节
 *
 * 这个值是给上层应用分配缓冲区时参考用的，保证缓冲区不会溢出。
 */
#define MS901M_CMD_FRAME_MAX      37u

/* ============================================================
 * 枚举类型（enum）：把数字编号映射成有意义的名称
 * ============================================================
 * 枚举是 C 语言中让代码更"自文档化"的重要手段。
 * 如果代码里全是裸数字 0x00, 0x01...，过两周自己都看不懂。
 * 用枚举名称（如 MS901M_CMD_SAVE）代替数字，可读性大大提升。
 * 编译器会把枚举常量替换为对应的整数值，运行时无额外开销。
 */

/**
 * @brief MS901M 全部可用的寄存器/指令 ID（命令码）
 *
 * 每个枚举成员对应一条可以发给传感器的指令。
 * 枚举值的数字就是要在命令帧 ID 字段里填的值。
 *
 * 上位机发送命令时，帧结构为：
 *   0x55 0xAF <枚举值> <LEN> <DATA> <CHECKSUM>
 *
 * 如果是要"读取"寄存器（不是写入），需要在枚举值上 OR 0x80
 * （即最高位置 1），具体由 ms901m_build_read_cmd() 处理。
 */
typedef enum {
    MS901M_CMD_SAVE        = 0x00u,  /* 保存当前配置到 Flash（掉电不丢失） */
    MS901M_CMD_SENCAL      = 0x01u,  /* 传感器校准（加速度/磁力计/气压计） */
    MS901M_CMD_SENSTA      = 0x02u,  /* 读取传感器校准状态 */
    MS901M_CMD_GYROFSR     = 0x03u,  /* 设置陀螺仪量程（满量程范围） */
    MS901M_CMD_ACCFSR      = 0x04u,  /* 设置加速度计量程 */
    MS901M_CMD_GYROBW      = 0x05u,  /* 设置陀螺仪带宽（滤波频率） */
    MS901M_CMD_ACCBW       = 0x06u,  /* 设置加速度计带宽 */
    MS901M_CMD_BAUD        = 0x07u,  /* 设置串口波特率 */
    MS901M_CMD_RETURNSET   = 0x08u,  /* 设置主动上报内容（哪些帧类型要发） */
    MS901M_CMD_RETURNSET2  = 0x09u,  /* 设置主动上报内容 2（扩展） */
    MS901M_CMD_RETURNRATE  = 0x0Au,  /* 设置主动上报频率（如 200 Hz, 100 Hz...） */
    MS901M_CMD_ALG         = 0x0Bu,  /* 设置姿态解算算法（6轴/9轴） */
    MS901M_CMD_ASM         = 0x0Cu,  /* 设置安装方向（水平/垂直） */
    MS901M_CMD_GAUCAL      = 0x0Du,  /* 设置陀螺仪自校准（上电自动校准） */
    MS901M_CMD_BAUCAL      = 0x0Eu,  /* 设置气压计自校准 */
    MS901M_CMD_LEDOFF      = 0x0Fu,  /* LED 开关（0=开，1=关——寄存器名叫 LEDOFF 有点反直觉） */
    MS901M_CMD_D0MODE      = 0x10u,  /* 设置 D0 端口模式 */
    MS901M_CMD_D1MODE      = 0x11u,  /* 设置 D1 端口模式 */
    MS901M_CMD_D2MODE      = 0x12u,  /* 设置 D2 端口模式 */
    MS901M_CMD_D3MODE      = 0x13u,  /* 设置 D3 端口模式 */
    MS901M_CMD_D1PULSE     = 0x16u,  /* 设置 D1 PWM 高电平脉宽（单位微秒 us） */
    MS901M_CMD_D3PULSE     = 0x1Au,  /* 设置 D3 PWM 高电平脉宽 */
    MS901M_CMD_D1PERIOD    = 0x1Fu,  /* 设置 D1 PWM 周期（单位微秒 us） */
    MS901M_CMD_D3PERIOD    = 0x23u,  /* 设置 D3 PWM 周期 */
    MS901M_CMD_RESET       = 0x7Fu   /* 恢复出厂设置（软复位） */
} ms901m_cmd_id_t;

/* ============================================================
 * 下面是各命令对应的参数取值枚举
 * ============================================================
 * 当我们要给传感器发"设置陀螺仪量程为 500°/s"这样的命令时，
 * 需要告诉传感器一个数值（如 0x01 代表 500 dps）。
 * 这些枚举就把"0x01"这个名字化了，写代码时用 MS901M_GYRO_FSR_500DPS 更清晰。
 */

/** 传感器校准类型选择 */
typedef enum {
    MS901M_SENCAL_ACC      = 0x00u,  /* 校准加速度计（需要水平静置） */
    MS901M_SENCAL_MAG      = 0x01u,  /* 校准磁力计（需要旋转 360°） */
    MS901M_SENCAL_BARO_ZERO = 0x02u  /* 气压计归零校准 */
} ms901m_sencal_t;

/** 陀螺仪满量程范围选择（值越大，能测量的最大角速度越大，但精度越低） */
typedef enum {
    MS901M_GYRO_FSR_250DPS  = 0x00u,  /* ±250 °/s   —— 精度高，适合慢速旋转 */
    MS901M_GYRO_FSR_500DPS  = 0x01u,  /* ±500 °/s   —— 适中 */
    MS901M_GYRO_FSR_1000DPS = 0x02u,  /* ±1000 °/s  —— 较大量程 */
    MS901M_GYRO_FSR_2000DPS = 0x03u   /* ±2000 °/s  —— 量程最大，精度最低，本项目默认值 */
} ms901m_gyro_fsr_t;

/** 加速度计满量程范围选择 */
typedef enum {
    MS901M_ACC_FSR_2G  = 0x00u,  /* ±2g  —— 精度最高，适合静态倾角测量 */
    MS901M_ACC_FSR_4G  = 0x01u,  /* ±4g  —— 适中，本项目默认值 */
    MS901M_ACC_FSR_8G  = 0x02u,  /* ±8g  —— 较大量程 */
    MS901M_ACC_FSR_16G = 0x03u   /* ±16g —— 量程最大，适合剧烈运动 */
} ms901m_acc_fsr_t;

/** 串口波特率选择 */
typedef enum {
    MS901M_BAUD_921600 = 0x00u,  /* 921600 bps —— 最快的，适合大量数据传输 */
    MS901M_BAUD_460800 = 0x01u,  /* 460800 bps */
    MS901M_BAUD_256000 = 0x02u,  /* 256000 bps */
    MS901M_BAUD_230400 = 0x03u,  /* 230400 bps */
    MS901M_BAUD_115200 = 0x04u,  /* 115200 bps —— 最常用的标准波特率，本项目用这个 */
    MS901M_BAUD_57600  = 0x05u,  /* 57600 bps */
    MS901M_BAUD_38400  = 0x06u,  /* 38400 bps */
    MS901M_BAUD_19200  = 0x07u,  /* 19200 bps */
    MS901M_BAUD_9600   = 0x08u,  /* 9600 bps —— 最慢的，但最稳定 */
    MS901M_BAUD_4800   = 0x09u,  /* 4800 bps */
    MS901M_BAUD_2400   = 0x0Au   /* 2400 bps —— 极慢，仅极端情况使用 */
} ms901m_baud_t;

/** 主动上报频率选择 */
typedef enum {
    MS901M_RETURN_RATE_250HZ = 0x00u,  /* 250 Hz = 每 4 毫秒一帧，最快 */
    MS901M_RETURN_RATE_200HZ = 0x01u,  /* 200 Hz = 每 5 毫秒一帧，本项目默认值 */
    MS901M_RETURN_RATE_125HZ = 0x02u,  /* 125 Hz = 每 8 毫秒一帧 */
    MS901M_RETURN_RATE_100HZ = 0x03u,  /* 100 Hz = 每 10 毫秒一帧 */
    MS901M_RETURN_RATE_50HZ  = 0x04u,  /* 50 Hz  = 每 20 毫秒一帧 */
    MS901M_RETURN_RATE_20HZ  = 0x05u,  /* 20 Hz  = 每 50 毫秒一帧 */
    MS901M_RETURN_RATE_10HZ  = 0x06u,  /* 10 Hz  = 每 100 毫秒一帧 */
    MS901M_RETURN_RATE_5HZ   = 0x07u,  /* 5 Hz   = 每 200 毫秒一帧 */
    MS901M_RETURN_RATE_2HZ   = 0x08u,  /* 2 Hz   = 每 500 毫秒一帧 */
    MS901M_RETURN_RATE_1HZ   = 0x09u   /* 1 Hz   = 每 1 秒一帧，最慢 */
} ms901m_return_rate_t;

/** 姿态解算算法选择 */
typedef enum {
    MS901M_ALG_6_AXIS = 0x00u,  /* 6 轴算法：只用陀螺仪+加速度计，无磁力计参与
                                  *   优点：不受电机 PWM 磁场干扰
                                  *   缺点：偏航角（yaw）会随时间漂移 */
    MS901M_ALG_9_AXIS = 0x01u   /* 9 轴算法：陀螺仪+加速度计+磁力计融合
                                  *   优点：偏航角有绝对参考（地磁场），不漂移
                                  *   缺点：磁力计易受电机/周围铁磁物质干扰 */
} ms901m_alg_t;

/** 传感器安装方向选择 */
typedef enum {
    MS901M_ASM_HORIZONTAL = 0x00u,  /* 水平安装（芯片面朝上） */
    MS901M_ASM_VERTICAL   = 0x01u   /* 垂直安装（芯片面朝前） */
} ms901m_asm_t;

/** 通用开关（用于陀螺仪自校准、气压计自校准、LED 等） */
typedef enum {
    MS901M_SWITCH_OFF = 0x00u,  /* 关闭 */
    MS901M_SWITCH_ON  = 0x01u   /* 开启 */
} ms901m_switch_t;

/** MS901M 的 D0~D3 端口工作模式 */
typedef enum {
    MS901M_PORT_MODE_ANALOG_IN   = 0x00u,  /* 模拟输入 */
    MS901M_PORT_MODE_DIGITAL_IN  = 0x01u,  /* 数字输入 */
    MS901M_PORT_MODE_DIGITAL_HI  = 0x02u,  /* 数字输出-高电平 */
    MS901M_PORT_MODE_DIGITAL_LO  = 0x03u,  /* 数字输出-低电平 */
    MS901M_PORT_MODE_PWM_OUT     = 0x04u   /* PWM 输出 */
} ms901m_port_mode_t;

/**
 * @brief 主动上报内容选择掩码（位标志）
 *
 * 这个不是枚举值，而是"位掩码"（bitmask）。
 * 工作原理：用一个字节的 8 个位（bit0 ~ bit7）分别代表 8 种数据类型的开关。
 * 如果要把"姿态 + 陀螺加速度"同时打开，就把对应位 OR 起来：
 *   mask = MS901M_RETURN_MASK_ATTITUDE | MS901M_RETURN_MASK_GYRO_ACC;
 * 此时 mask = 0b00000101 = 0x05
 *
 * 位运算的"或操作"（|）让每个位独立控制，可以组合任意多个开关。
 */
typedef enum {
    MS901M_RETURN_MASK_ATTITUDE  = (1u << 0),  /* bit0：是否上报姿态角（ID=0x01） */
    MS901M_RETURN_MASK_QUAT       = (1u << 1),  /* bit1：是否上报四元数（ID=0x02） */
    MS901M_RETURN_MASK_GYRO_ACC   = (1u << 2),  /* bit2：是否上报陀螺+加速度（ID=0x03） */
    MS901M_RETURN_MASK_MAG        = (1u << 3),  /* bit3：是否上报磁力+温度（ID=0x04） */
    MS901M_RETURN_MASK_BARO       = (1u << 4),  /* bit4：是否上报气压+高度（ID=0x05） */
    MS901M_RETURN_MASK_PORT       = (1u << 5),  /* bit5：是否上报端口状态 */
    MS901M_RETURN_MASK_ANON       = (1u << 6)   /* bit6：是否上报匿名数据 */
} ms901m_return_mask_t;

/* ============================================================
 * 公开 API 函数声明
 * ============================================================
 * 以下是被 .c 文件实现、可以被其他 .c 文件调用的函数。
 * 每个函数都有详细的注释说明。
 */

/**
 * @brief  初始化姿态传感器解析器
 *
 * 这个函数必须在主循环启动前调用一次。
 * 它做三件事：
 *   1. 设置加速度计和陀螺仪的量纲换算系数（把原始 int16 值转成物理量）
 *   2. 复位解析状态机（让解析器回到"等待帧头"的初始状态）
 *   3. 清空快照和统计计数
 *
 * @param  acc_fsr_g   加速度计满量程 (g)
 *         这个值必须和 MS901M 传感器当前的配置一致。
 *         如果传感器配置的是 ±4g，这里就传 4。
 *         如果传错，加速度数据会成比例地偏大/偏小。
 *         ATK 出厂默认：±4g → 传 4
 *
 * @param  gyro_fsr_dps 陀螺仪满量程 (°/s)
 *         同样必须和传感器配置一致。
 *         ATK 出厂默认：±2000 dps → 传 2000
 *
 * ⚠️ 常见错误：
 *   如果上位机（ATK 上位机软件）改过传感器的量程，
 *   但这里的参数没改，算出来的角度/角速度会完全错误！
 *   因为同样的原始数值（如 16384），在不同量程下代表的物理量不同。
 */
void ms901m_init(int16_t acc_fsr_g, int16_t gyro_fsr_dps);

/**
 * @brief  把串口接收到的字节"喂"给解析器（核心函数）
 *
 * 这是整个模块最核心的函数，它实现了一个"有限状态机"，
 * 每来一个字节就处理一个字节，逐步完成帧同步、数据接收、校验和验证。
 *
 * 典型调用方式：
 *   // 在主循环中（如每 1 ms 执行一次）：
 *   uint8_t buf[64];
 *   size_t n = bsp_imu_uart_rx_pop_bulk(buf, 64);  // 从串口缓冲区取一批字节
 *   if (n > 0) {
 *       ms901m_feed_bytes(buf, n);  // 喂给解析器
 *   }
 *
 * 状态机工作流程（详见 .c 文件中的实现）：
 *   等待 0x55 → 收到第二个 0x55 → 收 ID → 收 LEN → 收 DATA → 验 CHECKSUM → 回到等待
 *   整个过程中，任何一个环节发现数据不符合预期，就丢弃并重新同步。
 *
 * @param p    指向接收缓冲区的指针，里面是从串口读到的原始字节
 * @param n    本次要处理的字节数量
 *
 * @note  这个函数是 "纯函数"，不依赖任何中断上下文。
 *        它只操作内部静态变量，不访问硬件，因此可以在任意上下文调用。
 *        但建议只在主循环中调用，不要在 UART 中断 ISR 里直接调用，
 *        以保持 ISR 的短小精悍。
 */
void ms901m_feed_bytes(const uint8_t *p, size_t n);

/**
 * @brief  构造一帧"读取寄存器"命令
 *
 * 有些时候，我们需要主动问传感器要某个寄存器的值，
 * 比如"当前陀螺仪量程是多大？"。
 * 这种"读请求"帧和普通的"写设置"帧结构不同：
 * 读命令的 ID 需要把最高位置 1（OR 0x80），且 LEN 固定为 1（数据为 0x00）。
 *
 * @param  cmd_id  要读取的寄存器 ID（7 位编号，不带最高位）
 * @param  out     输出缓冲区（构造好的命令帧会写到这里）
 * @param  out_cap 输出缓冲区容量（至少 6 字节）
 * @return 实际写入的字节数，成功固定为 6；失败返回 0
 *
 * 构造结果示例（假设 cmd_id=0x03 读取陀螺量程）：
 *   out = [0x55, 0xAF, 0x83, 0x01, 0x00, sum]
 *                     ↑ 注意这里 ID=0x83（原值 0x03 | 0x80 = 0x83）
 */
size_t ms901m_build_read_cmd(uint8_t cmd_id, uint8_t *out, size_t out_cap);

/**
 * @brief  构造一帧"写入寄存器"命令
 *
 * 向传感器发送配置命令，比如"把你的量程改为 500 dps"。
 *
 * @param  cmd_id   目标寄存器 ID
 * @param  data     要写入的数据指针；如果 data_len=0 可以传 NULL
 * @param  data_len 数据长度（字节数）
 * @param  out      输出缓冲区
 * @param  out_cap  缓冲区容量（至少 5 + data_len 字节）
 * @return 实际帧长度（= 5 + data_len），失败返回 0
 *
 * 构造结果示例（假设设置加速度计量程为 ±8g）：
 *   cmd_id = 0x04 (ACCFSR), data = [0x02], data_len = 1
 *   out = [0x55, 0xAF, 0x04, 0x01, 0x02, sum]
 */
size_t ms901m_build_write_cmd(uint8_t cmd_id, const uint8_t *data, uint8_t data_len,
    uint8_t *out, size_t out_cap);

/* ============================================================
 * 便捷命令构造函数
 * ============================================================
 * 下面这些函数是对 ms901m_build_write_cmd() 的"语法糖"封装。
 * 它们把常见操作（如"保存配置""设置量程""改波特率"）封装成单个函数调用，
 * 调用方不用再关心 data 的组织方式。
 *
 * 每个函数都执行相同的三步：
 *   1. 确定 cmd_id（命令号，已在枚举中定义）
 *   2. 组织 data（可能为 1 字节或 2 字节）
 *   3. 调用 ms901m_build_write_cmd() 构造完整帧
 */

/** 构造"保存当前配置到 Flash"的命令
 *  执行后，当前所有配置（量程、波特率等）会被写入 Flash，
 *  下次上电自动加载。如果不保存，断电后就恢复成旧值。 */
size_t ms901m_build_save_cmd(uint8_t *out, size_t out_cap);

/** 构造"恢复出厂设置"的命令 */
size_t ms901m_build_reset_cmd(uint8_t *out, size_t out_cap);

/** 构造"传感器校准"的命令
 *  @param cal  指定要校准哪个传感器（加速度计/磁力计/气压计） */
size_t ms901m_build_sensor_cal_cmd(ms901m_sencal_t cal, uint8_t *out, size_t out_cap);

/** 构造"设置陀螺仪量程"的命令 */
size_t ms901m_build_set_gyro_fsr_cmd(ms901m_gyro_fsr_t fsr, uint8_t *out, size_t out_cap);

/** 构造"设置加速度计量程"的命令 */
size_t ms901m_build_set_acc_fsr_cmd(ms901m_acc_fsr_t fsr, uint8_t *out, size_t out_cap);

/** 构造"设置串口波特率"的命令
 *  ⚠️ 注意：改完波特率后，传感器会立刻以新波特率通信。
 *  单片机的 UART 配置也需要同步改变，否则握手失败。
 *  建议发送命令后，延时 > 10 ms，再重新初始化 UART。 */
size_t ms901m_build_set_baud_cmd(ms901m_baud_t baud, uint8_t *out, size_t out_cap);

/** 构造"设置主动上报内容"的命令
 *  @param mask  位掩码，可组合多个位（如 ATTITUDE | GYRO_ACC）
 *  例：想只收姿态和陀螺数据，传 mask = 0x05（bit0 + bit2） */
size_t ms901m_build_set_return_mask_cmd(uint8_t mask, uint8_t *out, size_t out_cap);

/** 构造"设置主动上报频率"的命令 */
size_t ms901m_build_set_return_rate_cmd(ms901m_return_rate_t rate, uint8_t *out, size_t out_cap);

/** 构造"设置姿态解算算法"的命令
 *  6 轴：只融合陀螺和加速度计，yaw 会漂移但不受磁场干扰
 *  9 轴：融合陀螺+加速度计+磁力计，yaw 稳定但易受干扰 */
size_t ms901m_build_set_alg_cmd(ms901m_alg_t alg, uint8_t *out, size_t out_cap);

/** 构造"设置安装方向"的命令 */
size_t ms901m_build_set_asm_cmd(ms901m_asm_t asm_mode, uint8_t *out, size_t out_cap);

/** 构造"设置陀螺仪自校准开关"的命令 */
size_t ms901m_build_set_gaucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap);

/** 构造"设置气压计自校准开关"的命令 */
size_t ms901m_build_set_baucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap);

/** 构造"设置 LED 开关"的命令
 *  ⚠️ 注意寄存器名叫 LEDOFF（关灯寄存器），命名有点反直觉：
 *     传 MS901M_SWITCH_ON → LED 关闭（灯灭）
 *     传 MS901M_SWITCH_OFF → LED 开启（灯亮） */
size_t ms901m_build_set_ledoff_cmd(ms901m_switch_t led_off, uint8_t *out, size_t out_cap);

/** 构造"设置 D0~D3 端口模式"的命令
 *  @param port_index 端口号 0~3
 *  @param mode       工作模式（模拟输入/数字输入/PWM 输出等） */
size_t ms901m_build_set_port_mode_cmd(uint8_t port_index, ms901m_port_mode_t mode,
    uint8_t *out, size_t out_cap);

/** 构造"设置 D1/D3 PWM 高电平脉宽"的命令
 *  @param port_index 端口号（仅 1 或 3 支持 PWM 输出）
 *  @param pulse_us   高电平脉宽（微秒 us）
 *  例：50 Hz 的 PWM，周期 20000 us，脉宽 1500 us 对应中位 */
size_t ms901m_build_set_pwm_pulse_cmd(uint8_t port_index, uint16_t pulse_us,
    uint8_t *out, size_t out_cap);

/** 构造"设置 D1/D3 PWM 周期"的命令
 *  @param port_index 端口号（仅 1 或 3 支持 PWM 输出）
 *  @param period_us  周期（微秒 us）
 *  例：50 Hz → period = 20000 us；100 Hz → 10000 us */
size_t ms901m_build_set_pwm_period_cmd(uint8_t port_index, uint16_t period_us,
    uint8_t *out, size_t out_cap);

/* ============================================================
 * 本地量程同步函数
 * ============================================================
 * 当我们通过上位机（或命令）改变了传感器的量程配置后，
 * 解析器内部的换算系数（s_acc_scale / s_gyro_scale）也需要同步更新，
 * 否则用旧系数换算新数据，结果就是错的。
 *
 * 这两个函数就是做这件事的：
 * 把寄存器的枚举值（如 MS901M_ACC_FSR_8G = 0x02）
 * 翻译成对应的换算系数（如 8.0 / 32768.0）。
 */

/**
 * @brief 把加速度计量程选择值同步为本地换算系数
 *
 * 使用场景举例：
 *   1. 通过 ms901m_build_set_acc_fsr_cmd() 告诉传感器"改为 ±8g 量程"
 *   2. 传感器返回"已确认"（或者我们自己记录已发送）
 *   3. 调用 ms901m_apply_acc_fsr(MS901M_ACC_FSR_8G)
 *   4. 解析器内部的 s_acc_scale 从 4/32768 变为 8/32768
 *   5. 之后收到的加速度数据就能正确换算
 *
 * @param fsr  新的加速度计量程枚举值
 * @return true  = 设置成功；false = 传入的枚举值无效
 */
bool ms901m_apply_acc_fsr(ms901m_acc_fsr_t fsr);

/** @brief 把陀螺仪量程选择值同步为本地换算系数。
 *  用法同上。 */
bool ms901m_apply_gyro_fsr(ms901m_gyro_fsr_t fsr);

/* ============================================================
 * 状态查询与数据获取函数
 * ============================================================
 */

/**
 * @brief  查询是否至少收到过一帧 0x01 姿态数据
 *
 * 这个函数用于启动时的 IMU 在线检测。
 * 在 main() 的初始化函数 wait_for_ms901m_attitude() 中就是一直轮询这个函数，
 * 直到它返回 true，说明 MS901M 已经正常工作。
 *
 * 如果长时间（如 500 ms）一直返回 false，大概率是：
 *   1. MS901M 没上电或硬件连接有问题
 *   2. UART 引脚接错或波特率不匹配
 *   3. MS901M 损坏
 *
 * @return true  = 姿态数据已就绪（至少收到了一帧 pitch/roll/yaw）
 *         false = 尚未收到任何姿态数据
 */
bool ms901m_has_attitude(void);

/**
 * @brief  获取最新姿态快照（深拷贝）
 *
 * 调用方传入一个 ms901m_snapshot_t 结构体变量的指针，
 * 函数会把解析器内部的最新数据完整地复制一份过去。
 * 这叫"深拷贝"——拿到的是独立的数据副本，后续即使解析器内部更新了，
 * 也不影响调用方已拿到的数据。
 *
 * 使用示例：
 *   ms901m_snapshot_t snap;
 *   ms901m_get_snapshot(&snap);
 *   printf("Pitch = %.2f°\n", snap.pitch_deg);
 *
 * @param out  指向输出结构体的指针（不能为 NULL）
 *             函数会填充这个结构体的所有字段
 */
void ms901m_get_snapshot(ms901m_snapshot_t *out);

/**
 * @brief  返回累计校验失败的帧数
 *
 * 校验和失败通常意味着串口通信有误码。
 * 如果这个值不断增长，说明：
 *   1. 波特率设置不匹配（数据经常被误码）
 *   2. 串口信号质量差（干扰大、线太长）
 *   3. 传感器或主控的串口硬件有故障
 *
 * 在 1 Hz 的日志输出中监控这个值，可以评估通信链路的质量。
 *
 * @return 累计失败帧数
 */
uint32_t ms901m_bad_frames(void);

/**
 * @brief  返回累计成功解析的帧数
 *
 * 这个值除以工作时长，可以估算出实际的数据接收频率。
 * 例如：运行了 10 秒，good_frames = 2000，
 *       则平均帧率 = 2000/10 = 200 Hz，符合 MS901M 的默认上报频率。
 *
 * @return 累计成功帧数
 */
uint32_t ms901m_good_frames(void);

#ifdef __cplusplus          /* 如果是 C++ 编译器 */
}                            /* 结束 extern "C" 块 */
#endif

#endif /* MS901M_H */        /* 头文件保护宏结束 */
