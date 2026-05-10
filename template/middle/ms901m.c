/**
 * @file    ms901m.c
 * @brief   MS901M 姿态传感器——字节级状态机解析器的 C 语言实现
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * MS901M 是一个"串口姿态传感器"，它通过一根线（UART串口）
 * 不断往外发送数据。但是，它发送的不是我们能直接读懂的文本
 * （比如 "pitch=10.5°"），而是一串"二进制字节流"——
 * 看起来就像一堆乱码：0x55 0x55 0x01 0x06 0x12 0x34 ...
 *
 * 这个文件的任务就是：
 *   把这一串看似乱码的字节，翻译成我们能用的物理量，
 *   比如 pitch = 10.5°（俯仰角）、gy = 200°/s（角速度）。
 *
 * 它用的方法是"有限状态机"——就像工厂流水线上的工人，
 * 每个字节传送带上传过来，工人看一眼当前处于哪个工位，
 * 就知道这个字节应该怎么处理。
 *
 * ============================================================
 * 数据帧结构速查（摘自 ms901m.h，方便看代码时对照）
 * ============================================================
 *   [0x55] [0x55] [ID] [LEN] [DATA...] [CHECKSUM]
 *     ↑       ↑     ↑     ↑       ↑          ↑
 *   同步头1  同步头2 帧类型 数据长度  N个数据字节  校验和
 *
 *   ID=0x01: 姿态角（roll/pitch/yaw），LEN=6
 *   ID=0x03: 陀螺+加速度（ax/ay/az/gx/gy/gz），LEN=12
 *   ID=0x04: 磁力计+温度（mx/my/mz/temp），LEN=8
 *
 * ============================================================
 * 单线程使用说明
 * ============================================================
 * 本模块的所有 static 变量只在以下两个函数中访问：
 *   - ms901m_feed_bytes()   —— 在主循环中调用
 *   - ms901m_get_snapshot() —— 在主循环中调用
 * UART 接收中断（ISR）只负责把收到的字节写入环形缓冲区，
 * 不直接调用本模块的任何函数，因此不需要加锁。
 * 这是嵌入式开发中常见的"中断只存数据，主循环再处理"的设计模式。
 */

/* ============================================================
 * 头文件包含
 * ============================================================
 * #include 的作用是"粘贴"：把别的文件的内容复制到这里。
 * "ms901m.h" 是配套的头文件，里面定义了我们需要的：
 *   - 结构体（ms901m_snapshot_t）
 *   - 枚举（各种命令ID、量程选项等）
 *   - 函数声明
 * 用引号 "" 而不是尖括号 <>，表示这是"我们自己写的头文件"，
 * 不是编译器的标准库头文件。
 */
#include "ms901m.h"

/* ============================================================
 * 宏定义常量
 * ============================================================
 * #define 是"编译预处理指令"——在编译器真正编译代码之前，
 * 预处理器会做单纯的"文本替换"，把所有 MS901M_DATA_MAX 替换成 32u。
 *
 * 为什么用宏而不用变量？
 *   1. 宏不占内存（编译后直接变成数字 32，不需要在 RAM 里存一个变量）
 *   2. 宏是常量，不能被意外修改
 *   3. 命名清晰，比直接写裸数字 32 更容易理解
 *
 * MS901M_DATA_MAX: 数据段(DATA)的最大字节数
 *   MS901M 各帧类型的数据长度：
 *     0x01 帧（姿态角）：6 字节
 *     0x02 帧（四元数）：8 字节
 *     0x03 帧（陀螺+加速度）：12 字节
 *     0x04 帧（磁力+温度）：8 字节
 *     0x05 帧（气压+高度）：10 字节
 *   设置 32 字节作为上限，既够用又节省静态数组的内存开销。
 *   原始 C++ 版本用了 64 字节，这里折半以节省单片机宝贵的 RAM。
 */
#define MS901M_DATA_MAX  32u

/**
 * SYNC_BYTE: 数据帧同步字节（帧头标志）
 * 传感器每帧数据的开头都是两个连续的 0x55。
 * 0x55 的二进制是 0b01010101，这个模式有两大好处：
 *   1. 交替的 0 和 1 有助于接收方做时钟同步（锁相环）
 *   2. 不太容易被干扰信号"碰巧"模仿出来
 * 如果收到不是 0x55 的字节，说明还没对齐帧头，继续等待。
 */
#define SYNC_BYTE        0x55u

/* ============================================================
 * 状态机状态枚举
 * ============================================================
 * 枚举（enum）让数字有名字。编译器会把 ST_SYNC1 替换为 0，
 * ST_SYNC2 替换为 1，以此类推。
 *
 * 这 6 个状态描述了"从接收一个字节到解析完一帧"的完整过程。
 * 形象的比喻——就像一个流水线工人，有 6 个工位：
 *
 *   工位 0（ST_SYNC1）：等第一个 0x55 到来
 *   工位 1（ST_SYNC2）：等第二个 0x55 到来（确认是帧头）
 *   工位 2（ST_ID）：  收帧类型 ID（判断这帧是什么数据）
 *   工位 3（ST_LEN）：  收数据长度 LEN（知道要收多少字节）
 *   工位 4（ST_DATA）： 收 LEN 个数据字节
 *   工位 5（ST_CHECKSUM）：收校验和，验证数据有没有传错
 *
 * 任何时候，工人只在一个工位上工作。
 * 每个字节过来，工人按当前工位的规则处理，
 * 处理完就移动到下一个工位。
 */
typedef enum {
    ST_SYNC1 = 0,   /* 状态 0：等待第一个同步字节 0x55
                     * 初始状态，也是"失步"后回到的状态 */
    ST_SYNC2,       /* 状态 1：等待第二个同步字节 0x55
                     * 收到第一个 0x55 后进入此状态 */
    ST_ID,          /* 状态 2：等待帧类型 ID 字节
                     * 确定这帧数据是什么内容 */
    ST_LEN,         /* 状态 3：等待数据长度 LEN 字节
                     * 知道后面还有多少个数据字节要收 */
    ST_DATA,        /* 状态 4：依次接收 LEN 个数据字节
                     * 把原始数据暂存到 s_data[] 数组中 */
    ST_CHECKSUM     /* 状态 5：等待校验和字节
                     * 验证之前收到的所有字节是否完整无误 */
} parse_state_t;

/* ============================================================
 * 解析器内部静态变量（模块级全局变量）
 * ============================================================
 * 关键字 static 在这里的作用：
 *   这些变量只在当前文件（ms901m.c）中可见，
 *   其他 .c 文件无法访问它们，实现了"信息隐藏"。
 *
 * 为什么用 static 全局变量而不是局部变量？
 *   状态机需要"记住"当前状态——每次调用 ms901m_feed_bytes()
 *   处理一批字节时，必须知道上次处理到了哪个状态。
 *   如果定义成局部变量，每次函数返回后变量就销毁了，
 *   下次调用时一切从头开始，状态机就无法工作。
 *
 * 初始化说明：
 *   所有变量都在定义时就赋了初始值，
 *   这对应了"上电复位后，解析器处于等待帧头状态"的初始条件。
 */

/* 当前状态机状态：一开始处于"等待第一个 0x55"状态 */
static parse_state_t s_state    = ST_SYNC1;

/* 当前正在接收的帧的 ID（帧类型编号），如 0x01=姿态、0x03=陀螺+加速度 */
static uint8_t       s_id       = 0u;

/* 当前帧的数据段长度（即 DATA 区域有多少个字节）
 * 例如 0x01 帧的 LEN=6，表示后面有 6 个数据字节 */
static uint8_t       s_len      = 0u;

/**
 * s_data[]: 当前帧的数据缓冲区
 * 收到的数据字节先暂存在这里，等校验通过后再解析。
 * 缓冲区大小 MS901M_DATA_MAX = 32 字节，足够容纳所有帧类型。
 * 
 * 为什么需要这个缓冲区？
 *   因为收到数据字节时还不知道校验和是否通过。
 *   如果校验失败，这些数据就是无效的，不能更新到快照里。
 *   所以先缓存，等校验通过再统一解析。
 */
static uint8_t       s_data[MS901M_DATA_MAX];

/* s_data_idx: 当前已收到多少个数据字节
 * 从 0 开始计数，每收一个数据字节 +1，
 * 当 s_data_idx == s_len 时，说明数据收完了，准备收校验和 */
static uint8_t       s_data_idx = 0u;

/**
 * s_chk: 累加校验和
 * 
 * 工作原理：
 *   从第一个同步字节 0x55 开始，每收到一个字节就累加：
 *   s_chk = s_chk + 新字节（只取低 8 位，uint8_t 自动溢出）
 *   
 * 到校验和阶段时，收到的校验和字节应该等于 s_chk 的值。
 * 如果相等 → 帧数据完整无误；不相等 → 传输中有错误，丢弃这帧。
 * 
 * uint8_t 溢出特性：
 *   uint8_t 的范围是 0~255，累加超过 255 时会"回绕"。
 *   比如 200 + 100 = 300，但 uint8_t 只能存 300 - 256 = 44。
 *   这和"& 0xFF（只取低 8 位）"的效果是一样的。
 *   这是 C 语言中利用数据类型特性简化代码的技巧。
 */
static uint8_t       s_chk      = 0u;

/* ============================================================
 * 量纲转换系数
 * ============================================================
 * 传感器传过来的原始数据是 int16（16 位有符号整数，范围 -32768~32767），
 * 我们需要把它转换成有实际意义的物理量。
 *
 * 转换公式：
 *   物理量 = 原始值 × (满量程值 / 32768)
 *
 * 为什么是 / 32768？
 *   传感器的 ADC（模数转换器）是 16 位的，
 *   输出范围是 -32768 ~ +32767，
 *   -32768 对应"负满量程"，+32767 对应"正满量程"。
 *   所以 1 个 LSB（最低有效位）对应的物理量 = 满量程 / 32768。
 *
 * 默认值说明：
 *   ATK-MS901M 出厂默认配置：
 *   - 加速度计：±4g 量程（s_acc_scale = 4.0 / 32768.0）
 *   - 陀螺仪：±2000 dps 量程（s_gyro_scale = 2000.0 / 32768.0）
 *
 * ⚠️ 常见错误：
 *   如果通过 ATK 上位机软件修改了传感器的量程，
 *   但没有调用 ms901m_apply_acc_fsr() 同步这里的系数，
 *   那么所有加速度/角速度数据都会按错误的系数换算！
 *   例如：传感器已改为 ±8g，但代码仍用 4/32768 换算，
 *   读出来的加速度值会偏小一半。
 */
static float s_acc_scale  = 4.0f / 32768.0f;     /* 加速度默认 ±4g 量程 */
static float s_gyro_scale = 2000.0f / 32768.0f;  /* 陀螺仪默认 ±2000 dps 量程 */

/* ============================================================
 * 最新快照 + 统计计数
 * ============================================================
 * s_snap: 最新一帧解析完成后的数据快照
 *   所有解析函数都在更新这个结构体。
 *   上层应用（如平衡控制算法）调用 ms901m_get_snapshot()
 *   就能拿到最新数据。
 *
 * s_bad_frames: 累计校验失败的帧数
 *   这个值持续增长 → 通信链路有问题（波特率不匹配？干扰太大？）
 *
 * s_good_frames: 累计成功解析的帧数
 *   除以运行时间 → 实际接收频率（如 10 秒收了 2000 帧 → 200 Hz）
 */
static ms901m_snapshot_t s_snap = { 0 };  /* { 0 } 把所有字段初始化为 0 */
static uint32_t          s_bad_frames  = 0u;  /* 校验失败的帧数统计 */
static uint32_t          s_good_frames = 0u;  /* 校验成功的帧数统计 */

/* ============================================================
 * 小工具函数（内部辅助函数）
 * ============================================================
 * 这里的函数都用 static 修饰，表示"只在本文件内部使用"。
 * 它们不是公开 API，别的 .c 文件不能调用。
 * 这符合"封装"原则——内部实现细节对外部不可见。
 */

/**
 * @brief  le16 —— 小端（Little-Endian）字节序转 int16
 *
 * 功能：把两个分开的字节（低字节 lo、高字节 hi）合并成一个 16 位有符号整数。
 *
 * 为什么需要这个函数？
 *   MS901M 传感器发送的 int16 数据是"小端"格式的：
 *   先发送低 8 位（lo），再发送高 8 位（hi）。
 *   例如角度值 90.0° 对应的 int16 原始值是 16384（0x4000），
 *   传感器会先发 0x00（低字节），再发 0x40（高字节）。
 *   我们需要把 [0x00, 0x40] 重新组合成 0x4000 = 16384。
 *
 * 知识点：小端（Little-Endian） vs 大端（Big-Endian）
 *   小端：低地址存放低字节 → [lo, hi]  → 值 = (hi << 8) | lo
 *   大端：低地址存放高字节 → [hi, lo]  → 值 = (lo << 8) | hi
 *   MS901M 和 MSPM0G3507 都是小端，所以直接用 (hi<<8)|lo 即可。
 *   如果传感器是大端的，同样的算法会读出错乱的值！
 *
 * 语法细节：
 *   inline 关键字：建议编译器把函数体直接嵌入调用处，避免函数调用开销。
 *   对于这种只有 1 条 return 语句的"小函数"，inline 非常合适。
 *   但 inline 只是"建议"，编译器可以忽略。
 *
 * @param lo  低字节（先收到的字节）
 * @param hi  高字节（后收到的字节）
 * @return int16_t  合并后的 16 位有符号整数
 *
 * 示例：
 *   le16(0x00, 0x40) = (int16_t)((0x40 << 8) | 0x00) = (int16_t)(0x4000) = 16384
 *   le16(0xCD, 0xFF) = (int16_t)((0xFF << 8) | 0xCD) = (int16_t)(0xFFCD) = -51
 */
static inline int16_t le16(uint8_t lo, uint8_t hi)
{
    /* ((uint16_t)hi << 8)：先把 hi 转成 uint16_t（16 位无符号），
     * 然后左移 8 位，把 hi 放到高 8 位的位置。
     * 例如 hi=0x40 → (uint16_t)0x0040 << 8 = 0x4000
     * 
     * | (uint16_t)lo：把 lo 放到低 8 位（按位或运算）。
     * 例如 lo=0x00 → 0x4000 | 0x0000 = 0x4000
     * 如果 lo=0x12 → 0x4000 | 0x0012 = 0x4012
     * 
     * (int16_t)(...)：把结果从无符号转成有符号。
     * 如果结果是 0x8000~0xFFFF，按 int16_t 解读就是负数。
     * 
     * 为什么先转 uint16_t 再左移？
     *   如果直接用 hi<<8，hi 是 uint8_t（8 位），
     *   左移 8 位会溢出（uint8_t 最多存 8 位），
     *   所以必须先把 hi 提升到 16 位再移位。 */
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/**
 * @brief  sum_bytes —— 计算一段数据的累加和
 *
 * 功能：对 data 指向的 len 个字节做累加，返回低 8 位结果。
 *
 * 这是 MS901M 协议中"校验和"的生成方式：
 *   发送方：把所有字节（帧头+ID+LEN+DATA）加起来，取低 8 位作为校验字节
 *   接收方：用同样的算法算一遍，和收到的校验字节对比
 *   如果一致 → 传输无误；不一致 → 有数据损坏，丢弃这帧
 *
 * 为什么是"累加和取低 8 位"而不是更复杂的 CRC？
 *   因为简单！嵌入式中，简单意味着：
 *   - 计算速度快（只需要加法和与运算）
 *   - 代码量小（就几行）
 *   当然，检错能力不如 CRC 强，但对付串口误码已经够了。
 *
 * @param data  指向要计算的数据的指针（不能为 NULL）
 * @param len   数据长度（字节数）
 * @return uint8_t  累加和的低 8 位
 *
 * 初学者注意：
 *   这里的 data 和 len 的含义和 s_data/s_len 不同！
 *   data 是"外部传入的任意数据"，s_data 是"帧中接收到的数据段"。
 *   两者的区别是：s_data 只是帧的一部分，而这里的 data 可以是任意数据。
 */
static uint8_t sum_bytes(const uint8_t *data, size_t len)
{
    /* 定义累加和变量，初始化为 0 */
    uint8_t sum = 0u;

    /* 防御性编程：如果传入的指针是 NULL（空指针），直接返回 0。
     * 为什么要检查 NULL？
     *   如果在调用 sum_bytes(NULL, 10) 时不解引用 NULL 指针，
     *   for 循环中的 data[i] 会访问地址 0，导致硬件异常（死机）！
     *   这是嵌入式开发中最常见的崩溃原因之一。
     * 所以，对传入的指针参数永远要做 NULL 检查。 */
    if (data == NULL) {
        return 0u;
    }

    /* 循环遍历每一个字节，累加到 sum 中。
     * sum 是 uint8_t（范围 0~255），累加超出 255 时会自动"回绕"，
     * 效果等价于 (sum + data[i]) & 0xFF。
     * 这是利用了 C 语言无符号整数溢出的确定性行为。
     * 
     * 注意：size_t i = 0u 中的 u 后缀表示"unsigned"（无符号）。
     * size_t 本身就是无符号类型，用 0u 赋值更"纯粹"。
     * 对于初学者，写成 size_t i = 0 也没问题。 */
    for (size_t i = 0u; i < len; ++i) {
        /* sum = (sum + data[i]) & 0xFF 的简化写法
         * uint8_t 自动溢出 → 只保留低 8 位
         * 强转 (uint8_t) 是为了明确告诉编译器和读者：
         * "我就是故意要溢出的，不是写错了" */
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

/**
 * @brief  set_acc_scale_by_sel —— 根据枚举值设置加速度换算系数
 *
 * 功能：把 ms901m_acc_fsr_t 枚举值（如 0x02 代表 ±8g）
 *       翻译成对应的换算系数（如 8.0 / 32768.0）。
 *
 * 使用场景：
 *   当通过命令修改了传感器的加速度计量程后，
 *   调用此函数同步更新解析器内部的 s_acc_scale。
 *
 * 为什么不直接用 ms901m_acc_fsr_t 枚举值做计算？
 *   因为 0x00 对应 2g，0x01 对应 4g，0x02 对应 8g，0x03 对应 16g，
 *   这个映射关系是"2 的幂次增长"，没有简单的数学公式可以一步算出，
 *   所以用 switch-case 做"查表映射"最清晰。
 *
 * @param fsr  加速度计量程的枚举值（2g/4g/8g/16g）
 * @return true  = 设置成功（传入的枚举值有效）
 *         false = 设置失败（传入的枚举值无效，如 0x04）
 */
static bool set_acc_scale_by_sel(uint8_t fsr)
{
    /* switch 语句：根据 fsr 的值跳转到对应的 case 分支。
     * 相当于多个 if-else if 的简化写法，但更清晰、执行效率更高。
     * 编译器通常会把 switch 编译成"跳转表"（O(1) 时间），
     * 而 if-else 链是顺序比较（O(n) 时间）。 */
    switch (fsr) {
        case MS901M_ACC_FSR_2G:   /* 当 fsr == 0x00（±2g 量程） */
            s_acc_scale = 2.0f / 32768.0f;   /* 设置对应系数 */
            return true;                       /* 返回成功 */
        
        case MS901M_ACC_FSR_4G:   /* 当 fsr == 0x01（±4g 量程，出厂默认） */
            s_acc_scale = 4.0f / 32768.0f;
            return true;
        
        case MS901M_ACC_FSR_8G:   /* 当 fsr == 0x02（±8g 量程） */
            s_acc_scale = 8.0f / 32768.0f;
            return true;
        
        case MS901M_ACC_FSR_16G:  /* 当 fsr == 0x03（±16g 量程） */
            s_acc_scale = 16.0f / 32768.0f;
            return true;
        
        default:                  /* 如果 fsr 不在上述 4 个值中 */
            return false;          /* 返回失败，系数保持不变 */
    }
    /* 注意：每个 case 都有 return，不需要 break。
     * 不加 break 会不会"穿透"（fall-through）？
     * 不会，因为 return 已经结束了函数执行。 */
}

/**
 * @brief  set_gyro_scale_by_sel —— 根据枚举值设置陀螺仪换算系数
 *
 * 功能同 set_acc_scale_by_sel()，但针对陀螺仪。
 * 陀螺仪量程选项：250、500、1000、2000 dps。
 *
 * @param fsr  陀螺仪量程的枚举值
 * @return true = 成功；false = 无效枚举值
 */
static bool set_gyro_scale_by_sel(uint8_t fsr)
{
    switch (fsr) {
        case MS901M_GYRO_FSR_250DPS:   /* ±250 °/s */
            s_gyro_scale = 250.0f / 32768.0f;
            return true;
        
        case MS901M_GYRO_FSR_500DPS:   /* ±500 °/s */
            s_gyro_scale = 500.0f / 32768.0f;
            return true;
        
        case MS901M_GYRO_FSR_1000DPS:  /* ±1000 °/s */
            s_gyro_scale = 1000.0f / 32768.0f;
            return true;
        
        case MS901M_GYRO_FSR_2000DPS:  /* ±2000 °/s，出厂默认 */
            s_gyro_scale = 2000.0f / 32768.0f;
            return true;
        
        default:
            return false;
    }
}

/**
 * @brief  build_frame —— 通用命令帧构造函数
 *
 * 功能：根据 MS901M 命令协议，构造一完整的命令帧写入 out 缓冲区。
 *
 * 这是所有 ms901m_build_xxx_cmd() 函数的核心底层函数。
 * 其他构造函数都是调用这个函数来完成实际工作的。
 *
 * 命令帧结构：
 *   [0x55] [0xAF] [CMD_ID] [LEN] [DATA...] [CHECKSUM]
 *    ↑       ↑       ↑       ↑       ↑          ↑
 *  帧头1   帧头2   命令ID  数据长度 数据内容   校验和（前面所有字节累加）
 *
 * 注意：数据帧的帧头是 0x55 0x55，而命令帧的帧头是 0x55 0xAF！
 *   AF 和 55 的区别——0xAF 二进制是 0b10101111，
 *   和 0x55 的 0b01010101 不同，用于区分"这是发给传感器的命令"和
 *   "这是传感器发给我们的数据"。
 *
 * @param cmd_id   命令 ID（来自 ms901m_cmd_id_t 枚举）
 *                 注意：如果是"读命令"，调用方需要先 OR 0x80
 * @param data     要写入的数据指针
 *                 如果 data_len=0 可以传 NULL（比如 RESET 命令不需要数据）
 * @param data_len 数据长度（字节数）
 * @param out      输出缓冲区，构造好的命令帧会写在这里
 * @param out_cap  输出缓冲区容量（字节数），必须 >= 5 + data_len
 * @return size_t  实际写入的字节数（即帧长度），失败返回 0
 *
 * 返回值示例：
 *   设置加速度计量程为 ±8g（cmd_id=0x04, data=[0x02], data_len=1）：
 *   帧长度 = 5 + 1 = 6 字节
 *   out = [0x55, 0xAF, 0x04, 0x01, 0x02, 0x05(校验和)]
 *
 * 安全考虑：
 *   函数会检查 out 是否为 NULL、out_cap 是否足够大，
 *   防止缓冲区溢出——这是嵌入式安全编程的基本要求。
 */
static size_t build_frame(uint8_t cmd_id, const uint8_t *data, uint8_t data_len,
    uint8_t *out, size_t out_cap)
{
    /* 帧总长度 = 5 个固定字节（2 字节帧头 + 1 字节 ID + 1 字节 LEN + 1 字节校验和）
     *           + data_len 个数据字节 */
    size_t frame_len = 5u + (size_t)data_len;

    /* 安全检查：
     * 1. 输出缓冲区必须是有效指针（不能是 NULL）
     * 2. 输出缓冲区必须足够大，能装下整个帧
     * 如果不满足，返回 0 表示构造失败。
     * 这是防止"缓冲区溢出"漏洞的关键检查。
     * 
     * 在嵌入式系统中，缓冲区溢出可能导致：
     * - 覆盖相邻变量（数据错乱）
     * - 覆盖函数返回地址（程序跑飞）
     * - 严重的安全漏洞 */
    if (out == NULL || out_cap < frame_len) {
        return 0u;
    }

    /* 如果有数据要写，但 data 指针是 NULL，说明调用方传参有误。
     * data_len > 0 但 data == NULL 是不合理的组合。 */
    if (data_len > 0u && data == NULL) {
        return 0u;
    }

    /* 开始组装帧 */

    /* out[0] = 命令帧同步字节 1：固定 0x55 */
    out[0] = MS901M_CMD_SYNC1;
    /* out[1] = 命令帧同步字节 2：固定 0xAF（注意！和数据帧的 0x55 不同） */
    out[1] = MS901M_CMD_SYNC2;
    /* out[2] = 命令 ID，告诉传感器要做什么（读/写哪个寄存器） */
    out[2] = cmd_id;
    /* out[3] = 数据长度，告诉传感器后面有多少字节的数据 */
    out[3] = data_len;

    /* 循环拷贝 data 中的数据到帧的 DATA 段。
     * 从 out[4] 开始存放，因为前 4 字节是 帧头+ID+LEN。
     * 
     * 为什么用 for 循环而不是 memcpy()？
     *   为了减少依赖。memcpy() 是 <string.h> 中的库函数，
     *   有些单片机开发环境的标准库可能不完整。
     *   用 for 循环写更"自包含"，不依赖外部库。
     *   但在性能要求高时，memcpy() 可能更快（可能经过汇编优化）。 */
    for (uint8_t i = 0u; i < data_len; ++i) {
        out[4u + i] = data[i];
    }

    /* 计算校验和并放到最后一个字节。
     * sum_bytes(out, frame_len - 1u) 计算 out[0] ~ out[frame_len-2] 的累加和。
     * 校验和字节 = 前面所有字节的和 & 0xFF。
     * 注意：校验和本身不参与计算（否则自己加自己，永远不对）。 */
    out[frame_len - 1u] = sum_bytes(out, frame_len - 1u);

    /* 返回实际帧长度，调用方知道要发送多少字节 */
    return frame_len;
}

/**
 * @brief  port_mode_cmd_id —— 端口号 → D0~D3 模式命令 ID 的映射函数
 *
 * MS901M 有 4 个多功能端口（D0~D3），每个端口都可以单独设置工作模式
 * （模拟输入、数字输入、数字输出、PWM 输出）。
 * 设置端口模式的命令 ID 分别为：
 *   D0 → 0x10 (MS901M_CMD_D0MODE)
 *   D1 → 0x11 (MS901M_CMD_D1MODE)
 *   D2 → 0x12 (MS901M_CMD_D2MODE)
 *   D3 → 0x13 (MS901M_CMD_D3MODE)
 *
 * 这个函数把端口号（0~3）映射到对应的命令 ID。
 * 之所以需要这个映射而不是让调用方直接传命令 ID，
 * 是为了"统一接口"——外部只需要说"我要设置 D0 端口"，
 * 内部自动查表找到对应的命令 ID，降低使用难度。
 *
 * @param port_index  端口号（0=D0, 1=D1, 2=D2, 3=D3）
 * @param cmd_id      输出参数：查找到的命令 ID 会写到这里
 * @return true  = 端口号有效（0~3），cmd_id 被正确设置
 *         false = 端口号无效（≥4），cmd_id 不变
 */
static bool port_mode_cmd_id(uint8_t port_index, uint8_t *cmd_id)
{
    /* 防御性编程：检查输出指针是否为 NULL */
    if (cmd_id == NULL) {
        return false;
    }

    /* switch 实现端口号到命令 ID 的映射 */
    switch (port_index) {
        case 0u: *cmd_id = MS901M_CMD_D0MODE; return true;  /* port 0 → D0MODE */
        case 1u: *cmd_id = MS901M_CMD_D1MODE; return true;  /* port 1 → D1MODE */
        case 2u: *cmd_id = MS901M_CMD_D2MODE; return true;  /* port 2 → D2MODE */
        case 3u: *cmd_id = MS901M_CMD_D3MODE; return true;  /* port 3 → D3MODE */
        default: return false;  /* 无效端口号，返回失败 */
    }
}

/**
 * @brief  pwm_pulse_cmd_id —— 端口号 → PWM 脉宽命令 ID 的映射函数
 *
 * MS901M 的 D1 和 D3 端口可以输出 PWM 信号。
 * 设置脉宽（高电平持续时间）的命令 ID 分别为：
 *   D1 → 0x16 (MS901M_CMD_D1PULSE)
 *   D3 → 0x1A (MS901M_CMD_D3PULSE)
 *
 * 注意：D0 和 D2 不支持 PWM 输出，所以映射表里只有 1 和 3。
 *
 * @param port_index  端口号（仅 1 或 3 支持 PWM 脉宽设置）
 * @param cmd_id      输出参数：查找到的命令 ID
 * @return true  = 端口支持 PWM 脉宽设置；false = 不支持
 */
static bool pwm_pulse_cmd_id(uint8_t port_index, uint8_t *cmd_id)
{
    if (cmd_id == NULL) {
        return false;
    }

    switch (port_index) {
        case 1u: *cmd_id = MS901M_CMD_D1PULSE; return true;  /* D1 的 PWM 脉宽命令 */
        case 3u: *cmd_id = MS901M_CMD_D3PULSE; return true;  /* D3 的 PWM 脉宽命令 */
        default: return false;  /* D0 和 D2 不支持 PWM */
    }
}

/**
 * @brief  pwm_period_cmd_id —— 端口号 → PWM 周期命令 ID 的映射函数
 *
 * 和 pwm_pulse_cmd_id() 类似，但设置的是 PWM 的"周期"（整个脉冲的时长），
 * 而不是"脉宽"（高电平时长）。
 *
 * 周期 = 高电平时间 + 低电平时间，单位微秒（us）。
 * 例如 50 Hz 的 PWM：周期 = 1/50 = 0.02 秒 = 20000 us。
 *
 * @param port_index  端口号（仅 1 或 3 支持 PWM 周期设置）
 * @param cmd_id      输出参数：查找到的命令 ID
 * @return true  = 成功；false = 端口不支持
 */
static bool pwm_period_cmd_id(uint8_t port_index, uint8_t *cmd_id)
{
    if (cmd_id == NULL) {
        return false;
    }

    switch (port_index) {
        case 1u: *cmd_id = MS901M_CMD_D1PERIOD; return true;
        case 3u: *cmd_id = MS901M_CMD_D3PERIOD; return true;
        default: return false;
    }
}

/**
 * @brief  reset_state_machine —— 复位状态机到初始状态
 *
 * 状态机复位意味着：
 *   1. 回到 ST_SYNC1（等待帧头的第一个 0x55）
 *   2. 校验和清零（重新开始累加）
 *   3. 数据索引清零（重新收数据）
 *
 * 什么时候会调用这个函数？
 *   1. ms901m_init() 初始化时
 *   2. 收到非法字节（不该出现 0x55 的地方出现 0x55 等）
 *   3. 帧长度异常（LEN 超过上限）
 *   4. 一帧处理完毕（无论校验成功还是失败），准备收下一帧
 *
 * 简单理解：就像卡住了就"重启流水线"，从头开始。
 */
static void reset_state_machine(void)
{
    s_state    = ST_SYNC1;  /* 回到"等待第一个 0x55"的状态 */
    s_chk      = 0u;         /* 校验和累加器清零 */
    s_data_idx = 0u;         /* 数据接收指针清零 */
}

/* ============================================================
 * 各帧解析函数
 * ============================================================
 * 当状态机成功收完一帧数据并且校验和验证通过后，
 * dispatch_frame() 会根据帧 ID 调用对应的解析函数。
 *
 * 每个解析函数做的事：
 *   1. 用 le16() 把两个字节合成 int16 原始值
 *   2. 乘以量纲换算系数，得到物理量（度、°/s、g、°C）
 *   3. 更新到 s_snap 结构体的对应字段
 *   4. 设置对应的"已收到"标志位
 *
 * 注意：这里接收的 d（data）和 len 是已经校验通过的，
 * 所以不需要再做格式检查（只做长度断言）。
 */

/**
 * @brief  parse_attitude —— 解析 0x01 姿态角帧
 *
 * 数据格式（6 字节，小端）：
 *   d[0..1]：横滚角 Roll 的 int16 原始值
 *   d[2..3]：俯仰角 Pitch 的 int16 原始值  ← 这是平衡控制最关心的！
 *   d[4..5]：偏航角 Yaw 的 int16 原始值
 *
 * 换算公式：角度 = int16原始值 × (180° / 32768)
 *
 * 为什么是 180° 而不是 360°？
 *   因为 MS901M 的姿态角输出范围是 -180° ~ +180°，
 *   正好对应 int16 的 -32768 ~ +32767。
 *   32768 对应 180°，所以系数是 180/32768。
 *
 * 对平衡控制的意义：
 *   pitch_deg（俯仰角）是平衡环最核心的输入。
 *   当小车直立时，pitch ≈ 0°。
 *   如果车体前倾，pitch 为正；后仰，pitch 为负。
 *   PD 控制器根据 pitch 的大小和方向决定电机的转动方向和速度。
 *
 * @param d   数据段指针（6 字节）
 * @param len 数据长度（期望为 6，否则不处理）
 */
static void parse_attitude(const uint8_t *d, uint8_t len)
{
    /* 长度检查：0x01 帧的数据段固定为 6 字节。
     * 如果 len != 6，说明帧结构异常，直接 return 不处理。
     * 这是"防御性编程"的又一体现——不信任外部数据的合法性。 */
    if (len != 6u) { return; }

    /* 换算横滚角 Roll：int16原始值 × (180/32768)
     * 对自平衡小车来说，roll 暂不参与控制，但保留以备将来扩展
     * （比如检测是否侧翻）。 */
    s_snap.roll_deg  = (float)le16(d[0], d[1]) * (180.0f / 32768.0f);

    /* 换算俯仰角 Pitch：⭐⭐⭐ 最重要的数据！⭐⭐⭐
     * 这是平衡控制 PD 控制器的 P（比例）项输入。
     * le16(d[2], d[3]) 把两个字节合成 int16，
     * (float) 强转成 float 后进行浮点运算。
     * 乘以 (180.0f / 32768.0f) = 0.005493 得到角度值。 */
    s_snap.pitch_deg = (float)le16(d[2], d[3]) * (180.0f / 32768.0f);

    /* 换算偏航角 Yaw
     * 注意：yaw 在有电机运转时可能受磁力计干扰而不准确。
     * 本项目不依赖 yaw 做平衡控制，仅供参考。 */
    s_snap.yaw_deg   = (float)le16(d[4], d[5]) * (180.0f / 32768.0f);

    /* 设置标志位：告诉上层应用"我已经成功收到过姿态数据了"
     * ms901m_has_attitude() 就是检查这个标志位。
     * 这个标志一旦被设为 true，不会再变回 false。 */
    s_snap.has_attitude = true;
}

/**
 * @brief  parse_gyro_acc —— 解析 0x03 陀螺仪+加速度计帧
 *
 * 数据格式（12 字节，小端）：
 *   d[0..1]：  X 轴加速度 ax（int16 原始值）
 *   d[2..3]：  Y 轴加速度 ay
 *   d[4..5]：  Z 轴加速度 az
 *   d[6..7]：  X 轴角速度 gx
 *   d[8..9]：  Y 轴角速度 gy（= pitch_rate，平衡控制的 D 项输入）
 *   d[10..11]：Z 轴角速度 gz
 *
 * 换算公式：
 *   加速度 (g) = int16原始值 × (acc_fsr / 32768)
 *   角速度 (°/s) = int16原始值 × (gyro_fsr / 32768)
 *
 * @param d   数据段指针（12 字节）
 * @param len 数据长度（期望为 12）
 */
static void parse_gyro_acc(const uint8_t *d, uint8_t len)
{
    /* 长度检查：0x03 帧固定为 12 字节 */
    if (len != 12u) { return; }

    /* ---- 加速度换算（单位：g）----
     * s_acc_scale 的默认值是 4.0 / 32768.0（±4g 量程）。
     * 如果传感器配置为 ±8g，需要先调用 ms901m_apply_acc_fsr()
     * 修改 s_acc_scale，否则这里的换算结果会偏小一半。
     * 
     * 平衡控制中，加速度数据主要用于：
     * 1. 静态倾角计算（互补滤波中与陀螺积分互补）
     * 2. 检测车体是否跌倒（az 突然大幅变化） */
    s_snap.ax_g = (float)le16(d[0], d[1]) * s_acc_scale;
    s_snap.ay_g = (float)le16(d[2], d[3]) * s_acc_scale;
    s_snap.az_g = (float)le16(d[4], d[5]) * s_acc_scale;

    /* ---- 角速度换算（单位：°/s）----
     * gy_dps（Y 轴角速度）= pitch_rate（俯仰角速度）
     * 这是平衡控制 PD 控制器的 D（微分）项输入！
     * 
     * P（比例）项：根据"当前偏离了多少"来决定纠正力度
     * D（微分）项：根据"当前倒得有多快"来提前预判和抑制
     * 
     * 例子：如果车体正在快速前倾（gy_dps 是正的大值），
     * D 项会输出一个相反的力矩来"刹车"，防止车体冲过头。
     * 没有 D 项，平衡环会来回震荡，无法稳定。 */
    s_snap.gx_dps = (float)le16(d[6],  d[7])  * s_gyro_scale;
    s_snap.gy_dps = (float)le16(d[8],  d[9])  * s_gyro_scale;
    s_snap.gz_dps = (float)le16(d[10], d[11]) * s_gyro_scale;

    /* 设置陀螺+加速度数据已接收标志 */
    s_snap.has_gyro_acc = true;
}

/**
 * @brief  parse_mag_temp —— 解析 0x04 磁力计+温度帧
 *
 * 数据格式（8 字节，小端）：
 *   d[0..1]：磁力计 X（本工程不用）
 *   d[2..3]：磁力计 Y（本工程不用）
 *   d[4..5]：磁力计 Z（本工程不用）
 *   d[6..7]：温度 int16 原始值，公式：temp_c = 原始值 / 100
 *
 * 为什么本工程不用磁力计数据？
 *   因为电机 PWM 驱动电流会产生强电磁场，
 *   严重干扰磁力计读数。如果用磁力计做 9 轴融合，
 *   偏航角会在电机加速时剧烈跳变。
 *   所以本项目只用 6 轴（陀螺+加速度），放弃磁力计。
 *
 * 温度数据有什么用？
 *   陀螺仪有"温漂"特性——温度变化时零偏会缓慢变化。
 *   高级应用中可以用温度做零偏补偿。
 *   本工程暂不实现，但保留了温度数据以备将来优化。
 *
 * @param d   数据段指针（8 字节）
 * @param len 数据长度（期望为 8）
 */
static void parse_mag_temp(const uint8_t *d, uint8_t len)
{
    if (len != 8u) { return; }

    /* 磁力计数据：d[0]~d[5] 对应 mx, my, mz
     * 本工程不保存磁力计数据到快照中，
     * 因为电机 PWM 电磁干扰会导致数据不可靠。
     * 注意：即使不保存，校验路径仍然会走，不会被视为 bad_frame。 */

    /* 温度换算：int16 原始值 / 100 = 摄氏度（°C）
     * 例如原始值 2545 → 2545 / 100 = 25.45°C
     * 乘以 0.01f 等价于除以 100。
     * 
     * 为什么用 0.01f 而不是直接 / 100？
     *   浮点数除法中，乘法通常比除法快一点点。
     *   虽然编译器会优化，但写 * 0.01f 语义上更明确——
     *   "温度换算系数是 0.01"。 */
    s_snap.temp_c = (float)le16(d[6], d[7]) * 0.01f;

    /* 设置磁力+温度数据已接收标志 */
    s_snap.has_mag_temp = true;
}

/**
 * @brief  parse_quaternion —— 解析 0x02 四元数帧（存根）
 *
 * 四元数（Quaternion）是姿态的一种数学表示方式，
 * 用 4 个数字（q0, q1, q2, q3）来描述三维旋转。
 * 相比欧拉角（roll/pitch/yaw），四元数没有"万向锁"问题。
 *
 * 但本工程用欧拉角就够了，不需要四元数。
 * 之所以保留这个解析函数，是为了：
 *   1. 让校验路径通过（不把 0x02 帧当成 bad_frame）
 *   2. 保留扩展可能性（将来如果想用四元数，只需填充函数体）
 *
 * (void)d 的作用：
 *   告诉编译器"参数 d 我没有使用，这是故意的，不是忘记用了"，
 *   避免编译器产生"unused parameter"警告。
 *
 * @param d   数据段指针
 * @param len 数据长度（期望为 8）
 */
static void parse_quaternion(const uint8_t *d, uint8_t len)
{
    (void)d;  /* 显式标记参数未使用，消除编译器警告 */
    if (len != 8u) { return; }
    /* 不进 snapshot，保留空壳供校验路径通过 */
}

/**
 * @brief  parse_baro_alt —— 解析 0x05 气压+高度帧（存根）
 *
 * 作用和 parse_quaternion() 类似——空壳函数，
 * 只做长度检查，不进 snapshot，避免被当成 bad_frame。
 *
 * 气压数据主要用于测高，对本自平衡小车暂无用处。
 *
 * @param d   数据段指针
 * @param len 数据长度（期望为 10）
 */
static void parse_baro_alt(const uint8_t *d, uint8_t len)
{
    (void)d;  /* 显式标记参数未使用 */
    if (len != 10u) { return; }
    /* 不进 snapshot */
}

/**
 * @brief  dispatch_frame —— 根据帧 ID 分发到对应的解析函数
 *
 * 这是"分发器"（dispatcher）模式：
 *   一个 switch 语句，根据帧 ID 决定调用哪个解析函数。
 *   类似于工厂流水线的"分拣员"——看标签（ID）决定送到哪个工位。
 *
 * 为什么用 switch 而不是 if-else if 链？
 *   对于少量（≤10）分支，switch 更清晰。
 *   编译器可能把 switch 优化成跳转表（O(1)），
 *   而 if-else 链是顺序比较（O(n)）。
 *
 * @param id  帧 ID（0x01~0x05）
 * @param d   数据段指针
 * @param len 数据段长度
 */
static void dispatch_frame(uint8_t id, const uint8_t *d, uint8_t len)
{
    /* 根据 ID 分发到对应的解析函数 */
    switch (id) {
        case 0x01: parse_attitude(d, len);   break;  /* 姿态角（roll/pitch/yaw） */
        case 0x02: parse_quaternion(d, len); break;  /* 四元数（暂不处理） */
        case 0x03: parse_gyro_acc(d, len);   break;  /* 陀螺+加速度 ⭐ 核心数据 */
        case 0x04: parse_mag_temp(d, len);   break;  /* 磁力+温度 */
        case 0x05: parse_baro_alt(d, len);   break;  /* 气压+高度（暂不处理） */
        default:
            /* 未知 ID：不处理，也不算 bad_frame。
             * 因为 ATK 后续固件可能增加新帧类型，
             * 直接丢弃即可，不要影响 good_frame 计数。 */
            break;
    }

    /* 无论哪个帧类型（包括未知 ID），只要走到这里，
     * 就说明"成功收到了一帧校验通过的数据"。
     * good_frames +1 用于统计通信质量。 */
    s_good_frames++;
}

/* ============================================================
 * 公开 API 函数
 * ============================================================
 * 这些函数在 ms901m.h 中有声明，可以被其他 .c 文件调用。
 * 它们是本模块对外提供的"服务接口"。
 */

/**
 * @brief  初始化姿态传感器解析器
 *
 * 这是本模块的"构造函数"，必须在开始接收数据前调用一次。
 *
 * 执行的操作：
 *   1. 设置加速度和陀螺仪的量纲换算系数
 *   2. 复位状态机到等待帧头的初始状态
 *   3. 清空所有快照字段和统计计数
 *
 * @param acc_fsr_g   加速度计满量程，单位 g
 *        必须和 MS901M 的当前配置一致！
 *        出厂默认：±4g → 传入 4
 *        如果传 8 但传感器是 ±4g，读出的加速度值会偏大一倍！
 *
 * @param gyro_fsr_dps 陀螺仪满量程，单位 °/s
 *        出厂默认：±2000 dps → 传入 2000
 *
 * ⚠️ 重要提醒：
 *   如果以后通过 ATK 上位机改了传感器的量程，
 *   这里的参数也要同步修改，否则数据换算会出错。
 */
void ms901m_init(int16_t acc_fsr_g, int16_t gyro_fsr_dps)
{
    /* 如果 acc_fsr_g > 0，用传入值重新计算系数。
     * 如果 acc_fsr_g <= 0（比如传了 0 或负数），保持默认值不变。
     * 这个判断是为了允许调用方传 0 表示"用默认值"。
     * 
     * (float)acc_fsr_g：把 int16 强转为 float，避免整数除法。
     * 如果写成 acc_fsr_g / 32768，两个整数相除结果还是整数，
     * 比如 4 / 32768 = 0（整数除法截断），完全错误！
     * 所以必须至少先把其中一个操作数转成 float。 */
    if (acc_fsr_g  > 0) { s_acc_scale  = (float)acc_fsr_g  / 32768.0f; }
    if (gyro_fsr_dps > 0) { s_gyro_scale = (float)gyro_fsr_dps / 32768.0f; }

    /* 复位状态机：回到 ST_SYNC1，清零校验和和数据索引 */
    reset_state_machine();

    /* ---- 清空快照的所有字段 ---- */
    /* 角度清零 */
    s_snap.pitch_deg = 0.0f;
    s_snap.roll_deg  = 0.0f;
    s_snap.yaw_deg   = 0.0f;

    /* 角速度清零（连续赋值：从右向左执行）
     * s_snap.gz_dps = 0.0f 先执行，返回 0.0f，
     * 然后 s_snap.gy_dps = 0.0f，再 s_snap.gx_dps = 0.0f */
    s_snap.gx_dps = s_snap.gy_dps = s_snap.gz_dps = 0.0f;

    /* 加速度清零 */
    s_snap.ax_g   = s_snap.ay_g   = s_snap.az_g   = 0.0f;

    /* 温度清零 */
    s_snap.temp_c = 0.0f;

    /* 所有"已收到"标志设为 false——还没收到过任何数据 */
    s_snap.has_attitude = false;
    s_snap.has_gyro_acc = false;
    s_snap.has_mag_temp = false;

    /* 统计计数清零 */
    s_bad_frames  = 0u;   /* 坏帧数 = 0 */
    s_good_frames = 0u;   /* 好帧数 = 0 */
}

/**
 * @brief  把串口接收到的字节"喂"给解析器（⭐核心函数）
 *
 * 这是整个模块最核心的函数——它实现了一个"有限状态机"，
 * 逐字节处理串口 RAW 数据。
 *
 * 调用方式（典型的主循环代码）：
 * @code
 *   // 在主循环中（如每 1 ms 执行一次）：
 *   uint8_t buf[64];
 *   size_t n = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
 *   if (n > 0) {
 *       ms901m_feed_bytes(buf, n);  // 喂给解析器
 *   }
 * @endcode
 *
 * 状态机执行流程：
 *   1. ST_SYNC1：等待第一个 0x55
 *   2. ST_SYNC2：等待第二个 0x55，确认帧头
 *   3. ST_ID：    接收帧 ID（知道是什么数据）
 *   4. ST_LEN：   接收数据长度（知道要收多少字节）
 *   5. ST_DATA：  循环接收 LEN 个数据字节，存入 s_data[]
 *   6. ST_CHECKSUM：接收校验和，验证帧完整性
 *
 * 异常处理：
 *   - 任何位置收到不符合预期的字节 → 复位状态机，重新同步
 *   - LEN 超出上限（> 32）→ bad_frames++，复位状态机
 *   - 校验和不匹配 → bad_frames++，复位状态机
 *
 * @param p    指向接收缓冲区的指针，里面是从 UART 读到的原始字节
 * @param n    本次要处理的字节数量
 *
 * @note  本函数是"纯处理"函数，不涉及任何硬件操作。
 *        建议在主循环中调用，不要在 UART 中断 ISR 中直接调用，
 *        以保持 ISR 尽可能短小精悍（中断服务的黄金法则）。
 */
void ms901m_feed_bytes(const uint8_t *p, size_t n)
{
    /* 防御性编程：传入指针不能为 NULL */
    if (p == NULL) { return; }

    /* 逐字节处理！一次处理 n 个字节。
     * 这是"流式处理"的核心思想——来多少处理多少，
     * 不需要等整帧收完再处理。 */
    for (size_t i = 0u; i < n; ++i) {
        /* 取出当前要处理的字节 */
        uint8_t b = p[i];

        /* 根据当前状态机状态，决定如何处理这个字节 */
        switch (s_state) {

        /* ================================================================
         * 状态 0：ST_SYNC1 —— 等待第一个同步字节 0x55
         * ================================================================
         * 这是状态机的"起点"（也是"异常恢复点"）。
         * 只要没收到 0x55，就卡在这里，不断丢弃进来的字节。
         * 
         * 想象一下：你在嘈杂的房间里等人喊你的名字（0x55），
         * 其他所有声音（非 0x55 字节）都被你忽略。
         * 一旦听到有人喊你名字，你立刻警觉——"来了！" */
        case ST_SYNC1:
            if (b == SYNC_BYTE) {       /* 这个字节是 0x55 吗？ */
                s_chk   = b;             /* 校验和从第一个字节开始累加 */
                s_state = ST_SYNC2;      /* 进入状态 1：等第二个 0x55 */
            }
            /* 如果 b != 0x55，什么都不做，继续保持 ST_SYNC1 状态
             * 这就实现了"丢弃非同步字节"的效果 */
            break;

        /* ================================================================
         * 状态 1：ST_SYNC2 —— 等待第二个同步字节 0x55
         * ================================================================
         * 已经收到了一个 0x55，现在等第二个。
         * 如果第二个也是 0x55 → 确认是帧头，准备收 ID。
         * 如果第二个不是 0x55 → 刚才那个 0x55 可能是巧合，重新同步。
         * 
         * 为什么需要连续两个 0x55 才能确认帧头？
         *   单个 0x55 可能被数据内容中的 0x55 "碰巧"匹配，
         *   但连续两个 0x55 的概率大大降低，提高了同步的可靠性。 */
        case ST_SYNC2:
            if (b == SYNC_BYTE) {       /* 第二个字节也是 0x55？ */
                s_chk  += b;             /* 累加校验和 */
                s_state = ST_ID;          /* 进入状态 2：准备收 ID */
            } else {
                /* 第二个字节不是 0x55 → 之前的 0x55 是误触发。
                 * 复位状态机回到 ST_SYNC1，重新找帧头。
                 * 
                 * 注意：当前的这个 b 有可能是下一个帧的第一个 0x55，
                 * 但 reset_state_machine() 会把这个 b "浪费"掉。
                 * 下一轮循环处理下一个字节时会重新从 ST_SYNC1 开始。
                 * 这是设计上的取舍——简单可靠优于不丢字节。 */
                reset_state_machine();
            }
            break;

        /* ================================================================
         * 状态 2：ST_ID —— 接收帧类型 ID
         * ================================================================
         * 帧头对齐完成，现在收帧类型 ID。
         * ID 决定了这帧数据的内容是什么（姿态/陀螺/磁力/...）。
         * 同时把 ID 累加到校验和里。 */
        case ST_ID:
            s_id    = b;      /* 保存帧 ID 到 s_id */
            s_chk  += b;      /* ID 字节加入校验和累加 */
            s_state = ST_LEN; /* 进入状态 3：准备收数据长度 LEN */
            break;

        /* ================================================================
         * 状态 3：ST_LEN —— 接收数据长度 LEN
         * ================================================================
         * LEN 告诉我们 DATA 段有多少个字节。
         * 例如 LEN=6 表示后面还有 6 个字节是数据。
         * 
         * 安全性检查：如果 LEN 超过 MS901M_DATA_MAX（32 字节），
         * 说明这帧数据异常（可能帧同步错误，把非数据字节当成了 LEN），
         * 丢弃并复位状态机。 */
        case ST_LEN:
            if (b > MS901M_DATA_MAX) {
                /* LEN 值异常（> 32），说明帧同步可能出错了。
                 * 比如数据内容中的某个字节 0x33 被当成了 LEN，
                 * 这显然不合理。记录一次错误，复位等待重新同步。 */
                s_bad_frames++;
                reset_state_machine();
                break;  /* 跳出 switch，继续处理下一个字节 */
            }

            s_len      = b;   /* 保存数据长度 */
            s_chk     += b;   /* LEN 加入校验和累加 */

            /* 重置数据接收索引，从头开始收数据 */
            s_data_idx = 0u;

            /* 如果 LEN == 0，说明这帧没有数据段，
             * 跳过 ST_DATA，直接进入 ST_CHECKSUM 等校验和。
             * 这是 C 语言中常用的"三元运算符"：
             *   条件 ? 值1 : 值2
             *   如果条件为真 → 取值1；否则 → 取值2
             * 等价于：
             *   if (s_len == 0) s_state = ST_CHECKSUM;
             *   else s_state = ST_DATA; */
            s_state    = (s_len == 0u) ? ST_CHECKSUM : ST_DATA;
            break;

        /* ================================================================
         * 状态 4：ST_DATA —— 接收数据内容
         * ================================================================
         * 连续接收 LEN 个字节，存入 s_data[] 数组。
         * 每收一个字节，s_data_idx 加 1。
         * 当 s_data_idx >= s_len 时，数据收满，转入 ST_CHECKSUM。
         * 
         * s_data_idx++ 是"先用再加"：
         *   s_data[s_data_idx] = b;   // 存入当前位置
         *   s_data_idx = s_data_idx + 1;  // 索引加 1
         * 所以 s_data_idx 始终指向"下一个要存的位置"。 */
        case ST_DATA:
            s_data[s_data_idx++] = b;  /* 存入数据缓冲区，索引自增 */
            s_chk += b;                 /* 数据字节加入校验和 */

            /* 检查是否收完了所有数据字节
             * 如果已收数量 >= 预期长度，数据收满 */
            if (s_data_idx >= s_len) {
                s_state = ST_CHECKSUM;  /* 进入状态 5：等待校验和 */
            }
            break;

        /* ================================================================
         * 状态 5：ST_CHECKSUM —— 验证校验和
         * ================================================================
         * 这是帧的最后一个字节——校验和。
         * 把收到的校验和（b）和本地累加的结果（s_chk）对比：
         * 
         *   ✅ b == s_chk：帧数据完整无误，调用 dispatch_frame() 解析
         *   ❌ b != s_chk：数据损坏或帧同步错误，bad_frames++
         * 
         * 无论校验成功还是失败，最后都要复位状态机准备收下一帧。 */
        case ST_CHECKSUM:
            if (b == s_chk) {
                /* 校验通过！
                 * 调用 dispatch_frame() 把数据分发到对应的解析函数。
                 * 注意：校验和字节本身不参与 s_chk 累加，
                 * 因为 dispatch_frame() 不依赖 s_chk。 */
                dispatch_frame(s_id, s_data, s_len);
            } else {
                /* 校验失败：收到的校验和与本地计算的不一致。
                 * 可能原因：波特率不匹配、电磁干扰、硬件故障。
                 * 记录一次坏帧，不通知上层。 */
                s_bad_frames++;
            }
            /* 无论成功失败，都复位状态机等待下一帧 */
            reset_state_machine();
            break;

        /* ================================================================
         * 默认分支：万一 s_state 出现了不在枚举中的值
         * ================================================================
         * 正常情况下不会执行到这里。
         * 但嵌入式系统要"容错"——即使程序跑飞了也要能恢复。
         * 这里直接复位状态机，让系统自愈。 */
        default:
            reset_state_machine();
            break;
        }  /* switch 结束 */
    }  /* for 循环结束 */
}

/**
 * @brief  构造一帧"读取寄存器"命令
 *
 * 发送此命令后，传感器会返回指定寄存器的当前值。
 *
 * 读命令的特殊规则：
 *   命令 ID 需要把最高位置 1（即 OR 0x80），
 *   例如读取陀螺量程寄存器（0x03）→ ID = 0x03 | 0x80 = 0x83
 *   数据段固定为 1 字节（payload = 0x00），表示"不写入数据"。
 *
 * @param  cmd_id  要读取的寄存器 ID（7 位编号，0x00~0x7F）
 * @param  out     输出缓冲区
 * @param  out_cap 缓冲区容量
 * @return 帧长度（固定 6 字节），失败返回 0
 */
size_t ms901m_build_read_cmd(uint8_t cmd_id, uint8_t *out, size_t out_cap)
{
    /* 读命令的 payload 固定为 0x00（不需要写入值） */
    uint8_t payload = 0x00u;

    /* cmd_id & 0x7F：确保只取低 7 位（清除最高位）
     * | 0x80：把最高位置 1，标记为"读"操作
     * 例如：0x03 → (0x03 & 0x7F) | 0x80 = 0x03 | 0x80 = 0x83 */
    return build_frame((uint8_t)((cmd_id & 0x7Fu) | 0x80u), &payload, 1u, out, out_cap);
}

/**
 * @brief  构造一帧"写入寄存器"命令
 *
 * 向传感器的指定寄存器写入配置数据。
 *
 * @param  cmd_id   目标寄存器 ID（0x00~0x7F，最高位为 0 表示写操作）
 * @param  data     要写入的数据指针
 * @param  data_len 数据长度（字节数）
 * @param  out      输出缓冲区
 * @param  out_cap  缓冲区容量
 * @return 帧长度（= 5 + data_len），失败返回 0
 */
size_t ms901m_build_write_cmd(uint8_t cmd_id, const uint8_t *data, uint8_t data_len,
    uint8_t *out, size_t out_cap)
{
    /* cmd_id & 0x7F：确保最高位为 0，标记为"写"操作
     * （即使调用方不小心传了带最高位的值，也强制清除）
     * 这是一种"宽容性"设计——让函数更健壮 */
    return build_frame((uint8_t)(cmd_id & 0x7Fu), data, data_len, out, out_cap);
}

/**
 * @brief  构造一帧"写入 1 字节"命令（便捷函数）
 *
 * 这是 ms901m_build_write_cmd() 的"语法糖"。
 * 当要写入的数据只有 1 个字节时，用这个函数更简洁。
 *
 * 调用示例对比：
 *   // 原始方式
 *   uint8_t val = 0x02;
 *   ms901m_build_write_cmd(0x04, &val, 1, out, cap);
 *   // 便捷方式
 *   ms901m_build_write_u8_cmd(0x04, 0x02, out, cap);
 *
 * @param cmd_id 寄存器 ID
 * @param value  要写入的 1 字节值
 * @param out    输出缓冲区
 * @param out_cap 缓冲区容量
 * @return 帧长度（固定 6 字节），失败返回 0
 */
size_t ms901m_build_write_u8_cmd(uint8_t cmd_id, uint8_t value, uint8_t *out, size_t out_cap)
{
    /* &value 把单字节变量的地址传给 build_frame，
     * 1u 表示数据长度为 1 字节 */
    return ms901m_build_write_cmd(cmd_id, &value, 1u, out, out_cap);
}

/**
 * @brief  构造一帧"写入 2 字节"命令（便捷函数，小端格式）
 *
 * 当要写入的数据是 16 位无符号整数时使用。
 * 注意数据按小端格式排列（低字节在前，高字节在后）。
 *
 * @param cmd_id 寄存器 ID
 * @param value  要写入的 16 位值（如 PWM 脉宽 1500 us）
 * @param out    输出缓冲区
 * @param out_cap 缓冲区容量
 * @return 帧长度（固定 7 字节），失败返回 0
 */
size_t ms901m_build_write_u16_cmd(uint8_t cmd_id, uint16_t value, uint8_t *out, size_t out_cap)
{
    /* 准备 2 字节的数据缓冲区 */
    uint8_t data[2];

    /* 小端格式：
     * data[0] = 低 8 位（value & 0xFF）
     * data[1] = 高 8 位（value >> 8）
     * 例如 value = 0x1234 → data[0] = 0x34, data[1] = 0x12 */
    data[0] = (uint8_t)(value & 0xFFu);   /* 取低 8 位 */
    data[1] = (uint8_t)((value >> 8) & 0xFFu);  /* 取高 8 位 */

    /* 调用通用写命令构造函数 */
    return ms901m_build_write_cmd(cmd_id, data, 2u, out, out_cap);
}

/* ============================================================
 * 便捷命令构造函数集
 * ============================================================
 * 以下每个函数都是对 ms901m_build_write_u8_cmd() 或
 * ms901m_build_write_u16_cmd() 的封装。
 *
 * 它们的作用是"给有名字的命令配上专用的函数"——
 * 调用方不需要记住命令 ID 和参数格式，
 * 只需要知道"我要保存配置"→ 调 ms901m_build_save_cmd()。
 *
 * 参数说明（以下所有函数通用）：
 *   @param out     输出缓冲区
 *   @param out_cap 缓冲区容量
 *   @return size_t 帧长度，失败返回 0
 */

/**
 * @brief  构造"保存当前配置到 Flash"的命令
 *
 * 传感器当前的配置（量程、波特率、上报频率等）默认只保存在 RAM 中，
 * 断电后丢失。调用此命令后，配置被写入 Flash，下次上电自动加载。
 *
 * 典型用法：在调试阶段用 ATK 上位机配置好参数后，
 * 发送一次 SAVE 命令固化配置，之后每次上电都是这个配置。
 */
size_t ms901m_build_save_cmd(uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_SAVE, 0x00u, out, out_cap);
}

/**
 * @brief  构造"恢复出厂设置"的命令
 * 所有配置恢复为出厂默认值（加速度±4g，陀螺±2000dps，波特率115200等）
 */
size_t ms901m_build_reset_cmd(uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_RESET, 0x00u, out, out_cap);
}

/**
 * @brief  构造"传感器校准"的命令
 *
 * 不同校准类型需要的操作：
 *   - 加速度计校准（ACC）：传感器水平静置，自动完成
 *   - 磁力计校准（MAG）：传感器水平旋转 360°，自动完成
 *   - 气压计归零（BARO_ZERO）：当前气压设为基准高度 0
 *
 * @param cal  校准类型枚举值
 */
size_t ms901m_build_sensor_cal_cmd(ms901m_sencal_t cal, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_SENCAL, (uint8_t)cal, out, out_cap);
}

/**
 * @brief  构造"设置陀螺仪量程"的命令
 *
 * 量程选择（精度 vs 范围）：
 *   ±250 dps：  精度最高（0.0076°/s/LSB），适合慢速旋转
 *   ±500 dps：  精度适中
 *   ±1000 dps： 量程较大
 *   ±2000 dps： 量程最大（精度最低，0.061°/s/LSB），出厂默认
 * 
 * 自平衡小车推荐 ±2000 dps，因为倾倒时的角速度可能很大。
 */
size_t ms901m_build_set_gyro_fsr_cmd(ms901m_gyro_fsr_t fsr, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_GYROFSR, (uint8_t)fsr, out, out_cap);
}

/**
 * @brief  构造"设置加速度计量程"的命令
 *
 * 自平衡小车推荐 ±4g 或 ±8g。
 * 太大的量程会降低精度（同 ADC 位数，量程越大，每 LSB 代表的 g 值越大）。
 */
size_t ms901m_build_set_acc_fsr_cmd(ms901m_acc_fsr_t fsr, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_ACCFSR, (uint8_t)fsr, out, out_cap);
}

/**
 * @brief  构造"设置串口波特率"的命令
 *
 * ⚠️ 重要：
 *   修改波特率后，传感器会立刻以新波特率通信。
 *   单片机必须在发送此命令后，重新初始化 UART 为新波特率，
 *   否则通信会中断！
 *
 *   建议流程：
 *     1. 发设置波特率命令
 *     2. 等待 10 ms 以上（让传感器切换到新波特率）
 *     3. 重新初始化 UART
 *     4. 发一条测试命令确认通信正常
 */
size_t ms901m_build_set_baud_cmd(ms901m_baud_t baud, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_BAUD, (uint8_t)baud, out, out_cap);
}

/**
 * @brief  构造"设置主动上报内容"的命令
 *
 * MS901M 默认会发送所有帧类型（姿态+四元数+陀螺加速度+磁力温度+气压高度）。
 * 为了节省带宽和解析时间，可以只开启需要的帧类型。
 *
 * 本工程需要：姿态（0x01）+ 陀螺加速度（0x03）
 * mask = MS901M_RETURN_MASK_ATTITUDE | MS901M_RETURN_MASK_GYRO_ACC = 0x05
 *
 * @param mask  位掩码，用 OR 组合多个 MS901M_RETURN_MASK_xxx 值
 */
size_t ms901m_build_set_return_mask_cmd(uint8_t mask, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_RETURNSET, mask, out, out_cap);
}

/**
 * @brief  构造"设置主动上报频率"的命令
 *
 * 自平衡控制通常需要 200 Hz 的数据更新率。
 * 频率越高 → 控制延迟越低 → 平衡越稳定，但 CPU 负载也越高。
 *
 * @param rate  频率枚举值（MS901M_RETURN_RATE_200HZ = 默认 200 Hz）
 */
size_t ms901m_build_set_return_rate_cmd(ms901m_return_rate_t rate, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_RETURNRATE, (uint8_t)rate, out, out_cap);
}

/**
 * @brief  构造"设置姿态解算算法"的命令
 *
 * 6 轴算法（推荐本工程使用）：
 *   只用陀螺仪 + 加速度计，不使用磁力计。
 *   优点：不受电机 PWM 电磁干扰
 *   缺点：偏航角（yaw）会随时间漂移
 *
 * 9 轴算法：
 *   融合陀螺仪 + 加速度计 + 磁力计。
 *   优点：偏航角有绝对参考，不漂移
 *   缺点：磁力计易受电磁干扰，需要良好的磁环境
 */
size_t ms901m_build_set_alg_cmd(ms901m_alg_t alg, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_ALG, (uint8_t)alg, out, out_cap);
}

/**
 * @brief  构造"设置安装方向"的命令
 * 水平安装：芯片面朝上（默认）
 * 垂直安装：芯片面朝前
 */
size_t ms901m_build_set_asm_cmd(ms901m_asm_t asm_mode, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_ASM, (uint8_t)asm_mode, out, out_cap);
}

/**
 * @brief  构造"设置陀螺仪自校准开关"的命令
 * 开启后，传感器上电时自动校准陀螺零偏。
 * 推荐开启——可减少陀螺温漂的影响。
 */
size_t ms901m_build_set_gaucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_GAUCAL, (uint8_t)enable, out, out_cap);
}

/**
 * @brief  构造"设置气压计自校准开关"的命令
 */
size_t ms901m_build_set_baucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_BAUCAL, (uint8_t)enable, out, out_cap);
}

/**
 * @brief  构造"设置 LED 开关"的命令
 *
 * ⚠️ 寄存器命名反直觉：
 *   寄存器名叫 LEDOFF（关灯寄存器），
 *   所以传 MS901M_SWITCH_ON（1）→ 灯灭
 *      传 MS901M_SWITCH_OFF（0）→ 灯亮
 *   这个命名确实容易搞混，看代码时要注意。
 */
size_t ms901m_build_set_ledoff_cmd(ms901m_switch_t led_off, uint8_t *out, size_t out_cap)
{
    return ms901m_build_write_u8_cmd(MS901M_CMD_LEDOFF, (uint8_t)led_off, out, out_cap);
}

/**
 * @brief  构造"设置端口模式"的命令
 *
 * MS901M 的 D0~D3 是 4 个多功能 GPIO 端口。
 * 可以配置为：
 *   - 模拟输入（测量外部电压）
 *   - 数字输入（读取开关状态）
 *   - 数字输出高/低电平（控制外部设备）
 *   - PWM 输出（仅 D1 和 D3 支持）
 *
 * @param port_index  端口号（0=D0, 1=D1, 2=D2, 3=D3）
 * @param mode        工作模式枚举值
 */
size_t ms901m_build_set_port_mode_cmd(uint8_t port_index, ms901m_port_mode_t mode,
    uint8_t *out, size_t out_cap)
{
    uint8_t cmd_id = 0u;

    /* 通过映射函数获取端口对应的命令 ID
     * 如果端口号无效（>=4），映射函数返回 false */
    if (!port_mode_cmd_id(port_index, &cmd_id)) {
        return 0u;  /* 端口号无效，构造失败 */
    }
    return ms901m_build_write_u8_cmd(cmd_id, (uint8_t)mode, out, out_cap);
}

/**
 * @brief  构造"设置 PWM 脉宽"的命令
 *
 * 设置 D1 或 D3 端口的 PWM 高电平持续时间。
 *
 * 舵机控制示例：
 *   舵机通常使用 50 Hz 的 PWM（周期 20000 us）
 *   脉宽 500 us  → 0° 位置
 *   脉宽 1500 us → 90° 位置（中位）
 *   脉宽 2500 us → 180° 位置
 *
 * @param port_index 端口号（仅 1 或 3）
 * @param pulse_us   高电平脉宽（单位微秒，范围通常 500~2500 us）
 */
size_t ms901m_build_set_pwm_pulse_cmd(uint8_t port_index, uint16_t pulse_us,
    uint8_t *out, size_t out_cap)
{
    uint8_t cmd_id = 0u;

    if (!pwm_pulse_cmd_id(port_index, &cmd_id)) {
        return 0u;
    }
    return ms901m_build_write_u16_cmd(cmd_id, pulse_us, out, out_cap);
}

/**
 * @brief  构造"设置 PWM 周期"的命令
 *
 * 周期决定了 PWM 的频率：频率 (Hz) = 1,000,000 / 周期 (us)
 * 例如：
 *   周期 20000 us → 50 Hz（标准舵机频率）
 *   周期 10000 us → 100 Hz
 *   周期 5000 us  → 200 Hz（更高频率，适合电机驱动）
 *
 * @param port_index 端口号（仅 1 或 3）
 * @param period_us  PWM 周期（单位微秒）
 */
size_t ms901m_build_set_pwm_period_cmd(uint8_t port_index, uint16_t period_us,
    uint8_t *out, size_t out_cap)
{
    uint8_t cmd_id = 0u;

    if (!pwm_period_cmd_id(port_index, &cmd_id)) {
        return 0u;
    }
    return ms901m_build_write_u16_cmd(cmd_id, period_us, out, out_cap);
}

/* ============================================================
 * 本地量程同步函数
 * ============================================================ */

/**
 * @brief  把加速度计量程枚举值同步为本地换算系数
 *
 * 使用场景：
 *   1. 通过 ms901m_build_set_acc_fsr_cmd() 告诉传感器"改为 ±8g"
 *   2. 调用 ms901m_apply_acc_fsr(MS901M_ACC_FSR_8G)
 *   3. 解析器内部 s_acc_scale 更新为 8.0/32768.0
 *   4. 后续收到的加速度数据按新系数换算
 *
 * @param fsr  加速度计量程枚举值
 * @return true = 成功；false = 无效枚举值
 */
bool ms901m_apply_acc_fsr(ms901m_acc_fsr_t fsr)
{
    /* 委托给内部函数 set_acc_scale_by_sel() 执行实际的系数更新 */
    return set_acc_scale_by_sel((uint8_t)fsr);
}

/**
 * @brief  把陀螺仪量程枚举值同步为本地换算系数
 * @param fsr  陀螺仪量程枚举值
 * @return true = 成功；false = 无效枚举值
 */
bool ms901m_apply_gyro_fsr(ms901m_gyro_fsr_t fsr)
{
    return set_gyro_scale_by_sel((uint8_t)fsr);
}

/* ============================================================
 * 状态查询与数据获取函数
 * ============================================================ */

/**
 * @brief  查询是否至少收到过一帧 0x01 姿态数据
 *
 * 用于启动时的 IMU 在线检测。
 * 在 main() 中会循环检查这个函数：
 * @code
 *   while (!ms901m_has_attitude()) {
 *       // 持续喂数据给解析器
 *       ms901m_feed_bytes(buf, n);
 *   }
 *   // 到这里说明 IMU 已正常工作
 * @endcode
 *
 * 如果 (ms901m_good_frames() > 100) 但 has_attitude() 仍为 false，
 * 说明 MS901M 可能没有配置为发送 0x01 帧（姿态帧）。
 * 需要检查 return mask 设置。
 *
 * @return true  = 姿态数据已就绪；false = 尚未收到
 */
bool ms901m_has_attitude(void)
{
    return s_snap.has_attitude;
}

/**
 * @brief  获取最新姿态快照（深拷贝）
 *
 * 调用方传入一个 ms901m_snapshot_t 变量的指针，
 * 函数会把解析器内部的最新数据完整复制一份到该变量中。
 *
 * 为什么是"深拷贝"而不是返回指针？
 *   如果返回 s_snap 的指针，外部可以直接访问内部数据，
 *   但当主循环下一次调用 ms901m_feed_bytes() 更新 s_snap 时，
 *   外部持有的指针指向的数据会"偷偷"变化（线程不安全）。
 *   而深拷贝给出一份独立副本，调用方拿到后不受后续更新影响。
 *
 * 使用示例：
 * @code
 *   ms901m_snapshot_t snap;
 *   ms901m_get_snapshot(&snap);
 *   // 现在可以放心使用 snap.pitch_deg，即使后续有更新也不受影响
 * @endcode
 *
 * @param out  指向输出结构体的指针（不能为 NULL）
 *             函数会将当前快照的所有字段写入此结构体
 */
void ms901m_get_snapshot(ms901m_snapshot_t *out)
{
    /* 防御性编程：检查输出指针是否为 NULL */
    if (out == NULL) { return; }

    /* 结构体直接赋值 = 深拷贝（逐字段复制）
     * C 语言中，结构体可以通过 = 运算符整体赋值。
     * 这等价于手动 memcpy(out, &s_snap, sizeof(ms901m_snapshot_t))。
     * 
     * 注意：结构体赋值在单片机编译器中可能被展开为
     * 多个 mov 指令（逐 4 字节拷贝），效率不错。
     * 对于大型结构体，用 memcpy 可能更好，但这里只有十几个 float/bool，
     * 直接赋值既清晰又高效。 */
    *out = s_snap;
}

/**
 * @brief  返回累计校验失败的帧数
 *
 * 这个值是衡量串口通信质量的重要指标。
 * 如果这个值持续快速增长，说明：
 *   1. 波特率设置不匹配（最常见原因）
 *   2. 串口线受到电磁干扰（PWM 电机线靠近了串口线）
 *   3. 传感器或主控的 UART 硬件故障
 *
 * 建议在调试日志中每 1 秒输出一次这个值。
 * 例如在 200 Hz 的帧率下，如果每秒 bad_frames 增长超过 10，
 * 就说明通信链路有问题需要排查。
 *
 * @return 累计坏帧数
 */
uint32_t ms901m_bad_frames(void)
{
    return s_bad_frames;
}

/**
 * @brief  返回累计成功解析的帧数
 *
 * 通过这个值可以估算实际的帧接收频率：
 *   actual_rate = (good_frames - prev_good_frames) / interval_seconds
 *
 * 例如：5 秒内 good_frames 从 0 增长到 1000，
 *       实际帧率 = 1000 / 5 = 200 Hz（符合 MS901M 默认配置）
 *
 * @return 累计好帧数
 */
uint32_t ms901m_good_frames(void)
{
    return s_good_frames;
}
