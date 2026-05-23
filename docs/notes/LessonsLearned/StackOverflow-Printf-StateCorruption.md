# 栈溢出写脏全局变量：心跳 `state=?` 与 K230 速率异常

**日期**：2026-05（本项目**第二次**同类事故；第一次见 Stage1.5 §12.2，2026-05-08）  
**影响范围**：`app_balance`、`app_safety`、`app_circle_demo`、`startup_mspm0g350x_uvision.s`  
**严重级别**：高（安全状态机被污染，可能误显示或误恢复 ARMED）  
**TaskLog 来源**：[Stage1.5-IMU-Swap-MS901M.md §12.2](../../TaskLog/Stage1.5-IMU-Swap-MS901M.md)（首次）；本次为 circle demo 复发  
**汇总索引**：[From-TaskLog-Summary.md §1](./From-TaskLog-Summary.md)

---

## 1. 现象

程序下载后，1 Hz 心跳报文持续显示：

```text
[hb] t=1s state=? ...
```

伴随历史症状（不一定同时出现）：

- `k230_rx` 出现离谱的大数值（例如几百、几千 byte/s），而 K230 实际未在线；
- `[safety] CORRUPTION` 日志：`s_state` 被写成类似 `0x20200da4` 的 SRAM 地址；
- 诊断版心跳曾显示 `state=?x88(raw=136)`，说明枚举值已超出 `{0..4}` 合法范围。

---

## 2. 排查过程中的误导方向

| 假设 | 为何看起来合理 | 为何被否定 |
|------|----------------|------------|
| 增量编译残留 stale `.o` | 改代码后现象不变很常见 | 诊断版 `?xNN(raw=NN)` 已烧进固件，说明新代码在跑；问题是**运行时值真的被写脏** |
| AC6 `-O3` / `--short-enums` 优化 bug | 枚举只有 0..4，`switch` 进 `default` 像编译器问题 | `raw=136`、`0x20200da4` 等值明显是**内存内容**，不是正常枚举 |
| 某 UART 模块 buffer 溢出 | 项目曾有 K230 相关内存问题 | K230 未活动时仍 corruption；map 显示 victim 在 **栈底紧邻的 .bss 区**，更符合栈溢出 |

---

## 3. 诊断手段（有效 vs 可退役）

### 3.1 有效且已保留

1. **`safety_state_to_str()` 的 `default → "?"`**  
   一旦 `s_state` 非法，心跳立刻可见。

2. **`app_safety_block_t` + canary**（`app_safety.c`）  
   用 struct 强制 `canary_before | state | canary_after` 连续布局；`app_safety_get_state()` 检测非法值与 canary，必要时 auto-heal 到 `DISARMED` 并打印 `[safety] CORRUPTION`。

3. **`APP_SAFETY_FORCE_INT32_` 哨兵**（`app_safety.h`）  
   强制 `app_safety_state_t` 占 4 字节，降低 `--short-enums` 单字节枚举被相邻写入污染的风险。

4. **链接 map 对照地址**  
   `.map` 中 `RW_IRAM2` 段可精确看到 STACK 与 `.bss` 的边界——本次定案的关键证据。

### 3.2 仅用于本次定位、已清理

- boot 时 `[mem-layout]` 打印各 static 地址；
- 心跳 `(raw=%d)`、`?xNN` hex fallback；
- `app_safety_dbg_addr_*()` 等 debug API。

---

## 4. 根因（map 文件定案）

### 4.1 内存布局

`.map` 中 SRAM 末尾（旧布局，栈 1 KB 时）大致为：

```text
0x20200d60   app_safety.s_state（旧：独立 static，后被 struct 挪走）
0x20200d64   s_startup_grace_until_ms
0x20200d68   ms901m.s_state
0x20200d6c   bsp_k230_uart.s_total_rx    ← K230 速率异常的 victim
0x20200d70   safety_state_to_str.buf[8]
0x20200d78   STACK 起始（向下增长，Size=0x400）
0x20201178   栈顶 SP 初值
```

栈只有 **1024 B**，且底边与 `.bss` **零间隙**相邻。

### 4.2 触发路径

本次 commit 在 **boot 必经路径** 重新引入了 `printf("%f")` / `printf("%.2f")`：

- `app_balance.c` → `print_pid_help()`：`R=%.2fm v=%.2fm/s ...`
- `app_circle_demo.c`：start / launch / timeout / done 多处 `%.1f` / `%.2f`

Keil AC6 标准库 `_printf_fp_dec` 单次浮点格式化栈帧典型 **200~300 B**；两个 `%.2f` 叠加调用链后，栈向下越过 `0x20200d78` 约 **24 B**，精确命中旧 `s_state` 地址 `0x20200d60`。

被写入的值 `0x20200da4 = STACK + 0x2C`，是 printf 浮点路径栈帧内某个局部 buffer / 指针——典型栈溢出写脏全局变量，不是「神秘 wild write」。

### 4.3 与历史注释一致

`template/keil/startup_mspm0g350x_uvision.s` 早已记录：

- 原 256 B 栈 → `printf("%f")` → HardFault；
- 曾扩到 1 KB，且业务层约定心跳等热路径**禁止 `%f`**，改用手动整数缩放（`BAL_F2_*`）。

本次在 circle demo 新代码中**违反该约定**，复现同类事故。

---

## 5. 修复措施

| 措施 | 说明 |
|------|------|
| 消除运行时 `%f` | `print_pid_help`、`ci?`、`app_circle_demo` 全部改为 mm/cm + `BAL_F2_*` / `CIRC_F1_*` 整数格式化 |
| 栈 1 KB → 2 KB | `Stack_Size EQU 0x800`；栈底下移，与 `.bss` 之间留出 >1 KB 缓冲 |
| 保留 canary + enum 哨兵 | 轻量运行时防护，未来再溢出时能立刻报警 |
| 清理临时诊断代码 | 见 §3.2 |

---

## 6. 经验教训

### 6.1 嵌入式 printf 使用规范（本项目强制）

1. **热路径与 boot 路径禁止 `printf("%f")`、`%.2f`、`%.1f`**。  
   常量参数也不例外——`APP_CIRCLE_RADIUS_M` 写成 `400mm` 即可，无需浮点格式符。

2. **已有范式必须复用**：  
   - 2 位小数 → `BAL_F2_S/I/F`（`app_balance.c`、`app_telemetry.c`）  
   - 1 位小数 → `CIRC_F1_*`（`app_circle_demo.c`）

3. **新增模块打印前**：grep 全仓库 `%f`，Code Review 必查项。

### 6.2 栈与 .bss 布局

1. 每次怀疑内存 corruption，**先看 `.map` 的 `RW_IRAM2`**，对照 victim 地址与 `STACK` 边界。  
2. 栈与 `.bss` 紧邻时，**少量溢出即可写脏关键全局变量**（安全状态、计数器、在线标志）。  
3. 症状「像指针的值」（`0x2020xxxx`）+ victim 距栈底固定偏移 → **优先怀疑栈溢出**，而非 UART 越界。

### 6.3 安全状态机

1. `s_state` 应用 **struct 打包 + canary**，不要依赖链接器把独立 static 排在一起。  
2. 枚举存安全关键状态时，用 **哨兵值撑满 4 字节**，避免 `--short-enums` 与邻接写入的耦合。  
3. `app_safety_get_state()` 的 corruption 检测 + auto-heal 成本低，建议长期保留。

### 6.4 调试策略

1. 先区分 **显示 bug** vs **数据被写脏**（`raw=` 诊断有价值，但不必长期留在心跳里）。  
2. **Clean & Rebuild** 只能排除 stale `.o`，不能替代 map / 栈分析。  
3. 把结论写进 `LessonsLearned`，比注释散落在 startup 文件里更易被新人看到。

---

## 7. 相关文件

| 文件 | 角色 |
|------|------|
| `template/keil/startup_mspm0g350x_uvision.s` | 栈大小与演进注释 |
| `template/app/app_safety.c` / `.h` | canary、corruption 检测、enum 哨兵 |
| `template/app/app_balance.c` | 心跳、`BAL_F2_*`、`print_pid_help` |
| `template/app/app_circle_demo.c` | `CIRC_F1_*`、circle 日志 |
| `template/app/app_telemetry.c` | 浮点格式化规范参考 |
| `EIDE/build/.../LP_MSPM0G3507.map` | 链接后内存布局（定案证据） |

