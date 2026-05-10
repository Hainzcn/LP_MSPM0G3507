# BSP_FLASH_PAD Flash 烧录对齐填充 学习笔记

## 📖 一、整体概述

### 1.1 这个文件是什么？

`bsp_flash_pad.c` 是整个项目中最短的文件——只有 3 行有效代码。
但它解决了一个"不解决就烧录失败"的硬问题。

它做的事情极其简单：**在程序的 Flash 镜像末尾加一个 8 字节的占位变量**，
使整个镜像的大小变成 8 字节的倍数。

### 1.2 为什么需要这个文件？

MSPM0G3507 的 Flash 控制器每次只能写 **64 位（8 字节）**。
烧录工具 dslite 在烧录前会校验：程序文件大小必须是 8 的倍数。
如果不是 → 直接报错拒绝烧录。

```
正常情况（OK）：
  程序大小 = 12600 字节 → 12600 ÷ 8 = 1575 ✓ 整除

异常情况（NG）：
  程序大小 = 12588 字节 → 12588 ÷ 8 = 1573.5 ✗ 不整除
  dslite 报错："Length of block is 12588, but it should be divisible by 8"
```

这个文件就是用来把 12588 补到 12600 的——在末尾多写 4 字节（凑到 8 的倍数）。

### 1.3 文件在项目中的位置

```
┌─────────────────────────────────────────────────────────┐
│              bsp_flash_pad.c（本模块）                   │
│                                                         │
│  const uint64_t _flash_image_pad = 0xFFFFFFFFFFFFFFFF;  │
│  放在 Flash 镜像的最后面，凑齐 8 字节边界                 │
└─────────────────────────────────────────────────────────┘
                         ↑ scatter 文件控制放置位置
┌─────────────────────────────────────────────────────────┐
│            template/keil/mspm0g3507.sct                  │
│                                                         │
│  bsp_flash_pad.o (.flash_pad, +Last)                    │
│  "把 bsp_flash_pad.o 中的 .flash_pad 段放在最后面"       │
└─────────────────────────────────────────────────────────┘
```

---

## 📋 二、核心概念

### 2.1 什么是 Scatter 文件？

Scatter 文件（分散加载文件，`.sct`）是 ARM 链接器使用的配置文件。
它告诉链接器：程序的各个"段"（section）应该放在内存的什么位置。

**本项目的 scatter 文件关键内容**：

```
ER_IROM1 0x00000000 ALIGNALL 8 0x00020000 {
    *.o (RESET, +First)                       ← 复位向量：最前面
    *(InRoot$$Sections)                       ← 标准库根段
    bsp_flash_pad.o (.flash_pad, +Last)       ← 填充：最后面 ⭐
    .ANY (+RO)                                ← 其他只读段：中间
    .ANY (+XO)                                ← 其他只执行段：中间
}
```

关键行 `bsp_flash_pad.o (.flash_pad, +Last)` 的含义：
- `bsp_flash_pad.o`：指定"bsp_flash_pad.c 编译出的目标文件"
- `(.flash_pad, +Last)`：这个文件中的 `.flash_pad` 段，放在所有段的最末尾

### 2.2 为什么必须单独成 .c 文件？

这是整个设计中最重要的知识点——和 ARMCLANG 链接器的分配规则有关。

**链接器的段分配规则**（优先级从高到低）：

| 优先级 | 规则类型 | 示例 |
|--------|---------|------|
| 最高 | 模块名 + 段名 | `bsp_flash_pad.o (.flash_pad, +Last)` ✅ |
| 中 | 通配模块名 | `*.o (RESET, +First)` |
| 低 | .ANY 通配符 | `.ANY (+RO)` |

如果 `_flash_image_pad` 放在 `main.c` 中，链接器看到的 selector 是：
```
.ANY3 (.flash_pad, +Last)   ← .ANY 的优先级低于前面的 .ANY1 (+RO)
```

实测 `.ANY1 (+RO)` 先抢走了段，`.flash_pad` 没被放到最后。

**结论**：必须使用模块级 selector（指定具体 `.o` 文件名），才能保证优先级高于 `.ANY`。

### 2.3 +Last 的额外约束

ARMCLANG 的 `+Last` 还有一个约束：只能出现在"单 section selector"的后面。

```
✅ 正确：bsp_flash_pad.o (.flash_pad, +Last)   ← 指定了具体 section
❌ 错误：bsp_flash_pad.o (+Last)                ← 没有指定具体 section
         链接器报 L6234E 错误
```

所以 selector 的写法必须是：**`<模块名>.o (<段名>, +Last)`**——模块名+段名双锁定。

---

## 🧩 三、编译属性详解

```c
__attribute__((used, section(".flash_pad"), aligned(8)))
const uint64_t _flash_image_pad = 0xFFFFFFFFFFFFFFFFULL;
```

### 3.1 `used` —— 防止被删除

ARMCLANG 在 `-O2` 优化下会执行"死代码消除"（dead-strip）。
如果变量看起来没有被任何代码引用，链接器可能把它删除。

`used` 属性强制编译器保留这个变量，即使它看起来"没有被使用"。

### 3.2 `section(".flash_pad")` —— 自定义段

普通变量放在 `.data` 或 `.rodata` 段中。
`section(".flash_pad")` 告诉编译器把这个变量放在一个自定义的 `.flash_pad` 段中。

scatter 文件通过这个段名找到它并放置到末尾。

### 3.3 `aligned(8)` —— 8 字节对齐

保证这个变量的起始地址是 8 的倍数。
配合它本身的大小也是 8 字节（uint64_t），
它的结束地址同样是 8 的倍数。

由于它被放在最后面，整个程序的结束地址也变成了 8 的倍数。

### 3.4 `const` —— 放在 ROM 中

`const` 修饰的变量放在 Flash（ROM）区域，不占用宝贵的 RAM。
这正是我们需要的——填充数据应该占 Flash 而不是 RAM。

### 3.5 为什么用 `uint64_t`？

`uint64_t` 正好 8 字节（64 位 ÷ 8 = 8 字节）。
配合 `aligned(8)` 和 `+Last`，这个段自身大小 = 8、起始对齐到 8 → 末尾天然对齐到 8。

### 3.6 为什么填充值用全 0xFF？

Flash 的"擦除态"就是全 1（每个 bit = 1，即 0xFF）。
写 0xFF 到 Flash 等同于"不操作"——不需要实际写功耗，不消耗 Flash 擦写寿命。

这 8 字节只是"占位符"，让镜像凑到 8 的倍数边界。

---

## 🔄 四、对齐效果图解

```
假设编译出的程序（不含填充）大小为 12588 字节：

  0x0000 ┌────────────────┐
         │  .text (代码)   │
         │  .rodata (数据)  │
         │  ...            │  12588 字节
         │                 │
  0x312B ├────────────────┤  ← 12588 = 0x312C，不是 8 的倍数
         │  (擦除态 0xFF)  │  ← 缺乏对齐，直接在这里结束
  0x312C └────────────────┘     dslite 报错！

加上填充后（12588 → 12600，多了 12 字节凑到 8 的倍数）：

  0x0000 ┌────────────────┐
         │  .text (代码)   │
         │  .rodata (数据)  │
         │  ...            │  12588 字节
         │                 │
  0x312B ├────────────────┤
         │  _flash_image   │  8 字节（0xFFFFFFFFFFFFFFFF）
         │  _pad           │  +Last 强制放在这里
  0x3133 ├────────────────┤  ← 12600 = 0x3138 ≡ 0 (mod 8) ✅
         │  (到此为止)     │
         └────────────────┘
```

---

## 🔍 五、C 语言关键语法知识点

### 5.1 `__attribute__` 编译器扩展

`__attribute__((...))` 是 GCC/ARMCLANG 提供的编译器扩展语法，
用于给变量或函数附加额外的编译属性。

它不是 C 标准的一部分，但在嵌入式开发中广泛使用。
IAR 和 MSVC 有自己的等价语法（`#pragma` 或 `__declspec`）。

### 5.2 `uint64_t` 定宽类型

来自 `<stdint.h>`，保证在所有平台上都是 64 位（8 字节）。
整个对齐方案建立在这个"精确 8 字节"的前提上。

### 5.3 ULL 后缀

`0xFFFFFFFFFFFFFFFFULL` 中的 `ULL` 表示 `unsigned long long`。
在 ARMCLANG 中，`unsigned long long` = 64 位，和 `uint64_t` 匹配。

`U` = unsigned，`LL` = long long。组合起来就是"无符号 64 位整数"。

---

## 🐛 六、常见踩坑

| 现象 | 原因 | 解决方法 |
|------|------|---------|
| dslite 报"not divisible by 8" | 填充未生效 | 检查 scatter 文件 selector 写法 |
| 填充后仍不对齐 | selector 被 .ANY 抢了 | 必须用 `模块.o (段名, +Last)` 双锁定 |
| 链接器报 L6234E | +Last 前没有指定具体 section | 加 section 名，如 `(.flash_pad, +Last)` |
| 变量被优化掉了 | 忘了 `used` 属性 | 加 `__attribute__((used))` |
| 填充占用了 RAM | 忘了 `const` | 加 `const` 修饰，放在 Flash 中 |

---

> 本文档配合 `bsp_flash_pad.c` 和 `mspm0g3507.sct` 中的详细注释阅读效果最佳。加油！
