# EIDE 构建修复日志

> 日期：2026-05-01
> 关联文档：[Stage1-IMU-BT-Telemetry.md](Stage1-IMU-BT-Telemetry.md) §8

---

## 1. SYSCONFIG_WEAK 宏未定义（ARM Compiler 6）

**现象**：编译 `ti_msp_dl_config.c` 报 8 个 `unknown type name 'SYSCONFIG_WEAK'` 错误。

**根因**：`ti_msp_dl_config.h` 中 `SYSCONFIG_WEAK` 宏的条件编译只覆盖了 TI / IAR / GCC 三种编译器，未覆盖 Keil ARM Compiler 6 (armclang)。armclang 定义的是 `__ARMCC_VERSION`，不定义上述任何宏。

**修复**：在 `ti_msp_dl_config.h` 的条件编译链末尾增加 ARM Compiler 6 分支：

```c
#elif defined(__ARMCC_VERSION)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif
```

**涉及文件**：
- `EIDE/ti_msp_dl_config.h`
- `template/ti_msp_dl_config.h`

> ⚠ 此文件由 SysConfig 自动生成，重新运行 SysConfig 后修改会被覆盖，需重新添加。

---

## 2. deviceName / packDir 为 null

**现象**：EIDE 项目 `eide.yml` 中 `deviceName: null` 和 `packDir: null`，无法识别目标设备 MSPM0G3507。

**根因**：项目从 Keil uVision 迁移到 EIDE 时，设备支持包配置未正确设置。

**修复**：
1. 从 TI 官方下载 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP.1.3.1.pack`
2. 解压到 `EIDE/.pack/TexasInstruments/MSPM0G1X0X_G3X0X_DFP.1.3.1/`
3. 更新 `eide.yml`：
   ```yaml
   deviceName: TexasInstruments::MSPM0G1X0X_G3X0X_DFP::MSPM0G3507
   packDir: .pack/TexasInstruments/MSPM0G1X0X_G3X0X_DFP.1.3.1
   ```

**涉及文件**：
- `EIDE/.eide/eide.yml`

---

## 3. ti_msp_dl_config.h 找不到

**现象**：编译 `bsp_gpio.h` 时 `fatal error: 'ti_msp_dl_config.h' file not found`。

**根因**：EIDE 项目的 `incList` 缺少 EIDE 工作目录（`ti_msp_dl_config.h` 所在目录）。

**修复**：在 `eide.yml` 的 `incList` 中添加 `.`（EIDE 工作目录）。此修改在后续 `srcDirs` 修复后自动解决。

---

## 4. scatter 文件 L6236E: No section matches selector

**现象**：链接阶段报 `L6236E: No section matches selector - no section to be FIRST/LAST`，指向 `mspm0g3507.sct` 第 41 行 `bsp_flash_pad.o (.flash_pad, +Last)`。

**根因**：`bsp_flash_pad.c` 不在 EIDE 项目源文件列表中，链接器找不到 `.flash_pad` section。

**修复过程**：
1. 首次尝试：手动编辑 `eide.yml` 添加源文件 → **失败**，EIDE 加载时覆盖外部修改
2. 第二次尝试：将 `_flash_image_pad` 变量移到 `main.c`，scatter 改为 `main.o (.flash_pad, +Last)` → **部分成功**，但后续 `srcDirs` 修复导致符号重复定义
3. 最终方案：使用 `srcDirs` 自动扫描源文件目录，scatter 保持 `bsp_flash_pad.o (.flash_pad, +Last)`

**涉及文件**：
- `EIDE/.eide/eide.yml` — 添加 `srcDirs`
- `template/main.c` — 移除临时添加的 `_flash_image_pad`，恢复注释
- `template/keil/mspm0g3507.sct` — 恢复 `bsp_flash_pad.o`

---

## 5. Undefined symbol 链接错误（10 个符号）

**现象**：链接阶段报 `bsp_gpio_init`、`mpu6050_init`、`app_telemetry_run` 等 10 个符号未定义。

**根因**：`hardware/`、`middle/`、`app/` 目录下的 `.c` 文件未加入 EIDE 项目。

**修复**：在 `eide.yml` 中设置 `srcDirs` 自动扫描：

```yaml
srcDirs:
  - ../template/hardware
  - ../template/middle
  - ../template/app
```

**涉及文件**：
- `EIDE/.eide/eide.yml`

---

## 修复链总结

```
SYSCONFIG_WEAK 未定义 → 修复头文件条件编译
    ↓
deviceName/packDir null → 下载 DFP Pack + 更新 eide.yml
    ↓
ti_msp_dl_config.h 找不到 → incList 添加 EIDE 工作目录
    ↓
scatter L6236E → srcDirs 自动包含 bsp_flash_pad.c
    ↓
10 个 Undefined symbol → srcDirs 自动包含 hardware/middle/app
    ↓
_flash_image_pad 重复定义 → 从 main.c 移除，scatter 恢复 bsp_flash_pad.o
```

---

## 6. 烧录工具链配置（DSLite + XDS110）

**需求**：通过 USB 直接烧录 MSPM0G3507 LaunchPad，与 CCS 相同的方式。

**方案**：使用 CCS 自带的 DSLite 命令行工具 + XDS110 板载调试探针。

**配置**：

| 项目 | 值 |
| --- | --- |
| 烧录器类型 | Custom (DSLite) |
| DSLite 路径 | `D:\ccs\ccs\ccs_base\DebugServer\bin\DSLite.exe` |
| ccxml 配置 | `C:\Users\liang\workspace_ccstheia\111\targetConfigs\MSPM0G3507.ccxml` |
| 烧录命令 | `DSLite.exe flash -c "MSPM0G3507.ccxml" -f "${ExecutableName}.axf"` |
| 擦除命令 | `DSLite.exe flash -c "MSPM0G3507.ccxml" -e` |

**eide.yml 变更**：

```yaml
uploadConfigMap:
  Custom:
    bin: ""
    commandLine: '"D:\ccs\ccs\ccs_base\DebugServer\bin\DSLite.exe" flash -c "C:\Users\liang\workspace_ccstheia\111\targetConfigs\MSPM0G3507.ccxml" -f "${ExecutableName}.axf"'
    eraseChipCommand: '"D:\ccs\ccs\ccs_base\DebugServer\bin\DSLite.exe" flash -c "C:\Users\liang\workspace_ccstheia\111\targetConfigs\MSPM0G3507.ccxml" -e'
  JLink:
    baseAddr: ""
    bin: ""
    cpuInfo:
      cpuName: "Cortex-M0+"
      vendor: "Texas Instruments"
    otherCmds: ""
    proType: 1
    speed: 8000
uploader: Custom
```

**涉及文件**：
- `EIDE/.eide/eide.yml`

**备注**：
- ccxml 文件来自 CCS Theia 工作区，使用 XDS110 USB Debug Probe + SWD 模式
- 同时保留了 J-Link 配置（cpuName 已修正为 `Cortex-M0+`），可通过 EIDE 切换烧录器
- 烧录前确保 LaunchPad 已通过 USB 连接，XDS110 驱动已安装

---

## 7. EIDE 烧录脚本（DSLite + XDS110 板载调试器）

**需求**：EIDE 直接点击烧录按钮，通过 USB + XDS110 烧录 MSPM0G3507。

**方案**：创建批处理脚本封装 DSLite 命令，EIDE Custom uploader 调用脚本。

**新增文件**：

| 文件 | 用途 |
| --- | --- |
| `EIDE/flash_mspm0.bat` | 烧录：`DSLite flash -c ccxml -f axf` |
| `EIDE/erase_mspm0.bat` | 擦除：`DSLite flash -c ccxml -e` |

**eide.yml 变更**：

```yaml
uploadConfigMap:
  Custom:
    bin: ""
    commandLine: '"${workspaceFolder}/flash_mspm0.bat" "${outDir}/${ExecutableName}.axf"'
    eraseChipCommand: '"${workspaceFolder}/erase_mspm0.bat"'
uploader: Custom
```

**使用方式**：
1. 确保 LaunchPad 通过 USB 连接电脑
2. EIDE 中点击 **Upload** 按钮即可烧录
3. 如需擦除整片 Flash，点击 **Erase** 按钮

**依赖**：
- DSLite: `D:\ccs\ccs\ccs_base\DebugServer\bin\DSLite.exe`
- ccxml: `C:\Users\liang\workspace_ccstheia\111\targetConfigs\MSPM0G3507.ccxml`
- XDS110 驱动（CCS 安装时自带）

**UniFlash v9.5.0 安装包**已下载到 `C:\Users\liang\Downloads\uniflash_sl.9.5.0.5651.exe`（341MB），安装后可将脚本中 DSLite 路径替换为 UniFlash 版本。
