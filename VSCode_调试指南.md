# TDT-Frame VSCode 编译调试指南 (GCC + CMake + Ninja)

本项目已从 Keil 迁移到 GCC 工具链，使用 CMake + Ninja 构建，VSCode + Cortex-Debug 调试。

---

## 1. 环境准备

### 1.1 必须安装的工具

| 工具 | 安装命令 | 用途 |
|------|---------|------|
| arm-none-eabi-gcc | `sudo apt install gcc-arm-none-eabi` | 交叉编译工具链 |
| CMake (>=3.20) | `sudo apt install cmake` | 构建系统 |
| Ninja | `sudo apt install ninja-build` | 快速并行编译 |
| arm-none-eabi-gdb | `sudo apt install gdb-multiarch` 或随工具链安装 | GDB 调试器 |

### 1.2 VSCode 必装插件

- **C/C++** (`ms-vscode.cpptools`) — 代码补全、语法高亮
- **Cortex-Debug** (`marus25.cortex-debug`) — ARM 芯片调试
- **CMake Tools** (`ms-vscode.cmake-tools`) — 可选，提供 CMake 语法支持

### 1.3 调试器驱动

- **J-Link**：安装 [J-Link Software Pack](https://www.segger.com/downloads/jlink/)，确保 `JLinkGDBServerCLExe` 在 PATH 中
- **ST-Link**：安装 OpenOCD (`sudo apt install openocd`)

---

## 2. 首次配置与编译

### 2.1 CMake Configure（首次必做）

```bash
cmake -B cmake_build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
```

或在 VSCode 中：`Ctrl+Shift+P` → `Tasks: Run Task` → **Configure**

### 2.2 编译

```bash
cmake --build cmake_build
```

或在 VSCode 中：`Ctrl+Shift+B`（默认 Build 任务）

### 2.3 清理并重新编译

```bash
cmake --build cmake_build --clean-first
```

或运行任务 **Clean Build**。

### 2.4 烧录

```bash
cmake --build cmake_build --target flash
```

或运行任务 **Flash**（依赖 J-Link）。

---

## 3. 调试

### 3.1 已配置的调试方案（launch.json）

| 名称 | 调试器 | 说明 |
|------|--------|------|
| **JLink Flash and Debug** | J-Link | 编译 → 烧录 → 停在 main |
| **Flash and Debug** | ST-Link (OpenOCD) | 编译 → 烧录 → 停在 main |
| **Debug (No Flash)** | ST-Link (OpenOCD) | 不烧录，直接加载 ELF 调试 |
| **Attach (Debug Running)** | ST-Link (OpenOCD) | 附加到正在运行的芯片 |

### 3.2 开始调试

1. 左侧点击 **运行和调试** 图标（或 `Ctrl+Shift+D`）
2. 顶部下拉选择调试方案
3. 按 `F5` 启动

### 3.3 常用调试操作

| 快捷键 | 功能 |
|--------|------|
| `F5` | 继续运行 |
| `F10` | 单步跳过 |
| `F11` | 单步进入 |
| `Shift+F11` | 单步跳出 |
| `Shift+F5` | 停止调试 |

### 3.4 查看外设寄存器

Cortex-Debug 调试时，左侧 **CORTEX PERIPHERALS** 面板可以展开查看 STM32F407 的外设寄存器（GPIO、TIM、CAN 等）。

### 3.5 Live Watch（实时变量监视）

J-Link 方案已启用 `liveWatch`，调试时可在 **LIVE WATCH** 面板添加变量名，不暂停即可实时查看变量值。

---

## 4. 重要注意事项

### 4.1 新增 .c/.cpp 文件后必须重新 Configure

CMakeLists.txt 使用 `file(GLOB)` 收集源文件，**GLOB 只在 configure 阶段执行一次**。如果你新增了 `.c` 或 `.cpp` 源文件：

```bash
# 必须重新 configure，否则新文件不会被编译！
cmake -B cmake_build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

# 然后再编译
cmake --build cmake_build
```

**典型症状**：链接时报 `undefined reference to ...`，而对应的源文件明明存在。

### 4.2 Makefile 与 CMake 双构建系统

项目同时保留了 Makefile 和 CMakeLists.txt：

| | Makefile | CMakeLists.txt |
|---|---------|----------------|
| 构建目录 | `build/` | `cmake_build/` |
| 编译命令 | `make` | `cmake --build cmake_build` |
| 源文件管理 | 手动列出每个文件 | `file(GLOB)` 自动收集 |
| VSCode 集成 | 无 | tasks.json + launch.json |

**VSCode 调试使用的是 CMake 构建**（`cmake_build/TDTFrame.elf`）。如果你只用 `make` 编译，调试时加载的仍然是旧的 CMake 产物。

### 4.3 从 Keil 迁移的关键差异

| 项目 | Keil | GCC (本项目) |
|------|------|-------------|
| DSP 库 | `arm_cortexM4lf_math.lib` 预编译库 | `cmsis_dsp_stubs.c` 手写实现 |
| `__errno` 符号 | 自带 | `gcc_errno.c` 提供 |
| 启动文件 | `startup_stm32f40_41xxx.s` (ARM 语法) | `startup_stm32f407xx_gcc.s` (GCC 语法) |
| 链接脚本 | `.sct` scatter file | `STM32F407IGH6_FLASH.ld` |
| 兼容头文件 | 不需要 | `gcc_compat.h`（处理 Keil 特有关键字） |

### 4.4 编译报 undefined reference 排查步骤

1. **确认源文件是否存在**：`ls TDT_Alg/src/XXX.cpp`
2. **确认是否被 CMake 收集到**：
   ```bash
   grep -r "XXX" cmake_build/build.ninja
   ```
3. 如果文件存在但 build.ninja 里没有 → 重新 configure（见 4.1）
4. 如果是 `arm_mat_xxx` 等 DSP 函数 → 检查 `cmsis_dsp_stubs.c` 是否实现了该函数
5. 如果是 `__errno` → 检查 `gcc_errno.c` 是否存在

### 4.5 调试器连接不上

- **J-Link**：确认 USB 连接，运行 `JLinkExe` 看能否识别芯片
- **ST-Link**：确认驱动，运行 `openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg` 测试
- **权限问题**：Linux 下可能需要 udev 规则，或用 `sudo` 运行

### 4.6 编译优化等级

当前使用 `-Og`（调试优化），适合开发调试。如需发布版本，可在 CMakeLists.txt 中改为 `-O2` 或 `-Os`。

---

## 5. 项目目录结构

```
2026_NewFrame/
├── cmake/                  # CMake 工具链文件
│   └── arm-none-eabi.cmake
├── cmake_build/            # CMake 构建输出 (gitignore)
│   ├── TDTFrame.elf        # 可执行文件 (调试用)
│   ├── TDTFrame.bin        # 二进制固件 (烧录用)
│   ├── TDTFrame.hex        # HEX 固件
│   └── TDTFrame.map        # 内存映射
├── .vscode/
│   ├── launch.json         # 调试配置
│   ├── tasks.json          # 编译/烧录任务
│   └── settings.json       # 编辑器设置
├── ST_Lib/                 # STM32 标准外设库
├── TDT_Alg/                # 算法模块 (PID, IMU, 滤波等)
├── TDT_Bsp/                # 板级支持 (ADC, CAN, SPI, IIC等)
├── TDT_Device/             # 设备驱动 (电机, 陀螺仪, 遥控器等)
├── TDT_Task/               # 任务模块
├── TDT_Widget/             # 组件模块
├── USB_*/                  # USB CDC 相关
├── User/                   # 用户代码 (main, board, schedule)
├── DSP_Lib/                # CMSIS DSP 头文件
├── cmsis_dsp_stubs.c       # DSP 函数实现 (替代 Keil .lib)
├── gcc_errno.c             # __errno 符号 (newlib 需要)
├── gcc_compat.h            # Keil → GCC 兼容宏
├── startup_stm32f407xx_gcc.s  # GCC 启动汇编
├── STM32F407IGH6_FLASH.ld  # GCC 链接脚本
├── CMakeLists.txt          # CMake 构建脚本
├── Makefile                # Make 构建脚本 (备用)
└── flash.jlink             # J-Link 烧录脚本
```

---

## 6. 快速上手流程

```bash
# 1. 首次 configure
cmake -B cmake_build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

# 2. 编译
cmake --build cmake_build

# 3. 烧录 (J-Link)
cmake --build cmake_build --target flash

# 4. 调试 → VSCode 按 F5
```
