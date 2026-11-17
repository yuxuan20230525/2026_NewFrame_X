cmake_minimum_required(VERSION 3.20)

# 用法 (推荐 Ninja, 自动并行编译):
#   cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build
#   cmake --build build --target flash
project(TDTFrame C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# MCU 配置 — 移植其他芯片时只需改这里
# ============================================================================
set(MCU_ARCH cortex-m4)
set(MCU_FLOAT_ABI hard)
set(MCU_FPU fpv4-sp-d16)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGH6_FLASH.ld)

set(MCU_FLAGS
    -mcpu=${MCU_ARCH}
    -mthumb
    -mfloat-abi=${MCU_FLOAT_ABI}
    -mfpu=${MCU_FPU}
)

# ============================================================================
# 编译宏定义
# ============================================================================
add_compile_definitions(
    STM32F40_41xxx
    USE_STDPERIPH_DRIVER
    ARM_MATH_CM4
    USE_STM324xG_EVAL
    USB_OTG_HS_CORE
    USE_USB_OTG_HS
    USE_EMBEDDED_PHY
)

# ============================================================================
# 公共编译选项
# ============================================================================
add_compile_options(
    ${MCU_FLAGS}
    -Og
    -g
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-sign-compare
)

# C 特有选项
set(CMAKE_C_STANDARD 11)
add_compile_options(
    "$<$<COMPILE_LANGUAGE:C>:-Wno-old-style-declaration>"
)

# C++ 特有选项
set(CMAKE_CXX_STANDARD 17)
add_compile_options(
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fpermissive>"
    "$<$<COMPILE_LANGUAGE:CXX>:-include$<SEMICOLON>${CMAKE_SOURCE_DIR}/gcc_compat.h>"
)

# ASM 选项
add_compile_options(
    "$<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>"
)

# ============================================================================
# 头文件路径
# ============================================================================
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/ST_Lib/inc
    ${CMAKE_SOURCE_DIR}/TDT_Alg/inc
    ${CMAKE_SOURCE_DIR}/TDT_Bsp/inc
    ${CMAKE_SOURCE_DIR}/TDT_Device/inc
    ${CMAKE_SOURCE_DIR}/TDT_Task/inc
    ${CMAKE_SOURCE_DIR}/TDT_Widget/inc
    ${CMAKE_SOURCE_DIR}/USB_BOARD/inc
    ${CMAKE_SOURCE_DIR}/USB_CDC/inc
    ${CMAKE_SOURCE_DIR}/USB_DRIVE/inc
    ${CMAKE_SOURCE_DIR}/USB_LIB/inc
    ${CMAKE_SOURCE_DIR}/USB_USER/inc
    ${CMAKE_SOURCE_DIR}/User
    ${CMAKE_SOURCE_DIR}/DSP_Lib
)

# ============================================================================
# 源文件收集 — 以后加模块只需要加一行 GLOB
# ============================================================================

# ST 标准外设库 (只编译用到的)
set(ST_LIB_SOURCES
    ST_Lib/src/misc.c
    ST_Lib/src/stm32f4xx_adc.c
    ST_Lib/src/stm32f4xx_can.c
    ST_Lib/src/stm32f4xx_crc.c
    ST_Lib/src/stm32f4xx_dma.c
    ST_Lib/src/stm32f4xx_exti.c
    ST_Lib/src/stm32f4xx_flash.c
    ST_Lib/src/stm32f4xx_gpio.c
    ST_Lib/src/stm32f4xx_i2c.c
    ST_Lib/src/stm32f4xx_iwdg.c
    ST_Lib/src/stm32f4xx_pwr.c
    ST_Lib/src/stm32f4xx_rcc.c
    ST_Lib/src/stm32f4xx_rng.c
    ST_Lib/src/stm32f4xx_rtc.c
    ST_Lib/src/stm32f4xx_sdio.c
    ST_Lib/src/stm32f4xx_spi.c
    ST_Lib/src/stm32f4xx_syscfg.c
    ST_Lib/src/stm32f4xx_tim.c
    ST_Lib/src/stm32f4xx_usart.c
    ST_Lib/src/stm32f4xx_wwdg.c
    ST_Lib/src/system_stm32f4xx.c
)

# TDT 各模块 — 用 GLOB 自动收集
file(GLOB TDT_ALG_SOURCES     TDT_Alg/src/*.c TDT_Alg/src/*.cpp)
file(GLOB TDT_BSP_SOURCES     TDT_Bsp/src/*.c TDT_Bsp/src/*.cpp)
file(GLOB TDT_DEVICE_SOURCES  TDT_Device/src/*.c TDT_Device/src/*.cpp)
file(GLOB TDT_TASK_SOURCES    TDT_Task/src/*.c TDT_Task/src/*.cpp)
file(GLOB TDT_WIDGET_SOURCES  TDT_Widget/src/*.c TDT_Widget/src/*.cpp)

# USB
file(GLOB USB_BOARD_SOURCES   USB_BOARD/src/*.c)
file(GLOB USB_CDC_SOURCES     USB_CDC/src/*.c)
file(GLOB USB_DRIVE_SOURCES   USB_DRIVE/src/*.c)
# USB_LIB 只用 Device 部分, 不 GLOB 避免引入 Host 文件
set(USB_LIB_SOURCES
    USB_LIB/src/usbd_core.c
    USB_LIB/src/usbd_ioreq.c
    USB_LIB/src/usbd_req.c
)
file(GLOB USB_USER_SOURCES    USB_USER/src/*.c)

# User
file(GLOB USER_SOURCES        User/*.c User/*.cpp)

# GCC 兼容: 替代 Keil .lib 的实现
set(GCC_COMPAT_SOURCES
    cmsis_dsp_stubs.c
)

# Startup 汇编 (GCC 语法)
set(STARTUP_SOURCE startup_stm32f407xx_gcc.s)

# ============================================================================
# 构建目标
# ============================================================================
add_executable(${PROJECT_NAME}.elf
    ${ST_LIB_SOURCES}
    ${TDT_ALG_SOURCES}
    ${TDT_BSP_SOURCES}
    ${TDT_DEVICE_SOURCES}
    ${TDT_TASK_SOURCES}
    ${TDT_WIDGET_SOURCES}
    ${USB_BOARD_SOURCES}
    ${USB_CDC_SOURCES}
    ${USB_DRIVE_SOURCES}
    ${USB_LIB_SOURCES}
    ${USB_USER_SOURCES}
    ${USER_SOURCES}
    ${GCC_COMPAT_SOURCES}
    ${STARTUP_SOURCE}
)

# ============================================================================
# 链接选项
# ============================================================================
target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -Og
    -specs=nosys.specs
    -lstdc++
    -lsupc++
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
    -Wl,--print-memory-usage
    -T${LINKER_SCRIPT}
)

# ============================================================================
# 后处理: 生成 .hex .bin 并打印 size
# ============================================================================
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating HEX and BIN files, printing size info"
)

# ============================================================================
# Flash 烧录
# ============================================================================
add_custom_target(flash
    COMMAND JLinkExe -device STM32F407IG -if swd -speed 4000
            -CommanderScript ${CMAKE_SOURCE_DIR}/flash.jlink
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing with J-Link..."
)
