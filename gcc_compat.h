/**
 * gcc_compat.h — Keil ARM Compiler → GCC 兼容层
 * 用 -include gcc_compat.h 自动注入到所有编译单元
 */
#ifndef GCC_COMPAT_H
#define GCC_COMPAT_H

#ifdef __GNUC__

/* Keil __nop() → GCC inline asm */
#ifndef __nop
#define __nop() __asm volatile("nop")
#endif

#endif /* __GNUC__ */
#endif /* GCC_COMPAT_H */
