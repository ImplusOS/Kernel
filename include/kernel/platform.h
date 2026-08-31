#ifndef IMPLUSOS_KERNEL_PLATFORM_H
#define IMPLUSOS_KERNEL_PLATFORM_H

#if defined(__aarch64__) || defined(PLATFORM_ARM64)
#ifndef PLATFORM_ARM64
#define PLATFORM_ARM64 1
#endif
#elif defined(__x86_64__) || defined(PLATFORM_X86_64)
#ifndef PLATFORM_X86_64
#define PLATFORM_X86_64 1
#endif
#else
#error "Unsupported architecture"
#endif

#endif
