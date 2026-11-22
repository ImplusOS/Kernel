#pragma once
#include <stdint.h>

/*
 * The shared Linux syscall implementation and the ABI front ends on top of it.
 *
 * Syscall_LinuxCompat.c implements Linux once, in the x86_64 syscall
 * numbering and with the x86_64 structure layouts. The other two Linux ABIs
 * are thin front ends that translate a call into that form and the result
 * back:
 *
 *   Compat/Linux/i386/Syscall_LinuxI386.c    Linux i386 on the x86_64 kernel
 *                                            (int $0x80, ILP32 structures)
 *   Compat/Linux/arm64/Syscall_LinuxArm64.c  Linux AArch64 on the arm64 kernel
 *                                            (svc #0, asm-generic numbers)
 *
 * A front end calls linux_syscall_core() with the x86_64 number and
 * arguments. The core leaves the result in `call` instead of writing it into
 * the caller's registers (the front end may have a structure to convert
 * first) and, when the call has to be re-executed (a blocking call parked
 * until something is ready), re-arms the caller's *own* syscall number --
 * `orig_num` -- so the program traps with exactly the call it made.
 */
typedef struct {
    uint8_t  abi;        /* PROCESS_ABI_LINUX_* of the calling process */
    uint64_t orig_num;   /* the syscall number the program used */
    int      restarted;  /* out: re-armed for re-execution, no result */
    int64_t  result;     /* out: the result, when !restarted */
} linux_call_t;

uint64_t linux_syscall_core(uint64_t saved_rsp, linux_call_t *call,
                            uint64_t num,
                            uint64_t arg1, uint64_t arg2, uint64_t arg3,
                            uint64_t arg4, uint64_t arg5, uint64_t arg6);

/* Write a syscall result into the caller's register frame (EAX for i386,
 * RAX for x86_64, x0 for AArch64). */
void linux_frame_set_result(uint64_t saved_rsp, uint8_t abi, int64_t value);

/* Front-end registration, called from linux_compat_layer_register(). */
void linux_i386_compat_layer_register(void);
void linux_arm64_compat_layer_register(void);
