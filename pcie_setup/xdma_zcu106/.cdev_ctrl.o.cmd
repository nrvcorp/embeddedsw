savedcmd_cdev_ctrl.o := gcc -Wp,-MMD,./.cdev_ctrl.o.d  -nostdinc -I/usr/src/linux/linux-6.16.1/arch/x86/include -I/usr/src/linux/linux-6.16.1/arch/x86/include/generated -I/usr/src/linux/linux-6.16.1/include -I/usr/src/linux/linux-6.16.1/include -I/usr/src/linux/linux-6.16.1/arch/x86/include/uapi -I/usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi -I/usr/src/linux/linux-6.16.1/include/uapi -I/usr/src/linux/linux-6.16.1/include/generated/uapi -include /usr/src/linux/linux-6.16.1/include/linux/compiler-version.h -include /usr/src/linux/linux-6.16.1/include/linux/kconfig.h -include /usr/src/linux/linux-6.16.1/include/linux/compiler_types.h -D__KERNEL__ -std=gnu11 -fshort-wchar -funsigned-char -fno-common -fno-PIE -fno-strict-aliasing -mno-sse -mno-mmx -mno-sse2 -mno-3dnow -mno-avx -fcf-protection=branch -fno-jump-tables -m64 -falign-jumps=1 -falign-loops=1 -mno-80387 -mno-fp-ret-in-387 -mpreferred-stack-boundary=3 -mskip-rax-setup -march=x86-64 -mtune=generic -mno-red-zone -mcmodel=kernel -mstack-protector-guard-reg=gs -mstack-protector-guard-symbol=__ref_stack_chk_guard -Wno-sign-compare -fno-asynchronous-unwind-tables -mindirect-branch=thunk-extern -mindirect-branch-register -mfunction-return=thunk-extern -fno-jump-tables -fpatchable-function-entry=16,16 -fno-delete-null-pointer-checks -O2 --param=allow-store-data-races=0 -fstack-protector-strong -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-stack-clash-protection -pg -mrecord-mcount -mfentry -DCC_USING_FENTRY -falign-functions=16 -fno-strict-overflow -fno-stack-check -fconserve-stack -fno-builtin-wcslen -Wall -Wextra -Wundef -Werror=implicit-function-declaration -Werror=implicit-int -Werror=return-type -Werror=strict-prototypes -Wno-format-security -Wno-trigraphs -Wno-frame-address -Wno-address-of-packed-member -Wmissing-declarations -Wmissing-prototypes -Wframe-larger-than=1024 -Wno-main -Wvla-larger-than=1 -Wno-pointer-sign -Wcast-function-type -Wno-array-bounds -Wno-stringop-overflow -Wno-alloc-size-larger-than -Wimplicit-fallthrough=5 -Werror=date-time -Werror=incompatible-pointer-types -Werror=designated-init -Wunused -Wno-unused-but-set-variable -Wno-unused-const-variable -Wno-packed-not-aligned -Wno-format-overflow -Wno-format-truncation -Wno-stringop-truncation -Wno-override-init -Wno-missing-field-initializers -Wno-type-limits -Wno-shift-negative-value -Wno-maybe-uninitialized -Wno-sign-compare -Wno-unused-parameter -g -I/home/nrvfpga01/xsrc/embeddedsw/pcie_setup/include  -fsanitize=bounds-strict -fsanitize=shift -fsanitize=bool -fsanitize=enum    -DMODULE  -DKBUILD_BASENAME='"cdev_ctrl"' -DKBUILD_MODNAME='"xdma_zcu106"' -D__KBUILD_MODNAME=kmod_xdma_zcu106 -c -o cdev_ctrl.o cdev_ctrl.c  

source_cdev_ctrl.o := cdev_ctrl.c

deps_cdev_ctrl.o := \
  /usr/src/linux/linux-6.16.1/include/linux/compiler-version.h \
    $(wildcard include/config/CC_VERSION_TEXT) \
  /usr/src/linux/linux-6.16.1/include/linux/kconfig.h \
    $(wildcard include/config/CPU_BIG_ENDIAN) \
    $(wildcard include/config/BOOGER) \
    $(wildcard include/config/FOO) \
  /usr/src/linux/linux-6.16.1/include/linux/compiler_types.h \
    $(wildcard include/config/DEBUG_INFO_BTF) \
    $(wildcard include/config/PAHOLE_HAS_BTF_TAG) \
    $(wildcard include/config/FUNCTION_ALIGNMENT) \
    $(wildcard include/config/CC_HAS_SANE_FUNCTION_ALIGNMENT) \
    $(wildcard include/config/X86_64) \
    $(wildcard include/config/ARM64) \
    $(wildcard include/config/LD_DEAD_CODE_DATA_ELIMINATION) \
    $(wildcard include/config/LTO_CLANG) \
    $(wildcard include/config/HAVE_ARCH_COMPILER_H) \
    $(wildcard include/config/CC_HAS_COUNTED_BY) \
    $(wildcard include/config/CC_HAS_MULTIDIMENSIONAL_NONSTRING) \
    $(wildcard include/config/UBSAN_INTEGER_WRAP) \
    $(wildcard include/config/CC_HAS_ASM_INLINE) \
  /usr/src/linux/linux-6.16.1/include/linux/compiler_attributes.h \
  /usr/src/linux/linux-6.16.1/include/linux/compiler-gcc.h \
    $(wildcard include/config/MITIGATION_RETPOLINE) \
    $(wildcard include/config/ARCH_USE_BUILTIN_BSWAP) \
    $(wildcard include/config/SHADOW_CALL_STACK) \
    $(wildcard include/config/KCOV) \
    $(wildcard include/config/CC_HAS_TYPEOF_UNQUAL) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/ioctl.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/ioctl.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/ioctl.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/ioctl.h \
  version.h \
  xdma_cdev.h \
  /usr/src/linux/linux-6.16.1/include/linux/kernel.h \
    $(wildcard include/config/PREEMPT_VOLUNTARY_BUILD) \
    $(wildcard include/config/PREEMPT_DYNAMIC) \
    $(wildcard include/config/HAVE_PREEMPT_DYNAMIC_CALL) \
    $(wildcard include/config/HAVE_PREEMPT_DYNAMIC_KEY) \
    $(wildcard include/config/PREEMPT_) \
    $(wildcard include/config/DEBUG_ATOMIC_SLEEP) \
    $(wildcard include/config/SMP) \
    $(wildcard include/config/MMU) \
    $(wildcard include/config/PROVE_LOCKING) \
    $(wildcard include/config/TRACING) \
    $(wildcard include/config/FTRACE_MCOUNT_RECORD) \
  /usr/src/linux/linux-6.16.1/include/linux/stdarg.h \
  /usr/src/linux/linux-6.16.1/include/linux/align.h \
  /usr/src/linux/linux-6.16.1/include/vdso/align.h \
  /usr/src/linux/linux-6.16.1/include/vdso/const.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/const.h \
  /usr/src/linux/linux-6.16.1/include/linux/array_size.h \
  /usr/src/linux/linux-6.16.1/include/linux/compiler.h \
    $(wildcard include/config/TRACE_BRANCH_PROFILING) \
    $(wildcard include/config/PROFILE_ALL_BRANCHES) \
    $(wildcard include/config/OBJTOOL) \
    $(wildcard include/config/CFI_CLANG) \
    $(wildcard include/config/64BIT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/rwonce.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/rwonce.h \
  /usr/src/linux/linux-6.16.1/include/linux/kasan-checks.h \
    $(wildcard include/config/KASAN_GENERIC) \
    $(wildcard include/config/KASAN_SW_TAGS) \
  /usr/src/linux/linux-6.16.1/include/linux/types.h \
    $(wildcard include/config/HAVE_UID16) \
    $(wildcard include/config/UID16) \
    $(wildcard include/config/ARCH_DMA_ADDR_T_64BIT) \
    $(wildcard include/config/PHYS_ADDR_T_64BIT) \
    $(wildcard include/config/ARCH_32BIT_USTAT_F_TINODE) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/types.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/types.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/int-ll64.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/int-ll64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/bitsperlong.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitsperlong.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/bitsperlong.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/posix_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/stddef.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/stddef.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/posix_types.h \
    $(wildcard include/config/X86_32) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/posix_types_64.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/posix_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/kcsan-checks.h \
    $(wildcard include/config/KCSAN) \
    $(wildcard include/config/KCSAN_WEAK_MEMORY) \
    $(wildcard include/config/KCSAN_IGNORE_ATOMICS) \
  /usr/src/linux/linux-6.16.1/include/linux/limits.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/limits.h \
  /usr/src/linux/linux-6.16.1/include/vdso/limits.h \
  /usr/src/linux/linux-6.16.1/include/linux/linkage.h \
    $(wildcard include/config/ARCH_USE_SYM_ANNOTATIONS) \
  /usr/src/linux/linux-6.16.1/include/linux/stringify.h \
  /usr/src/linux/linux-6.16.1/include/linux/export.h \
    $(wildcard include/config/MODVERSIONS) \
    $(wildcard include/config/GENDWARFKSYMS) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/linkage.h \
    $(wildcard include/config/CALL_PADDING) \
    $(wildcard include/config/MITIGATION_RETHUNK) \
    $(wildcard include/config/MITIGATION_SLS) \
    $(wildcard include/config/FUNCTION_PADDING_BYTES) \
    $(wildcard include/config/UML) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/ibt.h \
    $(wildcard include/config/X86_KERNEL_IBT) \
    $(wildcard include/config/FINEIBT_BHI) \
  /usr/src/linux/linux-6.16.1/include/linux/container_of.h \
  /usr/src/linux/linux-6.16.1/include/linux/build_bug.h \
  /usr/src/linux/linux-6.16.1/include/linux/bitops.h \
  /usr/src/linux/linux-6.16.1/include/linux/bits.h \
  /usr/src/linux/linux-6.16.1/include/linux/const.h \
  /usr/src/linux/linux-6.16.1/include/vdso/bits.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/bits.h \
  /usr/src/linux/linux-6.16.1/include/linux/overflow.h \
  /usr/src/linux/linux-6.16.1/include/linux/typecheck.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/kernel.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/sysinfo.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/generic-non-atomic.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/barrier.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/alternative.h \
    $(wildcard include/config/CALL_THUNKS) \
    $(wildcard include/config/MITIGATION_ITS) \
  /usr/src/linux/linux-6.16.1/include/linux/objtool.h \
    $(wildcard include/config/FRAME_POINTER) \
    $(wildcard include/config/NOINSTR_VALIDATION) \
    $(wildcard include/config/MITIGATION_UNRET_ENTRY) \
    $(wildcard include/config/MITIGATION_SRSO) \
  /usr/src/linux/linux-6.16.1/include/linux/objtool_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/asm.h \
    $(wildcard include/config/KPROBES) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/extable_fixup_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/bug.h \
    $(wildcard include/config/GENERIC_BUG) \
    $(wildcard include/config/DEBUG_BUGVERBOSE) \
  /usr/src/linux/linux-6.16.1/include/linux/instrumentation.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bug.h \
    $(wildcard include/config/BUG) \
    $(wildcard include/config/GENERIC_BUG_RELATIVE_POINTERS) \
  /usr/src/linux/linux-6.16.1/include/linux/once_lite.h \
  /usr/src/linux/linux-6.16.1/include/linux/panic.h \
    $(wildcard include/config/PANIC_TIMEOUT) \
  /usr/src/linux/linux-6.16.1/include/linux/printk.h \
    $(wildcard include/config/MESSAGE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_QUIET) \
    $(wildcard include/config/EARLY_PRINTK) \
    $(wildcard include/config/PRINTK) \
    $(wildcard include/config/PRINTK_INDEX) \
    $(wildcard include/config/DYNAMIC_DEBUG) \
    $(wildcard include/config/DYNAMIC_DEBUG_CORE) \
  /usr/src/linux/linux-6.16.1/include/linux/init.h \
    $(wildcard include/config/MEMORY_HOTPLUG) \
    $(wildcard include/config/HAVE_ARCH_PREL32_RELOCATIONS) \
  /usr/src/linux/linux-6.16.1/include/linux/kern_levels.h \
  /usr/src/linux/linux-6.16.1/include/linux/ratelimit_types.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/param.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/param.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/param.h \
    $(wildcard include/config/HZ) \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/param.h \
  /usr/src/linux/linux-6.16.1/include/linux/spinlock_types_raw.h \
    $(wildcard include/config/DEBUG_SPINLOCK) \
    $(wildcard include/config/DEBUG_LOCK_ALLOC) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/spinlock_types.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/qspinlock_types.h \
    $(wildcard include/config/NR_CPUS) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/qrwlock_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/byteorder.h \
  /usr/src/linux/linux-6.16.1/include/linux/byteorder/little_endian.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/byteorder/little_endian.h \
  /usr/src/linux/linux-6.16.1/include/linux/swab.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/swab.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/swab.h \
  /usr/src/linux/linux-6.16.1/include/linux/byteorder/generic.h \
  /usr/src/linux/linux-6.16.1/include/linux/lockdep_types.h \
    $(wildcard include/config/PROVE_RAW_LOCK_NESTING) \
    $(wildcard include/config/LOCKDEP) \
    $(wildcard include/config/LOCK_STAT) \
  /usr/src/linux/linux-6.16.1/include/linux/dynamic_debug.h \
    $(wildcard include/config/JUMP_LABEL) \
  /usr/src/linux/linux-6.16.1/include/linux/jump_label.h \
    $(wildcard include/config/HAVE_ARCH_JUMP_LABEL_RELATIVE) \
  /usr/src/linux/linux-6.16.1/include/linux/cleanup.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/jump_label.h \
    $(wildcard include/config/HAVE_JUMP_LABEL_HACK) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/nops.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/barrier.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/bitops.h \
    $(wildcard include/config/X86_CMOV) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/rmwcc.h \
  /usr/src/linux/linux-6.16.1/include/linux/args.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/sched.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/arch_hweight.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpufeatures.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/const_hweight.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/instrumented-atomic.h \
  /usr/src/linux/linux-6.16.1/include/linux/instrumented.h \
  /usr/src/linux/linux-6.16.1/include/linux/kmsan-checks.h \
    $(wildcard include/config/KMSAN) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/instrumented-non-atomic.h \
    $(wildcard include/config/KCSAN_ASSUME_PLAIN_WRITES_ATOMIC) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/instrumented-lock.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/le.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/bitops/ext2-atomic-setbit.h \
  /usr/src/linux/linux-6.16.1/include/linux/hex.h \
  /usr/src/linux/linux-6.16.1/include/linux/kstrtox.h \
  /usr/src/linux/linux-6.16.1/include/linux/log2.h \
    $(wildcard include/config/ARCH_HAS_ILOG2_U32) \
    $(wildcard include/config/ARCH_HAS_ILOG2_U64) \
  /usr/src/linux/linux-6.16.1/include/linux/math.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/div64.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/div64.h \
    $(wildcard include/config/CC_OPTIMIZE_FOR_PERFORMANCE) \
  /usr/src/linux/linux-6.16.1/include/linux/minmax.h \
  /usr/src/linux/linux-6.16.1/include/linux/sprintf.h \
  /usr/src/linux/linux-6.16.1/include/linux/static_call_types.h \
    $(wildcard include/config/HAVE_STATIC_CALL) \
    $(wildcard include/config/HAVE_STATIC_CALL_INLINE) \
  /usr/src/linux/linux-6.16.1/include/linux/instruction_pointer.h \
  /usr/src/linux/linux-6.16.1/include/linux/util_macros.h \
    $(wildcard include/config/FOO_SUSPEND) \
  /usr/src/linux/linux-6.16.1/include/linux/wordpart.h \
  /usr/src/linux/linux-6.16.1/include/linux/uaccess.h \
    $(wildcard include/config/PREEMPT_COUNT) \
    $(wildcard include/config/ARCH_HAS_SUBPAGE_FAULTS) \
    $(wildcard include/config/HARDENED_USERCOPY) \
  /usr/src/linux/linux-6.16.1/include/linux/fault-inject-usercopy.h \
    $(wildcard include/config/FAULT_INJECTION_USERCOPY) \
  /usr/src/linux/linux-6.16.1/include/linux/nospec.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched.h \
    $(wildcard include/config/PREEMPT_RT) \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING_NATIVE) \
    $(wildcard include/config/SCHED_INFO) \
    $(wildcard include/config/SCHEDSTATS) \
    $(wildcard include/config/SCHED_CORE) \
    $(wildcard include/config/FAIR_GROUP_SCHED) \
    $(wildcard include/config/RT_GROUP_SCHED) \
    $(wildcard include/config/RT_MUTEXES) \
    $(wildcard include/config/UCLAMP_TASK) \
    $(wildcard include/config/UCLAMP_BUCKETS_COUNT) \
    $(wildcard include/config/KMAP_LOCAL) \
    $(wildcard include/config/THREAD_INFO_IN_TASK) \
    $(wildcard include/config/MEM_ALLOC_PROFILING) \
    $(wildcard include/config/SCHED_CLASS_EXT) \
    $(wildcard include/config/CGROUP_SCHED) \
    $(wildcard include/config/PREEMPT_NOTIFIERS) \
    $(wildcard include/config/BLK_DEV_IO_TRACE) \
    $(wildcard include/config/PREEMPT_RCU) \
    $(wildcard include/config/TASKS_RCU) \
    $(wildcard include/config/TASKS_TRACE_RCU) \
    $(wildcard include/config/MEMCG_V1) \
    $(wildcard include/config/LRU_GEN) \
    $(wildcard include/config/COMPAT_BRK) \
    $(wildcard include/config/CGROUPS) \
    $(wildcard include/config/BLK_CGROUP) \
    $(wildcard include/config/PSI) \
    $(wildcard include/config/PAGE_OWNER) \
    $(wildcard include/config/EVENTFD) \
    $(wildcard include/config/ARCH_HAS_CPU_PASID) \
    $(wildcard include/config/X86_BUS_LOCK_DETECT) \
    $(wildcard include/config/TASK_DELAY_ACCT) \
    $(wildcard include/config/STACKPROTECTOR) \
    $(wildcard include/config/ARCH_HAS_SCALED_CPUTIME) \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING_GEN) \
    $(wildcard include/config/NO_HZ_FULL) \
    $(wildcard include/config/POSIX_CPUTIMERS) \
    $(wildcard include/config/POSIX_CPU_TIMERS_TASK_WORK) \
    $(wildcard include/config/KEYS) \
    $(wildcard include/config/SYSVIPC) \
    $(wildcard include/config/DETECT_HUNG_TASK) \
    $(wildcard include/config/IO_URING) \
    $(wildcard include/config/AUDIT) \
    $(wildcard include/config/AUDITSYSCALL) \
    $(wildcard include/config/DEBUG_MUTEXES) \
    $(wildcard include/config/DETECT_HUNG_TASK_BLOCKER) \
    $(wildcard include/config/TRACE_IRQFLAGS) \
    $(wildcard include/config/UBSAN) \
    $(wildcard include/config/UBSAN_TRAP) \
    $(wildcard include/config/COMPACTION) \
    $(wildcard include/config/TASK_XACCT) \
    $(wildcard include/config/CPUSETS) \
    $(wildcard include/config/X86_CPU_RESCTRL) \
    $(wildcard include/config/FUTEX) \
    $(wildcard include/config/COMPAT) \
    $(wildcard include/config/PERF_EVENTS) \
    $(wildcard include/config/DEBUG_PREEMPT) \
    $(wildcard include/config/NUMA) \
    $(wildcard include/config/NUMA_BALANCING) \
    $(wildcard include/config/RSEQ) \
    $(wildcard include/config/DEBUG_RSEQ) \
    $(wildcard include/config/SCHED_MM_CID) \
    $(wildcard include/config/FAULT_INJECTION) \
    $(wildcard include/config/LATENCYTOP) \
    $(wildcard include/config/KUNIT) \
    $(wildcard include/config/FUNCTION_GRAPH_TRACER) \
    $(wildcard include/config/MEMCG) \
    $(wildcard include/config/UPROBES) \
    $(wildcard include/config/BCACHE) \
    $(wildcard include/config/VMAP_STACK) \
    $(wildcard include/config/LIVEPATCH) \
    $(wildcard include/config/SECURITY) \
    $(wildcard include/config/BPF_SYSCALL) \
    $(wildcard include/config/GCC_PLUGIN_STACKLEAK) \
    $(wildcard include/config/X86_MCE) \
    $(wildcard include/config/KRETPROBES) \
    $(wildcard include/config/RETHOOK) \
    $(wildcard include/config/ARCH_HAS_PARANOID_L1D_FLUSH) \
    $(wildcard include/config/RV) \
    $(wildcard include/config/USER_EVENTS) \
    $(wildcard include/config/PREEMPTION) \
    $(wildcard include/config/MEM_ALLOC_PROFILING_DEBUG) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/sched.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/current.h \
    $(wildcard include/config/USE_X86_SEG_SUPPORT) \
  /usr/src/linux/linux-6.16.1/include/linux/cache.h \
    $(wildcard include/config/ARCH_HAS_CACHE_LINE_SIZE) \
  /usr/src/linux/linux-6.16.1/include/vdso/cache.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cache.h \
    $(wildcard include/config/X86_L1_CACHE_SHIFT) \
    $(wildcard include/config/X86_INTERNODE_CACHE_SHIFT) \
    $(wildcard include/config/X86_VSMP) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/percpu.h \
    $(wildcard include/config/CC_HAS_NAMED_AS) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/percpu.h \
    $(wildcard include/config/HAVE_SETUP_PER_CPU_AREA) \
  /usr/src/linux/linux-6.16.1/include/linux/threads.h \
    $(wildcard include/config/BASE_SMALL) \
  /usr/src/linux/linux-6.16.1/include/linux/percpu-defs.h \
    $(wildcard include/config/DEBUG_FORCE_WEAK_PER_CPU) \
    $(wildcard include/config/AMD_MEM_ENCRYPT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/processor.h \
    $(wildcard include/config/X86_VMX_FEATURE_NAMES) \
    $(wildcard include/config/X86_IOPL_IOPERM) \
    $(wildcard include/config/VM86) \
    $(wildcard include/config/X86_USER_SHADOW_STACK) \
    $(wildcard include/config/X86_DEBUG_FPU) \
    $(wildcard include/config/PARAVIRT_XXL) \
    $(wildcard include/config/CPU_SUP_AMD) \
    $(wildcard include/config/XEN) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/processor-flags.h \
    $(wildcard include/config/MITIGATION_PAGE_TABLE_ISOLATION) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/processor-flags.h \
  /usr/src/linux/linux-6.16.1/include/linux/mem_encrypt.h \
    $(wildcard include/config/ARCH_HAS_MEM_ENCRYPT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/mem_encrypt.h \
    $(wildcard include/config/X86_MEM_ENCRYPT) \
  /usr/src/linux/linux-6.16.1/include/linux/cc_platform.h \
    $(wildcard include/config/ARCH_HAS_CC_PLATFORM) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/math_emu.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/ptrace.h \
    $(wildcard include/config/PARAVIRT) \
    $(wildcard include/config/IA32_EMULATION) \
    $(wildcard include/config/X86_DEBUGCTLMSR) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/segment.h \
    $(wildcard include/config/XEN_PV) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/page_types.h \
    $(wildcard include/config/PHYSICAL_START) \
    $(wildcard include/config/PHYSICAL_ALIGN) \
    $(wildcard include/config/DYNAMIC_PHYSICAL_MASK) \
  /usr/src/linux/linux-6.16.1/include/vdso/page.h \
    $(wildcard include/config/PAGE_SHIFT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/page_64_types.h \
    $(wildcard include/config/KASAN) \
    $(wildcard include/config/RANDOMIZE_BASE) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/kaslr.h \
    $(wildcard include/config/RANDOMIZE_MEMORY) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/ptrace.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/ptrace-abi.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/paravirt_types.h \
    $(wildcard include/config/ZERO_CALL_USED_REGS) \
    $(wildcard include/config/PARAVIRT_DEBUG) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/desc_defs.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable_types.h \
    $(wildcard include/config/X86_INTEL_MEMORY_PROTECTION_KEYS) \
    $(wildcard include/config/X86_PAE) \
    $(wildcard include/config/MEM_SOFT_DIRTY) \
    $(wildcard include/config/HAVE_ARCH_USERFAULTFD_WP) \
    $(wildcard include/config/PGTABLE_LEVELS) \
    $(wildcard include/config/PROC_FS) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable_64_types.h \
    $(wildcard include/config/DEBUG_KMAP_LOCAL_FORCE_MAP) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/sparsemem.h \
    $(wildcard include/config/SPARSEMEM) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/nospec-branch.h \
    $(wildcard include/config/CALL_THUNKS_DEBUG) \
    $(wildcard include/config/MITIGATION_CALL_DEPTH_TRACKING) \
    $(wildcard include/config/MITIGATION_IBPB_ENTRY) \
  /usr/src/linux/linux-6.16.1/include/linux/static_key.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/msr-index.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/unwind_hints.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/orc_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/asm-offsets.h \
  /usr/src/linux/linux-6.16.1/include/generated/asm-offsets.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/GEN-for-each-reg.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/proto.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/ldt.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/sigcontext.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpuid/api.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpuid/types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/string.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/string_64.h \
    $(wildcard include/config/ARCH_HAS_UACCESS_FLUSHCACHE) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/paravirt.h \
    $(wildcard include/config/PARAVIRT_SPINLOCKS) \
    $(wildcard include/config/DEBUG_ENTRY) \
  /usr/src/linux/linux-6.16.1/include/linux/bug.h \
    $(wildcard include/config/BUG_ON_DATA_CORRUPTION) \
  /usr/src/linux/linux-6.16.1/include/linux/cpumask.h \
    $(wildcard include/config/FORCE_NR_CPUS) \
    $(wildcard include/config/HOTPLUG_CPU) \
    $(wildcard include/config/DEBUG_PER_CPU_MAPS) \
    $(wildcard include/config/CPUMASK_OFFSTACK) \
  /usr/src/linux/linux-6.16.1/include/linux/bitmap.h \
  /usr/src/linux/linux-6.16.1/include/linux/errno.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/errno.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/errno.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/errno.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/errno-base.h \
  /usr/src/linux/linux-6.16.1/include/linux/find.h \
  /usr/src/linux/linux-6.16.1/include/linux/string.h \
    $(wildcard include/config/BINARY_PRINTF) \
    $(wildcard include/config/FORTIFY_SOURCE) \
  /usr/src/linux/linux-6.16.1/include/linux/err.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/string.h \
  /usr/src/linux/linux-6.16.1/include/linux/fortify-string.h \
    $(wildcard include/config/CC_HAS_KASAN_MEMINTRINSIC_PREFIX) \
    $(wildcard include/config/GENERIC_ENTRY) \
  /usr/src/linux/linux-6.16.1/include/linux/bitfield.h \
  /usr/src/linux/linux-6.16.1/include/linux/bitmap-str.h \
  /usr/src/linux/linux-6.16.1/include/linux/cpumask_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/atomic.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/atomic.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cmpxchg.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cmpxchg_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/atomic64_64.h \
  /usr/src/linux/linux-6.16.1/include/linux/atomic/atomic-arch-fallback.h \
    $(wildcard include/config/GENERIC_ATOMIC64) \
  /usr/src/linux/linux-6.16.1/include/linux/atomic/atomic-long.h \
  /usr/src/linux/linux-6.16.1/include/linux/atomic/atomic-instrumented.h \
  /usr/src/linux/linux-6.16.1/include/linux/gfp_types.h \
    $(wildcard include/config/KASAN_HW_TAGS) \
    $(wildcard include/config/SLAB_OBJ_EXT) \
  /usr/src/linux/linux-6.16.1/include/linux/numa.h \
    $(wildcard include/config/NUMA_KEEP_MEMINFO) \
    $(wildcard include/config/HAVE_ARCH_NODE_DEV_GROUP) \
  /usr/src/linux/linux-6.16.1/include/linux/nodemask.h \
    $(wildcard include/config/HIGHMEM) \
  /usr/src/linux/linux-6.16.1/include/linux/nodemask_types.h \
    $(wildcard include/config/NODES_SHIFT) \
  /usr/src/linux/linux-6.16.1/include/linux/random.h \
    $(wildcard include/config/VMGENID) \
  /usr/src/linux/linux-6.16.1/include/linux/list.h \
    $(wildcard include/config/LIST_HARDENED) \
    $(wildcard include/config/DEBUG_LIST) \
  /usr/src/linux/linux-6.16.1/include/linux/poison.h \
    $(wildcard include/config/ILLEGAL_POINTER_VALUE) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/random.h \
  /usr/src/linux/linux-6.16.1/include/linux/irqnr.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/irqnr.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/frame.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/page.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/page_64.h \
    $(wildcard include/config/DEBUG_VIRTUAL) \
    $(wildcard include/config/X86_VSYSCALL_EMULATION) \
  /usr/src/linux/linux-6.16.1/include/linux/range.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/memory_model.h \
    $(wildcard include/config/FLATMEM) \
    $(wildcard include/config/SPARSEMEM_VMEMMAP) \
  /usr/src/linux/linux-6.16.1/include/linux/pfn.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/getorder.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/special_insns.h \
  /usr/src/linux/linux-6.16.1/include/linux/irqflags.h \
    $(wildcard include/config/IRQSOFF_TRACER) \
    $(wildcard include/config/PREEMPT_TRACER) \
    $(wildcard include/config/DEBUG_IRQFLAGS) \
    $(wildcard include/config/TRACE_IRQFLAGS_SUPPORT) \
  /usr/src/linux/linux-6.16.1/include/linux/irqflags_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/irqflags.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/fpu/types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/vmxfeatures.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/vdso/processor.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/shstk.h \
  /usr/src/linux/linux-6.16.1/include/linux/personality.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/personality.h \
  /usr/src/linux/linux-6.16.1/include/linux/math64.h \
    $(wildcard include/config/ARCH_SUPPORTS_INT128) \
  /usr/src/linux/linux-6.16.1/include/vdso/math64.h \
  /usr/src/linux/linux-6.16.1/include/linux/thread_info.h \
    $(wildcard include/config/ARCH_HAS_PREEMPT_LAZY) \
    $(wildcard include/config/HAVE_ARCH_WITHIN_STACK_FRAMES) \
    $(wildcard include/config/SH) \
  /usr/src/linux/linux-6.16.1/include/linux/restart_block.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/thread_info.h \
    $(wildcard include/config/X86_FRED) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpufeature.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/cpufeaturemasks.h \
  /usr/src/linux/linux-6.16.1/include/linux/preempt.h \
    $(wildcard include/config/TRACE_PREEMPT_TOGGLE) \
    $(wildcard include/config/PREEMPT_NONE) \
    $(wildcard include/config/PREEMPT_VOLUNTARY) \
    $(wildcard include/config/PREEMPT) \
    $(wildcard include/config/PREEMPT_LAZY) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/preempt.h \
  /usr/src/linux/linux-6.16.1/include/linux/smp_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/llist.h \
    $(wildcard include/config/ARCH_HAVE_NMI_SAFE_CMPXCHG) \
  /usr/src/linux/linux-6.16.1/include/linux/pid_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/sem_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/shm.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/shmparam.h \
  /usr/src/linux/linux-6.16.1/include/linux/kmsan_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/mutex_types.h \
    $(wildcard include/config/MUTEX_SPIN_ON_OWNER) \
  /usr/src/linux/linux-6.16.1/include/linux/osq_lock.h \
  /usr/src/linux/linux-6.16.1/include/linux/spinlock_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/rwlock_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/plist_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/hrtimer_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/timerqueue_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/rbtree_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/timer_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/seccomp_types.h \
    $(wildcard include/config/SECCOMP) \
  /usr/src/linux/linux-6.16.1/include/linux/refcount_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/resource.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/resource.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/time_types.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/resource.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/resource.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/resource.h \
  /usr/src/linux/linux-6.16.1/include/linux/latencytop.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/prio.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/types.h \
  /usr/src/linux/linux-6.16.1/include/linux/signal_types.h \
    $(wildcard include/config/OLD_SIGACTION) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/signal.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/signal.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/signal.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/signal-defs.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/siginfo.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/siginfo.h \
  /usr/src/linux/linux-6.16.1/include/linux/syscall_user_dispatch_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/mm_types_task.h \
    $(wildcard include/config/ARCH_WANT_BATCHED_UNMAP_TLB_FLUSH) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/tlbbatch.h \
  /usr/src/linux/linux-6.16.1/include/linux/netdevice_xmit.h \
    $(wildcard include/config/NET_EGRESS) \
    $(wildcard include/config/NET_ACT_MIRRED) \
    $(wildcard include/config/NF_DUP_NETDEV) \
  /usr/src/linux/linux-6.16.1/include/linux/task_io_accounting.h \
    $(wildcard include/config/TASK_IO_ACCOUNTING) \
  /usr/src/linux/linux-6.16.1/include/linux/posix-timers_types.h \
    $(wildcard include/config/POSIX_TIMERS) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/rseq.h \
  /usr/src/linux/linux-6.16.1/include/linux/seqlock_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/kcsan.h \
  /usr/src/linux/linux-6.16.1/include/linux/rv.h \
    $(wildcard include/config/RV_REACTORS) \
  /usr/src/linux/linux-6.16.1/include/linux/uidgid_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/tracepoint-defs.h \
    $(wildcard include/config/TRACEPOINTS) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/kmap_size.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/kmap_size.h \
    $(wildcard include/config/DEBUG_KMAP_LOCAL) \
  /usr/src/linux/linux-6.16.1/include/linux/sched/ext.h \
    $(wildcard include/config/EXT_GROUP_SCHED) \
  /usr/src/linux/linux-6.16.1/include/linux/spinlock.h \
  /usr/src/linux/linux-6.16.1/include/linux/bottom_half.h \
  /usr/src/linux/linux-6.16.1/include/linux/lockdep.h \
    $(wildcard include/config/DEBUG_LOCKING_API_SELFTESTS) \
  /usr/src/linux/linux-6.16.1/include/linux/smp.h \
    $(wildcard include/config/UP_LATE_INIT) \
    $(wildcard include/config/CSD_LOCK_WAIT_DEBUG) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/smp.h \
    $(wildcard include/config/DEBUG_NMI_SELFTEST) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpumask.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/mmiowb.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/mmiowb.h \
    $(wildcard include/config/MMIOWB) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/spinlock.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/qspinlock.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/qspinlock.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/qrwlock.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/qrwlock.h \
  /usr/src/linux/linux-6.16.1/include/linux/rwlock.h \
  /usr/src/linux/linux-6.16.1/include/linux/spinlock_api_smp.h \
    $(wildcard include/config/INLINE_SPIN_LOCK) \
    $(wildcard include/config/INLINE_SPIN_LOCK_BH) \
    $(wildcard include/config/INLINE_SPIN_LOCK_IRQ) \
    $(wildcard include/config/INLINE_SPIN_LOCK_IRQSAVE) \
    $(wildcard include/config/INLINE_SPIN_TRYLOCK) \
    $(wildcard include/config/INLINE_SPIN_TRYLOCK_BH) \
    $(wildcard include/config/UNINLINE_SPIN_UNLOCK) \
    $(wildcard include/config/INLINE_SPIN_UNLOCK_BH) \
    $(wildcard include/config/INLINE_SPIN_UNLOCK_IRQ) \
    $(wildcard include/config/INLINE_SPIN_UNLOCK_IRQRESTORE) \
    $(wildcard include/config/GENERIC_LOCKBREAK) \
  /usr/src/linux/linux-6.16.1/include/linux/rwlock_api_smp.h \
    $(wildcard include/config/INLINE_READ_LOCK) \
    $(wildcard include/config/INLINE_WRITE_LOCK) \
    $(wildcard include/config/INLINE_READ_LOCK_BH) \
    $(wildcard include/config/INLINE_WRITE_LOCK_BH) \
    $(wildcard include/config/INLINE_READ_LOCK_IRQ) \
    $(wildcard include/config/INLINE_WRITE_LOCK_IRQ) \
    $(wildcard include/config/INLINE_READ_LOCK_IRQSAVE) \
    $(wildcard include/config/INLINE_WRITE_LOCK_IRQSAVE) \
    $(wildcard include/config/INLINE_READ_TRYLOCK) \
    $(wildcard include/config/INLINE_WRITE_TRYLOCK) \
    $(wildcard include/config/INLINE_READ_UNLOCK) \
    $(wildcard include/config/INLINE_WRITE_UNLOCK) \
    $(wildcard include/config/INLINE_READ_UNLOCK_BH) \
    $(wildcard include/config/INLINE_WRITE_UNLOCK_BH) \
    $(wildcard include/config/INLINE_READ_UNLOCK_IRQ) \
    $(wildcard include/config/INLINE_WRITE_UNLOCK_IRQ) \
    $(wildcard include/config/INLINE_READ_UNLOCK_IRQRESTORE) \
    $(wildcard include/config/INLINE_WRITE_UNLOCK_IRQRESTORE) \
  /usr/src/linux/linux-6.16.1/include/linux/ucopysize.h \
    $(wildcard include/config/HARDENED_USERCOPY_DEFAULT_ON) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/uaccess.h \
    $(wildcard include/config/CC_HAS_ASM_GOTO_OUTPUT) \
    $(wildcard include/config/CC_HAS_ASM_GOTO_TIED_OUTPUT) \
    $(wildcard include/config/ARCH_HAS_COPY_MC) \
    $(wildcard include/config/X86_INTEL_USERCOPY) \
  /usr/src/linux/linux-6.16.1/include/linux/mm_types.h \
    $(wildcard include/config/HAVE_ALIGNED_STRUCT_PAGE) \
    $(wildcard include/config/HUGETLB_PMD_PAGE_TABLE_SHARING) \
    $(wildcard include/config/SLAB_FREELIST_HARDENED) \
    $(wildcard include/config/USERFAULTFD) \
    $(wildcard include/config/ANON_VMA_NAME) \
    $(wildcard include/config/PER_VMA_LOCK) \
    $(wildcard include/config/SWAP) \
    $(wildcard include/config/HAVE_ARCH_COMPAT_MMAP_BASES) \
    $(wildcard include/config/MEMBARRIER) \
    $(wildcard include/config/FUTEX_PRIVATE_HASH) \
    $(wildcard include/config/AIO) \
    $(wildcard include/config/MMU_NOTIFIER) \
    $(wildcard include/config/TRANSPARENT_HUGEPAGE) \
    $(wildcard include/config/SPLIT_PMD_PTLOCKS) \
    $(wildcard include/config/HUGETLB_PAGE) \
    $(wildcard include/config/IOMMU_MM_DATA) \
    $(wildcard include/config/KSM) \
    $(wildcard include/config/LRU_GEN_WALKS_MMU) \
    $(wildcard include/config/MM_ID) \
    $(wildcard include/config/CORE_DUMP_DEFAULT_ELF_HEADERS) \
  /usr/src/linux/linux-6.16.1/include/linux/auxvec.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/auxvec.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/auxvec.h \
  /usr/src/linux/linux-6.16.1/include/linux/kref.h \
  /usr/src/linux/linux-6.16.1/include/linux/refcount.h \
  /usr/src/linux/linux-6.16.1/include/linux/rbtree.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcupdate.h \
    $(wildcard include/config/TINY_RCU) \
    $(wildcard include/config/RCU_STRICT_GRACE_PERIOD) \
    $(wildcard include/config/RCU_LAZY) \
    $(wildcard include/config/RCU_STALL_COMMON) \
    $(wildcard include/config/KVM_XFER_TO_GUEST_WORK) \
    $(wildcard include/config/RCU_NOCB_CPU) \
    $(wildcard include/config/TASKS_RCU_GENERIC) \
    $(wildcard include/config/TASKS_RUDE_RCU) \
    $(wildcard include/config/TREE_RCU) \
    $(wildcard include/config/DEBUG_OBJECTS_RCU_HEAD) \
    $(wildcard include/config/PROVE_RCU) \
    $(wildcard include/config/ARCH_WEAK_RELEASE_ACQUIRE) \
  /usr/src/linux/linux-6.16.1/include/linux/context_tracking_irq.h \
    $(wildcard include/config/CONTEXT_TRACKING_IDLE) \
  /usr/src/linux/linux-6.16.1/include/linux/rcutree.h \
  /usr/src/linux/linux-6.16.1/include/linux/maple_tree.h \
    $(wildcard include/config/MAPLE_RCU_DISABLED) \
    $(wildcard include/config/DEBUG_MAPLE_TREE) \
  /usr/src/linux/linux-6.16.1/include/linux/rwsem.h \
    $(wildcard include/config/RWSEM_SPIN_ON_OWNER) \
    $(wildcard include/config/DEBUG_RWSEMS) \
  /usr/src/linux/linux-6.16.1/include/linux/completion.h \
  /usr/src/linux/linux-6.16.1/include/linux/swait.h \
  /usr/src/linux/linux-6.16.1/include/linux/wait.h \
  /usr/src/linux/linux-6.16.1/include/linux/uprobes.h \
  /usr/src/linux/linux-6.16.1/include/linux/timer.h \
    $(wildcard include/config/DEBUG_OBJECTS_TIMERS) \
  /usr/src/linux/linux-6.16.1/include/linux/ktime.h \
  /usr/src/linux/linux-6.16.1/include/linux/jiffies.h \
  /usr/src/linux/linux-6.16.1/include/linux/time.h \
  /usr/src/linux/linux-6.16.1/include/linux/time64.h \
  /usr/src/linux/linux-6.16.1/include/vdso/time64.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/time.h \
  /usr/src/linux/linux-6.16.1/include/linux/time32.h \
  /usr/src/linux/linux-6.16.1/include/linux/timex.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/timex.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/timex.h \
    $(wildcard include/config/X86_TSC) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/tsc.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/msr.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/msr.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/shared/msr.h \
  /usr/src/linux/linux-6.16.1/include/linux/percpu.h \
    $(wildcard include/config/MODULES) \
    $(wildcard include/config/RANDOM_KMALLOC_CACHES) \
    $(wildcard include/config/PAGE_SIZE_4KB) \
    $(wildcard include/config/NEED_PER_CPU_PAGE_FIRST_CHUNK) \
  /usr/src/linux/linux-6.16.1/include/linux/alloc_tag.h \
    $(wildcard include/config/MEM_ALLOC_PROFILING_ENABLED_BY_DEFAULT) \
  /usr/src/linux/linux-6.16.1/include/linux/codetag.h \
    $(wildcard include/config/CODE_TAGGING) \
  /usr/src/linux/linux-6.16.1/include/linux/mmdebug.h \
    $(wildcard include/config/DEBUG_VM) \
    $(wildcard include/config/DEBUG_VM_IRQSOFF) \
    $(wildcard include/config/DEBUG_VM_PGFLAGS) \
  /usr/src/linux/linux-6.16.1/include/vdso/time32.h \
  /usr/src/linux/linux-6.16.1/include/vdso/time.h \
  /usr/src/linux/linux-6.16.1/include/vdso/jiffies.h \
  /usr/src/linux/linux-6.16.1/include/generated/timeconst.h \
  /usr/src/linux/linux-6.16.1/include/vdso/ktime.h \
  /usr/src/linux/linux-6.16.1/include/linux/timekeeping.h \
    $(wildcard include/config/GENERIC_CMOS_UPDATE) \
  /usr/src/linux/linux-6.16.1/include/linux/clocksource_ids.h \
  /usr/src/linux/linux-6.16.1/include/linux/debugobjects.h \
    $(wildcard include/config/DEBUG_OBJECTS) \
    $(wildcard include/config/DEBUG_OBJECTS_FREE) \
  /usr/src/linux/linux-6.16.1/include/linux/seqlock.h \
  /usr/src/linux/linux-6.16.1/include/linux/mutex.h \
  /usr/src/linux/linux-6.16.1/include/linux/debug_locks.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/uprobes.h \
  /usr/src/linux/linux-6.16.1/include/linux/notifier.h \
    $(wildcard include/config/TREE_SRCU) \
  /usr/src/linux/linux-6.16.1/include/linux/srcu.h \
    $(wildcard include/config/TINY_SRCU) \
    $(wildcard include/config/NEED_SRCU_NMI_SAFE) \
  /usr/src/linux/linux-6.16.1/include/linux/workqueue.h \
    $(wildcard include/config/DEBUG_OBJECTS_WORK) \
    $(wildcard include/config/FREEZER) \
    $(wildcard include/config/SYSFS) \
    $(wildcard include/config/WQ_WATCHDOG) \
  /usr/src/linux/linux-6.16.1/include/linux/workqueue_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcu_segcblist.h \
  /usr/src/linux/linux-6.16.1/include/linux/srcutree.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcu_node_tree.h \
    $(wildcard include/config/RCU_FANOUT) \
    $(wildcard include/config/RCU_FANOUT_LEAF) \
  /usr/src/linux/linux-6.16.1/include/linux/page-flags-layout.h \
  /usr/src/linux/linux-6.16.1/include/generated/bounds.h \
  /usr/src/linux/linux-6.16.1/include/linux/percpu_counter.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/mmu.h \
    $(wildcard include/config/MODIFY_LDT_SYSCALL) \
    $(wildcard include/config/ADDRESS_MASKING) \
    $(wildcard include/config/BROADCAST_TLB_FLUSH) \
  /usr/src/linux/linux-6.16.1/include/linux/mmap_lock.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/mm.h \
    $(wildcard include/config/MMU_LAZY_TLB_REFCOUNT) \
    $(wildcard include/config/ARCH_HAS_MEMBARRIER_CALLBACKS) \
    $(wildcard include/config/ARCH_HAS_SYNC_CORE_BEFORE_USERMODE) \
  /usr/src/linux/linux-6.16.1/include/linux/gfp.h \
    $(wildcard include/config/ZONE_DMA) \
    $(wildcard include/config/ZONE_DMA32) \
    $(wildcard include/config/ZONE_DEVICE) \
    $(wildcard include/config/CONTIG_ALLOC) \
  /usr/src/linux/linux-6.16.1/include/linux/mmzone.h \
    $(wildcard include/config/ARCH_FORCE_MAX_ORDER) \
    $(wildcard include/config/PAGE_BLOCK_ORDER) \
    $(wildcard include/config/CMA) \
    $(wildcard include/config/MEMORY_ISOLATION) \
    $(wildcard include/config/ZSMALLOC) \
    $(wildcard include/config/UNACCEPTED_MEMORY) \
    $(wildcard include/config/IOMMU_SUPPORT) \
    $(wildcard include/config/LRU_GEN_STATS) \
    $(wildcard include/config/MEMORY_FAILURE) \
    $(wildcard include/config/PAGE_EXTENSION) \
    $(wildcard include/config/DEFERRED_STRUCT_PAGE_INIT) \
    $(wildcard include/config/HAVE_MEMORYLESS_NODES) \
    $(wildcard include/config/SPARSEMEM_EXTREME) \
    $(wildcard include/config/SPARSEMEM_VMEMMAP_PREINIT) \
    $(wildcard include/config/HAVE_ARCH_PFN_VALID) \
  /usr/src/linux/linux-6.16.1/include/linux/list_nulls.h \
  /usr/src/linux/linux-6.16.1/include/linux/pageblock-flags.h \
    $(wildcard include/config/HUGETLB_PAGE_SIZE_VARIABLE) \
  /usr/src/linux/linux-6.16.1/include/linux/page-flags.h \
    $(wildcard include/config/PAGE_IDLE_FLAG) \
    $(wildcard include/config/ARCH_USES_PG_ARCH_2) \
    $(wildcard include/config/ARCH_USES_PG_ARCH_3) \
    $(wildcard include/config/HUGETLB_PAGE_OPTIMIZE_VMEMMAP) \
  /usr/src/linux/linux-6.16.1/include/linux/local_lock.h \
  /usr/src/linux/linux-6.16.1/include/linux/local_lock_internal.h \
  /usr/src/linux/linux-6.16.1/include/linux/zswap.h \
    $(wildcard include/config/ZSWAP) \
  /usr/src/linux/linux-6.16.1/include/linux/memory_hotplug.h \
    $(wildcard include/config/ARCH_HAS_ADD_PAGES) \
    $(wildcard include/config/MEMORY_HOTREMOVE) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/mmzone.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/mmzone.h \
  /usr/src/linux/linux-6.16.1/include/linux/topology.h \
    $(wildcard include/config/USE_PERCPU_NUMA_NODE_ID) \
    $(wildcard include/config/SCHED_SMT) \
    $(wildcard include/config/GENERIC_ARCH_TOPOLOGY) \
  /usr/src/linux/linux-6.16.1/include/linux/arch_topology.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/topology.h \
    $(wildcard include/config/X86_LOCAL_APIC) \
    $(wildcard include/config/SCHED_MC_PRIO) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/mpspec.h \
    $(wildcard include/config/EISA) \
    $(wildcard include/config/X86_MPPARSE) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/mpspec_def.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/x86_init.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/apicdef.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/topology.h \
  /usr/src/linux/linux-6.16.1/include/linux/cpu_smt.h \
    $(wildcard include/config/HOTPLUG_SMT) \
  /usr/src/linux/linux-6.16.1/include/linux/sync_core.h \
    $(wildcard include/config/ARCH_HAS_PREPARE_SYNC_CORE_CMD) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/sync_core.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/coredump.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/smap.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/extable.h \
    $(wildcard include/config/BPF_JIT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/tlbflush.h \
  /usr/src/linux/linux-6.16.1/include/linux/mmu_notifier.h \
  /usr/src/linux/linux-6.16.1/include/linux/interval_tree.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/invpcid.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pti.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable.h \
    $(wildcard include/config/DEBUG_WX) \
    $(wildcard include/config/HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD) \
    $(wildcard include/config/ARCH_HAS_PTE_DEVMAP) \
    $(wildcard include/config/ARCH_SUPPORTS_PMD_PFNMAP) \
    $(wildcard include/config/ARCH_SUPPORTS_PUD_PFNMAP) \
    $(wildcard include/config/HAVE_ARCH_SOFT_DIRTY) \
    $(wildcard include/config/ARCH_ENABLE_THP_MIGRATION) \
    $(wildcard include/config/PAGE_TABLE_CHECK) \
    $(wildcard include/config/X86_SGX) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pkru.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/fpu/api.h \
    $(wildcard include/config/MATH_EMULATION) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/coco.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/pgtable_uffd.h \
  /usr/src/linux/linux-6.16.1/include/linux/page_table_check.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/fixmap.h \
    $(wildcard include/config/PROVIDE_OHCI1394_DMA_INIT) \
    $(wildcard include/config/X86_IO_APIC) \
    $(wildcard include/config/PCI_MMCONFIG) \
    $(wildcard include/config/ACPI_APEI_GHES) \
    $(wildcard include/config/INTEL_TXT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/vsyscall.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/fixmap.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable-invert.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/uaccess_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/runtime-const.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/access_ok.h \
    $(wildcard include/config/ALTERNATE_USER_ADDRESS_SPACE) \
  xdma_mod.h \
  /usr/src/linux/linux-6.16.1/include/linux/module.h \
    $(wildcard include/config/MODULES_TREE_LOOKUP) \
    $(wildcard include/config/STACKTRACE_BUILD_ID) \
    $(wildcard include/config/ARCH_USES_CFI_TRAPS) \
    $(wildcard include/config/MODULE_SIG) \
    $(wildcard include/config/KALLSYMS) \
    $(wildcard include/config/BPF_EVENTS) \
    $(wildcard include/config/DEBUG_INFO_BTF_MODULES) \
    $(wildcard include/config/EVENT_TRACING) \
    $(wildcard include/config/MODULE_UNLOAD) \
    $(wildcard include/config/CONSTRUCTORS) \
    $(wildcard include/config/FUNCTION_ERROR_INJECTION) \
  /usr/src/linux/linux-6.16.1/include/linux/stat.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/stat.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/stat.h \
  /usr/src/linux/linux-6.16.1/include/linux/uidgid.h \
    $(wildcard include/config/MULTIUSER) \
    $(wildcard include/config/USER_NS) \
  /usr/src/linux/linux-6.16.1/include/linux/highuid.h \
  /usr/src/linux/linux-6.16.1/include/linux/buildid.h \
    $(wildcard include/config/VMCORE_INFO) \
  /usr/src/linux/linux-6.16.1/include/linux/kmod.h \
  /usr/src/linux/linux-6.16.1/include/linux/umh.h \
  /usr/src/linux/linux-6.16.1/include/linux/sysctl.h \
    $(wildcard include/config/SYSCTL) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/sysctl.h \
  /usr/src/linux/linux-6.16.1/include/linux/elf.h \
    $(wildcard include/config/ARCH_HAVE_EXTRA_ELF_NOTES) \
    $(wildcard include/config/ARCH_USE_GNU_PROPERTY) \
    $(wildcard include/config/ARCH_HAVE_ELF_PROT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/elf.h \
    $(wildcard include/config/X86_X32_ABI) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/ia32.h \
  /usr/src/linux/linux-6.16.1/include/linux/compat.h \
    $(wildcard include/config/ARCH_HAS_SYSCALL_WRAPPER) \
    $(wildcard include/config/COMPAT_OLD_SIGACTION) \
    $(wildcard include/config/ODD_RT_SIGACTION) \
  /usr/src/linux/linux-6.16.1/include/linux/sem.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/sem.h \
  /usr/src/linux/linux-6.16.1/include/linux/ipc.h \
  /usr/src/linux/linux-6.16.1/include/linux/rhashtable-types.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/ipc.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/ipcbuf.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/ipcbuf.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/sembuf.h \
  /usr/src/linux/linux-6.16.1/include/linux/socket.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/socket.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/socket.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/sockios.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/sockios.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/sockios.h \
  /usr/src/linux/linux-6.16.1/include/linux/uio.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/uio.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/socket.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/if.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/libc-compat.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/hdlc/ioctl.h \
  /usr/src/linux/linux-6.16.1/include/linux/fs.h \
    $(wildcard include/config/FANOTIFY_ACCESS_PERMISSIONS) \
    $(wildcard include/config/READ_ONLY_THP_FOR_FS) \
    $(wildcard include/config/FS_POSIX_ACL) \
    $(wildcard include/config/CGROUP_WRITEBACK) \
    $(wildcard include/config/IMA) \
    $(wildcard include/config/FILE_LOCKING) \
    $(wildcard include/config/FSNOTIFY) \
    $(wildcard include/config/FS_ENCRYPTION) \
    $(wildcard include/config/FS_VERITY) \
    $(wildcard include/config/EPOLL) \
    $(wildcard include/config/UNICODE) \
    $(wildcard include/config/QUOTA) \
    $(wildcard include/config/FS_DAX) \
    $(wildcard include/config/BLOCK) \
  /usr/src/linux/linux-6.16.1/include/linux/vfsdebug.h \
    $(wildcard include/config/DEBUG_VFS) \
  /usr/src/linux/linux-6.16.1/include/linux/wait_bit.h \
  /usr/src/linux/linux-6.16.1/include/linux/kdev_t.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/kdev_t.h \
  /usr/src/linux/linux-6.16.1/include/linux/dcache.h \
  /usr/src/linux/linux-6.16.1/include/linux/rculist.h \
    $(wildcard include/config/PROVE_RCU_LIST) \
  /usr/src/linux/linux-6.16.1/include/linux/rculist_bl.h \
  /usr/src/linux/linux-6.16.1/include/linux/list_bl.h \
  /usr/src/linux/linux-6.16.1/include/linux/bit_spinlock.h \
  /usr/src/linux/linux-6.16.1/include/linux/lockref.h \
    $(wildcard include/config/ARCH_USE_CMPXCHG_LOCKREF) \
  /usr/src/linux/linux-6.16.1/include/linux/stringhash.h \
    $(wildcard include/config/DCACHE_WORD_ACCESS) \
  /usr/src/linux/linux-6.16.1/include/linux/hash.h \
    $(wildcard include/config/HAVE_ARCH_HASH) \
  /usr/src/linux/linux-6.16.1/include/linux/path.h \
  /usr/src/linux/linux-6.16.1/include/linux/list_lru.h \
  /usr/src/linux/linux-6.16.1/include/linux/shrinker.h \
    $(wildcard include/config/SHRINKER_DEBUG) \
  /usr/src/linux/linux-6.16.1/include/linux/xarray.h \
    $(wildcard include/config/XARRAY_MULTI) \
  /usr/src/linux/linux-6.16.1/include/linux/radix-tree.h \
  /usr/src/linux/linux-6.16.1/include/linux/pid.h \
  /usr/src/linux/linux-6.16.1/include/linux/capability.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/capability.h \
  /usr/src/linux/linux-6.16.1/include/linux/semaphore.h \
  /usr/src/linux/linux-6.16.1/include/linux/fcntl.h \
    $(wildcard include/config/ARCH_32BIT_OFF_T) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/fcntl.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/fcntl.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/fcntl.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/openat2.h \
  /usr/src/linux/linux-6.16.1/include/linux/migrate_mode.h \
  /usr/src/linux/linux-6.16.1/include/linux/percpu-rwsem.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcuwait.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/signal.h \
    $(wildcard include/config/SCHED_AUTOGROUP) \
    $(wildcard include/config/BSD_PROCESS_ACCT) \
    $(wildcard include/config/TASKSTATS) \
    $(wildcard include/config/STACK_GROWSUP) \
  /usr/src/linux/linux-6.16.1/include/linux/signal.h \
    $(wildcard include/config/DYNAMIC_SIGFRAME) \
  /usr/src/linux/linux-6.16.1/include/linux/sched/jobctl.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/task.h \
    $(wildcard include/config/HAVE_EXIT_THREAD) \
    $(wildcard include/config/ARCH_WANTS_DYNAMIC_TASK_STRUCT) \
    $(wildcard include/config/HAVE_ARCH_THREAD_STRUCT_WHITELIST) \
  /usr/src/linux/linux-6.16.1/include/linux/cred.h \
  /usr/src/linux/linux-6.16.1/include/linux/key.h \
    $(wildcard include/config/KEY_NOTIFICATIONS) \
    $(wildcard include/config/NET) \
  /usr/src/linux/linux-6.16.1/include/linux/assoc_array.h \
    $(wildcard include/config/ASSOCIATIVE_ARRAY) \
  /usr/src/linux/linux-6.16.1/include/linux/sched/user.h \
    $(wildcard include/config/VFIO_PCI_ZDEV_KVM) \
    $(wildcard include/config/IOMMUFD) \
    $(wildcard include/config/WATCH_QUEUE) \
  /usr/src/linux/linux-6.16.1/include/linux/ratelimit.h \
  /usr/src/linux/linux-6.16.1/include/linux/posix-timers.h \
  /usr/src/linux/linux-6.16.1/include/linux/alarmtimer.h \
    $(wildcard include/config/RTC_CLASS) \
  /usr/src/linux/linux-6.16.1/include/linux/hrtimer.h \
    $(wildcard include/config/HIGH_RES_TIMERS) \
    $(wildcard include/config/TIME_LOW_RES) \
    $(wildcard include/config/TIMERFD) \
  /usr/src/linux/linux-6.16.1/include/linux/hrtimer_defs.h \
  /usr/src/linux/linux-6.16.1/include/linux/timerqueue.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcuref.h \
  /usr/src/linux/linux-6.16.1/include/linux/rcu_sync.h \
  /usr/src/linux/linux-6.16.1/include/linux/delayed_call.h \
  /usr/src/linux/linux-6.16.1/include/linux/uuid.h \
  /usr/src/linux/linux-6.16.1/include/linux/errseq.h \
  /usr/src/linux/linux-6.16.1/include/linux/ioprio.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/rt.h \
  /usr/src/linux/linux-6.16.1/include/linux/iocontext.h \
    $(wildcard include/config/BLK_ICQ) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/ioprio.h \
  /usr/src/linux/linux-6.16.1/include/linux/fs_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/mount.h \
  /usr/src/linux/linux-6.16.1/include/linux/mnt_idmapping.h \
  /usr/src/linux/linux-6.16.1/include/linux/slab.h \
    $(wildcard include/config/FAILSLAB) \
    $(wildcard include/config/KFENCE) \
    $(wildcard include/config/SLUB_TINY) \
    $(wildcard include/config/SLUB_DEBUG) \
    $(wildcard include/config/SLAB_BUCKETS) \
    $(wildcard include/config/KVFREE_RCU_BATCHED) \
  /usr/src/linux/linux-6.16.1/include/linux/percpu-refcount.h \
  /usr/src/linux/linux-6.16.1/include/linux/kasan.h \
    $(wildcard include/config/KASAN_STACK) \
    $(wildcard include/config/KASAN_VMALLOC) \
  /usr/src/linux/linux-6.16.1/include/linux/kasan-enabled.h \
  /usr/src/linux/linux-6.16.1/include/linux/kasan-tags.h \
  /usr/src/linux/linux-6.16.1/include/linux/rw_hint.h \
  /usr/src/linux/linux-6.16.1/include/linux/file_ref.h \
  /usr/src/linux/linux-6.16.1/include/linux/unicode.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/fs.h \
  /usr/src/linux/linux-6.16.1/include/linux/quota.h \
    $(wildcard include/config/QUOTA_NETLINK_INTERFACE) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/dqblk_xfs.h \
  /usr/src/linux/linux-6.16.1/include/linux/dqblk_v1.h \
  /usr/src/linux/linux-6.16.1/include/linux/dqblk_v2.h \
  /usr/src/linux/linux-6.16.1/include/linux/dqblk_qtree.h \
  /usr/src/linux/linux-6.16.1/include/linux/projid.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/quota.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/aio_abi.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/unistd.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/unistd.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/unistd.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/unistd_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/unistd_64_x32.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/unistd_32_ia32.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/compat.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/task_stack.h \
    $(wildcard include/config/DEBUG_STACK_USAGE) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/magic.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/user32.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/compat.h \
    $(wildcard include/config/COMPAT_FOR_U64_ALIGNMENT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/syscall_wrapper.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/user.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/user_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/fsgsbase.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/vdso.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/elf.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/elf-em.h \
  /usr/src/linux/linux-6.16.1/include/linux/kobject.h \
    $(wildcard include/config/UEVENT_HELPER) \
    $(wildcard include/config/DEBUG_KOBJECT_RELEASE) \
  /usr/src/linux/linux-6.16.1/include/linux/sysfs.h \
  /usr/src/linux/linux-6.16.1/include/linux/kernfs.h \
    $(wildcard include/config/KERNFS) \
  /usr/src/linux/linux-6.16.1/include/linux/idr.h \
  /usr/src/linux/linux-6.16.1/include/linux/kobject_ns.h \
  /usr/src/linux/linux-6.16.1/include/linux/moduleparam.h \
    $(wildcard include/config/ALPHA) \
    $(wildcard include/config/PPC64) \
  /usr/src/linux/linux-6.16.1/include/linux/rbtree_latch.h \
  /usr/src/linux/linux-6.16.1/include/linux/error-injection.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/error-injection.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/module.h \
    $(wildcard include/config/UNWINDER_ORC) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/module.h \
    $(wildcard include/config/HAVE_MOD_ARCH_SPECIFIC) \
  /usr/src/linux/linux-6.16.1/include/linux/cdev.h \
  /usr/src/linux/linux-6.16.1/include/linux/device.h \
    $(wildcard include/config/GENERIC_MSI_IRQ) \
    $(wildcard include/config/ENERGY_MODEL) \
    $(wildcard include/config/PINCTRL) \
    $(wildcard include/config/ARCH_HAS_DMA_OPS) \
    $(wildcard include/config/DMA_DECLARE_COHERENT) \
    $(wildcard include/config/DMA_CMA) \
    $(wildcard include/config/SWIOTLB) \
    $(wildcard include/config/SWIOTLB_DYNAMIC) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_DEVICE) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_CPU) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_CPU_ALL) \
    $(wildcard include/config/DMA_OPS_BYPASS) \
    $(wildcard include/config/DMA_NEED_SYNC) \
    $(wildcard include/config/IOMMU_DMA) \
    $(wildcard include/config/PM_SLEEP) \
    $(wildcard include/config/OF) \
    $(wildcard include/config/DEVTMPFS) \
  /usr/src/linux/linux-6.16.1/include/linux/dev_printk.h \
  /usr/src/linux/linux-6.16.1/include/linux/energy_model.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/cpufreq.h \
    $(wildcard include/config/CPU_FREQ) \
  /usr/src/linux/linux-6.16.1/include/linux/sched/topology.h \
    $(wildcard include/config/SCHED_CLUSTER) \
    $(wildcard include/config/SCHED_MC) \
    $(wildcard include/config/CPU_FREQ_GOV_SCHEDUTIL) \
  /usr/src/linux/linux-6.16.1/include/linux/sched/idle.h \
  /usr/src/linux/linux-6.16.1/include/linux/sched/sd_flags.h \
  /usr/src/linux/linux-6.16.1/include/linux/ioport.h \
  /usr/src/linux/linux-6.16.1/include/linux/klist.h \
  /usr/src/linux/linux-6.16.1/include/linux/pm.h \
    $(wildcard include/config/VT_CONSOLE_SLEEP) \
    $(wildcard include/config/CXL_SUSPEND) \
    $(wildcard include/config/PM) \
    $(wildcard include/config/PM_CLK) \
    $(wildcard include/config/PM_GENERIC_DOMAINS) \
  /usr/src/linux/linux-6.16.1/include/linux/device/bus.h \
    $(wildcard include/config/ACPI) \
  /usr/src/linux/linux-6.16.1/include/linux/device/class.h \
  /usr/src/linux/linux-6.16.1/include/linux/device/devres.h \
    $(wildcard include/config/HAS_IOMEM) \
  /usr/src/linux/linux-6.16.1/include/linux/device/driver.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/device.h \
  /usr/src/linux/linux-6.16.1/include/linux/pm_wakeup.h \
  /usr/src/linux/linux-6.16.1/include/linux/dma-mapping.h \
    $(wildcard include/config/DMA_API_DEBUG) \
    $(wildcard include/config/HAS_DMA) \
    $(wildcard include/config/NEED_DMA_MAP_STATE) \
  /usr/src/linux/linux-6.16.1/include/linux/dma-direction.h \
  /usr/src/linux/linux-6.16.1/include/linux/scatterlist.h \
    $(wildcard include/config/NEED_SG_DMA_LENGTH) \
    $(wildcard include/config/NEED_SG_DMA_FLAGS) \
    $(wildcard include/config/DEBUG_SG) \
    $(wildcard include/config/SGL_ALLOC) \
    $(wildcard include/config/ARCH_NO_SG_CHAIN) \
    $(wildcard include/config/SG_POOL) \
  /usr/src/linux/linux-6.16.1/include/linux/mm.h \
    $(wildcard include/config/HAVE_ARCH_MMAP_RND_BITS) \
    $(wildcard include/config/HAVE_ARCH_MMAP_RND_COMPAT_BITS) \
    $(wildcard include/config/ARCH_USES_HIGH_VMA_FLAGS) \
    $(wildcard include/config/ARCH_HAS_PKEYS) \
    $(wildcard include/config/ARCH_PKEY_BITS) \
    $(wildcard include/config/ARM64_GCS) \
    $(wildcard include/config/PARISC) \
    $(wildcard include/config/SPARC64) \
    $(wildcard include/config/ARM64_MTE) \
    $(wildcard include/config/HAVE_ARCH_USERFAULTFD_MINOR) \
    $(wildcard include/config/PPC32) \
    $(wildcard include/config/SHMEM) \
    $(wildcard include/config/MIGRATION) \
    $(wildcard include/config/ARCH_HAS_GIGANTIC_PAGE) \
    $(wildcard include/config/ARCH_HAS_PTE_SPECIAL) \
    $(wildcard include/config/SPLIT_PTE_PTLOCKS) \
    $(wildcard include/config/HIGHPTE) \
    $(wildcard include/config/DEBUG_VM_RB) \
    $(wildcard include/config/PAGE_POISONING) \
    $(wildcard include/config/INIT_ON_ALLOC_DEFAULT_ON) \
    $(wildcard include/config/INIT_ON_FREE_DEFAULT_ON) \
    $(wildcard include/config/DEBUG_PAGEALLOC) \
    $(wildcard include/config/ARCH_WANT_OPTIMIZE_DAX_VMEMMAP) \
    $(wildcard include/config/HUGETLBFS) \
    $(wildcard include/config/MAPPING_DIRTY_HELPERS) \
    $(wildcard include/config/MSEAL_SYSTEM_MAPPINGS) \
    $(wildcard include/config/PAGE_POOL) \
  /usr/src/linux/linux-6.16.1/include/linux/pgalloc_tag.h \
  /usr/src/linux/linux-6.16.1/include/linux/page_ext.h \
  /usr/src/linux/linux-6.16.1/include/linux/stacktrace.h \
    $(wildcard include/config/ARCH_STACKWALK) \
    $(wildcard include/config/STACKTRACE) \
    $(wildcard include/config/HAVE_RELIABLE_STACKTRACE) \
  /usr/src/linux/linux-6.16.1/include/linux/page_ref.h \
    $(wildcard include/config/DEBUG_PAGE_REF) \
  /usr/src/linux/linux-6.16.1/include/linux/sizes.h \
  /usr/src/linux/linux-6.16.1/include/linux/pgtable.h \
    $(wildcard include/config/ARCH_HAS_NONLEAF_PMD_YOUNG) \
    $(wildcard include/config/ARCH_HAS_HW_PTE_YOUNG) \
    $(wildcard include/config/GUP_GET_PXX_LOW_HIGH) \
    $(wildcard include/config/ARCH_WANT_PMD_MKWRITE) \
    $(wildcard include/config/HAVE_ARCH_HUGE_VMAP) \
    $(wildcard include/config/X86_ESPFIX64) \
  /usr/src/linux/linux-6.16.1/include/linux/memremap.h \
    $(wildcard include/config/DEVICE_PRIVATE) \
    $(wildcard include/config/PCI_P2PDMA) \
  /usr/src/linux/linux-6.16.1/include/linux/cacheinfo.h \
    $(wildcard include/config/ACPI_PPTT) \
    $(wildcard include/config/ARM) \
    $(wildcard include/config/ARCH_HAS_CPU_CACHE_ALIASING) \
  /usr/src/linux/linux-6.16.1/include/linux/cpuhplock.h \
  /usr/src/linux/linux-6.16.1/include/linux/huge_mm.h \
    $(wildcard include/config/PGTABLE_HAS_HUGE_LEAVES) \
  /usr/src/linux/linux-6.16.1/include/linux/vmstat.h \
    $(wildcard include/config/VM_EVENT_COUNTERS) \
    $(wildcard include/config/DEBUG_TLBFLUSH) \
    $(wildcard include/config/PER_VMA_LOCK_STATS) \
  /usr/src/linux/linux-6.16.1/include/linux/vm_event_item.h \
    $(wildcard include/config/MEMORY_BALLOON) \
    $(wildcard include/config/BALLOON_COMPACTION) \
    $(wildcard include/config/X86) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/io.h \
    $(wildcard include/config/MTRR) \
    $(wildcard include/config/X86_PAT) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/early_ioremap.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/early_ioremap.h \
    $(wildcard include/config/GENERIC_EARLY_IOREMAP) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/shared/io.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/io.h \
    $(wildcard include/config/GENERIC_IOMAP) \
    $(wildcard include/config/TRACE_MMIO_ACCESS) \
    $(wildcard include/config/HAS_IOPORT) \
    $(wildcard include/config/GENERIC_IOREMAP) \
    $(wildcard include/config/HAS_IOPORT_MAP) \
  /usr/src/linux/linux-6.16.1/include/asm-generic/iomap.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/pci_iomap.h \
    $(wildcard include/config/PCI) \
    $(wildcard include/config/NO_GENERIC_PCI_IOPORT_MAP) \
    $(wildcard include/config/GENERIC_PCI_IOMAP) \
  /usr/src/linux/linux-6.16.1/include/linux/logic_pio.h \
    $(wildcard include/config/INDIRECT_PIO) \
  /usr/src/linux/linux-6.16.1/include/linux/fwnode.h \
  /usr/src/linux/linux-6.16.1/include/linux/delay.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/delay.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/delay.h \
  /usr/src/linux/linux-6.16.1/include/linux/fb.h \
    $(wildcard include/config/GUMSTIX_AM200EPD) \
    $(wildcard include/config/FB_NOTIFY) \
    $(wildcard include/config/FB_DEFERRED_IO) \
    $(wildcard include/config/FB_TILEBLITTING) \
    $(wildcard include/config/FB_BACKLIGHT) \
    $(wildcard include/config/FB_DEVICE) \
    $(wildcard include/config/FB_FOREIGN_ENDIAN) \
    $(wildcard include/config/FB_BOTH_ENDIAN) \
    $(wildcard include/config/FB_BIG_ENDIAN) \
    $(wildcard include/config/FB_LITTLE_ENDIAN) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/fb.h \
  /usr/src/linux/linux-6.16.1/include/linux/i2c.h \
    $(wildcard include/config/I2C) \
    $(wildcard include/config/I2C_SLAVE) \
    $(wildcard include/config/I2C_BOARDINFO) \
    $(wildcard include/config/I2C_MUX) \
  /usr/src/linux/linux-6.16.1/include/linux/acpi.h \
    $(wildcard include/config/ACPI_TABLE_LIB) \
    $(wildcard include/config/ACPI_DEBUGGER) \
    $(wildcard include/config/LOONGARCH) \
    $(wildcard include/config/RISCV) \
    $(wildcard include/config/ACPI_PROCESSOR_CSTATE) \
    $(wildcard include/config/ACPI_HOTPLUG_CPU) \
    $(wildcard include/config/ACPI_HOTPLUG_IOAPIC) \
    $(wildcard include/config/ACPI_WMI) \
    $(wildcard include/config/ACPI_THERMAL_LIB) \
    $(wildcard include/config/ACPI_HMAT) \
    $(wildcard include/config/ACPI_NUMA) \
    $(wildcard include/config/HIBERNATION) \
    $(wildcard include/config/ACPI_HOTPLUG_MEMORY) \
    $(wildcard include/config/ACPI_CONTAINER) \
    $(wildcard include/config/ACPI_GTDT) \
    $(wildcard include/config/ACPI_MRRM) \
    $(wildcard include/config/SUSPEND) \
    $(wildcard include/config/ACPI_EC) \
    $(wildcard include/config/GPIOLIB) \
    $(wildcard include/config/ACPI_TABLE_UPGRADE) \
    $(wildcard include/config/ACPI_WATCHDOG) \
    $(wildcard include/config/ACPI_SPCR_TABLE) \
    $(wildcard include/config/ACPI_GENERIC_GSI) \
    $(wildcard include/config/ACPI_LPIT) \
    $(wildcard include/config/ACPI_PROCESSOR_IDLE) \
    $(wildcard include/config/ACPI_PCC) \
    $(wildcard include/config/ACPI_FFH) \
  /usr/src/linux/linux-6.16.1/include/linux/resource_ext.h \
  /usr/src/linux/linux-6.16.1/include/linux/mod_devicetable.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/mei.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/mei_uuid.h \
  /usr/src/linux/linux-6.16.1/include/linux/property.h \
  /usr/src/linux/linux-6.16.1/include/linux/node.h \
    $(wildcard include/config/HMEM_REPORTING) \
  /usr/src/linux/linux-6.16.1/include/acpi/acpi.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/acenv.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/acgcc.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/aclinux.h \
    $(wildcard include/config/ACPI_REDUCED_HARDWARE_ONLY) \
    $(wildcard include/config/ACPI_DEBUG) \
  /usr/src/linux/linux-6.16.1/include/linux/ctype.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/acenv.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acnames.h \
  /usr/src/linux/linux-6.16.1/include/acpi/actypes.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acexcep.h \
  /usr/src/linux/linux-6.16.1/include/acpi/actbl.h \
  /usr/src/linux/linux-6.16.1/include/acpi/actbl1.h \
  /usr/src/linux/linux-6.16.1/include/acpi/actbl2.h \
  /usr/src/linux/linux-6.16.1/include/acpi/actbl3.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acrestyp.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/acenvex.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/aclinuxex.h \
  /usr/src/linux/linux-6.16.1/include/acpi/platform/acgccex.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acoutput.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acpiosxf.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acpixf.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acconfig.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acbuffer.h \
  /usr/src/linux/linux-6.16.1/include/acpi/acpi_numa.h \
  /usr/src/linux/linux-6.16.1/include/linux/fw_table.h \
    $(wildcard include/config/CXL_BUS) \
  /usr/src/linux/linux-6.16.1/include/acpi/acpi_bus.h \
    $(wildcard include/config/X86_ANDROID_TABLETS) \
    $(wildcard include/config/ACPI_SYSTEM_POWER_STATES_SUPPORT) \
    $(wildcard include/config/ACPI_SLEEP) \
  /usr/src/linux/linux-6.16.1/include/acpi/acpi_drivers.h \
    $(wildcard include/config/ACPI_DOCK) \
  /usr/src/linux/linux-6.16.1/include/acpi/acpi_io.h \
  /usr/src/linux/linux-6.16.1/include/linux/io.h \
    $(wildcard include/config/STRICT_DEVMEM) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/acpi.h \
    $(wildcard include/config/ACPI_APEI) \
  /usr/src/linux/linux-6.16.1/include/acpi/proc_cap_intel.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/numa.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/irq_vectors.h \
    $(wildcard include/config/HYPERV) \
    $(wildcard include/config/PCI_MSI) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/xen/hypervisor.h \
    $(wildcard include/config/XEN_PV_DOM0) \
    $(wildcard include/config/PVH) \
    $(wildcard include/config/XEN_DOM0) \
  /usr/src/linux/linux-6.16.1/include/xen/xen.h \
    $(wildcard include/config/XEN_PVH) \
    $(wildcard include/config/XEN_BALLOON) \
    $(wildcard include/config/XEN_UNPOPULATED_ALLOC) \
  /usr/src/linux/linux-6.16.1/include/xen/interface/hvm/start_info.h \
  /usr/src/linux/linux-6.16.1/include/xen/interface/xen.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/xen/interface.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/xen/interface_64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pvclock-abi.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/emulate_prefix.h \
  /usr/src/linux/linux-6.16.1/include/linux/regulator/consumer.h \
    $(wildcard include/config/REGULATOR) \
  /usr/src/linux/linux-6.16.1/include/linux/suspend.h \
    $(wildcard include/config/VT) \
    $(wildcard include/config/HIBERNATION_SNAPSHOT_DEV) \
    $(wildcard include/config/PM_SLEEP_DEBUG) \
    $(wildcard include/config/PM_AUTOSLEEP) \
  /usr/src/linux/linux-6.16.1/include/linux/swap.h \
    $(wildcard include/config/THP_SWAP) \
  /usr/src/linux/linux-6.16.1/include/linux/memcontrol.h \
    $(wildcard include/config/MEMCG_NMI_SAFETY_REQUIRES_ATOMIC) \
  /usr/src/linux/linux-6.16.1/include/linux/cgroup.h \
    $(wildcard include/config/DEBUG_CGROUP_REF) \
    $(wildcard include/config/CGROUP_CPUACCT) \
    $(wildcard include/config/SOCK_CGROUP_DATA) \
    $(wildcard include/config/CGROUP_DATA) \
    $(wildcard include/config/CGROUP_BPF) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/cgroupstats.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/taskstats.h \
  /usr/src/linux/linux-6.16.1/include/linux/seq_file.h \
  /usr/src/linux/linux-6.16.1/include/linux/string_helpers.h \
  /usr/src/linux/linux-6.16.1/include/linux/string_choices.h \
  /usr/src/linux/linux-6.16.1/include/linux/ns_common.h \
  /usr/src/linux/linux-6.16.1/include/linux/nsproxy.h \
  /usr/src/linux/linux-6.16.1/include/linux/user_namespace.h \
    $(wildcard include/config/INOTIFY_USER) \
    $(wildcard include/config/FANOTIFY) \
    $(wildcard include/config/BINFMT_MISC) \
    $(wildcard include/config/PERSISTENT_KEYRINGS) \
  /usr/src/linux/linux-6.16.1/include/linux/rculist_nulls.h \
  /usr/src/linux/linux-6.16.1/include/linux/kernel_stat.h \
    $(wildcard include/config/GENERIC_IRQ_STAT_SNAPSHOT) \
  /usr/src/linux/linux-6.16.1/include/linux/interrupt.h \
    $(wildcard include/config/IRQ_FORCED_THREADING) \
    $(wildcard include/config/GENERIC_IRQ_PROBE) \
    $(wildcard include/config/IRQ_TIMINGS) \
  /usr/src/linux/linux-6.16.1/include/linux/irqreturn.h \
  /usr/src/linux/linux-6.16.1/include/linux/hardirq.h \
  /usr/src/linux/linux-6.16.1/include/linux/context_tracking_state.h \
    $(wildcard include/config/CONTEXT_TRACKING_USER) \
    $(wildcard include/config/CONTEXT_TRACKING) \
  /usr/src/linux/linux-6.16.1/include/linux/ftrace_irq.h \
    $(wildcard include/config/HWLAT_TRACER) \
    $(wildcard include/config/OSNOISE_TRACER) \
  /usr/src/linux/linux-6.16.1/include/linux/vtime.h \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING) \
    $(wildcard include/config/IRQ_TIME_ACCOUNTING) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/hardirq.h \
    $(wildcard include/config/KVM_INTEL) \
    $(wildcard include/config/KVM) \
    $(wildcard include/config/X86_THERMAL_VECTOR) \
    $(wildcard include/config/X86_MCE_THRESHOLD) \
    $(wildcard include/config/X86_MCE_AMD) \
    $(wildcard include/config/X86_HV_CALLBACK_VECTOR) \
    $(wildcard include/config/X86_POSTED_MSI) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/irq.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/sections.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/sections.h \
    $(wildcard include/config/HAVE_FUNCTION_DESCRIPTORS) \
  /usr/src/linux/linux-6.16.1/include/linux/cgroup-defs.h \
    $(wildcard include/config/CGROUP_NET_CLASSID) \
    $(wildcard include/config/CGROUP_NET_PRIO) \
  /usr/src/linux/linux-6.16.1/include/linux/u64_stats_sync.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/asm/local64.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/local64.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/local.h \
  /usr/src/linux/linux-6.16.1/include/linux/bpf-cgroup-defs.h \
    $(wildcard include/config/BPF_LSM) \
  /usr/src/linux/linux-6.16.1/include/linux/psi_types.h \
  /usr/src/linux/linux-6.16.1/include/linux/kthread.h \
  /usr/src/linux/linux-6.16.1/include/linux/cgroup_subsys.h \
    $(wildcard include/config/CGROUP_DEVICE) \
    $(wildcard include/config/CGROUP_FREEZER) \
    $(wildcard include/config/CGROUP_PERF) \
    $(wildcard include/config/CGROUP_HUGETLB) \
    $(wildcard include/config/CGROUP_PIDS) \
    $(wildcard include/config/CGROUP_RDMA) \
    $(wildcard include/config/CGROUP_MISC) \
    $(wildcard include/config/CGROUP_DMEM) \
    $(wildcard include/config/CGROUP_DEBUG) \
  /usr/src/linux/linux-6.16.1/include/linux/cgroup_refcnt.h \
  /usr/src/linux/linux-6.16.1/include/linux/page_counter.h \
  /usr/src/linux/linux-6.16.1/include/linux/vmpressure.h \
  /usr/src/linux/linux-6.16.1/include/linux/eventfd.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/eventfd.h \
  /usr/src/linux/linux-6.16.1/include/linux/writeback.h \
  /usr/src/linux/linux-6.16.1/include/linux/flex_proportions.h \
  /usr/src/linux/linux-6.16.1/include/linux/backing-dev-defs.h \
    $(wildcard include/config/DEBUG_FS) \
  /usr/src/linux/linux-6.16.1/include/linux/blk_types.h \
    $(wildcard include/config/FAIL_MAKE_REQUEST) \
    $(wildcard include/config/BLK_CGROUP_IOCOST) \
    $(wildcard include/config/BLK_INLINE_ENCRYPTION) \
    $(wildcard include/config/BLK_DEV_INTEGRITY) \
  /usr/src/linux/linux-6.16.1/include/linux/bvec.h \
  /usr/src/linux/linux-6.16.1/include/linux/highmem.h \
  /usr/src/linux/linux-6.16.1/include/linux/cacheflush.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cacheflush.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/cacheflush.h \
  /usr/src/linux/linux-6.16.1/include/linux/kmsan.h \
  /usr/src/linux/linux-6.16.1/include/linux/highmem-internal.h \
  /usr/src/linux/linux-6.16.1/include/linux/pagevec.h \
  /usr/src/linux/linux-6.16.1/include/linux/bio.h \
    $(wildcard include/config/BLK_DEV_ZONED) \
  /usr/src/linux/linux-6.16.1/include/linux/mempool.h \
  /usr/src/linux/linux-6.16.1/include/linux/pagemap.h \
  /usr/src/linux/linux-6.16.1/include/linux/hugetlb_inline.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/mempolicy.h \
  /usr/src/linux/linux-6.16.1/include/linux/freezer.h \
  /usr/src/linux/linux-6.16.1/include/uapi/regulator/regulator.h \
  /usr/src/linux/linux-6.16.1/include/linux/rtmutex.h \
    $(wildcard include/config/DEBUG_RT_MUTEXES) \
  /usr/src/linux/linux-6.16.1/include/linux/irqdomain.h \
    $(wildcard include/config/IRQ_DOMAIN_HIERARCHY) \
    $(wildcard include/config/GENERIC_IRQ_DEBUGFS) \
    $(wildcard include/config/IRQ_DOMAIN) \
    $(wildcard include/config/IRQ_DOMAIN_NOMAP) \
  /usr/src/linux/linux-6.16.1/include/linux/irqdomain_defs.h \
  /usr/src/linux/linux-6.16.1/include/linux/irqhandler.h \
  /usr/src/linux/linux-6.16.1/include/linux/of.h \
    $(wildcard include/config/OF_DYNAMIC) \
    $(wildcard include/config/SPARC) \
    $(wildcard include/config/OF_PROMTREE) \
    $(wildcard include/config/OF_KOBJ) \
    $(wildcard include/config/OF_NUMA) \
    $(wildcard include/config/OF_OVERLAY) \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/i2c.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/vesa.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/video.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/video.h \
  /usr/src/linux/linux-6.16.1/include/linux/poll.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/poll.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/generated/uapi/asm/poll.h \
  /usr/src/linux/linux-6.16.1/include/uapi/asm-generic/poll.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/eventpoll.h \
  /usr/src/linux/linux-6.16.1/include/linux/pci.h \
    $(wildcard include/config/PCI_IOV) \
    $(wildcard include/config/PCIEAER) \
    $(wildcard include/config/PCIEPORTBUS) \
    $(wildcard include/config/PCIEASPM) \
    $(wildcard include/config/HOTPLUG_PCI_PCIE) \
    $(wildcard include/config/PCIE_PTM) \
    $(wildcard include/config/PCIE_DPC) \
    $(wildcard include/config/PCI_ATS) \
    $(wildcard include/config/PCI_PRI) \
    $(wildcard include/config/PCI_PASID) \
    $(wildcard include/config/PCI_DOE) \
    $(wildcard include/config/PCI_NPEM) \
    $(wildcard include/config/PCIE_TPH) \
    $(wildcard include/config/PCI_DOMAINS_GENERIC) \
    $(wildcard include/config/HOTPLUG_PCI) \
    $(wildcard include/config/PCI_DOMAINS) \
    $(wildcard include/config/PCI_QUIRKS) \
    $(wildcard include/config/ACPI_MCFG) \
    $(wildcard include/config/EEH) \
  /usr/src/linux/linux-6.16.1/include/linux/msi_api.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/pci.h \
  /usr/src/linux/linux-6.16.1/include/uapi/linux/pci_regs.h \
  /usr/src/linux/linux-6.16.1/include/linux/pci_ids.h \
  /usr/src/linux/linux-6.16.1/include/linux/dmapool.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pci.h \
    $(wildcard include/config/VMD) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/memtype.h \
  /usr/src/linux/linux-6.16.1/include/linux/vmalloc.h \
    $(wildcard include/config/HAVE_ARCH_HUGE_VMALLOC) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/vmalloc.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pgtable_areas.h \
  /usr/src/linux/linux-6.16.1/include/linux/aio.h \
  /usr/src/linux/linux-6.16.1/include/linux/splice.h \
  /usr/src/linux/linux-6.16.1/include/linux/pipe_fs_i.h \
  /usr/src/linux/linux-6.16.1/include/generated/uapi/linux/version.h \
  libxdma.h \
    $(wildcard include/config/BLOCK_ID) \
  xdma_thread.h \
  /usr/src/linux/linux-6.16.1/include/linux/cpuset.h \
    $(wildcard include/config/CPUSETS_V1) \
  /usr/src/linux/linux-6.16.1/include/linux/mmu_context.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/mmu_context.h \
  /usr/src/linux/linux-6.16.1/include/linux/pkeys.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/pkeys.h \
  /usr/src/linux/linux-6.16.1/include/trace/events/tlb.h \
  /usr/src/linux/linux-6.16.1/include/linux/tracepoint.h \
    $(wildcard include/config/HAVE_SYSCALL_TRACEPOINTS) \
  /usr/src/linux/linux-6.16.1/include/linux/rcupdate_trace.h \
    $(wildcard include/config/TASKS_TRACE_RCU_READ_MB) \
  /usr/src/linux/linux-6.16.1/include/linux/static_call.h \
  /usr/src/linux/linux-6.16.1/include/linux/cpu.h \
    $(wildcard include/config/GENERIC_CPU_DEVICES) \
    $(wildcard include/config/PM_SLEEP_SMP) \
    $(wildcard include/config/PM_SLEEP_SMP_NONZERO_CPU) \
    $(wildcard include/config/ARCH_HAS_CPU_FINALIZE_INIT) \
    $(wildcard include/config/CPU_MITIGATIONS) \
  /usr/src/linux/linux-6.16.1/include/linux/cpuhotplug.h \
    $(wildcard include/config/HOTPLUG_CORE_SYNC_DEAD) \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/static_call.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/text-patching.h \
    $(wildcard include/config/UML_X86) \
  /usr/src/linux/linux-6.16.1/include/trace/define_trace.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/debugreg.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/uapi/asm/debugreg.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/gsseg.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/desc.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/cpu_entry_area.h \
  /usr/src/linux/linux-6.16.1/arch/x86/include/asm/intel_ds.h \
  /usr/src/linux/linux-6.16.1/include/asm-generic/mmu_context.h \
  cdev_ctrl.h \

cdev_ctrl.o: $(deps_cdev_ctrl.o)

$(deps_cdev_ctrl.o):
