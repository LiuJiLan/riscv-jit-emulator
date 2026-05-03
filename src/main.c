//
// Created by liujilan on 2026/4/28.
// a_01 main。
// 当前形态: ram_init → loader → cpu_create + 字段设置 →
//             多次 dispatcher self-check (a01_2 mmu_translate_pc 端到端验证) → cpu_destroy。
// dispatcher 当前 a01_2 简化形态 (block 1+2 + 临时输出, 无循环), 完整 dispatcher
// (sigsetjmp + while loop + block 3 派发) 等 a01_5 真接入。
//

#include "config.h"
#include "core/cpu.h"
#include "core/dispatcher.h"
#include "loader.h"
#include "platform/ram.h"
#include "riscv.h"

#include <stdio.h>
#include <string.h>

static int has_suffix(const char *s, const char *suffix) {
    size_t ns = strlen(s);
    size_t nsuf = strlen(suffix);
    if (ns < nsuf) return 0;
    return strcmp(s + ns - nsuf, suffix) == 0;
}

// Debug 构建带 -fsanitize=address (含 LSan 的 exit-time 内存泄漏扫描)。
// 在 CLion Debug / gdb / strace 等 ptrace 环境下跑本程序, 必须设
//   ASAN_OPTIONS=abort_on_error=1:detect_leaks=0
// 否则 LSan 撞 ptrace 冲突会报 "LeakSanitizer has encountered a fatal error" 并 exit 1,
// 看起来像本程序的 bug 但其实是工具链限制。Run 模式(无 gdb)正常, 想查泄漏走 Run 即可。
int main(int argc, char **argv) {
    // a_01: 命令行参数 ./jit-emu <bin-or-elf-path>
    if (argc < 2) {
        fprintf(stderr, "usage: %s <bin-or-elf-path>\n", argv[0]);
        return 1;
    }

    // 调用 ram_init();成功后 ram.h 暴露的全局 host_ram_base /
    // gpa_to_hva_offset 可用。报错风格见 src/dummy.txt §5。
    if (ram_init() != 0) {
        fprintf(stderr, "ram_init failed\n");
        return 1;
    }

    // 文件后缀分发(.bin / .elf / 猜)
    // a_01 简化:ELF 路径全部 stub 返回 -1(内部 fprintf "not implemented"),
    //             guess_is_elf stub silently 返回 0,实际只走 .bin。
    int err = 0;
    const char *path = argv[1];
    if (has_suffix(path, ".bin")) {
        // 注意, 后续改进中, 如果有起点参数, 则使用起点参数, 否则使用
        err = guest_load_bin(path, GUEST_RAM_START);
    } else if (has_suffix(path, ".elf")) {
        err = guest_load_elf(path);                // a_01: stub
    } else {
        if (guest_is_elf(path)) {                  // a_01: stub
            err = guest_load_elf(path);            // a_01: stub
        } else {
            err = guest_load_bin(path, GUEST_RAM_START);
        }
    }
    if (err != 0) {                                // loader 内部已 fprintf "why"
        fprintf(stderr, "load failed\n");
        return 1;
    }

    // hart 构造: misa 参数 a_01 不读, 仅作未来 misa 驱动初始化预留 (cpu.h 已 doc)。
    // tlb_t **tlb_table[4] 是 cpu_t 字段; sigjmp_buf *jmp_buf_ptr 暂留 NULL (memset 0 已置)。
    // tlb 容器 + M 共享 leaf 已由 cpu_create eager alloc;
    // [PRIV_S][asid] 的 entries 由 dispatcher 懒分配 (a_01 仍 #if 0)。
    cpu_t *hart = cpu_create(0);
    if (hart == NULL) {                            // cpu_create 内部已 fprintf "why"
        fprintf(stderr, "cpu_create failed\n");
        return 1;
    }

    // hart 字段初始化 (以后会设计函数 / 启动协议接管)。
    // (但是一定是在 dispatcher 外部, 因为 hart 的热插拔 = hart寄存器初始化 + 开始运行)
    // 注: regs[0] 是 pc (cpu_t 内 regs[0] 物理占 x0 位置, 实际存 pc; 见 cpu.h)。
    hart->regs[0] = GUEST_RAM_START;       // pc; 程序起点; 未来热插拔核时由外部参数设置
    hart->satp    = 0;                      // bare 模式 (MODE=0, ASID=0, PPN=0 全 0)
    hart->priv    = PRIV_M;                 // M 模式
    // 下面的来自参考 https://docs.kernel.org/arch/riscv/boot.html
    // hart->regs[10] = 0;  // a0 = hartid #0    (Linux RV boot protocol; x10 = a0)
    // hart->regs[11] = 0;  // a1 = device tree pointer (暂无 dtb; x11 = a1)
    // 注: regs 已被 memset 0; 以上两行显式赋值是 boot protocol 文档化, 等价于 memset 后的现状。
    //     真上 OS 跑 (hartid > 0 / 有 dtb) 时解开注释 + 改右值。

    // ------------------------------------------------------------------------
    // a01_2 self-check: 多次设 hart->regs[0] (= pc) 调 dispatcher, 看 mmu_translate_pc
    // 路径输出。dispatcher 内部 (a01_2 简化形态) 走 block 1 (选 TLB) + block 2
    // (mmu_translate_pc) + 临时 fprintf, 不循环不真派发。
    //
    // 期望:
    //   - RAM 内 (含起点 / 中间 / 末尾对齐) → cause = 0, byte0 反映 loader 写入内容
    //   - RAM 外 (低地址 / 高地址 / RAM 紧后) → cause = 1 (Instruction Access Fault)
    //
    // 真激活完整 dispatcher 时这整段 self-check 删除, 只留一个 dispatcher(hart) 调用 + 内部 loop。
    // ------------------------------------------------------------------------

    /* test 1: RAM 起点, loader 已写入 (a01_1 fixture: 0xDEADBEEF 之类) */
    hart->regs[0] = GUEST_RAM_START;
    dispatcher(hart);

    /* test 2: RAM 内偏移, 仍在 loader 写入范围 (依 fixture) 或 mmap 零页 */
    hart->regs[0] = GUEST_RAM_START + 0x100;
    dispatcher(hart);

    /* test 3: RAM 末尾对齐, 还在范围 */
    hart->regs[0] = GUEST_RAM_START + GUEST_RAM_SIZE - 4;
    dispatcher(hart);

    /* test 4: 0 地址, RAM 外, 应 access fault */
    hart->regs[0] = 0;
    dispatcher(hart);

    /* test 5: 低地址但非 0, RAM 外, 应 access fault */
    hart->regs[0] = 0x1000;
    dispatcher(hart);

    /* test 6: 紧贴 RAM 后第一字节, RAM 外, 应 access fault */
    hart->regs[0] = GUEST_RAM_START + GUEST_RAM_SIZE;
    dispatcher(hart);

    /* test 7: 远高地址, RAM 外, 应 access fault */
    hart->regs[0] = 0xC0000000;
    dispatcher(hart);

    cpu_destroy(hart);
    return 0;
}
