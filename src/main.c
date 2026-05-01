//
// Created by liujilan on 2026/4/28.
// a_01 main。
// a_01 范围内只到 loader 调用为止;cpu / dispatcher / cpu_destroy
// 三段保留为 #if 0 占位,等 a01_4(cpu + tlb + mmu)+ a01_5
// (interpreter + dispatcher + main 翻译)入场再激活。
//

#include "config.h"
#include "loader.h"
#include "platform/ram.h"

#include <stdio.h>
#include <string.h>

static int has_suffix(const char *s, const char *suffix) {
    size_t ns = strlen(s);
    size_t nsuf = strlen(suffix);
    if (ns < nsuf) return 0;
    return strcmp(s + ns - nsuf, suffix) == 0;
}

int main(int argc, char **argv) {
    // a_01: 命令行参数 ./jit-emu <bin-or-elf-path>
    if (argc < 2) {
        fprintf(stderr, "usage: %s <bin-or-elf-path>\n", argv[0]);
        return 1;
    }

    // 调用 ram_init();成功后 ram.h 暴露的全局 host_ram_base /
    // host_ram_base_adjusted 可用。报错风格见 src/dummy.txt §5。
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

#if 0
    // ========================================================================
    // 以下段落 a01_4(cpu / tlb / mmu)+ a01_5(interpreter / dispatcher /
    // main 翻译)入场后激活。保留 LiuJiLan 风格的设计意图注释。
    // ========================================================================
    //
    // cpu 提供 alloc_cpu_t(misa) -> cpu_t*(失败返回 NULL)
    // 参数会被忽略, 完全无用; 返回时, 整个结构体全 0
    // 作用解释:
    // 由于 cpu_t 中部分 isa 相关的部分, 使用的是指针引用避免整体结构过大,
    // 只有当这个 misa 有使用它的意图时, 才会去对应 isa 的函数去分配
    // 但由于现在都没有 misa 这个位置, 只是留下这个参数, 表达这种未来的接口
    // PS:
    // void * tlb_table[4] 此时直接是 cpu_t 字段
    // PS: 暂时不真的设置 sigjmp_buf *jmp_buf_ptr
    cpu_t *hart = alloc_cpu_t(0);
    if (hart == NULL) {                            // alloc_cpu_t 内部已 fprintf "why"
        fprintf(stderr, "alloc_cpu_t failed\n");
        return 1;
    }

    // 早期代码会直接设置这个 cpu_t hart, 之后会是有个函数来,
    // 但是这个函数先不存在, 因为放在哪没想好
    hart->pc    = GUEST_RAM_START;  // 考虑到远远的未来会有热插拔核, 程序起点在外部设置
    // 下面的来自参考 https://docs.kernel.org/arch/riscv/boot.html
    hart->a0    = 0;  // hartid #0    // cpu_t 中除了 x0, 使用 abi 命名
    hart->a1    = 0;  // 暂时没有设备树
    hart->satp  = 0;
    hart->priv  = 3;  // M 态

    // 注意, tlb 的实际分配不在外部

    dispatcher(hart);  // 在远远的未来, 就是 pthread 启动了

    // a_01 后: 真正退出 = helper 走 exit(), 走不到这
    cpu_destroy(hart);
#endif

    return 0;
}
