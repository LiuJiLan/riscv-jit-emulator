//
// Created by liujilan on 2026/4/29.
// a_01 ram 实现。
//

#define _GNU_SOURCE  // for MADV_NOHUGEPAGE on Linux

#include "ram.h"
#include "config.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

void *host_ram_base = NULL;
uint8_t *gpa_to_hva_offset = NULL;

int ram_init(void) {
    // MAP_NORESERVE:不预扣 swap commit charge。
    // 128MB 名义大、实际写少的设计下合适;
    // 代价是真写满系统时是 SIGBUS(而非 mmap 时拒绝)。
    // 未来可能修改
    void *p = mmap(NULL, GUEST_RAM_SIZE,
                   PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE,
                   -1, 0);
    if (p == MAP_FAILED) {
        fprintf(stderr, "ram_init: mmap failed: %s\n", strerror(errno));
        return -1;
    }

    // 显式排除 transparent huge page,保证 SMC 检测的 4KB 颗粒度。
    // 相关协议见 dummy.txt §1 / 后续 jit/smc.c 设计。
    if (madvise(p, GUEST_RAM_SIZE, MADV_NOHUGEPAGE) != 0) {
        fprintf(stderr, "ram_init: madvise(MADV_NOHUGEPAGE) failed: %s\n",
                strerror(errno));
        munmap(p, GUEST_RAM_SIZE);
        return -1;
    }

    host_ram_base = p;
    gpa_to_hva_offset = (uint8_t *)p - GUEST_RAM_START;
    return 0;
}