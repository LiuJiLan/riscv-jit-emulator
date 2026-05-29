//
// Created by liujilan on 2026/4/29.
// ram 实现。
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
        fprintf(stderr, "ram_init: mmap failed: %s" EOL, strerror(errno));
        return -1;
    }

    // 显式排除 transparent huge page, 保 4KB 颗粒度服务 SMC 检测 (smc.c 的
    // page_dirty 位图按 4KB 索引; THP 把页合到 2MB 会让位图粒度失准)。
    if (madvise(p, GUEST_RAM_SIZE, MADV_NOHUGEPAGE) != 0) {
        fprintf(stderr, "ram_init: madvise(MADV_NOHUGEPAGE) failed: %s" EOL,
                strerror(errno));
        munmap(p, GUEST_RAM_SIZE);
        return -1;
    }

    host_ram_base = p;
    gpa_to_hva_offset = (uint8_t *)p - GUEST_RAM_START;
    return 0;
}

void ram_destroy(void) {
    if (host_ram_base == NULL) return;   // 未 init / 已 destroy, do nothing
    if (munmap(host_ram_base, GUEST_RAM_SIZE) != 0) {
        // POR 退出路径, 不 fatal; 进程结束后 OS 仍会回收。
        fprintf(stderr, "ram_destroy: munmap failed: %s" EOL, strerror(errno));
    }
    host_ram_base = NULL;
    gpa_to_hva_offset = NULL;
}