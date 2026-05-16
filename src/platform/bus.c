//
// Created by liujilan on 2026/5/16.
// bus 实现 — MMIO 注册表 (静态数组) + PA range 线性派发。
//
// 接口形态见 bus.h; 三层职责 (mmu / pmp / 内存) 见 src/dummy.txt §8;
// "0=成功" 接口约定 + cause 0 产生路径见 src/dummy.txt §9。
//
// 数据结构选型: 静态数组 + 线性查找。v1 设备数 < 5, log n / 树 / hash 都属
// 过度工程; 线性 < 16 cmp 在 slow path 入口里占比可忽略 (~ 几十 cycle / 派发,
// 而 device read/write 本身就是 helper-call 量级的 ~100+ cycle)。
//
// _Noreturn 失败路径: device 返非 0 cause → trap_raise_exception 透传; 未命中
// 由 bus 自产 CAUSE_LOAD/STORE_ACCESS_FAULT (5/7)。
//

#include "bus.h"

#include <stdint.h>
#include <stdio.h>

#include "core/trap.h"   // trap_raise_exception (_Noreturn longjmp)
#include "riscv.h"       // CAUSE_LOAD_ACCESS_FAULT / CAUSE_STORE_ACCESS_FAULT

#define BUS_MAX_DEV 16

static mmio_dev_t bus_table[BUS_MAX_DEV];
static uint32_t   bus_table_n = 0;

int bus_register_mmio(const mmio_dev_t *dev) {
    if (dev == NULL || dev->gpa_start >= dev->gpa_end) {
        fprintf(stderr, "bus_register_mmio: invalid dev or range\n");
        return -1;
    }
    if (bus_table_n >= BUS_MAX_DEV) {
        fprintf(stderr,
                "bus_register_mmio: bus_table full (%u/%u)\n",
                bus_table_n, (uint32_t)BUS_MAX_DEV);
        return -1;
    }
    // 半开 [a,b) ∩ [c,d) 非空 ⇔ a < d && c < b。强制不重叠, 避免派发歧义。
    for (uint32_t i = 0; i < bus_table_n; i++) {
        if (dev->gpa_start < bus_table[i].gpa_end &&
            bus_table[i].gpa_start < dev->gpa_end) {
            fprintf(stderr,
                    "bus_register_mmio: range [0x%08x,0x%08x) overlaps %s [0x%08x,0x%08x)\n",
                    dev->gpa_start, dev->gpa_end,
                    bus_table[i].name ? bus_table[i].name : "<noname>",
                    bus_table[i].gpa_start, bus_table[i].gpa_end);
            return -1;
        }
    }
    // 按值拷贝 — device init 用栈 mmio_dev_t 局部填好即可, 不需要保活。
    bus_table[bus_table_n] = *dev;
    bus_table_n++;
    return 0;
}

uint32_t mmio_read_helper(cpu_t *hart, uint32_t pa, uint32_t gva, uint32_t size) {
    for (uint32_t i = 0; i < bus_table_n; i++) {
        if (pa >= bus_table[i].gpa_start && pa < bus_table[i].gpa_end) {
            uint32_t value = 0;
            int cause = bus_table[i].read(bus_table[i].ctx,
                                          pa - bus_table[i].gpa_start,
                                          &value, size);
            if (cause == 0) return value;
            // device 拒绝: cause 由 device 自决, bus 透传
            trap_raise_exception(hart, (uint32_t)cause, gva);  // _Noreturn longjmp
        }
    }
    // 未命中: bus 自产 access fault
    trap_raise_exception(hart, CAUSE_LOAD_ACCESS_FAULT, gva);  // _Noreturn longjmp
}

void mmio_write_helper(cpu_t *hart, uint32_t pa, uint32_t gva,
                        uint32_t value, uint32_t size) {
    for (uint32_t i = 0; i < bus_table_n; i++) {
        if (pa >= bus_table[i].gpa_start && pa < bus_table[i].gpa_end) {
            int cause = bus_table[i].write(bus_table[i].ctx,
                                           pa - bus_table[i].gpa_start,
                                           &value, size);
            if (cause == 0) return;
            trap_raise_exception(hart, (uint32_t)cause, gva);  // _Noreturn longjmp
        }
    }
    trap_raise_exception(hart, CAUSE_STORE_ACCESS_FAULT, gva);  // _Noreturn longjmp
}
