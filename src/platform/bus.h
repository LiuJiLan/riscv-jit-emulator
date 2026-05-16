//
// Created by liujilan on 2026/5/16.
// bus 模块对外接口 — MMIO 注册 + PA 派发。
//
// 职责: 持一份 (PA range → device read/write fn) 注册表; lsu / mmu 在 PA 落
// 非 RAM 时调 bus_dispatch_read/write 派发到对应 device。
//
// 三层职责分工 (mmu / pmp / 内存) 见 src/dummy.txt §8。bus 属"内存访问层"
// 的 MMIO 分支, 跟 RAM 直通路径平行 (lsu 在 IS_GPA_RAM 不命中时落 bus)。
//
// ----------------------------------------------------------------------------
// device 接口约定 (mmio_dev_t 字段)
// ----------------------------------------------------------------------------
//
// device init (例 clint_init / uart_init / plic_init) 时填一份 mmio_dev_t 栈
// 局部变量, 调 bus_register_mmio 登记 — 表内按值拷贝, 局部变量栈寿不影响后续。
//
//   gpa_start    设备 MMIO 起点 PA (含)
//   gpa_end      设备 MMIO 终点 PA (不含, 半开 [gpa_start, gpa_end))
//   ctx          设备内部状态指针 (read/write 透传; bus 不解读)
//   read / write fn:
//     off        相对 gpa_start 的字节偏移 (bus 已减好, device 内直接用)
//     buf, size  数据缓冲 + 字节数 (size = 1/2/4; future 8 留 64-bit MMIO)
//     返回 0    成功 (含设备内部决定的"合法忽略"也算成功)
//     返回 非0  设备拒绝 (size / offset / alignment 不合法; device 自己 fprintf
//               "why", 见报错风格 dummy.txt §5) — bus 透传给 caller, lsu 转
//               access fault
//   name         调试 / 错误信息用 (string literal; bus 持指针不拷字符串)
//
// ----------------------------------------------------------------------------
// 注册表能力上限
// ----------------------------------------------------------------------------
//
// 内部宏 BUS_MAX_DEV = 16 (v1 预期 < 5 device: CLINT / UART / PLIC / 可能
// virtio-blk / 可能 sifive_test)。超限 → bus_register_mmio fail。改大几乎免费
// (struct ~40 字节 / 项, 阈值在 bus.c 内部)。
//
// ----------------------------------------------------------------------------
// 当前不做 (注释占位; 见 dummy.txt §8 三层分工 + 未来扩展点)
// ----------------------------------------------------------------------------
//
// (a) 可 fetch 范围表: 类 RAM 区域 (未来 ROM / flash 读镜像) 登记 host_ptr 给
//     mmu_translate_pc 取指直通。v1 不实装 (项目长期不需要 ROM/flash; RAM 走
//     IS_GPA_RAM 内联即可)。真做时本头加 bus_register_fetch_region /
//     bus_unregister_fetch_region / bus_resolve_fetch 三个 API, 跟现 mmio
//     表平行, 不掺进 mmio_dev_t 字段。
// (b) device 主动 unregister + 热插拔: v1 不做; 真做时加 bus_unregister_mmio
//     + 用 RCU / atomic pointer swap 处理 reader (hart 线程查 MMIO) / writer
//     (热插拔线程改表) 并发。
//

#ifndef PLATFORM_BUS_H
#define PLATFORM_BUS_H

#include <stdint.h>

typedef struct {
    uint32_t gpa_start;
    uint32_t gpa_end;
    void *ctx;
    int (*read) (void *ctx, uint32_t off, void *buf, uint32_t size);
    int (*write)(void *ctx, uint32_t off, const void *buf, uint32_t size);
    const char *name;
} mmio_dev_t;

// 注册一个 MMIO 设备。成功 0; 失败 -1 (fprintf 自报 why, 报错风格 dummy.txt §5)。
// 调用时机: device init, 一般在 main.c ram_init 之后、dispatcher 启动之前。
// 失败模式:
//   1. dev == NULL 或 range 非法 (gpa_start >= gpa_end)
//   2. 注册表满 (>= BUS_MAX_DEV)
//   3. range 跟已注册 device 重叠
int bus_register_mmio(const mmio_dev_t *dev);

// 派发 MMIO 读 / 写。
// 返回:
//   0    = 命中某 device + device 处理成功
//   非 0 = 无 device 命中此 PA / 命中但 device 拒绝
//          caller (通常 lsu load/store_helper) 应 raise access fault
// 调用时机: lsu 在 IS_GPA_RAM 不命中后, 把 PA / value / size 透传过来。
int bus_dispatch_read (uint32_t pa, void *buf, uint32_t size);
int bus_dispatch_write(uint32_t pa, const void *buf, uint32_t size);

#endif //PLATFORM_BUS_H
