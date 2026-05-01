//
// Created by liujilan on 2026/4/28.
// a_01 loader 模块对外接口。
//
// 三个函数:
//   guest_load_bin   把 .bin 文件内容拷到 guest GPA = dst 处
//   guest_load_elf   把 ELF 所有 PT_LOAD 段加载到各自 p_paddr 处
//   guest_is_elf     探测文件是否 ELF
//
// 报错风格见 src/dummy.txt §5。
//

#ifndef LOADER_H
#define LOADER_H

#include <stdint.h>

// 把 .bin 内容拷到 dst 处。成功返回 0,失败返回 -1
// (内部已 fprintf 具体原因)。
//
// dst 是 guest 物理地址, 必须落在 [GUEST_RAM_START, GUEST_RAM_START + GUEST_RAM_SIZE) 内。
// 类型用 uint64_t 是为 RV64 前瞻兼容 —— RV32 阶段实际值在 32 位内, 不引入运行时开销。
int guest_load_bin(const char *path, uint64_t dst);

// 把 ELF 文件的所有 PT_LOAD 段加载到各自 p_paddr 处。成功返回 0,失败返回 -1
// (内部已 fprintf 具体原因)。
//
// loader 的语义边界:
//   - 不挪 ELF: 段必须由使用者保证落在 RAM 区间内, loader 严格按 p_paddr 放, 越界即失败。
//   - 不管 entry: 入口由模拟器使用者保证 = GUEST_RAM_START。loader 不读 ehdr.e_entry。
//   - 不清 BSS: (p_memsz > p_filesz) 的差额由 guest 自身的 startup code 负责, loader 不动。
//
// 当前模拟器写死 RV32, 严格校验:
//   ELF magic / ELFCLASS32 / ELFDATA2LSB / EM_RISCV / ET_EXEC, 任一不符立即失败。
int guest_load_elf(const char *path);

// 探测 path 是否 ELF。返回 1 = 是,0 = 不是 / 无法读取。
// silently 返回(不 fprintf), 因为这是一个探测函数, 失败不算错误。
int guest_is_elf(const char *path);

#endif //LOADER_H
