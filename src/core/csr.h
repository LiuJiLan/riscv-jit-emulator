//
// csr 模块对外接口 (Zicsr 6 个指令变体的统一入口 + 大 switch decode 分发)。
//
// 职责 (plan §1.6):
//   1. csr_op 是 6 csr 指令变体 (CSRRW/RS/RC + 3 I 变体) 的统一入口, interpreter / 未来
//      JIT translator 共用; 内部完成 decode + 权限检查 + 分发到具体小 r/w helper +
//      按 op 算 read_old + write_back, 返回 read_old 给 caller 写 rd。
//   2. 大 switch (csr_addr) 负责 "decode 分发" — 把统一入口拆到各个具体 csr 的小 helper
//      (file-static), 加新 csr 时只在大 switch 里加 case + 写一对小 helper。
//   3. csr 编号自带权限位段 (riscv.h CSR_ADDR_PRIV_* / CSR_ADDR_RO_*), 入口直接判:
//        - priv < 要求 → trap_raise_exception(hart, 2 /*illegal*/, raw_inst); 不返回
//        - 写 RO csr   → trap_raise_exception(hart, 2 /*illegal*/, raw_inst); 不返回
//      入口判失败走 _Noreturn longjmp 跳回 dispatcher sigsetjmp 落点。
//
// "I 变体" (RWI / RSI / RCI) 与"普通变体" (RW / RS / RC) 在 csr 侧无差别:
//   - csr_op 不感知 op 是不是 I 变体, 它只看 (op_kind, new_val); caller 在 RWI/RSI/RCI
//     时把 5-bit zimm (decoded_inst_t.rs1 字段直接用作数值) 传 new_val, 在 RW/RS/RC 时
//     把 READ_REG(rs1) 传 new_val。csr 侧统一处理。
//
// 接口形态选择 (返回值 vs out 指针):
//   方案 A (本接口): 返回值 = read_old, trap 不返回 (helper longjmp 接管)
//   方案 B 否决: int + out 指针, 一旦 helper _Noreturn 后返回值变死语义, 多写多读多废一笔
//   理由: csr 指令的"产品"就是 read_old; trap 路径不返回不需要返回值表达 — 不返回 ≠
//         "返回值表 trap"。caller 端 (interpreter) 写起来更紧凑: WRITE_REG(rd, csr_op(...))。
//
// rd 写不在 csr_op 内做:
//   csr_op 不感知 rd, caller 自己 WRITE_REG(d.rd, csr_op_returned_old). 这样 csr_op 也方便
//   给未来 JIT translator 用 (translator emit 出来的 host code 自己处理 rd 写)。
//
// raw_inst 参数:
//   trap (cause 2 illegal) 的 mtval 字段 RV 规范要求 = 触发指令原始 32-bit 编码; raw_inst
//   就是 caller 从 decoded_inst_t.raw_inst 字段拿到的值, csr_op 内部 trap 时透传给 helper。
//   csr 状态本身没存"当前指令", 必须由 caller 显式传。
//

#ifndef CORE_CSR_H
#define CORE_CSR_H

#include <stdint.h>

#include "cpu.h"
#include "riscv.h"  // uxlen_t / u32_t (typedef family; dummy.txt §13)

// 6 csr 指令变体的内核操作类型。 caller (interpreter / translator) 把"指令变体"翻译到这
// 三个内核操作 + (new_val 是不是 zimm) 由 caller 自己决定:
//   CSRRW  → CSR_OP_RW  + new_val = READ_REG(rs1)
//   CSRRWI → CSR_OP_RW  + new_val = (uint32_t)d.rs1                    /* 5-bit zimm */
//   CSRRS  → CSR_OP_RS  + new_val = READ_REG(rs1)
//   CSRRSI → CSR_OP_RS  + new_val = (uint32_t)d.rs1
//   CSRRC  → CSR_OP_RC  + new_val = READ_REG(rs1)
//   CSRRCI → CSR_OP_RC  + new_val = (uint32_t)d.rs1
//
// 算法:
//   read_old = csr_<addr>_read(hart);
//   switch (op) {
//     case CSR_OP_RW: csr_<addr>_write(hart, new_val);                  break;
//     case CSR_OP_RS: csr_<addr>_write(hart, read_old |  new_val);      break;
//     case CSR_OP_RC: csr_<addr>_write(hart, read_old & ~new_val);      break;
//   }
//   return read_old;
//
// "纯读不写"判定 (RV spec §2.1.2):
//   CSRRS / CSRRC / CSRRSI / CSRRCI + new_val == 0 时, 视为不写 (副作用不发生; 写 RO csr
//   不 trap)。CSRRW / CSRRWI 总是写 (rs1=x0 / zimm=0 时也写, 用 0 覆盖)。
//   csr_op 入口的 RO 写 trap 检查由这条规则裁剪。
typedef enum {
    CSR_OP_RW,      /* CSRRW  / CSRRWI */
    CSR_OP_RS,      /* CSRRS  / CSRRSI */
    CSR_OP_RC,      /* CSRRC  / CSRRCI */
} csr_op_t;

// csr_op —— 6 csr 指令变体的统一入口
//
// 参数:
//   hart      - 当前 CPU 状态
//   csr_addr  - 12-bit csr 地址 (decoded_inst_t.imm 字段; 高 20 位 0)
//   new_val   - 待写入值: RW/RS/RC 是 READ_REG(rs1), I 变体是 5-bit zimm (= d.rs1)
//   op        - csr 内核操作 (CSR_OP_RW/RS/RC)
//   raw_inst  - 触发指令原始 32-bit 编码; trap 路径填 mtval 用
//
// 返回值: read_old —— csr 旧值, caller 写 rd (rd=x0 时由 WRITE_REG 宏丢)
//
// trap 路径:
//   - priv < csr 要求   → trap_raise_exception(hart, 2, raw_inst), 不返回 (入口判)
//   - 写 RO csr         → trap_raise_exception(hart, 2, raw_inst), 不返回 (入口判)
//   - 不存在的 csr addr → fprintf 提示 + trap_raise_exception(hart, 2, raw_inst), 不返回
//                          (大 switch default; 跟 lsu.h/c BARE 不在 RAM 路径 fprintf+trap 同
//                          风格, dev-friendly 定位 + 跟 RV spec §2.1 一致)
uxlen_t csr_op(cpu_t *hart, uint32_t csr_addr, uxlen_t new_val,
               csr_op_t op, u32_t raw_inst);


// ----------------------------------------------------------------------------
// csr_mip_read —— mip 合成读 (跨模块 export; trap.c::trap_check_interrupt 也调)
//
// 合成 6 bit (dummy.txt §6 第 5 类 _mip_sw 合成读模型):
//   bit 1/5/9 (SSIP/STIP/SEIP_sw) ← hart->trap._mip_sw 软件 inject 字段
//   bit 3     MSIP ← is_clint_msip_pending(hartid) 异步源
//   bit 7     MTIP ← is_clint_timer_pending(hartid) compute
//   bit 9     SEIP_hw ← is_plic_seip_pending(hartid) (PLIC s_pending; OR _mip_sw bit 9)
//   bit 11    MEIP    ← is_plic_meip_pending(hartid) (PLIC m_pending)
//
// 调用方:
//   - csr_op 入口 CSR_MIP 路径 (大 switch)
//   - csr_sip_read 内部 (sip 是 mip 的 mask view, 调 csr_mip_read & SIP_MASK)
//   - trap_check_interrupt (dispatcher polling, 算 mip & mie & 全局 IE)
//
// 并发: hart->trap._mip_sw 是 plain (本 hart 单线程访问); 异步源 atomic load 在 clint
// 模块内. csr_mip_read 整体无 atomic RMW (read-only 合成).
uxlen_t csr_mip_read(cpu_t *hart);

#endif //CORE_CSR_H
