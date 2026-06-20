//
// Created by liujilan on 2026/6/18.
// jit/backend_asmjit.cc —— JitBackend asmjit 实装 (T1+T2 真做).
//
// ============================================================================
// 跟 jit_entry.cc 的概念分工
// ============================================================================
//
// 本文件 = backend 实装细节; jit_entry.cc = jit_api.h 入口 (backend-agnostic).
//   - backend_asmjit.cc 暴露一个 file-static backend_t 实例 + backend_get_default()
//     extern "C" 返指针; 未来加 backend_llvm.cc 时是另一份 .cc, 不动本文件
//   - jit_entry.cc 通过 backend_get_default() 拿当前 backend_t* 调 vtable; 不感知
//     具体后端
//
// ============================================================================
// T1+T2+T3 实状
// ============================================================================
//
// 5 vtable 全真填:
//   asmjit_backend_init             alloc file-static asmjit::JitRuntime
//   asmjit_backend_compile_block    regime 分流: BARE → emit; SV32 → NOT_IMPLEMENTED
//                                    (set_blacklist BLACK 路径, interpret 兜底).
//                                    BARE 路径走 CodeHolder + Assembler emit
//                                    (prologue + 块体 + DISPATCH_EXIT + epilogue)
//                                    → JitRuntime::add 拿 RX 段地址, 返 jit_block_func_t
//   asmjit_backend_invalidate_block JitRuntime::release(host_code) (Q8 配合; backend
//                                    纯 unmap host code; 状态灯扫在 jit_entry.cc)
//   asmjit_backend_flush_all        JitRuntime::reset() (asmjit v1.18+ 自带 reset)
//   asmjit_backend_destroy          delete JitRuntime
//
// 块体 IR 翻译 (T1+T2+T3 范围): 48 op = RV32I 算术 21 + LOAD/STORE 8 + CSR 6 +
// AMO 9 + LR/SC 2 + FENCE/FENCE_I 2; IR_OP_DISPATCH_EXIT 收尾;
// IR_OP_UNSUPPORTED 哨兵 default case 写 __builtin_unreachable 兜底.
//
// T3 emit 形态分组 (BARE regime only; SV32 走 NOT_IMPLEMENTED):
//   LOAD 5  : inline IS_GPA_RAM 分流 + RAM 直 movsx/movzx eax, [hva] / MMIO
//             call mmio_read_helper (plan §1.1 fast path; lsu.h:119 line "JIT
//             自己 inline 等价逻辑")
//   STORE 3 : inline IS_GPA_RAM 分流 + RAM 走 call store_helper (LR/SC + SMC
//             副作用强制经 helper, lsu.h §load/store 不对称真机理) / MMIO call
//             mmio_write_helper
//   AMO 9   : 直接 call mmu_walker_helper_amo_xxx (BARE walker 内分流; 不在
//             backend inline IS_GPA_RAM 因 9 × 18 段重复 emit 不如统一 walker)
//   LR/SC 2 : 直接 call mmu_walker_helper_lr_w / sc_w (同 AMO)
//   CSR 6   : call csr_op(hart, csr_addr, new_val, op, raw_inst); 6 变体在
//             ir_inst.kind 区分 + new_val/op 静态选择; 硬边界, 后续紧跟
//             DISPATCH_EXIT
//   FENCE 2 : FENCE call fence_helper / FENCE_I call fence_i_helper (硬边界,
//             后续紧跟 DISPATCH_EXIT)
//
// 静态固定映射 (Q2=b): RV x1-x5 (ra/sp/gp/tp/t0) → host callee-saved 32-bit reg
//   x1 → ebx
//   x2 → r12d
//   x3 → r13d
//   x4 → r14d
//   x5 → r15d
// 其余 RV reg (x0 = 常数 0 / x6-x31 = cpu->regs[r] 内存) 走 emit_load_rv_reg /
// emit_store_rv_reg helper 分流.
//
// prologue/epilogue (Q6=a): 真做 load/store x1-x5 from/to cpu->regs[1..5]; 跟块体
// ADD/ADDI 真消费形成 "load + op + store" 三段闭环.
//
// host code 地址复用 (Q1=a 注): asmjit allocator 在 rt.release 后可能复用同地址,
// jit_cache 清旧 status + RCU grace 已等所有 reader 出, 不撞 stale.
//
// ============================================================================
// 调用约定 (jit_block_func_t)
// ============================================================================
//
// typedef void (*jit_block_func_t)(cpu_t *hart, tlb_t *current_tlb,
//                                  uint64_t *count_out);
//
// x86_64 SysV ABI:
//   rdi = hart        (cpu_t *)         — emit 内 base reg 用于 cpu->regs[N] 访问
//   rsi = current_tlb (tlb_t *)         — T1 c+ 不用; T3 LOAD/STORE 走 TLB 时用
//   rdx = count_out   (uint64_t *)      — emit 内目标地址写 *count_out
//
// callee-saved: rbx, rbp, r12-r15 (我们占 rbx + r12-r15 共 5 个映射 x1-x5; rbp
//   留 frame pointer)
// caller-saved: rax, rcx, rdx, rsi, rdi, r8-r11 (helper call 内可能改; 块体内
//   reload 协议 — 见下)
//
// T3 helper call 协议 (prologue 加 push rdi+rdx, 块体内 reload):
//   prologue push rbp / rbx / r12-r15 / rdi / rdx 共 8 个 8-byte → rsp % 16 == 0
//     (call 之前满足 SysV 16-byte 对齐, 进 helper 后 rsp % 16 == 8)
//   每次 helper call 之前 emit:
//     mov rdi, [rbp - 48]        ; reload hart 指针
//     (设其他参数: rsi/rdx/rcx 等按 helper signature)
//     call <helper_static_addr>
//   helper return 后 rax 装返回值, rdi/rsi/rdx/rcx 可能被 helper 改 (caller-saved)
//   下次 helper call 前再 reload rdi
//   映射 host reg (rbx/r12-r15 = x1-x5) callee-saved, helper 不破坏, 不 save/restore
//   emit_dispatch_exit 前置 reload rdi + rdx (写 cpu->regs[0] + *count_out)
//   epilogue 入口前 rdi 已 reload (emit_dispatch_exit 末), 直接 store x1-x5
//
// 栈布局 (push 顺序):
//   [rbp]       = old rbp (push rbp 时压)
//   [rbp - 8]   = rbx        (push rbx 时压)
//   [rbp - 16]  = r12
//   [rbp - 24]  = r13
//   [rbp - 32]  = r14
//   [rbp - 40]  = r15
//   [rbp - 48]  = rdi (hart pointer, save 用于 helper call reload)
//   [rbp - 56]  = rdx (count_out pointer, save 用于 emit_dispatch_exit reload)
//
// ============================================================================
// 命名 (项目体例)
// ============================================================================
//
// 5 个 file-static vtable fn prefix `asmjit_backend_*` (跟 stub 阶段一致); 内部
// emit helper 在 anonymous namespace 内 (file-static 等价 C++ 体例).
//
// 不引 class / 不继承 / 不多态 — 项目 C-style vtable, .cc 内部代码也保持 C-style
// (file-static fn + extern "C"); 见 plan §1.5 + §1.12 实状.
//

#include "backend.h"

#include <new>              // std::nothrow
#include <cstdio>           // fprintf (init / flush_all 失败报错)
#include <cstddef>          // offsetof
#include <cstdint>

#include <asmjit/x86.h>     // asmjit::JitRuntime / CodeHolder / x86::Assembler / imm()
                            // (asmjit/asmjit.h v1.18 已 deprecated, 用 x86.h)

#include "core/cpu.h"       // cpu_t (offsetof regs)
#include "core/mmu.h"       // regime_t (T3 regime 分流)
#include "ir.h"             // ir_inst_t + IR_OP_*
#include "api/helpers.h"    // T3 slow path helper 集中声明 (mmu_walker / csr_op /
                            //   store_helper / mmio_*_helper / fence_*_helper /
                            //   trap_raise_exception 等; backend emit call 时取
                            //   helper 静态地址)
#include "platform/ram.h"   // T3 LOAD/STORE BARE inline IS_GPA_RAM 用
                            //   gpa_to_hva_offset extern var + GUEST_RAM_START /
                            //   GUEST_RAM_SIZE config 宏


// ============================================================================
// file-static state (Q1=a JitRuntime 自管 RX 段)
// ============================================================================
//
// g_rt 指针 (非实例) — flush_all 需要 delete + new (asmjit 没 reset API). init
// 一次性 alloc, destroy 一次性 release, lifetime 内只 destroy/init 各 1 次
// (system reset 不触发, jit_init/jit_shutdown 配对 POR/teardown).

static asmjit::JitRuntime *g_rt = nullptr;


// ============================================================================
// emit helpers (anonymous namespace; file-static 等价)
// ============================================================================

namespace {

// regs[reg_idx] 字段的字节偏移 (regs 是 cpu_t 第一字段, _Alignas(64) → offset 0;
// 每条 4 byte = sizeof(uxlen_t)).
inline int32_t regs_offset(uint8_t reg_idx) {
    return static_cast<int32_t>(offsetof(cpu_t, regs)) +
           static_cast<int32_t>(reg_idx) * 4;
}

// 32-bit host reg 对应 RV reg 1..5 (Q2=b 拍法).
inline asmjit::x86::Gp host_reg_for_rv(uint8_t rv_reg) {
    switch (rv_reg) {
        case 1: return asmjit::x86::ebx;
        case 2: return asmjit::x86::r12d;
        case 3: return asmjit::x86::r13d;
        case 4: return asmjit::x86::r14d;
        case 5: return asmjit::x86::r15d;
        default: break;
    }
    // 不可达 (caller 保证 1..5); 兜底返 ebx 防编译警告
    return asmjit::x86::ebx;
}

// emit_reload_hart 前向声明 (定义见后); rs/rd > 5 case 用 rdi 作 cpu->regs[]
// base, 前置 reload 保证 rdi = hart (T3 块体内 helper call 可能破坏 rdi).
void emit_reload_hart(asmjit::x86::Assembler &a);

// 把 RV reg 值 load 到 host scratch reg (32-bit Gpd):
//   rv_reg == 0 → xor scratch, scratch (常数 0; dummy.txt §2 read x0 = 0)
//   1..5        → mov scratch, host_reg_for_rv(rv_reg) (固定 host reg 直读)
//   6..31       → reload rdi (T3 协议) + mov scratch, [rdi + regs_offset(rv_reg)]
void emit_load_rv_reg(asmjit::x86::Assembler &a, asmjit::x86::Gp scratch,
                      uint8_t rv_reg) {
    if (rv_reg == 0) {
        a.xor_(scratch, scratch);
    } else if (rv_reg <= 5) {
        a.mov(scratch, host_reg_for_rv(rv_reg));
    } else {
        emit_reload_hart(a);   /* T3: rdi 可能被 helper call 破坏, reload */
        a.mov(scratch,
              asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(rv_reg)));
    }
}

// 把 host scratch reg (32-bit) 写入 RV reg:
//   rv_reg == 0 → skip (dead store; dummy.txt §2 write x0 走 garbage)
//   1..5        → mov host_reg_for_rv(rv_reg), scratch
//   6..31       → reload rdi (T3 协议) + mov [rdi + regs_offset(rv_reg)], scratch
void emit_store_rv_reg(asmjit::x86::Assembler &a, uint8_t rv_reg,
                       asmjit::x86::Gp scratch) {
    if (rv_reg == 0) {
        return;
    }
    if (rv_reg <= 5) {
        a.mov(host_reg_for_rv(rv_reg), scratch);
    } else {
        emit_reload_hart(a);   /* T3: rdi 可能被 helper call 破坏, reload */
        a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(rv_reg)),
              scratch);
    }
}

// 栈 slot 偏移常量 (顶段 doc "栈布局" 段; T3 helper call 协议).
constexpr int32_t STACK_SLOT_HART      = -48;  // [rbp - 48] = saved rdi (hart pointer)
constexpr int32_t STACK_SLOT_COUNT_OUT = -56;  // [rbp - 56] = saved rdx (count_out ptr)

// prologue: push callee-saved (rbp + rbx + r12-r15) + push rdi/rdx (T3 helper call
// reload 用) + load x1-x5 from cpu->regs[1..5].
//
// 8 个 push (rbp + rbx + r12-r15 + rdi + rdx) 后 rsp % 16 == 0, helper call 进
// 入时 rsp % 16 == 8 (SysV ABI 满足).
void emit_prologue(asmjit::x86::Assembler &a) {
    a.push(asmjit::x86::rbp);
    a.mov(asmjit::x86::rbp, asmjit::x86::rsp);
    a.push(asmjit::x86::rbx);
    a.push(asmjit::x86::r12);
    a.push(asmjit::x86::r13);
    a.push(asmjit::x86::r14);
    a.push(asmjit::x86::r15);
    a.push(asmjit::x86::rdi);   // [rbp - 48] = hart pointer (helper call reload)
    a.push(asmjit::x86::rdx);   // [rbp - 56] = count_out ptr (emit_dispatch_exit reload)

    // load x1-x5 from cpu->regs[1..5] (rdi 入口还是 hart, 未被 helper call 破坏)
    a.mov(asmjit::x86::ebx,
          asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(1)));
    a.mov(asmjit::x86::r12d,
          asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(2)));
    a.mov(asmjit::x86::r13d,
          asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(3)));
    a.mov(asmjit::x86::r14d,
          asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(4)));
    a.mov(asmjit::x86::r15d,
          asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(5)));
}

// helper: reload hart 指针 (rdi) 从栈 saved slot. T3 每次 helper call 之前调.
void emit_reload_hart(asmjit::x86::Assembler &a) {
    a.mov(asmjit::x86::rdi,
          asmjit::x86::qword_ptr(asmjit::x86::rbp, STACK_SLOT_HART));
}

// epilogue: store x1-x5 to cpu->regs[1..5] + pop 对称 + ret.
//
// 入口前提: rdi 已是 hart 指针 (emit_dispatch_exit 末已 reload). pop 顺序对称
// prologue (后入先出).
void emit_epilogue(asmjit::x86::Assembler &a) {
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(1)),
          asmjit::x86::ebx);
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(2)),
          asmjit::x86::r12d);
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(3)),
          asmjit::x86::r13d);
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(4)),
          asmjit::x86::r14d);
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(5)),
          asmjit::x86::r15d);

    a.pop(asmjit::x86::rdx);
    a.pop(asmjit::x86::rdi);
    a.pop(asmjit::x86::r15);
    a.pop(asmjit::x86::r14);
    a.pop(asmjit::x86::r13);
    a.pop(asmjit::x86::r12);
    a.pop(asmjit::x86::rbx);
    a.pop(asmjit::x86::rbp);
    a.ret();
}

// emit IR 算术 (T1+T2 范围: RV32I 算术全集 21 op).
//
// scratch 用法 (T1+T2 不调 helper 不撞 caller-saved):
//   eax (caller-saved): 主累加器 (rs1 装 eax, op 后存 rd)
//   ecx (caller-saved): R-type 的 rs2 / shift count cl 来源
//   al  (eax 低 8 位):   setcc 目标 (SLT/SLTU/SLTI/SLTIU)
//   cl  (ecx 低 8 位):   shl/shr/sar 的 shift count (R-type shift)
//
// emit pattern 按 RV op 类型分组:
//   U-type (2):    a.mov(eax, imm); store rd          (translator 端常量已折叠)
//   I-imm 算术 (4): load rs1→eax; a.OP(eax, imm); store rd     (ADDI/ANDI/ORI/XORI)
//   I-imm 比较 (2): load rs1→eax; cmp+setcc(al)+movzx(eax,al); store rd  (SLTI/SLTIU)
//   I-imm shift (3): load rs1→eax; a.SHIFT(eax, imm&0x1F); store rd      (SLLI/SRLI/SRAI)
//   R-type 算术 (5): load rs1→eax; load rs2→ecx; a.OP(eax,ecx); store rd  (ADD/SUB/AND/OR/XOR)
//   R-type 比较 (2): load rs1→eax; load rs2→ecx; cmp+setcc+movzx; store  (SLT/SLTU)
//   R-type shift (3): load rs1→eax; load rs2→ecx; and ecx,0x1F; a.SHIFT(eax,cl); store
//                     (SLL/SRL/SRA; x86 shift count 必须放 cl; ecx 跟 cl 共享低 8 位;
//                      `and ecx, 0x1F` 跟 interpreter `& 0x1Fu` 文字对偶, x86 自身也是
//                      这行为, 不算 redundant)
//
// 函数顶引 `using namespace asmjit::x86` 简化 21 个 case 的 reg 引用 (eax/ecx/al/cl);
// 局部作用域不污染其他 helper.
void emit_ir_arith(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    switch (inst.kind) {
        /* ---- U-type (translator 端 imm 已合并 baked_pc + d.imm) ---- */
        case IR_OP_LUI:
        case IR_OP_AUIPC:
            a.mov(eax, asmjit::imm(inst.imm));
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- I-imm 算术 (4: ADDI/ANDI/ORI/XORI) ---- */
        case IR_OP_ADDI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.add(eax, asmjit::imm(inst.imm));
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_ANDI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.and_(eax, asmjit::imm(inst.imm));
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_ORI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.or_(eax, asmjit::imm(inst.imm));
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_XORI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.xor_(eax, asmjit::imm(inst.imm));
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- I-imm 比较 (2: SLTI signed / SLTIU unsigned; imm 先 sext 到 32) ---- */
        case IR_OP_SLTI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.cmp(eax, asmjit::imm(inst.imm));
            a.setl(al);                 /* signed less */
            a.movzx(eax, al);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SLTIU:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.cmp(eax, asmjit::imm(inst.imm));
            a.setb(al);                 /* unsigned less (below) */
            a.movzx(eax, al);
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- I-imm shift (3: SLLI/SRLI/SRAI; shamt 立即数走 imm & 0x1F) ---- */
        case IR_OP_SLLI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.shl(eax, asmjit::imm(inst.imm & 0x1F));
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SRLI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.shr(eax, asmjit::imm(inst.imm & 0x1F));
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SRAI:
            emit_load_rv_reg(a, eax, inst.rs1);
            a.sar(eax, asmjit::imm(inst.imm & 0x1F));
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- R-type 算术 (5: ADD/SUB/AND/OR/XOR) ---- */
        case IR_OP_ADD:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.add(eax, ecx);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SUB:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.sub(eax, ecx);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_AND:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.and_(eax, ecx);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_OR:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.or_(eax, ecx);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_XOR:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.xor_(eax, ecx);
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- R-type 比较 (2: SLT signed / SLTU unsigned) ---- */
        case IR_OP_SLT:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.cmp(eax, ecx);
            a.setl(al);
            a.movzx(eax, al);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SLTU:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.cmp(eax, ecx);
            a.setb(al);
            a.movzx(eax, al);
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- R-type shift (3: SLL/SRL/SRA; shift count 必须放 cl) ---- */
        case IR_OP_SLL:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.and_(ecx, asmjit::imm(0x1F));
            a.shl(eax, cl);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SRL:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.and_(ecx, asmjit::imm(0x1F));
            a.shr(eax, cl);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SRA:
            emit_load_rv_reg(a, eax, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            a.and_(ecx, asmjit::imm(0x1F));
            a.sar(eax, cl);
            emit_store_rv_reg(a, inst.rd, eax);
            return;

        /* ---- 哨兵 + T3 非算术 op (emit_ir_arith 不该看到; caller 路径已分流到
                emit_ir_load / store / amo / lr_sc / csr_fence) ---- */
        case IR_OP_UNSUPPORTED:
        case IR_OP_DISPATCH_EXIT:
        case IR_OP_LB: case IR_OP_LH: case IR_OP_LW: case IR_OP_LBU: case IR_OP_LHU:
        case IR_OP_SB: case IR_OP_SH: case IR_OP_SW:
        case IR_OP_CSRRW: case IR_OP_CSRRS: case IR_OP_CSRRC:
        case IR_OP_CSRRWI: case IR_OP_CSRRSI: case IR_OP_CSRRCI:
        case IR_OP_AMO_ADD_W: case IR_OP_AMO_SWAP_W: case IR_OP_AMO_XOR_W:
        case IR_OP_AMO_OR_W: case IR_OP_AMO_AND_W: case IR_OP_AMO_MIN_W:
        case IR_OP_AMO_MAX_W: case IR_OP_AMO_MINU_W: case IR_OP_AMO_MAXU_W:
        case IR_OP_LR_W: case IR_OP_SC_W:
        case IR_OP_FENCE: case IR_OP_FENCE_I:
            __builtin_unreachable();
    }
}

// ============================================================================
// T3 emit helper — helper call ABI 协议 (顶段 doc "T3 helper call 协议" 段)
// ============================================================================
//
// 通用 ABI (SysV x86_64):
//   - 参数顺序: rdi / rsi / rdx / rcx / r8 / r9
//   - 每次 helper call 前: emit_reload_hart (rdi = hart) + 设其他参数 + call
//   - call 时 rsp % 16 == 0 (prologue push 8 个 callee-saved 自然对齐, 不需 sub rsp)
//   - helper return 后 rax 装返回值; rdi/rsi/rdx/rcx/r8/r9 (caller-saved) 可能被改
//   - 映射 host reg (rbx/r12-r15 = x1-x5) callee-saved, helper 不破坏, 不 save/restore
//
// gpa_to_hva_offset baked into JIT 块: ram_init 在 main POR 一次性填, JIT 编译触
// 发时已 init; ram_destroy 在 POR 退出前才重置, JIT 块只在 POR 内跑 — baked 安全
// 且省一条 runtime memory load.
//
// ============================================================================

// helper: emit "call <absolute_addr>" (静态绑定 helper 函数指针; dummy.txt §10
// "helper 颗粒度 by design"). 跟 a.call(asmjit::imm(uint64_t)) 等价, 包装一层
// 防止 reinterpret_cast 反复 boilerplate.
template<typename Fn>
inline void emit_call_helper(asmjit::x86::Assembler &a, Fn *fn) {
    a.call(asmjit::imm(reinterpret_cast<uintptr_t>(fn)));
}

// emit_ir_load (5 op LB/LH/LW/LBU/LHU; BARE inline IS_GPA_RAM):
//
// 形态 (跟 lsu.h:144-160 line interpreter BARE 路径同结构):
//   eax = rs1 + imm (gva)
//   IS_GPA_RAM(gva) ? RAM (load *hva sext/zext) : MMIO (call mmio_read_helper)
//   .Ldone: emit_store_rv_reg rd, eax
//
// rd = x0 时 emit_store_rv_reg 走 skip 分支 — load 仍 emit (副作用 mmio_read_helper
// 仍触发, 跟 interpreter case 同), 只是值丢.
void emit_ir_load(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    asmjit::Label l_mmio = a.new_label();
    asmjit::Label l_done = a.new_label();

    // eax = gva = rs1 + imm
    emit_load_rv_reg(a, eax, inst.rs1);
    a.add(eax, asmjit::imm(inst.imm));

    // IS_GPA_RAM(gva): (uint32_t)(gva - GUEST_RAM_START) < GUEST_RAM_SIZE
    a.mov(ecx, eax);
    a.sub(ecx, asmjit::imm(static_cast<int32_t>(GUEST_RAM_START)));
    a.cmp(ecx, asmjit::imm(static_cast<int32_t>(GUEST_RAM_SIZE)));
    a.jae(l_mmio);   /* unsigned ≥ → MMIO 路径 */

    /* ---- RAM fast path ---- */
    /* hva = gpa_to_hva_offset + gva (gpa_to_hva_offset value baked) */
    const uintptr_t hva_offset_baked = reinterpret_cast<uintptr_t>(gpa_to_hva_offset);
    a.mov(rdx, asmjit::imm(hva_offset_baked));
    /* eax 高 32 位自动 zero-ext (x86 32-bit op 自动 clear upper 32 of rax), 所以 rax = (uint64_t)gva */
    switch (inst.kind) {
        case IR_OP_LB:   a.movsx(eax, byte_ptr (rdx, rax)); break;
        case IR_OP_LH:   a.movsx(eax, word_ptr (rdx, rax)); break;
        case IR_OP_LW:   a.mov  (eax, dword_ptr(rdx, rax)); break;
        case IR_OP_LBU:  a.movzx(eax, byte_ptr (rdx, rax)); break;
        case IR_OP_LHU:  a.movzx(eax, word_ptr (rdx, rax)); break;
        default: __builtin_unreachable();
    }
    a.jmp(l_done);

    /* ---- MMIO path (call mmio_read_helper(hart, pa=gva, gva, size)) ---- */
    a.bind(l_mmio);
    /* eax 仍是 gva (RAM path 改了 rax 但 jmp 前; MMIO 跳进来 eax 没动) */
    emit_reload_hart(a);              /* rdi = hart */
    a.mov(esi, eax);                  /* pa = gva (BARE identity) */
    a.mov(edx, eax);                  /* gva */
    /* size 按 op_kind */
    uint32_t size = 0;
    switch (inst.kind) {
        case IR_OP_LB: case IR_OP_LBU:  size = 1; break;
        case IR_OP_LH: case IR_OP_LHU:  size = 2; break;
        case IR_OP_LW:                  size = 4; break;
        default: __builtin_unreachable();
    }
    a.mov(ecx, asmjit::imm(size));
    emit_call_helper(a, mmio_read_helper);
    /* helper 返 raw uxlen_t in eax; sext/zext 按 op_kind (RAM 路径已做, MMIO 也做) */
    switch (inst.kind) {
        case IR_OP_LB:   a.movsx(eax, al); break;
        case IR_OP_LH:   a.movsx(eax, ax); break;
        case IR_OP_LW:   /* 32-bit 已满, noop */ break;
        case IR_OP_LBU:  a.movzx(eax, al); break;
        case IR_OP_LHU:  a.movzx(eax, ax); break;
        default: __builtin_unreachable();
    }

    a.bind(l_done);
    emit_store_rv_reg(a, inst.rd, eax);
}

// emit_ir_store (3 op SB/SH/SW; BARE inline IS_GPA_RAM):
//
// 形态:
//   eax = gva = rs1 + imm
//   r8d = value = rs2 (save before IS_GPA_RAM check, jump 两路都用)
//   IS_GPA_RAM(gva) ?
//     RAM: call store_helper(hart, hva=gpa_to_hva_offset+gva, gva, value, size) -- LR/SC + SMC 副作用强制经 helper
//     MMIO: call mmio_write_helper(hart, pa=gva, gva, value, size)
//
// STORE 无 rd, .Ldone 不写 rv reg.
void emit_ir_store(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    asmjit::Label l_mmio = a.new_label();
    asmjit::Label l_done = a.new_label();

    /* eax = gva = rs1 + imm */
    emit_load_rv_reg(a, eax, inst.rs1);
    a.add(eax, asmjit::imm(inst.imm));

    /* r8d = value = rs2 (跳两路前 save; r8 caller-saved 但 IS_GPA_RAM check 不
       会破坏 r8, helper call 路径会重设 r8 = size) */
    emit_load_rv_reg(a, r8d, inst.rs2);

    /* IS_GPA_RAM check */
    a.mov(ecx, eax);
    a.sub(ecx, asmjit::imm(static_cast<int32_t>(GUEST_RAM_START)));
    a.cmp(ecx, asmjit::imm(static_cast<int32_t>(GUEST_RAM_SIZE)));
    a.jae(l_mmio);

    /* ---- RAM path: call store_helper(hart, hva, gva, value, size) ---- */
    /* SysV ABI: hart=rdi, hva=rsi, gva=rdx, value=rcx, size=r8 */
    const uintptr_t hva_offset_baked = reinterpret_cast<uintptr_t>(gpa_to_hva_offset);
    emit_reload_hart(a);                                      /* rdi = hart */
    a.mov(rsi, asmjit::imm(hva_offset_baked));                /* rsi = gpa_to_hva_offset */
    a.add(rsi, rax);                                          /* rsi = hva = offset + gva (rax 高 32 已 0) */
    a.mov(edx, eax);                                          /* gva (传给 store_helper 当 tval 备用) */
    a.mov(ecx, r8d);                                          /* value (低 32-bit) */
    uint32_t size_ram = 0;
    switch (inst.kind) {
        case IR_OP_SB: size_ram = 1; break;
        case IR_OP_SH: size_ram = 2; break;
        case IR_OP_SW: size_ram = 4; break;
        default: __builtin_unreachable();
    }
    a.mov(r8d, asmjit::imm(size_ram));                        /* size */
    emit_call_helper(a, store_helper);
    a.jmp(l_done);

    /* ---- MMIO path: call mmio_write_helper(hart, pa=gva, gva, value, size) ---- */
    a.bind(l_mmio);
    /* eax 仍是 gva, r8d 仍是 value (从入口起没改) */
    emit_reload_hart(a);                                      /* rdi = hart */
    a.mov(esi, eax);                                          /* pa = gva */
    a.mov(edx, eax);                                          /* gva */
    a.mov(ecx, r8d);                                          /* value */
    uint32_t size_mmio = size_ram;   /* 跟 RAM 路径同 size 计算 */
    a.mov(r8d, asmjit::imm(size_mmio));
    emit_call_helper(a, mmio_write_helper);

    a.bind(l_done);
}

// emit_ir_amo (9 op AMO_*_W; BARE call walker_helper):
//
// 形态: call mmu_walker_helper_amo_xxx(hart, current_tlb=NULL, gva=rs1, value=rs2)
// 返 旧值 (RMW result before update) in eax → store rd.
//
// BARE walker 内: IS_GPA_RAM(pa) ? amo_xxx_apply(hva) : trap cause 7 (MMIO AMO 不
// 支持; access_helper_call_graph §3 注). backend 不在 inline IS_GPA_RAM 因 9 op
// × 18 段重复 emit 不如统一 walker.
void emit_ir_amo(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    /* SysV ABI: hart=rdi, current_tlb=rsi, gva=rdx, value=rcx */
    emit_load_rv_reg(a, edx, inst.rs1);   /* gva */
    emit_load_rv_reg(a, ecx, inst.rs2);   /* value */
    emit_reload_hart(a);                  /* rdi = hart */
    a.xor_(esi, esi);                     /* current_tlb = NULL (BARE) */

    switch (inst.kind) {
        case IR_OP_AMO_ADD_W:   emit_call_helper(a, mmu_walker_helper_amo_add_w);  break;
        case IR_OP_AMO_SWAP_W:  emit_call_helper(a, mmu_walker_helper_amo_swap_w); break;
        case IR_OP_AMO_XOR_W:   emit_call_helper(a, mmu_walker_helper_amo_xor_w);  break;
        case IR_OP_AMO_OR_W:    emit_call_helper(a, mmu_walker_helper_amo_or_w);   break;
        case IR_OP_AMO_AND_W:   emit_call_helper(a, mmu_walker_helper_amo_and_w);  break;
        case IR_OP_AMO_MIN_W:   emit_call_helper(a, mmu_walker_helper_amo_min_w);  break;
        case IR_OP_AMO_MAX_W:   emit_call_helper(a, mmu_walker_helper_amo_max_w);  break;
        case IR_OP_AMO_MINU_W:  emit_call_helper(a, mmu_walker_helper_amo_minu_w); break;
        case IR_OP_AMO_MAXU_W:  emit_call_helper(a, mmu_walker_helper_amo_maxu_w); break;
        default: __builtin_unreachable();
    }
    /* 返 旧值 (RMW result before update) in eax */
    emit_store_rv_reg(a, inst.rd, eax);
}

// emit_ir_lr_sc (2 op LR_W/SC_W; BARE call walker_helper):
//
// LR_W: call mmu_walker_helper_lr_w(hart, NULL, gva); 返 *hva in eax → store rd
// SC_W: call mmu_walker_helper_sc_w(hart, NULL, gva, value); 返 0=success/1=fail in eax → store rd
//
// BARE walker 内: walker → lrsc_lr_w/sc_w (apply 层, PA-based; access_helper_call_graph
// §4-§5). LR/SC 落 MMIO walker 内 trap cause 5/7.
void emit_ir_lr_sc(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    switch (inst.kind) {
        case IR_OP_LR_W:
            /* call mmu_walker_helper_lr_w(hart=rdi, tlb=rsi=NULL, gva=rdx) */
            emit_load_rv_reg(a, edx, inst.rs1);
            emit_reload_hart(a);
            a.xor_(esi, esi);
            emit_call_helper(a, mmu_walker_helper_lr_w);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        case IR_OP_SC_W:
            /* call mmu_walker_helper_sc_w(hart=rdi, tlb=rsi=NULL, gva=rdx, value=rcx) */
            emit_load_rv_reg(a, edx, inst.rs1);
            emit_load_rv_reg(a, ecx, inst.rs2);
            emit_reload_hart(a);
            a.xor_(esi, esi);
            emit_call_helper(a, mmu_walker_helper_sc_w);
            emit_store_rv_reg(a, inst.rd, eax);
            return;
        default: __builtin_unreachable();
    }
}

// emit_ir_csr (6 op CSRRW/RS/RC + RWI/RSI/RCI; call csr_op):
//
// csr_op(hart, csr_addr, new_val, op, raw_inst):
//   hart=rdi, csr_addr=esi (u32), new_val=edx (uxlen_t), op=ecx (csr_op_t enum),
//   raw_inst=r8d (u32_t)
//
// RW/RS/RC 变体: new_val = READ_REG(rs1) (寄存器值)
// RWI/RSI/RCI 变体: new_val = (uint32_t)d.rs1 (5-bit zimm, ir.rs1 字段直接装数值)
//
// 硬边界: emit 完后 translator 强制 break, 接下来一条 IR_OP_DISPATCH_EXIT.
void emit_ir_csr(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    using namespace asmjit::x86;

    /* new_val 来源: RW/RS/RC 从 reg, RWI/RSI/RCI 直接 imm */
    switch (inst.kind) {
        case IR_OP_CSRRW: case IR_OP_CSRRS: case IR_OP_CSRRC:
            emit_load_rv_reg(a, edx, inst.rs1);   /* new_val = READ_REG(rs1) */
            break;
        case IR_OP_CSRRWI: case IR_OP_CSRRSI: case IR_OP_CSRRCI:
            a.mov(edx, asmjit::imm(inst.rs1));    /* new_val = (uint32_t)rs1 (5-bit zimm) */
            break;
        default: __builtin_unreachable();
    }

    emit_reload_hart(a);                          /* rdi = hart */
    a.mov(esi, asmjit::imm(inst.imm));            /* csr_addr (12-bit) */

    /* op enum 选 */
    csr_op_t op_enum = CSR_OP_RW;
    switch (inst.kind) {
        case IR_OP_CSRRW:  case IR_OP_CSRRWI: op_enum = CSR_OP_RW; break;
        case IR_OP_CSRRS:  case IR_OP_CSRRSI: op_enum = CSR_OP_RS; break;
        case IR_OP_CSRRC:  case IR_OP_CSRRCI: op_enum = CSR_OP_RC; break;
        default: __builtin_unreachable();
    }
    a.mov(ecx, asmjit::imm(static_cast<int>(op_enum)));
    a.mov(r8d, asmjit::imm(static_cast<int32_t>(inst.raw_inst)));

    emit_call_helper(a, csr_op);
    /* 返 read_old in eax → store rd (rd=x0 时 emit_store_rv_reg 走 skip) */
    emit_store_rv_reg(a, inst.rd, eax);
}

// emit_ir_fence (2 op FENCE/FENCE_I):
//
// FENCE   : call fence_helper(hart) (BARE host strong order 退化 ~nop; lrsc 无关)
// FENCE_I : call fence_i_helper(hart) (含 lrsc_clear_self; 硬边界, 后续 DISPATCH_EXIT)
//
// 无 reg 参数; 无 rd.
void emit_ir_fence(asmjit::x86::Assembler &a, const ir_inst_t &inst) {
    emit_reload_hart(a);                          /* rdi = hart */
    switch (inst.kind) {
        case IR_OP_FENCE:    emit_call_helper(a, fence_helper);   return;
        case IR_OP_FENCE_I:  emit_call_helper(a, fence_i_helper); return;
        default: __builtin_unreachable();
    }
}

// emit DISPATCH_EXIT (块出口模板):
//   reload rdi (hart) + rdx (count_out) — 块体内 helper call 可能破坏 (T3)
//   写 cpu->regs[0] = target_pc (regs[0] 物理位置存 pc; uxlen_t = uint32_t for RV32)
//   写 *count_out = block_inst_count (块前缀 RV 指令数; rdx = uint64_t *)
//   后续 epilogue 紧接在 caller 处 emit (假设 rdi 仍是 hart, 因本函数末已 reload)
void emit_dispatch_exit(asmjit::x86::Assembler &a, uxlen_t target_pc,
                        uint64_t block_inst_count) {
    // T3: reload rdi + rdx 从 saved 栈 slot (T1+T2 不需要 reload 也无害,
    // 统一形态简洁; helper call 没有 - reload 一次 ~2 cycle 几乎免费)
    emit_reload_hart(a);
    a.mov(asmjit::x86::rdx,
          asmjit::x86::qword_ptr(asmjit::x86::rbp, STACK_SLOT_COUNT_OUT));

    // 写 cpu->regs[0] = target_pc; mov dword ptr [rdi + 0], imm32
    a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(0)),
          asmjit::imm(static_cast<int32_t>(target_pc)));

    // 写 *count_out = block_inst_count; mov qword ptr [rdx], imm
    // x86 没有 mov m64, imm64 直接形式 — 用 mov rax, imm64 + mov qword ptr [rdx], rax
    a.mov(asmjit::x86::rax, asmjit::imm(block_inst_count));
    a.mov(asmjit::x86::qword_ptr(asmjit::x86::rdx), asmjit::x86::rax);
}

}  // anonymous namespace


extern "C" {

// ============================================================================
// 5 vtable fn (file-static; backend_t 实例填这 5 个 fn pointer)
// ============================================================================

static int asmjit_backend_init(void) {
    if (g_rt != nullptr) {
        fprintf(stderr, "[asmjit] backend_init: g_rt already initialized\n");
        return -1;
    }
    g_rt = new (std::nothrow) asmjit::JitRuntime();
    if (g_rt == nullptr) {
        fprintf(stderr, "[asmjit] backend_init: alloc JitRuntime failed\n");
        return -1;
    }
    return 0;
}

static jit_status_t asmjit_backend_compile_block(uxlen_t pa, regime_t regime,
                                                 const ir_inst_t *insts,
                                                 size_t n_insts,
                                                 void **host_code_out) {
    (void)pa;

    // T3 regime 分流: BARE 真编译; SV32_S/_U 返 NOT_IMPLEMENTED → jit_entry.cc
    // set_blacklist (BLACK) 路径 → dispatcher 下次 lookup interpret 兜底. SV32
    // fast path (TLB tag compare + baked PTE_U + SUM/MXR 运行时读 mstatus) 推到
    // 后续 milestone (b_03+ 上 SMC + 真 SV32 OS guest 时一起做), 跟 plan §1.1 /
    // mmu.h:79 line "消除运行时 priv 分支" 一致.
    if (regime != REGIME_BARE) {
        *host_code_out = nullptr;
        return JIT_ERR_NOT_IMPLEMENTED;
    }

    // 块前缀空 (仅 DISPATCH_EXIT 收尾, n_insts == 1) → 不真编译, NOT_IMPLEMENTED
    // 让 jit_entry.cc step 3 set_blacklist + interpret 兜底真执行那条 RV
    if (n_insts <= 1) {
        *host_code_out = nullptr;
        return JIT_ERR_NOT_IMPLEMENTED;
    }
    if (g_rt == nullptr) {
        *host_code_out = nullptr;
        return JIT_BACKEND_INTERNAL;
    }

    asmjit::CodeHolder code;
    asmjit::Error err = code.init(g_rt->environment());
    if (err != asmjit::kErrorOk) {
        *host_code_out = nullptr;
        return JIT_BACKEND_INTERNAL;
    }

    asmjit::x86::Assembler a(&code);

    emit_prologue(a);

    // 块体: 前 n_insts-1 条 (T1+T2+T3 48 op), 末 1 条 DISPATCH_EXIT 单独处理
    for (size_t i = 0; i < n_insts - 1; i++) {
        const ir_inst_t &inst = insts[i];
        switch (inst.kind) {
            /* RV32I 算术全集 21 op → emit_ir_arith */
            case IR_OP_LUI:  case IR_OP_AUIPC:
            case IR_OP_ADDI: case IR_OP_SLTI: case IR_OP_SLTIU:
            case IR_OP_XORI: case IR_OP_ORI:  case IR_OP_ANDI:
            case IR_OP_SLLI: case IR_OP_SRLI: case IR_OP_SRAI:
            case IR_OP_ADD:  case IR_OP_SUB:  case IR_OP_SLL:
            case IR_OP_SLT:  case IR_OP_SLTU: case IR_OP_XOR:
            case IR_OP_SRL:  case IR_OP_SRA:  case IR_OP_OR:
            case IR_OP_AND:
                emit_ir_arith(a, inst);
                break;

            /* T3 LOAD 5 op → emit_ir_load (BARE inline IS_GPA_RAM) */
            case IR_OP_LB:  case IR_OP_LH:  case IR_OP_LW:
            case IR_OP_LBU: case IR_OP_LHU:
                emit_ir_load(a, inst);
                break;

            /* T3 STORE 3 op → emit_ir_store (BARE inline IS_GPA_RAM) */
            case IR_OP_SB:  case IR_OP_SH:  case IR_OP_SW:
                emit_ir_store(a, inst);
                break;

            /* T3 CSR 6 op → emit_ir_csr (call csr_op; 硬边界, 下一条必是 DISPATCH_EXIT) */
            case IR_OP_CSRRW:  case IR_OP_CSRRS:  case IR_OP_CSRRC:
            case IR_OP_CSRRWI: case IR_OP_CSRRSI: case IR_OP_CSRRCI:
                emit_ir_csr(a, inst);
                break;

            /* T3 AMO 9 op → emit_ir_amo (BARE call walker_helper) */
            case IR_OP_AMO_ADD_W:  case IR_OP_AMO_SWAP_W: case IR_OP_AMO_XOR_W:
            case IR_OP_AMO_OR_W:   case IR_OP_AMO_AND_W:
            case IR_OP_AMO_MIN_W:  case IR_OP_AMO_MAX_W:
            case IR_OP_AMO_MINU_W: case IR_OP_AMO_MAXU_W:
                emit_ir_amo(a, inst);
                break;

            /* T3 LR/SC 2 op → emit_ir_lr_sc (BARE call walker_helper) */
            case IR_OP_LR_W: case IR_OP_SC_W:
                emit_ir_lr_sc(a, inst);
                break;

            /* T3 FENCE 2 op → emit_ir_fence (FENCE_I 硬边界, 下一条 DISPATCH_EXIT) */
            case IR_OP_FENCE: case IR_OP_FENCE_I:
                emit_ir_fence(a, inst);
                break;

            case IR_OP_DISPATCH_EXIT:
                // 块前缀里出现 DISPATCH_EXIT 是 translator bug
                *host_code_out = nullptr;
                return JIT_BACKEND_INTERNAL;
            case IR_OP_UNSUPPORTED:
                // 哨兵 (-Wswitch-enum 完整性); 永不真撞, 撞 = translator bug
                __builtin_unreachable();
        }
    }

    // 末段 DISPATCH_EXIT + epilogue
    const ir_inst_t &exit_inst = insts[n_insts - 1];
    if (exit_inst.kind != IR_OP_DISPATCH_EXIT) {
        // translator bug: 末段必须是 DISPATCH_EXIT
        *host_code_out = nullptr;
        return JIT_BACKEND_INTERNAL;
    }
    emit_dispatch_exit(a, exit_inst.target_pc,
                       static_cast<uint64_t>(n_insts - 1));
    emit_epilogue(a);

    // finalize + commit to JitRuntime (asmjit 内部 mmap RX + relocate)
    jit_block_func_t fn = nullptr;
    err = g_rt->add(&fn, &code);
    if (err != asmjit::kErrorOk) {
        *host_code_out = nullptr;
        // T1 起步统一 BACKEND_INTERNAL; T5+ 真撞 OOM 时按 err code 细化到
        // CODE_CACHE_FULL (走 jit_entry.cc Q11 a Flush+retry)
        return JIT_BACKEND_INTERNAL;
    }

    *host_code_out = reinterpret_cast<void *>(fn);
    return JIT_OK;
}

static void asmjit_backend_invalidate_block(void *host_code) {
    if (g_rt == nullptr || host_code == nullptr) {
        return;
    }
    g_rt->release(host_code);
}

static void asmjit_backend_flush_all(void) {
    /* asmjit v1.18 JitRuntime 自带 reset() 接口 (默认 ResetPolicy::kSoft —
     * allocator 内部回收 RX 段保留 pool 给下次 add() 复用; kHard 释放所有 OS 内存).
     * 比 delete + new 高效, 不 reallocate. */
    if (g_rt != nullptr) {
        g_rt->reset();
    }
}

static void asmjit_backend_destroy(void) {
    if (g_rt != nullptr) {
        delete g_rt;
        g_rt = nullptr;
    }
}


// ============================================================================
// file-static backend_t 单例 + extern "C" backend_get_default() 暴露
// ============================================================================

static const backend_t asmjit_backend = {
    .backend_init             = asmjit_backend_init,
    .backend_compile_block    = asmjit_backend_compile_block,
    .backend_invalidate_block = asmjit_backend_invalidate_block,
    .backend_flush_all        = asmjit_backend_flush_all,
    .backend_destroy          = asmjit_backend_destroy,
};

const backend_t *backend_get_default(void) {
    return &asmjit_backend;
}

}  // extern "C"
