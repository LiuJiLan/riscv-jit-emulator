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
// T1+T2 实状
// ============================================================================
//
// 5 vtable 全真填:
//   asmjit_backend_init             alloc file-static asmjit::JitRuntime
//   asmjit_backend_compile_block    CodeHolder + Assembler emit (prologue + 块体
//                                    + DISPATCH_EXIT + epilogue) → JitRuntime::add
//                                    拿 RX 段地址, 返 jit_block_func_t
//   asmjit_backend_invalidate_block JitRuntime::release(host_code) (Q8 配合; backend
//                                    纯 unmap host code; 状态灯扫在 jit_entry.cc)
//   asmjit_backend_flush_all        JitRuntime::reset() (asmjit v1.18+ 自带 reset)
//   asmjit_backend_destroy          delete JitRuntime
//
// 块体 IR 翻译 (T1+T2 范围): RV32I 算术全集 21 op (U-type 2 + I-imm 9 + R-type 10)
// + IR_OP_DISPATCH_EXIT; IR_OP_UNSUPPORTED 哨兵 default case 写
// __builtin_unreachable 兜底.
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
// caller-saved: rax, rcx, rdx, rsi, rdi, r8-r11 (rax/rcx 可作 scratch; T1 不调
//   helper 不撞 caller-saved)
//
// 栈对齐: T1 c+ 不调 helper (helper call 推 T3), prologue push 6 个 callee-saved
// 后 rsp % 16 == 8; 不严格对齐 SysV 16-byte (跟内部 ret 无关; T3 真调 helper
// 时配套加 sub rsp, 8 对齐).
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
#include "ir.h"             // ir_inst_t + IR_OP_*


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

// 把 RV reg 值 load 到 host scratch reg (32-bit Gpd):
//   rv_reg == 0 → xor scratch, scratch (常数 0; dummy.txt §2 read x0 = 0)
//   1..5        → mov scratch, host_reg_for_rv(rv_reg) (固定 host reg 直读)
//   6..31       → mov scratch, [rdi + regs_offset(rv_reg)] (cpu->regs[r] 读)
void emit_load_rv_reg(asmjit::x86::Assembler &a, asmjit::x86::Gp scratch,
                      uint8_t rv_reg) {
    if (rv_reg == 0) {
        a.xor_(scratch, scratch);
    } else if (rv_reg <= 5) {
        a.mov(scratch, host_reg_for_rv(rv_reg));
    } else {
        a.mov(scratch,
              asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(rv_reg)));
    }
}

// 把 host scratch reg (32-bit) 写入 RV reg:
//   rv_reg == 0 → skip (dead store; dummy.txt §2 write x0 走 garbage)
//   1..5        → mov host_reg_for_rv(rv_reg), scratch
//   6..31       → mov [rdi + regs_offset(rv_reg)], scratch
void emit_store_rv_reg(asmjit::x86::Assembler &a, uint8_t rv_reg,
                       asmjit::x86::Gp scratch) {
    if (rv_reg == 0) {
        return;
    }
    if (rv_reg <= 5) {
        a.mov(host_reg_for_rv(rv_reg), scratch);
    } else {
        a.mov(asmjit::x86::dword_ptr(asmjit::x86::rdi, regs_offset(rv_reg)),
              scratch);
    }
}

// prologue: push callee-saved (rbp + rbx + r12-r15) + load x1-x5 from cpu->regs[1..5]
void emit_prologue(asmjit::x86::Assembler &a) {
    a.push(asmjit::x86::rbp);
    a.mov(asmjit::x86::rbp, asmjit::x86::rsp);
    a.push(asmjit::x86::rbx);
    a.push(asmjit::x86::r12);
    a.push(asmjit::x86::r13);
    a.push(asmjit::x86::r14);
    a.push(asmjit::x86::r15);

    // load x1-x5 from cpu->regs[1..5]
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

// epilogue: store x1-x5 to cpu->regs[1..5] + pop callee-saved + ret
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

        /* ---- 哨兵 (emit_ir_arith 不该看到; caller 路径已分流) ---- */
        case IR_OP_UNSUPPORTED:
        case IR_OP_DISPATCH_EXIT:
            __builtin_unreachable();
    }
}

// emit DISPATCH_EXIT (块出口模板):
//   写 cpu->regs[0] = target_pc (regs[0] 物理位置存 pc; uxlen_t = uint32_t for RV32)
//   写 *count_out = block_inst_count (块前缀 RV 指令数; rdx = uint64_t *)
//   后续 epilogue 紧接在 caller 处 emit (store x1-x5 + pop + ret)
void emit_dispatch_exit(asmjit::x86::Assembler &a, uxlen_t target_pc,
                        uint64_t block_inst_count) {
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
    (void)regime;

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

    // 块体: 前 n_insts-1 条 (RV32I 算术全集 21 op), 末 1 条 DISPATCH_EXIT 单独处理
    for (size_t i = 0; i < n_insts - 1; i++) {
        const ir_inst_t &inst = insts[i];
        switch (inst.kind) {
            /* RV32I 算术全集 21 op → 共用 emit_ir_arith 路径 */
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
