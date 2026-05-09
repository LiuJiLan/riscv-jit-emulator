# a_01_10 / 07_cross_page
# 验 BARE M-mode 下 interpreter 跨页保护 (interpreter.c 内 page_mask 检测推进后跨页 →
# goto out, dispatcher 重新 mmu_translate_pc 拿新 page hva 进新 block)。
#
# 模板风格: 纯 .S, BARE M-mode, 套 a_01_10/03-06 同形态。
#
# 内存布局 (link.ld . = 0x80000000 + .text section .org pad):
#   0x80000000  _start              入口块 (li + jal 跳到第 1 页末段)
#   0x80000FF8  page_end_block      第 1 页末 8 字节 — 2 条 32-bit ADDI 占 0xFF8/0xFFC,
#                                     第 2 条 0xFFC 是页内最后 4 字节 (RV32 4 字节占
#                                     0xFFC..0xFFF), 跑完推进 hva_pc = 0x80001000 跨页
#                                     → interpreter.c goto out, dispatcher 重派发
#   0x80001000  cross_page          第 2 页起点 — dispatcher 重 mmu_translate_pc 进新
#                                     block, 跑 ADDI x13/x14 + ecall
#
# 期望 dump (M-mode, mtvec=0 默认):
#   x10 = 1   (_start: ADDI; 入口块 li)
#   x11 = 2   (page_end_block 第 1 条 0xFF8 ADDI; 推进后 hva_pc=0xFFC 仍第 1 页, 不退出)
#   x12 = 3   (page_end_block 第 2 条 0xFFC ADDI; 推进后 hva_pc=0x1000 跨页 → goto out)
#   x13 = 4   (cross_page 第 1 条 0x1000 ADDI; 验 dispatcher 真重派发)
#   x14 = 5   (cross_page 第 2 条 0x1004 ADDI)
#   pc  = 0   (ecall trap_set_state 写 regs[0]=mtvec[PRIV_M]=0; 后续 fetch trap 链到 in_trap=3)
#   in_trap = 3
#   total_count = 6  (li + jal + 4 ADDI; ecall 触发 trap, SYNC_COUNT 在 trap_raise 之前
#                     count 还没 ++, ecall 那条不计入)
#
# 调试观察点 (gdb / CLion debug):
#   src/core/interpreter.c 内 `if (((uintptr_t)hva_pc & page_mask) != entry_page) goto out;`
#   下断点 — 本 fixture 应触发 1 次 (page_end_block 第 2 条 ADDI 推进后)。其他 fixture
#   (03-06) 不会触发本断点 (块都在单 page 内)。
#
# 注: jal 是 hard boundary, _start 块跑 li + jal 后立即 goto out (boundary check), 跟
# 跨页软边界无关; 跨页发生在 page_end_block 第 2 条 ADDI 推进后, 这是验跨页保护的核心点。

    .option norvc
    .text
    .global _start

_start:
    addi    x10, x0, 1                   # 0x80000000: x10 = 1
    jal     x0, page_end_block           # 0x80000004: jump to 0x80000FF8

    # _start 块到此结束 (jal hard boundary, count++ 后 goto out)。
    # .org pad 到 section 偏移 0xFF8 — 让 page_end_block 钉在 0x80000FF8。
    .org 0xFF8

page_end_block:
    addi    x11, x0, 2                   # 0x80000FF8: x11 = 2
    addi    x12, x0, 3                   # 0x80000FFC: x12 = 3 (页末 4 字节)
    # 本块跑完后 hva_pc 推进到 0x80001000 → 跨页 → goto out, dispatcher 重派发

cross_page:
    addi    x13, x0, 4                   # 0x80001000: x13 = 4 (第 2 页起点)
    addi    x14, x0, 5                   # 0x80001004: x14 = 5
    ecall                                # 0x80001008: trap → mtvec=0 → fetch trap 链 → in_trap=3
