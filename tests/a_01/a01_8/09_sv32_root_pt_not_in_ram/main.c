// a_01_8 / 09_sv32_root_pt_not_in_ram — setup_pt() (空)
//
// 跟 04-08 模板一致, 但 fixture 09 不分配 PT (satp.PPN 故意指 RAM 外, root_pt 不存在
// → walker level=1 PT 读检查直接 fail)。setup_pt 函数体空, 仅保留模板形态。
//
// 详见 stub.S 顶部 doc。

void setup_pt(void) {
    // 不分配任何 PT — fixture 09 故意让 satp.PPN 指 RAM 外, walker 走不到
    // root_pt 真位置就触发 RAM 检查 fail (mmu.c walker level=1 段, L283)。
}
