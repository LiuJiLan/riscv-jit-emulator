/* a03_8 / 01_m_ext_mul linker script
 * .text @ 0x80000000 = GUEST_RAM_START. 跟项目所有 raw-bin fixture 一致。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
