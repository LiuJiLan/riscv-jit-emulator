/* a02_3 / 02_s_mret_reject linker script
 * 把 .text 钉在 0x80000000 (GUEST_RAM_START)。
 * 与 a02_3/01_u_priv_reject 同形态 (fixture 自包含原则)。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
