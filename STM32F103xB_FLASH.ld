/*
******************************************************************************
**  STM32F103C8T6 (64KB Flash / 20KB RAM) 链接脚本
**  16 x ADS1220 / 32 路热电偶测温系统
**
**  GNU ld 语法, 供根目录 Makefile 使用 (Keil 工程不需要这个文件,
**  Keil 用的是 MDK-ARM/Thermocouple.sct, 由 .uvprojx 的 OnChipMemories 自动生成)。
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* 栈顶 = RAM 末尾 (栈向下生长) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;   /* 512 B  堆 (本工程不用 malloc, 留最小量) */
_Min_Stack_Size = 0x800;   /* 2 KB   栈 (与 startup/Keil 保持一致) */

/* Memories definition */
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K    /* STM32F103C8: 20KB SRAM */
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K    /* STM32F103C8: 64KB Flash */
}

/* Sections */
SECTIONS
{
  /* 中断向量表必须放在 Flash 最前面 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 代码 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 常量 / 只读数据 (NIST 多项式系数表在这里) */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* 已初始化变量: 运行在 RAM, 初值存在 Flash */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化变量 */
  . = ALIGN(4);
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 检查堆栈是否放得下 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
