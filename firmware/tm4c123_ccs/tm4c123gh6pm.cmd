--retain=g_pfnVectors

MEMORY
{
    FLASH (RX)  : origin = 0x00000000, length = 0x00040000
    SRAM  (RWX) : origin = 0x20000000, length = 0x00008000
}

SECTIONS
{
    .intvecs    : > 0x00000000
    .text       : > FLASH
    .const      : > FLASH
    .cinit      : > FLASH
    .pinit      : > FLASH
    .init_array : > FLASH

    .vtable     : > 0x20000000
    .data       : > SRAM
    .bss        : > SRAM
    .sysmem     : > SRAM
    .stack      : > SRAM
}

__STACK_TOP = __stack + 1024;
