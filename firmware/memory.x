MEMORY {
    /* RP2350 has 4 MB external flash via QSPI */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K

    /* Main SRAM banks */
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K

    /* Per-core dedicated SRAM banks */
    SRAM4 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM5 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /* RP2350 image definition block must be at the start of flash */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

_stext = ORIGIN(FLASH) + 0x140;
