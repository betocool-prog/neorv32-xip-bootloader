write_cfgmem  -format mcs -size 16 -interface SPIx4 \
    -loadbit {up 0x00000000 "/home/betocool/Projects/neorv32-xip-bootloader/arty_a7_100/arty_a7_100.runs/impl_1/neorv32_xip_bootloader.bit" } \
    -force -file "/home/betocool/Projects/neorv32-xip-bootloader/arty_a7_100/arty_a7_100.runs/impl_1/neorv32_xip_bootloader"
