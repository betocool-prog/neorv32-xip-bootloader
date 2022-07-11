# The NEORV32 XIP Bootloader

## Introduction
The XIP Bootloader allows you to write a binary file to flash, read data from flash, erase sectors, check the status byte and execute your program from flash using the XIP (eXecute In Place) component of the NEORV32. The XIP module uses an SPI interface to interface with the flash memory. It is still possible to use the NEORV32's SPI interface for other purposes, as both interfaces are independent from each other. 

The XIP Bootloader consists of two files, bootloader_xip.c and neorv_sh.py. 

[bootloader_xip.c](bootloader_xip.c) is the source code for the bootloader running inside the NEORV32 processor, inside the bootloader memory space.

[neorv_sh.py](neorv_sh.py) is a command line interface which lets you interact with the bootloader.

The commands between the CLI and the bootloader use the UART0 interface.

It is *highly* recommended to implement the i-cache in the FPGA fabric, otherwise the program may run awfully low!

## Usage
### Compiling the bootloader
Running
```bash
make bootloader
```
will generate the XIP bootloader VHDL file in */rtl/neorv32_bootloader_image.vhd*. You'll have to re-build your VHDL project.

The default settings for the XIP bootloader are based on a DE0-Nano Cyclone IV dev board:
- 3 byte addresses
- 8 MBytes size
- Executable start address in flash:  0x400000
- XIP base address 0x20000000

The values above can be updated in the [bootloader_xip.c](bootloader_xip.c) file:
```c
/** SPI flash boot base address */
#ifndef SPI_FLASH_BASE_ADDR
  #define SPI_FLASH_BASE_ADDR 0x00400000
#endif
/**@}*/
```

```c
/** SPI flash address width (in numbers of bytes; 2,3,4) */
#ifndef SPI_FLASH_ADDR_BYTES
  #define SPI_FLASH_ADDR_BYTES 3 // default = 3 address bytes = 24-bit
#endif
```

```c
/** XIP Page Base */
#ifndef XIP_PAGE_BASE
  #define XIP_PAGE_BASE 0x20000000
#endif
```

The timeout for the bootloader is 8 seconds. If the bootloader times out, it starts execution from the flash address at `SPI_FLASH_BASE_ADDR`.

### Compiling a firmware application
For a compiled application to run, the linker script located in *sw/common/neorv32.ld* needs to be updated. The line which defines the ROM address must have the new base address:
```c
  rom   (rx) : ORIGIN = DEFINED(make_bootloader) ? 0xFFFF0000 : 0x20400000, LENGTH = DEFINED(make_bootloader) ? 32K : 4M
```

### Running the command line interface, neorv_sh.py

The command line interface (CLI) requires Python 3.x to run, as it uses the CMD module. It is based on a simple request - response communication mode. The CLI sends a 1-byte command requesting a write or a read, waits for the response, and either sends the rest of the data or processes the information received.

Run the script using `python neorv_sh.py`.

At bootloader start, you have 8 seconds to *poke* the bootloader and prevent it from jumping to the application. There are two ways of doing this:
- Reset the processor. It starts in bootloader mode. Start the CLI script. It always sends a *poke* command at startup.
- If the CLI is already running, reset the processor and execute the *poke* command within 8 seconds.

The implemented commands are:

- help: Shows help information
- demo: Writes at the flash base address the 64-byte blinky demo. Default base address is 0x400000.
- erase_sector *address*: Erases a flash sector by address. Default sector size is 64KBytes.
- execute: Executes the program from flash stored at the base address using the XIP module.
- flash_read *address* *length*: Dumps the data at the specified address, for the specified length, on screen.
- flash_write *address* *path/to/file.bin*: Writes the contents of file.bin at the specified address.
- poke: Send a '0' character and expect a '0' back. Used to prevent the bootloader from timing out at the start or to check if it's still running.
- reset: Resets the bootloader
- status_byte: Returns the value of the flash memory's status byte

## FPGA implementations

### Cyclone IV DE0 Nano
The [de0_nano](de0_nano) folder contains all files and configurations necessary to open a Quartus Project. This has been created with Quartus 21.1.