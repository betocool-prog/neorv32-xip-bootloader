// #################################################################################################
// # << NEORV32 - XIP Bootloader >>                                                                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Alberto Fahrenkrog. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file bootloader_xip.c
 * @author Alberto Fahrenkrog
 * @brief NEORV32 XIP Bootloader
 **************************************************************************/

// Libraries
#include <stdint.h>
#include <neorv32.h>


/**********************************************************************//**
 * @name Bootloader configuration (override via console to customize)
 * default values are used if not explicitly customized
 **************************************************************************/
/**@{*/

/* ---- UART interface configuration ---- */

/** Set to 0 to disable UART interface */
#ifndef UART_EN
  #define UART_EN 1
#endif

/** UART BAUD rate for serial interface */
#ifndef UART_BAUD
  #define UART_BAUD 115200
#endif

/* ---- Status LED ---- */

/** Set to 0 to disable bootloader status LED (heart beat) at GPIO.gpio_o(STATUS_LED_PIN) */
#ifndef STATUS_LED_EN
  #define STATUS_LED_EN 1
#endif

/** GPIO output pin for high-active bootloader status LED (heart beat) */
#ifndef STATUS_LED_PIN
  #define STATUS_LED_PIN 0
#endif

/* ---- Boot configuration ---- */

/** Set to 1 to enable automatic (after reset) only boot from external SPI flash at address SPI_BOOT_BASE_ADDR */
#ifndef AUTO_BOOT_SPI_EN
  #define AUTO_BOOT_SPI_EN 0
#endif

/** Set to 1 to enable boot only via on-chip debugger (keep CPU in halt loop until OCD takes over control) */
#ifndef AUTO_BOOT_OCD_EN
  #define AUTO_BOOT_OCD_EN 0
#endif

/** Set to 1 to enable simple UART executable upload (no console, no SPI flash) */
#ifndef AUTO_BOOT_SIMPLE_UART_EN
  #define AUTO_BOOT_SIMPLE_UART_EN 0
#endif

/** Time until the auto-boot sequence starts (in seconds); 0 = disabled */
#ifndef AUTO_BOOT_TIMEOUT
  #define AUTO_BOOT_TIMEOUT 8
#endif

/* ---- XIP configuration ---- */

/** Enable XIP module (default) for flash boot options */
#ifndef XIP_EN
  #define XIP_EN 1
#endif

/** SPI flash address width (in numbers of bytes; 2,3,4) */
#ifndef SPI_FLASH_ADDR_BYTES
  #define SPI_FLASH_ADDR_BYTES 3 // default = 3 address bytes = 24-bit
#endif

/** SPI flash sector size in bytes */
#ifndef SPI_FLASH_SECTOR_SIZE
  #define SPI_FLASH_SECTOR_SIZE 65536 // default = 64kB
#endif

/** SPI flash boot base address */
#ifndef SPI_FLASH_BASE_ADDR
  #define SPI_FLASH_BASE_ADDR 0x00400000
#endif

/** XIP Page Base */
#ifndef XIP_PAGE_BASE
  #define XIP_PAGE_BASE 0x20000000
#endif
/**@}*/


/**********************************************************************//**
 * Error codes
 **************************************************************************/
enum ERROR_CODES {
  ERROR_SIGNATURE = 0x01, /**< 1: Wrong signature in executable */
  ERROR_SIZE      = 0x02, /**< 2: Insufficient instruction memory capacity */
  ERROR_CHECKSUM  = 0x04, /**< 4: Checksum error in executable */
  ERROR_FLASH     = 0x08, /**< 8: SPI flash access error */
  ERROR_XIP_SETUP = 0x10, /**< 16: XIP Setup error */
  ERROR_XIP_XFER  = 0x20, /**< 32: XIP transfer error */
  ERROR_FLASH_WR  = 0x40, /**< 64: Flash write error */
  ERROR_TRAP      = 0x80, /**< 128: Other error */
};


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD {
  SPI_FLASH_CMD_WRITE_STATUS  = 0x01, /**< Write Status */
  SPI_FLASH_CMD_WRITE_BYTES   = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /**< Disallow write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_READ_ID       = 0x16,  /**< Read silicon ID */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * SPI flash status register bits
 **************************************************************************/
enum SPI_FLASH_SREG {
  FLASH_SREG_BUSY = 0, /**< Busy, write/erase in progress when set, read-only */
  FLASH_SREG_WEL  = 1  /**< Write access enabled when set, read-only */
};


/**********************************************************************//**
 * NEORV32 executable
 **************************************************************************/
enum NEORV32_EXECUTABLE {
  EXE_OFFSET_SIGNATURE =  0, /**< Offset in bytes from start to signature (32-bit) */
  EXE_OFFSET_SIZE      =  4, /**< Offset in bytes from start to size (32-bit) */
  EXE_OFFSET_CHECKSUM  =  8, /**< Offset in bytes from start to checksum (32-bit) */
  EXE_OFFSET_DATA      = 12, /**< Offset in bytes from start to data (32-bit) */
};


/**********************************************************************//**
 * Bootloader Console commands
 **************************************************************************/
enum CONSOLE_CMDS {
    POKE            = 0x00, /** Just a simple poke to check if bootloader is on **/
    EXECUTE         = 0x01, /** Executes application from SPI Flash using XIP **/
    FLASH_WRITE     = 0x02, /** Loads new executable to flash **/
    FLASH_READ      = 0x03, /** Reads the executable from flash and sends it to the console **/
    RESET           = 0x04, /** Resets the bootloader **/
    ERASE_SECTOR    = 0x06, /** Flash Erase Sector Command **/
    STATUS_BYTE     = 0x07, /** Read Status Byte Command **/
    DEMO            = 0x08, /** Flash the demo program at 0x400000 **/
};

// SPI Flash data
union {
  uint64_t uint64;
  uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
} data;

uint32_t addr = 0;
uint32_t len = 0;
char data_page[256] = {0};

/**********************************************************************//**
 * @name Simple program to be stored to the XIP flash.
 * This is the "blink_led_asm" from the rv32i-version "blink_led" demo program.
 **************************************************************************/
const uint32_t xip_program[] = 
{
  0xfc800513,
  0x00052023,
  0x00000313,
  0x0ff37313,
  0x00652023,
  0x00130313,
  0x008000ef,
  0xff1ff06f,
  0x001003b7,
  0xfff38393,
  0x00038a63,
  0xfff38393,
  0x00000013,
  0x00000013,
  0xff1ff06f,
  0x00008067
};

// Function prototypes
void __attribute__((__interrupt__)) bootloader_trap_handler(void);
void start_app(void);
void get_exe(int src);
void save_exe(void);
uint32_t get_exe_word(int src, uint32_t addr);
void system_error(uint8_t err_code);

// Flash function prototypes 
void flash_read(uint32_t addr, uint32_t len);
void flash_write(uint32_t addr, uint32_t len, char* data_ptr);
void flash_erase_sector(uint32_t addr);
void flash_read_status(void);


/**********************************************************************//**
 * Sanity check: Base ISA only!
 **************************************************************************/
#if defined __riscv_atomic || defined __riscv_a || __riscv_b || __riscv_compressed || defined __riscv_c || defined __riscv_mul || defined __riscv_m
  #warning In order to allow the bootloader to run on *any* CPU configuration it should be compiled using the base ISA only.
#endif


/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void)
{
  char console_cmd = 0;
  uint32_t aux = 0;
  uint32_t retval = 0;

#if (XIP_EN != 0)
  retval = neorv32_xip_setup(CLK_PRSC_2, 0, 0, 0x03);
#else
  #error In order to use the XIP bootloader, the XIP module must be implemented
#endif
  if(retval)
  {
    // Wrong configuration
    system_error(0xFF & ERROR_XIP_SETUP);
  }

#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    // activate status LED, clear all others
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

#if (UART_EN != 0)
  // setup UART0 (primary UART, no parity bit, no hardware flow control)
  neorv32_uart0_setup(UART_BAUD, PARITY_NONE, FLOW_CONTROL_NONE);
#endif

  // Configure machine system timer interrupt
  if (neorv32_mtime_available()) {
    neorv32_mtime_set_timecmp(0 + (NEORV32_SYSINFO.CLK/4));
    // active timer IRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE); // activate MTIME IRQ source only!
  }

  // configure trap handler (bare-metal, no neorv32 rte available) 
  // after all peripherals are OK!
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&bootloader_trap_handler));
  neorv32_cpu_eint(); // enable global interrupts

  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------
#if (XIP_EN != 0)
#if (AUTO_BOOT_TIMEOUT != 0)
  if (neorv32_mtime_available())
  {
    uint64_t timeout_time = neorv32_mtime_get_time() + (uint64_t)(AUTO_BOOT_TIMEOUT * NEORV32_SYSINFO.CLK);

    while(retval == 0)
    {
      if (neorv32_uart0_available())
      { 
        // wait for a poke...
        if (neorv32_uart0_char_received())
        {
          neorv32_uart0_putc(0x00);
          retval = 1;
        }
      }

      if(retval == 0)
      {
        if (neorv32_mtime_get_time() >= timeout_time) 
        { // timeout? start auto boot sequence
          // configure and enable the actual XIP mode
          // * configure 3 address bytes send to the SPI flash for addressing
          // * map the XIP flash to the address space starting at XIP_PAGE_BASE
          if (neorv32_xip_start(SPI_FLASH_ADDR_BYTES, XIP_PAGE_BASE))
          {
            retval = 1;
            system_error(0xFF & ERROR_XIP_SETUP);
          }

          if(retval == 0)
          {
            // finally, jump to the XIP flash's base address we have configured to start execution **from there**
            asm volatile ("call %[dest]" : : [dest] "i" (XIP_PAGE_BASE + SPI_FLASH_BASE_ADDR));
            while(1);
          }
        }
      }
    }
  }
#else
  PRINT_TEXT("Aborted.\n\n");
#endif
#else
  PRINT_TEXT("\n\n");
#endif


  // ------------------------------------------------
  // Main loop
  // ------------------------------------------------
  while (1)
  {

    console_cmd = neorv32_uart0_getc();

    if(console_cmd == RESET)
    {
      neorv32_uart0_putc(console_cmd);
      while (neorv32_uart0_tx_busy()); /** Wait for transmission to end **/
      
      asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (BOOTLOADER_BASE_ADDRESS)); // jump to beginning of boot ROM
    }
    else if (console_cmd == POKE)
    {
      neorv32_uart0_putc(console_cmd);
    }
    else if (console_cmd == EXECUTE)
    {
      neorv32_uart0_putc(console_cmd);
      // configure and enable the actual XIP mode
      // * configure 3 address bytes send to the SPI flash for addressing
      // * map the XIP flash to the address space starting at XIP_PAGE_BASE
      retval = 0;
      if (neorv32_xip_start(SPI_FLASH_ADDR_BYTES, XIP_PAGE_BASE))
      {
        system_error(0xFF & ERROR_XIP_SETUP);
      }

      neorv32_gpio_port_set(0x0);
      // finally, jump to the XIP flash's base address we have configured to start execution **from there**
      asm volatile ("call %[dest]" : : [dest] "i" (XIP_PAGE_BASE + SPI_FLASH_BASE_ADDR));
      while(1);
    }
    else if (console_cmd == FLASH_WRITE)
    {
      /* This only writes max. 256 bytes at a time! */
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }

      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&len)[aux] = neorv32_uart0_getc();
      }
      
      for(aux = 0; aux < len; aux++)
      {
        data_page[aux] = neorv32_uart0_getc();
        neorv32_gpio_port_set(data_page[aux] & 0x7F); // Visual aid. Completely optional
      }

      flash_write(addr, aux, data_page);
      neorv32_uart0_putc(0x00);

    }
    else if (console_cmd == FLASH_READ)
    {
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }

      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&len)[aux] = neorv32_uart0_getc();
      }
      flash_read(addr, len);
    }
    else if (console_cmd == ERASE_SECTOR)
    {
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }
      flash_erase_sector(addr);
      neorv32_uart0_putc(console_cmd);
    }
    else if (console_cmd == DEMO)
    {
      flash_erase_sector(SPI_FLASH_BASE_ADDR);
      flash_write(SPI_FLASH_BASE_ADDR, 64, (char*)xip_program);
      neorv32_uart0_putc(DEMO);
    }
    else if (console_cmd == STATUS_BYTE)
    {
      flash_read_status();
    }
  }

  return 1; // bootloader should never return
}


/**********************************************************************//**
 * Erase flash sector by address (24 bit)
 **************************************************************************/
void flash_erase_sector(uint32_t addr)
{
  data.uint64 = 0; // Init to zero before any operation
  
  // set status bits to 000
  // 1 byte command
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  // data.uint32[1] = 0; // command: set write-enable latch
  if(0 != neorv32_xip_spi_trans(1, &data.uint64))
  {
    system_error(0xFF & ERROR_FLASH);
  };
    
  // set write-enable latch
  // 1 byte command
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_WRITE_ENABLE << 24) & 0xFF000000; // command: set write-enable latch
  if(0 != neorv32_xip_spi_trans(1, &data.uint64))
  {
    system_error(0xFF & ERROR_FLASH_WR);
  };

  data.uint64 = 0; 
  // erase sector
  // 1 byte command + 3 byte address
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_SECTOR_ERASE << 24) & 0xFF000000; // command: erase sector
  data.uint32[1] |= addr & 0x00FFFFFF; // address data
  if(0 != neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 1, &data.uint64))
  {
    system_error(0xFF & ERROR_FLASH_WR);
  };

  data.uint64 = 0; 
  // check status register: WIP bit has to clear
  while(1) {
    // data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
    if(0 != neorv32_xip_spi_trans(2, &data.uint64))
    {
      system_error(0xFF & ERROR_FLASH_WR);
    };
    if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
      break;
    }
  }
}


/**********************************************************************//**
 * Read content from flash and send it to client
 **************************************************************************/
void flash_read(uint32_t addr, uint32_t len)
{
  uint32_t tmp = 0;
  uint32_t cnt = 0;
  uint32_t aux = 0;

  cnt = 0;
  aux = addr;
  while(cnt < len)
  {
    // read word
    // 1 byte command, 3 bytes address, 1 byte data
    tmp = SPI_FLASH_CMD_READ << 24; // command: byte read
    tmp |= (aux & 0x00FFFFFF); // address
    data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = tmp;
    if(0 != neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 2, &data.uint64))
    {
      system_error(0xFF & ERROR_FLASH);
    };
    aux++;
    cnt++;

    neorv32_uart0_putc(data.uint32[0] & 0xFF);
  }
}

/**********************************************************************//**
 * Write data to flash
 **************************************************************************/
void flash_write(uint32_t addr, uint32_t len, char* data_ptr)
{
  uint32_t idx;
  uint32_t data_idx = 0;
  uint32_t tmp = 0;
  char* tmp_ptr = 0;

  tmp_ptr = (char *)&data.uint32[0];

  data_idx = 0;
  while(data_idx < len)
  {
    // Set Enable Write 
    data.uint32[1] = (SPI_FLASH_CMD_WRITE_ENABLE << 24) & 0xFF000000;
    if(0 != neorv32_xip_spi_trans(1, &data.uint64))
    {
      system_error(0xFF & ERROR_FLASH_WR);
    };
    // write word
    // 1 byte command, 3 bytes address, 4 bytes data
    tmp = SPI_FLASH_CMD_WRITE_BYTES << 24; // command: byte read
    tmp |= ((addr + data_idx) & 0x00FFFFFF); // address

    for(idx = 0; idx < 4; idx++)
    {
      if(data_idx < len)
      {
        tmp_ptr[3 - idx] = data_ptr[data_idx];
        data_idx++;
      }
    }
    data.uint32[1] = tmp;
    if(0 != neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 5, &data.uint64))
    {
      system_error(0xFF & ERROR_FLASH_WR);
    };

    // check status register: WIP bit has to clear
    while(1)
    {
      // data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
      data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
      if(0 != neorv32_xip_spi_trans(2, &data.uint64))
      {
        system_error(0xFF & ERROR_FLASH);
      };
      if ((data.uint32[0] & 0x01) == 0)
      { // WIP bit cleared?
        break;
      }
    }
  }
}


/**********************************************************************//**
 * Read status byte from flash
 **************************************************************************/
void flash_read_status(void)
{
  // read flash status byte
  // 1 byte command, 1 byte data
  data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
  if(0 != neorv32_xip_spi_trans(3, &data.uint64))
  {
    system_error(0xFF & ERROR_FLASH);
  };

  neorv32_uart0_putc(data.uint32[0] & 0xFF);  
}

/**********************************************************************//**
 * Output system error ID and stall.
 *
 * @param[in] err_code Error code. See #ERROR_CODES and #error_message.
 **************************************************************************/
void system_error(uint8_t err_code) {

  neorv32_cpu_dint(); // deactivate IRQs
#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(0xFF & err_code); // permanently light up status LED
  }
#endif

  while(1); // freeze
}


/**********************************************************************//**
 * Bootloader trap handler. Used for the MTIME tick and to capture any other traps.
 *
 * @warning Adapt exception PC only for sync exceptions!
 *
 * @note Since we have no runtime environment, we have to use the interrupt attribute here. Here and only here!
 **************************************************************************/
void __attribute__((__interrupt__)) bootloader_trap_handler(void) {

  register uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // Machine timer interrupt
  if (cause == TRAP_CODE_MTI) { // raw exception code for MTI
#if (STATUS_LED_EN != 0)
    if (neorv32_gpio_available()) {
      neorv32_gpio_pin_toggle(STATUS_LED_PIN); // toggle status LED
    }
#endif
    // set time for next IRQ
    if (neorv32_mtime_available()) {
      neorv32_mtime_set_timecmp(neorv32_mtime_get_timecmp() + (NEORV32_SYSINFO.CLK/4));
    }
  }

  // Bus store access error during get_exe
  else if (cause == TRAP_CODE_S_ACCESS) {
    system_error(ERROR_TRAP); //
  }

  // Anything else (that was not expected); output exception notifier and try to resume
  else {
    register uint32_t epc = neorv32_cpu_csr_read(CSR_MEPC);
    neorv32_cpu_csr_write(CSR_MEPC, epc + 4); // advance to next instruction
    system_error(0xFF); //
  }
}