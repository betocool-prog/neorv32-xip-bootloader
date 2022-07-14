#!/usr/bin/env python

 ################################################################################################# 
 # << NEORV32 - RISC-V XIP Bootloader Command Line Interface >>                                  # 
 # ********************************************************************************************* # 
 # BSD 3-Clause License                                                                          # 
 #                                                                                               # 
 # Copyright (c) 2022, Alberto Fahrenkrog. All rights reserved.                                     #
 #                                                                                               # 
 # Redistribution and use in source and binary forms, with or without modification, are          # 
 # permitted provided that the following conditions are met:                                     # 
 #                                                                                               # 
 # 1. Redistributions of source code must retain the above copyright notice, this list of        # 
 #    conditions and the following disclaimer.                                                   # 
 #                                                                                               # 
 # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     # 
 #    conditions and the following disclaimer in the documentation and/or other materials        # 
 #    provided with the distribution.                                                            # 
 #                                                                                               # 
 # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  # 
 #    endorse or promote products derived from this software without specific prior written      # 
 #    permission.                                                                                # 
 #                                                                                               # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   # 
 # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               # 
 # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    # 
 # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     # 
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE # 
 # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    # 
 # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     # 
 # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  # 
 # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            # 
 # ********************************************************************************************* # 
 # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
 ################################################################################################# 

 ################################################################################################# 
 # @file neorv_sh.py
 # @author Alberto Fahrenkrog
 # @brief XIP Bootloader Command Line interface
 ################################################################################################# 

import cmd
import client

class CLI(cmd.Cmd):

    intro = "Command line interpreter for NEORV32 Bootloader"
    prompt = 'neorv_sh>>>'

    def __init__(self):
        
        # Initialise the Command Line Interface
        self.client = client.Client()
        super().__init__()

    def do_execute(self, arg):
        '''execute
Start application execution using XIP'''
        self.client.execute()

    def do_flash_write(self, arg):
        '''flash_write addr(hex or int) filename
Examples:
- flash_write 0x0 ./main.bin
- flash_write 0x2000 ../folder_to/main.bin'''
        self.client.flash_write(arg)

    def do_status_byte(self, arg):
        '''status_byte
Reads flash status byte'''
        self.client.status_byte()

    def do_flash_read(self, arg):
        '''flash_read addr(hex or int) length_in_bytes(hex or int)
Examples:
- flash_read 0x2000 128
- flash_read 1024 0x100'''
        self.client.flash_read(arg)

    def do_reset(self, arg):
        '''reset
Resets the bootloader'''
        self.client.reset()

    def do_poke(self, arg):
        '''poke
Sends a 0x00 character to the serial port, expecting the same answer back'''
        self.client.poke_bl()

    def do_erase_sector(self, arg):
        '''erase_sector addr(hex or int)
Examples:
- erase_sector 0x10000
- erase_sector 1024'''
        self.client.erase_sector(arg)

    def do_demo(self, arg):
        '''Copy the memory blinky to address 0x400000'''
        self.client.demo()

    def do_exit(self, arg):
        '''exit
Exits the application'''
        print("")
        return True


if __name__ == "__main__":

    neorv_sh = CLI()
    neorv_sh.cmdloop()