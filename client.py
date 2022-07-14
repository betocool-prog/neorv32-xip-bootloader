 ################################################################################################# 
 # << NEORV32 - RISC-V XIP Bootloader Client module >>                                  # 
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
 # @file client.py
 # @author Alberto Fahrenkrog
 # @brief Client for the XIP Bootloader Command Line Interface
 ################################################################################################# 

import serial
import struct
import numpy as np
import json

class Client:

    POKE = b'\x00'
    EXECUTE = b'\x01'
    FLASH_WRITE = b'\x02'
    FLASH_READ = b'\x03'
    RESET = b'\x04'
    ERASE_SECTOR = b'\x06'
    STATUS_BYTE = b'\x07'
    DEMO = b'\x08'

    SECTORS = 128
    SECTOR_SIZE = 0x10000
    # FLASH_SIZE = SECTORS * SECTOR_SIZE
    FLASH_SIZE = 0x1000000

    def __init__(self):
        
        # Initialise Config
        try:
            self.config = json.load(open('neorv_sh.json'))
            self.board = self.config['board']
            self.ser_port = self.config[self.board]['serial']['port']
            self.baudrate = self.config[self.board]['serial']['baudrate']
            self.flash_mem = self.config[self.board]['flash']
        except Exception as e:
            print("Could not open config file: {}".format(repr(e)))
            exit(1)
        
        # Initialise Serial port
        try:
            self.ser = serial.Serial(port=self.ser_port, baudrate=self.baudrate,
                                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS, timeout=1)
        except Exception as e:
            print("Could not open serial port{}: ".format(repr(e)))
            exit(1)

    def to_int(self, text):
        ret_Val = None
        try:
            if text[0:2] == '0x':
                ret_Val = int(text, 16)
            else:
                ret_Val = int(text, 10)
        except Exception as e:
            print("Bad number format, e.g.: use 0x10 or 16")
        
        return ret_Val

    def execute(self):
        self.ser.write(self.EXECUTE)
        reply = self.ser.read()
        if reply == self.EXECUTE:
            print("Jumping to application...")

    def status_byte(self):
        self.ser.write(self.STATUS_BYTE)
        reply = self.ser.read(1)
        print("Status byte: 0x{:02X}".format(ord(reply)))

    def flash_write_page(self, addr, data):
        # This function writes 256 bytes or less only!
        self.ser.write(self.FLASH_WRITE)
        self.ser.write(struct.pack("<I", addr))
        self.ser.write(struct.pack("<I", len(data)))
        self.ser.write(data)
        
    def flash_write(self, arg):

        arg = arg.split(' ')
        
        if len(arg) != 2:
            print("Start address and file name required!")
            return

        addr = self.to_int(arg[0])
        if addr == None:
            return
        
        # Open file to write
        try:
            data = np.fromfile(file=arg[1], dtype='I').tobytes()
        except Exception as e:
            print("Could not open file: {}".format(repr(e)))
            return

        # Confirm data fits into flash
        data_len = len(data)
        if (addr + data_len) > self.FLASH_SIZE:
            print("Data size exceeds memory size!")
            return

        # Check which sectors need to be erased. That depends on the 
        # start address and data length.
        start_sector = addr >> 16
        stop_sector = (addr + data_len) >> 16

        for sector in range(start_sector, stop_sector + 1):
            # Reusing erase sector command... it uses string as input!
            self.erase_sector(sector << 16)
            print("Sector {} erased...".format(sector))

        start = 0
        step = 256 # This is the block of data we're sending in one go!
        stop = step
        while stop <= data_len:
            self.flash_write_page(addr, data[start:stop])
            self.ser.read(1)
            print("{:.1f}%\r".format(stop * 100 / data_len), end='')
            start = stop
            stop += step
            addr += step

        self.flash_write_page(addr, data[start:])
        self.ser.read(1)
        
        print("100.0%")            
        print("Finished writing {} bytes...".format(data_len))


    def flash_read(self, arg):

        args = arg.split(' ')
        
        if len(args) != 2:
            print("Need to provide address and length!")
            return
        
        addr = self.to_int(args[0])
        data_len = self.to_int(args[1])

        if None in [addr, data_len]:
            return

        self.ser.write(self.FLASH_READ)
        self.ser.write(struct.pack("<I", addr))
        self.ser.write(struct.pack("<I", data_len))
        reply = self.ser.read(data_len)
        
        # Format the output
        print()
        start = 0
        step = 16 # Change this to print more or less values per line
        stop = step
        addr_text = "0x{:08x}: "
        while stop <= data_len:
            text = addr_text + (stop - start) * "{:02X} "
            print(text.format(addr + start, *(reply[start:stop])))
            start = stop
            stop += step

        if (data_len - start) > 0:
            text = addr_text + (data_len - start) * "{:02X} "
            print(text.format(addr + start, *(reply[start:data_len])))

        print()

    def erase_sector(self, arg):

        addr = None

        if(type(arg) == str):
            addr = self.to_int(arg)
        elif(type(arg) == int):
            addr = arg

        if addr == None:
            return

        self.ser.write(self.ERASE_SECTOR)
        self.ser.write(struct.pack('<I', addr))
        reply = self.ser.read(1)
        if reply == self.ERASE_SECTOR:
            print("Sector erased!")

    def demo(self):
        self.ser.write(self.DEMO)
        reply = self.ser.read(1)
        if reply == self.DEMO:
            print("Flashed demo!")

    def reset(self):
        self.ser.write(self.RESET)
        reply = self.ser.read()
        if reply == self.RESET:
            print("Reset!")

    def poke_bl(self):
        self.ser.write(self.POKE)
        reply = self.ser.read()
        if(reply):
            print("Poked!")
        else:
            print("No reply... Not in bootloader mode")