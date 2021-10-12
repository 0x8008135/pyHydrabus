# Copyright 2021 Karim SUDKI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from .rawwire import RawWire
from struct import unpack
import time

class PIC18(RawWire):
    #TODO: Write example

    """
    PIC18 ICSP Protocol handler

    :example:

    >>> import pyHydrabus
    >>> pic = pyHydrabus.PIC18('/dev/hydrabus')
    >>> pic.write(0x800, "ABCD")
    >>> pic.read(0x800,4)
    >>> pic.get_cfg_ID()
    >>> pic.get_loc_ID()
    >>> pic.get_dev_ID()
    
    ---
    [PIC Commands]
    CORE_INST                   b"\x00"
    TABLAT_OUT                  b"\x02"
    TABLE_READ                  b"\x08"
    TABLE_READ_POST_INC         b"\x09"
    TABLE_READ_POST_DEC         b"\x0A"
    TABLE_READ_PRE_INC          b"\x0B"
    TABLE_WRITE                 b"\x0C"
    TABLE_WRITE_POST_INC2       b"\x0D"
    TABLE_WRITE_POST_INC2_PGM   b"\x0E"
    TABLE_WRITE_PGM             b"\x0F"
    
    [PIC Panels]
    Chip Erase                  b"\x0F\x8F"
    User ID                     b"\x00\x88"
    EEPROM                      b"\x00\x84"
    EEPROM                      b"\x00\x81"
    Config Bits                 b"\x00\x82"
    EEPROM Block 0              b"\x01\x80"
    EEPROM Block 1              b"\x02\x80"
    EEPROM Block 2              b"\x04\x80"
    EEPROM Block 3              b"\x08\x80"
    """

    def __init__(self, port=""):
        super().__init__(port)
        self._config = 0xA
        self._configure_port()
        self.AUX[0].direction = 0
        self.AUX[1].direction = 0

    def _enter_prog(self):
        self.sda = 0 #PGD
        self.clk = 0 #PGC
        self.AUX[1].value = 1 #PGM
        self.AUX[0].value = 1 #/MCLR

    def _exit_prog(self):
        
        self.sda = 0 #PGD        
        self.clk = 0 #PGC
        self.AUX[0].value = 0 #PGM
        self.AUX[1].value = 0 #/MCLR
    
    def nop(self, repeat=1):
        for _ in range(0, repeat):
            #NOP
            self.send_cmd(b"\x00", b"\x00\x00") 
    
    def send_cmd(self, cmd, payload):
        super().write_bits(cmd, 4)
        if len(payload) % 2 == 0:
            super().write(payload[1:2])
            super().write(payload[0:1])
        else:
            super().write(payload[0:1])

    def set_eeaddr(self, address):
        EEADDR = address.to_bytes(2, byteorder='big')
        #MOVLW xx
        self.send_cmd(b"\x00", b"\x0E"+EEADDR[0:1])
        #MOVWF EEADRH
        self.send_cmd(b"\x00", b"\x6E\xAA")
        #MOVLW xx
        self.send_cmd(b"\x00", b"\x0E"+EEADDR[1:2])
        #MOVWF EEADR
        self.send_cmd(b"\x00", b"\x6E\xA9")

    def set_tblptr(self, address):
        TBLPTR = address.to_bytes(3, byteorder='big')
        #MOVLW xx
        self.send_cmd(b"\x00", b"\x0E"+TBLPTR[0:1])
        #MOVWF TBLPTRU
        self.send_cmd(b"\x00", b"\x6E\xF8")
        #MOVLW xx
        self.send_cmd(b"\x00", b"\x0E"+TBLPTR[1:2])
        #MOVWF TBLPTRH
        self.send_cmd(b"\x00", b"\x6E\xF7")
        #MOVLW xx
        self.send_cmd(b"\x00", b"\x0E"+TBLPTR[2:3])
        #MOVWF TBLPTRL
        self.send_cmd(b"\x00", b"\x6E\xF6")

    def poll_wr(self):
        #MOVF EECON1, W, 0
        self.send_cmd(b"\x00", b"\x50\xA6")
        #MOVWF TABLAT
        self.send_cmd(b"\x00", b"\x6E\xF5")
        #NOP
        self.nop()
        #TABLAT_OUT
        self.send_cmd(b"\x02", b"\x00") 
        return (int.from_bytes(super().read(1),byteorder="big") >>1) & 1

    def read(self, addr, length, eeprom=False):
        if length == 0:
            raise ValueError(f"Length must be at least one")
        
        self._enter_prog()
        data = b""
        if eeprom:
            #BCF EECON1, EEPGD
            self.send_cmd(b"\x00", b"\x9E\xA6")
            #BCF EECON1, CFGS
            self.send_cmd(b"\x00", b"\x9C\xA6")
            for offset in range(0, length, 1):
                self.set_eeaddr(addr+offset)
                #BSF EECON1, RD
                self.send_cmd(b"\x00", b"\x80\xA6")
                #MOVF EEDATA, W, 0
                self.send_cmd(b"\x00", b"\x6E\xF5")
                #MOVWF TABLAT
                self.send_cmd(b"\x00", b"\x50\xA8")
                #NOP
                self.nop()
                data += super().read(1)
        else:
            self.set_tblptr(addr)
            for _ in range(0,length,1):
                #TABLE_READ_POST_INC
                self.send_cmd(b"\x09", b"\x00") 
                data += super().read(1)

        self._exit_prog()      
        return data
    
    def write(self, addr, data, eeprom=False, erase=False):
        length = len(data)
        if length < 2:
            raise ValueError(f"Data should be at least two byte")
        if (length %2) == 1:
            raise ValueError(f"Data should be a multiple of two bytes")
        
        self._enter_prog()
        if eeprom:
            #BCF EECON1, EEPGD
            self.send_cmd(b"\x00", b"\x9E\xA6")
            #BCF EECON1, CFGS
            self.send_cmd(b"\x00", b"\x9C\xA6")
            self.set_eeaddr(addr)
            #BSF EECON1, WREN
            self.send_cmd(b"\x00", b"\x84\xA6") 
            for d in range(0, length):
                self.send_cmd(b"\x0E", data[d:d+1])
                #BSF EECON1, WREN
                self.send_cmd(b"\x00", b"\x84\xA6")
                
                self.nop(2)
                while self.poll_wr():
                    continue
        else:
            #BSF EECON1, EEPGD
            self.send_cmd(b"\x00", b"\x8E\xA6")
            #BCF EECON1, CFGS
            self.send_cmd(b"\x00", b"\x9C\xA6")
            self.set_tblptr(addr)
            #BSF EECON1, WREN
            self.send_cmd(b"\x00", b"\x84\xA6")
            #BSF EECON1, FREE
            self.send_cmd(b"\x00", b"\x88\xA6")
            if erase:
                #BSF EECON1, FREE
                self.send_cmd(b"\x00", b"\x88\xA6")
                #BSF EECON1, WR
                self.send_cmd(b"\x00", b"\x82\xA6")
                #NOP *2
                self.nop(2)
                while self.poll_wr():
                    continue
            if length == 2:
                self.send_cmd(b"\x0F",data) 
            else:
                for d in range(0, length-2, 2):
                    self.send_cmd(b"\x0D",data[d:d+2][::-1])
                # Write last two bytes
                self.send_cmd(b"\x0F",data[d+2:][::-1])
            self.sda = 0
            #Clock *3
            self.clocks(3)
            #PGC high on 4th clock
            self.clk = 1
            time.sleep(1)
        self.clk = 0
        #NOP *2
        self.nop(2)
        #BCF EECON1, WREN
        self.send_cmd(b"\x00", b"\x94\xA6") 
        self._exit_prog()

    def erase_panel(self, panel):
        self._enter_prog()
        self.set_tblptr(0x3c0005)
        self.send_cmd(b"\x0C",panel[0:1]*2)
        self.set_tblptr(0x3c0004)
        self.send_cmd(b"\x0C",panel[1:2]*2)
        self.send_cmd(b"\x00",b"\x00\x00")
        self.send_cmd(b"\x00",b"\x00\x00")
        self._exit_prog()

    def get_cfg_ID(self, address=None):
        self._enter_prog()
        if address is None:
            address = 0x300000
        data = b""
        self.set_tblptr(address)
        for _ in range(0, 14, 1):
            self.send_cmd(b"\x09",b"\x00")
            data += super().read(1)
        self._exit_prog()
        CONFIG = [x for x in unpack('HHHHHHH', data)]
        c = 1
        for i in CONFIG:
            print(f"CONFIG{c} :"+" {:04x}".format(i))
            c += 1

    def get_loc_ID(self, address=None):
        self._enter_prog()
        if address is None:
            address = 0x200000
        data = b""
        for _ in range(address,address+8,1):
            self.set_tblptr(address)
            self.send_cmd(b"\x08",b"\x00")
            data += super().read(1)
        self._exit_prog()
        ID = [x for x in unpack('HHHH', data)]
        c = 1
        for i in ID:
            print(f"LOC_ID {c} :"+" {:04x}".format(i))
            c += 1

    def get_dev_ID(self, address=None):
        self._enter_prog()
        if address is None:
            address = 0x3FFFFE
        data = b""
        self.set_tblptr(address)
        for _ in range(0,2):
            self.send_cmd(b"\x09",b"\x00")
            data += super().read(1)
        self._exit_prog()
        c = 1
        for i in data:
            print(f"DEV_ID {c} :"+" {:02x}".format(i))
            c += 1
