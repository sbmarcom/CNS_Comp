#!/usr/bin/env python3
import spidev
import hal
import time
import sys
import traceback
import configparser
import linuxcnc
import struct
import os
from abc import ABC,abstractmethod
sys.path.append('/home/pi/bin')
from message_handler import message_handler

######################
# region GLOBAL FUNCTIONS#
######################

def spi_csv(filename):
    data = {}
    with open(filename,"r") as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            variable_name,code,send_type,recieve_type = row
            data[variable_name]=
                {
                "code":int(code),
                "send_type":c_type_mapping[send_type],
                "recieve_type":c_type_mapping[recieve_type]
                }

    return data



#endregion 
#################
#region MODULE CLASSES
#################

class Module(ABC):

    def __init__(self):
        self.Var = 0
        
    def send_parameters(self, variable, action, data= None):
        if action not in ['READ','WRITE']:
            print("Invalid Action Type")
            return

        header_byte = list(bytes(SPI_CODES['header']['code']))
        action_code_byte= list(bytes(SPI_CODES[action]['code']))
        data_bytes = list(struct.pack(SPI_CODES[variable][send_type],SPI_CODES[variable][receive_type]))
        

        transmit_packet = header_byte +action_code_byte +data_bytes
        temp_packet = []
        for byte in transmit_packet:
            if byte == SPI_CODES['header']['code'] or byte ==  SPI_CODES["stuffing_byte"]['code']:
                temp_packet.append( SPI_CODES["stuffing_byte"]['code'])
                temp_packet.append(byte ^ SPI_CODES["xor_value"]['code'])
            else:
                temp_packet.append(byte)
        transmit_packet = temp_packet
        CRC = 0
        for byte in transmit_packet:
            CRC = CRC ^ byte
        
        transmit_packet.append(chr(CRC))

        spi.xfer(transmit_packet)
        return

    def read_parameters(self):
        received_data = spi.xfer2(SPI_RECEIVE_FILLER)
        stuffing = 0 
        for i in received_data:
            if i == SPI_CODES['RECEIVE_FILLER']['code']:
                continue
            if stuffing:
                spi_receive_buffer.append(i^SPI_CODES['XOR_BYTE']['code'] )
                stuffing = 0
            if i ==SPI_CODES['STUFFING_BYTE']['code']:
                stuffing = 1
                continue
            else:
                spi_receive_buffer.append(i)

        #remove items until the header value is received
        while spi_receive_buffer:
            if spi_receive_buffer[0] == SPI_CODES['RECEIVE_HEADER']['code']:
                break
            item = spi_receive_buffer.pop(0)

        #if the packet is not long enough return
        if len(spi_receive_buffer)<SPI_RECEIVE_PACKET_LENGTH:
            return
        #check the CRC with all the values in the list
        CRC = 0
        for i in spi_receive_buffer[:6]:
            CRC = CRC ^ i
        
        #if the CRC is not valid discard the packet
        if CRC != spi_receive_buffer[6]:
            spi_receive_buffer = spi_receive_buffer[7:]
            return 

        #pull out the variable code from the now validated packet
        variable_code = spi_receive_buffer[1]
        #search for the variable associated with this code
        variable = [k for k, v in main_dict.items() if v['code'] == search_value][0]

        value = struct.unpack(SPI_CODES[variable][recieve_type], spi_receive_buffer[2:6])
        spi_receive_buffer = spi_receive_buffer[5:]
        try:
            setattr(self,variable,value)
        except AttributeError:
            print(f"{variable} is not a valid attribute")

        return

    
    def identify_module(cls):

    @abstractmethod
    def implement_on(self):
        pass
    
    @abstractmethod
    def implement_off(self):
        pass

    @abstractmethod
    def probe_cycle(self):
        pass
    
    @abstractmethod
    def height_control(self):
        pass

    @abstractmethod
    def check_state(self)
        pass


class SemiAutoGasControl(Module):

    def __init__(self):
        super.__init__():
     
    def implement_on(self):
        pass
    
    
    def implement_off(self):
        pass

    
    def probe_cycle(self):
        pass
    
    
    def height_control(self):
        pass

    
    def check_state(self)
        pass

class ModulePrototype(Module):

    def __init__(self):
        super.__init__():
     
    def implement_on(self):
        pass
    
    
    def implement_off(self):
        pass

    
    def probe_cycle(self):
        pass
    
    
    def height_control(self):
        pass

    
    def check_state(self)
        pass

#endregion 

SPI_CODES_FILE= '/home/pi/resources/spi_codes_file.csv'
SPI_CODES = spi_csv(SPI_CODES_FILE)
SPI_RECEIVE_FILLER = 5*list(struct.pack(SPI_CODES['TRANSMIT_FILLER']["code"],SPI_CODES['TRANSMIT_FILLER']['send_type']))
spi_receive_buffer = []
SPI_RECEIVE_PACKET_LENGTH = 5

#DAEMON_WRITE_FILE = '/home/pi/bin/pwr-out'
#DAEMON_READ_FILE = '/home/pi/bin/pwr-inp'

c_type_mapping = {
    "CHAR": 'c',
    "FLOAT": 'f',
    'UINT_16': 'H',
    'INT_16' : 'h',
    "BYTE": 'B'
}


module = Module()

while True:
    
    #check Daemon here


