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

type_mapping = {
    "CHAR": 'c',
    "FLOAT": 'f',
    'UINT_16': 'H',
    'INT_16' : 'h',
    "BYTE": 'B'
}

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
                    "send_type":type_mapping[send_type],
                    "recieve_type":type_mapping[recieve_type]

                }

    return data

class Module(ABC):

    def __init__(self):
        self.Var = 0
        
    def send_parameters(self, variable, action, data= None):
        if action not in ['READ']['WRITE']:
            print("Invalid Action Type")
            return

        header_byte = list(bytes(SPI_CODES['header']['code']))
        action_code_byte= list(bytes(SPI_CODES[action]['code']))
        data_bytes = list(struct.pack(SPI_CODES[variable][send_type],SPI_CODES[variable][receive_type]))
        CRC = list(bytes(15)) #need to actually implement CRC this is a placeholder

        transmit_packet = header_byte +action_code_byte +data_bytes +CRC

        spi.xfer(transmit_packet)
        return

    def read_parameters(self):
        received_data = spi.xfer2(bytes(SPI_RECEIVE_FILLER))
        for i in received_data:
            if i != SPI_CODES['IGNORE']['code']:
                spi_receive_buffer.append(i)

        if len(spi_receive_buffer)<=SPI_RECEIVE_PACKET_LENGTH:
            return

        variable_code = spi_receive_buffer[1]
        variable = [k for k, v in main_dict.items() if v['code'] == search_value][0]
        value = struct.unpack(SPI_CODES[variable][recieve_type], spi_receive_buffer[2:6])
        spi_receive_buffer = spi_receive_buffer[5:]
        try:
            setattr(self,varstring,value)
        except AttributeError:
            print(f"{varstring} is not a valid attribute")

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

SPI_CODES_FILE= '/home/pi/resources/spi_codes_file.csv'
SPI_CODES = spi_csv(SPI_CODES_FILE)
SPI_RECEIVE_FILLER = 5*list(struct.pack(SPI_CODES['IGNORE']["code"],SPI_CODES['IGNORE']['send_type']))
spi_receive_buffer = []
SPI_RECEIVE_PACKET_LENGTH = 5

