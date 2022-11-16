#!/usr/bin/env python
import serial
from serial.tools import list_ports
#import hal
import time
import sys
import pdb
#import configparser
#import linuxcnc

DriveSerial = serial.Serial(port='/dev/ttyS0',baudrate= 38400,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS, timeout=1)

#config = configparser.ConfigParser(strict=False)
#config.read('/home/pi/linuxcnc/configs/my-mill/my-mill.ini')
#DOne=config.get('Ultra3000','JointOneDriveAddress')
#DTwo=config.get('Ultra3000','JointTwoDriveAddress')
#j=hal.component('Drive')
#
#j.newpin("MOnePos",hal.HAL_S32,hal.HAL_IN)
#j.newpin("MTwoPos",hal.HAL_S32,hal.HAL_IN)
#j.newpin("DrivesEnable",hal.HAL_BIT,hal.HAL_OUT)
#j.newpin("MOneUI", hal.HAL_S32,hal.HAL_IN)
#j.newpin("MTwoUI",hal.HAL_S32,hal.HAL_IN)
#
#time.sleep(.5)
#
#j.ready()

Commands ={
    'Enable':['06B1','01'],
    'Disable':['06B1','00'],
    'EnableQuery':['06B0'],
    'PosQuery':['07E0'],
    'RunStatus':['0750'],
    'ResetFault':['06F1']
        }

def SendReceive (address,command):
    cmd = Commands.get(command)
    if(cmd is None):
        raise ValueError("Unknown command")
    
    #Setup drive write command
    datastr = address+''.join(Commands.get(command))
    checksum=256-sum([ord(i) for i in datastr])
    TwosComp = hex((checksum + (1 << 8)) % (1 << 8))
    CC = ('0x' + TwosComp[2:].zfill(2))[-2:]
    Command = ':'+datastr+CC.upper()+'\r'
    #Write to UART Port
    DriveSerial.write(Command.encode('ascii'))
    #Process recieved signal 
    prefix=Commands.get(command)[0]
    data = DriveSerial.read_until('\r')
    data = data.decode('ascii').replace('\x00','').strip('\r')
    if len(data)<9:
        return None, None
    #data =data[prefix:-3]
    return data[7:-2] , data

#State= j.DriveEnable
#while True:
#    time.sleep(.1)
#    if not j.DriveEnable and State:
#        response1 = SendReceive(DOne,'Disable')
#        response2 = SendRecieve(DTwo,'Disable')
#        if response1=='08'  or response2 == '08':
#            continue
#        State= False
#    if not j.DriveEnable and not State:
#        position1=SendReceive(DOne,'PosQuery')
#        position2=SendRecieve(DTwo,'PosQuery')
#        if position1 == '08' or position2== '08':
#            continue
#        j.MOnePos=position1
#        j.MTwoPos=position2/8000*365
#
#

