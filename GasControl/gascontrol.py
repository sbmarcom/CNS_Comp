#!/usr/bin/env python3
import spidev
import hal
import time
import sys
import traceback
import configparser
import linuxcnc
import struct

ParameterDict = {
    'Cutting' : {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0},
    #'Oxy':      {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0},
    #'Fuel':     {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0}
}

PacketHeaders = {'kp' : 0x54 , 'ki' : 0x56 , 'kd' : 0x55 , 'Setpoint' : 0x73}

config = configparser.ConfigParser(strict=False)
config.read('/homes/pi/linuxcnc/configs/my-mill/my-mill.ini')

spi = spidev.SpiDev()
spi.open(6,0) #spi bus 6, device 0

try:
    s= linuxcnc.stat()
    s.poll()
except linuxcnc.error, detail:
    print("error ",detail)
    sys.exit(1)


c =hal.component('GC')
c.newpin("ModuleStatus",hal.HAL_U32,hal.HAL_IO) #TBA
c.newpin("ModulePluggedIn",hal.HAL_BIT,hal.HAL_IO) #TBA
c.newpin("CuttingJet",hal.HAL_U32,hal.HAL_IO)
time.sleep(.1)
c.ready()


for (p_id, p_info in ParameterDict.items()): 
    #Go through each type of parameter and pull the data from the INI file
    for key in p_info:
        try:
            p_info[key] = config.get('GAS_CONTROL',f'{key}')
        except:
            print("Variable {key} for {p_id} is not in INI file")


class module:
    def __init__(self):
        self.PluggedIn = False
        self.Initialized = False
        self.CuttingJet = False
    def Initialize():
        c.ModuleStatus=1
        for (p_id, p_info in ParameterDict.items()): 
            for key in p_info:
                self.SendParameters(PacketHeaders[key],p_info[key])
        self.Initialized = True

    def DeInit():
        c.ModuleStatus = 0
        self.Initialized = False

    def SendParameters(header, data):
        Tx =struct.pack('cf',header,data) #pack the header and data into a byte array
        crc = #calculate checksum of the byte array
        spi.xfer(Tx)
    def TurnOnJet():
        self.SendParameters(PacketHeaders['Setpoint'],ParameterDict['Cutting']['Setpoint'])
    def TurnOffJet():
        self.SendParameters(PacketHeaders['Setpoint'],0)



Manifold = module()

while (True):
    Manifold.PluggedIn = c.ModuleStatus
    if Manifold.PluggedIn ==False and Manifold.Initialized ==False:
        continue
    if Manifold.PluggedIn ==False and Manifold.Initialized==True:
        Manifold.Deinit()
        continue
    if Manifold.PluggedIn ==True and Manifold.Initialized ==False:
        Manifold.Initialize()
        continue
    s.poll()
    if (s.estop == STATE_ESTOP):
        continue
        #tell gas to turn off
    match c.CuttingJet:
        case False:
            if Manifold.CuttingJet==True:
                Manifold.TurnOffJet()   
            continue

        case True:
            if Manifold.CuttingJet==False:
                Manifold.TurnOnJet()
    
            
                
    

    
    




       

    
        

    
    
